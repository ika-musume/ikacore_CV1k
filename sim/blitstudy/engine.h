// The DDR3 blitter — discrete-event model of the two-plane architecture
// (docs/blitter_ddr3_sched.md §10):
//   * timing plane: the cost-model governor (cost_model.h) generates the
//     CPU-visible timeline open-loop — fetch cadence, golden op start/finish,
//     BUSY. It never waits on DDR3 state.
//   * execution plane: N worker threads execute draw/upload ops against the
//     statistical DDR3 port (ddr3_stat.h). A thread may start an op as soon
//     as the op is fetched, a thread is free, and no hazard blocks it —
//     i.e. it may run AHEAD of the golden engine timeline (that headroom is
//     what absorbs jitter). It may never start before fetch_ready.
// The governed timeline never slips; the design question is bounding
// LATENESS = ddr3_finish - golden_finish per op.
#pragma once

#include "cost_model.h"
#include "ddr3_stat.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace engine {

// Terminology (project convention, cf. tileline/objline in other cores):
// an OBJLINE is one horizontal line of a blit rectangle in VRAM space
// (along VRAM X = contiguous addresses). Not a scanout hline, not a DRAM
// row, not the golden model's 32x32 tile span.
//
// read-batching granularity: the engine stages K objlines (src + dst)
// on chip, issues them as one read train, then writes them back — paying
// one R->W->R turnaround pair per batch instead of per objline. 0 =
// whole-op batching (unbounded staging). K=1 = naive per-line interleave.
// The required staging BRAM is K * (src_line + dst_line) words * 8 B.
inline int g_objline_batch = 0;

struct Rect {
    int x0 = 0, y0 = 0, x1 = -1, y1 = -1;   // inclusive; empty if x1<x0
    bool overlaps(const Rect &o) const
    {
        return x0 <= o.x1 && o.x0 <= x1 && y0 <= o.y1 && o.y0 <= y1;
    }
    bool empty() const { return x1 < x0 || y1 < y0; }
};

// One executable op (draws and uploads; clip ops don't reach the engine).
struct EngOp {
    bool   upload = false;
    Rect   src, dst;          // hazard rects (src empty for uploads)
    int    sx = 0, dx = 0;    // x coords for word counts (linear layout)
    int    w = 0, h = 0;      // clip-clamped dimensions
    bool   blend = true;      // conservative: dst read included
    double fetch_ready = 0;   // from the governor fetch model
    double golden_finish = 0; // from the governor engine timeline
};

struct OpResult {
    double issue = 0, finish = 0, lateness = 0;
    int    thread = -1;
};

struct RunStats {
    double max_lateness = 0;      // ns, over all ops
    double p99_lateness = 0;      // ns
    int64_t n_ops = 0, n_late = 0;       // n_late: lateness > 0
    int64_t n_late_hline = 0;            // lateness > one hline (63.586us)
    double busy_end_golden = 0, busy_end_ddr3 = 0; // per-EXEC, max over trace
    int    max_inflight_wait = 0; // max ops fetched but waiting for a thread
};

// Execute one EXEC's op sequence on N threads over one shared port.
// Returns per-op results; caller aggregates across records/seeds.
inline std::vector<OpResult> run_exec(const std::vector<EngOp> &ops,
                                      int nthreads, ddr3::Port &port,
                                      int *max_wait_out = nullptr)
{
    std::vector<double> thread_free(size_t(nthreads), 0.0);
    struct InFlight { Rect span; double finish; };
    std::vector<InFlight> inflight;
    std::vector<OpResult> res(ops.size());
    int max_wait = 0;

    for (size_t i = 0; i < ops.size(); i++) {
        const EngOp &op = ops[i];

        // hazard: earliest time all overlapping in-flight ops have finished.
        // span = src U dst of the in-flight op vs our src and dst (RAW+WAW+WAR).
        double hazard_clear = 0.0;
        for (const auto &f : inflight) {
            if (f.span.overlaps(op.dst) || (!op.src.empty() && f.span.overlaps(op.src)))
                hazard_clear = std::max(hazard_clear, f.finish);
        }

        // earliest-free thread
        auto tmin = std::min_element(thread_free.begin(), thread_free.end());
        int wait = 0;
        for (const auto &f : inflight)
            if (f.finish > op.fetch_ready) wait++;
        max_wait = std::max(max_wait, wait);

        double t = std::max({op.fetch_ready, *tmin, hazard_clear});

        // DDR3 work (linear layout; PIPE read train per K-objline batch):
        // reads: src lines (+ dst lines if blending), writes: dst lines.
        const int64_t src_line = ddr3::words_linear(op.sx, op.w);
        const int64_t dst_line = ddr3::words_linear(op.dx, op.w);
        double done;
        if (op.upload) {
            // The original streams upload chunks to VRAM as they arrive over
            // the (much slower) SDRAM fetch; the DDR3 write rides along the
            // fetch window. Port occupancy may start when the payload starts
            // arriving; completion cannot precede the last fetched byte.
            const int64_t w_dst = dst_line * op.h;
            const double fetch_dur = (16.0 + 2.0 * double(op.w) * op.h) *
                                     (cost::T_CHUNK_UPLD / cost::CHUNK);
            const double t_up =
                std::max({op.fetch_ready - fetch_dur, *tmin, hazard_clear});
            done = port.write(t_up, w_dst, ddr3::bursts_of(w_dst) + op.h);
            if (done < op.fetch_ready) done = op.fetch_ready;
        } else {
            const int K = g_objline_batch > 0 ? g_objline_batch : op.h;
            done = t;
            for (int r0 = 0; r0 < op.h; r0 += K) {
                const int lines = r0 + K <= op.h ? K : op.h - r0;
                const int64_t rd = (src_line + (op.blend ? dst_line : 0)) * lines;
                const int nb = lines * (op.blend ? 2 : 1);
                done = port.read(done, rd, nb);
                done = port.write(done, dst_line * lines, lines);
            }
        }

        *tmin = done;
        inflight.push_back({Rect{std::min(op.src.x0, op.dst.x0),
                                 std::min(op.src.y0, op.dst.y0),
                                 std::max(op.src.x1, op.dst.x1),
                                 std::max(op.src.y1, op.dst.y1)},
                            done});
        // prune settled entries to keep the scan short
        if (inflight.size() > 64) {
            std::vector<InFlight> keep;
            for (const auto &f : inflight)
                if (f.finish > t) keep.push_back(f);
            inflight.swap(keep);
        }

        res[i].issue = t;
        res[i].finish = done;
        res[i].lateness = done - op.golden_finish;
        res[i].thread = int(tmin - thread_free.begin());
    }
    if (max_wait_out) *max_wait_out = max_wait;
    return res;
}

} // namespace engine
