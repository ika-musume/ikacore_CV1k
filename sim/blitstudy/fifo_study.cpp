// fifo_study — pre-H2 punch item 1: pin the attribute-FIFO depths by
// measurement (replaces the broken max_wait column; FINDINGS.md §4.1/§7).
//
// Two depths fall out of the two-plane architecture (sched doc §10):
//
//  * D_orig — the fetch lookahead the H4 governor emulates. The governed
//    fetch issues chunk c at max(open-loop cadence, golden consumption of
//    chunk c-D_orig): a virtual original-FIFO of D_orig chunks drained at
//    golden op-start times. This is what caps the execution plane's
//    run-ahead (§4.1 assumed ~2 chunks; the P sweeps used open-loop
//    arrivals and never actually tested the cap). Measured here: lateness
//    stats with arrivals capped at D_orig = 2/3/4 vs open-loop baseline,
//    plus the golden-timeline shift (must be ~0 or BUSY authenticity and
//    the fetch-bound anchors break).
//
//  * D_phys — the physical RTL FIFO in blit_fetch.sv. Occupancy =
//    governed-fetched chunks minus execution-plane-consumed chunks;
//    consumption: draws pop at engine issue, clip/clipped/END pop at
//    arrival (decode discard), upload header+payload pop at streaming
//    start (max(arrival, t_up)). Measured: max occupancy in bytes over
//    all EXECs x jitter seeds.
//
// Usage: fifo_study <trace.blit> [D_list=0,2,3,4] [seeds=5]
//   D=0 means open-loop (unbounded lookahead) — the P-study baseline.
#include "blit_trace.h"
#include "cost_model.h"
#include "ddr3_stat.h"
#include "engine.h"
#include "workload.h"

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using workload::ExecWork;
using workload::build_work;

namespace {

// Per-record chunk layout, mirroring cost::fetch_ready_times exactly:
// chunk c's cadence is that of the gov op whose fill fetched it.
struct ChunkLayout {
    std::vector<double> cadence;   // ns per chunk (T_CHUNK_NS or T_CHUNK_UPLD)
    std::vector<double> drainA;    // conservative: all non-upload ops
                                   // consumed at golden op start
    std::vector<double> drainB;    // decode-discard: clip/clipped ops leave
                                   // at arrival; only surviving draws wait
                                   // for golden start (-1 = no constraint)
    std::vector<char>   occupies;  // chunk holds non-upload op bytes (a FIFO
                                   // slot); upload header/payload stream
                                   // through and never occupy a slot
    std::vector<size_t> op_first;  // per gov op: first chunk index
    std::vector<size_t> op_last;   // per gov op: last chunk index
};

ChunkLayout build_chunks(const ExecWork &wk, const cost::GovTimeline &tl)
{
    ChunkLayout cl;
    std::vector<long> gov2eng(wk.gov.size(), -1);
    for (size_t j = 0; j < wk.eng.size(); j++)
        gov2eng[wk.eng2gov[j]] = long(j);

    long filled = 0, pos = 0;
    for (const auto &op : wk.gov) {
        cl.op_first.push_back(size_t(pos / cost::CHUNK));
        pos += op.nbytes;
        cl.op_last.push_back(size_t((pos - 1) / cost::CHUNK));
        while (filled < pos) {
            cl.cadence.push_back(op.upload ? cost::T_CHUNK_UPLD
                                           : cost::T_CHUNK_NS);
            filled += cost::CHUNK;
        }
    }
    cl.drainA.assign(cl.cadence.size(), -1.0);
    cl.drainB.assign(cl.cadence.size(), -1.0);
    cl.occupies.assign(cl.cadence.size(), 0);
    for (size_t i = 0; i < wk.gov.size(); i++) {
        if (wk.gov[i].upload) continue;
        for (size_t c = cl.op_first[i]; c <= cl.op_last[i]; c++) {
            cl.drainA[c] = std::max(cl.drainA[c], tl.start[i]);
            if (gov2eng[i] >= 0)
                cl.drainB[c] = std::max(cl.drainB[c], tl.start[i]);
            cl.occupies[c] = 1;
        }
    }
    return cl;
}

// governed chunk arrivals; D = 0 -> open-loop (matches fetch_ready_times).
// The D-deep backpressure window spans surviving-draw chunks only (drainB
// model): discard-class and upload-payload chunks pass through without
// holding a slot.
std::vector<double> arrivals(const ChunkLayout &cl, int D,
                             int64_t *nbind = nullptr)
{
    std::vector<double> arr(cl.cadence.size());
    std::vector<double> ring;   // drains of slot-holding chunks, in order
    double t = cost::T_EXEC2BRQ;
    for (size_t c = 0; c < cl.cadence.size(); c++) {
        t += cl.cadence[c];
        if (D > 0 && cl.drainB[c] >= 0) {
            const size_t j = ring.size();
            if (j >= size_t(D) && ring[j - D] > t) {
                t = ring[j - D];
                if (nbind) (*nbind)++;
            }
            ring.push_back(cl.drainB[c]);
        }
        arr[c] = t;
    }
    return arr;
}

// cost::governor with externally supplied ready times (to measure the
// golden-timeline shift the governed fetch pacing would introduce)
cost::GovTimeline governor_with_ready(const std::vector<cost::GovOp> &ops,
                                      const std::vector<double> &ready)
{
    cost::GovTimeline tl;
    double engine_free = 0.0;
    for (size_t i = 0; i < ops.size(); i++) {
        const double start = std::max(engine_free, ready[i]);
        engine_free = cost::add_steals(start, ops[i].cost_ns);
        tl.start.push_back(start);
        tl.finish.push_back(engine_free);
    }
    tl.busy_end = engine_free;
    return tl;
}

struct Event {
    double t;
    int    delta;   // bytes (FIFO scan) or +-1 (op-queue scan)
};

double scan_max(std::vector<Event> &ev)
{
    // arrivals before releases at equal timestamps (conservative)
    std::sort(ev.begin(), ev.end(), [](const Event &a, const Event &b) {
        return a.t != b.t ? a.t < b.t : a.delta > b.delta;
    });
    double occ = 0, mx = 0;
    for (const auto &e : ev) {
        occ += e.delta;
        mx = std::max(mx, occ);
    }
    return mx;
}

struct DAgg {
    double max_late = 0;
    int64_t n = 0, n_pos = 0, n_hline = 0;
    double max_shift = 0, max_busy_shift = 0;   // governed vs open-loop golden
    double max_occ = 0;                          // bytes
    uint64_t occ_exec = 0;
    int max_opq = 0;
    int64_t nbind = 0;                           // window-binding chunk count
    // virtual backlog: occupying chunks between arrival and golden
    // consumption — the never-binds lower bound for the governed window
    // (A = conservative decode-at-dispatch, B = decode-discard)
    double max_virtA = 0, max_virtB = 0;         // bytes
    uint64_t virtA_exec = 0, virtB_exec = 0;
    // structure of the max-occupancy exec
    size_t occ_nchunks = 0, occ_nocc = 0, occ_neng = 0;
    double occ_busy = 0;
    // upload-fence stats (H3 upload-path P-row, not attribute-FIFO sizing):
    // an upload whose dst overlaps a prior draw's src/dst cannot stream
    // until that draw retires; payload arriving before then must park
    int64_t n_upl = 0, n_fenced = 0;
    double max_park_us = 0, max_park_bytes = 0;
    uint64_t park_exec = 0;
};

} // namespace

int main(int argc, char **argv)
{
    if (!cost::validate_anchors()) {
        std::fprintf(stderr, "FAIL: cost-model anchors broken\n");
        return 1;
    }
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: fifo_study <trace.blit> [D_list=0,2,3,4] [seeds=5]\n");
        return 2;
    }
    std::vector<int> dlist = {0, 8, 16, 32, 64, 128, 256, 512, 1024};
    if (argc >= 3) {
        dlist.clear();
        for (const char *p = argv[2]; *p;) {
            dlist.push_back(std::atoi(p));
            while (*p && *p != ',') p++;
            if (*p) p++;
        }
    }
    const int nseeds = argc >= 4 ? std::atoi(argv[3]) : 5;
    engine::g_objline_batch = 8;                 // frozen (FINDINGS.md §5)
    if (const char *k = std::getenv("BLIT_OBJLINE_BATCH"))
        engine::g_objline_batch = std::atoi(k);
    std::printf("K = %d objlines, 1 thread, %d seeds\n",
                engine::g_objline_batch, nseeds);

    std::vector<ExecWork> work;
    {
        blit::TraceReader rd(argv[1]);
        blit::ExecRecord rec;
        while (rd.next(rec)) work.push_back(build_work(rec));
        std::printf("trace: %zu EXEC records\n", work.size());
    }

    std::printf("\n%-6s %12s %12s %12s %10s %8s %14s %8s\n", "D", "shift_us",
                "busy_shift", "max_late_us", ">hline", "late%", "max_occ_B(ch)",
                "max_opq");
    for (int D : dlist) {
        DAgg agg;
        uint64_t exec_idx = 0;
        for (auto &wk : work) {
            exec_idx++;
            if (wk.gov.empty()) continue;
            const auto tl = cost::governor(wk.gov);
            const auto cl = build_chunks(wk, tl);
            const auto arr = arrivals(cl, D, &agg.nbind);

            // governed per-op ready = arrival of the op's last byte
            std::vector<double> ready(wk.gov.size());
            for (size_t i = 0; i < wk.gov.size(); i++)
                ready[i] = arr[cl.op_last[i]];
            if (D == 0) {
                // self-check: must reproduce fetch_ready_times bit-for-bit
                const auto ref = cost::fetch_ready_times(wk.gov);
                for (size_t i = 0; i < ready.size(); i++)
                    if (std::fabs(ready[i] - ref[i]) > 1e-6) {
                        std::fprintf(stderr, "FAIL: open-loop arrival mismatch "
                                     "exec %" PRIu64 " op %zu\n", exec_idx, i);
                        return 1;
                    }
            }

            // golden-timeline shift under governed fetch pacing
            const auto tl2 = governor_with_ready(wk.gov, ready);
            for (size_t i = 0; i < wk.gov.size(); i++)
                agg.max_shift = std::max(agg.max_shift,
                                         tl2.start[i] - tl.start[i]);
            agg.max_busy_shift = std::max(agg.max_busy_shift,
                                          tl2.busy_end - tl.busy_end);

            for (size_t j = 0; j < wk.eng.size(); j++) {
                wk.eng[j].fetch_ready = ready[wk.eng2gov[j]];
                wk.eng[j].golden_finish = tl.finish[wk.eng2gov[j]];
            }

            // executable flag per gov op (draws/uploads with an eng entry)
            std::vector<long> gov2eng(wk.gov.size(), -1);
            for (size_t j = 0; j < wk.eng.size(); j++)
                gov2eng[wk.eng2gov[j]] = long(j);

            // virtual backlog (occupying chunks vs golden consumption)
            {
                std::vector<Event> va, vb;
                for (size_t c = 0; c < arr.size(); c++)
                    if (cl.occupies[c]) {
                        va.push_back({arr[c], +cost::CHUNK});
                        va.push_back({std::max(arr[c], cl.drainA[c]),
                                      -cost::CHUNK});
                        vb.push_back({arr[c], +cost::CHUNK});
                        vb.push_back({std::max(arr[c], cl.drainB[c]),
                                      -cost::CHUNK});
                    }
                const double a = scan_max(va), b = scan_max(vb);
                if (a > agg.max_virtA) {
                    agg.max_virtA = a;
                    agg.virtA_exec = exec_idx;
                }
                if (b > agg.max_virtB) {
                    agg.max_virtB = b;
                    agg.virtB_exec = exec_idx;
                }
            }

            for (int seed = 0; seed < nseeds; seed++) {
                std::vector<engine::OpResult> res;
                if (!wk.eng.empty()) {
                    ddr3::Port port(0x1234567 + uint64_t(seed) * 1000003 +
                                    exec_idx);
                    res = engine::run_exec(wk.eng, 1, port);
                    for (size_t j = 0; j < res.size(); j++) {
                        agg.n++;
                        if (res[j].lateness > 0) {
                            agg.n_pos++;
                            agg.max_late = std::max(agg.max_late,
                                                    res[j].lateness);
                            if (res[j].lateness > cost::HLINE_PERIOD_NS)
                                agg.n_hline++;
                        }
                    }
                }

                // physical FIFO: release[c] = latest pop among ops in chunk c
                std::vector<double> release(arr);
                std::vector<Event> opq;
                for (size_t i = 0; i < wk.gov.size(); i++) {
                    const long j = gov2eng[i];
                    if (j < 0) {
                        // clip / clipped draw: decode-discard on full arrival
                        const double pop = arr[cl.op_last[i]];
                        for (size_t c = cl.op_first[i]; c <= cl.op_last[i]; c++)
                            release[c] = std::max(release[c], pop);
                    } else if (wk.eng[j].upload) {
                        // RTL-faithful payload pop: posted VRAM writes
                        // stream on arrival, fenced only by rect-overlap
                        // with prior draws (run_exec's serialization behind
                        // *tmin is a 1-thread port-model artifact; the
                        // trickle is FINDINGS §5.1-class in RTL). Span rule
                        // mirrors run_exec's inflight entries.
                        const auto &e = wk.eng[j];
                        const double fetch_dur =
                            (16.0 + 2.0 * double(e.w) * e.h) *
                            (cost::T_CHUNK_UPLD / cost::CHUNK);
                        double hz = 0;
                        for (size_t p = 0; p < size_t(j); p++) {
                            const auto &q = wk.eng[p];
                            if (e.dst.overlaps(q.dst) ||
                                (!q.src.empty() && e.dst.overlaps(q.src)))
                                hz = std::max(hz, res[p].finish);
                        }
                        const double t_up =
                            std::max(e.fetch_ready - fetch_dur, hz);
                        for (size_t c = cl.op_first[i]; c <= cl.op_last[i]; c++)
                            release[c] = std::max(release[c], t_up);
                        opq.push_back({arr[cl.op_first[i]], +1});
                        opq.push_back({std::max(t_up, arr[cl.op_first[i]]), -1});
                        agg.n_upl++;
                        double parked = 0;
                        for (size_t c = cl.op_first[i]; c <= cl.op_last[i]; c++)
                            if (arr[c] < t_up) parked += cost::CHUNK;
                        if (parked > 0) {
                            agg.n_fenced++;
                            const double park_us =
                                (t_up - arr[cl.op_first[i]]) / 1000.0;
                            agg.max_park_us = std::max(agg.max_park_us, park_us);
                            if (parked > agg.max_park_bytes) {
                                agg.max_park_bytes = parked;
                                agg.park_exec = exec_idx;
                            }
                        }
                    } else {
                        const double pop = res[j].issue;
                        for (size_t c = cl.op_first[i]; c <= cl.op_last[i]; c++)
                            release[c] = std::max(release[c], pop);
                        opq.push_back({arr[cl.op_last[i]], +1});
                        opq.push_back({pop, -1});
                    }
                }
                // attribute-FIFO occupancy: non-payload chunks only —
                // payload routes down the upload write path, not the FIFO
                std::vector<Event> ev;
                ev.reserve(2 * arr.size());
                for (size_t c = 0; c < arr.size(); c++) {
                    if (!cl.occupies[c]) continue;
                    ev.push_back({arr[c], +cost::CHUNK});
                    ev.push_back({release[c], -cost::CHUNK});
                }
                const double occ = scan_max(ev);
                if (occ > agg.max_occ) {
                    agg.max_occ = occ;
                    agg.occ_exec = exec_idx;
                    agg.occ_nchunks = arr.size();
                    agg.occ_nocc = 0;
                    for (char o : cl.occupies) agg.occ_nocc += o;
                    agg.occ_neng = wk.eng.size();
                    agg.occ_busy = tl.busy_end;
                }
                agg.max_opq = std::max(agg.max_opq, int(scan_max(opq)));
            }
        }
        std::printf("%-6d %12.3f %12.3f %12.2f %10" PRId64 " %7.3f%% "
                    "%8.0f(%3.0f) %8d   occ@exec#%" PRIu64
                    " [%zuch/%zuocc/%zueng busy=%.0fus] binds=%" PRId64 "\n",
                    D, agg.max_shift / 1000.0, agg.max_busy_shift / 1000.0,
                    agg.max_late / 1000.0, agg.n_hline,
                    agg.n ? 100.0 * double(agg.n_pos) / double(agg.n) : 0.0,
                    agg.max_occ, agg.max_occ / cost::CHUNK, agg.max_opq,
                    agg.occ_exec, agg.occ_nchunks, agg.occ_nocc, agg.occ_neng,
                    agg.occ_busy / 1000.0, agg.nbind);
        std::printf("       virt backlog A %0.0f B (%0.0f ch) @exec#%" PRIu64
                    "   B %0.0f B (%0.0f ch) @exec#%" PRIu64 "\n",
                    agg.max_virtA, agg.max_virtA / cost::CHUNK, agg.virtA_exec,
                    agg.max_virtB, agg.max_virtB / cost::CHUNK, agg.virtB_exec);
        std::printf("       uploads %" PRId64 " fenced %" PRId64
                    "  max park %.1f us / %.0f B @exec#%" PRIu64 "\n",
                    agg.n_upl / nseeds, agg.n_fenced / nseeds,
                    agg.max_park_us, agg.max_park_bytes, agg.park_exec);
    }
    return 0;
}
