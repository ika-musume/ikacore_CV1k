// study_main — the P-stage driver.
//
// Per EXEC record of a .blit trace:
//   1. rebuild the governor's view (clip tracking mirrors MAME's
//      gfx_create_shadow_copy, u16 semantics) -> golden timeline,
//   2. cross-check our golden delay against MAME's mame_delay_ns
//      (same P_MAME constants; report ratio stats),
//   3. run the threaded DDR3 engine for each thread count x seed,
//      accumulate lateness / queue statistics.
//
// Usage: blit_study <trace.blit> [nthreads_list] [nseeds]
//   e.g.  blit_study traces/ibara_attract.blit 1,2,3,4,6,8 5
#include "blit_trace.h"
#include "cost_model.h"
#include "ddr3_stat.h"
#include "engine.h"
#include "workload.h"

#include <cinttypes>
#include <cmath>
#include <cstring>
#include <map>

using workload::ExecWork;
using workload::build_work;

namespace {

struct Agg {
    double max_late = 0, sum_late_pos = 0;
    int64_t n = 0, n_pos = 0, n_hline = 0;
    std::vector<float> lates;   // for percentiles
    int max_wait = 0;
    double worst_golden_busy = 0;
    char worst[256] = {0};
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
                     "usage: blit_study <trace.blit> [threads=1,2,3,4,6,8] [seeds=5]\n");
        return 2;
    }
    std::vector<int> threads = {1, 2, 3, 4, 6, 8};
    if (argc >= 3) {
        threads.clear();
        for (const char *p = argv[2]; *p;) {
            threads.push_back(std::atoi(p));
            while (*p && *p != ',') p++;
            if (*p) p++;
        }
    }
    const int nseeds = argc >= 4 ? std::atoi(argv[3]) : 5;
    const char *k = std::getenv("BLIT_OBJLINE_BATCH");
    if (!k) k = std::getenv("BLIT_ROW_BATCH");    // legacy name
    if (k) {
        engine::g_objline_batch = std::atoi(k);
        std::printf("(objline-batch K = %d objlines per read train)\n",
                    engine::g_objline_batch);
    }

    // pass 1: load + build all EXEC work items, cross-check vs MAME delay
    std::vector<ExecWork> work;
    {
        blit::TraceReader rd(argv[1]);
        blit::ExecRecord rec;
        double ratio_min = 1e9, ratio_max = 0, ratio_sum = 0;
        int64_t nratio = 0;
        while (rd.next(rec)) {
            work.push_back(build_work(rec));
            const auto &wk = work.back();
            if (wk.mame_delay_ns > 1000 && wk.our_mame_vclk > 0) {
                // MAME delay includes upload/idle/hline terms; compare only
                // draw-dominated records (uploads absent) for a clean ratio.
                bool has_upload = false;
                for (const auto &g : wk.gov) has_upload |= g.upload;
                if (!has_upload) {
                    const double r = double(wk.our_mame_vclk) * cost::V_NS /
                                     double(wk.mame_delay_ns);
                    ratio_min = std::min(ratio_min, r);
                    ratio_max = std::max(ratio_max, r);
                    ratio_sum += r;
                    nratio++;
                }
            }
        }
        std::printf("trace: %zu EXEC records\n", work.size());
        if (nratio)
            std::printf("cross-check ours(P_MAME)/mame_delay (draw-only recs): "
                        "min %.3f avg %.3f max %.3f  (n=%" PRId64 ")\n",
                        ratio_min, ratio_sum / nratio, ratio_max, nratio);
    }

    // pass 2: sweep thread counts x seeds
    std::printf("\n%-8s %-6s %12s %12s %10s %10s %8s\n", "threads", "seeds",
                "max_late_us", "p99_late_us", "late%", ">hline", "max_wait");
    for (int nt : threads) {
        Agg agg;
        for (int seed = 0; seed < nseeds; seed++) {
            uint64_t exec_idx = 0;
            for (auto &wk : work) {
                exec_idx++;
                if (wk.eng.empty()) continue;
                // fresh per-EXEC port: each EXEC's timeline is relative to
                // its own EXEC write (the governor restarts at t=0 too)
                ddr3::Port port(0x1234567 + uint64_t(seed) * 1000003 + exec_idx);
                const auto tl = cost::governor(wk.gov);
                agg.worst_golden_busy =
                    std::max(agg.worst_golden_busy, tl.busy_end);
                const auto ready = cost::fetch_ready_times(wk.gov);
                for (size_t i = 0; i < wk.eng.size(); i++) {
                    wk.eng[i].fetch_ready = ready[wk.eng2gov[i]];
                    wk.eng[i].golden_finish = tl.finish[wk.eng2gov[i]];
                }
                int mw = 0;
                const auto res = engine::run_exec(wk.eng, nt, port, &mw);
                agg.max_wait = std::max(agg.max_wait, mw);
                for (size_t ri = 0; ri < res.size(); ri++) {
                    const auto &r = res[ri];
                    agg.n++;
                    agg.lates.push_back(float(r.lateness));
                    if (r.lateness > 0) {
                        agg.n_pos++;
                        agg.sum_late_pos += r.lateness;
                        if (r.lateness > agg.max_late) {
                            agg.max_late = r.lateness;
                            const auto &e = wk.eng[ri];
                            std::snprintf(agg.worst, sizeof(agg.worst),
                                "worst op: %s %dx%d @dst(%d,%d) op#%zu/%zu "
                                "exec#%" PRIu64 " ready=%.1fus golden_fin=%.1fus "
                                "issue=%.1fus fin=%.1fus",
                                e.upload ? "UPLOAD" : "DRAW", e.w, e.h,
                                e.dst.x0, e.dst.y0, ri, wk.eng.size(),
                                exec_idx, e.fetch_ready / 1e3,
                                e.golden_finish / 1e3, r.issue / 1e3,
                                r.finish / 1e3);
                        }
                        if (r.lateness > cost::HLINE_PERIOD_NS) agg.n_hline++;
                    }
                }
            }
        }
        double p99 = 0;
        if (!agg.lates.empty()) {
            auto v = agg.lates;
            const size_t k = size_t(double(v.size() - 1) * 0.99);
            std::nth_element(v.begin(), v.begin() + k, v.end());
            p99 = v[k];
        }
        std::printf("%-8d %-6d %12.2f %12.2f %9.3f%% %10" PRId64 " %8d\n", nt,
                    nseeds, agg.max_late / 1000.0, p99 / 1000.0,
                    agg.n ? 100.0 * double(agg.n_pos) / double(agg.n) : 0.0,
                    agg.n_hline, agg.max_wait);
        if (agg.worst[0]) std::printf("  %s\n", agg.worst);
        if (nt == threads.back())
            std::printf("\nworst golden BUSY over trace: %.1f us "
                        "(frame budget 16683 us)\n",
                        agg.worst_golden_busy / 1000.0);
    }
    return 0;
}
