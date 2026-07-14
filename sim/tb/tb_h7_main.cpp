// ikacore CV1k - H7a step-4 testbench harness.
//
// Drives tb_h7 (blit_draw + blit_batch + blit_video[PREFETCH] + ddr3_harness)
// at the TARGET clock configuration (153.6 MHz, CKIO enable every 3rd cycle)
// against a C++ DDRAM slave whose timing is the calibrated ddr3_stat.h model
// (M-DDR3 measurements: latency histogram under HPS load, beta_R/beta_W,
// T_TURN, G_CMD).  Per exec:
//
//   * the attribute FIFO feed is paced by cost::fetch_ready_times() - each
//     op's words are withheld until the governed fetch would have delivered
//     them, the same information timing engine.h's DES was validated under;
//   * the DDRAM image is diffed against the H1 golden model (pixel-exact or
//     fail) - the whole stack, batching + arbitration + jitter included,
//     must be execution-plane-invisible;
//   * per-op LATENESS (RTL finish vs cost::governor() golden finish) is
//     measured via the descriptor/serve/write-drain taps.  The H7a accept
//     bar is the FINDINGS one: no op later than one hline (63.586 us),
//     worst case comparable to the study's +9.83 us.
//
// After the last exec, two full video frames are scanned out and the second
// is compared against a C++ render of the DDRAM image at the final SCROLL -
// the video train path (prefetch + absolute-priority arbitration) must
// deliver the exact scanout MAME would.
//
//   ./Vtb_h7 --trace f.blit [--execs N] [--seed S] [--noframe]
//     seed 0 = perfect port (bring-up); seed != 0 = stat timing, HPS tail
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include <verilated.h>
#include "Vtb_h7.h"
#include "Vtb_h7___024root.h"

#include "blitgold/golden.h"
#include "blitstudy/blit_trace.h"
#include "blitstudy/cost_model.h"
#include "blitstudy/ddr3_stat.h"
#include "blitstudy/workload.h"

using namespace gold;

static constexpr double TICK_NS  = 1000.0 / 153.6;   // 6.5104
static constexpr double HLINE_NS = 63586.0;

// ---------------------------------------------------------------------------
// DDRAM slave: 64 MB VRAM behind the MiSTer pins, ddr3_stat-calibrated.
// Reads are SNAPSHOT at command accept (in-order visibility regardless of
// delivery time); writes apply at accept.
// ---------------------------------------------------------------------------
struct DdrSlave {
    static constexpr uint32_t BASE_W = 0x06000000u;  // VRAM_BASE_W

    std::vector<uint16_t> mem;
    ddr3::LatencySampler  lat;
    uint32_t seed;

    struct Burst {
        std::vector<uint64_t> data;
        size_t k = 0;
        double next_ns = 0;
    };
    std::deque<Burst> rq;
    double free_ns = 0, last_end_ns = 0;
    bool   have_dir = false, dir_read = false;
    double wr_busy_until = 0;
    uint64_t n_rd_words = 0, n_wr_words = 0, n_bursts = 0;

    explicit DdrSlave(uint32_t s)
        : mem(size_t(1) << 25, 0), lat(s ? s : 1), seed(s) {}

    double p_lat()  const { return seed ? 0 : 0; } // (histogram via lat)
    double p_turn() const { return seed ? ddr3::T_TURN * ddr3::CLK_NS : 0; }
    double p_br()   const { return seed ? ddr3::BETA_R * ddr3::CLK_NS : TICK_NS; }
    double p_bw()   const { return seed ? (ddr3::BETA_W + ddr3::C_W) * ddr3::CLK_NS
                                        : TICK_NS; }
    double p_gcmd() const { return seed ? ddr3::G_CMD * ddr3::CLK_NS : TICK_NS; }
    double p_l(double)    { return seed ? lat.sample() * ddr3::CLK_NS
                                        : 2.0 * TICK_NS; }

    bool busy() const {
        return rq.size() >= 8 || wr_busy_until > 0;
    }

    // call each tick BEFORE the posedge with current time; drives DOUT pins
    void drive(double now, Vtb_h7 &tb) {
        if (wr_busy_until > 0 && now >= wr_busy_until) wr_busy_until = 0;
        tb.DDRAM_BUSY = busy();
        tb.DDRAM_DOUT_READY = 0;
        if (!rq.empty()) {
            Burst &b = rq.front();
            if (b.k < b.data.size() && now >= b.next_ns) {
                tb.DDRAM_DOUT_READY = 1;
                tb.DDRAM_DOUT = b.data[b.k];
                b.k++;
                b.next_ns += p_br();
                n_rd_words++;
                if (b.k == b.data.size()) rq.pop_front();
            }
        }
    }

    uint64_t rd_word(uint32_t w) const {
        const size_t p = (size_t(w) & 0x7FFFFF) * 4;
        return (uint64_t(mem[p + 3]) << 48) | (uint64_t(mem[p + 2]) << 32)
             | (uint64_t(mem[p + 1]) << 16) |  uint64_t(mem[p]);
    }

    // call each tick AFTER settling comb at clk=0 (pins hold this cycle's
    // command); updates state for the next cycle
    void accept(double now, Vtb_h7 &tb) {
        const bool was_busy = tb.DDRAM_BUSY;
        if (tb.DDRAM_RD && !was_busy) {
            if (tb.DDRAM_ADDR < BASE_W || tb.DDRAM_ADDR >= BASE_W + (1u << 23)) {
                std::printf("FATAL: DDRAM read outside VRAM window %08x\n",
                            tb.DDRAM_ADDR);
                std::exit(1);
            }
            const uint32_t w0 = tb.DDRAM_ADDR - BASE_W;
            Burst b;
            b.data.reserve(tb.DDRAM_BURSTCNT);
            for (int k = 0; k < int(tb.DDRAM_BURSTCNT); k++)
                b.data.push_back(rd_word(w0 + uint32_t(k)));
            double t = std::max(now, free_ns);
            if (rq.empty()) {                    // fresh train
                if (have_dir && !dir_read) t += p_turn();
                t += p_l(now);
            }
            else t = last_end_ns + p_gcmd();     // pipelined behind prev burst
            b.next_ns = t + p_br();
            last_end_ns = t + double(b.data.size()) * p_br();
            free_ns = last_end_ns;
            have_dir = true; dir_read = true;
            n_bursts++;
            rq.push_back(std::move(b));
        }
        if (tb.DDRAM_WE && !was_busy) {
            if (tb.DDRAM_ADDR < BASE_W || tb.DDRAM_ADDR >= BASE_W + (1u << 23)) {
                std::printf("FATAL: DDRAM write outside VRAM window %08x\n",
                            tb.DDRAM_ADDR);
                std::exit(1);
            }
            const size_t p = (size_t(tb.DDRAM_ADDR - BASE_W) & 0x7FFFFF) * 4;
            const uint64_t d = tb.DDRAM_DIN;
            const uint8_t be = tb.DDRAM_BE;
            for (int l = 0; l < 4; l++)
                if (be & (3u << (2 * l)))
                    mem[p + size_t(l)] = uint16_t(d >> (16 * l));
            double t = std::max(now, free_ns);
            if (have_dir && dir_read) t += p_turn();
            free_ns = t + p_bw();
            wr_busy_until = free_ns;             // throttles the next accept
            have_dir = true; dir_read = false;
            n_wr_words++;
        }
    }
};

// ---------------------------------------------------------------------------
// rig
// ---------------------------------------------------------------------------
struct Rig {
    Vtb_h7 tb;
    DdrSlave ddr;
    uint64_t ticks = 0;
    int ckio_ph = 0;

    ~Rig() { tb.final(); }

    explicit Rig(uint32_t seed) : ddr(seed)
    {
        tb.i_CLK = 0; tb.i_CKIO_PCEN = 0; tb.i_RST_n = 0;
        tb.i_exec = 0; tb.i_clip_x = 0; tb.i_clip_y = 0;
        tb.i_scroll_x = 0; tb.i_scroll_y = 0;
        tb.i_fifo_valid = 0; tb.i_fifo_word = 0;
        tb.DDRAM_BUSY = 0; tb.DDRAM_DOUT = 0; tb.DDRAM_DOUT_READY = 0;
        for (int i = 0; i < 12; i++) tick();
        tb.i_RST_n = 1;
        for (int i = 0; i < 6; i++) tick();
    }

    double now_ns() const { return double(ticks) * TICK_NS; }

    bool pop = false;
    void tick()
    {
        tb.i_CKIO_PCEN = (ckio_ph == 0);
        tb.i_CLK = 0;
        ddr.drive(now_ns(), tb);
        tb.eval();
        pop = tb.o_fifo_pop;
        ddr.accept(now_ns(), tb);
        tb.i_CLK = 1; tb.eval();
        ticks++;
        ckio_ph = (ckio_ph + 1) % 3;
    }
};

// ---------------------------------------------------------------------------
// lateness bookkeeping
// ---------------------------------------------------------------------------
struct EngRef {
    bool upload; uint32_t dst0; int w, h; double golden_finish;
};

struct LatStats {
    double max_late = -1e18, max_early = 1e18;
    long   n = 0, n_pos = 0, n_hline = 0;
    double worst_op_late = -1e18; long worst_exec = -1;
    void add(double lat, long exec) {
        n++;
        if (lat > 0) n_pos++;
        if (lat > HLINE_NS) n_hline++;
        if (lat > max_late) { max_late = lat; worst_exec = exec; }
        if (lat < max_early) max_early = lat;
    }
};

static long g_ghosts = 0, g_extras = 0;

// ---------------------------------------------------------------------------
static bool compare_vram(Rig &rig, const Engine &e, long execno)
{
    const uint32_t *g = e.vram.data();
    long bad = 0;
    for (size_t i = 0; i < VRAM::SIZE; i++) {
        const uint16_t want = pen_to_argb1555(g[i]);
        if (rig.ddr.mem[i] != want) {
            if (bad < 6)
                std::printf("  MISMATCH exec-%ld at (%zu,%zu): rtl=%04x gold=%04x\n",
                            execno, i % VRAM_W, i / VRAM_W, rig.ddr.mem[i], want);
            bad++;
        }
    }
    if (bad) { std::printf("  [exec-%ld] %ld bad pixels\n", execno, bad); return false; }
    return true;
}

// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    if (!cost::validate_anchors()) {
        std::printf("FATAL: cost-model anchors failed\n");
        return 1;
    }

    const char *trace = nullptr;
    long max_execs = 400;
    uint32_t seed = 0xC0FFEE01u;
    bool do_frame = true;
    for (int i = 1; i < argc; i++) {
        if      (!std::strcmp(argv[i], "--trace") && i + 1 < argc) trace = argv[++i];
        else if (!std::strcmp(argv[i], "--execs") && i + 1 < argc) max_execs = std::atol(argv[++i]);
        else if (!std::strcmp(argv[i], "--seed")  && i + 1 < argc) seed = uint32_t(std::strtoul(argv[++i], nullptr, 0));
        else if (!std::strcmp(argv[i], "--noframe")) do_frame = false;
    }
    if (!trace) {
        std::printf("usage: %s --trace f.blit [--execs N] [--seed S] [--noframe]\n",
                    argv[0]);
        return 2;
    }

    Rig rig(seed);
    Engine gold_e;
    blit::TraceReader tr(trace);
    blit::ExecRecord rec;
    LatStats up_st, dr_st;
    long n = 0;
    uint16_t last_sx = 0, last_sy = 0;

    while (tr.next(rec) && n != max_execs) {
        const auto wk    = workload::build_work(rec);
        const auto ready = cost::fetch_ready_times(wk.gov);
        const auto tl    = cost::governor(wk.gov);

        // golden-finish reference list, program order
        std::vector<EngRef> eng;
        eng.reserve(wk.eng.size());
        for (size_t i = 0; i < wk.eng.size(); i++) {
            const auto &e = wk.eng[i];
            eng.push_back({e.upload,
                           uint32_t(e.dst.y0) * 8192u + uint32_t(e.dst.x0),
                           e.w, e.h, tl.finish[wk.eng2gov[i]]});
        }

        // per-word arrival times: the same chunked cadence as
        // cost::fetch_ready_times(), at word resolution so upload payload
        // streams in (the FINDINGS streaming-upload rule) instead of
        // arriving en bloc at the op's last byte
        const auto ops = blit::walk(rec.words);
        std::vector<double> word_ready(rec.words.size(), 0.0);
        {
            double t = cost::T_EXEC2BRQ;
            long filled = 0, pos = 0;
            for (size_t oi = 0; oi < ops.size(); oi++) {
                const size_t end = (oi + 1 < ops.size()) ? ops[oi + 1].off
                                                         : rec.words.size();
                const bool upl = (ops[oi].kind == blit::OpKind::Upload);
                for (size_t k = ops[oi].off; k < end; k++) {
                    pos += 2;
                    while (filled < pos) {
                        t += upl ? cost::T_CHUNK_UPLD : cost::T_CHUNK_NS;
                        filled += cost::CHUNK;
                    }
                    word_ready[k] = t;
                }
            }
        }
        (void)ready;

        rig.tb.i_scroll_x = rec.scroll_x;
        rig.tb.i_scroll_y = rec.scroll_y;
        last_sx = rec.scroll_x; last_sy = rec.scroll_y;

        // kick
        rig.tb.i_exec = 1;
        rig.tb.i_clip_x = rec.clip_x;
        rig.tb.i_clip_y = rec.clip_y;
        rig.tick();
        rig.tb.i_exec = 0;
        const double t0 = rig.now_ns();

        // run
        size_t idx = 0, ep = 0;
        std::deque<long> dsc_q;
        struct Await { size_t i; double srv; };
        std::deque<Await> await;
        long open_upl = -1;
        bool prev_wr_idle = true, prev_done = false;
        double last_idle_rise = t0;
        const double tmo = 40.0e9;
        bool done = false, saw_done = false;

        // upload finish = the last write-drain completion inside its window
        auto close_upl = [&](double) {
            if (open_upl >= 0) {
                up_st.add(last_idle_rise - t0
                          - eng[size_t(open_upl)].golden_finish, n);
                open_upl = -1;
            }
        };

        while (!done) {
            const bool present = (idx < rec.words.size()) &&
                                 (rig.now_ns() - t0 >= word_ready[idx]);
            rig.tb.i_fifo_valid = present;
            rig.tb.i_fifo_word  = present ? rec.words[idx] : 0;
            rig.tick();
            if (rig.pop && present) idx++;

            // ---- monitor taps (all 1-cycle pulses / levels) ----
            if (rig.tb.o_dsc_vld) {
                close_upl(rig.now_ns());
                const uint32_t d0 = rig.tb.o_dsc_dst0;
                const int npx = rig.tb.o_dsc_npx, rows = rig.tb.o_dsc_rows;
                // probe forward over draws only; the study's u16 clip view
                // omits some draws the engine legitimately executes
                // (negative-dst clamps) - those are EXTRAS with no golden
                // reference, measured as no-ops.  Draws the study lists but
                // the engine rejects (wrap guard) are GHOSTS, skipped only
                // when a later match commits the scan.
                size_t q = ep;
                while (q < eng.size() && !eng[q].upload &&
                       !(eng[q].dst0 == d0 && eng[q].w == npx && eng[q].h == rows))
                    q++;
                if (q < eng.size() && !eng[q].upload) {
                    g_ghosts += long(q - ep);
                    dsc_q.push_back(long(q));
                    ep = q + 1;
                }
                else {
                    g_extras++;
                    dsc_q.push_back(-1);
                }
            }
            if (rig.tb.o_dsc_upl) {
                close_upl(rig.now_ns());
                while (ep < eng.size() && !eng[ep].upload) { g_ghosts++; ep++; }
                if (ep >= eng.size()) {
                    std::printf("FATAL exec-%ld: upload match lost\n", n);
                    return 1;
                }
                open_upl = long(ep); ep++;
            }
            if (rig.tb.o_op_srv) {
                if (dsc_q.empty()) { std::printf("FATAL: srv w/o dsc\n"); return 1; }
                if (dsc_q.front() >= 0)
                    await.push_back({size_t(dsc_q.front()), rig.now_ns()});
                dsc_q.pop_front();
            }
            const bool wi = rig.tb.o_wr_idle;
            if (wi && !prev_wr_idle) last_idle_rise = rig.now_ns();
            if (wi) {
                while (!await.empty()) {
                    const auto a = await.front();
                    dr_st.add(rig.now_ns() - t0 - eng[a.i].golden_finish, n);
                    await.pop_front();
                }
            }
            prev_wr_idle = wi;

            if (rig.tb.o_done && !prev_done) close_upl(rig.now_ns());
            prev_done = rig.tb.o_done;
            saw_done |= bool(rig.tb.o_done);     // o_done is a 1-cycle pulse

            if (saw_done && rig.tb.o_bat_idle && idx >= rec.words.size()
                && await.empty())
                done = true;
            if ((rig.ticks & 0x3FFFFF) == 0)
                std::printf("[dbg] exec %ld t=%.2fms fed=%zu/%zu busy=%d batidle=%d "
                            "rd=%llu wr=%llu own=%d oqc=%d oqh=%d oql=%d rq=%zu\n",
                            n, (rig.now_ns() - t0) / 1e6, idx, rec.words.size(),
                            int(rig.tb.o_busy), int(rig.tb.o_bat_idle),
                            (unsigned long long)rig.ddr.n_rd_words,
                            (unsigned long long)rig.ddr.n_wr_words,
                            int(rig.tb.rootp->tb_h7__DOT__u_harness__DOT__own),
                            0,
                            int(rig.tb.rootp->tb_h7__DOT__u_harness__DOT__oq_head_v),
                            int(rig.tb.rootp->tb_h7__DOT__u_harness__DOT__oq_left),
                            rig.ddr.rq.size());
            if (rig.now_ns() - t0 > tmo) {
                std::printf("TIMEOUT exec-%ld: fed %zu/%zu busy=%d\n",
                            n, idx, rec.words.size(), int(rig.tb.o_busy));
                return 1;
            }
        }
        rig.tb.i_fifo_valid = 0;
        // drain the DDRAM face: the exec's last posted word can sit in the
        // WE register across BUSY/turnaround stalls well past o_done
        for (int q = 0; q < 4; ) {
            rig.tick();
            q = (!rig.tb.DDRAM_WE && rig.ddr.wr_busy_until == 0) ? q + 1 : 0;
        }

        gold_e.exec(rec.words, rec.clip_x, rec.clip_y);
        if (!compare_vram(rig, gold_e, n)) return 1;
        n++;
        if ((n % 20) == 0)
            std::printf("[h7] %ld execs pixel-exact  (draw lateness max %.2f us, >hline %ld)\n",
                        n, dr_st.max_late / 1e3, dr_st.n_hline);
    }

    std::printf("[h7] %s: %ld execs pixel-exact through batch+harness+DDRAM(stat seed=%u)\n",
                trace, n, seed);
    std::printf("[h7] DRAW lateness:   n=%ld  max=%+.2f us  min=%+.2f us  n>0=%ld  n>hline=%ld (worst exec %ld)\n",
                dr_st.n, dr_st.max_late / 1e3, dr_st.max_early / 1e3,
                dr_st.n_pos, dr_st.n_hline, dr_st.worst_exec);
    std::printf("[h7] UPLOAD lateness: n=%ld  max=%+.2f us  n>hline=%ld\n",
                up_st.n, up_st.max_late / 1e3, up_st.n_hline);
    std::printf("[h7] ghosts=%ld extras=%ld  ddr: %llu rd words, %llu wr words, %llu bursts\n",
                g_ghosts, g_extras, (unsigned long long)rig.ddr.n_rd_words,
                (unsigned long long)rig.ddr.n_wr_words,
                (unsigned long long)rig.ddr.n_bursts);

    int rc = (dr_st.n_hline || up_st.n_hline) ? 1 : 0;

    // -----------------------------------------------------------------
    // video frame check: scan two frames, compare the second against a
    // C++ render of the DDRAM image at the final scroll
    // -----------------------------------------------------------------
    if (do_frame) {
        std::vector<uint16_t> frame(320 * 240, 0xFFFF);
        int vs = 0; long px_i = 0;
        bool prev_vs = false;
        while (vs < 3) {
            rig.tick();
            if (rig.tb.o_vsync && !prev_vs) { vs++; px_i = 0; }
            prev_vs = rig.tb.o_vsync;
            if (vs == 2 && rig.tb.o_px_de && px_i < 320 * 240)
                frame[size_t(px_i++)] = rig.tb.o_px;
        }
        long bad = 0;
        for (int y = 0; y < 240; y++) {
            const uint32_t yv = (uint32_t(last_sy) + uint32_t(y)) & 4095;
            const uint32_t x0 = uint32_t(last_sx) & 0x1FE0;   // 32-px aligned
            const uint32_t off = uint32_t(last_sx) & 31;
            for (int x = 0; x < 320; x++) {
                const uint32_t i = off + uint32_t(x);
                const uint32_t beat = (x0 + (i & ~3u)) & 8191;
                const uint32_t addr = (yv * 8192 + beat + (i & 3)) & 0x1FFFFFF;
                const uint16_t want = rig.ddr.mem[addr];
                if (frame[size_t(y) * 320 + size_t(x)] != want) {
                    if (bad < 6)
                        std::printf("  FRAME MISMATCH (%d,%d): rtl=%04x want=%04x\n",
                                    x, y, frame[size_t(y) * 320 + size_t(x)], want);
                    bad++;
                }
            }
        }
        std::printf("[h7] video frame vs DDRAM render: %s (%ld bad px, scroll %u,%u)\n",
                    bad ? "FAIL" : "PIXEL-EXACT", bad, last_sx, last_sy);
        if (bad) rc = 1;
    }

    std::printf("[h7] %s\n", rc ? "FAIL" : "PASS");
    return rc;
}
