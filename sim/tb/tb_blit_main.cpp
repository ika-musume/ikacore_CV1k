// ikacore CV1k - H3/H4 blitter testbench harness.
//
// Drives tb_blit (blit_draw + blit_vram_beh + blit_gov) with op-word streams
// exactly as the attribute FIFO would deliver them, runs the SAME stream
// through the H1 golden model (blitgold/golden.h), and compares the full
// 64 MB VRAM after every EXEC.  Pixel-exact or fail - the I-1.5/6/7 accept.
//
// H4: the governor rides the same word stream (warp mode - synthetic arrival
// times); after every exec its per-op {kind, cost} records are compared
// against the C++ golden cost model (blitstudy/cost_model.h via the
// workload.h clip semantics) - cost-exact or fail (I-2.1/I-2.2).  The
// real-time busy/IRQ1 anchors run in the board sim (+blitanchor).
//
// H6 (conformance, no RTL change): the .blit replay additionally
//   (a) BINDS the RTL governor taps to workload::build_work(rec).gov - the
//       EXACT per-op cost array the P-stage jitter engine consumes via
//       cost::governor() (this harness never runs engine::run_exec; the
//       execution-plane DES lives only in blit_study - the two-plane split),
//   (b) folds an order-sensitive gov_hash + an RTL vram_hash so the driver
//       can prove GOVERNOR INVARIANCE: with --jitter SEED the harness inserts
//       seeded stall gaps into the FIFO feed (perturbing ONLY the execution-
//       plane pacing / draw-engine backpressure), and both hashes must be
//       bit-identical to the un-jittered run.
//
//   ./Vtb_blit --selftest [--fuzz N]        unit vectors + 64-way blend grid
//                                           + hazard/clip corners + gov anchors
//                                           + table reload + random fuzz
//   ./Vtb_blit --trace f.blit [--execs N] [--out DIR] [--png] [--jitter SEED]
//                                           .blit replay, per-EXEC compare
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include <sys/stat.h>

#include <verilated.h>
#include "Vtb_blit.h"
#include "Vtb_blit___024root.h"

#include "blitgold/golden.h"
#include "blitgold/png.h"
#include "blitstudy/blit_trace.h"
#include "blitstudy/cost_model.h"
#include "blitstudy/workload.h"

using namespace gold;

// P-set the governor tables currently hold (kept in sync by the table-load
// test; expect_gov() prices draws with it)
static cost::PSet g_pset = cost::P_PDF;

// ---------------------------------------------------------------------------
// RTL rig
// ---------------------------------------------------------------------------
struct Rig {
    Vtb_blit tb;
    uint64_t cycles = 0;

    ~Rig() { tb.final(); }   // run SV final blocks (dsc_check beat-count report)

    // per-exec governor records: {kind, cost} in push order (kind 0 = zero-
    // cost, 1 = draw with VCLK cost, 2 = end/fault)
    std::vector<std::pair<int, int64_t>> gov;

    // H6 execution-plane perturbation: when feed_seed != 0 the FIFO feed
    // inserts seeded stall gaps (i_fifo_valid deasserted) - the draw engine
    // stalls on its existing backpressure while the governor's arrival stream
    // (mirrored from real pops in tick()) is undisturbed.  The recorded gov
    // timeline must be INVARIANT to this; pixels must stay golden-exact.
    uint32_t feed_seed = 0;
    uint32_t feed_rng = 1;
    int      feed_stall = 0;
    bool feed_gate()
    {
        if (feed_seed == 0) return true;      // H3/H4 behaviour: feed at rate
        if (feed_stall > 0) { feed_stall--; return false; }
        feed_rng ^= feed_rng << 13; feed_rng ^= feed_rng >> 17; feed_rng ^= feed_rng << 5;
        if ((feed_rng & 15) == 0) {           // ~1/16 words open a 1..8-cyc stall
            feed_stall = 1 + int((feed_rng >> 8) & 7);
            return false;
        }
        return true;
    }

    Rig()
    {
        tb.i_CLK = 0;
        tb.i_RST_n = 0;
        tb.i_exec = 0;
        tb.i_clip_x = 0;
        tb.i_clip_y = 0;
        tb.i_fifo_valid = 0;
        tb.i_fifo_word = 0;
        tb.i_gov_push = 0;
        tb.i_gov_word = 0;
        tb.i_tbl_we = 0;
        tb.i_tbl_idx = 0;
        tb.i_tbl_data = 0;
        for (int i = 0; i < 8; i++) tick();
        tb.i_RST_n = 1;
        for (int i = 0; i < 4; i++) tick();
    }

    // one clock: settle comb with inputs (clk=0), then posedge.  The word
    // being handed to the decoder this edge is mirrored into the governor's
    // arrival stream, and the governor's per-op debug tap is recorded.
    bool pop = false;
    void tick()
    {
        tb.i_CLK = 0; tb.eval();
        pop = tb.o_fifo_pop;
        tb.i_gov_push = (pop && tb.i_fifo_valid) ? 1 : 0;
        tb.i_gov_word = tb.i_fifo_word;
        tb.i_CLK = 1; tb.eval();
        tb.i_gov_push = 0;
        cycles++;
        if (tb.o_dbg_vld)
            gov.emplace_back(int(tb.o_dbg_kind), int64_t(tb.o_dbg_cost));
    }

    VlUnpacked<SData, 33554432> &mem() { return tb.rootp->tb_blit__DOT__u_vram__DOT__mem; }

    // run one EXEC's word stream; returns false on timeout
    bool run_exec(const std::vector<uint16_t> &w, uint16_t clip_x, uint16_t clip_y)
    {
        gov.clear();
        tb.i_exec = 1; tb.i_clip_x = clip_x; tb.i_clip_y = clip_y;
        tb.i_fifo_valid = 0;
        tick();
        tb.i_exec = 0;

        // jitter stalls stretch wall-clock; give the timeout headroom for it
        const uint64_t per_word = feed_seed ? 800ull : 400ull;
        const uint64_t limit = cycles + per_word * (w.size() + 16) + 4000000ull;
        size_t idx = 0;
        bool done = false;
        while (!done) {
            const bool present = (idx < w.size()) && feed_gate();
            tb.i_fifo_valid = present;
            tb.i_fifo_word  = present ? w[idx] : 0;
            tick();
            if (pop && present) idx++;        // consumed pre-edge (valid & pop)
            if (tb.o_done) done = true;
            if (cycles > limit) {
                std::printf("  TIMEOUT: fed %zu/%zu words, busy=%d\n",
                            idx, w.size(), (int)tb.o_busy);
                return false;
            }
        }
        tb.i_fifo_valid = 0;
        // drain the governor's modeled timeline (warp mode: fast)
        while (tb.o_gov_busy) {
            tick();
            if (cycles > limit + 8000000ull) {
                std::printf("  TIMEOUT: governor never retired\n");
                return false;
            }
        }
        // drain the batch layer's write trains (H7a build; constant 1 else)
        while (!tb.o_bat_idle) {
            tick();
            if (cycles > limit + 12000000ull) {
                std::printf("  TIMEOUT: batch layer never drained\n");
                return false;
            }
        }
        for (int i = 0; i < 4; i++) tick();
        return true;
    }
};

// ---------------------------------------------------------------------------
// compare RTL VRAM vs golden VRAM
// ---------------------------------------------------------------------------
static long g_fail_total = 0;

static bool compare_vram(Rig &rig, const Engine &e, const char *tag)
{
    const uint32_t *g = e.vram.data();
    auto &m = rig.mem();
    long bad = 0;
    for (size_t i = 0; i < VRAM::SIZE; i++) {
        const uint16_t want = pen_to_argb1555(g[i]);
        const uint16_t got  = m[i];
        if (got != want) {
            if (bad < 8)
                std::printf("  MISMATCH %s at (%zu,%zu): rtl=%04x gold=%04x\n",
                            tag, i % VRAM_W, i / VRAM_W, got, want);
            bad++;
        }
    }
    if (bad) {
        std::printf("  [%s] %ld mismatching pixels\n", tag, bad);
        g_fail_total++;
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// H4 governor cost scoreboard: expected per-op {kind, cost} for a word
// stream.  Mirrors workload::build_work (the fifo_study/P-stage view of an
// exec) but keeps integer VCLK costs; draws are priced with g_pset.
// kind: 0 zero-cost (clip / clipped draw / upload), 1 draw, 2 end.
// ---------------------------------------------------------------------------
static std::vector<std::pair<int, int64_t>>
expect_gov(const std::vector<uint16_t> &words, uint16_t clip_x, uint16_t clip_y)
{
    std::vector<std::pair<int, int64_t>> exp;
    workload::Clip clip = workload::window_clip(clip_x, clip_y);

    const auto ops = blit::walk(words);
    for (const auto &op : ops) {
        const uint16_t *w = &words[op.off];
        switch (op.kind) {
        case blit::OpKind::End:
            exp.emplace_back(2, 0);
            break;
        case blit::OpKind::Clip:
            clip = w[1] ? workload::window_clip(clip_x, clip_y)
                        : workload::Clip{0, 0x2000 - 1, 0, 0x1000 - 1};
            exp.emplace_back(0, 0);
            break;
        case blit::OpKind::Upload:
            exp.emplace_back(0, 0);
            break;
        case blit::OpKind::Draw: {
            blit::DrawView d(w);
            const uint16_t sx = d.src_x(), sy = d.src_y();
            uint16_t dx0 = d.dst_x(), dy0 = d.dst_y();
            uint16_t dimx = d.dimx(), dimy = d.dimy();
            const uint16_t dx1 = uint16_t(dx0 + dimx - 1);
            const uint16_t dy1 = uint16_t(dy0 + dimy - 1);
            if (dx0 > clip.max_x || dx1 < clip.min_x ||
                dy0 > clip.max_y || dy1 < clip.min_y) {
                exp.emplace_back(0, 0);           // fully clipped: costless
                break;
            }
            const uint16_t cx0 = std::max(dx0, clip.min_x);
            const uint16_t cy0 = std::max(dy0, clip.min_y);
            const uint16_t cx1 = std::min(dx1, clip.max_x);
            const uint16_t cy1 = std::min(dy1, clip.max_y);
            exp.emplace_back(1, cost::orig_draw_vclk(sx, sy, cx0, cy0,
                                                     uint16_t(cx1 - cx0 + 1),
                                                     uint16_t(cy1 - cy0 + 1),
                                                     g_pset));
            break;
        }
        }
    }
    return exp;
}

static bool gov_check(Rig &rig, const std::vector<uint16_t> &words,
                      uint16_t clip_x, uint16_t clip_y, const char *tag)
{
    const auto exp = expect_gov(words, clip_x, clip_y);
    if (rig.gov.size() != exp.size()) {
        std::printf("  GOV MISMATCH %s: %zu ops recorded, %zu expected\n",
                    tag, rig.gov.size(), exp.size());
        g_fail_total++;
        return false;
    }
    for (size_t i = 0; i < exp.size(); i++) {
        if (rig.gov[i] != exp[i]) {
            std::printf("  GOV MISMATCH %s at op %zu: rtl kind=%d cost=%lld, "
                        "gold kind=%d cost=%lld\n",
                        tag, i, rig.gov[i].first, (long long)rig.gov[i].second,
                        exp[i].first, (long long)exp[i].second);
            g_fail_total++;
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// H6 conformance binding: certify the RTL governor emits, element for element,
// the SAME per-op cost array the P-stage jitter engine consumes.  The study
// feeds workload::build_work(rec).gov -> cost::governor()/fetch_ready_times()
// -> engine::run_exec().  We read ONLY .gov here; engine::run_exec is never
// called in this binary (execution-plane DES stays in blit_study), so the
// two-plane split holds.  build_work omits End ops from .gov; the RTL records
// a kind==2 tap for them, so those are filtered before comparing.
// ---------------------------------------------------------------------------
static long g_bind_fail = 0;

static bool gov_bind_check(Rig &rig, const blit::ExecRecord &rec, const char *tag)
{
    const auto wk = workload::build_work(rec);
    std::vector<std::pair<int, int64_t>> rtl;
    rtl.reserve(rig.gov.size());
    for (const auto &kc : rig.gov)
        if (kc.first != 2) rtl.push_back(kc);          // drop end/fault taps

    if (rtl.size() != wk.gov.size()) {
        std::printf("  BIND MISMATCH %s: rtl %zu cost-ops, study %zu\n",
                    tag, rtl.size(), wk.gov.size());
        g_bind_fail++;
        return false;
    }
    for (size_t i = 0; i < wk.gov.size(); i++) {
        const bool draw = wk.gov[i].cost_ns > 0.0;
        const int kind = draw ? 1 : 0;                 // clip/upload/clipped = 0
        const int64_t cost = draw ? std::llround(wk.gov[i].cost_ns / cost::V_NS) : 0;
        if (rtl[i].first != kind || rtl[i].second != cost) {
            std::printf("  BIND MISMATCH %s op %zu: rtl {%d,%lld} study {%d,%lld}\n",
                        tag, i, rtl[i].first, (long long)rtl[i].second,
                        kind, (long long)cost);
            g_bind_fail++;
            return false;
        }
    }
    return true;
}

// order-sensitive fold of the recorded gov timeline (FNV-1a) - the governor-
// invariance witness the driver compares across --jitter runs.
static uint64_t g_govhash = 1469598103934665603ull;
static void govhash_reset() { g_govhash = 1469598103934665603ull; }
static void govhash_fold(const std::vector<std::pair<int, int64_t>> &g)
{
    for (const auto &kc : g) {
        const uint64_t v = (uint64_t(uint32_t(kc.first)) << 40) ^ uint64_t(kc.second);
        for (int b = 0; b < 8; b++) {
            g_govhash ^= (v >> (8 * b)) & 0xff;
            g_govhash *= 1099511628211ull;
        }
    }
}

// FNV-1a over the whole RTL VRAM - the pixel-invariance witness across jitter.
static uint64_t rtl_vram_hash(Rig &rig)
{
    auto &m = rig.mem();
    uint64_t h = 1469598103934665603ull;
    for (size_t i = 0; i < VRAM::SIZE; i++) {
        h ^= m[i];
        h *= 1099511628211ull;
    }
    return h;
}

// load one governor table entry (I-2.1 runtime-loadable proof path)
static void set_tbl(Rig &rig, int idx, uint32_t val)
{
    rig.tb.i_tbl_we = 1;
    rig.tb.i_tbl_idx = uint8_t(idx);
    rig.tb.i_tbl_data = val;
    rig.tick();
    rig.tb.i_tbl_we = 0;
    rig.tick();
}

static void load_pset(Rig &rig, const cost::PSet &p)
{
    set_tbl(rig, 0, uint32_t(p.src));
    set_tbl(rig, 1, uint32_t(p.rw));
    set_tbl(rig, 2, uint32_t(p.wr));
    set_tbl(rig, 3, uint32_t(p.spr));
    g_pset = p;
}

// ---------------------------------------------------------------------------
// op-stream builders (same field packing as gold_main.cpp helpers)
// ---------------------------------------------------------------------------
static void put_upload(std::vector<uint16_t> &w, int dx, int dy, int dimx, int dimy,
                       const std::vector<uint16_t> &px)
{
    w.push_back(0x2000); w.push_back(0x0000);
    w.push_back(0x9999); w.push_back(0x9999);
    w.push_back(uint16_t(dx)); w.push_back(uint16_t(dy));
    w.push_back(uint16_t(dimx - 1)); w.push_back(uint16_t(dimy - 1));
    for (uint16_t p : px) w.push_back(p);
}
static void put_draw(std::vector<uint16_t> &w, uint16_t attr, uint16_t alpha,
                     int sx, int sy, int dx, int dy, int dimx, int dimy,
                     uint16_t tint_r = 0x0080, uint16_t tint_gb = 0x8080)
{
    w.push_back(attr); w.push_back(alpha);
    w.push_back(uint16_t(sx)); w.push_back(uint16_t(sy));
    w.push_back(uint16_t(dx)); w.push_back(uint16_t(dy));
    w.push_back(uint16_t(dimx - 1)); w.push_back(uint16_t(dimy - 1));
    w.push_back(tint_r); w.push_back(tint_gb);
}
static void put_clip(std::vector<uint16_t> &w, bool window)
{
    w.push_back(0xc000); w.push_back(window ? 0x0001 : 0x0000);
}

// run one op stream through both worlds; compare pixels AND governor costs
static bool both(Rig &rig, Engine &e, std::vector<uint16_t> w,
                 int clip_x, int clip_y, const char *tag)
{
    w.push_back(0x0000);                      // END
    if (!rig.run_exec(w, uint16_t(clip_x), uint16_t(clip_y))) {
        std::printf("  [%s] RTL timeout\n", tag);
        g_fail_total++;
        return false;
    }
    e.exec(w, clip_x, clip_y);
    if (!gov_check(rig, w, uint16_t(clip_x), uint16_t(clip_y), tag))
        return false;
    return compare_vram(rig, e, tag);
}

// deterministic PRNG (xorshift32)
static uint32_t g_rng = 0xC0FFEE01u;
static uint32_t rnd() { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5; return g_rng; }
static int rndi(int lo, int hi) { return lo + int(rnd() % uint32_t(hi - lo + 1)); }

// ---------------------------------------------------------------------------
// selftest
// ---------------------------------------------------------------------------
static int selftest(int fuzz_execs)
{
    Rig rig;                                  // ONE rig + ONE engine: state
    Engine e;                                 // accumulates across sections
    std::printf("[h3] selftest: RTL vs golden, full-VRAM diff per exec\n");

    // src material: two seeded tiles (checker/gradient with transparent holes)
    {
        std::vector<uint16_t> t0, t1;
        for (int y = 0; y < 32; y++)
            for (int x = 0; x < 64; x++) {
                uint16_t p = uint16_t(((x * 41 + y * 17) & 1 ? 0x8000 : 0x0000)
                                    | ((x & 31) << 10) | ((y & 31) << 5) | ((x + y) & 31));
                if (((x ^ y) & 7) == 3) p &= 0x7fff;      // scatter A=0 holes
                t0.push_back(p);
            }
        for (int i = 0; i < 16 * 16; i++)
            t1.push_back(uint16_t(0x8000 | (rnd() & 0x7fff)));
        std::vector<uint16_t> w;
        put_upload(w, 0, 0, 64, 32, t0);
        put_upload(w, 128, 0, 16, 16, t1);
        if (!both(rig, e, w, 0, 0, "seed-tiles")) return 1;
    }

    // 1..7: the golden unit vectors (same ops as blitgold --selftest)
    {
        std::vector<uint16_t> w;
        put_upload(w, 0, 100, 2, 2, {0xFFFF, 0xFC00, 0x0000, 0x801F});
        put_upload(w, 4, 100, 2, 1, {0xFC00, 0x83E0});
        put_draw(w, 0x1000, 0x0000, 4, 100, 100, 50, 2, 1);
        put_upload(w, 10, 110, 1, 1, {0xFC1F});
        put_upload(w, 6, 100, 1, 1, {0x0000});
        put_draw(w, 0x1100, 0x0000, 6, 100, 10, 110, 1, 1);
        put_draw(w, 0x1000, 0x0000, 4, 100, 5, 5, 1, 1, 0x0040, 0x8080);
        put_draw(w, uint16_t(0x1000 | (1 << 9) | (1 << 8) | 4), 0xFFFF, 4, 100, 100, 50, 1, 1);
        put_draw(w, uint16_t(0x1000 | (1 << 9) | (1 << 8)), 0x8080, 4, 100, 100, 50, 1, 1);
        put_upload(w, 20, 100, 3, 1, {0xFC00, 0x83E0, 0x801F});
        put_draw(w, uint16_t(0x1000 | (1 << 11)), 0x0000, 20, 100, 200, 100, 3, 1);
        if (!both(rig, e, w, 0, 0, "unit-vectors")) return 1;
    }

    // 64-way smode x dmode grid, several alpha pairs, tint/trans mixed in;
    // draws stack onto an evolving background = diverse dst inputs
    for (int pass = 0; pass < 4; pass++) {
        static const uint16_t alphas[4] = {0x0000, 0xA753, 0xFFFF, 0x1FE0};
        std::vector<uint16_t> w;
        for (int sm = 0; sm < 8; sm++)
            for (int dm = 0; dm < 8; dm++) {
                const uint16_t attr = uint16_t(0x1000 | (1 << 9) | ((pass & 1) << 8)
                                             | (sm << 4) | dm);
                const uint16_t tr = (pass == 2) ? 0x0040 : 0x0080;
                const uint16_t tgb = (pass == 2) ? 0xC030 : 0x8080;
                put_draw(w, attr, alphas[pass], (sm * 7) & 63, (dm * 3) & 31,
                         40 + sm * 32, 30 + dm * 24, 24, 16, tr, tgb);
            }
        char tag[32];
        std::snprintf(tag, sizeof tag, "blend-grid-p%d", pass);
        if (!both(rig, e, w, 0, 0, tag)) return 1;
    }

    // flips x trans x tint (non-blend paths incl. REALLY_SIMPLE)
    {
        std::vector<uint16_t> w;
        for (int f = 0; f < 4; f++)
            for (int tr = 0; tr < 2; tr++)
                for (int ti = 0; ti < 2; ti++) {
                    const uint16_t attr = uint16_t(0x1000 | (f << 10) | (tr << 8));
                    put_draw(w, attr, 0x0000, 5, 3, 60 + f * 70, 120 + tr * 40 + ti * 20,
                             33, 17, ti ? 0x0060 : 0x0080, ti ? 0x40A0 : 0x8080);
                }
        if (!both(rig, e, w, 0, 0, "flip-trans-tint")) return 1;
    }

    // clip corners: window edges, CLIP toggles, negative dst, row-underflow
    // wrap (dx<0, dy>=1), didx<0 skip, fully-outside, x>8191 row spill,
    // rows past y=4095 (didx >= SIZE skip)
    {
        std::vector<uint16_t> w;
        put_draw(w, 0x1000, 0, 0, 0, -40, -40, 64, 32);        // corner across min
        put_draw(w, 0x1000, 0, 0, 0, 310, 250, 64, 32);        // across max
        put_draw(w, 0x1800, 0, 0, 0, -10, 1, 20, 3);           // flipx + dx<0 wrap
        put_draw(w, 0x1000, 0, 0, 0, -10, 0, 20, 2);           // didx<0 at row 0
        put_clip(w, false);                                    // full-VRAM clip
        put_draw(w, 0x1000, 0, 0, 0, 500, 400, 40, 20);
        put_draw(w, 0x1000, 0, 0, 0, -500, 300, 40, 20);       // fully left, x-trim
        put_clip(w, true);                                     // window again
        put_draw(w, 0x1000, 0, 0, 0, 340, -20, 64, 32);        // corner
        if (!both(rig, e, w, 0, 0, "clip-a")) return 1;

        std::vector<uint16_t> w2;                              // big clip origins
        put_draw(w2, 0x1000, 0, 0, 0, 8180, 4000, 40, 20);     // x spill > 8191
        put_draw(w2, 0x1000, 0, 0, 0, 8100, 4090, 30, 20);     // rows past 4095
        put_draw(w2, uint16_t(0x1000 | (1 << 9)), 0x8080, 0, 0, 8185, 4002, 20, 8);
        if (!both(rig, e, w2, 8100, 4000, "clip-b")) return 1;
    }

    // src 0x2000-edge wrap guard (draw skipped entirely), both flips
    {
        std::vector<uint16_t> w;
        put_upload(w, 8000, 200, 250, 2,
                   std::vector<uint16_t>(500, 0xFFFF));        // material at the edge
        put_draw(w, 0x1000, 0, 8100, 200, 300, 60, 200, 2);    // crosses -> skip
        put_draw(w, 0x1800, 0, 8100, 200, 300, 80, 200, 2);    // flipx form -> skip
        put_draw(w, 0x1000, 0, 8100, 200, 300, 100, 80, 2);    // stays -> draws
        if (!both(rig, e, w, 0, 0, "wrap-guard")) return 1;
    }

    // self-overlap smear: dst = src + small shift, all flip combos - strict
    // and 1-px paths must reproduce golden's sequential feedback exactly
    {
        static const int sh[][2] = {{1,0},{3,0},{-2,0},{4,0},{8,0},{0,1},{1,1},{-3,-1},{5,3}};
        int n = 0;
        for (auto &s : sh)
            for (int f = 0; f < 4; f++) {
                std::vector<uint16_t> w;
                put_draw(w, uint16_t(0x1000 | (f << 10)), 0, 8, 4, 8 + s[0], 4 + s[1], 24, 12);
                char tag[32];
                std::snprintf(tag, sizeof tag, "smear-%d-f%d", n, f);
                if (!both(rig, e, w, 0, 0, tag)) return 1;
                n++;
            }
        // blended self-overlap (dst reads + smear)
        std::vector<uint16_t> w;
        put_draw(w, uint16_t(0x1000 | (1 << 9)), 0x8080, 8, 4, 10, 5, 24, 12);
        if (!both(rig, e, w, 0, 0, "smear-blend")) return 1;
    }

    // upload corners: 1x1, 3x2, row spill at x~8191, draw-from-upload
    {
        std::vector<uint16_t> w;
        put_upload(w, 300, 300, 1, 1, {0x8888});
        put_upload(w, 302, 300, 3, 2, {1, 2, 3, 4, 5, 6});
        put_upload(w, 8190, 500, 5, 2, {10, 11, 12, 13, 14, 15, 16, 17, 18, 19});
        put_draw(w, 0x1000, 0, 302, 300, 320, 310, 3, 2);
        if (!both(rig, e, w, 280, 280, "upload-corners")) return 1;
    }

    // H4 governor cost anchors (I-2.2): 8x8=93, 16x12=189, 240x64=12090 VCLK
    // (the 240x64@768,0 anchor moved to (64,0) - same &31 phases, same cost -
    // so all three sit inside the clip window at clip=(32,32))
    {
        std::vector<uint16_t> w;
        put_draw(w, 0x1000, 0, 0, 0, 0, 0, 8, 8);
        put_draw(w, 0x1000, 0, 0, 0, 0, 0, 16, 12);
        put_draw(w, 0x1000, 0, 64, 0, 64, 0, 240, 64);
        if (!both(rig, e, w, 32, 32, "gov-anchors")) return 1;
        static const int64_t want[3] = {93, 189, 12090};
        for (int i = 0; i < 3; i++) {
            if (rig.gov[size_t(i)] != std::make_pair(1, want[i])) {
                std::printf("  GOV ANCHOR %d: kind=%d cost=%lld, want draw/%lld\n",
                            i, rig.gov[size_t(i)].first,
                            (long long)rig.gov[size_t(i)].second, (long long)want[i]);
                g_fail_total++;
                return 1;
            }
        }
        std::printf("[h4] governor anchors 93/189/12090 VCLK: PASS\n");
    }

    // H4 runtime table reload (I-2.1): load the P_MAME set over the table
    // port, expect its costs on the same draws, restore P_PDF
    {
        load_pset(rig, cost::P_MAME);
        std::vector<uint16_t> w;
        put_draw(w, 0x1000, 0, 0, 0, 0, 0, 8, 8);
        put_draw(w, 0x1000, 0, 0, 0, 0, 0, 16, 12);
        put_draw(w, 0x1000, 0, 64, 0, 64, 0, 240, 64);
        const bool ok = both(rig, e, w, 32, 32, "gov-table-reload");
        std::printf("[h4] table reload P_PDF->P_MAME (8x8 cost %lld->%lld): %s\n",
                    (long long)cost::orig_draw_vclk(0, 0, 0, 0, 8, 8),
                    (long long)cost::orig_draw_vclk(0, 0, 0, 0, 8, 8, cost::P_MAME),
                    ok ? "PASS" : "FAIL");
        load_pset(rig, cost::P_PDF);
        if (!ok) return 1;
    }

    // random fuzz: mixed op lists, random clips per exec
    for (int fx = 0; fx < fuzz_execs; fx++) {
        std::vector<uint16_t> w;
        const int nops = rndi(20, 60);
        for (int i = 0; i < nops; i++) {
            const int kind = rndi(0, 99);
            if (kind < 12) put_clip(w, rndi(0, 1) != 0);
            else if (kind < 28) {
                const int dimx = rndi(1, 16), dimy = rndi(1, 8);
                std::vector<uint16_t> px;
                for (int k = 0; k < dimx * dimy; k++)
                    px.push_back(uint16_t(rnd() & 0xffff));
                put_upload(w, rndi(0, 8191), rndi(0, 4095), dimx, dimy, px);
            }
            else {
                const uint16_t attr = uint16_t(0x1000 | (rnd() & 0x0f77));
                const uint16_t alpha = uint16_t(rnd());
                const int sx = rndi(0, 200), sy = rndi(0, 60);
                const int dx = rndi(-64, 400), dy = rndi(-64, 300);
                const int dimx = rndi(1, 48), dimy = rndi(1, 24);
                const uint16_t tr  = (rnd() & 1) ? 0x0080 : uint16_t(rnd() & 0xff);
                const uint16_t tgb = (rnd() & 1) ? 0x8080 : uint16_t(rnd());
                put_draw(w, attr, alpha, sx, sy, dx, dy, dimx, dimy, tr, tgb);
            }
        }
        const int cx = (rnd() & 3) ? rndi(0, 500) : 0;
        const int cy = (rnd() & 3) ? rndi(0, 400) : 0;
        char tag[32];
        std::snprintf(tag, sizeof tag, "fuzz-%03d", fx);
        if (!both(rig, e, w, cx, cy, tag)) return 1;
    }

    std::printf("[h3] selftest %s (%llu RTL cycles)\n",
                g_fail_total ? "FAILED" : "PASSED: all execs pixel-exact",
                (unsigned long long)rig.cycles);
    return g_fail_total ? 1 : 0;
}

// ---------------------------------------------------------------------------
// op-level bisect inside one exec (H7a debug): replay execs 0..N-1 normally,
// then probe exec N with op-truncated word lists (prefix + END) against the
// golden engine run the same way, binary-searching the first op whose
// inclusion diverges the VRAM.  State is snapshotted/restored around probes.
// ---------------------------------------------------------------------------
static long first_diff(Rig &rig, const Engine &e)
{
    auto &m = rig.mem();
    const uint32_t *g = e.vram.data();
    for (size_t i = 0; i < VRAM::SIZE; i++)
        if (m[i] != pen_to_argb1555(g[i])) return long(i);
    return -1;
}

static int bisect(const char *path, long target)
{
    Rig rig;
    Engine e;
    blit::TraceReader tr(path);
    blit::ExecRecord rec;
    long n = 0;
    while (tr.next(rec)) {
        if (n == target) break;
        if (!rig.run_exec(rec.words, rec.clip_x, rec.clip_y)) return 1;
        e.exec(rec.words, rec.clip_x, rec.clip_y);
        n++;
        if ((n % 200) == 0) std::printf("[bisect] warmup %ld\n", n);
    }
    if (n != target) { std::printf("[bisect] trace too short\n"); return 1; }

    // snapshot pre-exec state
    std::vector<uint16_t> m0(VRAM::SIZE);
    { auto &m = rig.mem(); for (size_t i = 0; i < VRAM::SIZE; i++) m0[i] = m[i]; }
    Engine e0 = e;

    const auto ops = blit::walk(rec.words);
    std::printf("[bisect] exec %ld: %zu ops\n", target, ops.size());

    auto probe = [&](size_t nops) -> long {
        { auto &m = rig.mem(); for (size_t i = 0; i < VRAM::SIZE; i++) m[i] = m0[i]; }
        Engine ep = e0;
        std::vector<uint16_t> w;
        const size_t end_off = (nops < ops.size())
            ? ops[nops].off : rec.words.size();
        w.assign(rec.words.begin(), rec.words.begin() + long(end_off));
        w.push_back(0x0000);                       // END
        if (!rig.run_exec(w, rec.clip_x, rec.clip_y)) return -2;
        ep.exec(w, rec.clip_x, rec.clip_y);
        return first_diff(rig, ep);
    };

    size_t lo = 0, hi = ops.size();               // lo clean, hi divergent
    long d = probe(hi);
    if (d < 0) { std::printf("[bisect] full exec is CLEAN here?!\n"); return 1; }
    std::printf("[bisect] full exec diverges at px %ld (%ld,%ld)\n",
                d, d % long(VRAM_W), d / long(VRAM_W));
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        long r = probe(mid);
        std::printf("[bisect] ops[0..%zu): %s\n", mid,
                    r < 0 ? "clean" : "DIVERGED");
        if (r < 0) lo = mid; else hi = mid;
    }
    const uint16_t *w = &rec.words[ops[hi - 1].off];
    std::printf("[bisect] first divergent op = #%zu kind=%d off=%zu words:",
                hi - 1, int(ops[hi - 1].kind), ops[hi - 1].off);
    for (int k = 0; k < 10 && ops[hi - 1].off + size_t(k) < rec.words.size(); k++)
        std::printf(" %04x", w[k]);
    std::printf("\n");
    long r = probe(hi);
    std::printf("[bisect] with that op: first bad px at (%ld,%ld)\n",
                r % long(VRAM_W), r / long(VRAM_W));
    return 0;
}

// ---------------------------------------------------------------------------
// .blit trace replay
// ---------------------------------------------------------------------------
static int replay(const char *path, long max_execs, const std::string &outdir,
                  bool png, uint32_t jitter)
{
    Rig rig;
    rig.feed_seed = jitter;
    rig.feed_rng  = jitter ? jitter : 1u;
    govhash_reset();
    Engine e;
    blit::TraceReader tr(path);
    blit::ExecRecord rec;
    long n = 0, ok = 0;
    uint16_t lcx = 0, lcy = 0, lsx = 0, lsy = 0;

    while (tr.next(rec)) {
        if (!rig.run_exec(rec.words, rec.clip_x, rec.clip_y)) {
            std::printf("[h6] exec %ld: RTL TIMEOUT\n", n);
            return 1;
        }
        e.exec(rec.words, rec.clip_x, rec.clip_y);
        char tag[32];
        std::snprintf(tag, sizeof tag, "exec-%04ld", n);
        if (!gov_check(rig, rec.words, rec.clip_x, rec.clip_y, tag))
            return 1;                         // governor cost divergence (H4)
        if (!gov_bind_check(rig, rec, tag))
            return 1;                         // RTL != study input array (H6)
        govhash_fold(rig.gov);
        if (compare_vram(rig, e, tag)) ok++;
        else return 1;                        // stop at first divergence
        lcx = rec.clip_x; lcy = rec.clip_y;
        lsx = rec.scroll_x; lsy = rec.scroll_y;
        n++;
        if ((n % 20) == 0)
            std::printf("[h6] %ld execs pixel-exact + gov-bound (%.1fM cycles)\n",
                        n, double(rig.cycles) / 1e6);
        if (n == max_execs) break;
    }

    // gov_hash / rtl_vram_hash are the invariance witnesses: identical with
    // and without --jitter proves the CPU-visible timeline and the pixels are
    // both independent of execution-plane pacing.
    std::printf("[h6] %s: %ld/%ld execs pixel-exact + gov-bound, "
                "gold_hash=%016llx rtl_vram_hash=%016llx gov_hash=%016llx "
                "jitter=%u %llu cycles\n",
                path, ok, n, (unsigned long long)e.vram.hash(),
                (unsigned long long)rtl_vram_hash(rig),
                (unsigned long long)g_govhash, jitter,
                (unsigned long long)rig.cycles);

    if (png) {
        ::mkdir(outdir.c_str(), 0755);
        // RTL VRAM -> golden container for the shared PNG dumper
        Engine r;
        auto &m = rig.mem();
        for (size_t i = 0; i < VRAM::SIZE; i++)
            r.vram.flat(i) = argb1555_to_pen(m[i]);
        const int cw = 384, ch = 304;
        const int x0 = lsx - (cw - 320) / 2, y0 = lsy - (ch - 240) / 2;
        uint64_t hr = dump_vram_png(outdir + "/rtl_scroll.png",  r.vram, x0, y0, cw, ch, 1);
        uint64_t hg = dump_vram_png(outdir + "/gold_scroll.png", e.vram, x0, y0, cw, ch, 1);
        std::printf("[h3] wrote %s/{rtl,gold}_scroll.png rgb_hash rtl=%016llx gold=%016llx%s\n",
                    outdir.c_str(), (unsigned long long)hr, (unsigned long long)hg,
                    hr == hg ? " (match)" : " (MISMATCH)");
        (void)lcx; (void)lcy;
    }
    return (ok == n) ? 0 : 1;
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);

    if (!cost::validate_anchors()) {
        std::printf("FATAL: C++ cost-model anchors failed (cost_model.h)\n");
        return 1;
    }

    const char *trace = nullptr;
    std::string outdir = "build/h3_out";
    long execs = -1, bisect_at = -1;
    int fuzz = 40;
    uint32_t jitter = 0;
    bool self = false, png = false;
    for (int i = 1; i < argc; i++) {
        if      (!std::strcmp(argv[i], "--selftest")) self = true;
        else if (!std::strcmp(argv[i], "--fuzz")   && i + 1 < argc) fuzz = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--trace")  && i + 1 < argc) trace = argv[++i];
        else if (!std::strcmp(argv[i], "--execs")  && i + 1 < argc) execs = std::atol(argv[++i]);
        else if (!std::strcmp(argv[i], "--out")    && i + 1 < argc) outdir = argv[++i];
        else if (!std::strcmp(argv[i], "--jitter") && i + 1 < argc) jitter = uint32_t(std::strtoul(argv[++i], nullptr, 0));
        else if (!std::strcmp(argv[i], "--bisect") && i + 1 < argc) bisect_at = std::atol(argv[++i]);
        else if (!std::strcmp(argv[i], "--png")) png = true;
    }
    if (trace && bisect_at >= 0) return bisect(trace, bisect_at);
    if (self)  return selftest(fuzz);
    if (trace) return replay(trace, execs, outdir, png, jitter);
    std::printf("usage: %s --selftest [--fuzz N]\n"
                "       %s --trace f.blit [--execs N] [--out DIR] [--png] [--jitter SEED]\n",
                argv[0], argv[0]);
    return 2;
}
