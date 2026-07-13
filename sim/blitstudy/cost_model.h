// C++ port of sim/scripts/blit_cost_model.py (ORIGINAL model only) —
// the golden timing governor. Accept: anchors 8x8=93 / 16x12=189 /
// 240x64@768,0=12090 VCLK; 80x clipped ~17.5us; upload 256x5 ~58.77us.
// This is the formula the RTL pacing FSM will implement (BD §7.6).
#pragma once

#include <cstdint>
#include <vector>

namespace cost {

constexpr double V_NS = 13.0208;            // VRAM CLK (76.8 MHz)
constexpr double HLINE_PERIOD_NS = 63586.0;
constexpr int    HLINE_STEAL_VCLK = 166;

// per-tile-span penalties (see vram_tile_spans below — NOT per objline)
struct PSet { int src, rw, wr, spr; };
constexpr PSet P_PDF  = {5, 20, 10, 10};    // hits all three anchors exactly
constexpr PSet P_MAME = {6, 20, 11, 12};    // MAME/upstream set (cross-check)

// op-list fetch model (BREQ/BACK chunk cadence, blitter_detail §5)
constexpr double T_CHUNK_NS   = 700.0;
constexpr double T_CHUNK_UPLD = 1442.5;
constexpr double T_EXEC2BRQ   = 200.0;      // P-22 Level A placeholder
constexpr int    CHUNK        = 64;

// 32x32-px VRAM tile spans touched by a rect: one tile = 2 KiB = one DDR1
// page across the chip pair, so this counts page activations. Mirrors MAME
// calculate_vram_accesses ("VRAM data is laid out in 32x32 pixel rows");
// named vram_rows in blit_cost_model.py — renamed here because these spans
// are unrelated to the objlines the DDR3 engine batches (K=8).
inline int vram_tile_spans(int x, int y, int w, int h)
{
    int xr = 0;
    for (int xp = w; xp > 0; xp -= 32) {
        xr += 1;
        if ((x & 31) + (xp < 32 ? xp : 32) > 32) xr += 1;
    }
    int n = 0;
    for (int yp = h; yp > 0; yp -= 32) {
        n += xr;
        if ((y & 31) + (yp < 32 ? yp : 32) > 32) n += xr;
    }
    return n;
}

// blitter_detail §6.5 draw_cost_vclk (blend/trans do NOT change cost)
inline int64_t orig_draw_vclk(int sx, int sy, int dx, int dy, int w, int h,
                              const PSet &P = P_PDF)
{
    const int64_t src_px = int64_t(w) * h;
    const int dx0 = dx & ~3;
    const int dx1 = (dx + w - 1) | 3;
    const int64_t dst_px = int64_t(dx1 - dx0 + 1) * h;
    return src_px / 4 + 2 * (dst_px / 4)
         + int64_t(vram_tile_spans(sx, sy, w, h)) * P.src
         + int64_t(vram_tile_spans(dx, dy, w, h)) * (P.rw + P.wr)
         + P.spr;
}

// One op as seen by the governor.
struct GovOp {
    int    nbytes;        // list bytes (draw/clipped=20, clip-op=4, upload=16+2wh)
    bool   upload;        // payload streams at the upload chunk cadence
    double cost_ns;       // engine cost (0 for clip ops / fully clipped / upload)
};

// absolute time each op's last byte is in the FIFO
inline std::vector<double> fetch_ready_times(const std::vector<GovOp> &ops)
{
    std::vector<double> ready;
    ready.reserve(ops.size());
    double t = T_EXEC2BRQ;
    long filled = 0, pos = 0;
    for (const auto &op : ops) {
        pos += op.nbytes;
        while (filled < pos) {
            t += op.upload ? T_CHUNK_UPLD : T_CHUNK_NS;
            filled += CHUNK;
        }
        ready.push_back(t);
    }
    return ready;
}

// engine-side hline steal: +2.16us at each free-running 63.586us boundary
inline double add_steals(double t0, double dur_ns)
{
    double end = t0 + dur_ns;
    long k = long(t0 / HLINE_PERIOD_NS) + 1;
    while (k * HLINE_PERIOD_NS < end) {
        end += HLINE_STEAL_VCLK * V_NS;
        k += 1;
    }
    return end;
}

struct GovTimeline {
    std::vector<double> start;    // golden op start (ns from EXEC)
    std::vector<double> finish;   // golden op finish (ns from EXEC)
    double busy_end = 0.0;        // golden BUSY deassert / frame blit time
};

// two-process coupling: op_start = max(engine_free, fetch_ready)
inline GovTimeline governor(const std::vector<GovOp> &ops, bool steals = true)
{
    GovTimeline tl;
    const auto ready = fetch_ready_times(ops);
    double engine_free = 0.0;
    tl.start.reserve(ops.size());
    tl.finish.reserve(ops.size());
    for (size_t i = 0; i < ops.size(); i++) {
        const double start = engine_free > ready[i] ? engine_free : ready[i];
        engine_free = steals ? add_steals(start, ops[i].cost_ns)
                             : start + ops[i].cost_ns;
        tl.start.push_back(start);
        tl.finish.push_back(engine_free);
    }
    tl.busy_end = engine_free;
    return tl;
}

// self-test — call at program start; returns false on any anchor miss
bool validate_anchors();

inline bool validate_anchors()
{
    if (orig_draw_vclk(0, 0, 0, 0, 8, 8) != 93) return false;
    if (orig_draw_vclk(0, 0, 0, 0, 16, 12) != 189) return false;
    if (orig_draw_vclk(768, 0, 768, 0, 240, 64) != 12090) return false;

    // 80x clipped 1x324 (fetch-bound) ~ 17.5 us +-3%
    std::vector<GovOp> clip80(80, GovOp{20, false, 0.0});
    const double t_clip = governor(clip80).busy_end / 1000.0;
    if (t_clip < 17.5 * 0.97 || t_clip > 17.5 * 1.03) return false;

    // upload 256x5 (bus-bound) ~ 58.77 us +-3%
    std::vector<GovOp> up1{GovOp{16 + 2 * 256 * 5, true, 0.0}};
    const double t_up = governor(up1).busy_end / 1000.0;
    if (t_up < 58.77 * 0.97 || t_up > 58.77 * 1.03) return false;

    return true;
}

} // namespace cost
