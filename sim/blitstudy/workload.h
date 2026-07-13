// Shared trace→workload builder (factored verbatim out of study_main.cpp
// so fifo_study.cpp reuses the exact same governor/engine view of a
// .blit record — clip semantics mirrored from gfx_create_shadow_copy /
// gfx_draw_shadow_copy, MAME cv1k_v.cpp).
#pragma once

#include "blit_trace.h"
#include "cost_model.h"
#include "engine.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace workload {

struct ExecWork {
    std::vector<cost::GovOp> gov;       // all ops, governor view
    std::vector<engine::EngOp> eng;     // executable ops only (draw/upload)
    std::vector<size_t> eng2gov;        // index map eng -> gov
    uint64_t mame_delay_ns = 0;
    int64_t our_mame_vclk = 0;          // P_MAME draw cost sum (cross-check)
};

// u16 clip semantics mirrored from gfx_create_shadow_copy /
// gfx_draw_shadow_copy (MAME cv1k_v.cpp).
struct Clip {
    uint16_t min_x, max_x, min_y, max_y;
};

constexpr int CLIP_MARGIN = 32;

inline Clip window_clip(uint16_t cx, uint16_t cy)
{
    return Clip{uint16_t(cx - CLIP_MARGIN), uint16_t(cx + 320 - 1 + CLIP_MARGIN),
                uint16_t(cy - CLIP_MARGIN), uint16_t(cy + 240 - 1 + CLIP_MARGIN)};
}

inline ExecWork build_work(const blit::ExecRecord &rec)
{
    ExecWork wk;
    wk.mame_delay_ns = rec.mame_delay_ns;
    Clip clip = window_clip(rec.clip_x, rec.clip_y);

    const auto ops = blit::walk(rec.words);
    for (const auto &op : ops) {
        const uint16_t *w = &rec.words[op.off];
        switch (op.kind) {
        case blit::OpKind::End:
            break;
        case blit::OpKind::Clip:
            clip = w[1] ? window_clip(rec.clip_x, rec.clip_y)
                        : Clip{0, 0x2000 - 1, 0, 0x1000 - 1};
            wk.gov.push_back({4, false, 0.0});
            break;
        case blit::OpKind::Upload: {
            const int dst_x = w[4] & 0x1fff, dst_y = w[5] & 0x0fff;
            const int dimx = (w[6] & 0x1fff) + 1, dimy = (w[7] & 0x0fff) + 1;
            wk.gov.push_back({16 + 2 * dimx * dimy, true, 0.0});
            engine::EngOp e;
            e.upload = true;
            e.dst = {dst_x, dst_y, dst_x + dimx - 1, dst_y + dimy - 1};
            e.src = {};                     // no VRAM source
            e.dx = dst_x; e.w = dimx; e.h = dimy;
            wk.eng2gov.push_back(wk.gov.size() - 1);
            wk.eng.push_back(e);
            break;
        }
        case blit::OpKind::Draw: {
            blit::DrawView d(w);
            const uint16_t sx = d.src_x(), sy = d.src_y();
            uint16_t dx0 = d.dst_x(), dy0 = d.dst_y();
            uint16_t dimx = d.dimx(), dimy = d.dimy();
            const uint16_t dx1 = uint16_t(dx0 + dimx - 1);
            const uint16_t dy1 = uint16_t(dy0 + dimy - 1);
            // fully outside -> costless list entry (still 20 fetch bytes)
            if (dx0 > clip.max_x || dx1 < clip.min_x ||
                dy0 > clip.max_y || dy1 < clip.min_y) {
                wk.gov.push_back({20, false, 0.0});
                break;
            }
            // clamp (MAME keeps src origin, shrinks dims)
            const uint16_t cx0 = std::max(dx0, clip.min_x);
            const uint16_t cy0 = std::max(dy0, clip.min_y);
            const uint16_t cx1 = std::min(dx1, clip.max_x);
            const uint16_t cy1 = std::min(dy1, clip.max_y);
            dimx = uint16_t(cx1 - cx0 + 1);
            dimy = uint16_t(cy1 - cy0 + 1);
            dx0 = cx0; dy0 = cy0;

            const int64_t vclk =
                cost::orig_draw_vclk(sx, sy, dx0, dy0, dimx, dimy);
            wk.our_mame_vclk +=
                cost::orig_draw_vclk(sx, sy, dx0, dy0, dimx, dimy, cost::P_MAME);
            wk.gov.push_back({20, false, double(vclk) * cost::V_NS});

            engine::EngOp e;
            e.src = {sx, sy, sx + dimx - 1, sy + dimy - 1};
            e.dst = {dx0, dy0, dx0 + dimx - 1, dy0 + dimy - 1};
            e.sx = sx; e.dx = dx0; e.w = dimx; e.h = dimy;
            e.blend = true;                 // conservative until decoded finer
            wk.eng2gov.push_back(wk.gov.size() - 1);
            wk.eng.push_back(e);
            break;
        }
        }
    }
    return wk;
}

} // namespace workload
