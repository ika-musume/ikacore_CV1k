// ikacore CV1k - golden pixel model.                    [H1 / I-1.4 + I-1.2]
//
// A faithful, timing-free C++ port of MAME's Cave CV1000 blitter
// (docs/mame_cv1k_src/cv1k_v*.{cpp,h,ipp}, BSD-3-Clause, David Haywood et al.)
// used as the pixel-exact GOLDEN REFERENCE the RTL draw engine (H3) is diffed
// against.  Semantics are level C (blitter_detail.md §4/§7.4/§8); the handful of
// not-fully-confirmed constants are tagged with their blitter_todo.md P-row so
// the eventual PCB-measurement overhaul knows exactly what to revisit:
//   - CLIP_MARGIN = 32           -> P-13 ([MAME]/MMP, "B", re-confirmable)
//   - alpha = top 5 of 8 bits    -> P-41 ([MAME] >>3)
//   - tint  = 6-bit, 0x20 = 1.0  -> P-42 ([MAME] >>2)
//   - blend LUT rounding law     -> P-40 (min(31, a*b/31); MAME colrtable)
// This model carries NO timing (that is blitstudy/cost_model.h); it only
// produces pixels.
//
// Op-list decode (I-1.2) mirrors gfx_exec()/gfx_create_shadow_copy():
//   word&0xf000: 0x0000/0xf000=END  0xc000=CLIP  0x2000=UPLOAD  0x1000=DRAW.
#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include "vram.h"

namespace gold {

// blitter_detail.md §8 / MAME CV1K_CLIP_MARGIN.  [blitter_todo P-13, level B]
static constexpr int CLIP_MARGIN = 32;

// ---------------------------------------------------------------------------
// Blend lookup tables (MAME cv1k_v.cpp device_reset).  [blitter_todo P-40]
//   colrtable    [x<32][y<64] = min((x*y)/31, 31)   (5-bit * 6-bit multiply)
//   colrtable_rev[a<32][y<64] = min(((31-a)*y)/31, 31)
//   colrtable_add[x<32][y<32] = min(x+y, 31)
// ---------------------------------------------------------------------------
struct BlendTables {
    uint8_t mul [0x20][0x40];   // colrtable
    uint8_t rev [0x20][0x40];   // colrtable_rev
    uint8_t add [0x20][0x20];   // colrtable_add
    BlendTables()
    {
        for (int y = 0; y < 0x40; y++)
            for (int x = 0; x < 0x20; x++) {
                const uint8_t v = uint8_t(std::min((x * y) / 0x1f, 0x1f));
                mul[x][y]          = v;
                rev[x ^ 0x1f][y]   = v;
            }
        for (int y = 0; y < 0x20; y++)
            for (int x = 0; x < 0x20; x++)
                add[x][y] = uint8_t(std::min(x + y, 0x1f));
    }
};
inline const BlendTables &tables() { static const BlendTables t; return t; }

// 5-bit RGB colour, matching MAME clr_t (values 0..31; tint index up to 0..63).
struct Clr {
    uint8_t r = 0, g = 0, b = 0;

    // pen (--t- ---- rrrrr--- ggggg--- bbbbb---) -> clr  (MAME pen_to_clr)
    static Clr from_pen(uint32_t pen)
    {
        Clr c;
        c.r = uint8_t((pen >> 19) & 0x1f);   // pen>>(16+3), high bits are 0
        c.g = uint8_t((pen >> 11) & 0x1f);   // pen>>(8+3)
        c.b = uint8_t((pen >>  3) & 0x1f);
        return c;
    }
    uint32_t to_pen() const
    {
        return (uint32_t(r) << (16 + 3)) | (uint32_t(g) << (8 + 3)) | (uint32_t(b) << 3);
    }

    // --- MAME clr_t helpers (only the ones the pixel switch actually calls) ---
    void mul(const Clr &t)            { const auto &T = tables(); r = T.mul[r][t.r]; g = T.mul[g][t.g]; b = T.mul[b][t.b]; }
    void square(const Clr &c)         { const auto &T = tables(); r = T.mul[c.r][c.r]; g = T.mul[c.g][c.g]; b = T.mul[c.b][c.b]; }
    void mul_3param(const Clr &c1, const Clr &c2) { const auto &T = tables(); r = T.mul[c2.r][c1.r]; g = T.mul[c2.g][c1.g]; b = T.mul[c2.b][c1.b]; }
    void mul_rev_square(const Clr &c) { const auto &T = tables(); r = T.rev[c.r][c.r]; g = T.rev[c.g][c.g]; b = T.rev[c.b][c.b]; }
    void mul_rev_3param(const Clr &c1, const Clr &c2) { const auto &T = tables(); r = T.rev[c2.r][c1.r]; g = T.rev[c2.g][c1.g]; b = T.rev[c2.b][c1.b]; }
    void mul_fixed(uint8_t v, const Clr &c0)     { const auto &T = tables(); r = T.mul[v][c0.r]; g = T.mul[v][c0.g]; b = T.mul[v][c0.b]; }
    void mul_fixed_rev(uint8_t v, const Clr &c0) { const auto &T = tables(); r = T.rev[v][c0.r]; g = T.rev[v][c0.g]; b = T.rev[v][c0.b]; }
    void copy(const Clr &c0)          { r = c0.r; g = c0.g; b = c0.b; }

    void add(const Clr &c0, const Clr &c1) { const auto &T = tables(); r = T.add[c0.r][c1.r]; g = T.add[c0.g][c1.g]; b = T.add[c0.b][c1.b]; }
    void add_with_clr_mul_fixed(const Clr &c0, uint8_t v, const Clr &mc) { const auto &T = tables();
        r = T.add[c0.r][T.mul[mc.r][v]]; g = T.add[c0.g][T.mul[mc.g][v]]; b = T.add[c0.b][T.mul[mc.b][v]]; }
    void add_with_clr_mul_3param(const Clr &c0, const Clr &c1, const Clr &c2) { const auto &T = tables();
        r = T.add[c0.r][T.mul[c2.r][c1.r]]; g = T.add[c0.g][T.mul[c2.g][c1.g]]; b = T.add[c0.b][T.mul[c2.b][c1.b]]; }
    void add_with_clr_square(const Clr &c0, const Clr &c1) { const auto &T = tables();
        // NB: MAME uses c0.r for all three green/blue adds too (bug-for-bug faithful)
        r = T.add[c0.r][T.mul[c1.r][c1.r]]; g = T.add[c0.r][T.mul[c1.g][c1.g]]; b = T.add[c0.r][T.mul[c1.b][c1.b]]; }
    void add_with_clr_mul_fixed_rev(const Clr &c0, uint8_t v, const Clr &c1) { const auto &T = tables();
        r = T.add[c0.r][T.rev[v][c1.r]]; g = T.add[c0.g][T.rev[v][c1.g]]; b = T.add[c0.b][T.rev[v][c1.b]]; }
    void add_with_clr_mul_rev_3param(const Clr &c0, const Clr &c1, const Clr &c2) { const auto &T = tables();
        r = T.add[c0.r][T.rev[c2.r][c1.r]]; g = T.add[c0.g][T.rev[c2.g][c1.g]]; b = T.add[c0.b][T.rev[c2.b][c1.b]]; }
    void add_with_clr_mul_rev_square(const Clr &c0, const Clr &c1) { const auto &T = tables();
        r = T.add[c0.r][T.rev[c1.r][c1.r]]; g = T.add[c0.g][T.rev[c1.g][c1.g]]; b = T.add[c0.b][T.rev[c1.b][c1.b]]; }
};

// Clip rectangle (inclusive), MAME rectangle semantics.
struct Clip { int min_x, max_x, min_y, max_y; };

// ---------------------------------------------------------------------------
// Per-pixel blend combine - the 64-way s_mode/d_mode switch from
// cv1k_v_pixel.ipp.  Given source colour `s` (already tinted) and destination
// colour `d`, returns the merged 5-bit colour.  s_alpha/d_alpha are 5-bit.
// (blend==true path only; the non-blend paths are handled in draw_sprite.)
// ---------------------------------------------------------------------------
inline Clr blend_combine(int smode, int dmode, Clr s, const Clr &d,
                         uint8_t s_alpha, uint8_t d_alpha)
{
    const auto &T = tables();
    const uint8_t *sat = T.mul[s_alpha];   // salpha_table = colrtable[s_alpha]
    const uint8_t *dat = T.mul[d_alpha];   // dalpha_table = colrtable[d_alpha]
    Clr clr0;

    switch (smode) {
    case 0:
        switch (dmode) {
        case 0:  // s_alpha*src + d_alpha*dst   (very common: ingame, titles)
            s.r = T.add[sat[s.r]][dat[d.r]]; s.g = T.add[sat[s.g]][dat[d.g]]; s.b = T.add[sat[s.b]][dat[d.b]]; return s;
        case 1:
            s.r = T.add[sat[s.r]][T.mul[s.r][d.r]]; s.g = T.add[sat[s.g]][T.mul[s.g][d.g]]; s.b = T.add[sat[s.b]][T.mul[s.b][d.b]]; return s;
        case 2:  clr0.mul_fixed(s_alpha, s); s.add_with_clr_square(clr0, d); return s;
        case 3:  clr0.mul_fixed(s_alpha, s); s.add(clr0, d); return s;
        case 4:  clr0.mul_fixed(s_alpha, s); s.add_with_clr_mul_fixed_rev(clr0, d_alpha, d); return s;
        case 5:
            s.r = T.add[sat[s.r]][T.rev[s.r][d.r]]; s.g = T.add[sat[s.g]][T.rev[s.g][d.g]]; s.b = T.add[sat[s.b]][T.rev[s.b][d.b]]; return s;
        case 6:  clr0.mul_fixed(s_alpha, s); s.add_with_clr_mul_rev_square(clr0, d); return s;
        default: clr0.mul_fixed(s_alpha, s); s.add(clr0, d); return s;   // 7
        }
    case 2:
        switch (dmode) {
        case 0:  // heavy on espgal2 highscore
            s.r = T.add[T.mul[d.r][s.r]][dat[d.r]]; s.g = T.add[T.mul[d.g][s.g]][dat[d.g]]; s.b = T.add[T.mul[d.b][s.b]][dat[d.b]]; return s;
        case 1:  clr0.mul_3param(s, d); s.add_with_clr_mul_3param(clr0, d, s); return s;
        case 2:  clr0.mul_3param(s, d); s.add_with_clr_square(clr0, d); return s;
        case 3:  clr0.mul_3param(s, d); s.add(clr0, d); return s;
        case 4:  clr0.mul_3param(s, d); s.add_with_clr_mul_fixed_rev(clr0, d_alpha, d); return s;
        case 5:  clr0.mul_3param(s, d); s.add_with_clr_mul_rev_3param(clr0, d, s); return s;
        case 6:  clr0.mul_3param(s, d); s.add_with_clr_mul_rev_square(clr0, d); return s;
        default: clr0.mul_3param(s, d); s.add(clr0, d); return s;   // 7
        }
    default:
        // smode 1/3/4/5/6/7: clr0 computed here, then the shared d_mode add-block.
        switch (smode) {
        case 1: clr0.square(s); break;
        case 3: clr0.copy(s); break;
        case 4: clr0.mul_fixed_rev(s_alpha, s); break;
        case 5: clr0.mul_rev_square(s); break;
        case 6: clr0.mul_rev_3param(s, d); break;
        default: clr0.copy(s); break;   // 7
        }
        switch (dmode) {
        case 0:  s.add_with_clr_mul_fixed(clr0, d_alpha, d); return s;
        case 1:  s.add_with_clr_mul_3param(clr0, d, s); return s;
        case 2:  s.add_with_clr_square(clr0, d); return s;
        case 3:  s.add(clr0, d); return s;
        case 4:  s.add_with_clr_mul_fixed_rev(clr0, d_alpha, d); return s;
        case 5:  s.add_with_clr_mul_rev_3param(clr0, d, s); return s;
        case 6:  s.add_with_clr_mul_rev_square(clr0, d); return s;
        default: s.add(clr0, d); return s;   // 7
        }
    }
}

// ---------------------------------------------------------------------------
// draw_sprite - port of cv1k_v_in.ipp + cv1k_v_pixel.ipp (all variants folded
// into one runtime function; a golden model favours clarity over the 128
// compile-time specializations MAME uses for speed).
// ---------------------------------------------------------------------------
inline void draw_sprite(VRAM &vram, const Clip &clip,
                        int src_x, int src_y, int dst_x_start, int dst_y_start,
                        int dimx, int dimy, bool flipx, bool flipy,
                        bool tint, bool trans, bool blend, int smode, int dmode,
                        uint8_t s_alpha, uint8_t d_alpha, const Clr &tint_clr)
{
    int yf;
    if (flipx) src_x += (dimx - 1);
    if (flipy) { yf = -1; src_y += (dimy - 1); } else { yf = +1; }

    int starty = 0;
    const int dst_y_end = dst_y_start + dimy;
    if (dst_y_start < clip.min_y) starty = clip.min_y - dst_y_start;
    if (dst_y_end   > clip.max_y) dimy  -= (dst_y_end - 1) - clip.max_y;

    // src-wrap guard (MAME: don't draw if the sprite would wrap the 0x2000 edge)
    if (flipx) {
        if ((src_x & 0x1fff) < ((src_x - (dimx - 1)) & 0x1fff)) return;
    } else {
        if ((src_x & 0x1fff) > ((src_x + (dimx - 1)) & 0x1fff)) return;
    }

    int startx = 0;
    const int dst_x_end = dst_x_start + dimx;
    if (dst_x_start < clip.min_x) startx = clip.min_x - dst_x_start;
    if (dst_x_end   > clip.max_x) dimx  -= (dst_x_end - 1) - clip.max_x;

    for (int y = starty; y < dimy; y++) {
        const int dy = dst_y_start + y;
        int dx = dst_x_start + startx;
        const int ysrc = ((src_y + yf * y) & 0x0fff);
        int sx = flipx ? (src_x - startx) : (src_x + startx);
        const int xend = dimx;   // exclusive dst-x range end (relative), = dimx
        for (int x = startx; x < xend; x++, dx++, sx += (flipx ? -1 : +1)) {
            const uint32_t pen = vram.get(sx & 0x1fff, ysrc);
            if (trans && !(pen & PEN_T)) continue;

            // Flat dst index: clip margin (clip_x-32) can push dx<0.  A flat
            // index reproduces MAME's bitmap.pix() row-underflow wrap for y>=1;
            // truly off-allocation pixels (y==0 & x<0, or past the end) are
            // skipped - real-HW off-map behaviour is unmeasured (§8, level A/B).
            const long didx = long(dy) * VRAM_W + dx;
            if (didx < 0 || size_t(didx) >= VRAM::SIZE) continue;
            uint32_t &dpx = vram.flat(size_t(didx));
            if (!blend && !tint) {           // REALLY_SIMPLE: straight copy
                dpx = pen;
                continue;
            }
            Clr s = Clr::from_pen(pen);
            if (tint) s.mul(tint_clr);
            if (blend) {
                Clr d = Clr::from_pen(dpx);
                s = blend_combine(smode, dmode, s, d, s_alpha, d_alpha);
            }
            dpx = s.to_pen() | (pen & PEN_T);
        }
    }
}

// ---------------------------------------------------------------------------
// Engine: holds a VRAM + shadow clip/scroll, executes op streams.
// ---------------------------------------------------------------------------
class Engine {
public:
    VRAM vram;

    // Execute one EXEC's op-word stream (as carried by a .blit ExecRecord or a
    // backdoor list walk).  clip_x/clip_y are the shadow-latched CLIP origin;
    // the window clip is set at exec start and on each CLIP op (§8, gfx_exec).
    void exec(const std::vector<uint16_t> &w, int clip_x, int clip_y)
    {
        set_window_clip(clip_x, clip_y);
        size_t i = 0;
        while (i < w.size()) {
            const uint16_t op = w[i];
            switch (op & 0xf000) {
            case 0x0000:
            case 0xf000:
                return;                        // END
            case 0xc000:                       // CLIP (op, cliptype)
                if (i + 2 > w.size()) return;
                if (w[i + 1]) set_window_clip(clip_x, clip_y);
                else          m_clip = {0, VRAM_W - 1, 0, VRAM_H - 1};
                i += 2;
                break;
            case 0x2000:                       // UPLOAD
                i = do_upload(w, i);
                break;
            case 0x1000:                       // DRAW
                if (i + 10 > w.size()) return;
                do_draw(&w[i]);
                i += 10;
                break;
            default:
                return;                        // unknown -> stop (MAME popmessage)
            }
        }
    }

private:
    Clip m_clip {0, VRAM_W - 1, 0, VRAM_H - 1};

    void set_window_clip(int clip_x, int clip_y)
    {
        m_clip = { clip_x - CLIP_MARGIN, clip_x + 320 - 1 + CLIP_MARGIN,
                   clip_y - CLIP_MARGIN, clip_y + 240 - 1 + CLIP_MARGIN };
    }

    // UPLOAD: 8 header words + dimx*dimy ARGB1555 payload words, written
    // row-major at (dst_x + x, dst_y + y).  Returns next op offset.
    //
    // Off-the-bottom uploads (dst_y + dimy > 4096): MAME's gfx_upload is raw
    // pointer arithmetic past the bitmap = UB, so it defines nothing; real HW
    // is unmeasured (§8 level A - games never do it).  We define it as the
    // flat index wrapping mod 2^25 - the same rule the H3 RTL gets for free
    // from its 25-bit addressing - so golden and RTL agree over ALL inputs
    // (and this model stops writing outside its own allocation).
    size_t do_upload(const std::vector<uint16_t> &w, size_t i)
    {
        if (i + 8 > w.size()) return w.size();
        const int dst_x = w[i + 4] & 0x1fff;
        const int dst_y = w[i + 5] & 0x0fff;
        const int dimx  = (w[i + 6] & 0x1fff) + 1;
        const int dimy  = (w[i + 7] & 0x0fff) + 1;
        size_t p = i + 8;
        for (int y = 0; y < dimy; y++) {
            for (int x = 0; x < dimx; x++) {
                if (p >= w.size()) return w.size();
                const size_t idx = (size_t(dst_y + y) * VRAM_W + size_t(dst_x + x))
                                   & (VRAM::SIZE - 1);
                vram.flat(idx) = argb1555_to_pen(w[p++]);
            }
        }
        return p;
    }

    // DRAW: parse the 10-word op (blitter_detail.md §4.1) and dispatch.
    void do_draw(const uint16_t *w)
    {
        const uint16_t attr    = w[0];
        const uint16_t alpha   = w[1];
        int src_x              = w[2] & 0x1fff;
        int src_y              = w[3] & 0x0fff;
        const int dst_x        = int16_t(w[4]);     // signed 16-bit
        const int dst_y        = int16_t(w[5]);
        const int dimx         = (w[6] & 0x1fff) + 1;
        const int dimy         = (w[7] & 0x0fff) + 1;
        const uint16_t tint_r  = w[8];
        const uint16_t tint_gb = w[9];

        const int  dmode = attr & 0x0007;
        const int  smode = (attr & 0x0070) >> 4;
        const bool trans = (attr >> 8) & 1;
        bool       blend = (attr >> 9) & 1;
        const bool flipy = (attr >> 10) & 1;
        const bool flipx = (attr >> 11) & 1;

        const uint8_t d_alpha = uint8_t((alpha & 0x00ff)      >> 3);   // top 5 bits [P-41]
        const uint8_t s_alpha = uint8_t(((alpha & 0xff00) >> 8) >> 3);

        // tint: 8-bit -> 6-bit (>>2); 0x80 -> 0x20 = unity  [P-42]
        Clr tint_clr;
        tint_clr.r = uint8_t((tint_r & 0x00ff) >> 2);
        tint_clr.g = uint8_t(((tint_gb >> 8) & 0xff) >> 2);
        tint_clr.b = uint8_t((tint_gb & 0xff) >> 2);
        const bool tint = (tint_clr.r != 0x20) || (tint_clr.g != 0x20) || (tint_clr.b != 0x20);

        // MAME's "surprisingly frequent" blend->copy fast-path collapse.
        if (smode == 0 && s_alpha == 0x1f && dmode == 4 && d_alpha == 0x1f)
            blend = false;

        draw_sprite(vram, m_clip, src_x, src_y, dst_x, dst_y, dimx, dimy,
                    flipx, flipy, tint, trans, blend, smode, dmode,
                    s_alpha, d_alpha, tint_clr);
    }
};

} // namespace gold
