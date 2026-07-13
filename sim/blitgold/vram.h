// ikacore CV1k - golden VRAM model (functional, no DDR timing).  [H1 / I-1.3]
//
// The blitter's VRAM is 2x MT46V16M16 in lockstep = one logical 8192x4096
// ARGB1555 image = 64 MB (blitter_detail.md §6.1).  This model stores it as a
// FLAT, linear u32[y*8192 + x] framebuffer - exactly MAME's `bitmap_rgb32`
// layout (cv1k_v.cpp), one u32 per pixel.
//
// Why linear (and not the DDR tile-swizzle of §6.1 / px2addr):
//   The §6.1 BA/ROW/COL mapping (one DRAM row = one 32x32-px tile) is a HARDWARE
//   detail that changes only WHICH DRAM rows a draw touches -> it feeds the
//   timing/cost model, never the pixel values.  Functionally the swizzle is
//   invisible: the same 64 MB, addressed differently.  So the golden pixel
//   model is linear (matches MAME); the swizzle lives in the RTL VRAM backend
//   (H3) and is validated by diffing that backend's output against this model.
//
// Pixel u32 "pen" format (MAME cv1k_v, --t- ---- rrrrr--- ggggg--- bbbbb---):
//   bit29 = t (opaque/alpha flag)   bits[23:19]=R5  bits[15:11]=G5  bits[7:3]=B5
// i.e. 5-bit channels left-justified within their byte, alpha at 0x20000000.
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace gold {

static constexpr int VRAM_W = 0x2000;   // 8192
static constexpr int VRAM_H = 0x1000;   // 4096
static constexpr uint32_t PEN_T = 0x20000000u;   // opaque/alpha flag bit (bit29)

// ARGB1555 (as stored in the op-list UPLOAD payload) -> u32 pen.
// Exact MAME conversion (cv1k_v.cpp gfx_upload):
//   t = a<<29, r = R5<<19, g = G5<<11, b = B5<<3.
static inline uint32_t argb1555_to_pen(uint16_t p)
{
    return ((uint32_t(p) & 0x8000u) << 14)   // A  -> bit29
         | ((uint32_t(p) & 0x7c00u) <<  9)   // R5 -> bits[23:19]
         | ((uint32_t(p) & 0x03e0u) <<  6)   // G5 -> bits[15:11]
         | ((uint32_t(p) & 0x001fu) <<  3);  // B5 -> bits[7:3]
}

// u32 pen -> ARGB1555 (for PNG export / readback). Inverse of the above.
static inline uint16_t pen_to_argb1555(uint32_t v)
{
    return uint16_t(((v & PEN_T)   ? 0x8000u : 0u)
                  | ((v >> 9) & 0x7c00u)
                  | ((v >> 6) & 0x03e0u)
                  | ((v >> 3) & 0x001fu));
}

// pen -> 8-bit-per-channel RGB for PNG (5-bit expanded to 8 via bit replication).
static inline void pen_to_rgb8(uint32_t v, uint8_t &r, uint8_t &g, uint8_t &b)
{
    const uint8_t r5 = uint8_t((v >> 19) & 0x1f);
    const uint8_t g5 = uint8_t((v >> 11) & 0x1f);
    const uint8_t b5 = uint8_t((v >>  3) & 0x1f);
    r = uint8_t((r5 << 3) | (r5 >> 2));
    g = uint8_t((g5 << 3) | (g5 >> 2));
    b = uint8_t((b5 << 3) | (b5 >> 2));
}

class VRAM {
public:
    static constexpr size_t SIZE = size_t(VRAM_W) * VRAM_H;

    VRAM() : m_px(SIZE, 0) {}

    inline uint32_t  get(int x, int y) const { return m_px[size_t(y) * VRAM_W + x]; }
    inline uint32_t &at (int x, int y)       { return m_px[size_t(y) * VRAM_W + x]; }
    inline uint32_t &flat(size_t i)          { return m_px[i]; }
    inline uint32_t *row(int y)              { return &m_px[size_t(y) * VRAM_W]; }

    void clear(uint32_t v = 0) { std::fill(m_px.begin(), m_px.end(), v); }

    const uint32_t *data() const { return m_px.data(); }

    // 64-bit FNV-1a over the whole VRAM - cheap regression / trace-equivalence key.
    uint64_t hash() const
    {
        uint64_t h = 1469598103934665603ull;
        for (uint32_t v : m_px) { h ^= v; h *= 1099511628211ull; }
        return h;
    }

private:
    std::vector<uint32_t> m_px;
};

} // namespace gold
