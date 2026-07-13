// ikacore CV1k - dependency-free PNG writer for VRAM captures.  [H1 / I-1.8]
//
// No libpng/zlib dependency: emits a valid PNG using DEFLATE "stored"
// (uncompressed) blocks, so the whole encoder is ~a page of code.  Files are
// larger than a compressed PNG but open in any viewer; fine for debug captures.
// 8-bit RGB (color type 2).  Also exposes an FNV hash of the RGB for regression.
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "vram.h"

namespace gold {

inline uint32_t crc32_of(const uint8_t *p, size_t n, uint32_t crc = 0xffffffffu)
{
    static uint32_t tab[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) c = (c & 1) ? (0xedb88320u ^ (c >> 1)) : (c >> 1);
            tab[i] = c;
        }
        init = true;
    }
    for (size_t i = 0; i < n; i++) crc = tab[(crc ^ p[i]) & 0xff] ^ (crc >> 8);
    return crc;
}

inline void put_be32(std::vector<uint8_t> &v, uint32_t x)
{
    v.push_back(uint8_t(x >> 24)); v.push_back(uint8_t(x >> 16));
    v.push_back(uint8_t(x >> 8));  v.push_back(uint8_t(x));
}

inline void put_chunk(std::vector<uint8_t> &out, const char tag[4], const std::vector<uint8_t> &data)
{
    put_be32(out, uint32_t(data.size()));
    const size_t crc_start = out.size();
    out.insert(out.end(), tag, tag + 4);
    out.insert(out.end(), data.begin(), data.end());
    const uint32_t crc = crc32_of(out.data() + crc_start, out.size() - crc_start) ^ 0xffffffffu;
    put_be32(out, crc);
}

// zlib stream wrapping `raw` in stored deflate blocks (<=65535 B each) + Adler32.
inline void deflate_stored(const std::vector<uint8_t> &raw, std::vector<uint8_t> &z)
{
    z.push_back(0x78); z.push_back(0x01);   // zlib header (CM=8, no preset dict)
    size_t off = 0;
    while (off < raw.size() || raw.empty()) {
        const size_t n = std::min<size_t>(raw.size() - off, 0xffff);
        const bool final = (off + n >= raw.size());
        z.push_back(final ? 1 : 0);                 // BFINAL, BTYPE=00 (stored)
        z.push_back(uint8_t(n)); z.push_back(uint8_t(n >> 8));            // LEN
        z.push_back(uint8_t(~n)); z.push_back(uint8_t((~n) >> 8));        // NLEN
        z.insert(z.end(), raw.begin() + off, raw.begin() + off + n);
        off += n;
        if (raw.empty()) break;
    }
    // Adler-32 of raw
    uint32_t a = 1, b = 0;
    for (uint8_t byte : raw) { a = (a + byte) % 65521; b = (b + a) % 65521; }
    put_be32(z, (b << 16) | a);
}

// Write an 8-bit RGB image (rgb = h rows of w*3 bytes).  Returns false on I/O error.
inline bool write_png_rgb(const std::string &path, int w, int h, const uint8_t *rgb)
{
    std::vector<uint8_t> out = {0x89,'P','N','G',0x0d,0x0a,0x1a,0x0a};

    std::vector<uint8_t> ihdr;
    put_be32(ihdr, uint32_t(w)); put_be32(ihdr, uint32_t(h));
    ihdr.push_back(8);   // bit depth
    ihdr.push_back(2);   // color type 2 = truecolor RGB
    ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);  // deflate/adaptive/no-interlace
    put_chunk(out, "IHDR", ihdr);

    // raw scanlines: filter byte 0 (None) + RGB row
    std::vector<uint8_t> raw;
    raw.reserve(size_t(h) * (size_t(w) * 3 + 1));
    for (int y = 0; y < h; y++) {
        raw.push_back(0);
        raw.insert(raw.end(), rgb + size_t(y) * w * 3, rgb + size_t(y + 1) * w * 3);
    }
    std::vector<uint8_t> idat;
    deflate_stored(raw, idat);
    put_chunk(out, "IDAT", idat);
    put_chunk(out, "IEND", {});

    std::FILE *fp = std::fopen(path.c_str(), "wb");
    if (!fp) return false;
    const bool ok = std::fwrite(out.data(), 1, out.size(), fp) == out.size();
    std::fclose(fp);
    return ok;
}

// Build an RGB buffer from a VRAM rectangle and write it.  (x0,y0,w,h) in VRAM
// pixels; out-of-range pixels are black.  `stride` decimates (nearest pixel):
// stride=1 = full res, stride=8 = 1/8 in each axis (whole 8192x4096 -> 1024x512).
// Returns the FNV hash of the emitted RGB.
inline uint64_t dump_vram_png(const std::string &path, const VRAM &vram,
                              int x0, int y0, int w, int h, int stride = 1)
{
    if (stride < 1) stride = 1;
    const int ow = w / stride, oh = h / stride;
    std::vector<uint8_t> rgb(size_t(ow) * oh * 3);
    uint64_t fnv = 1469598103934665603ull;
    for (int oy = 0; oy < oh; oy++) {
        for (int ox = 0; ox < ow; ox++) {
            uint8_t r = 0, g = 0, b = 0;
            const int vx = x0 + ox * stride, vy = y0 + oy * stride;
            if (vx >= 0 && vx < VRAM_W && vy >= 0 && vy < VRAM_H)
                pen_to_rgb8(vram.get(vx, vy), r, g, b);
            const size_t o = (size_t(oy) * ow + ox) * 3;
            rgb[o] = r; rgb[o + 1] = g; rgb[o + 2] = b;
            fnv = (fnv ^ r) * 1099511628211ull;
            fnv = (fnv ^ g) * 1099511628211ull;
            fnv = (fnv ^ b) * 1099511628211ull;
        }
    }
    write_png_rgb(path, ow, oh, rgb.data());
    return fnv;
}

} // namespace gold
