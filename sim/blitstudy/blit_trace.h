// ikacore CV1k — .blit workload trace reader.
// Format spec: docs/blitter_ddr3_sched.md §10.1; emitter:
// mame/src/mame/cave/cv1k_v.cpp trace_exec_record() (patch kept in
// sim/blitstudy/mame_blit_trace.patch).
//
// File: 8-byte header {'B','L','T','1', u32 version}, then EXEC records:
//   u32 "EXEC" | u64 t_ns | u64 frame | u32 gfx_addr |
//   u16 clip_x, clip_y, scroll_x, scroll_y | u64 mame_delay_ns |
//   u32 nwords | u16 words[nwords]     (exact shadow-walk order, no dups)
// All little-endian.
#pragma once

#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace blit {

struct ExecRecord {
    uint64_t t_ns = 0;         // MAME machine time at the EXEC write
    uint64_t frame = 0;        // screen frame number
    uint32_t gfx_addr = 0;     // LIST_ADDR latched at EXEC
    uint16_t clip_x = 0, clip_y = 0;     // shadow-latched at EXEC
    uint16_t scroll_x = 0, scroll_y = 0; // live registers at EXEC
    uint64_t mame_delay_ns = 0;          // MAME's built-in (Buffi) estimate
    std::vector<uint16_t> words;         // op stream
};

enum class OpKind : uint8_t { End, Clip, Upload, Draw };

struct Op {
    OpKind kind;
    size_t off;     // word offset into ExecRecord::words
    size_t nwords;  // total size of the op in words
};

// Walk one record's op stream. Mirrors gfx_create_shadow_copy():
//   0x0000/0xf000 -> END (1 word, stream stops)
//   0xC000        -> CLIP, 2 words (op, cliptype)
//   0x2000        -> UPLOAD, 8 header words + dimx*dimy data words
//                    (dimx = (w[6] & 0x1fff)+1, dimy = (w[7] & 0x0fff)+1)
//   0x1000        -> DRAW, 10 words
// Throws on unknown opcodes or truncation — a failed walk means the trace
// (or this walker) is wrong; never "salvage".
inline std::vector<Op> walk(const std::vector<uint16_t> &w)
{
    std::vector<Op> ops;
    size_t i = 0;
    while (i < w.size()) {
        const uint16_t op = w[i];
        switch (op & 0xf000) {
        case 0x0000:
        case 0xf000:
            ops.push_back({OpKind::End, i, 1});
            return ops;
        case 0xc000:
            if (i + 2 > w.size()) throw std::runtime_error("truncated CLIP");
            ops.push_back({OpKind::Clip, i, 2});
            i += 2;
            break;
        case 0x2000: {
            if (i + 8 > w.size()) throw std::runtime_error("truncated UPLOAD header");
            const size_t dimx = (w[i + 6] & 0x1fff) + 1;
            const size_t dimy = (w[i + 7] & 0x0fff) + 1;
            const size_t n = 8 + dimx * dimy;
            if (i + n > w.size()) throw std::runtime_error("truncated UPLOAD data");
            ops.push_back({OpKind::Upload, i, n});
            i += n;
            break;
        }
        case 0x1000:
            if (i + 10 > w.size()) throw std::runtime_error("truncated DRAW");
            ops.push_back({OpKind::Draw, i, 10});
            i += 10;
            break;
        default:
            throw std::runtime_error("unknown op 0x" + std::to_string(op));
        }
    }
    throw std::runtime_error("op stream ended without END");
}

// DRAW field accessors (word offsets per gfx_draw_shadow_copy walk order).
struct DrawView {
    const uint16_t *w;
    explicit DrawView(const uint16_t *p) : w(p) {}
    uint16_t attr0()  const { return w[0]; }
    uint16_t attr1()  const { return w[1]; }
    uint16_t src_x()  const { return w[2]; }
    uint16_t src_y()  const { return w[3]; }
    uint16_t dst_x()  const { return w[4]; }  // signed on hw; raw here
    uint16_t dst_y()  const { return w[5]; }
    uint16_t dimx()   const { return uint16_t((w[6] & 0x1fff) + 1); }
    uint16_t dimy()   const { return uint16_t((w[7] & 0x0fff) + 1); }
    uint16_t alpha0() const { return w[8]; }
    uint16_t alpha1() const { return w[9]; }
};

class TraceReader {
public:
    explicit TraceReader(const std::string &path)
    {
        m_fp = std::fopen(path.c_str(), "rb");
        if (!m_fp) throw std::runtime_error("cannot open " + path);
        uint8_t hdr[8];
        if (std::fread(hdr, 1, 8, m_fp) != 8 || hdr[0] != 'B' || hdr[1] != 'L' ||
            hdr[2] != 'T' || hdr[3] != '1')
            throw std::runtime_error("bad .blit header");
        m_version = uint32_t(hdr[4]) | uint32_t(hdr[5]) << 8 |
                    uint32_t(hdr[6]) << 16 | uint32_t(hdr[7]) << 24;
    }
    ~TraceReader() { if (m_fp) std::fclose(m_fp); }
    TraceReader(const TraceReader &) = delete;
    TraceReader &operator=(const TraceReader &) = delete;

    uint32_t version() const { return m_version; }

    // Returns false on clean EOF; throws on corruption.
    bool next(ExecRecord &rec)
    {
        uint32_t magic;
        if (std::fread(&magic, 4, 1, m_fp) != 1) return false; // EOF
        if (magic != 0x43455845) throw std::runtime_error("bad record magic");
        uint16_t regs[4];
        uint32_t nwords;
        if (std::fread(&rec.t_ns, 8, 1, m_fp) != 1 ||
            std::fread(&rec.frame, 8, 1, m_fp) != 1 ||
            std::fread(&rec.gfx_addr, 4, 1, m_fp) != 1 ||
            std::fread(regs, 2, 4, m_fp) != 4 ||
            std::fread(&rec.mame_delay_ns, 8, 1, m_fp) != 1 ||
            std::fread(&nwords, 4, 1, m_fp) != 1)
            throw std::runtime_error("truncated record header");
        rec.clip_x = regs[0]; rec.clip_y = regs[1];
        rec.scroll_x = regs[2]; rec.scroll_y = regs[3];
        rec.words.resize(nwords);
        if (nwords && std::fread(rec.words.data(), 2, nwords, m_fp) != nwords)
            throw std::runtime_error("truncated word stream");
        return true;
    }

private:
    std::FILE *m_fp = nullptr;
    uint32_t m_version = 0;
};

} // namespace blit
