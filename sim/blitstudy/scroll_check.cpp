// scroll_check — does the blitter ever write into the currently-displayed
// window? Evidence for the double-buffering claim (FINDINGS.md §1 note):
// per EXEC, classify every draw/upload dst pixel as landing in the window
// SCROLL currently points at (front), in the alternate window seen in the
// trace (back), or elsewhere (sprite staging / off-screen).
#include "blit_trace.h"

#include <cinttypes>
#include <map>

namespace {

struct Win { int x, y; };

int64_t overlap(int x0, int y0, int x1, int y1, const Win &w)
{
    const int ox0 = std::max(x0, w.x), ox1 = std::min(x1, w.x + 319);
    const int oy0 = std::max(y0, w.y), oy1 = std::min(y1, w.y + 239);
    if (ox0 > ox1 || oy0 > oy1) return 0;
    return int64_t(ox1 - ox0 + 1) * (oy1 - oy0 + 1);
}

} // namespace

int main(int argc, char **argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: scroll_check <trace>\n"); return 2; }

    // pass 1: collect distinct SCROLL values
    std::map<std::pair<int,int>, int64_t> scrolls;
    {
        blit::TraceReader rd(argv[1]);
        blit::ExecRecord rec;
        while (rd.next(rec)) scrolls[{rec.scroll_x, rec.scroll_y}]++;
    }
    std::printf("scroll values seen:");
    for (auto &s : scrolls)
        std::printf("  (%d,%d) x%" PRId64, s.first.first, s.first.second, s.second);
    std::printf("\n");

    // pass 2: dst pixels vs front (current scroll) / back (previous scroll),
    // with hardware CLIP applied to draws (mirrors study_main build_work:
    // window = clip_x/y +-32 margin, u16 semantics; CLIP op word toggles
    // window-clip vs full-surface; uploads are not clipped)
    int64_t px_front = 0, px_back = 0, px_else = 0, px_total = 0;
    int64_t execs_dirty = 0, execs = 0;      // EXECs writing >0 px into front
    blit::TraceReader rd(argv[1]);
    blit::ExecRecord rec;
    bool have_prev = false;
    Win prev{};
    while (rd.next(rec)) {
        const Win front{rec.scroll_x, rec.scroll_y};
        const Win back = have_prev ? prev : front;
        const auto wclip = [&] {
            struct C { int x0, x1, y0, y1; } c;
            c.x0 = uint16_t(rec.clip_x - 32); c.x1 = uint16_t(rec.clip_x + 320 - 1 + 32);
            c.y0 = uint16_t(rec.clip_y - 32); c.y1 = uint16_t(rec.clip_y + 240 - 1 + 32);
            return c;
        };
        auto clip = wclip();
        int64_t f = 0, b = 0, e = 0;
        for (const auto &op : blit::walk(rec.words)) {
            const uint16_t *w = &rec.words[op.off];
            int dx, dy, dimx, dimy;
            if (op.kind == blit::OpKind::Clip) {
                if (w[1]) clip = wclip();
                else clip = {0, 0x2000 - 1, 0, 0x1000 - 1};
                continue;
            } else if (op.kind == blit::OpKind::Draw) {
                blit::DrawView d(w);
                dx = d.dst_x(); dy = d.dst_y();
                dimx = d.dimx(); dimy = d.dimy();
                // clamp to the live clip window; fully outside -> no pixels
                const int x1 = std::min(dx + dimx - 1, clip.x1);
                const int y1 = std::min(dy + dimy - 1, clip.y1);
                dx = std::max(dx, clip.x0); dy = std::max(dy, clip.y0);
                if (dx > x1 || dy > y1) continue;
                dimx = x1 - dx + 1; dimy = y1 - dy + 1;
            } else if (op.kind == blit::OpKind::Upload) {
                dx = w[4] & 0x1fff; dy = w[5] & 0x0fff;
                dimx = (w[6] & 0x1fff) + 1; dimy = (w[7] & 0x0fff) + 1;
            } else {
                continue;
            }
            const int64_t area = int64_t(dimx) * dimy;
            const int64_t of = overlap(dx, dy, dx + dimx - 1, dy + dimy - 1, front);
            const int64_t ob = have_prev && (back.x != front.x || back.y != front.y)
                             ? overlap(dx, dy, dx + dimx - 1, dy + dimy - 1, back) : 0;
            f += of; b += ob; e += area - of - ob;
        }
        px_front += f; px_back += b; px_else += e; px_total += f + b + e;
        execs++;
        if (f > 0) execs_dirty++;
        prev = front;
        have_prev = true;
    }
    std::printf("dst px into displayed(front) window: %" PRId64 "  (%.4f%%)\n",
                px_front, px_total ? 100.0 * double(px_front) / double(px_total) : 0.0);
    std::printf("dst px into previous(back) window:   %" PRId64 "  (%.4f%%)\n",
                px_back, px_total ? 100.0 * double(px_back) / double(px_total) : 0.0);
    std::printf("dst px elsewhere (staging etc):      %" PRId64 "  (%.4f%%)\n",
                px_else, px_total ? 100.0 * double(px_else) / double(px_total) : 0.0);
    std::printf("EXECs writing into the displayed window: %" PRId64 " / %" PRId64 "\n",
                execs_dirty, execs);
    return 0;
}
