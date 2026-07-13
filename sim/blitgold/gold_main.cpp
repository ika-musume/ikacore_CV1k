// ikacore CV1k - golden pixel-model harness.                        [H1]
//
//   ./blitgold --selftest
//        Run synthetic unit vectors (upload/copy/trans/tint/blend/flip) and
//        assert exact pixels.  Exit 0 on pass.
//
//   ./blitgold --trace <f.blit> [--execs N] [--out DIR] [--full] [--cw W --ch H]
//        Replay EXEC records (op streams incl. UPLOAD payloads + per-EXEC
//        clip/scroll) through the golden model; after N execs (or EOF) dump a
//        visible crop at the scroll origin and at the clip origin, optionally
//        the full 8192x4096 VRAM, plus VRAM hashes.  This is the first-images /
//        MAME cross-check path (I-1.4 accept), needing no board sim.
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "../blitstudy/blit_trace.h"
#include "golden.h"
#include "png.h"

using namespace gold;

// Native CV1000 display window (blitter_detail.md §9.1).
static constexpr int DISP_W = 320, DISP_H = 240;

// Dump the display window anchored at (org_x,org_y) but expanded to cw x ch,
// CENTERED - so the extra (cw-320, ch-240) becomes an EQUAL margin on every
// side (default 384x304 = the 320x240 window + the ±32-px CLIP margin all
// around, matching the hardware clip window clip-32 .. clip+320+31 etc).
static uint64_t dump_window(const std::string &path, const VRAM &vram,
                            int org_x, int org_y, int cw, int ch, int stride = 1)
{
    const int x0 = org_x - (cw - DISP_W) / 2;
    const int y0 = org_y - (ch - DISP_H) / 2;
    return dump_vram_png(path, vram, x0, y0, cw, ch, stride);
}

// ------------------------------- self-test --------------------------------
static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::printf("  FAIL: %s\n", msg); g_fail++; } } while (0)

// Build op-stream helpers (semantic u16 words, big-endian-agnostic values).
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

static int selftest()
{
    std::printf("[selftest] golden pixel model\n");

    // Wide-open clip so nothing is clipped (clip origin far from our tiles).
    const int CX = 0, CY = 0;   // window = [-32,351]x[-32,271]; keep tiles inside

    // --- 1. UPLOAD conversion ---
    {
        Engine e;
        // ARGB1555: opaque white 0xFFFF, opaque red 0xFC00, transparent 0x0000, opaque blue 0x801F
        std::vector<uint16_t> w;
        put_upload(w, 0, 0, 2, 2, {0xFFFF, 0xFC00, 0x0000, 0x801F});
        w.push_back(0x0000);
        e.exec(w, CX, CY);
        CHECK(e.vram.get(0,0) == argb1555_to_pen(0xFFFF), "upload white");
        CHECK(e.vram.get(1,0) == argb1555_to_pen(0xFC00), "upload red");
        CHECK(e.vram.get(0,1) == 0u,                      "upload transparent black");
        CHECK((e.vram.get(1,1) & PEN_T) != 0,             "upload blue opaque bit");
        CHECK(((e.vram.get(1,1) >> 3) & 0x1f) == 0x1f,    "upload blue B=31");
    }

    // --- 2. plain copy (no blend/tint/trans): dst == src ---
    {
        Engine e;
        std::vector<uint16_t> w;
        put_upload(w, 0, 0, 2, 1, {0xFC00, 0x83E0});     // red, green-ish
        put_draw(w, 0x1000, 0x0000, /*src*/0,0, /*dst*/100,50, 2,1);   // attr=DRAW, no flags
        w.push_back(0x0000);
        e.exec(w, CX, CY);
        CHECK(e.vram.get(100,50) == argb1555_to_pen(0xFC00), "copy px0");
        CHECK(e.vram.get(101,50) == argb1555_to_pen(0x83E0), "copy px1");
    }

    // --- 3. transparency: A=0 source pixel leaves dst untouched ---
    {
        Engine e;
        std::vector<uint16_t> w;
        // dst pre-filled via an upload of magenta at (10,10)
        put_upload(w, 10, 10, 1, 1, {0xFC1F});
        // src tile: one transparent pixel
        put_upload(w, 0, 0, 1, 1, {0x0000});
        put_draw(w, 0x1100, 0x0000, 0,0, 10,10, 1,1);    // attr bit8 = trans
        w.push_back(0x0000);
        e.exec(w, CX, CY);
        CHECK(e.vram.get(10,10) == argb1555_to_pen(0xFC1F), "trans keeps dst");
    }

    // --- 4. tint: half-intensity red (tint_r=0x40 -> 6-bit 0x10) halves R ---
    {
        Engine e;
        std::vector<uint16_t> w;
        put_upload(w, 0, 0, 1, 1, {0xFC00});             // opaque full red (R=31)
        // tinted, non-blend: attr ti path is chosen by tint != 0x20; keep blend off
        put_draw(w, 0x1000, 0x0000, 0,0, 5,5, 1,1, /*tint_r*/0x0040, /*tint_gb*/0x8080);
        w.push_back(0x0000);
        e.exec(w, CX, CY);
        // R = colrtable[31][0x10] = min(31*16/31,31)=16 ; expect pen R field = 16
        const uint32_t p = e.vram.get(5,5);
        CHECK(((p >> 19) & 0x1f) == 16, "tint half red");
        CHECK((p & PEN_T) != 0, "tint keeps opaque bit");
    }

    // --- 5. blend->copy collapse: smode0/dmode4 with s_alpha=d_alpha=0x1f
    //        (MAME fast path) becomes a straight src copy ---
    {
        Engine e;
        std::vector<uint16_t> w;
        put_upload(w, 0,0, 1,1, {0xFC00});               // src = red 31
        put_upload(w, 5,5, 1,1, {0x8010});               // dst = blue 16 (opaque)
        // attr: DRAW|blend(bit9)|trans(bit8), smode0 dmode4 ; alpha s=d=0xff -> 5b 0x1f
        put_draw(w, uint16_t(0x1000 | (1<<9) | (1<<8) | (0<<4) | 4), 0xFFFF, 0,0, 5,5, 1,1);
        w.push_back(0x0000);
        e.exec(w, CX, CY);
        CHECK(e.vram.get(5,5) == argb1555_to_pen(0xFC00), "s0d4 full-alpha collapses to copy");
    }

    // --- 6. blend smode0/dmode0 real add (alphas not both 0x1f) ---
    {
        Engine e;
        std::vector<uint16_t> w;
        put_upload(w, 0,0, 1,1, {0xFC00});               // src red 31
        put_upload(w, 5,5, 1,1, {0x8010});               // dst blue 16
        // s_alpha=0x80(->0x10), d_alpha=0x80(->0x10); smode0 dmode0
        put_draw(w, uint16_t(0x1000 | (1<<9) | (1<<8)), 0x8080, 0,0, 5,5, 1,1);
        w.push_back(0x0000);
        e.exec(w, CX, CY);
        const auto &T = tables();
        // R: add[ mul[0x10][31] ][ mul[0x10][0] ] = add[16][0] = 16
        // B: add[ mul[0x10][0]  ][ mul[0x10][16]] = add[0][8]  = 8
        const uint32_t p = e.vram.get(5,5);
        CHECK(((p >> 19) & 0x1f) == T.add[T.mul[0x10][31]][T.mul[0x10][0]],  "blend R");
        CHECK(((p >>  3) & 0x1f) == T.add[T.mul[0x10][0]][T.mul[0x10][16]],  "blend B");
    }

    // --- 7. flipx: 3-px row reversed ---
    {
        Engine e;
        std::vector<uint16_t> w;
        put_upload(w, 0,0, 3,1, {0xFC00, 0x83E0, 0x801F});  // red, green, blue
        put_draw(w, uint16_t(0x1000 | (1<<11)), 0x0000, 0,0, 200,100, 3,1); // flipx bit11
        w.push_back(0x0000);
        e.exec(w, CX, CY);
        CHECK(e.vram.get(200,100) == argb1555_to_pen(0x801F), "flipx px0=blue");
        CHECK(e.vram.get(201,100) == argb1555_to_pen(0x83E0), "flipx px1=green");
        CHECK(e.vram.get(202,100) == argb1555_to_pen(0xFC00), "flipx px2=red");
    }

    std::printf(g_fail ? "[selftest] %d FAILURES\n" : "[selftest] all vectors pass\n", g_fail);
    return g_fail ? 1 : 0;
}

// ------------------------------ trace replay ------------------------------
static int replay(const char *path, long max_execs, const std::string &outdir,
                  bool full, int cw, int ch, int scale)
{
    blit::TraceReader tr(path);
    Engine e;
    blit::ExecRecord rec;
    long n = 0;
    int last_clip_x = 0, last_clip_y = 0, last_scroll_x = 0, last_scroll_y = 0;

    while (tr.next(rec)) {
        e.exec(rec.words, rec.clip_x, rec.clip_y);
        last_clip_x = rec.clip_x; last_clip_y = rec.clip_y;
        last_scroll_x = rec.scroll_x; last_scroll_y = rec.scroll_y;
        if (++n == max_execs) break;
    }
    std::printf("[replay] %s: %ld execs, last clip=(%d,%d) scroll=(%d,%d), vram_hash=%016llx\n",
                path, n, last_clip_x, last_clip_y, last_scroll_x, last_scroll_y,
                (unsigned long long)e.vram.hash());

    const std::string base = outdir + "/frame";
    uint64_t hs = dump_window(base + "_scroll.png", e.vram, last_scroll_x, last_scroll_y, cw, ch);
    uint64_t hc = dump_window(base + "_clip.png",   e.vram, last_clip_x,   last_clip_y,   cw, ch);
    std::printf("[replay] wrote %s_scroll.png (rgb_hash=%016llx) and _clip.png (%016llx)\n",
                base.c_str(), (unsigned long long)hs, (unsigned long long)hc);
    if (full) {
        std::printf("[replay] dumping full 8192x4096 VRAM at 1/%d...\n", scale);
        dump_vram_png(base + "_full.png", e.vram, 0, 0, VRAM_W, VRAM_H, scale);
    }
    return 0;
}

// Replay the board-sim text dump (tb_cv1k +blitdump): blocks of
//   EXEC frame=.. addr=.. clip=X,Y scroll=X,Y
//   <hex word> ...
// This renders OUR OWN HS3+blit_regs output through the golden model (H1b).
// With --raw <file> (H3): additionally diff the tb_cv1k +blitvram dump (raw
// little-endian ARGB1555, 8192x4096) pixel-for-pixel against the replay.
// With --frame <file> (H5): diff the tb_cv1k +blitframe scanout capture
// (raw little-endian ARGB1555, 320x240) against the visible window at the
// replay's last scroll, wrapping mod 8192x4096 (MAME copyscrollbitmap
// semantics); also writes the capture as board_frame.png for eyeballing.
static int replay_board(const char *path, const std::string &outdir, bool full,
                        int cw, int ch, int scale, const char *rawpath,
                        const char *framepath, int fsx = INT_MIN, int fsy = 0)
{
    std::FILE *fp = std::fopen(path, "r");
    if (!fp) { std::printf("cannot open %s\n", path); return 2; }
    Engine e;
    std::vector<uint16_t> words;
    int clip_x = 0, clip_y = 0, scroll_x = 0, scroll_y = 0;
    int last_clip_x = 0, last_clip_y = 0, last_scroll_x = 0, last_scroll_y = 0;
    long n = 0;
    bool have = false;
    char line[256];

    auto flush = [&]() {
        if (!have) return;
        e.exec(words, clip_x, clip_y);
        last_clip_x = clip_x; last_clip_y = clip_y;
        last_scroll_x = scroll_x; last_scroll_y = scroll_y;
        n++;
        words.clear();
    };

    while (std::fgets(line, sizeof line, fp)) {
        if (line[0] == '#') continue;
        if (!std::strncmp(line, "EXEC", 4)) {
            flush();
            std::sscanf(line, "EXEC frame=%*d addr=%*x clip=%d,%d scroll=%d,%d",
                        &clip_x, &clip_y, &scroll_x, &scroll_y);
            have = true;
        } else {
            unsigned v;
            if (std::sscanf(line, "%x", &v) == 1) words.push_back(uint16_t(v));
        }
    }
    flush();
    std::fclose(fp);

    std::printf("[board] %s: %ld execs, last clip=(%d,%d) scroll=(%d,%d), vram_hash=%016llx\n",
                path, n, last_clip_x, last_clip_y, last_scroll_x, last_scroll_y,
                (unsigned long long)e.vram.hash());
    const std::string base = outdir + "/board";
    uint64_t hs = dump_window(base + "_scroll.png", e.vram, last_scroll_x, last_scroll_y, cw, ch);
    uint64_t hc = dump_window(base + "_clip.png",   e.vram, last_clip_x,   last_clip_y,   cw, ch);
    std::printf("[board] wrote %s_scroll.png (%016llx) and _clip.png (%016llx)\n",
                base.c_str(), (unsigned long long)hs, (unsigned long long)hc);
    if (full) dump_vram_png(base + "_full.png", e.vram, 0, 0, VRAM_W, VRAM_H, scale);

    if (rawpath) {                       // H3: diff the RTL VRAM dump
        std::FILE *rf = std::fopen(rawpath, "rb");
        if (!rf) { std::printf("[board] cannot open raw dump %s\n", rawpath); return 2; }
        std::vector<uint16_t> raw(VRAM::SIZE);
        if (std::fread(raw.data(), 2, VRAM::SIZE, rf) != VRAM::SIZE) {
            std::printf("[board] raw dump %s truncated\n", rawpath);
            std::fclose(rf);
            return 2;
        }
        std::fclose(rf);
        long bad = 0;
        for (size_t i = 0; i < VRAM::SIZE; i++) {
            const uint16_t want = pen_to_argb1555(e.vram.get(int(i % VRAM_W), int(i / VRAM_W)));
            if (raw[i] != want) {
                if (bad < 8)
                    std::printf("[board] RAW MISMATCH at (%zu,%zu): rtl=%04x gold=%04x\n",
                                i % VRAM_W, i / VRAM_W, raw[i], want);
                bad++;
            }
        }
        std::printf("[board] H3 RTL-vs-golden VRAM diff: %s (%ld bad pixels)\n",
                    bad ? "FAIL" : "PIXEL-EXACT", bad);
        if (bad) return 1;
    }

    if (framepath) {                     // H5: diff the RTL scanout capture
        constexpr int FW = 320, FH = 240;
        // crop origin: the last exec's scroll unless the caller passed the
        // scan-time scroll (--framescroll; the game may flip scroll between
        // its last EXEC and the captured frame - tb_cv1k prints it)
        if (fsx != INT_MIN) { last_scroll_x = fsx; last_scroll_y = fsy; }
        std::FILE *ff = std::fopen(framepath, "rb");
        if (!ff) { std::printf("[board] cannot open frame capture %s\n", framepath); return 2; }
        std::vector<uint16_t> frm(size_t(FW) * FH);
        if (std::fread(frm.data(), 2, frm.size(), ff) != frm.size()) {
            std::printf("[board] frame capture %s truncated\n", framepath);
            std::fclose(ff);
            return 2;
        }
        std::fclose(ff);
        long bad = 0;
        for (int y = 0; y < FH; y++)
            for (int x = 0; x < FW; x++) {
                const int vx = (last_scroll_x + x) & (VRAM_W - 1);
                const int vy = (last_scroll_y + y) & (VRAM_H - 1);
                const uint16_t want = pen_to_argb1555(e.vram.get(vx, vy));
                if (frm[size_t(y) * FW + x] != want) {
                    if (bad < 8)
                        std::printf("[board] FRAME MISMATCH at (%d,%d): rtl=%04x gold=%04x\n",
                                    x, y, frm[size_t(y) * FW + x], want);
                    bad++;
                }
            }
        {   // capture as PNG (RTL side, for eyeballing)
            std::vector<uint8_t> rgb(size_t(FW) * FH * 3);
            for (size_t i = 0; i < frm.size(); i++) {
                const uint16_t p = frm[i];
                const uint8_t r5 = (p >> 10) & 0x1f, g5 = (p >> 5) & 0x1f, b5 = p & 0x1f;
                rgb[i * 3 + 0] = uint8_t((r5 << 3) | (r5 >> 2));
                rgb[i * 3 + 1] = uint8_t((g5 << 3) | (g5 >> 2));
                rgb[i * 3 + 2] = uint8_t((b5 << 3) | (b5 >> 2));
            }
            write_png_rgb(outdir + "/board_frame.png", FW, FH, rgb.data());
        }
        std::printf("[board] H5 scanout-vs-golden frame diff: %s (%ld bad pixels), wrote %s/board_frame.png\n",
                    bad ? "FAIL" : "PIXEL-EXACT", bad, outdir.c_str());
        return bad ? 1 : 0;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && std::strcmp(argv[1], "--selftest") == 0)
        return selftest();

    const char *trace = nullptr;
    const char *board = nullptr;
    const char *raw   = nullptr;
    const char *frame = nullptr;
    int fsx = INT_MIN, fsy = 0;
    std::string outdir = ".";
    long execs = -1;
    bool full = false;
    int cw = 384, ch = 304, scale = 8;
    for (int i = 1; i < argc; i++) {
        if (!std::strcmp(argv[i], "--trace") && i + 1 < argc) trace = argv[++i];
        else if (!std::strcmp(argv[i], "--boardtrace") && i + 1 < argc) board = argv[++i];
        else if (!std::strcmp(argv[i], "--raw") && i + 1 < argc) raw = argv[++i];
        else if (!std::strcmp(argv[i], "--frame") && i + 1 < argc) frame = argv[++i];
        else if (!std::strcmp(argv[i], "--framescroll") && i + 1 < argc)
            std::sscanf(argv[++i], "%d,%d", &fsx, &fsy);
        else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) outdir = argv[++i];
        else if (!std::strcmp(argv[i], "--execs") && i + 1 < argc) execs = std::atol(argv[++i]);
        else if (!std::strcmp(argv[i], "--full")) full = true;
        else if (!std::strcmp(argv[i], "--cw") && i + 1 < argc) cw = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--ch") && i + 1 < argc) ch = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--scale") && i + 1 < argc) scale = std::atoi(argv[++i]);
    }
    if (board)
        return replay_board(board, outdir, full, cw, ch, scale, raw, frame, fsx, fsy);
    if (!trace) {
        std::printf("usage: %s --selftest\n"
                    "       %s --trace f.blit [--execs N] [--out DIR] [--full] [--cw W --ch H]\n"
                    "       %s --boardtrace f.txt [--raw vram.bin] [--frame frame.bin] [--out DIR] [--full] [--cw W --ch H]\n",
                    argv[0], argv[0], argv[0]);
        return 2;
    }
    return replay(trace, execs, outdir, full, cw, ch, scale);
}
