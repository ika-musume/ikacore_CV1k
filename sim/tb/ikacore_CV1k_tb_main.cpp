// ikacore CV1k - H7b.3 MiSTer top-level testbench harness.
//
// Drives ikacore_CV1k_tb (the portable core top in its final MISTER_SDRAM +
// CV1K_NAND configuration + the 128 MB module SDRAM chip model) with BOTH
// clocks generated on the exact dual-clock grid, and serves the MiSTer
// DDRAM face with the region-mapped, ddr3_stat.h-calibrated slave:
//
//   grid unit U = 1/614.4 MHz: 153.6 MHz toggles every 2 U, 102.4 every
//   3 U, coincident rising edges every 12 U (= the 51.2 MHz CKIO grid),
//   EXTAL2 toggles every 9375 U (exact 32768 Hz).  Sim time runs in
//   tb_cv1k's scaled units (102.4 clock = 10 ns), rendered on the ps grid
//   (U = 5/3 ns; the +-0.33 ps mid-grid rounding is sampled by nothing).
//
//   DDR3 byte map (the H7b plan of record):
//     VRAM  64 MB @ word 0x0600_0000  - RAM-backed (+ the frame source)
//     NAND 132 MB @ word 0x0680_0000  - file-backed  roms/ibara/u2
//     YMZ   16 MB @ word 0x07A0_0000  - file-backed  u23 @+0, u24 @+8MB
//   Writes: VRAM -> RAM; elsewhere -> byte-merged overlay map (the H7b.4
//   ioctl loader path).  Reads outside the map are fatal.
//
//   Read/write timing = tb_h7_main's DdrSlave verbatim (latency histogram
//   under HPS load, beta_R/beta_W, T_TURN, G_CMD; --seed 0 = perfect
//   port).  The slave's latency bookkeeping stays in REAL ns (ticks of
//   the 153.6 clock x 6.5104), so the calibration is sim-timescale-free.
//
//   ./Vikacore_CV1k_tb [--seed S] [--vram f.bin] [--frame f.bin]
//                      [+maxinsn=N +trace=f +blitdump=f +norhex=f ...]
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <verilated.h>
#include "Vikacore_CV1k_tb.h"

#include "blitstudy/ddr3_stat.h"

static constexpr double TICK_NS = 1000.0 / 153.6;      // one 153.6 clock, real ns

// ---------------------------------------------------------------------------
// region-mapped DDRAM slave (timing = tb_h7_main's DdrSlave)
// ---------------------------------------------------------------------------
struct DdrSlave {
    static constexpr uint32_t VRAM_W0   = 0x06000000u;
    static constexpr uint32_t VRAM_WSZ  = 1u << 23;               // 64 MB
    static constexpr uint32_t NAND_W0   = 0x06800000u;
    static constexpr uint32_t NAND_WSZ  = 0x01080000u;            // 132 MB
    static constexpr uint32_t YMZ_W0    = 0x07A00000u;
    static constexpr uint32_t YMZ_WSZ   = 0x00200000u;            // 16 MB
    static constexpr uint32_t YMZ_CHIP_W = 0x00100000u;           // 8 MB slots

    std::vector<uint16_t> vram;                                   // 32M px
    std::unordered_map<uint32_t, uint64_t> ovl;                   // non-VRAM writes
    FILE *f_nand = nullptr, *f_u23 = nullptr, *f_u24 = nullptr;
    ddr3::LatencySampler lat;
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
    double burst_gate = 0;   // >= 1 idle port cycle between read bursts: the
                             // harness reloads its return-queue head in that
                             // bubble (same contract ddr3_beh's GAP provides;
                             // H7b.8 must confirm f2sdram or add a skid)
    uint64_t n_rd_words = 0, n_wr_words = 0, n_bursts = 0, n_nand_bursts = 0;

    explicit DdrSlave(uint32_t s)
        : vram(size_t(1) << 25, 0), lat(s ? s : 1), seed(s)
    {
        f_nand = std::fopen("roms/ibara/u2", "rb");
        if (!f_nand) { std::printf("FATAL: cannot open roms/ibara/u2\n"); std::exit(1); }
        f_u23 = std::fopen("roms/ibara/u23", "rb");               // optional (H7b.5)
        f_u24 = std::fopen("roms/ibara/u24", "rb");
    }
    ~DdrSlave() {
        if (f_nand) std::fclose(f_nand);
        if (f_u23)  std::fclose(f_u23);
        if (f_u24)  std::fclose(f_u24);
    }

    double p_turn() const { return seed ? ddr3::T_TURN * ddr3::CLK_NS : 0; }
    double p_br()   const { return seed ? ddr3::BETA_R * ddr3::CLK_NS : TICK_NS; }
    double p_bw()   const { return seed ? (ddr3::BETA_W + ddr3::C_W) * ddr3::CLK_NS
                                        : TICK_NS; }
    double p_gcmd() const { return seed ? ddr3::G_CMD * ddr3::CLK_NS : TICK_NS; }
    double p_l()          { return seed ? lat.sample() * ddr3::CLK_NS
                                        : 2.0 * TICK_NS; }

    static uint64_t file_word(FILE *f, long off8) {
        uint8_t b[8] = {0};
        if (f && std::fseek(f, off8, SEEK_SET) == 0)
            (void)!std::fread(b, 1, 8, f);                        // short read -> zero fill
        uint64_t w = 0;
        for (int k = 0; k < 8; k++) w |= uint64_t(b[k]) << (8 * k);
        return w;
    }

    uint64_t rd_word(uint32_t w) {
        if (w >= VRAM_W0 && w < VRAM_W0 + VRAM_WSZ) {
            const size_t p = size_t(w - VRAM_W0) * 4;
            return (uint64_t(vram[p + 3]) << 48) | (uint64_t(vram[p + 2]) << 32)
                 | (uint64_t(vram[p + 1]) << 16) |  uint64_t(vram[p]);
        }
        auto it = ovl.find(w);
        if (it != ovl.end()) return it->second;
        if (w >= NAND_W0 && w < NAND_W0 + NAND_WSZ)
            return file_word(f_nand, long(w - NAND_W0) * 8);
        if (w >= YMZ_W0 && w < YMZ_W0 + YMZ_WSZ) {
            const uint32_t o = w - YMZ_W0;
            return (o < YMZ_CHIP_W) ? file_word(f_u23, long(o) * 8)
                                    : file_word(f_u24, long(o - YMZ_CHIP_W) * 8);
        }
        std::printf("FATAL: DDRAM read outside region map: word %08x\n", w);
        std::exit(1);
    }

    void wr_word(uint32_t w, uint64_t d, uint8_t be) {
        if (w >= VRAM_W0 && w < VRAM_W0 + VRAM_WSZ) {
            const size_t p = size_t(w - VRAM_W0) * 4;
            for (int l = 0; l < 4; l++)
                if (be & (3u << (2 * l)))
                    vram[p + size_t(l)] = uint16_t(d >> (16 * l));
            return;
        }
        uint64_t cur = rd_word(w);                                 // file/zero background
        for (int k = 0; k < 8; k++)
            if (be & (1u << k)) {
                cur &= ~(uint64_t(0xFF) << (8 * k));
                cur |=  ((d >> (8 * k)) & 0xFF) << (8 * k);
            }
        ovl[w] = cur;
    }

    bool busy() const { return rq.size() >= 8 || wr_busy_until > 0; }

    // BEFORE the falling-edge eval: present BUSY/DOUT for the next cycle
    void drive(double now, Vikacore_CV1k_tb &tb) {
        if (wr_busy_until > 0 && now >= wr_busy_until) wr_busy_until = 0;
        tb.DDRAM_BUSY = busy();
        tb.DDRAM_DOUT_READY = 0;
        if (!rq.empty()) {
            Burst &b = rq.front();
            if (b.k < b.data.size() && now >= b.next_ns && now >= burst_gate) {
                tb.DDRAM_DOUT_READY = 1;
                tb.DDRAM_DOUT = b.data[b.k];
                b.k++;
                b.next_ns += p_br();
                n_rd_words++;
                if (b.k == b.data.size()) {
                    rq.pop_front();
                    burst_gate = now + 1.5 * TICK_NS;   // 1 dead cycle
                }
            }
        }
    }

    // AFTER the falling-edge eval: pins hold this cycle's command
    void accept(double now, Vikacore_CV1k_tb &tb) {
        const bool was_busy = tb.DDRAM_BUSY;
        if (tb.DDRAM_RD && !was_busy) {
            const uint32_t w0 = tb.DDRAM_ADDR;
            Burst b;
            b.data.reserve(tb.DDRAM_BURSTCNT);
            for (int k = 0; k < int(tb.DDRAM_BURSTCNT); k++)
                b.data.push_back(rd_word(w0 + uint32_t(k)));
            double t = std::max(now, free_ns);
            if (rq.empty()) {                    // fresh train
                if (have_dir && !dir_read) t += p_turn();
                t += p_l();
            }
            else t = last_end_ns + p_gcmd();     // pipelined behind prev burst
            b.next_ns = t + p_br();
            last_end_ns = t + double(b.data.size()) * p_br();
            free_ns = last_end_ns;
            have_dir = true; dir_read = true;
            n_bursts++;
            if (w0 >= NAND_W0 && w0 < NAND_W0 + NAND_WSZ) n_nand_bursts++;
            rq.push_back(std::move(b));
        }
        if (tb.DDRAM_WE && !was_busy) {
            wr_word(tb.DDRAM_ADDR, tb.DDRAM_DIN, uint8_t(tb.DDRAM_BE));
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
int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    const std::unique_ptr<VerilatedContext> ctx{new VerilatedContext};
    ctx->commandArgs(argc, argv);

    uint32_t seed = 0;
    const char *vram_f = nullptr, *frame_f = nullptr;
    for (int i = 1; i < argc; i++) {
        if      (!std::strcmp(argv[i], "--seed")  && i + 1 < argc) seed = uint32_t(std::strtoul(argv[++i], nullptr, 0));
        else if (!std::strcmp(argv[i], "--vram")  && i + 1 < argc) vram_f = argv[++i];
        else if (!std::strcmp(argv[i], "--frame") && i + 1 < argc) frame_f = argv[++i];
    }

    Vikacore_CV1k_tb tb{ctx.get()};
    DdrSlave ddr(seed);

    tb.i_CLK102 = 0; tb.i_CLK153 = 0; tb.i_EXTAL2 = 0;
    tb.i_INITRST_n = 0; tb.i_SOFTRST_n = 0;
    tb.DDRAM_BUSY = 0; tb.DDRAM_DOUT = 0; tb.DDRAM_DOUT_READY = 0;

    // reset schedule in grid units (odd -> between edges; SOFTRST parity
    // chosen so CKIO's first rise lands on a coincident edge - the DUT's
    // pcen23 checker enforces it)
    static constexpr uint64_t U_INITRST = 1201;        // ~2 us scaled
    static constexpr uint64_t U_SOFTRST = 123001;      // ~205 us scaled

    // ps offset of grid unit u within its 20,000 ps block (12 U); -1 = no edge
    static const int32_t OFFS[12] = {0, -1, 3333, 5000, 6667, -1,
                                     10000, -1, 13333, 15000, 16667, -1};

    // frame capture (vsync-to-vsync with all 76,800 px_de strobes)
    std::vector<uint16_t> fr_cur(320 * 240, 0), fr_last(320 * 240, 0);
    long fr_idx = 0, fr_frames = 0;
    bool prev_vsync = false;

    uint64_t u = 0, ticks153 = 0;
    uint64_t last_report = 0;
    while (!ctx->gotFinish()) {
        // reset releases land on no-edge units (between all clock edges);
        // the pins settle here and the next edge eval samples them
        if (u == U_INITRST) tb.i_INITRST_n = 1;
        if (u == U_SOFTRST) {
            tb.i_SOFTRST_n = 1;
            std::printf("[cv1k_tb] reset released @ u=%llu (~%.1f us)\n",
                        (unsigned long long)u, double(u) * 5.0 / 3.0 / 1e3);
        }

        const int32_t off = OFFS[u % 12];
        const bool ext_ev = (u % 9375) == 0;
        if (off < 0 && !ext_ev) { u++; continue; }

        // absolute ps time of this grid unit (block base + offset)
        const uint64_t t_ps = (u / 12) * 20000ull
                            + uint64_t(off >= 0 ? off : int32_t((u % 12) * 5000 / 3));

        // let any model-internal timed events (mt48 checker branches) fire
        while (tb.eventsPending() && tb.nextTimeSlot() < t_ps) {
            ctx->time(tb.nextTimeSlot());
            tb.eval();
        }
        ctx->time(t_ps);

        const bool t153 = (u % 2) == 0;
        const bool t102 = (u % 3) == 0;
        bool rise153 = false, fall153 = false;
        if (t153) {
            tb.i_CLK153 ^= 1;
            rise153 = tb.i_CLK153;
            fall153 = !tb.i_CLK153;
        }
        if (t102) tb.i_CLK102 ^= 1;
        if (ext_ev && u) tb.i_EXTAL2 ^= 1;

        const double now = double(ticks153) * TICK_NS;
        if (fall153) ddr.drive(now, tb);
        tb.eval();
        if (fall153) ddr.accept(now, tb);

        if (rise153) {
            ticks153++;
            // frame capture on the post-edge registered outputs
            if (tb.o_VSYNC && !prev_vsync) {
                if (fr_idx == 320 * 240) { fr_last = fr_cur; fr_frames++; }
                fr_idx = 0;
            }
            prev_vsync = tb.o_VSYNC;
            if (tb.o_PX_DE && fr_idx < 320 * 240)
                fr_cur[size_t(fr_idx++)] = tb.o_PX;
        }

        if ((u >> 20) != last_report) {          // liveness marker (~1.75 ms)
            last_report = u >> 20;
            std::printf("[cv1k_tb] t=%.2f ms  ddr rd=%llu wr=%llu bursts=%llu (nand %llu)  frames=%ld\n",
                        double(t_ps) / 1e9,
                        (unsigned long long)ddr.n_rd_words,
                        (unsigned long long)ddr.n_wr_words,
                        (unsigned long long)ddr.n_bursts,
                        (unsigned long long)ddr.n_nand_bursts, fr_frames);
        }
        u++;
    }
    tb.final();

    std::printf("[cv1k_tb] done: %.2f ms sim, ddr rd=%llu wr=%llu bursts=%llu (nand %llu), frames=%ld, seed=%u\n",
                double(ctx->time()) / 1e9,
                (unsigned long long)ddr.n_rd_words,
                (unsigned long long)ddr.n_wr_words,
                (unsigned long long)ddr.n_bursts,
                (unsigned long long)ddr.n_nand_bursts, fr_frames, seed);

    if (frame_f) {
        if (fr_frames == 0)
            std::printf("[cv1k_tb] no complete frame captured\n");
        else if (FILE *f = std::fopen(frame_f, "wb")) {
            for (auto px : fr_last) { std::fputc(px & 0xFF, f); std::fputc(px >> 8, f); }
            std::fclose(f);
            std::printf("[cv1k_tb] wrote %s (frame %ld)\n", frame_f, fr_frames);
        }
    }
    if (vram_f) {
        if (FILE *f = std::fopen(vram_f, "wb")) {
            for (auto px : ddr.vram) { std::fputc(px & 0xFF, f); std::fputc(px >> 8, f); }
            std::fclose(f);
            std::printf("[cv1k_tb] wrote %s (DDR3 VRAM region)\n", vram_f);
        }
    }
    return 0;
}
