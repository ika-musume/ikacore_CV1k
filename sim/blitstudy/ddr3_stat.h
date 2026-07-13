// Statistical model of the MiSTer HPS-DDR3 port @153.6 MHz, calibrated
// from benchmarks/MiSTerDDR3Test-CV1k (M-DDR3-0..4, measured 2026-07-11):
//   read latency under HPS load: 93.4% in 17-24 clk, ~4.0% in 25-64,
//   2.56% in 65-96, 0.0035% in 97-165 (max 165 = 1.07us)
//   beta_R 1.016 clk/word, beta_W 1.017, T_TURN ~14 clk, G_CMD/C_W ~1
// Port is PIPE (M-DDR3-1): command FIFO overlaps burst latencies; a train
// of reads pays one exposed L_R. Scanout line fetch preempts every hline.
#pragma once

#include <cstdint>
#include <random>

namespace ddr3 {

constexpr double CLK_NS  = 6.5104;   // 153.6 MHz port clock
constexpr double BETA_R  = 1.016;    // clk/word within a read burst
constexpr double BETA_W  = 1.017;    // clk/word within a write burst
constexpr int    G_CMD   = 1;        // per read-burst cmd slot in a train
constexpr int    C_W     = 1;        // per write-burst overhead
constexpr int    T_TURN  = 14;       // R<->W direction switch
constexpr int    BL_MAX  = 128;      // words per burst

// scanout line fetch: 320 px = 80 x 64-bit words, every hline period
constexpr double HLINE_PERIOD_NS = 63586.0;
constexpr int    SCANOUT_WORDS   = 80;

class LatencySampler {
public:
    explicit LatencySampler(uint64_t seed) : m_rng(seed) {}

    // one read-latency sample (port clk), HPS-load histogram
    int sample()
    {
        const double u = m_uni(m_rng);
        if (u < 0.934)     return uniform(17, 24);
        if (u < 0.974)     return uniform(25, 64);
        if (u < 0.9996)    return uniform(65, 96);
        return uniform(97, 165);
    }

private:
    int uniform(int lo, int hi)
    {
        return lo + int(m_uni(m_rng) * (hi - lo + 1));
    }
    std::mt19937_64 m_rng;
    std::uniform_real_distribution<double> m_uni{0.0, 1.0};
};

// Single-owner port timeline. Threads reserve occupancy through this; it
// serializes bursts (one physical port), inserts turnarounds, applies the
// sampled first-of-train latency, and steals scanout slots each hline.
class Port {
public:
    explicit Port(uint64_t seed) : m_lat(seed) {}

    // Occupy the port for a read train totalling `words` in `nbursts`
    // bursts starting no earlier than t_ns; returns completion time (ns).
    double read(double t_ns, int64_t words, int nbursts)
    {
        double t = begin(t_ns, /*is_read=*/true);
        const double clk = m_lat.sample() + words * BETA_R + nbursts * G_CMD;
        t += clk * CLK_NS;
        return end(t);
    }

    double write(double t_ns, int64_t words, int nbursts)
    {
        double t = begin(t_ns, /*is_read=*/false);
        const double clk = words * BETA_W + nbursts * C_W;
        t += clk * CLK_NS;
        return end(t);
    }

    double free_at() const { return m_free; }

private:
    double begin(double t_ns, bool is_read)
    {
        double t = t_ns > m_free ? t_ns : m_free;
        t = scanout_defer(t);
        if (m_have_dir && is_read != m_last_read)
            t += T_TURN * CLK_NS;
        m_last_read = is_read;
        m_have_dir = true;
        return t;
    }

    double end(double t)
    {
        m_free = t;
        return t;
    }

    // scanout preemption: if a request would start inside a line-fetch
    // window, it waits until the window ends
    double scanout_defer(double t)
    {
        const double win = (m_lat_scanout + SCANOUT_WORDS * BETA_R) * CLK_NS;
        const double k = double(long(t / HLINE_PERIOD_NS));
        const double w0 = k * HLINE_PERIOD_NS;
        if (t < w0 + win) return w0 + win;
        return t;
    }

    LatencySampler m_lat;
    double m_free = 0.0;
    bool m_have_dir = false, m_last_read = false;
    static constexpr int m_lat_scanout = 21; // avg latency for the line fetch
};

// word-count helpers for the linear VRAM layout (addr = (Y*8192+X)*2)
inline int64_t words_linear(int x, int w)
{
    const int64_t b0 = (int64_t(x) * 2) / 8;
    const int64_t b1 = ((int64_t(x) + w) * 2 - 1) / 8;
    return b1 - b0 + 1;
}

inline int bursts_of(int64_t words)
{
    return int((words + BL_MAX - 1) / BL_MAX);
}

} // namespace ddr3
