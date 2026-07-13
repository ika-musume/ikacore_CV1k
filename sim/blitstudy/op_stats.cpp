// op_stats — sprite-geometry distribution of a .blit trace: the data that
// sizes the DDR3 engine's objline-staging buffer (batch K x objline words).
#include "blit_trace.h"
#include "ddr3_stat.h"

#include <algorithm>
#include <cinttypes>

namespace {

void pct(const char *name, std::vector<uint32_t> &v)
{
    if (v.empty()) return;
    std::sort(v.begin(), v.end());
    auto at = [&](double q) { return v[size_t(q * double(v.size() - 1))]; };
    std::printf("%-22s p50 %6u  p90 %6u  p99 %6u  p99.9 %6u  max %6u\n",
                name, at(0.50), at(0.90), at(0.99), at(0.999), v.back());
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::fprintf(stderr, "usage: op_stats <trace.blit>\n");
        return 2;
    }
    try {
        blit::TraceReader rd(argv[1]);
        blit::ExecRecord rec;
        std::vector<uint32_t> hs, ws, line_bytes, op_bytes;
        uint64_t n_draw = 0;
        while (rd.next(rec)) {
            for (const auto &op : blit::walk(rec.words)) {
                if (op.kind != blit::OpKind::Draw) continue;
                blit::DrawView d(&rec.words[op.off]);
                const uint32_t w = d.dimx(), h = d.dimy();
                const uint32_t rb = uint32_t(
                    (ddr3::words_linear(d.src_x(), int(w)) +
                     ddr3::words_linear(d.dst_x(), int(w))) * 8);
                n_draw++;
                hs.push_back(h);
                ws.push_back(w);
                line_bytes.push_back(rb);          // src+dst staging per objline
                op_bytes.push_back(rb * h);        // whole-op staging
            }
        }
        std::printf("draws: %" PRIu64 "\n", n_draw);
        pct("height (objlines)", hs);
        pct("width (px)", ws);
        pct("staging B/objline", line_bytes);
        pct("staging B/op (K=h)", op_bytes);
        return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
