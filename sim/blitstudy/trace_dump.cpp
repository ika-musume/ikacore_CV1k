// trace_dump — validate and summarize a .blit workload trace.
// Sanity gates (hard-fail): header magic, record magics, clean op walk,
// monotonic time. Prints per-record and whole-trace statistics.
#include "blit_trace.h"

#include <cinttypes>
#include <cstring>

int main(int argc, char **argv)
{
    const bool verbose = (argc == 3 && std::strcmp(argv[1], "-v") == 0);
    const char *path = verbose ? argv[2] : (argc == 2 ? argv[1] : nullptr);
    if (!path) {
        std::fprintf(stderr, "usage: trace_dump [-v] <trace.blit>\n");
        return 2;
    }

    try {
        blit::TraceReader rd(path);
        blit::ExecRecord rec;
        uint64_t n_rec = 0, n_draw = 0, n_upload = 0, n_clip = 0;
        uint64_t total_words = 0, max_words = 0, max_draws_one_exec = 0;
        uint64_t prev_t = 0, max_delay = 0;
        uint64_t max_delay_frame = 0;

        while (rd.next(rec)) {
            if (rec.t_ns < prev_t)
                throw std::runtime_error("time went backwards");
            prev_t = rec.t_ns;

            const auto ops = blit::walk(rec.words);
            uint64_t draws = 0, uploads = 0, clips = 0, px = 0;
            for (const auto &op : ops) {
                switch (op.kind) {
                case blit::OpKind::Draw: {
                    draws++;
                    blit::DrawView d(&rec.words[op.off]);
                    px += uint64_t(d.dimx()) * d.dimy();
                    break;
                }
                case blit::OpKind::Upload: uploads++; break;
                case blit::OpKind::Clip:   clips++;   break;
                case blit::OpKind::End:    break;
                }
            }
            n_rec++; n_draw += draws; n_upload += uploads; n_clip += clips;
            total_words += rec.words.size();
            if (rec.words.size() > max_words) max_words = rec.words.size();
            if (draws > max_draws_one_exec) max_draws_one_exec = draws;
            if (rec.mame_delay_ns > max_delay) {
                max_delay = rec.mame_delay_ns;
                max_delay_frame = rec.frame;
            }

            if (verbose)
                std::printf("f=%6" PRIu64 " t=%12" PRIu64 "ns addr=%08x "
                            "draws=%4" PRIu64 " up=%2" PRIu64 " clip=%2" PRIu64
                            " px=%8" PRIu64 " delay=%9" PRIu64 "ns scr=(%u,%u)\n",
                            rec.frame, rec.t_ns, rec.gfx_addr, draws, uploads,
                            clips, px, rec.mame_delay_ns,
                            rec.scroll_x, rec.scroll_y);
        }

        std::printf("OK  records=%" PRIu64 "  draws=%" PRIu64
                    "  uploads=%" PRIu64 "  clips=%" PRIu64 "\n",
                    n_rec, n_draw, n_upload, n_clip);
        std::printf("    words total=%" PRIu64 "  max/exec=%" PRIu64
                    "  max draws/exec=%" PRIu64 "\n",
                    total_words, max_words, max_draws_one_exec);
        std::printf("    worst MAME delay=%" PRIu64 " ns (frame %" PRIu64
                    ", frame budget ~16683333 ns)\n",
                    max_delay, max_delay_frame);
        return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
