`default_nettype none
//============================================================================
// blit_gov.sv - CV1000-B blitter timing governor        [H4 / I-2.1+I-2.2+I-2.5]
//
// The timing plane of the two-plane architecture (blitter_ddr3_sched.md §10,
// BD §7.6): generates the CPU-visible timeline -- op start/finish, STATUS
// BUSY, IRQ1 retirement, and the governed fetch window -- as a pure function
// of the op list and the runtime-loadable cost tables.  It NEVER touches the
// execution plane: blit_draw runs at full native speed regardless (user
// directive 2026-07-13: max-speed datapath, tunable cost model).
//
// Model (C++ golden: sim/blitstudy/cost_model.h, anchors 8x8=93 / 16x12=189 /
// 240x64=12090 VCLK; 80x clipped ~17.5us; 256x5 upload ~58.77us):
//
//   op_start    = max(engine_free, fetch_ready)
//   engine_free = op_start + cost(op)  (+166 VCLK per hline boundary crossed)
//   cost(DRAW)  = src_px/4 + 2*(dst_px/4) + spans_src*P_SRC
//               + spans_dst*(P_RW+P_WR) + P_SPR        [BD §6.5, clip-clamped]
//   cost(CLIP / fully-clipped DRAW / UPLOAD) = 0        (fetch-bound)
//   BUSY        = EXEC .. governed finish of the op list (incl. END arrival)
//
// Two halves:
//
//  * ARRIVAL PARSER -- snoops blit_fetch's FIFO-push stream (so fetch_ready =
//    the REAL arrival time of each op's last word over the BREQ/BACK bus;
//    timing = f(op list) only, independent of the draw engine).  Re-frames
//    ops, applies the u16 clip test (mirrors gfx_create/draw_shadow_copy via
//    workload.h -- src origin kept, dims shrunk, u16 wrap semantics
//    bug-for-bug), computes the DRAW cost from the tables, and pushes one
//    {kind, cost, nslot} entry per op into the cost queue at the op's
//    last-word arrival.
//
//  * TIMELINE FSM -- `now` counts half-VCLK ticks (76.8 MHz VCLK = exactly
//    1.5x the 51.2 MHz CKIO, so now += 3 per CKIO edge; no derived clocks).
//    Free-running since reset (H5); per-EXEC times are deltas vs r_t0.
//    Pops an entry only once now >= engine_free, so op_start =
//    max(engine_free, arrival) with only a few-tick non-cumulative error.
//    hline steals are added incrementally against a running boundary
//    register (next_bnd, one P-step per cycle -- no division) as in
//    cost_model.h add_steals, but with the boundary PHASE re-anchored on
//    the real scanline via i_hline (H5) instead of reset at EXEC: both
//    the video counters and this time base count the same CKIO grid, so
//    the re-anchor is exact and idempotent (zero drift); with i_hline
//    tied 0 the phase free-runs from reset (trace TB).  NOTE: table 6
//    (HLINE_P) should match the video module's real line period --
//    default 9768 half-VCLK == blit_video's 3256 CKIO.
//
// Governed fetch window (fifo_study 2026-07-13, drainB semantics): only
// chunks holding SURVIVING-DRAW bytes occupy a virtual original-FIFO slot;
// slots free at governed op START.  o_fetch_hold stalls the next attribute
// chunk while (arrived - started) slot-chunks >= WINDOW (default 512);
// upload-payload chunks are exempt (they stream through, never holding a
// slot) -- blit_fetch bypasses the hold while mid-UPLOAD.
//
// Runtime tables (i_tbl_we/idx/data; defaults = P_PDF set, the one that hits
// all three anchors exactly).  The board rig refines these later (M-5/M-8)
// without touching RTL:
//   0 P_SRC   [11:0]  VCLK per src 32x32 tile span            (default 5)
//   1 P_RW    [11:0]  VCLK per dst tile span, read->write     (default 20)
//   2 P_WR    [11:0]  VCLK per dst tile span, write->read     (default 10)
//   3 P_SPR   [11:0]  VCLK per draw (sprite switch)           (default 10)
//   4 C_SRC4  [7:0]   VCLK per 4 src px                       (default 1)
//   5 C_DST4  [7:0]   VCLK per 4 padded dst px                (default 2)
//   6 HLINE_P [15:0]  hline period in HALF-VCLK               (default 9768
//                     = 4884 VCLK = 63.594us; C++ uses 63.586us -- 0.01%)
//   7 STEAL   [11:0]  hline steal in VCLK                     (default 166)
//   8 STEAL_EN[0]     engine-side hline steal enable          (default 1)
//   9 WINDOW  [15:0]  governed fetch window, chunks           (default 512)
//  10 EXEC2BRQ[7:0]   fetch pacing, CKIO (P-22)               (default 10)
//  11 CHUNK   [7:0]   attribute chunk cadence, CKIO (P-20)    (default 36)
//  12 UPLD    [7:0]   upload chunk cadence, CKIO (P-21)       (default 74)
//
// Reported at retirement: r_busy_end = engine_free at END pop, in half-VCLK
// ticks since EXEC.  This is the C++-comparable busy_end (the fetch-bound
// anchors 17.5us/58.77us) -- o_busy itself deasserts at max(engine_free,
// END arrival), the honest hardware-facing window, which additionally trails
// by the END chunk fetch.
//
// Synthesis notes (H7b.8): the DRAW cost is a staged pipeline keyed to the
// op's own word arrivals -- X-clamp at w7, Y-clamp + coefficient products at
// w8, raw products at w9 (the emit edge), final sum+saturate comb into the
// queue write port one cycle later -- so no cycle carries more than one DSP
// level.  Bit-exact with the old single-cone form: the entry lands in q_mem
// at the same edge with the same value (reassociation is exact -- dpw is
// always a multiple of 4, and the src_px/4 truncation is corrected with a
// mod-4 remainder term; all narrowed widths are proven bounds for surviving
// draws, and rejected draws store 0).  The cost queue infers M10K with the
// read address register absorbed; mixed-port read-during-write is "don't
// care" there, so q_head carries an explicit one-cycle new-data bypass
// (value-identical in RTL sim, defined data on silicon).
//============================================================================
module blit_gov (
    input  wire        i_CLK,
    input  wire        i_CKIO_PCEN,      // pulses the i_CLK cycle CKIO rises
    input  wire        i_RST_n,

    // EXEC kick (same pulse + shadow clips blit_fetch/blit_draw receive)
    input  wire        i_exec,
    input  wire [15:0] i_clip_x,
    input  wire [15:0] i_clip_y,

    // arrival snoop: blit_fetch's attribute-FIFO push stream
    input  wire        i_push,
    input  wire [15:0] i_word,

    // real scanline anchor (H5): one pulse per line at the steal point.
    // Re-anchors the boundary register whenever it isn't pre-accounted
    // (next_bnd <= now), so the timeline's steal cadence = the video
    // module's actual free-running phase instead of resetting at EXEC
    // (I-2.3 second half).  Tie 0 where no video exists (trace TB) --
    // the boundary register then free-runs from reset via catch-up.
    input  wire        i_hline,

    // trace-TB mode: allow `now` to jump forward to engine_free while
    // waiting to pop (arrival pacing is synthetic there; costs stay exact,
    // absolute times don't matter).  Tie 0 in the board sim.
    input  wire        i_warp,

    // cost/pace table load port (runtime-loadable; defaults on reset)
    input  wire        i_tbl_we,
    input  wire [3:0]  i_tbl_idx,
    input  wire [31:0] i_tbl_data,

    // governed fetch pacing -> blit_fetch
    output wire        o_fetch_hold,     // window / cost-queue backpressure
    output wire [7:0]  o_pace_exec2brq,
    output wire [7:0]  o_pace_chunk,
    output wire [7:0]  o_pace_upld,

    // CPU-visible timeline
    output reg         o_busy,           // model BUSY: EXEC .. governed retire
    output reg         o_retire,         // 1-cycle pulse at retirement -> IRQ1

    // per-op debug taps (TB cost scoreboard, I-2.2)
    output reg         o_dbg_vld,        // one pulse per op entry
    output reg  [1:0]  o_dbg_kind,       // 0 zero-cost  1 draw  2 end/fault
    output reg  [26:0] o_dbg_cost        // draw cost in VCLK
);

    //------------------------------------------------------------------
    // runtime tables
    //------------------------------------------------------------------
    reg [11:0] t_p_src, t_p_rw, t_p_wr, t_p_spr;
    reg [12:0] t_rwwr;                   // P_RW + P_WR, kept pre-summed
    reg [7:0]  t_c_src4, t_c_dst4;
    reg [15:0] t_hline_p;                // half-VCLK
    reg [11:0] t_steal;                  // VCLK
    reg        t_steal_en;
    reg [15:0] t_window;                 // chunks
    reg [7:0]  t_exec2brq, t_chunk, t_upld;

    assign o_pace_exec2brq = t_exec2brq;
    assign o_pace_chunk    = t_chunk;
    assign o_pace_upld     = t_upld;

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            t_p_src    <= 12'd5;         // P_PDF set (hits all three anchors)
            t_p_rw     <= 12'd20;
            t_p_wr     <= 12'd10;
            t_rwwr     <= 13'd30;
            t_p_spr    <= 12'd10;
            t_c_src4   <= 8'd1;
            t_c_dst4   <= 8'd2;
            t_hline_p  <= 16'd9768;      // 4884 VCLK
            t_steal    <= 12'd166;
            t_steal_en <= 1'b1;
            t_window   <= 16'd512;       // fifo_study frozen window
            t_exec2brq <= 8'd10;         // P-22
            t_chunk    <= 8'd36;         // P-20 (~700 ns)
            t_upld     <= 8'd74;         // P-21 (~1442.5 ns)
        end
        else if (i_tbl_we) begin
            case (i_tbl_idx)
                4'd0 : t_p_src    <= i_tbl_data[11:0];
                4'd1 : begin
                    t_p_rw <= i_tbl_data[11:0];
                    t_rwwr <= {1'b0, i_tbl_data[11:0]} + {1'b0, t_p_wr};
                end
                4'd2 : begin
                    t_p_wr <= i_tbl_data[11:0];
                    t_rwwr <= {1'b0, t_p_rw} + {1'b0, i_tbl_data[11:0]};
                end
                4'd3 : t_p_spr    <= i_tbl_data[11:0];
                4'd4 : t_c_src4   <= i_tbl_data[7:0];
                4'd5 : t_c_dst4   <= i_tbl_data[7:0];
                4'd6 : t_hline_p  <= i_tbl_data[15:0];
                4'd7 : t_steal    <= i_tbl_data[11:0];
                4'd8 : t_steal_en <= i_tbl_data[0];
                4'd9 : t_window   <= i_tbl_data[15:0];
                4'd10: t_exec2brq <= i_tbl_data[7:0];
                4'd11: t_chunk    <= i_tbl_data[7:0];
                4'd12: t_upld     <= i_tbl_data[7:0];
                default: ;
            endcase
        end
    end

    //------------------------------------------------------------------
    // 32x32 tile-span count along one axis -- mirrors MAME
    // calculate_vram_accesses / cost_model.h vram_tile_spans slice loop
    // (NOT the geometric tile count: each 32-px slice re-tests the same
    // origin offset, so a partial tail can double-count -- bug-for-bug).
    //------------------------------------------------------------------
    function automatic [12:0] f_tspan(input [4:0] off, input [15:0] len);
        reg [10:0] nfull;
        reg [4:0]  part;
        begin
            nfull   = len[15:5];
            part    = len[4:0];
            f_tspan = {2'd0, nfull}
                    + ((part != 5'd0) ? 13'd1 : 13'd0)
                    + ((off  != 5'd0) ? {2'd0, nfull} : 13'd0)
                    + ((part != 5'd0 &&
                        ({1'b0, off} + {1'b0, part}) > 6'd32) ? 13'd1 : 13'd0);
        end
    endfunction

    //------------------------------------------------------------------
    // arrival parser: re-frame the push stream, clip-test + cost each op
    //------------------------------------------------------------------
    typedef enum logic [1:0] { GP_HDR, GP_BODY, GP_HALT } gpstate_e;
    gpstate_e   gp;
    reg  [3:0]  gp_idx;                  // word index within the op (hdr = 0)
    reg  [1:0]  gp_kind;                 // 0 clip  1 draw  2 upload
    reg  [25:0] gp_need;                 // body words left (payload max 2^25)
    reg  [25:0] gp_wcnt;                 // words since EXEC; chunk = [25:5]
    reg  [20:0] gp_c0;                   // current op's first-word chunk

    // captured DRAW fields (raw u16, DrawView slices).  The w/h words are
    // folded into end-coordinate pre-adds at capture (dxe = dx + (w-1),
    // dye = dy + (h-1), u16 wrap) so the clip-clamp stages start from
    // registers -- see the cost pipeline below.
    reg  [15:0] gd_sx, gd_sy, gd_dx, gd_dy;
    reg  [13:0] gu_dimx1;                // UPLOAD (w6 & 0x1fff) + 1

    // clip window state (u16 wrap semantics, workload.h window_clip)
    reg  [15:0] gc_minx, gc_maxx, gc_miny, gc_maxy;
    reg  [15:0] gl_clip_x, gl_clip_y;    // EXEC-latched clip regs (CLIP re-derive)

    // window bookkeeping
    reg  [20:0] win_f;                   // surviving-draw chunks arrived
    reg  [20:0] last_mark;               // next unmarked chunk candidate

    //------------------------------------------------------------------
    // DRAW cost pipeline (H7b.8 Fmax respin -- bit-exact with the old
    // single-cycle cone; see header).  Stages are keyed to the draw's own
    // word arrivals, which can be back-to-back i_CLK cycles when the fetch
    // skid has backlog, so each stage carries at most one DSP level:
    //   w6 edge : x_dxe  = dx + (w-1)           (u16 pre-add, live word)
    //             x_cx0  = max(dx, minx)        (settled since w4)
    //   w7 edge : X regs = x-axis clamp + spans (cx1/cw/dpw/tspans/reject)
    //             y_dye  = dy + (h-1)           (u16 pre-add, live word)
    //   w8 edge : Y regs = y-axis clamp + spans, PLUS the four coefficient
    //             products off the X regs (reassociated -- exact: integer
    //             multiplication is associative, and dpw is always a
    //             multiple of 4 so dst_px/4 = (dpw/4)*chh needs no
    //             correction; src_px/4 truncation is repaired with the
    //             mod-4 remainder term rcs = (cw[1:0]*chh[1:0])[1:0]*c)
    //   w9 edge : M regs = four raw products (single DSP each) -- this is
    //             the emit edge (q_push/q_pkind/q_pnslot as before)
    //   w9+1    : sum + saturate comb into the q_mem write port -- the
    //             entry lands in the queue at the same edge with the same
    //             value as the old design.
    // Widths are proven bounds for SURVIVING draws (clip windows are at
    // most 8192 x 4096, and a passing clip test excludes u16 wrap in the
    // end coordinates -- see blitter_todo H7b.8); rejected draws store 0,
    // so truncated junk in the narrow regs is never observable.
    //------------------------------------------------------------------
    reg  [15:0] x_dxe, y_dye;            // end-coordinate pre-adds
    reg  [15:0] x_cx0;
    reg         x_rej;                   // x-axis reject
    reg  [13:0] x_cw;                    // clip-clamped width   (<= 8192)
    reg  [11:0] x_dpw4;                  // padded dst width / 4 (<= 2050)
    reg  [9:0]  x_ts_sx, x_ts_dx;        // x tile spans         (<= 514)
    reg         y_rej;                   // full reject (x OR y)
    reg  [12:0] y_chh;                   // clip-clamped height  (<= 4096)
    reg  [9:0]  y_ts_sy, y_ts_dy;        // y tile spans         (<= 258)
    reg  [21:0] y_cwc;                   // cw    * C_SRC4
    reg  [19:0] y_dpwc;                  // dpw/4 * C_DST4
    reg  [21:0] y_tsxc;                  // ts_sx * P_SRC
    reg  [22:0] y_tsdc;                  // ts_dx * (P_RW + P_WR)
    reg  [34:0] m_psrc;                  // (cw * C_SRC4) * chh
    reg  [32:0] m_pdst;                  // (dpw/4 * C_DST4) * chh
    reg  [31:0] m_psps;                  // (ts_sx * P_SRC) * ts_sy
    reg  [32:0] m_pspd;                  // (ts_dx * P_RWWR) * ts_dy
    reg  [9:0]  m_rcs;                   // (src_px mod 4) * C_SRC4

    // X-stage comb (w6 -> w7 window)
    wire [15:0] xc_cx1  = (x_dxe < gc_maxx) ? x_dxe : gc_maxx;
    wire [15:0] xc_cw   = xc_cx1 - x_cx0 + 16'd1;
    wire [15:0] xc_dxa  = {x_cx0[15:2], 2'b00};
    wire [15:0] xc_dxb  = xc_cx1 | 16'd3;
    wire [15:0] xc_dpw  = xc_dxb - xc_dxa + 16'd1;   // always a multiple of 4

    // Y-stage comb (w7 -> w8 window)
    wire [15:0] yc_cy0  = (gd_dy > gc_miny) ? gd_dy : gc_miny;
    wire [15:0] yc_cy1  = (y_dye < gc_maxy) ? y_dye : gc_maxy;
    wire [15:0] yc_chh  = yc_cy1 - yc_cy0 + 16'd1;

    // C-stage comb (w9 -> w9+1 window): sum + saturate into the queue.
    // m_psrc - m_rcs is exactly 4x the old floor(src_px/4)*C_SRC4 term.
    wire [35:0] c_tsrc  = {1'b0, m_psrc} - {26'd0, m_rcs};
    wire [35:0] c_sum   = {2'b0, c_tsrc[35:2]}
                        + {3'd0, m_pdst}
                        + {4'd0, m_psps}
                        + {3'd0, m_pspd}
                        + {24'd0, t_p_spr};
    wire [26:0] c_sat   = (|c_sum[35:27]) ? 27'h7FF_FFFF : c_sum[26:0];

    // chunk-slot marking for the governed window (surviving draws only)
    wire [20:0] op_c1   = gp_wcnt[25:5];                 // last-word chunk
    wire [20:0] mark_lo = (gp_c0 > last_mark) ? gp_c0 : last_mark;
    wire [1:0]  nslot_w = (op_c1 >= mark_lo) ? 2'(op_c1 - mark_lo + 21'd1)
                                             : 2'd0;    // draw spans <= 2 chunks

    // cost-queue push interface (driven by the parser below).  The cost
    // itself is NOT a register: wr_cost is the C-stage comb result, muxed
    // to zero for every non-surviving-draw entry, feeding the queue write
    // port directly at the write edge.
    reg         q_push;
    reg  [1:0]  q_pkind;
    reg  [1:0]  q_pnslot;
    wire [26:0] wr_cost = (q_pkind == 2'd1) ? c_sat : 27'd0;

    // cost pipeline registers (stage gates keyed to the draw word index;
    // pauses between words only widen the comb windows)
    wire        gp_dbody = i_push && (gp == GP_BODY) && (gp_kind == 2'd1);
    wire [12:0] xc_tssx  = f_tspan(gd_sx[4:0], xc_cw);
    wire [12:0] xc_tsdx  = f_tspan(x_cx0[4:0], xc_cw);
    wire [12:0] yc_tssy  = f_tspan(gd_sy[4:0], yc_chh);
    wire [12:0] yc_tsdy  = f_tspan(yc_cy0[4:0], yc_chh);
    wire [3:0]  mc_r4    = x_cw[1:0] * y_chh[1:0];   // src_px mod 4 in [1:0]

    // UPLOAD sizing product (w7 live word; see the parser below)
    wire [12:0] up_h1    = {1'b0, i_word[11:0]} + 13'd1;
    wire [26:0] up_px    = gu_dimx1 * up_h1;         // 14x13, <= 2^25

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            x_dxe   <= 16'd0;  x_cx0   <= 16'd0;
            y_dye   <= 16'd0;
            x_rej   <= 1'b0;   x_cw    <= 14'd0;  x_dpw4 <= 12'd0;
            x_ts_sx <= 10'd0;  x_ts_dx <= 10'd0;
            y_rej   <= 1'b0;   y_chh   <= 13'd0;
            y_ts_sy <= 10'd0;  y_ts_dy <= 10'd0;
            y_cwc   <= 22'd0;  y_dpwc  <= 20'd0;
            y_tsxc  <= 22'd0;  y_tsdc  <= 23'd0;
            m_psrc  <= 35'd0;  m_pdst  <= 33'd0;
            m_psps  <= 32'd0;  m_pspd  <= 33'd0;
            m_rcs   <= 10'd0;
        end
        else if (gp_dbody) begin
            case (gp_idx)
                4'd6: begin                  // w6 live: fold w-1 into dxe
                    x_dxe   <= gd_dx + {3'd0, i_word[12:0]};
                    x_cx0   <= (gd_dx > gc_minx) ? gd_dx : gc_minx;
                end
                4'd7: begin                  // X stage + w7 live pre-add
                    x_rej   <= (gd_dx > gc_maxx) || (x_dxe < gc_minx);
                    x_cw    <= xc_cw[13:0];
                    x_dpw4  <= xc_dpw[13:2];
                    x_ts_sx <= xc_tssx[9:0];
                    x_ts_dx <= xc_tsdx[9:0];
                    y_dye   <= gd_dy + {4'd0, i_word[11:0]};
                end
                4'd8: begin                  // Y stage + coefficient DSPs
                    y_rej   <= x_rej || (gd_dy > gc_maxy) || (y_dye < gc_miny);
                    y_chh   <= yc_chh[12:0];
                    y_ts_sy <= yc_tssy[9:0];
                    y_ts_dy <= yc_tsdy[9:0];
                    y_cwc   <= x_cw    * t_c_src4;
                    y_dpwc  <= x_dpw4  * t_c_dst4;
                    y_tsxc  <= x_ts_sx * t_p_src;
                    y_tsdc  <= x_ts_dx * t_rwwr;
                end
                4'd9: begin                  // M stage (emit edge)
                    m_psrc  <= y_cwc   * y_chh;
                    m_pdst  <= y_dpwc  * y_chh;
                    m_psps  <= y_tsxc  * y_ts_sy;
                    m_pspd  <= y_tsdc  * y_ts_dy;
                    m_rcs   <= mc_r4[1:0] * t_c_src4;
                end
                default: ;
            endcase
        end
    end

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            gp        <= GP_HALT;
            gp_idx    <= 4'd0;
            gp_kind   <= 2'd0;
            gp_need   <= 26'd0;
            gp_wcnt   <= 26'd0;
            gp_c0     <= 21'd0;
            gd_sx     <= 16'd0;  gd_sy <= 16'd0;
            gd_dx     <= 16'd0;  gd_dy <= 16'd0;
            gu_dimx1  <= 14'd0;
            gc_minx   <= 16'd0;  gc_maxx <= 16'd0;
            gc_miny   <= 16'd0;  gc_maxy <= 16'd0;
            gl_clip_x <= 16'd0;  gl_clip_y <= 16'd0;
            win_f     <= 21'd0;
            last_mark <= 21'd0;
            q_push    <= 1'b0;
            q_pkind   <= 2'd0;
            q_pnslot  <= 2'd0;
        end
        else begin
            q_push <= 1'b0;

            if (i_exec && !o_busy) begin
                gp        <= GP_HDR;
                gp_wcnt   <= 26'd0;
                win_f     <= 21'd0;
                last_mark <= 21'd0;
                gl_clip_x <= i_clip_x;
                gl_clip_y <= i_clip_y;
                gc_minx   <= i_clip_x - 16'd32;      // window_clip(cx, cy)
                gc_maxx   <= i_clip_x + 16'd351;     // cx + 320-1 + 32
                gc_miny   <= i_clip_y - 16'd32;
                gc_maxy   <= i_clip_y + 16'd271;     // cy + 240-1 + 32
            end

            if (i_push && gp != GP_HALT) begin
                gp_wcnt <= gp_wcnt + 26'd1;

                case (gp)
                    GP_HDR: begin
                        gp_idx <= 4'd1;
                        gp_c0  <= gp_wcnt[25:5];
                        case (i_word[15:12])
                            4'h0, 4'hF: begin        // END
                                q_push  <= 1'b1;
                                q_pkind <= 2'd2;
                                q_pnslot<= 2'd0;
                                gp      <= GP_HALT;
                            end
                            4'hC: begin gp_kind <= 2'd0; gp_need <= 26'd1; gp <= GP_BODY; end
                            4'h1: begin gp_kind <= 2'd1; gp_need <= 26'd9; gp <= GP_BODY; end
                            4'h2: begin gp_kind <= 2'd2; gp_need <= 26'd7; gp <= GP_BODY; end
                            default: begin           // fault: retire like END
                                q_push  <= 1'b1;
                                q_pkind <= 2'd2;
                                q_pnslot<= 2'd0;
                                gp      <= GP_HALT;
`ifndef SYNTHESIS
                                $display("[blit_gov] WALK FAULT: op %04x", i_word);
`endif
                            end
                        endcase
                    end

                    GP_BODY: begin
                        gp_idx  <= (gp_idx == 4'hF) ? 4'hF : gp_idx + 4'd1;
                        gp_need <= gp_need - 26'd1;

                        // DRAW field capture (DrawView slices; the w6/w7
                        // dimension words are captured as end-coordinate
                        // pre-adds in the cost pipeline block above)
                        if (gp_kind == 2'd1) begin
                            case (gp_idx)
                                4'd2: gd_sx <= i_word;
                                4'd3: gd_sy <= i_word;
                                4'd4: gd_dx <= i_word;
                                4'd5: gd_dy <= i_word;
                                default: ;
                            endcase
                        end

                        // UPLOAD payload sizing (same law as the fetch
                        // walker).  dimx+1 is pre-registered so the sizing
                        // product is a single narrow multiply (14x13; the
                        // payload bound 8192*4096 = 2^25 fits gp_need) --
                        // the next push can land on the very next cycle.
                        if (gp_kind == 2'd2 && gp_idx == 4'd6)
                            gu_dimx1 <= {1'b0, i_word[12:0]} + 14'd1;
                        if (gp_kind == 2'd2 && gp_idx == 4'd7)
                            gp_need <= up_px[25:0];
                        else if (gp_need == 26'd1) begin
                            // op complete on THIS word: emit its entry
                            gp <= GP_HDR;
                            case (gp_kind)
                                2'd0: begin          // CLIP: zero cost + window update
                                    q_push  <= 1'b1;
                                    q_pkind <= 2'd0;
                                    q_pnslot<= 2'd0;
                                    if (i_word != 16'd0) begin
                                        gc_minx <= gl_clip_x - 16'd32;
                                        gc_maxx <= gl_clip_x + 16'd351;
                                        gc_miny <= gl_clip_y - 16'd32;
                                        gc_maxy <= gl_clip_y + 16'd271;
                                    end
                                    else begin
                                        gc_minx <= 16'd0;
                                        gc_maxx <= 16'd8191;
                                        gc_miny <= 16'd0;
                                        gc_maxy <= 16'd4095;
                                    end
                                end
                                2'd1: begin          // DRAW: clip test + cost
                                    q_push  <= 1'b1;
                                    if (y_rej) begin // registered clip reject
                                        q_pkind <= 2'd0;
                                        q_pnslot<= 2'd0;
                                    end
                                    else begin
                                        q_pkind <= 2'd1;
                                        q_pnslot<= nslot_w;
                                        win_f   <= win_f + {19'd0, nslot_w};
                                        if (nslot_w != 2'd0)
                                            last_mark <= op_c1 + 21'd1;
                                    end
                                end
                                default: begin       // UPLOAD: fetch-bound, zero cost
                                    q_push  <= 1'b1;
                                    q_pkind <= 2'd0;
                                    q_pnslot<= 2'd0;
                                end
                            endcase
                        end
                    end

                    default: ;                       // GP_HALT: ignore
                endcase
            end
        end
    end

    //------------------------------------------------------------------
    // cost queue (arrival -> timeline).  4096 entries never binds on the
    // trace corpus (worst real backlog ~1.6k draw-class ops = 513 chunks,
    // fifo_study); the almost-full hold below is the safety net.
    //------------------------------------------------------------------
    localparam int unsigned QDEPTH = 4096;

    reg  [30:0] q_mem [0:QDEPTH-1];      // {kind[1:0], nslot[1:0], cost[26:0]}
    reg  [11:0] q_wp, q_rp;
    reg  [12:0] q_lvl;

    wire [30:0] q_head   = q_mem[q_rp];
    wire [1:0]  h_kind   = q_head[30:29];
    wire [1:0]  h_nslot  = q_head[28:27];
    wire [26:0] h_cost   = q_head[26:0];
    wire        q_avail  = (q_lvl != 13'd0);

    reg         q_pop;                   // from the timeline FSM below

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            q_wp  <= 12'd0;
            q_rp  <= 12'd0;
            q_lvl <= 13'd0;
        end
        else begin
            if (i_exec && !o_busy) begin
                q_wp  <= 12'd0;
                q_rp  <= 12'd0;
                q_lvl <= 13'd0;
            end
            else begin
                if (q_push) begin
                    q_mem[q_wp] <= {q_pkind, q_pnslot, wr_cost};
                    q_wp        <= q_wp + 12'd1;
`ifndef SYNTHESIS
                    if (q_lvl >= 13'(QDEPTH)) $fatal(1, "[blit_gov] cost queue overflow");
`endif
                end
                if (q_pop)
                    q_rp <= q_rp + 12'd1;
                q_lvl <= q_lvl + (q_push ? 13'd1 : 13'd0)
                               - (q_pop  ? 13'd1 : 13'd0);
            end
        end
    end

    //------------------------------------------------------------------
    // timeline FSM: half-VCLK time base, op_start = max(engine_free, ready)
    //------------------------------------------------------------------
    reg  [47:0] now, engine_free, next_bnd;
    reg  [47:0] r_t0;                    // now at EXEC (delta base)
    reg  [20:0] win_r;                   // surviving-draw chunks gov-started
    reg         stealing;                // draw popped, boundary loop running
    reg         r_first_v;               // r_first_start captured this exec

    // retirement report (read hierarchically by the TBs; ticks since EXEC)
    reg  [47:0] r_busy_end   /*verilator public_flat_rd*/;  // engine_free @ END
    reg  [47:0] r_first_start/*verilator public_flat_rd*/;  // first op_start
    reg  [31:0] r_ndraw      /*verilator public_flat_rd*/;
    reg  [47:0] r_cost_sum   /*verilator public_flat_rd*/;  // VCLK, draws only

    assign o_fetch_hold = ({4'd0, win_f} - {4'd0, win_r} >= {9'd0, t_window})
                       || (q_lvl >= 13'(QDEPTH - 64));

    wire [47:0] steal2 = {35'd0, t_steal, 1'b0};             // VCLK -> half-VCLK
    wire [47:0] cost2  = {20'd0, h_cost, 1'b0};

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            now           <= 48'd0;
            engine_free   <= 48'd0;
            next_bnd      <= 48'd0;
            r_t0          <= 48'd0;
            win_r         <= 21'd0;
            stealing      <= 1'b0;
            r_first_v     <= 1'b0;
            o_busy        <= 1'b0;
            o_retire      <= 1'b0;
            o_dbg_vld     <= 1'b0;
            o_dbg_kind    <= 2'd0;
            o_dbg_cost    <= 27'd0;
            q_pop         <= 1'b0;
            r_busy_end    <= 48'd0;
            r_first_start <= 48'd0;
            r_ndraw       <= 32'd0;
            r_cost_sum    <= 48'd0;
        end
        else begin
            o_retire  <= 1'b0;
            q_pop     <= 1'b0;

            // per-op debug tap (arrival side)
            o_dbg_vld  <= q_push;
            o_dbg_kind <= q_pkind;
            o_dbg_cost <= wr_cost;

            // time base: 3 half-VCLK per CKIO, free-running since reset
            if (i_CKIO_PCEN)
                now <= now + 48'd3;

            // boundary tracker, busy or idle.  While `stealing`, one hline
            // boundary per cycle (C++ add_steals loop, pre-accounting the
            // popped draw's steals); otherwise keep next_bnd = first
            // boundary after `now`, snapping to the real scanline whenever
            // i_hline pulses (exact + idempotent: both sides count the
            // same CKIO grid).  Boundaries pre-accounted ahead of `now`
            // ignore the pulse.
            if (stealing) begin
                if (next_bnd < engine_free) begin
                    engine_free <= engine_free + steal2;
                    next_bnd    <= next_bnd + {32'd0, t_hline_p};
                end
                else
                    stealing <= 1'b0;
            end
            else if (next_bnd <= now)
                next_bnd <= i_hline ? (now + {32'd0, t_hline_p})
                                    : (next_bnd + {32'd0, t_hline_p});

            if (i_exec) begin
                if (!o_busy) begin
                    o_busy      <= 1'b1;
                    engine_free <= now;
                    r_t0        <= now;
                    win_r       <= 21'd0;
                    r_first_v   <= 1'b0;
                    r_ndraw     <= 32'd0;
                    r_cost_sum  <= 48'd0;
                end
`ifndef SYNTHESIS
                else $display("[blit_gov] WARNING: EXEC while governor busy");
`endif
            end
            else if (o_busy && !stealing && next_bnd > now
                     && q_avail && !q_pop) begin
                if (now >= engine_free) begin
                    // op_start = now (= max(engine_free, arrival))
                    q_pop <= 1'b1;
                    if (!r_first_v) begin
                        r_first_v     <= 1'b1;
                        r_first_start <= now - r_t0;
                    end
                    case (h_kind)
                        2'd0: begin              // zero-cost op
                            engine_free <= now;
                        end
                        2'd1: begin              // draw
                            engine_free <= now + cost2;
                            win_r       <= win_r + {19'd0, h_nslot};
                            stealing    <= t_steal_en;
                            r_ndraw     <= r_ndraw + 32'd1;
                            r_cost_sum  <= r_cost_sum + {21'd0, h_cost};
                        end
                        default: begin           // END / fault: retire
                            o_busy     <= 1'b0;
                            o_retire   <= 1'b1;
                            r_busy_end <= engine_free - r_t0;
`ifndef SYNTHESIS
                            if (!i_warp)   // board sim only (TB replays 80k execs)
                                $display("[blit_gov] retire: busy_end(model)=%0d ticks (%.2f us), deassert=%0d ticks (%.2f us), draws=%0d, cost_sum=%0d VCLK",
                                         engine_free - r_t0, real'(engine_free - r_t0) * 6.5104e-3,
                                         now - r_t0,         real'(now - r_t0)         * 6.5104e-3,
                                         r_ndraw, r_cost_sum);
`endif
                        end
                    endcase
                end
                else if (i_warp)
                    now <= engine_free;          // TB fast-forward
            end
        end
    end

endmodule
`default_nettype none
