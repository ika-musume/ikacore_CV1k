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
// w8, raw products at w9 (the emit edge) -- so no cycle carries more than
// one DSP level.  H7b.8e: the final sum + saturate no longer combs into the
// queue write port; the span DSPs and the sum are spread over three
// post-emit register stages (P1 = span products + src/dst partial, P2 =
// second partial, F = final add + saturate + kind mux), and the M10K
// write port + pointers are driven from the F registers one edge later
// still.  Entry VALUES are bit-exact vs the single-cone form (pure
// reassociation of a mod-2^36 zero-extended sum -- dpw is always a
// multiple of 4, and the src_px/4 truncation is corrected with a mod-4
// remainder term; all narrowed widths are proven bounds for surviving
// draws, and rejected draws store 0).  Every entry becomes pop-visible
// three i_CLK edges later than the old design; op_start/engine_free
// quantize on the CKIO grid and the pop schedule is engine-bound at the
// contractual points, so the timeline accepts (datum + anchors + matrix)
// adjudicate -- see blitter_todo Part V H7b.8e.  The cost queue infers M10K with the
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

    // f_tspan with the off-dependent terms pre-decoded (nz = off != 0,
    // thr = 32 - off): part > thr <=> off + part > 32.  Value-identical --
    // off = 0 gives thr = 32, which part <= 31 can never exceed, matching
    // the (off + part) > 32 impossibility at off = 0.
    function automatic [12:0] f_tspan_p(input nz, input [5:0] thr,
                                        input [15:0] len);
        reg [10:0] nfull;
        reg [4:0]  part;
        begin
            nfull     = len[15:5];
            part      = len[4:0];
            f_tspan_p = {2'd0, nfull}
                      + ((part != 5'd0) ? 13'd1 : 13'd0)
                      + (nz ? {2'd0, nfull} : 13'd0)
                      + (((part != 5'd0) &&
                          ({1'b0, part} > thr)) ? 13'd1 : 13'd0);
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
    reg  [12:0] gu_dimy1;                // r4 iter3: (w7 & 0xfff) + 1
    reg         gu_dx1_is1, gu_dy1_is1;  // 1-word-payload end detect
    reg         gu_dx1_is2, gu_dy1_is2;  // px==2 detect for gp_is1
    reg         gu_pend;                 // deferred product pending
    reg         gp_is1;                  // r4 iter4: registered (gp_need == 1)
                                         // -- the emit-decision compare off a
                                         // flag instead of the 26-bit counter
                                         // (mirrors blit_fetch w_is1)

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
    //   w9 edge : m_psrc/m_pdst/m_rcs + the span coefficient products
    //             (single DSP each) -- this is the emit edge
    //             (q_push/q_pkind/q_pnslot as before)
    //   w9+1    : P1 -- span products s_psps/s_pspd (single DSP each) +
    //             the src/dst partial p_a
    //   w9+2    : P2 -- second partial p_b (+ p_a pass-through)
    //   w9+3    : F  -- final add + saturate + kind mux (registered)
    //   w9+4    : the entry lands in q_mem, write port fed from registers
    //             (H7b.8e round 3; entry values identical, visibility +3
    //             edges vs the original comb form)
    // Widths are proven bounds for SURVIVING draws (clip windows are at
    // most 8192 x 4096, and a passing clip test excludes u16 wrap in the
    // end coordinates -- see blitter_todo H7b.8); rejected draws store 0,
    // so truncated junk in the narrow regs is never observable.
    //------------------------------------------------------------------
    reg  [15:0] x_dxe, y_dye;            // end-coordinate pre-adds
    reg  [15:0] x_cx0;
    reg  [15:0] y_cy0;                   // w6 pre-reg: max(gd_dy, gc_miny)
    reg         ys_nz;                   // w6: gd_sy[4:0] != 0
    reg  [5:0]  ys_thr;                  // w6: 32 - gd_sy[4:0] (straddle base)
    reg         xs_nz;                   // w6: gd_sx[4:0] != 0     (H7b.8e)
    reg  [5:0]  xs_thr;                  // w6: 32 - gd_sx[4:0]
    reg         xd_nz;                   // w6: x_cx0[4:0] != 0 (via the mux)
    reg  [5:0]  xd_thr;                  // w6: 32 - x_cx0[4:0]
    reg         yd_nz;                   // w7: y_cy0[4:0] != 0
    reg  [5:0]  yd_thr;                  // w7: 32 - y_cy0[4:0]
    reg  [15:0] y_gmc;                   // w7: gc_maxy - y_cy0 + 1 (clamp arm)
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
    reg  [9:0]  m_rcs;                   // (src_px mod 4) * C_SRC4
    // (the span products ts*P land in the P1 stage regs s_psps/s_pspd)

    // X-stage comb (w6 -> w7 window).  H7b.8e: the clamp subtracts are
    // DISTRIBUTED over the select (exact mod 2^16 -- same arithmetic on
    // both arms), mirroring the Y-stage recipe, so the compare runs in
    // parallel with the subtracts instead of in series before them.
    wire        xc_clmp = (x_dxe < gc_maxx);
    wire [15:0] xc_cw   = xc_clmp ? (x_dxe - x_cx0 + 16'd1)
                                  : (gc_maxx - x_cx0 + 16'd1);
    wire [15:0] xc_dxa  = {x_cx0[15:2], 2'b00};
    wire [15:0] xc_dpw  = xc_clmp                    // always a multiple of 4
                          ? ((x_dxe   | 16'd3) - xc_dxa + 16'd1)
                          : ((gc_maxx | 16'd3) - xc_dxa + 16'd1);

    // Y-stage comb (w7 -> w8 window).  H7b.8c pre-regs: yc_cy0 and the
    // f_tspan off-terms land in registers at w6/w7 (same values -- gd_dy is
    // a w5 register, gd_sy a w3 register, and the clip window regs cannot
    // change inside a draw body since ops are strictly serial in the push
    // stream), so this window is one compare-mux + one subtract + the span
    // sums off settled registers.
    // Second round: chh = min(y_dye, gc_maxy) - y_cy0 + 1 with the subtract
    // DISTRIBUTED over the select (exact mod 2^16 -- same arithmetic either
    // side of the mux): the clamp arm is pre-subtracted into y_gmc at w7, so
    // the w8 window runs the live subtract and the compare IN PARALLEL
    // instead of compare-mux -> subtract in series.
    wire [15:0] yc_chh  = (y_dye < gc_maxy) ? (y_dye - y_cy0 + 16'd1) : y_gmc;

    // C-stage (H7b.8e): the sum + saturate is a two-stage register pipe
    // BEHIND the emit edge (P/F stages below the parser) instead of one
    // comb cone into the q_mem write port -- see the header note.  This
    // APPLIES the previously shelved reassociation lever (proven bit-exact
    // 2026-07-18: balance the products in a pair tree; every term is
    // zero-extended into 36 bits and no partial wraps, so the mod-2^36
    // value is identical to the old left-fold) and then splits the tree at
    // register boundaries.  m_psrc - m_rcs is exactly 4x the old
    // floor(src_px/4)*C_SRC4 term, and cannot underflow (m_rcs =
    // (src_px mod 4)*C_SRC4 <= src_px*C_SRC4 = m_psrc).
    wire [35:0] c_tsrc  = {1'b0, m_psrc} - {26'd0, m_rcs};

    // chunk-slot marking for the governed window (surviving draws only).
    // H7b.8e r4: the old comb form -- mark_lo = max(gp_c0, last_mark),
    // nslot = (op_c1 >= mark_lo) ? op_c1-mark_lo+1 : 0 -- chained two
    // 21-bit compares and the win_f carry chain into the emit edge.  A
    // draw is 10 words, so between its HDR word (gp_c0 loads) and its
    // emit word both gp_c0 and last_mark are STABLE for >= 8 pushes, and
    // the "spans <= 2 chunks" invariant means op_c1 is either gp_c0 or
    // gp_c0+1.  Enumerating both candidates one cycle behind their inputs
    // (free-running regs, settled long before any emit) leaves only a
    // 2:1 select by the registered crossing flag at the emit edge:
    //   op_c1 == gp_c0   : nslot = (lm <= c0) ? 1 : 0
    //   op_c1 == gp_c0+1 : nslot = (lm <= c0) ? 2 : (lm == c0+1) ? 1 : 0
    // The sim-only oracle below re-proves select == old formula at every
    // surviving-draw emit.
    reg  [1:0]  ns_same_q, ns_cross_q;   // nslot candidates
    reg  [20:0] lm_same_q, lm_cross_q;   // last_mark update candidates
    reg         gp_crossed;              // this op's words crossed a chunk edge
    always_ff @(posedge i_CLK) begin
        ns_same_q  <= (last_mark <= gp_c0) ? 2'd1 : 2'd0;
        ns_cross_q <= (last_mark <= gp_c0) ? 2'd2
                    : (last_mark == gp_c0 + 21'd1) ? 2'd1 : 2'd0;
        lm_same_q  <= gp_c0 + 21'd1;
        lm_cross_q <= gp_c0 + 21'd2;
    end

    // cost-queue push interface (driven by the parser below).  q_push marks
    // the EMIT edge; the entry then takes the P/F register stages and lands
    // in the RAM (pop-visible) two edges later.
    reg         q_push;
    reg  [1:0]  q_pkind;
    reg  [1:0]  q_pnslot;

    // H7b.8e write-side stages (round 3 shape): P1 (w9+1) = the two span
    // DSP products + the src/dst partial, P2 (w9+2) = the second partial,
    // F (w9+3) = final add + saturate + non-surviving mux, RAM write at
    // w9+4.  No window carries more than one DSP or two adder levels.
    // The w9 regs are stable through P1 (they reload no earlier than the
    // NEXT draw's w9, >= 10 pushes away), s_* / p_a through P2 likewise,
    // and the table regs cannot change mid-list (uploads are serialized
    // against EXEC).
    reg         p1_vld, p2_vld, f_vld;
    reg  [1:0]  p1_kind, p1_nslot, p2_kind, p2_nslot, f_kind, f_nslot;
    reg  [31:0] s_psps;                  // (ts_sx * P_SRC) * ts_sy
    reg  [32:0] s_pspd;                  // (ts_dx * P_RWWR) * ts_dy
    reg  [35:0] p_a, p_a2, p_b;
    reg  [26:0] f_cost;
    wire [35:0] f_sum = p_a2 + p_b;
    wire [26:0] f_sat = (|f_sum[35:27]) ? 27'h7FF_FFFF : f_sum[26:0];

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            p1_vld  <= 1'b0;  p2_vld   <= 1'b0;  f_vld <= 1'b0;
            p1_kind <= 2'd0;  p1_nslot <= 2'd0;
            p2_kind <= 2'd0;  p2_nslot <= 2'd0;
            f_kind  <= 2'd0;  f_nslot  <= 2'd0;
            s_psps  <= 32'd0; s_pspd   <= 33'd0;
            p_a     <= 36'd0; p_a2     <= 36'd0;  p_b <= 36'd0;
            f_cost  <= 27'd0;
        end
        else if (i_exec && !o_busy) begin
            // queue reset instant: the pipe is provably empty here (END was
            // pushed, drained through W, and popped before o_busy fell, and
            // GP_HALT pushes nothing after END) -- cleared for hygiene.
            p1_vld <= 1'b0;
            p2_vld <= 1'b0;
            f_vld  <= 1'b0;
        end
        else begin
            p1_vld  <= q_push;
            p1_kind <= q_pkind;
            p1_nslot<= q_pnslot;
            if (q_push) begin
                s_psps <= y_tsxc * y_ts_sy;
                s_pspd <= y_tsdc * y_ts_dy;
                p_a    <= {2'b0, c_tsrc[35:2]} + {3'd0, m_pdst};
            end
            p2_vld  <= p1_vld;
            p2_kind <= p1_kind;
            p2_nslot<= p1_nslot;
            if (p1_vld) begin
                p_b  <= {4'd0, s_psps} + {3'd0, s_pspd} + {24'd0, t_p_spr};
                p_a2 <= p_a;
            end
            f_vld   <= p2_vld;
            f_kind  <= p2_kind;
            f_nslot <= p2_nslot;
            if (p2_vld)
                f_cost <= (p2_kind == 2'd1) ? f_sat : 27'd0;
        end
    end

    // cost pipeline registers (stage gates keyed to the draw word index;
    // pauses between words only widen the comb windows)
    wire        gp_dbody = i_push && (gp == GP_BODY) && (gp_kind == 2'd1);
    // H7b.8e: X spans use the pre-split form too (xs_*/xd_* registered at
    // w6, off gd_sx and the same max() select that produces x_cx0); their
    // f_tspan_p calls sit in the w8 arm off the REGISTERED x_cw
    wire [4:0]  xc_cx0lo = (gd_dx > gc_minx) ? gd_dx[4:0] : gc_minx[4:0];
    wire [12:0] xc8_tssx = f_tspan_p(xs_nz, xs_thr, {2'd0, x_cw});
    wire [12:0] xc8_tsdx = f_tspan_p(xd_nz, xd_thr, {2'd0, x_cw});
    // r4 iter2: the y tile spans move to the w9 arm, computed off the
    // REGISTERED y_chh (w8) -- their only consumers are the P1-stage
    // products one cycle after the w9 emit edge, so capturing them at w9
    // is value- and instant-identical, and the w8 window no longer chains
    // subtract -> mux -> tspan off the live y_dye clamp.
    wire [12:0] yc9_tssy = f_tspan_p(ys_nz, ys_thr, {3'd0, y_chh});
    wire [12:0] yc9_tsdy = f_tspan_p(yd_nz, yd_thr, {3'd0, y_chh});
    wire [3:0]  mc_r4    = x_cw[1:0] * y_chh[1:0];   // src_px mod 4 in [1:0]

    // UPLOAD sizing product (w7 live word; see the parser below)
    // r4 iter3: register x register (the live-word form up_h1/up_px is
    // retired; the product lands one push later under gu_pend)
    wire [26:0] up_pxm  = gu_dimx1 * gu_dimy1;       // 14x13, <= 2^25
    wire [26:0] up_pxm1 = up_pxm - 27'd1;

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            x_dxe   <= 16'd0;  x_cx0   <= 16'd0;
            y_dye   <= 16'd0;  y_cy0   <= 16'd0;
            ys_nz   <= 1'b0;   ys_thr  <= 6'd0;
            yd_nz   <= 1'b0;   yd_thr  <= 6'd0;
            xs_nz   <= 1'b0;   xs_thr  <= 6'd0;
            xd_nz   <= 1'b0;   xd_thr  <= 6'd0;
            y_gmc   <= 16'd0;
            x_rej   <= 1'b0;   x_cw    <= 14'd0;  x_dpw4 <= 12'd0;
            x_ts_sx <= 10'd0;  x_ts_dx <= 10'd0;
            y_rej   <= 1'b0;   y_chh   <= 13'd0;
            y_ts_sy <= 10'd0;  y_ts_dy <= 10'd0;
            y_cwc   <= 22'd0;  y_dpwc  <= 20'd0;
            y_tsxc  <= 22'd0;  y_tsdc  <= 23'd0;
            m_psrc  <= 35'd0;  m_pdst  <= 33'd0;
            m_rcs   <= 10'd0;
        end
        else if (gp_dbody) begin
            case (gp_idx)
                4'd6: begin                  // w6 live: fold w-1 into dxe
                    x_dxe   <= gd_dx + {3'd0, i_word[12:0]};
                    x_cx0   <= (gd_dx > gc_minx) ? gd_dx : gc_minx;
                    y_cy0   <= (gd_dy > gc_miny) ? gd_dy : gc_miny;
                    ys_nz   <= (gd_sy[4:0] != 5'd0);
                    ys_thr  <= 6'd32 - {1'b0, gd_sy[4:0]};
                    xs_nz   <= (gd_sx[4:0] != 5'd0);
                    xs_thr  <= 6'd32 - {1'b0, gd_sx[4:0]};
                    xd_nz   <= (xc_cx0lo != 5'd0);
                    xd_thr  <= 6'd32 - {1'b0, xc_cx0lo};
                end
                4'd7: begin                  // X clamp stage + w7 live pre-add
                    x_rej   <= (gd_dx > gc_maxx) || (x_dxe < gc_minx);
                    x_cw    <= xc_cw[13:0];
                    x_dpw4  <= xc_dpw[13:2];
                    y_dye   <= gd_dy + {4'd0, i_word[11:0]};
                    yd_nz   <= (y_cy0[4:0] != 5'd0);
                    yd_thr  <= 6'd32 - {1'b0, y_cy0[4:0]};
                    y_gmc   <= gc_maxy - y_cy0 + 16'd1;
                end
                4'd8: begin                  // Y stage + X spans (H7b.8e:
                                             // tspans off the REGISTERED
                                             // x_cw -- no clamp in series)
                    y_rej   <= x_rej || (gd_dy > gc_maxy) || (y_dye < gc_miny);
                    y_chh   <= yc_chh[12:0];
                    y_cwc   <= x_cw    * t_c_src4;
                    y_dpwc  <= x_dpw4  * t_c_dst4;
                    x_ts_sx <= xc8_tssx[9:0];
                    x_ts_dx <= xc8_tsdx[9:0];
                end
                4'd9: begin                  // M stage (emit edge; the span
                                             // DSPs moved here -- their
                                             // products land in P1.  r4
                                             // iter2: y tile spans off the
                                             // registered y_chh, consumed
                                             // at P1 -- instant-identical)
                    m_psrc  <= y_cwc   * y_chh;
                    m_pdst  <= y_dpwc  * y_chh;
                    y_ts_sy <= yc9_tssy[9:0];
                    y_ts_dy <= yc9_tsdy[9:0];
                    y_tsxc  <= x_ts_sx * t_p_src;
                    y_tsdc  <= x_ts_dx * t_rwwr;
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
            gp_crossed<= 1'b0;
            gd_sx     <= 16'd0;  gd_sy <= 16'd0;
            gd_dx     <= 16'd0;  gd_dy <= 16'd0;
            gu_dimx1  <= 14'd0;
            gu_dimy1  <= 13'd0;
            gu_dx1_is1<= 1'b0;
            gu_dy1_is1<= 1'b0;
            gu_dx1_is2<= 1'b0;
            gu_dy1_is2<= 1'b0;
            gu_pend   <= 1'b0;
            gp_is1    <= 1'b0;
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
                        // r4: crossed iff a later word of THIS op sits in
                        // the next chunk -- seeded by the HDR word itself
                        // being the last word of its chunk
                        gp_crossed <= (gp_wcnt[4:0] == 5'd31);
                        case (i_word[15:12])
                            4'h0, 4'hF: begin        // END
                                q_push  <= 1'b1;
                                q_pkind <= 2'd2;
                                q_pnslot<= 2'd0;
                                gp      <= GP_HALT;
                            end
                            4'hC: begin gp_kind <= 2'd0; gp_need <= 26'd1; gp_is1 <= 1'b1; gp <= GP_BODY; end
                            4'h1: begin gp_kind <= 2'd1; gp_need <= 26'd9; gp_is1 <= 1'b0; gp <= GP_BODY; end
                            4'h2: begin gp_kind <= 2'd2; gp_need <= 26'd7; gp_is1 <= 1'b0; gp <= GP_BODY; end
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
                        gp_is1  <= (gp_need == 26'd2);   // exact next-value
                        if (gp_wcnt[4:0] == 5'd31)   // r4: next word crosses
                            gp_crossed <= 1'b1;

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
                        // walker).  r4 iter3: the dimy side gets the fetch
                        // walker's r4 recipe -- dimy+1 registered at word
                        // 7 and the product DEFERRED one push (register x
                        // register into the DSP), so the live snoop-word
                        // mux leaves the gp_need cone.  The 1-word payload
                        // (dimx=dimy=0: the deferral word is also the last)
                        // emits via the pre-registered is1 flags -- same
                        // word, same entry, same instants.
                        if (gp_kind == 2'd2 && gp_idx == 4'd6) begin
                            gu_dimx1  <= {1'b0, i_word[12:0]} + 14'd1;
                            gu_dx1_is1<= (i_word[12:0] == 13'd0);
                            gu_dx1_is2<= (i_word[12:0] == 13'd1);
                        end
                        if (gu_pend) begin
                            gu_pend <= 1'b0;
                            gp_need <= up_pxm1[25:0];
                            gp_is1  <= (gu_dx1_is2 && gu_dy1_is1)   // px == 2
                                    || (gu_dx1_is1 && gu_dy1_is2);
                            if (gu_dx1_is1 && gu_dy1_is1) begin
                                gp      <= GP_HDR;
                                q_push  <= 1'b1;
                                q_pkind <= 2'd0;     // UPLOAD: zero cost
                                q_pnslot<= 2'd0;
                            end
                        end
                        else if (gp_kind == 2'd2 && gp_idx == 4'd7) begin
                            gu_dimy1  <= {1'b0, i_word[11:0]} + 13'd1;
                            gu_dy1_is1<= (i_word[11:0] == 12'd0);
                            gu_dy1_is2<= (i_word[11:0] == 12'd1);
                            gu_pend   <= 1'b1;
                        end
                        else if (gp_is1) begin           // == (gp_need == 1)
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
                                    else begin       // r4: staged candidates,
                                                     // 2:1 select at the edge
                                        automatic logic [1:0] ns_w;
                                        ns_w = gp_crossed ? ns_cross_q
                                                          : ns_same_q;
                                        q_pkind <= 2'd1;
                                        q_pnslot<= ns_w;
                                        win_f   <= win_f + {19'd0, ns_w};
                                        if (ns_w != 2'd0)
                                            last_mark <= gp_crossed ? lm_cross_q
                                                                    : lm_same_q;
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

`ifndef SYNTHESIS
    // r4 iter4 oracle: gp_is1 must track (gp_need == 1) in GP_BODY
    always @(posedge i_CLK)
        if (i_RST_n && gp == GP_BODY && gp_is1 != (gp_need == 26'd1))
            $fatal(2, "[blit_gov] r4 gp_is1 diverged (is1=%b need=%0d) t=%0t",
                   gp_is1, gp_need, $time);
`endif

`ifndef SYNTHESIS
    // r4 shadow oracle: the staged nslot/last_mark candidates must equal
    // the retired comb formula at every surviving-draw emit instant.
    always @(posedge i_CLK) begin
        if (i_RST_n && i_push && gp == GP_BODY && gp_kind == 2'd1 &&
            gp_need == 26'd1 && !y_rej) begin
            automatic logic [20:0] chk_c1  = gp_wcnt[25:5];
            automatic logic [20:0] chk_mlo = (gp_c0 > last_mark) ? gp_c0
                                                                 : last_mark;
            automatic logic [1:0]  chk_ns  = (chk_c1 >= chk_mlo)
                                   ? 2'(chk_c1 - chk_mlo + 21'd1) : 2'd0;
            automatic logic [1:0]  got_ns  = gp_crossed ? ns_cross_q
                                                        : ns_same_q;
            if (got_ns != chk_ns)
                $fatal(2, "[blit_gov] r4 nslot stage diverged: got %0d want %0d (c0=%0d c1=%0d lm=%0d crossed=%0d) t=%0t",
                       got_ns, chk_ns, gp_c0, chk_c1, last_mark, gp_crossed, $time);
            if (chk_ns != 2'd0 &&
                (gp_crossed ? lm_cross_q : lm_same_q) != chk_c1 + 21'd1)
                $fatal(2, "[blit_gov] r4 last_mark stage diverged: got %0d want %0d t=%0t",
                       gp_crossed ? lm_cross_q : lm_same_q, chk_c1 + 21'd1, $time);
        end
    end
`endif

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
                if (f_vld) begin
                    q_mem[q_wp] <= {f_kind, f_nslot, f_cost};
                    q_wp        <= q_wp + 12'd1;
`ifndef SYNTHESIS
                    if (q_lvl >= 13'(QDEPTH)) $fatal(1, "[blit_gov] cost queue overflow");
`endif
                end
                if (q_pop)
                    q_rp <= q_rp + 12'd1;
                q_lvl <= q_lvl + (f_vld ? 13'd1 : 13'd0)
                               - (q_pop ? 13'd1 : 13'd0);
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

            // per-op debug tap (arrival side; tracks the RAM write edge --
            // same (kind, cost) sequence as before, instants +2 i_CLK)
            o_dbg_vld  <= f_vld;
            o_dbg_kind <= f_kind;
            o_dbg_cost <= f_cost;

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
