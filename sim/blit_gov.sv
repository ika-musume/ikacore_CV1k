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
// Synthesis notes (sim-first, same convention as blit_draw): the DRAW cost
// is computed combinationally at the op's last-word arrival (three 26-38 bit
// multiply-adds -- DSP-able but long; >= 10 cycles of slack exist per draw,
// so this can be pipelined/multi-cycled for Fmax later), and the cost queue
// is a comb-read 4096x32 array (respin to sync-read BRAM at the MiSTer pass).
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
                4'd1 : t_p_rw     <= i_tbl_data[11:0];
                4'd2 : t_p_wr     <= i_tbl_data[11:0];
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

    // captured DRAW fields (raw u16, DrawView slices)
    reg  [15:0] gd_sx, gd_sy, gd_dx, gd_dy;
    reg  [13:0] gd_w;                    // (w6 & 0x1fff) + 1
    reg  [12:0] gd_h;                    // (w7 & 0x0fff) + 1
    reg  [12:0] gu_dimx;                 // UPLOAD w6 & 0x1fff

    // clip window state (u16 wrap semantics, workload.h window_clip)
    reg  [15:0] gc_minx, gc_maxx, gc_miny, gc_maxy;
    reg  [15:0] gl_clip_x, gl_clip_y;    // EXEC-latched clip regs (CLIP re-derive)

    // window bookkeeping
    reg  [20:0] win_f;                   // surviving-draw chunks arrived
    reg  [20:0] last_mark;               // next unmarked chunk candidate

    // clip test + clamp on the registered DRAW fields (valid at word idx 9;
    // fields settle by idx 7, so this comb cone has >= 2 spare cycles)
    wire [15:0] dxe    = gd_dx + {2'd0, gd_w} - 16'd1;
    wire [15:0] dye    = gd_dy + {3'd0, gd_h} - 16'd1;
    wire        reject = (gd_dx > gc_maxx) || (dxe < gc_minx) ||
                         (gd_dy > gc_maxy) || (dye < gc_miny);
    wire [15:0] cx0    = (gd_dx > gc_minx) ? gd_dx : gc_minx;
    wire [15:0] cy0    = (gd_dy > gc_miny) ? gd_dy : gc_miny;
    wire [15:0] cx1    = (dxe   < gc_maxx) ? dxe   : gc_maxx;
    wire [15:0] cy1    = (dye   < gc_maxy) ? dye   : gc_maxy;
    wire [15:0] cw     = cx1 - cx0 + 16'd1;      // MAME keeps src origin,
    wire [15:0] chh    = cy1 - cy0 + 16'd1;      // shrinks dims

    // BD §6.5 draw cost from the tables
    wire [31:0] src_px  = cw * chh;
    wire [15:0] dxa     = {cx0[15:2], 2'b00};
    wire [15:0] dxb     = cx1 | 16'd3;
    wire [15:0] dpw     = dxb - dxa + 16'd1;
    wire [31:0] dst_px  = dpw * chh;
    wire [25:0] sp_s    = f_tspan(gd_sx[4:0], cw) * f_tspan(gd_sy[4:0], chh);
    wire [25:0] sp_d    = f_tspan(cx0[4:0],   cw) * f_tspan(cy0[4:0],   chh);
    wire [39:0] cost_w  = {10'd0, src_px[31:2]} * {32'd0, t_c_src4}
                        + {10'd0, dst_px[31:2]} * {32'd0, t_c_dst4}
                        + {14'd0, sp_s} * {28'd0, t_p_src}
                        + {14'd0, sp_d} * ({27'd0, t_p_rw} + {27'd0, t_p_wr})
                        + {28'd0, t_p_spr};
    wire [26:0] cost_v  = (|cost_w[39:27]) ? 27'h7FF_FFFF : cost_w[26:0];

    // chunk-slot marking for the governed window (surviving draws only)
    wire [20:0] op_c1   = gp_wcnt[25:5];                 // last-word chunk
    wire [20:0] mark_lo = (gp_c0 > last_mark) ? gp_c0 : last_mark;
    wire [1:0]  nslot_w = (op_c1 >= mark_lo) ? 2'(op_c1 - mark_lo + 21'd1)
                                             : 2'd0;    // draw spans <= 2 chunks

    // cost-queue push interface (driven by the parser below)
    reg         q_push;
    reg  [1:0]  q_pkind;
    reg  [1:0]  q_pnslot;
    reg  [26:0] q_pcost;

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
            gd_w      <= 14'd0;  gd_h  <= 13'd0;
            gu_dimx   <= 13'd0;
            gc_minx   <= 16'd0;  gc_maxx <= 16'd0;
            gc_miny   <= 16'd0;  gc_maxy <= 16'd0;
            gl_clip_x <= 16'd0;  gl_clip_y <= 16'd0;
            win_f     <= 21'd0;
            last_mark <= 21'd0;
            q_push    <= 1'b0;
            q_pkind   <= 2'd0;
            q_pnslot  <= 2'd0;
            q_pcost   <= 27'd0;
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
                                q_pcost <= 27'd0;
                                gp      <= GP_HALT;
                            end
                            4'hC: begin gp_kind <= 2'd0; gp_need <= 26'd1; gp <= GP_BODY; end
                            4'h1: begin gp_kind <= 2'd1; gp_need <= 26'd9; gp <= GP_BODY; end
                            4'h2: begin gp_kind <= 2'd2; gp_need <= 26'd7; gp <= GP_BODY; end
                            default: begin           // fault: retire like END
                                q_push  <= 1'b1;
                                q_pkind <= 2'd2;
                                q_pnslot<= 2'd0;
                                q_pcost <= 27'd0;
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

                        // DRAW field capture (DrawView slices)
                        if (gp_kind == 2'd1) begin
                            case (gp_idx)
                                4'd2: gd_sx <= i_word;
                                4'd3: gd_sy <= i_word;
                                4'd4: gd_dx <= i_word;
                                4'd5: gd_dy <= i_word;
                                4'd6: gd_w  <= {1'b0, i_word[12:0]} + 14'd1;
                                4'd7: gd_h  <= {1'b0, i_word[11:0]} + 13'd1;
                                default: ;
                            endcase
                        end

                        // UPLOAD payload sizing (same law as the fetch walker)
                        if (gp_kind == 2'd2 && gp_idx == 4'd6)
                            gu_dimx <= i_word[12:0];
                        if (gp_kind == 2'd2 && gp_idx == 4'd7)
                            gp_need <= (26'(gu_dimx) + 26'd1) *
                                       (26'({14'd0, i_word[11:0]}) + 26'd1);
                        else if (gp_need == 26'd1) begin
                            // op complete on THIS word: emit its entry
                            gp <= GP_HDR;
                            case (gp_kind)
                                2'd0: begin          // CLIP: zero cost + window update
                                    q_push  <= 1'b1;
                                    q_pkind <= 2'd0;
                                    q_pnslot<= 2'd0;
                                    q_pcost <= 27'd0;
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
                                    if (reject) begin
                                        q_pkind <= 2'd0;
                                        q_pnslot<= 2'd0;
                                        q_pcost <= 27'd0;
                                    end
                                    else begin
                                        q_pkind <= 2'd1;
                                        q_pnslot<= nslot_w;
                                        q_pcost <= cost_v;
                                        win_f   <= win_f + {19'd0, nslot_w};
                                        if (nslot_w != 2'd0)
                                            last_mark <= op_c1 + 21'd1;
                                    end
                                end
                                default: begin       // UPLOAD: fetch-bound, zero cost
                                    q_push  <= 1'b1;
                                    q_pkind <= 2'd0;
                                    q_pnslot<= 2'd0;
                                    q_pcost <= 27'd0;
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
                    q_mem[q_wp] <= {q_pkind, q_pnslot, q_pcost};
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
            o_dbg_cost <= q_pcost;

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
