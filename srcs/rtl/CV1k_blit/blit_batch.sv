`default_nettype none
//============================================================================
// blit_batch.sv - K=8-objline train batcher                     [H7a / I-4.3]
//
// Sits between blit_top's beat channels (H3 fixed-latency read contract,
// per-pixel masked writes) and a train-level memory port (CV1k_ddr3_harness on
// target, blit_port_beh in the trace TB).  Realizes the FINDINGS.md §5
// frozen contract: the engine's row-major beat stream is served from
// on-chip staging that is filled K=8 objlines at a time as ONE read train
// (burst order s0 d0 s1 d1 ... - one exposed latency), and the engine's
// writes are collected and flushed as ONE write train per batch - one
// R->W turnaround pair per K rows instead of per row.
//
// Port discipline (mirrors engine.h's DES exactly):
//   * The port is strictly serial: R(b0) W(b0) R(b1) W(b1) ... - a read
//     train is granted only when every buffered write has drained.  This
//     ONE rule realizes all orderings at once: the waitpipe cross-op RAW
//     interlock (flush-writes-first), the strict-op write-before-next-read
//     smear order, and the DES's W(b)->R(b+1) sequence.  No cross-op read
//     prefetch: batch formation starts at the descriptor (B_S3 commit),
//     the same information timing the DES was validated under.
//   * Because the port never overlaps trains, staging is single-buffered:
//     R(b+1) is only requested after the engine has consumed batch b, and
//     W(b) runs between them - there is no concurrent fill/drain window to
//     ping-pong across.  16 KB staging < the ~24 KB budget.
//
// Engine service (descriptor-driven counting): the descriptor fully
// determines the beat stream - rows x ceil(npx/4) src beats in row-major
// order (+ a dst beat per src beat when blending); blit_dsc_check proved
// every emitted beat lies inside this footprint over the whole corpus.
// The serve unit counts beats, translates addresses to staging offsets,
// and serves at full rate while the requested row is staged; a request
// beyond the fill (always a row-first beat: rows land atomically) parks
// in the ONE-outstanding-read slot and stalls the engine via i_rd_vld
// until the row lands.  Staging is pixel-lane organized: staged pixel p
// lives in lane p%4 at word p/4, so any unaligned 4-px beat reads in one
// cycle.
//
// Fallback: strict/px1 ops (self-overlap smear - the engine already
// serializes them beat-by-beat) and ops whose row exceeds staging
// capacity bypass staging entirely; each read is a 2-word port fetch and
// buffered writes drain before it (the universal rule again), preserving
// the golden sequential-smear feedback through the port.  Batch length
// adapts to capacity: L = max k<=8 with k rows fitting both stagings -
// the full corpus (worst row 322 px) always gets K=8.
//
// Write path: engine write beats (draw + upload, ascending didx within an
// op) are queued in a beat FIFO, coalesced into aligned 64-bit words with
// 4-px lane enables, and streamed as write trains whenever the read side
// is idle.  A beat is accepted exactly once: on the cycles where request,
// ready and rd_vld are all high - the same condition that advances the
// engine's B4 stage - so held requests during stalls never re-queue.
// Trains are framed for the harness's non-preemptive arbitration by
// o_rd_train/o_wr_train.
//============================================================================
module blit_batch #(
    parameter int unsigned SRC_CAP_W  = 1024,  // src staging, 64-bit words
    parameter int unsigned DST_CAP_W  = 1024,  // dst staging, 64-bit words
    parameter int unsigned K_ROWS     = 8,     // frozen objline batch size
    parameter int unsigned WF_LOG2    = 10     // write-beat FIFO depth log2
)(
    input  wire        i_CLK,
    input  wire        i_RST_n,

    //------------------------------------------------------------------
    // engine face (connects to blit_top's beat channels)
    //------------------------------------------------------------------
    input  wire        i_srd_req,
    input  wire [24:0] i_srd_addr,
    output wire [63:0] o_srd_data,
    input  wire        i_drd_req,
    input  wire [24:0] i_drd_addr,
    output wire [63:0] o_drd_data,
    output reg         o_rd_vld,       // -> blit_top i_rd_vld
    input  wire        i_wr_req,
    input  wire [24:0] i_wr_addr,
    input  wire [63:0] i_wr_data,
    input  wire [3:0]  i_wr_mask,
    output wire        o_wr_rdy,       // -> blit_top i_wr_rdy

    // descriptor sideband (from blit_top o_dsc_*)
    input  wire        i_dsc_vld,
    input  wire [12:0] i_dsc_sx_lo,
    input  wire [11:0] i_dsc_sy0,
    input  wire [12:0] i_dsc_rows,
    input  wire [13:0] i_dsc_npx,
    input  wire [31:0] i_dsc_dst0,
    input  wire        i_dsc_flipy,
    input  wire        i_dsc_blend,
    input  wire        i_dsc_strict,
    input  wire        i_dsc_px1,
    input  wire        i_dsc_wait,

    //------------------------------------------------------------------
    // train port (CV1k_ddr3_harness / blit_port_beh)
    //------------------------------------------------------------------
    output reg         o_prd_req,      // read burst command (one per segment)
    output wire [22:0] o_prd_addr,     // 64-bit-word address (flat px >> 2)
    output wire [10:0] o_prd_len,      // words in this burst
    input  wire        i_prd_rdy,
    input  wire        i_prd_dvld,     // in-order response word stream
    input  wire [63:0] i_prd_data,

    output reg         o_pwr_req,      // write word (posted)
    output reg  [22:0] o_pwr_addr,
    output reg  [63:0] o_pwr_data,
    output reg  [3:0]  o_pwr_be,       // pixel-lane enables (harness -> byte BE)
    input  wire        i_pwr_rdy,

    output wire        o_rd_train,     // train framing for non-preemptive arb
    output wire        o_wr_train,
    output wire        o_idle,         // fully drained (TB end-of-exec gate)

    // lateness-monitor taps (sim-only consumers; free to leave open)
    output reg         o_op_srv,       // 1-cycle: current op's last beat requested
    output wire        o_wr_idle       // write path empty (op-finish witness)
);

    localparam int unsigned SAW = $clog2(SRC_CAP_W);
    localparam int unsigned DAW = $clog2(DST_CAP_W);

    // ---------------------------------------------------------------------
    // descriptor FIFO (depth 4; the engine holds at most ~2 ops in flight).
    // H7b.8 Fmax respin, round 2.  The push-time decode stores only the
    // ADD-class geometry (span word counts, bpr, the sy0-pre-added src_lo0)
    // -- fit #3 showed a full push-time decode (incl. the fit-cap compare
    // tree) forms a one-cycle cone from DRAW's S-stage registers across the
    // hierarchy into the entry registers (-7.8).  The fit-cap products
    // (L / strict fold / first blen-after split) are computed from the head
    // in TWO forms:
    //   * lv_* comb  -- live, used ONLY for the c_*/sv_* register loads at
    //     ld_fire (a plain register-datain cone, no roll arithmetic);
    //   * hd_* regs  -- refreshed from lv_* every cycle, i.e. valid for any
    //     head that has existed for >= 1 cycle.  The serve/roll path uses
    //     hd_* exclusively: a read request can only coincide with ld_fire
    //     when the load was HELD BACK (parked read / active train), and a
    //     held load's head is aged by construction -- asserted below.
    // Values are bit-identical in every reachable case (both forms are the
    // same pure function of the stored entry).
    // ramstyle "logic": 4 entries read combinationally at the head -- as
    // registers the head mux is ~1 ns; inferred M10K costs ~3 ns + bypass.
    // ---------------------------------------------------------------------
    localparam int unsigned DSCW = 5 + 13 + 11 + 11 + 14 + 25 + 25;

    (* ramstyle = "logic" *) reg [DSCW-1:0] dq [0:3];
    reg [1:0]      dq_wp, dq_rp;
    reg [2:0]      dq_cnt;

    wire [DSCW-1:0] dhead     = dq[dq_rp];
    wire            dh_wait   = dhead[103];
    wire            dh_flipy  = dhead[102];
    wire            dh_blend  = dhead[101];
    wire            dh_strictr= dhead[100];       // raw dsc strict (pre-fold)
    wire            dh_px1    = dhead[99];
    wire [12:0]     dh_rows   = dhead[98:86];
    wire [10:0]     ld_snw    = dhead[85:75];
    wire [10:0]     ld_dnw    = dhead[74:64];
    wire [13:0]     ld_bpr    = dhead[63:50];
    wire [24:0]     ld_src_lo0= dhead[49:25];
    wire [24:0]     dh_dst0   = dhead[24:0];

    // push-time decode (comb from the i_dsc_* pulse; the descriptor fields
    // are draw's S-stage registers, stable through the pulse cycle).
    // Per-row word counts are op constants: row strides are 8192 px, so
    // every row's span alignment is identical.
    wire signed [14:0] pd_sxm3   = $signed({2'b00, i_dsc_sx_lo}) - 15'sd3;
    wire [24:0]        pd_srcoff = {{10{pd_sxm3[14]}}, pd_sxm3} & 25'h1FFFFFC;
    wire [1:0]         pd_sal    = pd_sxm3[1:0];
    wire [1:0]         pd_dal    = i_dsc_dst0[1:0];
    // src span = npx+3 px: the uniform sx_lo-3 underhang covers the
    // flip-mode beat-base adjust (costs <=1 extra word on non-flip rows)
    wire [10:0] pd_snw = 11'(({12'd0, pd_sal} + {1'b0, i_dsc_npx} + 15'd6) >> 2);
    wire [10:0] pd_dnw = 11'(({12'd0, pd_dal} + {1'b0, i_dsc_npx} + 15'd3) >> 2);

    // largest k <= 8 with k*nw <= cap (0 = one row does not fit -> strict)
    function automatic logic [3:0] f_fitcap(input logic [10:0] nw,
                                            input int unsigned cap);
        logic [13:0] c;
        c = 14'(cap);
        if      ({nw, 3'b000}                                  <= c) f_fitcap = 4'd8;
        else if ({1'b0, nw, 2'b00} + {2'b00, nw, 1'b0} + {3'b000, nw} <= c) f_fitcap = 4'd7;
        else if ({1'b0, nw, 2'b00} + {2'b00, nw, 1'b0}         <= c) f_fitcap = 4'd6;
        else if ({1'b0, nw, 2'b00} + {3'b000, nw}              <= c) f_fitcap = 4'd5;
        else if ({1'b0, nw, 2'b00}                             <= c) f_fitcap = 4'd4;
        else if ({2'b00, nw, 1'b0} + {3'b000, nw}              <= c) f_fitcap = 4'd3;
        else if ({2'b00, nw, 1'b0}                             <= c) f_fitcap = 4'd2;
        else if ({3'b000, nw}                                  <= c) f_fitcap = 4'd1;
        else                                                         f_fitcap = 4'd0;
    endfunction

    wire [13:0] pd_bpr  = i_dsc_px1 ? i_dsc_npx : 14'(({1'b0, i_dsc_npx} + 15'd3) >> 2);
    wire [24:0] pd_srclo0 = ({i_dsc_sy0, 13'd0} + pd_srcoff) & 25'h1FFFFFF;

    // head fit-cap products, live form (see the FIFO header note)
    wire [3:0] lv_fit_s = f_fitcap(ld_snw, SRC_CAP_W);
    wire [3:0] lv_fit_d = dh_blend ? f_fitcap(ld_dnw, DST_CAP_W) : 4'(K_ROWS);
    wire [3:0] lv_fit   = (lv_fit_s < lv_fit_d) ? lv_fit_s : lv_fit_d;
    wire [3:0] lv_L     = (lv_fit > 4'(K_ROWS)) ? 4'(K_ROWS) : lv_fit;
    wire       lv_strict= dh_strictr || (lv_L == 4'd0);
    wire [3:0]  lv_blen0  = ({9'd0, lv_L} < dh_rows) ? lv_L : dh_rows[3:0];
    wire [12:0] lv_after0 = ({9'd0, lv_L} < dh_rows)
                            ? (dh_rows - {9'd0, lv_L}) : 13'd0;

    // aged-head registered form + freshness (hd_* describe the current head
    // iff it already existed last cycle: no pop and no push-to-empty then)
    reg  [3:0]  hd_L;
    reg         hd_strict;
    reg  [3:0]  hd_blen0;
    reg  [12:0] hd_after0;
    reg         hd_fresh;

    // ---------------------------------------------------------------------
    // current op + serve state
    // ---------------------------------------------------------------------
    reg        cur_v;
    reg        sv_done;                 // all beats requested; op context must
                                        // outlive the parked last read + fill
    reg        c_flipy, c_blend, c_strict, c_px1, c_wait;
    reg [12:0] c_rows;
    reg [13:0] c_bpr;
    reg [10:0] c_snw, c_dnw;
    reg [3:0]  c_L;

    // serve counters: the (row, beat) the NEXT engine request belongs to
    reg [12:0] sv_row;
    reg [13:0] sv_beat;
    reg [3:0]  sv_slot;
    reg [3:0]  sv_blen;                 // rows in the current batch
    reg [12:0] sv_after;                // rows remaining beyond it
    reg [24:0] sv_src_lo;               // staged src span base (flat, aligned)
    reg [24:0] sv_dst_lo;               // dst row base (flat, unaligned)
    reg [12:0] sv_sbase, sv_dbase;      // staging px base of current slot

    // one-outstanding-read slot (the whole i_rd_vld protocol)
    reg        pend_v;
    reg        pend_drd;
    reg [12:0] pend_row;
    reg [12:0] pend_spx, pend_dpx;      // staging px index (batched resume)
    reg [24:0] pend_sa, pend_da;        // flat addrs (strict fetch)

    // fill progress / strict assembly (owned by the port block; read here)
    reg [12:0] fl_rows_abs;             // rows staged since op start
    reg        st_ready;                // strict fetch data assembled

    reg  [1:0]  sch;                    // port scheduler state
    localparam [1:0] S_IDLE = 2'd0, S_WR = 2'd1, S_RD = 2'd2;
    wire rd_active = (sch == S_RD);

    // load waits out a parked read / an active train so the previous op's
    // fill bookkeeping is never reset under it.  A pend at sv_done is always
    // on the op's LAST row, whose landing both resumes it and ends the
    // train, so this gate only ever delays the load by a couple of cycles -
    // well inside the engine's op-to-op request gap (the one residual
    // coincidence, load edge == new op's first request, is composed by the
    // effective context below).
    wire ld_fire = (!cur_v || sv_done) && (dq_cnt != 3'd0)
                   && !pend_v && !rd_active;

    // ---------------------------------------------------------------------
    // effective serve context: the op load (ld_fire) and the NEW op's first
    // read request can share one edge (the load is held back by the old
    // op's parked last read / closing train while the engine sprints ahead)
    // - the request must then be evaluated against the descriptor being
    // loaded, not the stale registers, and its roll updates override the
    // load's initializers (serve branch runs after the load branch).
    // ---------------------------------------------------------------------
    // fit-cap-derived terms come from the AGED registered form here: the
    // serve/roll consumers below only ever see ld_fire together with a
    // request when the load was held back, and a held head is aged (hd_*
    // valid) -- asserted in the serve branch.
    wire        e_strict = ld_fire ? hd_strict : c_strict;
    wire        e_flipy  = ld_fire ? dh_flipy  : c_flipy;
    wire [13:0] e_bpr    = ld_fire ? ld_bpr    : c_bpr;
    wire [12:0] e_rows   = ld_fire ? dh_rows   : c_rows;
    wire [10:0] e_snw    = ld_fire ? ld_snw    : c_snw;
    wire [10:0] e_dnw    = ld_fire ? ld_dnw    : c_dnw;
    wire [3:0]  e_L      = ld_fire ? hd_L      : c_L;
    wire [12:0] e_row    = ld_fire ? 13'd0     : sv_row;
    wire [13:0] e_beat   = ld_fire ? 14'd0     : sv_beat;
    wire [3:0]  e_slot   = ld_fire ? 4'd0      : sv_slot;
    wire [3:0]  e_blen   = ld_fire ? hd_blen0  : sv_blen;
    wire [12:0] e_after  = ld_fire ? hd_after0 : sv_after;
    wire [24:0] e_src_lo = ld_fire ? ld_src_lo0 : sv_src_lo;
    wire [24:0] e_dst_lo = ld_fire ? dh_dst0   : sv_dst_lo;
    wire [12:0] e_sbase  = ld_fire ? 13'd0     : sv_sbase;
    wire [12:0] e_dbase  = ld_fire ? 13'd0     : sv_dbase;
    wire [12:0] e_flabs  = ld_fire ? 13'd0     : fl_rows_abs;

    // load-branch form: fresh heads (pushed to an empty queue last edge)
    // take the live decode -- a plain register-datain cone
    wire        el_strict = hd_fresh ? hd_strict : lv_strict;
    wire [3:0]  el_L      = hd_fresh ? hd_L      : lv_L;
    wire [3:0]  el_blen0  = hd_fresh ? hd_blen0  : lv_blen0;
    wire [12:0] el_after0 = hd_fresh ? hd_after0 : lv_after0;

    wire [24:0] off_s = (i_srd_addr - e_src_lo) & 25'h1FFFFFF;
    wire [24:0] off_d = (i_drd_addr - {e_dst_lo[24:2], 2'b00}) & 25'h1FFFFFF;
    wire [12:0] px_s  = e_sbase + off_s[12:0];
    wire [12:0] px_d  = e_dbase + off_d[12:0];

    wire req_hit   = !e_strict && (e_row < e_flabs);
    wire resume_ok = pend_v && (c_strict ? st_ready
                                         : (fl_rows_abs > pend_row));

    // staging read strobes/addresses: live stream on hits, parked request
    // on resume (both sampled by the stage BRAMs at this edge)
    wire        s_ren = pend_v ? resume_ok : i_srd_req;
    wire        d_ren = pend_v ? (resume_ok && pend_drd) : i_drd_req;
    wire [12:0] s_rpx = pend_v ? pend_spx : px_s;
    wire [12:0] d_rpx = pend_v ? pend_dpx : px_d;

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            dq_wp <= 2'd0; dq_rp <= 2'd0; dq_cnt <= 3'd0;
            cur_v <= 1'b0; sv_done <= 1'b0;
            c_flipy <= 1'b0; c_blend <= 1'b0; c_strict <= 1'b0; c_px1 <= 1'b0;
            c_rows <= '0; c_bpr <= '0; c_snw <= '0; c_dnw <= '0;
            c_L <= '0;
            sv_row <= '0; sv_beat <= '0; sv_slot <= '0;
            sv_blen <= '0; sv_after <= '0;
            sv_src_lo <= '0; sv_dst_lo <= '0; sv_sbase <= '0; sv_dbase <= '0;
            pend_v <= 1'b0; pend_drd <= 1'b0; pend_row <= '0;
            pend_spx <= '0; pend_dpx <= '0; pend_sa <= '0; pend_da <= '0;
            hd_L <= '0; hd_strict <= 1'b0; hd_blen0 <= '0; hd_after0 <= '0;
            hd_fresh <= 1'b0;
            o_rd_vld <= 1'b1;
            o_op_srv <= 1'b0;
        end
        else begin
            o_op_srv <= 1'b0;
            // descriptor push (fields are valid during the pulse cycle)
            if (i_dsc_vld) begin
                dq[dq_wp] <= {i_dsc_wait, i_dsc_flipy, i_dsc_blend,
                              i_dsc_strict, i_dsc_px1, i_dsc_rows,
                              pd_snw, pd_dnw, pd_bpr,
                              pd_srclo0, i_dsc_dst0[24:0]};
                dq_wp  <= dq_wp + 2'd1;
                dq_cnt <= dq_cnt + 3'd1;
`ifndef SYNTHESIS
                if (dq_cnt == 3'd4)
                    $fatal(2, "[blit_batch] descriptor FIFO overflow t=%0t", $time);
`endif
            end

            if (ld_fire) begin
                cur_v    <= 1'b1;
                sv_done  <= 1'b0;
                c_flipy  <= dh_flipy;  c_blend <= dh_blend;
                c_strict <= el_strict; c_px1   <= dh_px1;
                c_wait   <= dh_wait;
                c_rows   <= dh_rows;   c_bpr   <= ld_bpr;
                c_snw    <= ld_snw;    c_dnw   <= ld_dnw;
                c_L      <= el_L;
                sv_row  <= '0; sv_beat <= '0; sv_slot <= '0;
                sv_blen <= el_blen0;
                sv_after<= el_after0;
                sv_src_lo <= ld_src_lo0;
                sv_dst_lo <= dh_dst0;
                sv_sbase  <= '0; sv_dbase <= '0;
                dq_rp  <= dq_rp + 2'd1;
                dq_cnt <= dq_cnt - 3'd1 + (i_dsc_vld ? 3'd1 : 3'd0);
            end

            // aged-head decode registers (see the FIFO header note)
            hd_L      <= lv_L;
            hd_strict <= lv_strict;
            hd_blen0  <= lv_blen0;
            hd_after0 <= lv_after0;
            hd_fresh  <= !ld_fire && !(i_dsc_vld && dq_cnt == 3'd0);

            // serve counting: every src-read request is one beat leaving B1
            // (evaluated in the e_* context so a load on this same edge
            // composes: the roll below overrides the load's initializers)
            if (i_srd_req) begin
`ifndef SYNTHESIS
                if ((!cur_v || sv_done) && !ld_fire)
                    $fatal(2, "[blit_batch] read request with no descriptor a=%07x cur_v=%b done=%b row=%0d beat=%0d rows=%0d dqc=%0d pend=%b t=%0t",
                           i_srd_addr, cur_v, sv_done, sv_row, sv_beat,
                           c_rows, dq_cnt, pend_v, $time);
                // roll-context invariant: a request coinciding with a load
                // means the load was held back, so the head must be aged
                // (the e_* fit-cap terms read hd_* -- see the FIFO header)
                if (ld_fire && !hd_fresh)
                    $fatal(2, "[blit_batch] compose-load with a fresh head t=%0t", $time);
                if (req_hit && (off_s[24:13] != 12'd0 ||
                                off_s[12:0] >= {e_snw, 2'b00}))
                    $fatal(2, "[blit_batch] src offset 0x%07x outside segment t=%0t",
                           off_s, $time);
                if (req_hit && i_drd_req && (off_d[24:13] != 12'd0 ||
                                off_d[12:0] >= {e_dnw, 2'b00}))
                    $fatal(2, "[blit_batch] dst offset 0x%07x outside segment t=%0t",
                           off_d, $time);
`endif
                if (!req_hit) begin
                    // park the one outstanding read; freeze the engine
                    o_rd_vld <= 1'b0;
                    pend_v   <= 1'b1;
                    pend_drd <= i_drd_req;
                    pend_row <= e_row;
                    pend_spx <= px_s;       pend_dpx <= px_d;
                    pend_sa  <= i_srd_addr; pend_da  <= i_drd_addr;
                end

                // roll the (row, beat) walk - mirrors the engine's B_BEAT
                if (e_beat == e_bpr - 14'd1) begin
                    sv_beat   <= '0;
                    sv_row    <= e_row + 13'd1;
                    sv_src_lo <= e_src_lo + (e_flipy ? -25'd8192 : 25'd8192);
                    sv_dst_lo <= e_dst_lo + 25'd8192;
                    if (e_row + 13'd1 == e_rows) begin
                        sv_done  <= 1'b1;       // op served; straggler writes
                                                // drain via the write path
                        o_op_srv <= 1'b1;
                    end
                    else if (e_slot == e_blen - 4'd1) begin
                        sv_slot  <= '0;
                        sv_sbase <= '0; sv_dbase <= '0;
                        sv_blen  <= ({9'd0, e_L} < e_after) ? e_L
                                                            : e_after[3:0];
                        sv_after <= ({9'd0, e_L} < e_after)
                                    ? (e_after - {9'd0, e_L}) : 13'd0;
                    end
                    else begin
                        sv_slot  <= e_slot + 4'd1;
                        sv_sbase <= e_sbase + {e_snw, 2'b00};
                        sv_dbase <= e_dbase + {e_dnw, 2'b00};
                    end
                end
                else sv_beat <= e_beat + 14'd1;
            end

            // resume: present the parked read (stage BRAMs sample s_rpx at
            // this edge -> data next cycle, rd_vld high the same cycle)
            if (resume_ok) begin
                pend_v   <= 1'b0;
                o_rd_vld <= 1'b1;
            end
        end
    end

    // rotation of the 4-px window, captured with each staging read
    reg [1:0] rot_s, rot_d;
    always_ff @(posedge i_CLK) begin
        if (s_ren) rot_s <= s_rpx[1:0];
        if (d_ren) rot_d <= d_rpx[1:0];
    end

    // ---------------------------------------------------------------------
    // staging BRAMs + strict assembly, engine-facing data muxes
    // ---------------------------------------------------------------------
    wire [63:0] stq_s, stq_d;
    reg  [12:0] fw_wptr_s, fw_wptr_d;    // fill word pointers (per batch)
    reg         fl_kind;                 // 0 src, 1 dst (segment being filled)
    reg         fl_strict;

    wire fill_batched = rd_active && i_prd_dvld && !fl_strict;
    wire fw_s_en = fill_batched && !fl_kind;
    wire fw_d_en = fill_batched &&  fl_kind;

    bb_stage #(.CAPW(SRC_CAP_W)) u_stage_src (
        .i_CLK     (i_CLK),
        .i_fw_en   (fw_s_en),
        .i_fw_addr (fw_wptr_s[SAW-1:0]),
        .i_fw_data (i_prd_data),
        .i_rd_en   (s_ren),
        .i_rd_px   (s_rpx[SAW+1:0]),
        .i_rot     (rot_s),
        .o_q       (stq_s)
    );
    bb_stage #(.CAPW(DST_CAP_W)) u_stage_dst (
        .i_CLK     (i_CLK),
        .i_fw_en   (fw_d_en),
        .i_fw_addr (fw_wptr_d[DAW-1:0]),
        .i_fw_data (i_prd_data),
        .i_rd_en   (d_ren),
        .i_rd_px   (d_rpx[DAW+1:0]),
        .i_rot     (rot_d),
        .o_q       (stq_d)
    );

    // strict-path assembly: src 2 words + dst 2 words, window-rotated
    reg [63:0]   st_w [0:3];
    reg [1:0]    st_srot, st_drot;
    wire [127:0] st_swin = {st_w[1], st_w[0]};
    wire [127:0] st_dwin = {st_w[3], st_w[2]};

    assign o_srd_data = c_strict ? st_swin[{st_srot, 4'd0} +: 64] : stq_s;
    assign o_drd_data = c_strict ? st_dwin[{st_drot, 4'd0} +: 64] : stq_d;

    // ---------------------------------------------------------------------
    // write path: beat FIFO -> aligned-word coalescer -> posted port words
    // ---------------------------------------------------------------------
    wire wf_push = i_wr_req && o_wr_rdy && o_rd_vld;   // exactly-once accept

`ifndef SYNTHESIS
    // temporary bring-up trace: +wtrace_lo/+wtrace_hi pixel-addr window
    int unsigned wt_lo = 1, wt_hi = 0;
    bit dsctrace = 0;
    initial begin
        void'($value$plusargs("wtrace_lo=%d", wt_lo));
        void'($value$plusargs("wtrace_hi=%d", wt_hi));
        void'($value$plusargs("dsctrace=%d", dsctrace));
    end
    always @(posedge i_CLK) begin
        if (wf_push && i_wr_addr >= 25'(wt_lo) && i_wr_addr <= 25'(wt_hi))
            $display("[wf_push] a=%07x m=%b d=%016x t=%0t",
                     i_wr_addr, i_wr_mask, i_wr_data, $time);
        if (dsctrace && i_dsc_vld)
            $display("[dsc] sxlo=%0d sy0=%0d rows=%0d npx=%0d dst0=%0d fy=%b bl=%b st=%b p1=%b t=%0t",
                     i_dsc_sx_lo, i_dsc_sy0, i_dsc_rows, i_dsc_npx,
                     i_dsc_dst0, i_dsc_flipy, i_dsc_blend, i_dsc_strict,
                     i_dsc_px1, $time);
    end
`endif
    wire wf_vld, wf_pop;
    wire [92:0] wf_head;
    wire [10:0] wf_cnt;

    bb_wfifo #(.LOG2(WF_LOG2), .W(93)) u_wfifo (
        .i_CLK   (i_CLK),
        .i_RST_n (i_RST_n),
        .i_push  (wf_push),
        .i_data  ({i_wr_addr, i_wr_mask, i_wr_data}),
        .i_pop   (wf_pop),
        .o_vld   (wf_vld),
        .o_head  (wf_head),
        .o_cnt   (wf_cnt)
    );
    assign o_wr_rdy = (wf_cnt < 11'((1 << WF_LOG2) - 8));

    wire [24:0] wh_a  = wf_head[92:68];
    wire [3:0]  wh_m  = wf_head[67:64];
    wire [63:0] wh_d  = wf_head[63:0];
    wire [22:0] wh_w0 = wh_a[24:2];
    wire [1:0]  wh_sh = wh_a[1:0];

    // beat split into the two words it straddles (sh=0 -> low word only)
    logic [63:0] lo_d, hi_d;
    logic [3:0]  lo_m, hi_m;
    always_comb begin
        lo_d = '0; hi_d = '0; lo_m = '0; hi_m = '0;
        for (int l = 0; l < 4; l++) begin
            automatic int p = int'(wh_sh) + l;      // word lane of beat lane l
            if (p < 4) begin
                lo_m[p] = wh_m[l];
                lo_d[p*16 +: 16] = wh_d[l*16 +: 16];
            end
            else begin
                hi_m[p-4] = wh_m[l];
                hi_d[(p-4)*16 +: 16] = wh_d[l*16 +: 16];
            end
        end
    end

    reg         co_v;                    // carry: the straddled upper word
    reg [22:0]  co_w;
    reg [63:0]  co_d;
    reg [3:0]   co_m;

    wire out_free   = !o_pwr_req || i_pwr_rdy;
    wire co_gap     = co_v && (co_w != wh_w0);
    assign wf_pop   = (sch == S_WR) && out_free && wf_vld && !co_gap;
    wire wr_side_ne = wf_vld || co_v || o_pwr_req;

    // merged low word: head beat lanes override the (older) carry lanes
    logic [63:0] mg_d;
    logic [3:0]  mg_m;
    always_comb begin
        mg_d = co_v ? co_d : '0;
        mg_m = co_v ? co_m : 4'd0;
        for (int l = 0; l < 4; l++)
            if (lo_m[l]) begin
                mg_d[l*16 +: 16] = lo_d[l*16 +: 16];
                mg_m[l] = 1'b1;
            end
    end

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            co_v <= 1'b0; co_w <= '0; co_d <= '0; co_m <= '0;
            o_pwr_req <= 1'b0; o_pwr_addr <= '0;
            o_pwr_data <= '0; o_pwr_be <= '0;
        end
        else if (out_free) begin
            o_pwr_req <= 1'b0;
            if (sch == S_WR) begin
                if (wf_vld) begin
                    if (co_gap) begin                // flush the gapped carry
                        o_pwr_req  <= 1'b1;
                        o_pwr_addr <= co_w;
                        o_pwr_data <= co_d;
                        o_pwr_be   <= co_m;
                        co_v       <= 1'b0;          // head pops next cycle
                    end
                    else begin                       // emit merged low word
                        o_pwr_req  <= (mg_m != 4'd0);
                        o_pwr_addr <= wh_w0;
                        o_pwr_data <= mg_d;
                        o_pwr_be   <= mg_m;
                        co_v <= (hi_m != 4'd0);
                        co_w <= wh_w0 + 23'd1;
                        co_d <= hi_d;
                        co_m <= hi_m;
                    end
                end
                else if (co_v) begin                 // drain the final carry
                    o_pwr_req  <= 1'b1;
                    o_pwr_addr <= co_w;
                    o_pwr_data <= co_d;
                    o_pwr_be   <= co_m;
                    co_v       <= 1'b0;
                end
            end
        end
    end

    // ---------------------------------------------------------------------
    // read-train side: batch prefetch walker + strict fetcher + scheduler
    // ---------------------------------------------------------------------
    // batch trigger: staging is free exactly when the serve walk has
    // consumed every row before the next batch (sv_row can overshoot pf_row
    // by the one parked request, hence >=)
    reg [12:0] pf_row;                  // next batch's first row
    reg [24:0] pf_src_lo, pf_dst_lo;    // command address walk
    wire [12:0] pf_rem  = c_rows - pf_row;
    wire [3:0]  pf_blen = ({9'd0, c_L} < pf_rem) ? c_L : pf_rem[3:0];
    // waitpipe (cross-op RAW): the previous op's tail writes can still sit
    // in the ENGINE's pixel pipe at op load, invisible to the write FIFO.
    // The engine only emits a wait-op's first beat after draining its pipe
    // (B_ROW gate), so that beat's request is the proof every prior write
    // has been accepted - hold batch 0 until it appears; the write-drain-
    // before-read rule then orders the port.  Non-wait ops carry no
    // overlap (engine hazard test), so their batch 0 prefetches freely.
    wire pf_go0  = !c_wait || pend_v || (sv_row != 13'd0) || (sv_beat != 14'd0);
    wire pf_want = cur_v && !c_strict && (pf_row != c_rows)
                   && (sv_row >= pf_row)
                   && ((pf_row != 13'd0) || pf_go0);
    wire st_want = cur_v && c_strict && pend_v && !st_ready && !rd_active;

    // in-train walkers (command issue and data landing run independently;
    // commands pipeline ahead so the port exposes one latency per train)
    reg        rc_kind;                 // 0 src, 1 dst
    reg [3:0]  rc_row, rc_rows;
    reg        rc_strict;
    reg        rc_stword;               // strict: 0 src cmd, 1 dst cmd

    reg [3:0]  fl_row, fl_rows;
    reg [10:0] fl_cnt;                  // words left in the current segment
    reg        fl_stw;                  // strict: 0 src words, 1 dst words

    assign o_prd_addr = rc_strict ? (rc_stword ? pend_da[24:2]
                                               : pend_sa[24:2])
                                  : (rc_kind ? pf_dst_lo[24:2]
                                             : pf_src_lo[24:2]);
    assign o_prd_len  = rc_strict ? 11'd2 : (rc_kind ? c_dnw : c_snw);

    assign o_rd_train = rd_active;
    assign o_wr_train = (sch == S_WR);
    assign o_wr_idle  = !wr_side_ne;
    assign o_idle = (!cur_v || sv_done) && (dq_cnt == 3'd0) && !pend_v
                    && (sch == S_IDLE) && !wr_side_ne;

    // stat taps (step-4 lateness monitor / debug)
    longint unsigned n_rd_trains /*verilator public_flat_rd*/;
    longint unsigned n_wr_trains /*verilator public_flat_rd*/;
    longint unsigned n_rd_words  /*verilator public_flat_rd*/;
    longint unsigned n_wr_words  /*verilator public_flat_rd*/;
    longint unsigned n_strict_fet/*verilator public_flat_rd*/;

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            sch <= S_IDLE;
            o_prd_req <= 1'b0;
            rc_kind <= 1'b0; rc_row <= '0; rc_rows <= '0;
            rc_strict <= 1'b0; rc_stword <= 1'b0;
            fl_kind <= 1'b0; fl_row <= '0; fl_rows <= '0; fl_cnt <= '0;
            fl_strict <= 1'b0; fl_stw <= 1'b0;
            fl_rows_abs <= '0;
            pf_row <= '0; pf_src_lo <= '0; pf_dst_lo <= '0;
            fw_wptr_s <= '0; fw_wptr_d <= '0;
            st_ready <= 1'b0;
            st_srot <= 2'd0; st_drot <= 2'd0;
            st_w[0] <= '0; st_w[1] <= '0; st_w[2] <= '0; st_w[3] <= '0;
            n_rd_trains <= 0; n_wr_trains <= 0;
            n_rd_words <= 0; n_wr_words <= 0; n_strict_fet <= 0;
        end
        else begin
            // per-op init of the prefetch walk (same edge as the op load)
            if (ld_fire) begin
                pf_row      <= '0;
                pf_src_lo   <= ld_src_lo0;
                pf_dst_lo   <= dh_dst0;
                fl_rows_abs <= '0;
            end

            // strict resume consumed: re-arm for the next parked beat
            if (resume_ok && c_strict)
                st_ready <= 1'b0;

            case (sch)
            S_IDLE: begin
                if (wr_side_ne)
                    sch <= S_WR;
                else if (st_want) begin
                    sch <= S_RD;
                    rc_strict <= 1'b1; rc_stword <= 1'b0;
                    fl_strict <= 1'b1; fl_stw <= 1'b0; fl_cnt <= 11'd2;
                    st_srot <= pend_sa[1:0];
                    st_drot <= pend_da[1:0];
                    o_prd_req <= 1'b1;
                    n_strict_fet <= n_strict_fet + 1;
                    n_rd_trains  <= n_rd_trains + 1;
                end
                else if (pf_want) begin
                    sch <= S_RD;
                    rc_strict <= 1'b0; rc_kind <= 1'b0;
                    rc_row <= '0; rc_rows <= pf_blen;
                    fl_strict <= 1'b0; fl_kind <= 1'b0;
                    fl_row <= '0; fl_rows <= pf_blen;
                    fl_cnt <= c_snw;
                    fw_wptr_s <= '0; fw_wptr_d <= '0;
                    pf_row <= pf_row + {9'd0, pf_blen};
                    o_prd_req <= 1'b1;
                    n_rd_trains <= n_rd_trains + 1;
                end
            end

            S_WR: begin
                if (o_pwr_req && i_pwr_rdy)
                    n_wr_words <= n_wr_words + 1;
                if (!wr_side_ne) begin
                    sch <= S_IDLE;
                    n_wr_trains <= n_wr_trains + 1;
                end
            end

            S_RD: begin
                // command issue walk
                if (o_prd_req && i_prd_rdy) begin
                    if (rc_strict) begin
                        if (!rc_stword && pend_drd)
                            rc_stword <= 1'b1;
                        else
                            o_prd_req <= 1'b0;
                    end
                    else begin
                        if (!rc_kind && c_blend)
                            rc_kind <= 1'b1;
                        else begin
                            rc_kind   <= 1'b0;
                            pf_src_lo <= pf_src_lo +
                                         (c_flipy ? -25'd8192 : 25'd8192);
                            pf_dst_lo <= pf_dst_lo + 25'd8192;
                            if (rc_row == rc_rows - 4'd1)
                                o_prd_req <= 1'b0;
                            else rc_row <= rc_row + 4'd1;
                        end
                    end
                end

                // response landing walk
                if (i_prd_dvld) begin
                    n_rd_words <= n_rd_words + 1;
                    if (fl_strict) begin
                        st_w[{fl_stw, ~fl_cnt[1]}] <= i_prd_data;
                        if (fl_cnt == 11'd1) begin
                            if (!fl_stw && pend_drd) begin
                                fl_stw <= 1'b1;
                                fl_cnt <= 11'd2;
                            end
                            else begin
                                st_ready <= 1'b1;
                                sch      <= S_IDLE;
                            end
                        end
                        else fl_cnt <= fl_cnt - 11'd1;
                    end
                    else begin
`ifndef SYNTHESIS
                        if (fl_kind  && fw_wptr_d >= 13'(DST_CAP_W))
                            $fatal(2, "[blit_batch] dst staging overrun t=%0t", $time);
                        if (!fl_kind && fw_wptr_s >= 13'(SRC_CAP_W))
                            $fatal(2, "[blit_batch] src staging overrun t=%0t", $time);
`endif
                        if (fl_kind) fw_wptr_d <= fw_wptr_d + 13'd1;
                        else         fw_wptr_s <= fw_wptr_s + 13'd1;
                        if (fl_cnt == 11'd1) begin
                            if (!fl_kind && c_blend) begin
                                fl_kind <= 1'b1;
                                fl_cnt  <= c_dnw;
                            end
                            else begin
                                fl_kind <= 1'b0;
                                fl_cnt  <= c_snw;
                                fl_rows_abs <= fl_rows_abs + 13'd1;
                                if (fl_row == fl_rows - 4'd1)
                                    sch <= S_IDLE;
                                else fl_row <= fl_row + 4'd1;
                            end
                        end
                        else fl_cnt <= fl_cnt - 11'd1;
                    end
                end
            end

            default: sch <= S_IDLE;
            endcase

`ifndef SYNTHESIS
            if (i_prd_dvld && sch != S_RD)
                $fatal(2, "[blit_batch] stray port read data t=%0t", $time);
`endif
        end
    end

endmodule

//============================================================================
// bb_stage - one staging buffer, pixel-lane organization.  Staged pixel p
// lives in lane p%4 at word address p/4; the port fill writes one aligned
// 4-px word to all lanes at one address, the engine read fetches any
// unaligned 4-px window in one cycle (each lane at its own address) and
// rotates lanes back into beat order.  Read is enable-gated so the output
// holds between requests (the H3 hold contract).
//============================================================================
module bb_stage #(
    parameter int unsigned CAPW = 1024
)(
    input  wire                      i_CLK,
    input  wire                      i_fw_en,
    input  wire [$clog2(CAPW)-1:0]   i_fw_addr,
    input  wire [63:0]               i_fw_data,
    input  wire                      i_rd_en,
    input  wire [$clog2(CAPW)+1:0]   i_rd_px,
    input  wire [1:0]                i_rot,   // registered at the same edge
    output wire [63:0]               o_q
);
    localparam int unsigned AW = $clog2(CAPW);

    reg [15:0] lane0 [0:CAPW-1];
    reg [15:0] lane1 [0:CAPW-1];
    reg [15:0] lane2 [0:CAPW-1];
    reg [15:0] lane3 [0:CAPW-1];

    // lane m holds the pixel at px + ((m - px%4) & 3)
    wire [AW+1:0] pm0 = i_rd_px + {{AW{1'b0}}, (2'd0 - i_rd_px[1:0])};
    wire [AW+1:0] pm1 = i_rd_px + {{AW{1'b0}}, (2'd1 - i_rd_px[1:0])};
    wire [AW+1:0] pm2 = i_rd_px + {{AW{1'b0}}, (2'd2 - i_rd_px[1:0])};
    wire [AW+1:0] pm3 = i_rd_px + {{AW{1'b0}}, (2'd3 - i_rd_px[1:0])};

    reg [15:0] q0, q1, q2, q3;

    always_ff @(posedge i_CLK) begin
        if (i_fw_en) begin
            lane0[i_fw_addr] <= i_fw_data[15:0];
            lane1[i_fw_addr] <= i_fw_data[31:16];
            lane2[i_fw_addr] <= i_fw_data[47:32];
            lane3[i_fw_addr] <= i_fw_data[63:48];
        end
        if (i_rd_en) begin
            q0 <= lane0[pm0[AW+1:2]];
            q1 <= lane1[pm1[AW+1:2]];
            q2 <= lane2[pm2[AW+1:2]];
            q3 <= lane3[pm3[AW+1:2]];
        end
    end

    // beat lane l = staged pixel px+l = physical lane (rot+l)&3
    wire [15:0] qv [0:3];
    assign qv[0] = q0; assign qv[1] = q1; assign qv[2] = q2; assign qv[3] = q3;
    assign o_q = { qv[2'((i_rot + 2'd3))],
                   qv[2'((i_rot + 2'd2))],
                   qv[2'((i_rot + 2'd1))],
                   qv[i_rot] };

endmodule

//============================================================================
// bb_wfifo - BRAM first-word-fall-through FIFO (write-beat queue)
//============================================================================
module bb_wfifo #(
    parameter int unsigned LOG2 = 10,
    parameter int unsigned W    = 93
)(
    input  wire          i_CLK,
    input  wire          i_RST_n,
    input  wire          i_push,
    input  wire [W-1:0]  i_data,
    input  wire          i_pop,
    output reg           o_vld,
    output reg  [W-1:0]  o_head,
    output wire [10:0]   o_cnt
);
    localparam int unsigned DEPTH = 1 << LOG2;

    reg [W-1:0]    mem [0:DEPTH-1];
    reg [LOG2-1:0] wp, rp;
    reg [LOG2:0]   cnt;                  // entries in mem (head not counted)

    assign o_cnt = 11'(cnt) + {10'd0, o_vld};

    wire refill = (!o_vld || i_pop) && (cnt != '0);

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            wp <= '0; rp <= '0; cnt <= '0;
            o_vld <= 1'b0;
        end
        else begin
            if (i_push) begin
                mem[wp] <= i_data;
                wp <= wp + 1'b1;
            end
`ifndef SYNTHESIS
            if (i_push && cnt == (LOG2+1)'(DEPTH))
                $fatal(2, "[bb_wfifo] overflow t=%0t", $time);
`endif
            if (refill) begin
                o_head <= mem[rp];
                rp <= rp + 1'b1;
            end
            o_vld <= refill || (o_vld && !i_pop);
            cnt <= cnt + ((LOG2+1)'(i_push ? 1 : 0))
                       - ((LOG2+1)'(refill ? 1 : 0));
        end
    end

endmodule
`default_nettype none
