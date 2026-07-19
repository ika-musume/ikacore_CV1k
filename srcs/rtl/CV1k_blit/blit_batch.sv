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
    input  wire        i_srd_req,      // r4: sim oracle only -- the serve
    input  wire [24:0] i_srd_addr,     //  logic consumes the LOCAL rebuild
    output wire [63:0] o_srd_data,     //  from the raw legs below
    input  wire        i_drd_req,
    input  wire [24:0] i_drd_addr,
    output wire [63:0] o_drd_data,
    output reg         o_rd_vld,       // -> blit_top i_rd_vld
    input  wire        i_wr_req,
    input  wire [24:0] i_wr_addr,
    input  wire [63:0] i_wr_data,
    input  wire [3:0]  i_wr_mask,
    output wire        o_wr_rdy,       // -> blit_top i_wr_rdy
    // r4: raw request legs (blit_top o_rq_* + o_steal).  The request ANDs
    // are rebuilt HERE -- b1_v && !(draw_wr && !(wr_rdy && !steal)) &&
    // rd_vld -- from register-launched inputs, so the o_af almost-full
    // register no longer times a batch->draw->batch double crossing to
    // reach the serve/roll cone (sv_*).  Cycle-identical to i_srd_req /
    // i_drd_req by construction; the oracle below re-proves it every run.
    input  wire        i_rq_v,
    input  wire        i_rq_wr,
    input  wire        i_rq_blend,
    input  wire        i_steal,

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
    // H7b.8 Fmax respin, round 2 + H7b.8e round 3.  The dq ENTRY stores
    // only the ADD-class geometry (span word counts, bpr, the sy0-pre-added
    // src_lo0) -- the H7b.8 fit #3 lesson stands: folding the fit-cap tree
    // into the ENTRY registers formed a one-cycle cone from DRAW's S-stage
    // registers across the hierarchy into the wide datain fan (-7.8).
    // H7b.8e moves the fit-cap chain to push time DIFFERENTLY: the results
    // land in 4-bit PARALLEL side registers per slot (pf_L / pf_st below),
    // so the dq datain fan is untouched and the head decode lv_L/lv_strict
    // becomes a slot-register read.  lv_blen0/lv_after0 keep the live
    // compare/subtract tail off pf_L + the dh_rows slice.  The aged hd_*
    // copies and the compose invariant are unchanged:
    //   * lv_* -- used ONLY for the c_*/sv_* register loads at ld_fire;
    //   * hd_* regs -- refreshed every cycle (r6: the AGED subset lands
    //     one cycle behind the fields, via the as2 bank; the settle+1
    //     steer covers the gap), valid for any head that has existed for
    //     >= 1 cycle; the serve/roll path uses hd_*/ah_* exclusively
    //     (a request can only coincide with ld_fire when the load was
    //     HELD BACK, and a held load's head is aged -- asserted below).
    // Values are bit-identical in every reachable case (all forms are the
    // same pure function of the descriptor fields).
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
    wire            dh_strictr= dhead[100];       // raw dsc strict (pre-fold;
                                                   // folded at push since 8e)
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

    // largest k <= 8 with k*nw <= cap (0 = one row does not fit -> strict).
    // H7b.8c strength reduction: k*nw <= cap  <=>  nw <= floor(cap/k) for
    // positive integers (k*floor(cap/k) <= cap covers the always-true side),
    // and cap is a localparam constant -- so the old shifted-adder compare
    // tree collapses to eight PARALLEL constant-threshold compares plus a
    // priority encode.  Bit-exact for every nw/cap.
    function automatic logic [3:0] f_fitcap(input logic [10:0] nw,
                                            input int unsigned cap);
        logic [13:0] nwx;
        nwx = {3'b000, nw};
        if      (nwx <= 14'(cap / 8)) f_fitcap = 4'd8;
        else if (nwx <= 14'(cap / 7)) f_fitcap = 4'd7;
        else if (nwx <= 14'(cap / 6)) f_fitcap = 4'd6;
        else if (nwx <= 14'(cap / 5)) f_fitcap = 4'd5;
        else if (nwx <= 14'(cap / 4)) f_fitcap = 4'd4;
        else if (nwx <= 14'(cap / 3)) f_fitcap = 4'd3;
        else if (nwx <= 14'(cap / 2)) f_fitcap = 4'd2;
        else if (nwx <= 14'(cap / 1)) f_fitcap = 4'd1;
        else                          f_fitcap = 4'd0;
    endfunction

    wire [13:0] pd_bpr  = i_dsc_px1 ? i_dsc_npx : 14'(({1'b0, i_dsc_npx} + 15'd3) >> 2);
    wire [24:0] pd_srclo0 = ({i_dsc_sy0, 13'd0} + pd_srcoff) & 25'h1FFFFFF;

    // head fit-cap products, live form (see the FIFO header note)
    // H7b.8e: the two f_fitcap encodes are computed ONCE at push time (off
    // the registered i_dsc_* descriptor taps, in the push window) and
    // stored per slot -- the live head decode keeps only the min/cap/
    // subtract tail, so the dq read no longer feeds the compare trees.
    // Values identical: pd_snw/pd_dnw/i_dsc_blend are exactly what the
    // stored ld_snw/ld_dnw/dh_blend would decode to.  (The H7b.8 "full
    // decode at push" regression moved the WHOLE chain into the dq datain
    // fan; this is the front half only, in parallel registers.)
    reg  [3:0] pf_L  [0:3];
    reg        pf_st [0:3];
    // r4 iter2: the fit-cap compare trees move to push+1.  The pulse edge
    // captures only the folded span words + flags into pfp_* (the cross-
    // module cone from DRAW's S-regs now ends in plain capture registers),
    // and the trees run one cycle later off batch-local registers into the
    // slot regs.  The earliest possible consume of a pushed entry is
    // push+1 (dq_cnt is a register, so ld_fire sees the entry no sooner) --
    // exactly the cycle the write-through bypass below covers.  Values and
    // consumption instants are IDENTICAL to the push-time form.
    reg         pfp_v;
    reg  [1:0]  pfp_slot;
    reg  [10:0] pfp_snw, pfp_dnw;
    reg         pfp_blend, pfp_strictr;
    reg  [12:0] pfp_rows;                // for the pfp-aware hd_* aging
    wire [3:0] pfp_fit_s = f_fitcap(pfp_snw, SRC_CAP_W);
    wire [3:0] pfp_fit_d = pfp_blend ? f_fitcap(pfp_dnw, DST_CAP_W)
                                     : 4'(K_ROWS);
    wire [3:0] pfp_fit   = (pfp_fit_s < pfp_fit_d) ? pfp_fit_s : pfp_fit_d;
    wire [3:0] pfp_L     = (pfp_fit > 4'(K_ROWS)) ? 4'(K_ROWS) : pfp_fit;
    wire       pfp_strict= pfp_strictr || (pfp_L == 4'd0);
    // blen/after forms of the pfp entry (its own rows): feed ONLY the
    // hd_* aging registers below, so the compare tree has a full cycle
    // (r5 fit-#6: the chained blen0/after0 forms moved to the ap_* one-hot
    // candidates below -- see the threshold-algebra note)
    // r4 iter4: no write-through bypass -- ld_fire is HELD one cycle while
    // the head's fit-cap results are still in flight (pfp_v on dq_rp), so
    // lv_* are plain slot-register reads.  The hold only engages on a
    // push-to-the-head-slot (the elastic op-arrival case), and hd_fresh
    // is extended so the aged-head registers are never trusted while
    // stale.  r6: the hold stays ONE cycle -- a load (or composed
    // request) on the settle+1 cycle is STEERED to the as2_ bank below
    // instead of hd_* (the engine's first request can trail the push by
    // exactly 2 cycles, so the load instant is part of the contract; the
    // r6 anchor abort proved a 2-cycle hold breaks it).
    wire       pfp_head = pfp_v && (pfp_slot == dq_rp);
    // r7: registered exact next-values of pfp_head / pfp_head2.  The live
    // compares sat at the ld_fire root and in every steer select, putting
    // pfp_slot/dq_rp routing under the whole e_* fan (the r6 ship -2.12
    // pfp_slot->pend_* family).  Both operand sets move only at known
    // registered sites (push loads pfp_v/pfp_slot/pfp_slot2, ld_fire moves
    // dq_rp), so the flags fold those updates; oracle below.
    reg        ph_q, ph2_q;
    wire [3:0] lv_L     = pf_L[dq_rp];
    wire       lv_strict= pf_st[dq_rp];

    // r5 fit-#5: lt0/diff0/blen01 (and blen0/after0) aging forms.  Three
    // lessons folded in: the 13-bit subtract behind the fit-cap encode
    // was serial (-2.64, fit #3); a for-loop if(sel==k) select is a
    // 9-deep PRIORITY cascade (-3.52, fit #4); and the LV arm's launch
    // is the dq_rp-indexed dhead mux in front of the same math (-2.61,
    // fit #5).  r6 split (probe: compares+one-hot+candidate plane in one
    // cycle is a 7-LUT-level cone):
    //   * settle cycle: THRESHOLD algebra one-hot ONLY --
    //       L >= k  <=>  (snw <= cap_s/k) && (!blend || dnw <= cap_d/k)
    //       (f_fitcap's own inequalities; k <= K so the K-clamp cannot
    //        change any ge_k);  Lh[k] = ge[k] & !ge[k+1], Lh[8] = ge[8]
    //     -- registered into as2_lh + staged rows (a 4-level cone).
    //   * settle+1: the AND-OR candidate plane (f_ag_* / a2_* below) runs
    //     off those registers into the per-slot stores and the hd_ arm,
    //     and the steer serves a coinciding load from the same wires.
    //   * LV arm: the SAME values stored per slot beside pf_L/pf_st
    //     (pf_lt0/diff0/blen01/blen0/after0) -- the live read is a plain
    //     4:1 slot-register mux.
    // The sim oracles re-prove Lh == (pfp_L == k) at every settled push
    // and the hd_* coherence relations every cycle.
    logic [8:0] ag_ge;
    always_comb begin
        ag_ge[0] = 1'b1;
        for (int k = 1; k <= 8; k++)
            ag_ge[k] = ({3'b000, pfp_snw} <= 14'(SRC_CAP_W / k))
                    && (!pfp_blend ||
                        ({3'b000, pfp_dnw} <= 14'(DST_CAP_W / k)));
    end
    logic [8:0] ag_lh;
    always_comb begin
        for (int k = 0; k < 8; k++)
            ag_lh[k] = ag_ge[k] && !ag_ge[k+1];
        ag_lh[8] = ag_ge[8];
    end
    // r6 fit-#1: the full one-hot candidate plane behind the compare trees
    // is a 7-LUT-level cone (probe: 2 compare + 1 combine + 1 lh + 3
    // mask/OR) -- register the ONE-HOT + staged rows instead, and run the
    // AND-OR plane on the CONSUME side off those registers (3-4 levels).
    // Same pure functions of (lh, rows); the elaboration self-check below
    // proves the candidate algebra over the whole (L, rows) domain.
    function automatic logic f_ag_lt0(input logic [8:0] lh,
                                      input logic [12:0] rows);
        f_ag_lt0 = 1'b0;
        for (int k = 0; k <= 8; k++)
            f_ag_lt0 |= lh[k] && (13'(2*k) < rows);
    endfunction
    function automatic logic [12:0] f_ag_diff0(input logic [8:0] lh,
                                               input logic [12:0] rows);
        f_ag_diff0 = '0;
        for (int k = 0; k <= 8; k++)
            f_ag_diff0 |= lh[k] ? ((13'(k) < rows) ? 13'(rows - 13'(2*k))
                                                   : 13'(-k))
                                : 13'd0;
    endfunction
    function automatic logic f_ag_blen01(input logic [8:0] lh,
                                         input logic [12:0] rows);
        f_ag_blen01 = 1'b0;
        for (int k = 0; k <= 8; k++)
            f_ag_blen01 |= lh[k] && ((13'(k) < rows) ? (k == 1)
                                                     : (rows == 13'd1));
    endfunction
    function automatic logic [3:0] f_ag_blen0(input logic [8:0] lh,
                                              input logic [12:0] rows);
        f_ag_blen0 = '0;
        for (int k = 0; k <= 8; k++)
            f_ag_blen0 |= lh[k] ? ((13'(k) < rows) ? 4'(k) : rows[3:0])
                                : 4'd0;
    endfunction
    function automatic logic [12:0] f_ag_after0(input logic [8:0] lh,
                                                input logic [12:0] rows);
        f_ag_after0 = '0;
        for (int k = 0; k <= 8; k++)
            f_ag_after0 |= lh[k] ? ((13'(k) < rows) ? 13'(rows - 13'(k))
                                                    : 13'd0)
                                 : 13'd0;
    endfunction
    // r6 aging stage: the compare trees end in THIS capture bank at push+1
    // (one-hot level + staged rows -- a 4-level cone; they no longer reach
    // the pf_ slot banks or the hd_ compose mux in one cycle, the r5 ship
    // -2.60/-2.40 families); the AND-OR candidate plane (a2_* wires below)
    // runs off the bank on the CONSUME side at push+2 into the pf_ writes,
    // the hd_ pfp arm, and the aged-arm steer muxes.  A load or composed
    // request on the settle+1 cycle itself (pfp_head2) reads the plane
    // directly -- every leg register-fed -- so every load/pop/request
    // INSTANT is identical to the pre-r6 schedule (no elasticity, nothing
    // deferred).
    reg         pfp_v2;
    reg  [1:0]  pfp_slot2;
    reg  [3:0]  as2_L;
    reg         as2_strict;
    reg  [8:0]  as2_lh;      // settled one-hot fit level (lh[k] == (L == k))
    reg  [12:0] as2_rows;    // staged pfp_rows (oracle form)
    // r7: the per-candidate values are REGISTERED at the settle edge (nine
    // parallel compare/select cones off pfp_rows, each ending here); the
    // consume-side plane below is then a pure one-hot AND-OR over
    // registered bits instead of compare+select+mask -- the r6 ship
    // -2.22/-2.08 as2_rows->sr_* families lose the compare/subtract
    // levels and the 13-bit rows fan-in.  Same candidate algebra as the
    // f_ag_* functions; the r7 oracle re-proves every a2_* value.
    reg         as2_clt  [0:8];
    reg  [12:0] as2_cdiff[0:8];
    reg         as2_cbl01[0:8];
    reg  [3:0]  as2_cbl0 [0:8];
    reg  [12:0] as2_caft [0:8];
    wire pfp_head2 = pfp_v2 && (pfp_slot2 == dq_rp);
    logic        a2_lt0, a2_blen01;
    logic [12:0] a2_diff0, a2_after0;
    logic [3:0]  a2_blen0;
    always_comb begin
        a2_lt0 = 1'b0; a2_blen01 = 1'b0;
        a2_diff0 = '0; a2_after0 = '0; a2_blen0 = '0;
        for (int k = 0; k <= 8; k++) begin
            a2_lt0    |= as2_lh[k] && as2_clt[k];
            a2_blen01 |= as2_lh[k] && as2_cbl01[k];
            a2_diff0  |= as2_lh[k] ? as2_cdiff[k] : 13'd0;
            a2_after0 |= as2_lh[k] ? as2_caft[k]  : 13'd0;
            a2_blen0  |= as2_lh[k] ? as2_cbl0[k]  : 4'd0;
        end
    end

    // per-slot stored aging results (written at the settle+1 cycle)
    reg        pf_lt0    [0:3];
    reg [12:0] pf_diff0  [0:3];
    reg        pf_blen01 [0:3];
    reg [3:0]  pf_blen0  [0:3];
    reg [12:0] pf_after0 [0:3];

`ifndef SYNTHESIS
    // r5: exhaustive proof of the single-level aging algebra (see the
    // hd_lt0/hd_diff0/hd_blen01 note in the always block) over the whole
    // (L, rows) domain -- compiled into every sim build.
    initial begin
        for (int Li = 0; Li < 16; Li++)
            for (int Ri = 0; Ri < 8192; Ri++) begin
                automatic logic        ltr = (Li < Ri);
                automatic logic [3:0]  bl0 = ltr ? 4'(Li) : Ri[3:0];
                automatic logic [12:0] af0 = ltr ? 13'(Ri - Li) : 13'd0;
                if ((13'({Li, 1'b0}) < 13'(Ri)) !== (13'(Li) < af0))
                    $fatal(2, "[blit_batch] r5 lt0 algebra L=%0d R=%0d", Li, Ri);
                if ((ltr ? 13'(Ri - 2*Li) : 13'(-Li)) !== 13'(af0 - Li))
                    $fatal(2, "[blit_batch] r5 diff0 algebra L=%0d R=%0d", Li, Ri);
                if ((ltr ? (Li == 1) : (Ri == 1)) !== (bl0 == 4'd1))
                    $fatal(2, "[blit_batch] r5 blen01 algebra L=%0d R=%0d", Li, Ri);
            end
    end
`endif

    // aged-head registered form + freshness (hd_* describe the current head
    // iff it already existed last cycle: no pop and no push-to-empty then)
    reg  [3:0]  hd_L;
    reg         hd_strict;
    // r4 iter6: the aged bank covers EVERY head field -- the serve/load
    // composes read ONLY registers, and the dq_rp-indexed dhead mux fans
    // solely into this aging capture (a registered consumer).  Validity
    // at any ld_fire follows from the same freshness proof (entry written
    // at push, head reads from push+1, loads gated to >= push+2).
    reg         hd_wait, hd_flipy, hd_blend, hd_px1;
    reg  [12:0] hd_rows;
    reg  [10:0] hd_snw, hd_dnw;
    reg  [13:0] hd_bpr;
    reg  [24:0] hd_srclo0, hd_dst0;
    reg  [3:0]  hd_blen0;
    reg  [12:0] hd_after0;
    reg         hd_fresh;
    // r5 serve-roll restage: aged decision flags / next-values of the head
    // (the ld_fire arm of the e_* selects below reads ONLY these registers;
    // each is the same pure function of the descriptor as the live form)
    reg         hd_bpr1;       // (hd_bpr  == 1): beat_last of a fresh row 0
    reg         hd_rows1;      // (hd_rows == 1): row_last  of a fresh row 0
    reg         hd_blen01;     // (hd_blen0 == 1): slot_last of a fresh slot 0
    reg         hd_lt0;        // ({0,hd_L} < hd_after0)
    reg  [12:0] hd_diff0;      // hd_after0 - hd_L (mod 2^13)
    reg  [24:0] hd_srclo0_nx;  // hd_srclo0 -+ 8192 (hd_flipy)
    reg  [24:0] hd_dst0_nx;    // hd_dst0 + 8192

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

    // r5 serve-roll restage: the roll's branch decisions and rolled values
    // are pre-computed into registers, maintained at EVERY assignment site
    // of their base registers (the r4 gov nslot recipe).  The request-edge
    // consumers below read only ld_fire + these flags -- the compares,
    // subtracts and adds all end at these registers instead of feeding the
    // sv_* sload cones.  The sim oracle re-proves flag == retired formula
    // every cur_v cycle.
    reg        sr_beat_last;   // == (sv_beat == c_bpr - 1)
    reg        sr_row_last;    // == (sv_row + 1 == c_rows)
    reg        sr_slot_last;   // == (sv_slot == sv_blen - 1)
    reg        sr_lt;          // == ({0,c_L} < sv_after)
    reg [12:0] sr_diff;        // == sv_after - c_L   (mod 2^13)
    reg [24:0] sr_srclo_nx;    // == sv_src_lo -+ 8192 (c_flipy)
    reg [24:0] sr_dstlo_nx;    // == sv_dst_lo + 8192
    reg [12:0] sr_sbase_nx;    // == sv_sbase + {c_snw,2'b00}
    reg [12:0] sr_dbase_nx;    // == sv_dbase + {c_dnw,2'b00}
    reg        c_bpr1;         // == (c_bpr == 1), an op constant

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
    wire unused_dh = dh_strictr;
    wire ld_fire = (!cur_v || sv_done) && (dq_cnt != 3'd0)
                   && !pend_v && !rd_active && !ph_q;

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
    // r6 steer: on the settle+1 cycle (pfp_head2) the aged hd_* registers
    // are one write behind the as2_ bank, so the AGED terms read the bank
    // directly there.  3:1 per bit, every leg register-fed, both selects
    // shallow -- same LUT level as the old 2:1.  Field terms never need
    // the steer (dhead aging already lands them at push+1).
    wire        ah_strict = ph2_q ? as2_strict : hd_strict;
    wire [3:0]  ah_L      = ph2_q ? as2_L      : hd_L;
    wire [3:0]  ah_blen0  = ph2_q ? a2_blen0   : hd_blen0;
    wire [12:0] ah_after0 = ph2_q ? a2_after0  : hd_after0;
    wire        ah_blen01 = ph2_q ? a2_blen01  : hd_blen01;
    wire        ah_lt0    = ph2_q ? a2_lt0     : hd_lt0;
    wire [12:0] ah_diff0  = ph2_q ? a2_diff0   : hd_diff0;
    wire        e_strict = ld_fire ? ah_strict : c_strict;
    wire        e_flipy  = ld_fire ? hd_flipy  : c_flipy;
    wire [13:0] e_bpr    = ld_fire ? hd_bpr    : c_bpr;
    wire [12:0] e_rows   = ld_fire ? hd_rows   : c_rows;
    wire [10:0] e_snw    = ld_fire ? hd_snw    : c_snw;
    wire [10:0] e_dnw    = ld_fire ? hd_dnw    : c_dnw;
    wire [3:0]  e_L      = ld_fire ? ah_L      : c_L;
    wire [12:0] e_row    = ld_fire ? 13'd0     : sv_row;
    wire [13:0] e_beat   = ld_fire ? 14'd0     : sv_beat;
    wire [3:0]  e_slot   = ld_fire ? 4'd0      : sv_slot;
    wire [3:0]  e_blen   = ld_fire ? ah_blen0  : sv_blen;
    wire [12:0] e_after  = ld_fire ? ah_after0 : sv_after;
    wire [12:0] e_flabs  = ld_fire ? 13'd0     : fl_rows_abs;

    // r5 restage: composed forms of the pre-computed decisions/next-values
    // (the ld_fire arm is the aged head bank -- valid by the same freshness
    // proof as hd_* -- and the roll arm is the maintained sr_* bank)
    wire        e_beat_last = ld_fire ? hd_bpr1       : sr_beat_last;
    wire        e_row_last  = ld_fire ? hd_rows1      : sr_row_last;
    wire        e_slot_last = ld_fire ? ah_blen01     : sr_slot_last;
    wire        e_lt        = ld_fire ? ah_lt0        : sr_lt;
    wire [12:0] e_diff      = ld_fire ? ah_diff0      : sr_diff;
    wire [24:0] e_srclo_nx  = ld_fire ? hd_srclo0_nx  : sr_srclo_nx;
    wire [24:0] e_dstlo_nx  = ld_fire ? hd_dst0_nx    : sr_dstlo_nx;
    wire [12:0] e_sbase_nx  = ld_fire ? {hd_snw,2'b00}: sr_sbase_nx;
    wire [12:0] e_dbase_nx  = ld_fire ? {hd_dnw,2'b00}: sr_dbase_nx;
    wire        e_bpr1      = ld_fire ? hd_bpr1       : c_bpr1;

    // r4 iter5: the load branch reads the aged registers DIRECTLY -- the
    // pfp gate + pfp-aware aging guarantee hd_fresh at every ld_fire (the
    // compose-load $fatal re-proves it each run), so the old el_* fresh
    // fallback (hd_fresh ? hd_* : lv_*) was dead logic whose lv arm put
    // the dq_rp-indexed subtract cone into the load path.

    // r4 iter3: the offset subtract and base add are DISTRIBUTED over the
    // ld_fire select (the gov X/Y clamp recipe): both arms run in parallel
    // with ld_fire's AND instead of mux -> 25b subtract -> 13b add in
    // series into the staging-RAM address registers.  The load arm's base
    // is the constant 0, so its add folds away.  Exact: identical
    // arithmetic on each arm of the same 2:1 select.
    wire [24:0] off_s_ld = (i_srd_addr - hd_srclo0) & 25'h1FFFFFF;
    wire [24:0] off_s_c  = (i_srd_addr - sv_src_lo ) & 25'h1FFFFFF;
    wire [24:0] off_d_ld = (i_drd_addr - {hd_dst0[24:2],  2'b00}) & 25'h1FFFFFF;
    wire [24:0] off_d_c  = (i_drd_addr - {sv_dst_lo[24:2], 2'b00}) & 25'h1FFFFFF;
    wire [24:0] off_s = ld_fire ? off_s_ld : off_s_c;
    wire [24:0] off_d = ld_fire ? off_d_ld : off_d_c;
    wire [12:0] px_s  = ld_fire ? off_s_ld[12:0] : (sv_sbase + off_s_c[12:0]);
    wire [12:0] px_d  = ld_fire ? off_d_ld[12:0] : (sv_dbase + off_d_c[12:0]);

    wire req_hit   = !e_strict && (e_row < e_flabs);
    wire resume_ok = pend_v && (c_strict ? st_ready
                                         : (fl_rows_abs > pend_row));

    // r4: local rebuild of the engine requests from the raw legs -- the
    // exact expression blit_top composes for blit_draw's adv (steal-gated
    // wr_rdy, rd_vld) ANDed with the b1 stage flags.  wf_af / o_rd_vld
    // are THIS module's registers; i_rq_* / i_steal are register taps.
    wire eng_wr_rdy = !wf_af && !i_steal;
    wire srd_req_l  = i_rq_v && !(i_rq_wr && !eng_wr_rdy) && o_rd_vld;
    wire drd_req_l  = srd_req_l && i_rq_blend;

`ifndef SYNTHESIS
    // r4 oracle: the rebuild must match the engine-side requests exactly
    always @(posedge i_CLK) begin
        if (i_RST_n) begin
            if (srd_req_l !== i_srd_req)
                $fatal(2, "[blit_batch] r4 srd rebuild diverged (l=%b eng=%b) t=%0t",
                       srd_req_l, i_srd_req, $time);
            if (drd_req_l !== i_drd_req)
                $fatal(2, "[blit_batch] r4 drd rebuild diverged (l=%b eng=%b) t=%0t",
                       drd_req_l, i_drd_req, $time);
        end
    end

    // r5 oracle: the serve-roll restage banks must equal the retired live
    // formulas on every cycle with a loaded op (they are maintained at
    // every assignment site of their base registers, so any divergence is
    // a maintenance bug -- fail fast, not at the next wrong roll).
    always @(posedge i_CLK) begin
        if (i_RST_n && cur_v) begin
            if (sr_beat_last !== (sv_beat == c_bpr - 14'd1))
                $fatal(2, "[blit_batch] r5 sr_beat_last diverged t=%0t", $time);
            if (sr_row_last !== (sv_row + 13'd1 == c_rows))
                $fatal(2, "[blit_batch] r5 sr_row_last diverged t=%0t", $time);
            if (sr_slot_last !== (sv_slot == sv_blen - 4'd1))
                $fatal(2, "[blit_batch] r5 sr_slot_last diverged t=%0t", $time);
            if (sr_lt !== ({9'd0, c_L} < sv_after))
                $fatal(2, "[blit_batch] r5 sr_lt diverged t=%0t", $time);
            if (sr_diff !== 13'(sv_after - {9'd0, c_L}))
                $fatal(2, "[blit_batch] r5 sr_diff diverged t=%0t", $time);
            if (sr_srclo_nx !== 25'(sv_src_lo + (c_flipy ? -25'd8192
                                                         :  25'd8192)))
                $fatal(2, "[blit_batch] r5 sr_srclo_nx diverged t=%0t", $time);
            if (sr_dstlo_nx !== 25'(sv_dst_lo + 25'd8192))
                $fatal(2, "[blit_batch] r5 sr_dstlo_nx diverged t=%0t", $time);
            if (sr_sbase_nx !== 13'(sv_sbase + {c_snw, 2'b00}))
                $fatal(2, "[blit_batch] r5 sr_sbase_nx diverged t=%0t", $time);
            if (sr_dbase_nx !== 13'(sv_dbase + {c_dnw, 2'b00}))
                $fatal(2, "[blit_batch] r5 sr_dbase_nx diverged t=%0t", $time);
            if (c_bpr1 !== (c_bpr == 14'd1))
                $fatal(2, "[blit_batch] r5 c_bpr1 diverged t=%0t", $time);
        end
    end

    // r5: aged-flag coherence -- the per-candidate select must agree with
    // the base aged registers it summarizes (all load from the same
    // selected source every cycle, so this is an invariant, not a timing
    // window).  r6: EXCEPT the settle+1 cycle (pfp_head2) on a push to
    // the head slot -- the field registers already aged the pushed entry
    // while the as2 bank has not reached the aged registers yet.  Nothing
    // reads hd_* there (loads/requests are steered to as2_*), so the one
    // documented mixed cycle is skipped.
    always @(posedge i_CLK) begin
        if (i_RST_n && !pfp_head2) begin
            if (hd_lt0 !== ({9'd0, hd_L} < hd_after0))
                $fatal(2, "[blit_batch] r5 hd_lt0 incoherent t=%0t", $time);
            if (hd_diff0 !== 13'(hd_after0 - {9'd0, hd_L}))
                $fatal(2, "[blit_batch] r5 hd_diff0 incoherent t=%0t", $time);
            if (hd_blen01 !== (hd_blen0 == 4'd1))
                $fatal(2, "[blit_batch] r5 hd_blen01 incoherent t=%0t", $time);
            if (hd_after0 !== (({9'd0, hd_L} < hd_rows)
                               ? (hd_rows - {9'd0, hd_L}) : 13'd0))
                $fatal(2, "[blit_batch] r5 hd_after0 incoherent t=%0t", $time);
            if (hd_blen0 !== (({9'd0, hd_L} < hd_rows) ? hd_L : hd_rows[3:0]))
                $fatal(2, "[blit_batch] r5 hd_blen0 incoherent t=%0t", $time);
        end
    end

    // r5 fit-#4: the threshold one-hot must agree with the encoded pfp_L
    // at every settled push (re-proves the ge_k algebra against f_fitcap
    // on live descriptors, on top of its derivation)
    always @(posedge i_CLK) begin
        if (i_RST_n && pfp_v) begin
            for (int k = 0; k < 8; k++)
                if ((ag_ge[k] && !ag_ge[k+1]) !== (pfp_L == 4'(k)))
                    $fatal(2, "[blit_batch] r5 ag_lh[%0d] != (pfp_L==%0d) (L=%0d) t=%0t",
                           k, k, pfp_L, $time);
            if (ag_ge[8] !== (pfp_L == 4'd8))
                $fatal(2, "[blit_batch] r5 ag_lh[8] != (pfp_L==8) (L=%0d) t=%0t",
                       pfp_L, $time);
        end
    end

    // r7 oracle: the registered head flags track the live compares (gated
    // on the first push -- before it the pfp_v/pfp_v2 pipeline still holds
    // its power-up X for a cycle or two)
    reg r7_ph_seen = 1'b0;
    always @(posedge i_CLK) begin
        if (i_RST_n && i_dsc_vld) r7_ph_seen <= 1'b1;
        if (i_RST_n && r7_ph_seen && (ph_q !== pfp_head || ph2_q !== pfp_head2))
            $fatal(2, "[blit_batch] r7 ph_q/ph2_q diverged (%b%b vs %b%b) t=%0t",
                   ph_q, ph2_q, pfp_head, pfp_head2, $time);
    end

    // r7 oracle: the registered-candidate plane must equal the f_ag_*
    // algebra on (as2_lh, as2_rows) from the first settle onward (the
    // whole bank is written together, so it is coherent from then on)
    reg r7_as2_seen = 1'b0;
    always @(posedge i_CLK) begin
        if (i_RST_n && pfp_v) r7_as2_seen <= 1'b1;
        if (i_RST_n && r7_as2_seen) begin
            if (a2_lt0    !== f_ag_lt0   (as2_lh, as2_rows) ||
                a2_diff0  !== f_ag_diff0 (as2_lh, as2_rows) ||
                a2_blen01 !== f_ag_blen01(as2_lh, as2_rows) ||
                a2_blen0  !== f_ag_blen0 (as2_lh, as2_rows) ||
                a2_after0 !== f_ag_after0(as2_lh, as2_rows))
                $fatal(2, "[blit_batch] r7 candidate plane diverged t=%0t", $time);
        end
    end
`endif

    // staging read strobes/addresses: live stream on hits, parked request
    // on resume (both sampled by the stage BRAMs at this edge)
    wire        s_ren = pend_v ? resume_ok : srd_req_l;
    wire        d_ren = pend_v ? (resume_ok && pend_drd) : drd_req_l;
    wire [12:0] s_rpx = pend_v ? pend_spx : px_s;
    wire [12:0] d_rpx = pend_v ? pend_dpx : px_d;

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            dq_wp <= 2'd0; dq_rp <= 2'd0; dq_cnt <= 3'd0;
            ph_q <= 1'b0; ph2_q <= 1'b0;
            cur_v <= 1'b0; sv_done <= 1'b0;
            c_flipy <= 1'b0; c_blend <= 1'b0; c_strict <= 1'b0; c_px1 <= 1'b0;
            c_rows <= '0; c_bpr <= '0; c_snw <= '0; c_dnw <= '0;
            c_L <= '0;
            sv_row <= '0; sv_beat <= '0; sv_slot <= '0;
            sv_blen <= '0; sv_after <= '0;
            sv_src_lo <= '0; sv_dst_lo <= '0; sv_sbase <= '0; sv_dbase <= '0;
            sr_beat_last <= 1'b0; sr_row_last <= 1'b0; sr_slot_last <= 1'b0;
            sr_lt <= 1'b0; sr_diff <= '0;
            sr_srclo_nx <= '0; sr_dstlo_nx <= '0;
            sr_sbase_nx <= '0; sr_dbase_nx <= '0;
            c_bpr1 <= 1'b0;
            hd_bpr1 <= 1'b0; hd_rows1 <= 1'b0; hd_blen01 <= 1'b0;
            hd_lt0 <= 1'b0; hd_diff0 <= '0;
            hd_srclo0_nx <= '0; hd_dst0_nx <= '0;
            pend_v <= 1'b0; pend_drd <= 1'b0; pend_row <= '0;
            pend_spx <= '0; pend_dpx <= '0; pend_sa <= '0; pend_da <= '0;
            hd_L <= '0; hd_strict <= 1'b0; hd_blen0 <= '0; hd_after0 <= '0;
            hd_wait <= 1'b0; hd_flipy <= 1'b0; hd_blend <= 1'b0; hd_px1 <= 1'b0;
            hd_rows <= '0; hd_snw <= '0; hd_dnw <= '0; hd_bpr <= '0;
            hd_srclo0 <= '0; hd_dst0 <= '0;
            hd_fresh <= 1'b0;
            o_rd_vld <= 1'b1;
            o_op_srv <= 1'b0;
        end
        else begin
            o_op_srv <= 1'b0;
            pfp_v    <= 1'b0;
            // descriptor push (fields are valid during the pulse cycle;
            // r4 iter2: the fit-cap trees run at push+1 -- see pfp_* above)
            if (i_dsc_vld) begin
                dq[dq_wp] <= {i_dsc_wait, i_dsc_flipy, i_dsc_blend,
                              i_dsc_strict, i_dsc_px1, i_dsc_rows,
                              pd_snw, pd_dnw, pd_bpr,
                              pd_srclo0, i_dsc_dst0[24:0]};
                pfp_v      <= 1'b1;
                pfp_slot   <= dq_wp;
                pfp_snw    <= pd_snw;
                pfp_dnw    <= pd_dnw;
                pfp_blend  <= i_dsc_blend;
                pfp_strictr<= i_dsc_strict;
                pfp_rows   <= i_dsc_rows;
                dq_wp  <= dq_wp + 2'd1;
                dq_cnt <= dq_cnt + 3'd1;
`ifndef SYNTHESIS
                if (dq_cnt == 3'd4)
                    $fatal(2, "[blit_batch] descriptor FIFO overflow t=%0t", $time);
                if (pfp_v)
                    $fatal(2, "[blit_batch] r4 dsc pulses on adjacent cycles t=%0t", $time);
`endif
            end
            // r7: head-compare flags, exact next-values (see the decl note).
            // dq_rp moves only at ld_fire; the push side loads pfp_v/
            // pfp_slot (and, one cycle on, pfp_v2/pfp_slot2) -- when the
            // valid arm is 0 the slot compare is masked, so the unconditional
            // operand read is exact.
            ph_q  <= i_dsc_vld && (dq_wp   == (ld_fire ? dq_rp + 2'd1 : dq_rp));
            ph2_q <= pfp_v     && (pfp_slot == (ld_fire ? dq_rp + 2'd1 : dq_rp));
            // r6 aging stage: the settle cycle captures the algebra into
            // the as2_ bank; the slot stores happen one cycle later off
            // plain registers.  Pushes are never adjacent (assert above),
            // so the bank is always consumed before it can be reloaded.
            pfp_v2 <= pfp_v;
            if (pfp_v) begin
                pfp_slot2  <= pfp_slot;
                as2_L      <= pfp_L;
                as2_strict <= pfp_strict;
                as2_lh     <= ag_lh;
                as2_rows   <= pfp_rows;
                // r7 candidate bank (see the decl note; same algebra as
                // f_ag_*, cut at a register instead of at the plane)
                for (int k = 0; k <= 8; k++) begin
                    as2_clt  [k] <= (13'(2*k) < pfp_rows);
                    as2_cdiff[k] <= (13'(k) < pfp_rows)
                                    ? 13'(pfp_rows - 13'(2*k)) : 13'(-k);
                    as2_cbl01[k] <= (13'(k) < pfp_rows)
                                    ? (k == 1) : (pfp_rows == 13'd1);
                    as2_cbl0 [k] <= (13'(k) < pfp_rows) ? 4'(k)
                                                        : pfp_rows[3:0];
                    as2_caft [k] <= (13'(k) < pfp_rows)
                                    ? 13'(pfp_rows - 13'(k)) : 13'd0;
                end
            end
            if (pfp_v2) begin
                pf_L [pfp_slot2] <= as2_L;
                pf_st[pfp_slot2] <= as2_strict;
                pf_lt0   [pfp_slot2] <= a2_lt0;
                pf_diff0 [pfp_slot2] <= a2_diff0;
                pf_blen01[pfp_slot2] <= a2_blen01;
                pf_blen0 [pfp_slot2] <= a2_blen0;
                pf_after0[pfp_slot2] <= a2_after0;
            end

            if (ld_fire) begin
                cur_v    <= 1'b1;
                sv_done  <= 1'b0;
                c_flipy  <= hd_flipy;  c_blend <= hd_blend;
                c_strict <= ah_strict; c_px1   <= hd_px1;
                c_wait   <= hd_wait;
                c_rows   <= hd_rows;   c_bpr   <= hd_bpr;
                c_snw    <= hd_snw;    c_dnw   <= hd_dnw;
                c_L      <= ah_L;
                sv_row  <= '0; sv_beat <= '0; sv_slot <= '0;
                sv_blen <= ah_blen0;
                sv_after<= ah_after0;
                sv_src_lo <= hd_srclo0;
                sv_dst_lo <= hd_dst0;
                sv_sbase  <= '0; sv_dbase <= '0;
                // r5 restage bank load (aged forms of the same functions;
                // a composed request this edge overrides in the serve branch)
                sr_beat_last <= hd_bpr1;
                sr_row_last  <= hd_rows1;
                sr_slot_last <= ah_blen01;
                sr_lt        <= ah_lt0;
                sr_diff      <= ah_diff0;
                sr_srclo_nx  <= hd_srclo0_nx;
                sr_dstlo_nx  <= hd_dst0_nx;
                sr_sbase_nx  <= {hd_snw, 2'b00};
                sr_dbase_nx  <= {hd_dnw, 2'b00};
                c_bpr1       <= hd_bpr1;
                dq_rp  <= dq_rp + 2'd1;
                dq_cnt <= dq_cnt - 3'd1 + (i_dsc_vld ? 3'd1 : 3'd0);
            end

            // aged-head decode registers (see the FIFO header note).
            // r6: the aged arms consume the as2 bank one cycle after the
            // settle (pfp_head2) -- the lv_*/pf_* slot reads would be one
            // cycle stale there.  During the settle+1 cycle itself the hd
            // AGED registers still show the previous slot content while
            // the FIELD registers already show the pushed entry; nothing
            // reads hd_* on that cycle (a load or composed request is
            // steered to as2_* directly, see the e_*/c_* arms), and the
            // coherence oracle skips exactly that documented window.
            hd_L      <= ph2_q ? as2_L      : lv_L;
            hd_strict <= ph2_q ? as2_strict : lv_strict;
            hd_blen0  <= ph2_q ? a2_blen0   : pf_blen0[dq_rp];
            hd_after0 <= ph2_q ? a2_after0  : pf_after0[dq_rp];
            hd_wait   <= dh_wait;      // entry fields: written at push, so
            hd_flipy  <= dh_flipy;     // plain aging suffices (no pfp arm)
            hd_blend  <= dh_blend;
            hd_px1    <= dh_px1;
            hd_rows   <= dh_rows;
            hd_snw    <= ld_snw;
            hd_dnw    <= ld_dnw;
            hd_bpr    <= ld_bpr;
            hd_srclo0 <= ld_src_lo0;
            hd_dst0   <= dh_dst0;
            // r5 restage: aged decision flags / next-values (same sources
            // and refresh cadence as their base hd_* registers above).
            // fit-#2: lt0/diff0 computed THROUGH after0 chained a second
            // subtract behind the fit-cap tree (-2.66) -- rewritten to
            // single-level algebra on (L, rows) directly:
            //   lt0   == (L < after0) == (2L < rows)      [all cases]
            //   diff0 == after0 - L   == (L < rows) ? rows - 2L : -L
            //   blen01== (blen0 == 1) == (L < rows) ? (L == 1) : (rows == 1)
            // (mod-2^13 arithmetic matches the retired forms bit-for-bit;
            // the elaboration self-check below proves all three over the
            // full (L, rows) domain every sim build.)
            hd_bpr1      <= (ld_bpr  == 14'd1);
            hd_rows1     <= (dh_rows == 13'd1);
            hd_blen01    <= ph2_q ? a2_blen01 : pf_blen01[dq_rp];
            hd_lt0       <= ph2_q ? a2_lt0    : pf_lt0[dq_rp];
            hd_diff0     <= ph2_q ? a2_diff0  : pf_diff0[dq_rp];
            hd_srclo0_nx <= dh_flipy ? (ld_src_lo0 - 25'd8192)
                                     : (ld_src_lo0 + 25'd8192);
            hd_dst0_nx   <= dh_dst0 + 25'd8192;
            hd_fresh  <= !ld_fire && !(i_dsc_vld && dq_cnt == 3'd0)
                                  && !pfp_head;

            // serve counting: every src-read request is one beat leaving B1
            // (evaluated in the e_* context so a load on this same edge
            // composes: the roll below overrides the load's initializers)
            if (srd_req_l) begin
`ifndef SYNTHESIS
                if ((!cur_v || sv_done) && !ld_fire)
                    $fatal(2, "[blit_batch] read request with no descriptor a=%07x cur_v=%b done=%b row=%0d beat=%0d rows=%0d dqc=%0d pend=%b t=%0t",
                           i_srd_addr, cur_v, sv_done, sv_row, sv_beat,
                           c_rows, dq_cnt, pend_v, $time);
                // roll-context invariant: a request coinciding with a load
                // means the load was held back, so the head must be aged
                // (the e_* fit-cap terms read hd_* -- see the FIFO header)
                // roll-context invariant: with the pfp-aware aging, hd_*
                // are valid at every possible ld_fire cycle (a pop-cycle
                // or push-to-empty is never followed by an eligible load;
                // the settle cycle is gated, and a settle+1 load reads
                // the as2 bank through the r6 steer) -- re-proven forever
                if (ld_fire && !hd_fresh && !pfp_head2)
                    $fatal(2, "[blit_batch] compose-load with a fresh head t=%0t", $time);
                if (req_hit && (off_s[24:13] != 12'd0 ||
                                off_s[12:0] >= {e_snw, 2'b00}))
                    $fatal(2, "[blit_batch] src offset 0x%07x outside segment t=%0t",
                           off_s, $time);
                if (req_hit && drd_req_l && (off_d[24:13] != 12'd0 ||
                                off_d[12:0] >= {e_dnw, 2'b00}))
                    $fatal(2, "[blit_batch] dst offset 0x%07x outside segment t=%0t",
                           off_d, $time);
`endif
                if (!req_hit) begin
                    // park the one outstanding read; freeze the engine
                    o_rd_vld <= 1'b0;
                    pend_v   <= 1'b1;
                    pend_drd <= drd_req_l;
                    pend_row <= e_row;
                    pend_spx <= px_s;       pend_dpx <= px_d;
                    pend_sa  <= i_srd_addr; pend_da  <= i_drd_addr;
                end

                // roll the (row, beat) walk - mirrors the engine's B_BEAT.
                // r5 restage: every branch decision and rolled value is a
                // registered pre-compute (e_* selects of the sr_/hd_ banks);
                // the live arithmetic here only MAINTAINS the banks -- each
                // compare/add ends at an sr_* register, never in the sv_*
                // load cones.  Values and instants identical by the oracle.
                if (e_beat_last) begin
                    sv_beat      <= '0;
                    sr_beat_last <= e_bpr1;
                    sv_row       <= e_row + 13'd1;
                    sr_row_last  <= (e_row + 13'd2 == e_rows);
                    sv_src_lo    <= e_srclo_nx;
                    sr_srclo_nx  <= e_flipy ? (e_srclo_nx - 25'd8192)
                                            : (e_srclo_nx + 25'd8192);
                    sv_dst_lo    <= e_dstlo_nx;
                    sr_dstlo_nx  <= e_dstlo_nx + 25'd8192;
                    if (e_row_last) begin
                        sv_done  <= 1'b1;       // op served; straggler writes
                                                // drain via the write path
                        o_op_srv <= 1'b1;
                    end
                    else if (e_slot_last) begin
                        sv_slot  <= '0;
                        sr_slot_last <= e_lt ? (e_L == 4'd1)
                                             : (e_after[3:0] == 4'd1);
                        sv_sbase <= '0; sv_dbase <= '0;
                        sr_sbase_nx <= {e_snw, 2'b00};
                        sr_dbase_nx <= {e_dnw, 2'b00};
                        sv_blen  <= e_lt ? e_L : e_after[3:0];
                        sv_after <= e_lt ? e_diff : 13'd0;
                        sr_lt    <= e_lt && ({9'd0, e_L} < e_diff);
                        sr_diff  <= (e_lt ? e_diff : 13'd0) - {9'd0, e_L};
                    end
                    else begin
                        sv_slot  <= e_slot + 4'd1;
                        sr_slot_last <= ({1'b0, e_slot} + 5'd2 ==
                                         {1'b0, e_blen});
                        sv_sbase <= e_sbase_nx;
                        sr_sbase_nx <= e_sbase_nx + {e_snw, 2'b00};
                        sv_dbase <= e_dbase_nx;
                        sr_dbase_nx <= e_dbase_nx + {e_dnw, 2'b00};
                    end
                end
                else begin
                    sv_beat      <= e_beat + 14'd1;
                    sr_beat_last <= (e_beat + 14'd2 == e_bpr);
                end
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

    wire wf_af, wf_af2;
    bb_wfifo #(.LOG2(WF_LOG2), .W(93),
               .AF_TH((1 << WF_LOG2) - 8)) u_wfifo (
        .i_CLK   (i_CLK),
        .i_RST_n (i_RST_n),
        .i_push  (wf_push),
        .i_data  ({i_wr_addr, i_wr_mask, i_wr_data}),
        .i_pop   (wf_pop),
        .o_vld   (wf_vld),
        .o_head  (wf_head),
        .o_cnt   (wf_cnt),
        .o_af    (wf_af),
        .o_af2   (wf_af2)
    );
    // H7b.8e: registered almost-full, timing-identical to the old comb
    // compare (exact next-value inside bb_wfifo).  r7: the EXPORT reads
    // the o_af2 placement duplicate -- the engine-side adv/pop fan and
    // the batch-local eng_wr_rdy rebuild no longer share one launch.
    assign o_wr_rdy = !wf_af2;

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
                pf_src_lo   <= hd_srclo0;
                pf_dst_lo   <= hd_dst0;
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
    parameter int unsigned W    = 93,
    parameter int unsigned AF_TH = 0    // o_af = (count >= AF_TH); 0 = unused
)(
    input  wire          i_CLK,
    input  wire          i_RST_n,
    input  wire          i_push,
    input  wire [W-1:0]  i_data,
    input  wire          i_pop,
    output reg           o_vld,
    output reg  [W-1:0]  o_head,
    output wire [10:0]   o_cnt,
    (* preserve *) output reg o_af,    // registered almost-full (H7b.8e)
    (* preserve *) output reg o_af2    // r7: placement duplicate of o_af
                                        // (same next-value; (* preserve *)
                                        // keeps it a separate register so
                                        // the exported wr_rdy fan and the
                                        // batch-local rebuild fan can be
                                        // placed independently -- the r7
                                        // farm's o_af->fetch-flag crossing)
);
    localparam int unsigned DEPTH = 1 << LOG2;

    reg [W-1:0]    mem [0:DEPTH-1];
    reg [LOG2-1:0] wp, rp;
    reg [LOG2:0]   cnt;                  // entries in mem (head not counted)

    assign o_cnt = 11'(cnt) + {10'd0, o_vld};

    wire refill = (!o_vld || i_pop) && (cnt != '0);

    // H7b.8e: exact next-value almost-full -- o_af during any cycle equals
    // (o_cnt >= AF_TH) of that same cycle (total count is conserved across
    // the mem/head handoff: refill moves an entry, push/pop add/remove one),
    // so consumers see IDENTICAL timing to the old comb compare while the
    // count adder + threshold compare come off a register.
    wire [LOG2:0] tot_nx = (cnt + {{LOG2{1'b0}}, o_vld})
                         + ((LOG2+1)'(i_push ? 1 : 0))
                         - ((LOG2+1)'((i_pop && o_vld) ? 1 : 0));

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            wp <= '0; rp <= '0; cnt <= '0;
            o_vld <= 1'b0;
            o_af  <= 1'b0;
            o_af2 <= 1'b0;
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
            o_af  <= (AF_TH != 0) && (tot_nx >= (LOG2+1)'(AF_TH));
            o_af2 <= (AF_TH != 0) && (tot_nx >= (LOG2+1)'(AF_TH));
        end
    end

endmodule
`default_nettype none
