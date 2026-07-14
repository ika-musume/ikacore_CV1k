`default_nettype none
//============================================================================
// tb_blit.sv - H3/H4 blitter trace testbench top
//
// Thin wrapper: blit_draw + blit_vram_beh + blit_gov, all control from the
// C++ harness (tb_blit_main.cpp) which feeds op words over the FIFO interface
// exactly as blit_fetch would, and diffs the VRAM against the golden model
// (sim/blitgold/golden.h) after every EXEC.  No board, no bus - the accept
// is trace-driven by design (board integration is a separate smoke).
//
// H4: the governor rides the same word stream (i_gov_push mirrors each word
// as it is handed to the decoder) in warp mode - arrival pacing is synthetic
// here, so only the per-op COSTS are checked (vs workload.h/cost_model.h);
// the real-time anchors run in the board sim (+blitanchor).
//
// H7a (-DBLIT_BATCH build): blit_batch + blit_port_beh replace blit_vram_beh
// behind the SAME engine channels - the step-3 A/B: pixel/gov/vram hashes
// must match the non-batch build bit for bit (add +portjit=SEED for port
// timing jitter).  o_bat_idle gates the end-of-exec VRAM compare on the
// write-train drain (constant 1 in the non-batch build).
//============================================================================
module tb_blit (
    input  wire        i_CLK,
    input  wire        i_RST_n,

    input  wire        i_exec,
    input  wire [15:0] i_clip_x,
    input  wire [15:0] i_clip_y,

    input  wire        i_fifo_valid,
    input  wire [15:0] i_fifo_word,
    output wire        o_fifo_pop,

    // governor arrival stream + table load port (harness-driven)
    input  wire        i_gov_push,
    input  wire [15:0] i_gov_word,
    input  wire        i_tbl_we,
    input  wire [3:0]  i_tbl_idx,
    input  wire [31:0] i_tbl_data,

    // governor taps
    output wire        o_gov_busy,
    output wire        o_gov_retire,
    output wire        o_gov_hold,
    output wire        o_dbg_vld,
    output wire [1:0]  o_dbg_kind,
    output wire [26:0] o_dbg_cost,

    output wire        o_busy,
    output wire        o_done,
    output wire        o_bat_idle       // batch drained (1 when no batch)
);

    wire        srd_req, drd_req, wr_req, wr_rdy, rd_vld;
    wire [24:0] srd_addr, drd_addr, wr_addr;
    wire [63:0] srd_data, drd_data, wr_data;
    wire [3:0]  wr_mask;

    // H7 descriptor sideband + footprint checker nets
    wire        dsc_vld, dsc_flipx, dsc_flipy, dsc_blend, dsc_strict,
                dsc_px1, dsc_wait, dsc_upl;
    wire [12:0] dsc_sx_lo, dsc_sx_hi, dsc_rows;
    wire [11:0] dsc_sy0;
    wire [13:0] dsc_npx, dsc_upl_dimx;
    wire [31:0] dsc_dst0;
    wire [24:0] dsc_upl_addr;
    wire [12:0] dsc_upl_dimy;

    blit_draw u_draw (
        .i_CLK        (i_CLK),
        .i_RST_n      (i_RST_n),
        .i_exec       (i_exec),
        .i_clip_x     (i_clip_x),
        .i_clip_y     (i_clip_y),
        .i_fifo_valid (i_fifo_valid),
        .i_fifo_word  (i_fifo_word),
        .o_fifo_pop   (o_fifo_pop),
        .o_srd_req    (srd_req),
        .o_srd_addr   (srd_addr),
        .i_srd_data   (srd_data),
        .o_drd_req    (drd_req),
        .o_drd_addr   (drd_addr),
        .i_drd_data   (drd_data),
        .o_wr_req     (wr_req),
        .o_wr_addr    (wr_addr),
        .o_wr_data    (wr_data),
        .o_wr_mask    (wr_mask),
        .i_wr_rdy     (wr_rdy),
        .i_rd_vld     (rd_vld),        // 1'b1 unless the batch build stalls it
        .o_dsc_vld    (dsc_vld),       // H7 descriptor sideband
        .o_dsc_sx_lo  (dsc_sx_lo),
        .o_dsc_sx_hi  (dsc_sx_hi),
        .o_dsc_sy0    (dsc_sy0),
        .o_dsc_rows   (dsc_rows),
        .o_dsc_npx    (dsc_npx),
        .o_dsc_dst0   (dsc_dst0),
        .o_dsc_flipx  (dsc_flipx),
        .o_dsc_flipy  (dsc_flipy),
        .o_dsc_blend  (dsc_blend),
        .o_dsc_strict (dsc_strict),
        .o_dsc_px1    (dsc_px1),
        .o_dsc_wait   (dsc_wait),
        .o_dsc_upl    (dsc_upl),
        .o_dsc_upl_addr (dsc_upl_addr),
        .o_dsc_upl_dimx (dsc_upl_dimx),
        .o_dsc_upl_dimy (dsc_upl_dimy),
        .o_busy       (o_busy),
        .o_done       (o_done)
    );

    // every beat must land inside the descriptor-predicted footprint -
    // the property blit_batch's train formation relies on ($fatal on miss)
    blit_dsc_check u_dsc_check (
        .i_CLK          (i_CLK),
        .i_RST_n        (i_RST_n),
        .i_dsc_vld      (dsc_vld),
        .i_dsc_sx_lo    (dsc_sx_lo),
        .i_dsc_sx_hi    (dsc_sx_hi),
        .i_dsc_sy0      (dsc_sy0),
        .i_dsc_rows     (dsc_rows),
        .i_dsc_npx      (dsc_npx),
        .i_dsc_dst0     (dsc_dst0),
        .i_dsc_flipy    (dsc_flipy),
        .i_dsc_upl      (dsc_upl),
        .i_dsc_upl_addr (dsc_upl_addr),
        .i_dsc_upl_dimx (dsc_upl_dimx),
        .i_dsc_upl_dimy (dsc_upl_dimy),
        .i_srd_req      (srd_req),
        .i_srd_addr     (srd_addr),
        .i_wr_req       (wr_req),
        .i_wr_addr      (wr_addr),
        .i_wr_mask      (wr_mask)
    );

    blit_gov u_gov (
        .i_CLK       (i_CLK),
        .i_CKIO_PCEN (1'b1),           // free-running tick; warp mode anyway
        .i_RST_n     (i_RST_n),
        .i_exec      (i_exec),
        .i_clip_x    (i_clip_x),
        .i_clip_y    (i_clip_y),
        .i_push      (i_gov_push),
        .i_word      (i_gov_word),
        .i_hline     (1'b0),           // no video here: boundary free-runs
        .i_warp      (1'b1),           // synthetic arrivals: fast-forward `now`
        .i_tbl_we    (i_tbl_we),
        .i_tbl_idx   (i_tbl_idx),
        .i_tbl_data  (i_tbl_data),
        .o_fetch_hold(o_gov_hold),
        .o_pace_exec2brq (),
        .o_pace_chunk(),
        .o_pace_upld (),
        .o_busy      (o_gov_busy),
        .o_retire    (o_gov_retire),
        .o_dbg_vld   (o_dbg_vld),
        .o_dbg_kind  (o_dbg_kind),
        .o_dbg_cost  (o_dbg_cost)
    );

`ifdef BLIT_BATCH
    //------------------------------------------------------------------
    // H7a backend: engine channels -> blit_batch trains -> blit_port_beh
    //------------------------------------------------------------------
    wire        prd_req, prd_rdy, prd_dvld;
    wire [22:0] prd_addr;
    wire [10:0] prd_len;
    wire [63:0] prd_data;
    wire        pwr_req, pwr_rdy;
    wire [22:0] pwr_addr;
    wire [63:0] pwr_data;
    wire [3:0]  pwr_be;
    wire        rd_train, wr_train;

    blit_batch u_batch (
        .i_CLK        (i_CLK),
        .i_RST_n      (i_RST_n),
        .i_srd_req    (srd_req),
        .i_srd_addr   (srd_addr),
        .o_srd_data   (srd_data),
        .i_drd_req    (drd_req),
        .i_drd_addr   (drd_addr),
        .o_drd_data   (drd_data),
        .o_rd_vld     (rd_vld),
        .i_wr_req     (wr_req),
        .i_wr_addr    (wr_addr),
        .i_wr_data    (wr_data),
        .i_wr_mask    (wr_mask),
        .o_wr_rdy     (wr_rdy),
        .i_dsc_vld    (dsc_vld),
        .i_dsc_sx_lo  (dsc_sx_lo),
        .i_dsc_sy0    (dsc_sy0),
        .i_dsc_rows   (dsc_rows),
        .i_dsc_npx    (dsc_npx),
        .i_dsc_dst0   (dsc_dst0),
        .i_dsc_flipy  (dsc_flipy),
        .i_dsc_blend  (dsc_blend),
        .i_dsc_strict (dsc_strict),
        .i_dsc_px1    (dsc_px1),
        .i_dsc_wait   (dsc_wait),
        .o_prd_req    (prd_req),
        .o_prd_addr   (prd_addr),
        .o_prd_len    (prd_len),
        .i_prd_rdy    (prd_rdy),
        .i_prd_dvld   (prd_dvld),
        .i_prd_data   (prd_data),
        .o_pwr_req    (pwr_req),
        .o_pwr_addr   (pwr_addr),
        .o_pwr_data   (pwr_data),
        .o_pwr_be     (pwr_be),
        .i_pwr_rdy    (pwr_rdy),
        .o_rd_train   (rd_train),
        .o_wr_train   (wr_train),
        .o_idle       (o_bat_idle),
        .o_op_srv     (),
        .o_wr_idle    ()
    );

    blit_port_beh u_vram (
        .i_CLK      (i_CLK),
        .i_RST_n    (i_RST_n),
        .i_prd_req  (prd_req),
        .i_prd_addr (prd_addr),
        .i_prd_len  (prd_len),
        .o_prd_rdy  (prd_rdy),
        .o_prd_dvld (prd_dvld),
        .o_prd_data (prd_data),
        .i_pwr_req  (pwr_req),
        .i_pwr_addr (pwr_addr),
        .i_pwr_data (pwr_data),
        .i_pwr_be   (pwr_be),
        .o_pwr_rdy  (pwr_rdy),
        .i_rd_train (rd_train),
        .i_wr_train (wr_train)
    );
`else
    assign rd_vld     = 1'b1;          // fixed-latency behavioral VRAM
    assign o_bat_idle = 1'b1;

    blit_vram_beh u_vram (
        .i_CLK      (i_CLK),
        .i_srd_req  (srd_req),
        .i_srd_addr (srd_addr),
        .o_srd_data (srd_data),
        .i_drd_req  (drd_req),
        .i_drd_addr (drd_addr),
        .o_drd_data (drd_data),
        .i_vrd_req  (1'b0),            // no video in the trace TB
        .i_vrd_addr (25'd0),
        .o_vrd_data (),
        .i_wr_req   (wr_req),
        .i_wr_addr  (wr_addr),
        .i_wr_data  (wr_data),
        .i_wr_mask  (wr_mask),
        .o_wr_rdy   (wr_rdy)
    );
`endif

endmodule
`default_nettype none
