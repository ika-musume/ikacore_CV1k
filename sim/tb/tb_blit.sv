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
    output wire        o_done
);

    wire        srd_req, drd_req, wr_req, wr_rdy;
    wire [24:0] srd_addr, drd_addr, wr_addr;
    wire [63:0] srd_data, drd_data, wr_data;
    wire [3:0]  wr_mask;

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
        .o_busy       (o_busy),
        .o_done       (o_done)
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

endmodule
`default_nettype none
