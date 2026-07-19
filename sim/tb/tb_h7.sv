`default_nettype none
//============================================================================
// tb_h7.sv - H7a step-4 testbench top: the full execution-plane stack
//
//   blit_draw -> blit_batch -> CV1k_ddr3_harness -> DDRAM pins (C++ stat slave)
//                blit_video (PREFETCH=1) ----^   (video line trains)
//
// vs tb_blit: no behavioral VRAM - memory lives behind the MiSTer DDRAM
// face, served by the C++ slave with ddr3_stat.h-calibrated timing.  The
// harness runs at the TARGET clock configuration: i_CLK = 153.6 MHz,
// i_CKIO_PCEN every 3rd cycle (CKIO = 51.2 MHz), so engine speed, steal
// windows and video cadence carry target-accurate ratios and the per-op
// lateness measured by the C++ monitor is the H7a acceptance quantity.
//
// blit_top's steal arbitration is replicated verbatim on the write channel
// (blit_top itself carries the CS6/BREQ bus face the trace TB doesn't
// model).  The timing plane (blit_gov) is not instantiated: golden per-op
// finish times come from cost_model.h in the C++ harness, exactly as the
// P-stage study consumed them.
//============================================================================
module tb_h7 (
    input  wire        i_CLK,
    input  wire        i_CKIO_PCEN,
    input  wire        i_RST_n,

    // exec kick + shadow clip + scroll (per trace record)
    input  wire        i_exec,
    input  wire [15:0] i_clip_x,
    input  wire [15:0] i_clip_y,
    input  wire [15:0] i_scroll_x,
    input  wire [15:0] i_scroll_y,

    // attribute FIFO feed (C++ paces at modeled fetch-arrival times)
    input  wire        i_fifo_valid,
    input  wire [15:0] i_fifo_word,
    output wire        o_fifo_pop,

    // MiSTer DDRAM face (C++ slave)
    output wire        DDRAM_CLK,
    input  wire        DDRAM_BUSY,
    output wire [7:0]  DDRAM_BURSTCNT,
    output wire [28:0] DDRAM_ADDR,
    input  wire [63:0] DDRAM_DOUT,
    input  wire        DDRAM_DOUT_READY,
    output wire        DDRAM_RD,
    output wire [63:0] DDRAM_DIN,
    output wire [7:0]  DDRAM_BE,
    output wire        DDRAM_WE,

    // lateness-monitor + status taps
    output wire        o_dsc_vld,
    output wire [31:0] o_dsc_dst0,
    output wire [13:0] o_dsc_npx,
    output wire [12:0] o_dsc_rows,
    output wire        o_dsc_upl,
    output wire [24:0] o_dsc_upl_addr,
    output wire        o_op_srv,
    output wire        o_wr_idle,
    output wire        o_bat_idle,
    output wire        o_busy,
    output wire        o_done,

    // video taps (frame capture + timing)
    output wire        o_steal,
    output wire        o_hline,
    output wire        o_vsync,
    output wire        o_px_de,
    output wire [15:0] o_px
);

    // ------------------------------------------------------------------
    // engine <-> batch beat channels (blit_top's steal gating replicated)
    // ------------------------------------------------------------------
    wire        srd_req, drd_req, rd_vld;
    wire        rq_v, rq_wr, rq_blend;    // r4 raw request legs
    wire [24:0] srd_addr, drd_addr;
    wire [63:0] srd_data, drd_data;
    wire        eng_wr_req, bat_wr_rdy;
    wire [24:0] wr_addr;
    wire [63:0] wr_data;
    wire [3:0]  wr_mask;

    wire steal;
    assign o_steal = steal;
    wire bat_wr_req = eng_wr_req && !steal;        // H5 steal semantics
    wire eng_wr_rdy = bat_wr_rdy && !steal;

    // descriptor sideband
    wire        dsc_vld, dsc_flipx, dsc_flipy, dsc_blend, dsc_strict,
                dsc_px1, dsc_wait, dsc_upl;
    wire [12:0] dsc_sx_lo, dsc_sx_hi, dsc_rows;
    wire [11:0] dsc_sy0;
    wire [13:0] dsc_npx, dsc_upl_dimx;
    wire [31:0] dsc_dst0;
    wire [24:0] dsc_upl_addr;
    wire [12:0] dsc_upl_dimy;

    assign o_dsc_vld      = dsc_vld;
    assign o_dsc_dst0     = dsc_dst0;
    assign o_dsc_npx      = dsc_npx;
    assign o_dsc_rows     = dsc_rows;
    assign o_dsc_upl      = dsc_upl;
    assign o_dsc_upl_addr = dsc_upl_addr;

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
        .o_wr_req     (eng_wr_req),
        .o_wr_addr    (wr_addr),
        .o_wr_data    (wr_data),
        .o_wr_mask    (wr_mask),
        .i_wr_rdy     (eng_wr_rdy),
        .i_rd_vld     (rd_vld),
        .o_rq_v       (rq_v),          // r4 raw request legs
        .o_rq_wr      (rq_wr),
        .o_rq_blend   (rq_blend),
        .o_dsc_vld    (dsc_vld),
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
        .i_wr_req       (bat_wr_req),
        .i_wr_addr      (wr_addr),
        .i_wr_mask      (wr_mask)
    );

    // ------------------------------------------------------------------
    // batch <-> harness train port
    // ------------------------------------------------------------------
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
        .i_wr_req     (bat_wr_req),
        .i_wr_addr    (wr_addr),
        .i_wr_data    (wr_data),
        .i_wr_mask    (wr_mask),
        .o_wr_rdy     (bat_wr_rdy),
        .i_rq_v       (rq_v),          // r4 raw request legs
        .i_rq_wr      (rq_wr),
        .i_rq_blend   (rq_blend),
        .i_steal      (steal),
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
        .o_op_srv     (o_op_srv),
        .o_wr_idle    (o_wr_idle)
    );

    // ------------------------------------------------------------------
    // video: target config = 1-hline train prefetch
    // ------------------------------------------------------------------
    wire        lf_req, lf_dvld;
    wire [11:0] lf_y;
    wire [12:0] lf_x0;
    wire [63:0] lf_data;

    blit_video #(
        .PREFETCH (1'b1)
    ) u_video (
        .i_CLK       (i_CLK),
        .i_CKIO_PCEN (i_CKIO_PCEN),
        .i_RST_n     (i_RST_n),
        .i_scroll_x  (i_scroll_x),
        .i_scroll_y  (i_scroll_y),
        .i_irq_ack   (1'b0),
        .o_vrd_req   (),
        .o_vrd_addr  (),
        .i_vrd_data  (64'd0),
        .o_lf_req    (lf_req),
        .o_lf_y      (lf_y),
        .o_lf_x0     (lf_x0),
        .i_lf_dvld   (lf_dvld),
        .i_lf_data   (lf_data),
        .o_steal     (steal),
        .o_hline     (o_hline),
        .o_vsync     (o_vsync),
        .o_px_de     (o_px_de),
        .o_px        (o_px)
    );

    // ------------------------------------------------------------------
    // harness (NAND client tied off until step 5)
    // ------------------------------------------------------------------
    CV1k_ddr3_harness u_harness (
        .i_CLK        (i_CLK),
        .i_RST_n      (i_RST_n),
        .i_lf_req     (lf_req),
        .i_lf_y       (lf_y),
        .i_lf_x0      (lf_x0),
        .o_lf_dvld    (lf_dvld),
        .o_lf_data    (lf_data),
        .i_prd_req    (prd_req),
        .i_prd_addr   (prd_addr),
        .i_prd_len    (prd_len),
        .o_prd_rdy    (prd_rdy),
        .o_prd_dvld   (prd_dvld),
        .o_prd_data   (prd_data),
        .i_pwr_req    (pwr_req),
        .i_pwr_addr   (pwr_addr),
        .i_pwr_data   (pwr_data),
        .i_pwr_be     (pwr_be),
        .o_pwr_rdy    (pwr_rdy),
        .i_rd_train   (rd_train),
        .i_wr_train   (wr_train),
        .i_nd_req     (1'b0),
        .i_nd_addr    (29'd0),
        .i_nd_len     (11'd0),
        .o_nd_rdy     (),
        .o_nd_dvld    (),
        .o_nd_data    (),
        .i_ym_req     (1'b0),
        .i_ym_addr    (29'd0),
        .i_ym_len     (11'd0),
        .o_ym_rdy     (),
        .o_ym_dvld    (),
        .o_ym_data    (),
        .DDRAM_CLK    (DDRAM_CLK),
        .DDRAM_BUSY   (DDRAM_BUSY),
        .DDRAM_BURSTCNT (DDRAM_BURSTCNT),
        .DDRAM_ADDR   (DDRAM_ADDR),
        .DDRAM_DOUT   (DDRAM_DOUT),
        .DDRAM_DOUT_READY (DDRAM_DOUT_READY),
        .DDRAM_RD     (DDRAM_RD),
        .DDRAM_DIN    (DDRAM_DIN),
        .DDRAM_BE     (DDRAM_BE),
        .DDRAM_WE     (DDRAM_WE)
    );

endmodule
`default_nettype none
