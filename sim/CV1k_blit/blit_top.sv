`default_nettype none
//============================================================================
// blit_top.sv - CV1000 blitter core top                        [H7 / I-4.3]
//
// Integration wrapper for the FROZEN H0-H6 blitter core: blit_regs (CS6
// register file) + blit_fetch (BREQ/BACK op-list bus master) + blit_gov
// (timing-plane governor) + blit_draw (native-speed draw engine) +
// blit_video (sync gen / line fetch / scanout) + the IRQ1/IRQ2 pulse
// shapers.  Pure code motion from ikacore_CV1k.sv - no logic, state or
// timing change; the H6 conformance run and the FASTBOOT board regression
// are the accept for this file.
//
// Platform-agnostic by construction: everything below this boundary is the
// same RTL on the Verilator board sim and on the MiSTer core.  The memory
// side is exported as the H3 beat channels (read-stall i_rd_vld protocol,
// per-pixel-lane masked writes, no RMW) - since H7b.2 the board top wires
// them to blit_batch (K=8-objline trains) in front of the swappable
// CV1k_ddr3_harness; the H6 unit rigs keep the fixed-latency
// blit_vram_beh (i_rd_vld tied 1, PREFETCH=0).  blit_fetch stays on the
// SH-3 shared bus (authentic BREQ/BACK) and never touches the DDR3 stack.
//
// Steal arbitration (H5 semantics, unchanged): while the line fetcher owns
// the memory (o_steal), the draw engine's write channel backpressures on
// its i_wr_rdy port and o_wr_req is gated off - scanout owns the memory.
// Engine reads keep flowing (pipe prefetch); command-level read
// arbitration is the DDR3 harness's job behind these same channels.
//============================================================================
module blit_top #(
    // H7b.2: PREFETCH=1 selects blit_video's 1-hline train prefetch (the
    // o_lf_* face below, served by CV1k_ddr3_harness) instead of the
    // beat-wise o_vrd channel - the tb_h7-proven target configuration.
    // Default 0 keeps the H6 fixed-latency rigs bit-identical.
    parameter bit PREFETCH = 1'b0
)(
    input  wire [3:0]  i_DSW_S2,       // DIP S2 (H7b: runtime input from the
                                       // MiSTer OSD; was a parameter)
    input  wire        i_CLK,          // blit domain clock (H7b.2: 153.6 MHz =
                                       // 3x CKIO; H6 rigs drove 102.4 = 2x)
    input  wire        i_CKIO_PCEN,    // pulses the i_CLK cycle CKIO rises
    input  wire        i_RST_n,

    //------------------------------------------------------------------
    // CS6 register-file slave (SH-3 shared bus, decoded by U13)
    //------------------------------------------------------------------
    input  wire        i_BLIT_n,       // U13 o_BLITTER_n (= CS6_n)
    input  wire        i_RD_n,
    input  wire [3:0]  i_WE_n,
    input  wire        i_RD_WR,
    input  wire [6:2]  i_A,
    input  wire [31:0] i_D_CPU,        // SH-3 write-data view of the bus (D_O)
    output wire [31:0] o_D,            // CS6 read data onto the shared bus
    output wire        o_D_OE,

    //------------------------------------------------------------------
    // op-list bus mastering (BREQ/BACK tenure + U1 SDRAM command pins)
    //------------------------------------------------------------------
    output wire        o_BREQ_n,
    input  wire        i_BACK_n,
    output wire        o_bus_drive,    // fetch unit owns the shared bus
    output wire [25:0] o_BF_A,
    output wire        o_BF_CS_n,
    output wire        o_BF_RAS_n,
    output wire        o_BF_CAS_n,
    output wire        o_BF_WE,
    output wire [3:0]  o_BF_DQM,
    input  wire [31:0] i_D_BUS,        // resolved shared bus (fetch read data)
    output wire        o_REF_WIN,      // fetch guarantees >= 5 CKIO w/o PALL/ACT
                                       // (CV1k_sdram_control refresh scheduler)

    //------------------------------------------------------------------
    // interrupts (falling-edge sources, INTC-visible shapes)
    //------------------------------------------------------------------
    input  wire        i_IRQ1_EN,      // sim A/B knob (+noirq1); tie 1 on target
    output wire        o_IRQ1_n,       // blitter done (governor retirement)
    output wire        o_IRQ2_n,       // vblank (real vsync)

    //------------------------------------------------------------------
    // governor cost tables (runtime-loadable; MiSTer HPS / TB pokes)
    //------------------------------------------------------------------
    input  wire        i_tbl_we,
    input  wire [3:0]  i_tbl_idx,
    input  wire [31:0] i_tbl_data,

    //------------------------------------------------------------------
    // VRAM beat channels (H3 contract: reads serve a 4-px beat one cycle
    // after the request cycle and hold; writes are per-pixel masked)
    //------------------------------------------------------------------
    output wire        o_srd_req,      // draw src read
    output wire [24:0] o_srd_addr,
    input  wire [63:0] i_srd_data,
    output wire        o_drd_req,      // draw dst read (blend)
    output wire [24:0] o_drd_addr,
    input  wire [63:0] i_drd_data,
    output wire        o_wr_req,       // draw/upload write (steal-gated here)
    output wire [24:0] o_wr_addr,
    output wire [63:0] o_wr_data,
    output wire [3:0]  o_wr_mask,
    input  wire        i_wr_rdy,
    input  wire        i_rd_vld,       // read-stall (H7): tie 1 on fixed-latency backends
    output wire        o_vrd_req,      // video line fetch (PREFETCH=0 only)
    output wire [24:0] o_vrd_addr,
    input  wire [63:0] i_vrd_data,
    output wire        o_lf_req,       // PREFETCH=1 line-train face
    output wire [11:0] o_lf_y,         // (CV1k_ddr3_harness video client;
    output wire [12:0] o_lf_x0,        //  idle when PREFETCH=0)
    input  wire        i_lf_dvld,
    input  wire [63:0] i_lf_data,
    output wire        o_steal,        // scanout-owns-memory window (debug/arb tap)

    //------------------------------------------------------------------
    // descriptor sideband (H7): blit_draw's output-only geometry taps,
    // consumed by blit_batch for K=8-objline train formation.  Leave
    // open on fixed-latency backends (board sim / blit_vram_beh).
    //------------------------------------------------------------------
    output wire        o_dsc_vld,
    output wire [12:0] o_dsc_sx_lo,
    output wire [12:0] o_dsc_sx_hi,
    output wire [11:0] o_dsc_sy0,
    output wire [12:0] o_dsc_rows,
    output wire [13:0] o_dsc_npx,
    output wire [31:0] o_dsc_dst0,
    output wire        o_dsc_flipx,
    output wire        o_dsc_flipy,
    output wire        o_dsc_blend,
    output wire        o_dsc_strict,
    output wire        o_dsc_px1,
    output wire        o_dsc_wait,
    output wire        o_dsc_upl,
    output wire [24:0] o_dsc_upl_addr,
    output wire [13:0] o_dsc_upl_dimx,
    output wire [12:0] o_dsc_upl_dimy,

    //------------------------------------------------------------------
    // video timing + pixel stream (o_px valid on the o_px_de strobe)
    //------------------------------------------------------------------
    output wire        o_hline,        // 1-cycle pulse per line (steal point)
    output wire        o_vsync,        // 1-cycle pulse at VSYNC_LINE start
    output wire        o_px_de,
    output wire [15:0] o_px
);

//------------------------------------------------------------------
// internal nets (names kept from ikacore_CV1k.sv for tap continuity)
//------------------------------------------------------------------
wire        blit_exec;
wire [28:0] blit_list;
wire        blit_busy, blit_done;
wire        blit_fifo_valid, blit_fifo_pop;
wire [15:0] blit_fifo_word;

wire [15:0] blit_clip_x, blit_clip_y;
wire        blit_draw_busy, blit_draw_done;

wire        blit_gov_busy, blit_gov_retire, blit_gov_hold;
wire        blit_snoop_push;
wire [15:0] blit_snoop_word;
wire [7:0]  blit_pace_e2b, blit_pace_chunk, blit_pace_upld;

wire [15:0] blit_scroll_x, blit_scroll_y;
wire        blit_irq_ack;
wire        blit_steal, blit_hline, blit_vsync;

wire        bv_wr_req_raw;

assign o_steal = blit_steal;
assign o_hline = blit_hline;
assign o_vsync = blit_vsync;

// H5 steal arbitration: gate both faces of the write channel with the
// steal window (identical to the board-top wiring it replaces; the
// behavioral backend's wr_rdy is constant 1, so gating order is inert).
assign o_wr_req = bv_wr_req_raw && !blit_steal;

//==================================================================
//  Register file (CS6 0x18000000, blitter_detail.md §3)  [H0/I-1.1]
//==================================================================
blit_regs u_blit_regs (
    .i_DSW_S2    (i_DSW_S2),
    .i_CLK       (i_CLK),
    .i_CKIO_PCEN (i_CKIO_PCEN),
    .i_RST_n     (i_RST_n),
    .i_BLIT_n    (i_BLIT_n),
    .i_RD_n      (i_RD_n),
    .i_WE_n      (i_WE_n),
    .i_RD_WR     (i_RD_WR),
    .i_A         (i_A),
    .i_D         (i_D_CPU),
    .o_D         (o_D),
    .o_D_OE      (o_D_OE),
    .i_busy      (blit_gov_busy | blit_busy | blit_draw_busy), // H4: governed
                                       // BUSY owns the window; fetch/draw OR'd
                                       // in as a belt-and-braces floor (the
                                       // governed end >= both by construction)
    .o_exec      (blit_exec),
    .o_list_addr (blit_list),
    .o_clip_x    (blit_clip_x),
    .o_clip_y    (blit_clip_y),
    .o_scroll_x  (blit_scroll_x),
    .o_scroll_y  (blit_scroll_y),
    .o_irq_ack   (blit_irq_ack)
);

//==================================================================
//  Op-list fetch unit (H2 / I-4.1 + I-2.4)
//==================================================================
blit_fetch u_blit_fetch (
    .i_CLK        (i_CLK),
    .i_CKIO_PCEN  (i_CKIO_PCEN),
    .i_RST_n      (i_RST_n),
    .i_exec       (blit_exec),
    .i_list_addr  (blit_list),
    .i_exec2brq_ckio (blit_pace_e2b),  // H4: pacing from the governor tables
    .i_chunk_ckio (blit_pace_chunk),
    .i_upld_ckio  (blit_pace_upld),
    .i_hold       (blit_gov_hold),     // H4: governed 512-chunk fetch window
    .o_BREQ_n     (o_BREQ_n),
    .i_BACK_n     (i_BACK_n),
    .o_bus_drive  (o_bus_drive),
    .o_A          (o_BF_A),
    .o_CS_n       (o_BF_CS_n),
    .o_RAS_n      (o_BF_RAS_n),
    .o_CAS_n      (o_BF_CAS_n),
    .o_WE         (o_BF_WE),
    .o_DQM        (o_BF_DQM),
    .i_D          (i_D_BUS),
    .o_fifo_valid (blit_fifo_valid),
    .o_fifo_word  (blit_fifo_word),
    .i_fifo_pop   (blit_fifo_pop),
    .o_snoop_push (blit_snoop_push),   // H4: governor arrival stream
    .o_snoop_word (blit_snoop_word),
    .o_busy       (blit_busy),
    .o_done       (blit_done),
    .o_REF_WIN    (o_REF_WIN)
);

//==================================================================
//  Timing governor (H4 / I-2.1+I-2.2+I-2.5)
//==================================================================
blit_gov u_blit_gov (
    .i_CLK       (i_CLK),
    .i_CKIO_PCEN (i_CKIO_PCEN),
    .i_RST_n     (i_RST_n),
    .i_exec      (blit_exec),
    .i_clip_x    (blit_clip_x),
    .i_clip_y    (blit_clip_y),
    .i_push      (blit_snoop_push),
    .i_word      (blit_snoop_word),
    .i_hline     (blit_hline),         // H5: steal phase = real scanline
    .i_warp      (1'b0),               // real time base outside the trace TB
    .i_tbl_we    (i_tbl_we),
    .i_tbl_idx   (i_tbl_idx),
    .i_tbl_data  (i_tbl_data),
    .o_fetch_hold(blit_gov_hold),
    .o_pace_exec2brq (blit_pace_e2b),
    .o_pace_chunk(blit_pace_chunk),
    .o_pace_upld (blit_pace_upld),
    .o_busy      (blit_gov_busy),
    .o_retire    (blit_gov_retire),
    .o_dbg_vld   (),                   // TB scoreboard taps (hierarchical)
    .o_dbg_kind  (),
    .o_dbg_cost  ()
);

//==================================================================
//  Draw engine (H3 / I-1.5/6/7) - native speed, never throttled
//==================================================================
blit_draw u_blit_draw (
    .i_CLK        (i_CLK),
    .i_RST_n      (i_RST_n),
    .i_exec       (blit_exec),
    .i_clip_x     (blit_clip_x),
    .i_clip_y     (blit_clip_y),
    .i_fifo_valid (blit_fifo_valid),
    .i_fifo_word  (blit_fifo_word),
    .o_fifo_pop   (blit_fifo_pop),
    .o_srd_req    (o_srd_req),
    .o_srd_addr   (o_srd_addr),
    .i_srd_data   (i_srd_data),
    .o_drd_req    (o_drd_req),
    .o_drd_addr   (o_drd_addr),
    .i_drd_data   (i_drd_data),
    .o_wr_req     (bv_wr_req_raw),
    .o_wr_addr    (o_wr_addr),
    .o_wr_data    (o_wr_data),
    .o_wr_mask    (o_wr_mask),
    .i_wr_rdy     (i_wr_rdy && !blit_steal),   // H5: scanout owns VRAM
    .i_rd_vld     (i_rd_vld),                  // H7: tie 1 on fixed-latency backends
    .o_dsc_vld    (o_dsc_vld),                 // H7 descriptor sideband
    .o_dsc_sx_lo  (o_dsc_sx_lo),
    .o_dsc_sx_hi  (o_dsc_sx_hi),
    .o_dsc_sy0    (o_dsc_sy0),
    .o_dsc_rows   (o_dsc_rows),
    .o_dsc_npx    (o_dsc_npx),
    .o_dsc_dst0   (o_dsc_dst0),
    .o_dsc_flipx  (o_dsc_flipx),
    .o_dsc_flipy  (o_dsc_flipy),
    .o_dsc_blend  (o_dsc_blend),
    .o_dsc_strict (o_dsc_strict),
    .o_dsc_px1    (o_dsc_px1),
    .o_dsc_wait   (o_dsc_wait),
    .o_dsc_upl    (o_dsc_upl),
    .o_dsc_upl_addr (o_dsc_upl_addr),
    .o_dsc_upl_dimx (o_dsc_upl_dimx),
    .o_dsc_upl_dimy (o_dsc_upl_dimy),
    .o_busy       (blit_draw_busy),
    .o_done       (blit_draw_done)
);

//==================================================================
//  Video scanout (H5 / I-3.1/2/3, provisional params)
//==================================================================
blit_video #(
    .PREFETCH    (PREFETCH)
) u_blit_video (
    .i_CLK       (i_CLK),
    .i_CKIO_PCEN (i_CKIO_PCEN),
    .i_RST_n     (i_RST_n),
    .i_scroll_x  (blit_scroll_x),
    .i_scroll_y  (blit_scroll_y),
    .i_irq_ack   (blit_irq_ack),
    .o_vrd_req   (o_vrd_req),
    .o_vrd_addr  (o_vrd_addr),
    .i_vrd_data  (i_vrd_data),
    .o_lf_req    (o_lf_req),
    .o_lf_y      (o_lf_y),
    .o_lf_x0     (o_lf_x0),
    .i_lf_dvld   (i_lf_dvld),
    .i_lf_data   (i_lf_data),
    .o_steal     (blit_steal),
    .o_hline     (blit_hline),
    .o_vsync     (blit_vsync),
    .o_px_de     (o_px_de),
    .o_px        (o_px)
);

//==================================================================
//  Vblank IRQ2  [H5 - REAL VSYNC]
//
//  One falling edge per frame at the sync generator's vsync (853,072
//  CKIO = 60.0184 Hz; position within the frame provisional until M-2).
//  ICR1 configures IRQ2 as falling-edge / priority 4, so the pulse
//  shape is invisible to the INTC; the ISR clears IRR0.2 (INTC-side
//  ack) and pulses 0x24 (video-side ack, consumed by blit_video,
//  provisional no-op until M-2).
//==================================================================
localparam int unsigned IRQ2_LOW_CKIO = 64;           // low-pulse width (edge is what matters)

int unsigned irq2_low  = 0;
reg          irq2_n_r  = 1'b1;
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if (!i_RST_n) begin
        irq2_low <= 0;
        irq2_n_r <= 1'b1;
    end
    else begin
        if (blit_vsync)
            irq2_low <= IRQ2_LOW_CKIO;                // arm the low pulse
        else if (i_CKIO_PCEN && irq2_low != 0) begin
            irq2_low <= irq2_low - 1;
            irq2_n_r <= 1'b0;
            if (irq2_low == 1)
                irq2_n_r <= 1'b1;                     // release, ready for next edge
        end
    end
end
assign o_IRQ2_n = irq2_n_r;

//==================================================================
//  Blitter-done IRQ1  [H4 - GOVERNOR-TIMED]
//
//  Falling edge at the GOVERNOR's retirement (o_retire = governed end
//  of the op list per the cost model).  The game's loading/queue
//  pipeline arms an async mode where the NEXT blit is issued from the
//  IRQ1 callback chain - without IRQ1 the blit queue dead-stops.  Boot
//  enables IRQ1 at priority 3 (IPRC|=0x0030) falling-edge (ICR1), ISR
//  shell 0c0021c8 acks via blitter 0x24 bit1 + clears IRR0.1.
//  i_IRQ1_EN is the +noirq1 A/B knob (board top owns the plusarg).
//==================================================================
localparam int unsigned IRQ1_LOW_CKIO = 64;           // same shape as IRQ2's pulse

int unsigned irq1_low  = 0;
reg          irq1_n_r  = 1'b1;
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if (!i_RST_n) begin
        irq1_low <= 0;
        irq1_n_r <= 1'b1;
    end
    else begin
        if (blit_gov_retire && i_IRQ1_EN)
            irq1_low <= IRQ1_LOW_CKIO;                // arm the low pulse
        else if (i_CKIO_PCEN && irq1_low != 0) begin
            irq1_low <= irq1_low - 1;
            irq1_n_r <= 1'b0;
            if (irq1_low == 1)
                irq1_n_r <= 1'b1;                     // release, ready for next edge
        end
    end
end
assign o_IRQ1_n = irq1_n_r;

endmodule
`default_nettype none
