`default_nettype none
`timescale 1ns/1ps
//============================================================================
// ikacore_CV1k_tb.sv - MiSTer top-level simulation testbench       [H7b.3]
//
// The FINAL-STACK board sim: the portable core top (MISTER_SDRAM +
// CV1K_NAND arms = exactly what ships inside module emu) + the 128 MB
// MiSTer module SDRAM chip model, with the DDRAM face exported at the TB
// ports where the C++ main (tb/ikacore_CV1k_tb_main.cpp) serves it with
// the ddr3_stat.h-calibrated region-mapped slave (VRAM RAM-backed, NAND +
// YMZ file-backed).  No vendor NDA models compile into this build.
//
// Clocking: ALL clocks are inputs, driven by the C++ dual-clock scheduler
// on the exact 1/614.4 MHz grid (102.4 toggles every 3 units, 153.6 every
// 2, coincident rising edges every 12 = the 51.2 MHz CKIO grid; EXTAL2
// every 9375 = exact 32768 Hz).  Sim-time scale matches tb_cv1k's
// convention (102.4 MHz clock = 10 ns period); the DDR3 slave keeps its
// latency bookkeeping in real ns internally, so the calibration is
// timescale-independent.
//
// Reset roles (C++ driven, mirroring tb_cv1k / the MiSTer framework):
//   INITRST_n at ~2 us  (pump JEDEC init runs, CPU soft-held)
//   SOFTRST_n at 205 us (-> cpu_go; the in-DUT sequencer staggers
//                        POR/RESETM; parity chosen so CKIO's first rise
//                        lands on a coincident grid edge - the in-DUT
//                        pcen23 checker enforces it)
//
// Plusargs (same semantics as tb_cv1k): +maxinsn +trace +tracefrom
// +blitdump[max] +norhex +irq2log +cycles +noirq1 +noirq2 +ndtrace
// C++ args: --vram <raw> --frame <raw> --seed <n>  (see the main)
//============================================================================
module ikacore_CV1k_tb (
    input  wire        i_CLK102,       // 102.4 MHz CPU/board/SDRAM domain
    input  wire        i_CLK153,       // 153.6 MHz blit/DDR3 domain
    input  wire        i_EXTAL2,       // RTC 32.768 kHz
    input  wire        i_INITRST_n,    // hard reset (memory subsystem)
    input  wire        i_SOFTRST_n,    // soft reset (CPU/blitter)

    // MiSTer DDRAM face -> C++ region-mapped stat slave
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

    // video face -> C++ frame capture
    output wire [15:0] o_PX,
    output wire        o_PX_DE,
    output wire        o_VSYNC,
    output wire        o_HLINE,
    output wire        o_INIT_DONE
);

    //------------------------------------------------------------------
    // DUT - the portable core top, final-stack configuration
    //------------------------------------------------------------------
    wire [12:0] sdram_a;
    wire [1:0]  sdram_ba, sdram_dqm;
    wire        sdram_ncs, sdram_nras, sdram_ncas, sdram_nwe, sdram_cke;
    wire [15:0] sdram_dq_o;
    wire        sdram_dq_oe;
    wire [15:0] sdram_dq_i;

    ikacore_CV1k dut (
        .i_EMU_CLK102M   (i_CLK102),
        .i_EMU_CLK153M   (i_CLK153),
        .i_EXTAL2        (i_EXTAL2),
        .i_EMU_INITRST_n (i_INITRST_n),
        .i_EMU_SOFTRST_n (i_SOFTRST_n),
        .i_SYS_n         (6'h3F),          // active low: nothing pressed
        .i_P1_n          (8'hFF),
        .i_P2_n          (8'hFF),
        .i_S3_TEST_n     (1'b1),
        .i_DSW_S2        (4'h0),
        .o_PX            (o_PX),
        .o_PX_DE         (o_PX_DE),
        .o_VSYNC         (o_VSYNC),
        .o_HLINE         (o_HLINE),
        .o_SND_L         (),
        .o_SND_R         (),
        .i_IOCTL_DOWNLOAD(1'b0),           // ioctl option lands at H7b.4
        .i_IOCTL_WR      (1'b0),
        .i_IOCTL_ADDR    (27'd0),
        .i_IOCTL_DATA    (8'd0),
        .i_IOCTL_INDEX   (16'd0),
        .o_IOCTL_WAIT    (),
        .o_SDRAM_A       (sdram_a),
        .o_SDRAM_BA      (sdram_ba),
        .o_SDRAM_nCS     (sdram_ncs),
        .o_SDRAM_nRAS    (sdram_nras),
        .o_SDRAM_nCAS    (sdram_ncas),
        .o_SDRAM_nWE     (sdram_nwe),
        .o_SDRAM_DQM     (sdram_dqm),
        .o_SDRAM_CKE     (sdram_cke),
        .o_SDRAM_DQ_O    (sdram_dq_o),
        .o_SDRAM_DQ_OE   (sdram_dq_oe),
        .i_SDRAM_DQ_I    (sdram_dq_i),
        .o_DDRAM_CLK     (DDRAM_CLK),
        .i_DDRAM_BUSY    (DDRAM_BUSY),
        .o_DDRAM_BURSTCNT(DDRAM_BURSTCNT),
        .o_DDRAM_ADDR    (DDRAM_ADDR),
        .i_DDRAM_DOUT    (DDRAM_DOUT),
        .i_DDRAM_DOUT_READY(DDRAM_DOUT_READY),
        .o_DDRAM_RD      (DDRAM_RD),
        .o_DDRAM_DIN     (DDRAM_DIN),
        .o_DDRAM_BE      (DDRAM_BE),
        .o_DDRAM_WE      (DDRAM_WE),
        .o_INIT_DONE     (o_INIT_DONE)
    );

    //------------------------------------------------------------------
    // 128 MB MiSTer module chip model + pad tristate (split-DQ recipe)
    //------------------------------------------------------------------
    wire [15:0] s_dq;
    assign s_dq       = sdram_dq_oe ? sdram_dq_o : 16'hzzzz;
    assign sdram_dq_i = s_dq;

    mister_128mb u_sdram (
        .clk  (i_CLK102),
        .dq   (s_dq),
        .dq_in(sdram_dq_o),
        .a(sdram_a), .ba(sdram_ba), .ncs(sdram_ncs),
        .nras(sdram_nras), .ncas(sdram_ncas), .nwe(sdram_nwe),
        .dqml(sdram_dqm[0]), .dqmh(sdram_dqm[1]),
        .cke(sdram_cke)
    );

    //------------------------------------------------------------------
    // NOR window preload (zero-time; same image + scatter as tb_cv1k).
    // +norhex=<hex> overrides for the H7b.D diag ROMs.
    //------------------------------------------------------------------
    reg [7:0] nor_bytes [0:4194303];
    initial begin
        integer pk;
        string norhex_path;
        if ($value$plusargs("norhex=%s", norhex_path)) begin
            $readmemh(norhex_path, nor_bytes);
            $display("[tb] +norhex: NOR window <- %s", norhex_path);
        end
        else begin
`ifdef IBARA_FASTBOOT
            $readmemh("roms/ibara_patched/ibara_u4_4M_fastboot.hex", nor_bytes);
`else
            $readmemh("roms/ibara_patched/ibara_u4_4M.hex", nor_bytes);
`endif
        end
        for (pk = 0; pk < 2097152; pk = pk + 1)
            u_sdram.chip0.Bank0[23'h40_0000 + pk] =
                {nor_bytes[2*pk+1], nor_bytes[2*pk]};
        $display("[tb] NOR window preloaded, [0]=%04x (expect df3d)",
                 u_sdram.chip0.Bank0[23'h40_0000]);
    end

`ifdef IBARA_FASTBOOT
    // fastboot work-RAM preload scattered into the 16-bit chip geometry
    // (32-bit word w of grid bank b -> Bank_b[{w[18:8], w[7:0], beat}])
    reg [31:0] wram0 [0:524287];
    reg [31:0] wram1 [0:524287];
    initial begin
        integer wk, ix;
        $readmemh("roms/ibara_patched/ibara_sdram_bank0.hex", wram0);
        $readmemh("roms/ibara_patched/ibara_sdram_bank1.hex", wram1);
        for (wk = 0; wk < 524288; wk = wk + 1) begin
            ix = ((wk >> 8) << 10) | ((wk & 255) << 1);
            u_sdram.chip0.Bank0[ix]   = wram0[wk][31:16];
            u_sdram.chip0.Bank0[ix+1] = wram0[wk][15:0];
            u_sdram.chip0.Bank1[ix]   = wram1[wk][31:16];
            u_sdram.chip0.Bank1[ix+1] = wram1[wk][15:0];
        end
        $display("[tb] fastboot work-RAM preloaded (grid banks 0/1)");
    end
`endif

    //------------------------------------------------------------------
    // cycle watchdog (bounds every run; +maxinsn is the primary stop)
    //------------------------------------------------------------------
    longint max_cycles = 64'd100_000_000_000;
    longint cyc = 0;
    initial void'($value$plusargs("cycles=%d", max_cycles));
    always @(posedge i_CLK102) begin
        cyc = cyc + 1;
        if (cyc >= max_cycles) begin
            $display("[tb] cycle watchdog (%0d) reached - stop", cyc);
            $finish;
        end
    end

    //------------------------------------------------------------------
    // IRQ pin-fall log (+irq2log=<file>) - same monitor as tb_cv1k; the
    // $time instants are directly comparable between the two TBs only in
    // cadence (different reset instants), but within THIS TB they are the
    // exact CKIO-rise-anchored IRQ times.
    //------------------------------------------------------------------
    integer i2l_fd = 0;
    reg     i2l_irq2_d = 1'b1, i2l_irq1_d = 1'b1;
    initial begin
        string i2l_file;
        if ($value$plusargs("irq2log=%s", i2l_file))
            i2l_fd = $fopen(i2l_file, "w");
    end
    always @(posedge i_CLK102) if (i2l_fd != 0) begin
        if (i2l_irq2_d && !dut.pth_irq2_n) $fdisplay(i2l_fd, "IRQ2 %0t", $time);
        if (i2l_irq1_d && !dut.pth_irq1_n) $fdisplay(i2l_fd, "IRQ1 %0t", $time);
        i2l_irq2_d <= dut.pth_irq2_n;
        i2l_irq1_d <= dut.pth_irq1_n;
    end

    //------------------------------------------------------------------
    // board blit dump (+blitdump=<file> [+blitdumpmax=N]) - backdoor op
    // list walk out of the module SDRAM on every EXEC, tb_cv1k's MISTER
    // flavour.  Feed to blitgold --boardtrace (accept path).
    //------------------------------------------------------------------
    integer bd_fd     = 0;
    integer bd_done   = 0;
    integer bd_max    = 4;
    reg     bd_on     = 1'b0;
    reg     bd_exec_d = 1'b0;
    string  bd_file;

    initial begin
        if ($value$plusargs("blitdump=%s", bd_file)) begin
            void'($value$plusargs("blitdumpmax=%d", bd_max));
            bd_fd = $fopen(bd_file, "w");
            $fdisplay(bd_fd, "# ikacore board blit dump (op words, one hex per line)");
            bd_on = (bd_fd != 0);
        end
    end

    function automatic [15:0] bd_word(input [31:0] a);
        // 16-bit chip geometry: halfword index = row*1024 + col*2 + beat
        integer ix;
        begin
            ix = (a[20:10] * 1024) + (a[9:2] * 2) + a[1];
            case (a[22:21])
                2'd0: bd_word = u_sdram.chip0.Bank0[ix];
                2'd1: bd_word = u_sdram.chip0.Bank1[ix];
                2'd2: bd_word = u_sdram.chip0.Bank2[ix];
                2'd3: bd_word = u_sdram.chip0.Bank3[ix];
            endcase
        end
    endfunction

    task automatic bd_dump_exec;
        reg [31:0] a;
        reg [15:0] w0;
        integer    dimx, dimy, n, k;
        reg        stop;
        begin
            a = {3'b0, dut.u_blit.u_blit_regs.o_list_addr};
            $fdisplay(bd_fd, "EXEC frame=%0d addr=%07x clip=%0d,%0d scroll=%0d,%0d",
                      bd_done, a,
                      dut.u_blit.u_blit_regs.o_clip_x,   dut.u_blit.u_blit_regs.o_clip_y,
                      dut.u_blit.u_blit_regs.o_scroll_x, dut.u_blit.u_blit_regs.o_scroll_y);
            stop = 1'b0;
            for (k = 0; (k < 4*1024*1024) && !stop; k = k + 1) begin
                w0 = bd_word(a);
                $fdisplay(bd_fd, "%04x", w0);
                case (w0[15:12])
                    4'h0, 4'hf: stop = 1'b1;                              // EXIT
                    4'hc: begin                                          // CLIP
                        $fdisplay(bd_fd, "%04x", bd_word(a + 32'd2));
                        a = a + 32'd4;
                    end
                    4'h2: begin                                          // UPLOAD
                        dimx = (bd_word(a + 32'd12) & 16'h1fff) + 1;
                        dimy = (bd_word(a + 32'd14) & 16'h0fff) + 1;
                        n = 8 + dimx * dimy;
                        for (int j = 1; j < n; j = j + 1)
                            $fdisplay(bd_fd, "%04x", bd_word(a + 32'(2*j)));
                        a = a + 32'(2*n);
                    end
                    4'h1: begin                                          // DRAW
                        for (int j = 1; j < 10; j = j + 1)
                            $fdisplay(bd_fd, "%04x", bd_word(a + 32'(2*j)));
                        a = a + 32'd20;
                    end
                    default: stop = 1'b1;
                endcase
            end
        end
    endtask

    // EXEC pulse lives in the blit domain (153.6)
    always @(posedge i_CLK153) begin
        bd_exec_d <= dut.u_blit.u_blit_regs.o_exec;
        if (bd_on && dut.u_blit.u_blit_regs.o_exec && !bd_exec_d) begin
            bd_dump_exec();
            bd_done = bd_done + 1;
            if (bd_done >= bd_max) begin
                $fclose(bd_fd);
                bd_on = 1'b0;
                $display("[blitdump] %0d execs dumped to %s", bd_done, bd_file);
            end
        end
    end

    //------------------------------------------------------------------
    // H7 descriptor-sideband footprint checker (always on, blit domain)
    //------------------------------------------------------------------
    blit_dsc_check u_dsc_check (
        .i_CLK          (i_CLK153),
        .i_RST_n        (dut.i_POR_n),
        .i_dsc_vld      (dut.u_blit.o_dsc_vld),
        .i_dsc_sx_lo    (dut.u_blit.o_dsc_sx_lo),
        .i_dsc_sx_hi    (dut.u_blit.o_dsc_sx_hi),
        .i_dsc_sy0      (dut.u_blit.o_dsc_sy0),
        .i_dsc_rows     (dut.u_blit.o_dsc_rows),
        .i_dsc_npx      (dut.u_blit.o_dsc_npx),
        .i_dsc_dst0     (dut.u_blit.o_dsc_dst0),
        .i_dsc_flipy    (dut.u_blit.o_dsc_flipy),
        .i_dsc_upl      (dut.u_blit.o_dsc_upl),
        .i_dsc_upl_addr (dut.u_blit.o_dsc_upl_addr),
        .i_dsc_upl_dimx (dut.u_blit.o_dsc_upl_dimx),
        .i_dsc_upl_dimy (dut.u_blit.o_dsc_upl_dimy),
        .i_srd_req      (dut.bv_srd_req),
        .i_srd_addr     (dut.bv_srd_addr),
        .i_wr_req       (dut.bv_wr_req),
        .i_wr_addr      (dut.bv_wr_addr),
        .i_wr_mask      (dut.bv_wr_mask)
    );

    //------------------------------------------------------------------
    // end-of-run stack state (drain forensics)
    //------------------------------------------------------------------
    longint tb_rdy_cnt = 0, tb_rd_cmd = 0, tb_rd_words_cmd = 0;
    longint tb_last_rd_t = 0, tb_last_rdy_t = 0, tb_last_we_t = 0;
    reg [28:0] tb_last_rd_a = 0;
    reg [7:0]  tb_last_rd_b = 0;
    always @(posedge i_CLK153) begin
        if (DDRAM_DOUT_READY) begin
            tb_rdy_cnt <= tb_rdy_cnt + 1;
            tb_last_rdy_t <= longint'($time);
        end
        if (DDRAM_RD && !DDRAM_BUSY) begin
            tb_rd_cmd <= tb_rd_cmd + 1;
            tb_rd_words_cmd <= tb_rd_words_cmd + longint'(DDRAM_BURSTCNT);
            tb_last_rd_t <= longint'($time);
            tb_last_rd_a <= DDRAM_ADDR;
            tb_last_rd_b <= DDRAM_BURSTCNT;
        end
        if (DDRAM_WE && !DDRAM_BUSY) tb_last_we_t <= longint'($time);
    end
    final $display("[tb] face census: rd cmds=%0d words_cmd=%0d rdy_words_seen=%0d | last rd @%0d ns addr=%07x cnt=%0d, last rdy @%0d, last we @%0d",
                   tb_rd_cmd, tb_rd_words_cmd, tb_rdy_cnt,
                   tb_last_rd_t, tb_last_rd_a, tb_last_rd_b, tb_last_rdy_t, tb_last_we_t);

    // +prdlog: first 60 batch read requests (train-shape forensics - a
    // stream of len=2 requests means an op fell into the strict/beat path,
    // which is latency-bound on real DDR3 timing; see diag_mini.c's
    // draw_clear note for the wrap-overlap case this caught)
    integer prd_n = 0;
    reg prdlog = 1'b0;
    initial prdlog = $test$plusargs("prdlog");
    always @(posedge i_CLK153) if (prdlog && dut.prd_req && dut.prd_rdy && prd_n < 60) begin
        $display("[prd] #%0d t=%0t addr=%06x len=%0d", prd_n, $time, dut.prd_addr, dut.prd_len);
        prd_n <= prd_n + 1;
    end
    final begin
        $display("[tb] end state: bat idle=%b wr_idle=%b op_srv=%b | harness own=%0d oq_head_v=%b oq_left=%0d vid_pend=%b vid_seg=%0d bp_v=%b np_v=%b bat_busy=%b | pwr_req=%b pwr_rdy=%b wr_train=%b | DDRAM WE=%b RD=%b BUSY=%b",
                 dut.u_batch.o_idle, dut.u_batch.o_wr_idle, dut.u_batch.o_op_srv,
                 dut.u_harness.own, dut.u_harness.oq_head_v, dut.u_harness.oq_left,
                 dut.u_harness.vid_pend, dut.u_harness.vid_seg,
                 dut.u_harness.bp_v, dut.u_harness.np_v, dut.u_harness.bat_busy,
                 dut.pwr_req, dut.pwr_rdy, dut.wr_train,
                 DDRAM_WE, DDRAM_RD, DDRAM_BUSY);
    end

endmodule

// retired-instruction probe inside the (read-only) core - +trace/+maxinsn
// (same bind as tb_cv1k; the two TBs are separate builds)
bind cpu_core cpu_tracer u_trace (
    .i_CLK   (i_CLK),
    .i_CEN   (i_CEN),
    .i_RST_n (i_RST_n),
    .valid   (dbg_o_RETIRE_VALID),
    .pc      (dbg_o_RETIRE_PC),
    .inst    (dbg_o_RETIRE_INST),
    .gpr_we  (dbg_o_RETIRE_GPR_WE),
    .gpr     (dbg_o_RETIRE_GPR),
    .gpr_data(dbg_o_RETIRE_GPR_DATA),
    .fetch_pc(dbg_o_FETCH_PC)
);
`default_nettype none
