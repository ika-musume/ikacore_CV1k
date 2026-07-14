`default_nettype none
`timescale 1ns/1ps
//============================================================================
// tb_cv1k - top-level board simulation testbench for ikacore_CV1k
//
// Drives the SH-3 architectural clock + reset, loads the U4 program flash, and
// lets the CPU boot from 0xA0000000. The bound cpu_tracer streams the retired
// instruction flow to build/trace_rtl.txt for comparison with the MAME SH-3.
//
// Plusargs:  +rom=<hex>  +trace=<file>  +maxinsn=<n>  +cycles=<n>
//============================================================================
module tb_cv1k;

    reg i_CLK   = 1'b0;
    reg i_CEN   = 1'b1;
    reg i_POR_n = 1'b0;
    reg i_RST_n = 1'b0;
    reg i_EXTAL2= 1'b0;

    // ~102.4 MHz architectural clock (period scaled to 10 ns sim units)
    always #5 i_CLK = ~i_CLK;

    // RTC 32.768 kHz crystal - slow, asynchronous; divide the main clock down
    integer rtc_div = 0;
    always @(posedge i_CLK) begin
        rtc_div = rtc_div + 1;
        if (rtc_div >= 1560) begin rtc_div = 0; i_EXTAL2 = ~i_EXTAL2; end
    end

    // reset release: the MX29LV320E model gates reads until its Tvcs=200us Vcc
    // setup elapses (the board's MAX690S supervisor holds the SH-3 in reset the
    // same way), so keep the CPU in reset until the flash is powered up.
    initial begin
        i_POR_n = 1'b0; i_RST_n = 1'b0;
        #205_000;                          // > Tvcs (200 us) so the flash answers fetch 0
        if ($test$plusargs("ioctl_test")) begin
            $display("[tb] ioctl_test: CPU held in reset for the download");
        end else begin
            @(posedge i_CLK) i_POR_n = 1'b1;
            repeat (8) @(posedge i_CLK);
            i_RST_n = 1'b1;
            $display("[tb] reset released @ %0t (flash powered up)", $time);
        end
    end

    // watchdog: bound number of clock cycles so the run always terminates.
    // 64-bit: long runs (+maxinsn in the tens of millions) need >2^31 cycles.
    // Default is effectively unbounded so +maxinsn alone controls the stop.
    longint max_cycles = 64'd100_000_000_000;
    longint cyc = 0;
    initial void'($value$plusargs("cycles=%d", max_cycles));
    always @(posedge i_CLK) begin
        cyc = cyc + 1;
        if (cyc >= max_cycles) begin
            $display("[tb] cycle watchdog (%0d) reached - stop", cyc);
            $finish;
        end
    end

    // DUT: the CV1000-B PCB
    ikacore_CV1k #(
`ifdef IBARA_FASTBOOT
        .ROM_FILE("roms/ibara_patched/ibara_u4_4M_fastboot.hex")   // copy/FPGA/delay loops NOP'd; SDRAM preloaded
`else
        .ROM_FILE("roms/ibara_patched/ibara_u4_4M.hex")
`endif
    ) dut (
        .i_CLK   (i_CLK),
        .i_CEN   (i_CEN),
        .i_POR_n (i_POR_n),
        .i_RST_n (i_RST_n),
        .i_EXTAL2(i_EXTAL2)
`ifdef MISTER_SDRAM
        ,
        .i_MEM_RST_n     (mem_rst_n),
        .i_IOCTL_DOWNLOAD(ioctl_download),
        .i_IOCTL_WR      (ioctl_wr),
        .i_IOCTL_ADDR    (ioctl_addr),
        .i_IOCTL_DATA    (ioctl_data),
        .i_IOCTL_INDEX   (ioctl_index),
        .o_IOCTL_WAIT    (ioctl_wait)
`endif
    );

`ifdef MISTER_SDRAM
    //------------------------------------------------------------------
    // MiSTer memory subsystem: pump reset, HPS ioctl emulator, preloads
    //------------------------------------------------------------------
    reg  mem_rst_n   = 1'b0;
    reg  ioctl_start = 1'b0;
    wire ioctl_download, ioctl_wr, ioctl_wait, ioctl_done;
    wire [26:0] ioctl_addr;
    wire [7:0]  ioctl_data;
    wire [15:0] ioctl_index;

    ioctl_sim u_ioctl (
        .i_CLK(i_CLK), .i_START(ioctl_start),
        .o_DOWNLOAD(ioctl_download), .o_ADDR(ioctl_addr), .o_DATA(ioctl_data),
        .o_WR(ioctl_wr), .o_INDEX(ioctl_index),
        .i_WAIT(ioctl_wait), .o_DONE(ioctl_done)
    );

    // release the memory subsystem well before the CPU POR (205 us): the
    // pump's JEDEC init takes ~3 us and must finish before the first fetch
    initial begin
        #2_000;
        @(posedge i_CLK) mem_rst_n = 1'b1;
    end

    // Default mode: zero-time NOR-window preload with the same image the
    // MX29LV320E served in the baseline build, so boot content and timing
    // are diffable bit-for-bit.  Flash hex = one byte per line in raw file
    // order; the SH-3 bus halfword is {odd byte, even byte}.
    reg [7:0] nor_bytes [0:4194303];
    initial if (!$test$plusargs("ioctl_test")) begin
        integer pk;
`ifdef IBARA_FASTBOOT
        $readmemh("roms/ibara_patched/ibara_u4_4M_fastboot.hex", nor_bytes);
`else
        $readmemh("roms/ibara_patched/ibara_u4_4M.hex", nor_bytes);
`endif
        for (pk = 0; pk < 2097152; pk = pk + 1)
            dut.u_sdram.chip0.Bank0[23'h40_0000 + pk] =
                {nor_bytes[2*pk+1], nor_bytes[2*pk]};
        $display("[tb] MISTER: NOR window preloaded (2M halfwords), [0]=%04x (expect df3d)",
                 dut.u_sdram.chip0.Bank0[23'h40_0000]);
    end

`ifdef IBARA_FASTBOOT
    // fastboot work-RAM preload scattered into the 16-bit chip geometry:
    // 32-bit word w of grid bank b -> Bank_b[{w[18:8], w[7:0], beat}]
    reg [31:0] wram0 [0:524287];
    reg [31:0] wram1 [0:524287];
    initial begin
        integer wk, ix;
        $readmemh("roms/ibara_patched/ibara_sdram_bank0.hex", wram0);
        $readmemh("roms/ibara_patched/ibara_sdram_bank1.hex", wram1);
        for (wk = 0; wk < 524288; wk = wk + 1) begin
            ix = ((wk >> 8) << 10) | ((wk & 255) << 1);
            dut.u_sdram.chip0.Bank0[ix]   = wram0[wk][31:16];
            dut.u_sdram.chip0.Bank0[ix+1] = wram0[wk][15:0];
            dut.u_sdram.chip0.Bank1[ix]   = wram1[wk][31:16];
            dut.u_sdram.chip0.Bank1[ix+1] = wram1[wk][15:0];
        end
        $display("[tb] MISTER: fastboot work-RAM preloaded (grid banks 0/1)");
        $display("[tb] fb check: wram0[0xA8C]=%08x Bank0[0x2918..9]=%04x %04x (expect 4f030002 / 4f03 0002)",
                 wram0[19'h0A8C],
                 dut.u_sdram.chip0.Bank0[23'h02918], dut.u_sdram.chip0.Bank0[23'h02919]);
    end
`endif

    // ioctl smoke test: +ioctl_test [+ioctl_bytes=N] [+ioctl_bin=<raw dump>]
    // CPU stays in reset; the file is streamed through the pump's loader and
    // the NOR window is verified against the file afterwards.
    initial if ($test$plusargs("ioctl_test")) begin
        integer f, bb;
        longint k, nhw, errs, tst_bytes;
        reg [7:0]  lo8;
        reg [15:0] expd, got;
        string p2;
        tst_bytes = 65536; p2 = "roms/ibara/u4";
        void'($value$plusargs("ioctl_bytes=%d", tst_bytes));
        void'($value$plusargs("ioctl_bin=%s", p2));
        #12_000;                                   // pump init complete
        @(posedge i_CLK) ioctl_start = 1'b1;
        @(posedge ioctl_done);
        repeat (64) @(posedge i_CLK);
        f = $fopen(p2, "rb");
        nhw = tst_bytes / 2; errs = 0;
        for (k = 0; k < nhw; k = k + 1) begin
            bb = $fgetc(f); lo8 = bb[7:0];
            bb = $fgetc(f);
            expd = {bb[7:0], lo8};
            got  = dut.u_sdram.chip0.Bank0[23'h40_0000 + k[22:0]];
            if (got !== expd) begin
                errs = errs + 1;
                if (errs <= 10)
                    $display("[ioctl_test] MISMATCH hw %0d: got %04x expected %04x", k, got, expd);
            end
        end
        $fclose(f);
        if (errs == 0) $display("[ioctl_test] PASS: %0d halfwords verified", nhw);
        else           $display("[ioctl_test] FAIL: %0d/%0d mismatches", errs, nhw);
        $finish;
    end
`endif

    // optional shared-bus probe (+dbg=1): log the first external bus cycles
    // (PCB-level nets only, so it is independent of the read-only IP internals)
    integer dbg_bus = 0;
    integer nprobe  = 0;
    initial void'($value$plusargs("dbg=%d", dbg_bus));
    always @(posedge i_CLK) if ((dbg_bus != 0) && i_RST_n && nprobe < 80) begin
        if (dut.CS0_n === 1'b0 || dut.CS3_n === 1'b0) begin
            $display("[bus] t=%0t CKIO=%b A=%07x CS0=%b CS3=%b RD=%b RDWR=%b WEn=%b D=%08x D_OE=%b",
                     $time, dut.CKIO, dut.A, dut.CS0_n, dut.CS3_n, dut.RD_n,
                     dut.RD_WR, dut.WE_n, dut.D, dut.D_OE);
            nprobe = nprobe + 1;
        end
    end

    // optional DMA/NAND timing probe (+define+DMA_MON): segments the U2 read
    // stream into bursts (a gap of >64 i_CLK cycles between RE_n pulses starts
    // a new burst) and reports pulse count + span, plus SDRAM write commands.
`ifdef DMA_MON
    longint ckio_cyc = 0;
    always @(posedge i_CLK) if (dut.CKIO_PCEN) ckio_cyc = ckio_cyc + 1;

    longint sdram_wr = 0;
    always @(posedge i_CLK) if (dut.CKIO_PCEN)
        if (!dut.CS3_n && dut.RAS3L_n && !dut.CASL_n && !dut.RD_WR)
            sdram_wr = sdram_wr + 1;

    reg     re_d    = 1'b1;
    longint re_tot  = 0;                 // every RE_n fall since reset
    longint burst_n = 0;                 // RE_n falls in the current burst
    longint burst_t0 = 0, burst_t1 = 0, burst_c0 = 0, burst_c1 = 0;
    longint bursts  = 0;

    task automatic flush_burst;
        if (burst_n != 0) begin
            bursts = bursts + 1;
            $display("[dma] burst %0d: %0d RE pulses, %0d i_CLK cyc (%0d CKIO cyc), %0d i_CLK/byte",
                     bursts, burst_n, burst_t1 - burst_t0, burst_c1 - burst_c0,
                     (burst_t1 - burst_t0) / burst_n);
            burst_n = 0;
        end
    endtask

    always @(posedge i_CLK) begin
        re_d <= dut.u2_re_n;
        if (burst_n != 0 && (cyc - burst_t1) > 64) flush_burst();
        if (re_d && !dut.u2_re_n) begin              // RE_n falling edge = 1 byte out
            if (burst_n == 0) begin burst_t0 = cyc; burst_c0 = ckio_cyc; end
            burst_n = burst_n + 1;
            re_tot  = re_tot + 1;
            burst_t1 = cyc; burst_c1 = ckio_cyc;
        end
    end

    // capture the bytes U2 actually drives (valid on RE_n rising) so they can be
    // diffed against the raw dump:  cmp build/nand_bytes.bin roms/ibara/u2
    integer nb_fd = 0, nb_cnt = 0;
    integer nb_max = 0;
    initial void'($value$plusargs("nandbytes=%d", nb_max));
    initial if (nb_max != 0) nb_fd = $fopen("build/nand_bytes.bin", "wb");
    always @(posedge i_CLK) if (!re_d && dut.u2_re_n && nb_fd != 0 && nb_cnt < nb_max) begin
        $fwrite(nb_fd, "%c", dut.D[7:0]);
        nb_cnt = nb_cnt + 1;
        if (nb_cnt == nb_max) begin $fclose(nb_fd); nb_fd = 0; $display("[dma] captured %0d NAND bytes", nb_cnt); end
    end

    final begin
        $display("[dma] TOTAL: %0d NAND RE pulses, %0d bursts, %0d SDRAM writes, %0d i_CLK cyc, %0d CKIO cyc",
                 re_tot, bursts, sdram_wr, cyc, ckio_cyc);
    end
`endif

    //========================================================================
    // H1b - board blit dump: backdoor list walk on every EXEC.
    //
    // On each blit_regs EXEC pulse, walk the op list straight out of the U1
    // SDRAM model at the latched LIST_ADDR (no bus fetch - that is H2's job)
    // and emit a text trace of the exact op-word stream + clip/scroll.  Feed it
    // to sim/blitgold (`blitgold --boardtrace <file>`) to render/diff OUR OWN
    // HS3+blit_regs output through the golden pixel model - closing H0<->H1.
    //
    // Enable: +blitdump=<file> [+blitdumpmax=<n execs>].  Off by default.
    //
    // SDRAM address map (verified by the FASTBOOT preload, make_fastboot.py):
    //   bank = P[22:21],  index = P[20:2],  Bank[index] = big-endian longword,
    //   word(P) = P[1] ? LW[15:0] : LW[31:16].
    //========================================================================
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

    // 16-bit op word at physical work-RAM address `a`, read from the SDRAM model.
    function automatic [15:0] bd_word(input [31:0] a);
`ifdef MISTER_SDRAM
        // 16-bit chip geometry: halfword index = row*1024 + col*2 + beat
        integer ix;
        begin
            ix = (a[20:10] * 1024) + (a[9:2] * 2) + a[1];
            case (a[22:21])
                2'd0: bd_word = dut.u_sdram.chip0.Bank0[ix];
                2'd1: bd_word = dut.u_sdram.chip0.Bank1[ix];
                2'd2: bd_word = dut.u_sdram.chip0.Bank2[ix];
                2'd3: bd_word = dut.u_sdram.chip0.Bank3[ix];
            endcase
        end
`else
        reg [31:0] lw;
        begin
            case (a[22:21])
                2'd0: lw = dut.u_u1_sdram.Bank0[a[20:2]];
                2'd1: lw = dut.u_u1_sdram.Bank1[a[20:2]];
                2'd2: lw = dut.u_u1_sdram.Bank2[a[20:2]];
                2'd3: lw = dut.u_u1_sdram.Bank3[a[20:2]];
            endcase
            bd_word = a[1] ? lw[15:0] : lw[31:16];
        end
`endif
    endfunction

    // Walk one op list (mirrors gfx_exec) and emit each word until EXIT.
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
                    4'hc: begin                                          // CLIP: op + cliptype
                        $fdisplay(bd_fd, "%04x", bd_word(a + 32'd2));
                        a = a + 32'd4;
                    end
                    4'h2: begin                                          // UPLOAD: 8 hdr + w*h
                        dimx = (bd_word(a + 32'd12) & 16'h1fff) + 1;
                        dimy = (bd_word(a + 32'd14) & 16'h0fff) + 1;
                        n = 8 + dimx * dimy;
                        for (int j = 1; j < n; j = j + 1)
                            $fdisplay(bd_fd, "%04x", bd_word(a + 32'(2*j)));
                        a = a + 32'(2*n);
                    end
                    4'h1: begin                                          // DRAW: 10 words
                        for (int j = 1; j < 10; j = j + 1)
                            $fdisplay(bd_fd, "%04x", bd_word(a + 32'(2*j)));
                        a = a + 32'd20;
                    end
                    default: stop = 1'b1;                                 // unknown
                endcase
            end
        end
    endtask

    always @(posedge i_CLK) begin
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

    //========================================================================
    // H2 - attribute-FIFO drain log: the words the fetch unit REALLY read
    // over the BREQ/BACK bus, in pop order, same format as +blitdump.
    // Accept: diff -q build/board_blit.txt build/board_fifo.txt -> clean.
    //
    // Enable: +blitfifo=<file> [+blitfifomax=<n execs>].  Off by default.
    // (H3: the draw engine is the FIFO consumer now - the log triggers on
    // its actual pops, and closes on ITS done, which trails the fetch's.)
    //========================================================================
    integer bf_fd   = 0;
    integer bf_done = 0;
    integer bf_max  = 4;
    reg     bf_on   = 1'b0;
    string  bf_file;

    initial begin
        if ($value$plusargs("blitfifo=%s", bf_file)) begin
            void'($value$plusargs("blitfifomax=%d", bf_max));
            bf_fd = $fopen(bf_file, "w");
            $fdisplay(bf_fd, "# ikacore board blit dump (op words, one hex per line)");
            bf_on = (bf_fd != 0);
        end
    end

    always @(posedge i_CLK) begin
        if (bf_on && dut.u_blit.u_blit_regs.o_exec && !bd_exec_d) begin
            $fdisplay(bf_fd, "EXEC frame=%0d addr=%07x clip=%0d,%0d scroll=%0d,%0d",
                      bf_done, {3'b000, dut.u_blit.u_blit_regs.o_list_addr},
                      dut.u_blit.u_blit_regs.o_clip_x,   dut.u_blit.u_blit_regs.o_clip_y,
                      dut.u_blit.u_blit_regs.o_scroll_x, dut.u_blit.u_blit_regs.o_scroll_y);
            bf_done = bf_done + 1;
        end
        if (bf_on && dut.u_blit.u_blit_fetch.o_fifo_valid && dut.u_blit.blit_fifo_pop)
            $fdisplay(bf_fd, "%04x", dut.u_blit.u_blit_fetch.o_fifo_word);
        if (bf_on && bf_done >= bf_max && dut.u_blit.u_blit_draw.o_done) begin
            $fclose(bf_fd);
            bf_on = 1'b0;
            $display("[blitfifo] %0d execs logged to %s", bf_done, bf_file);
        end
    end

    //========================================================================
    // H3 debug - +blitbusy: log every fetch/draw busy transition with the
    // draw engine's FRONT/BACK FSM states (stuck-busy triage).
    //========================================================================
    reg       bb_on = 1'b0;
    reg [1:0] bb_d  = 2'b00;
    initial if ($test$plusargs("blitbusy")) bb_on = 1'b1;
    always @(posedge i_CLK) begin
        if (bb_on && ({dut.u_blit.u_blit_fetch.o_busy, dut.u_blit.u_blit_draw.o_busy} != bb_d)) begin
            $display("[blitbusy] t=%0t fetch=%b draw=%b fst=%0d bst=%0d fifo_v=%b ob_v=%b",
                     $time, dut.u_blit.u_blit_fetch.o_busy, dut.u_blit.u_blit_draw.o_busy,
                     dut.u_blit.u_blit_draw.fst, dut.u_blit.u_blit_draw.bst,
                     dut.u_blit.blit_fifo_valid, dut.u_blit.u_blit_draw.ob_v);
            bb_d <= {dut.u_blit.u_blit_fetch.o_busy, dut.u_blit.u_blit_draw.o_busy};
        end
    end

    //========================================================================
    // H3 debug - +blitirq1: log every IRQ1 pin fall + governor retire pulse,
    // with the game's IRQ1 callback slot (0c002224, via the SDRAM backdoor)
    // so "delivered but unregistered" vs "never fired" is decidable.
    //========================================================================
    reg bi_on = 1'b0;
    reg bi_d  = 1'b1;
    initial if ($test$plusargs("blitirq1")) bi_on = 1'b1;
    always @(posedge i_CLK) begin
        if (bi_on) begin
            if (bi_d && !dut.pth_irq1_n)
`ifdef MISTER_SDRAM
                $display("[blitirq1] t=%0t IRQ1 fall, callback@0c002224=%08x",
                         $time, {dut.u_sdram.chip0.Bank0[23'h02112],
                                 dut.u_sdram.chip0.Bank0[23'h02113]});
`else
                $display("[blitirq1] t=%0t IRQ1 fall, callback@0c002224=%08x",
                         $time, dut.u_u1_sdram.Bank0[19'h00889]);
`endif
            bi_d <= dut.pth_irq1_n;
            if (dut.u_blit.blit_gov_retire)
                $display("[blitirq1] t=%0t gov retire", $time);
        end
    end

    //========================================================================
    // H4 - governor anchor injection: +blitanchor [+blitanchorat=<ckio>]
    //
    // Backdoor-writes three op lists into unused work RAM (0x0C7C0000+, top
    // MB below the boot stack), fires backdoor EXECs through blit_regs (so
    // the real fetch unit bus-masters them and the governor times them), and
    // checks the H4 anchors:
    //   A. per-draw costs 8x8=93 / 16x12=189 / 240x64=12,090 VCLK
    //      (the 240x64@768 anchor moved to (64,0): same &31 phases = same
    //      cost, inside the injected clip window)
    //   B. 80x fully-clipped 1x324 draws: busy_end(model) ~17.5 us +-3%
    //   C. 256x5 UPLOAD: busy_end(model) ~58.77 us +-3%
    // busy_end(model) = the governor's engine_free at END pop (the
    // C++-comparable number, cost_model.h validate_anchors); the honest
    // BUSY deassert additionally trails by the END-chunk fetch.
    //
    // Run standalone with +noirq1 (injected retires would otherwise fire
    // IRQ1 into the booting game before its handler is armed):
    //   ./Vtb_cv1k +blitanchor +noirq1
    // Injects after the BSC is programmed and only while the blitter is
    // quiet; $finish's when done.
    //========================================================================
    integer ba_on   = 0;
    longint ba_at   = 600_000;           // CKIO trigger (past BSC init)
    longint ba_ckio = 0;
    integer ba_fail = 0;
    int     ba_kind[$];                  // per-op governor records this run
    int     ba_cost[$];

    initial begin
        if ($test$plusargs("blitanchor")) begin
            ba_on = 1;
            void'($value$plusargs("blitanchorat=%d", ba_at));
        end
    end

    always @(posedge i_CLK) if (dut.CKIO_PCEN) ba_ckio = ba_ckio + 1;

    always @(posedge i_CLK) if (ba_on != 0 && dut.u_blit.u_blit_gov.o_dbg_vld) begin
        ba_kind.push_back(int'(dut.u_blit.u_blit_gov.o_dbg_kind));
        ba_cost.push_back(int'(dut.u_blit.u_blit_gov.o_dbg_cost));
    end

    // bus-quiet detector: consecutive CKIO with no CPU CS0/CS3 activity.
    // The fetch-bound anchors assume the paced chunk cadence (grant ~ 0),
    // which is the authentic measurement scenario - a game polling STATUS
    // out of cache.  Injecting while the boot hammers the bus makes chunk
    // spacing tenure-limited (grant latency = P-23, unmeasured until M-4)
    // and would time the BSC, not the governor.
    longint ba_quiet = 0;
    always @(posedge i_CLK) if (dut.CKIO_PCEN) begin
        if (dut.CS0_n === 1'b0 || dut.CS3_n === 1'b0) ba_quiet = 0;
        else ba_quiet = ba_quiet + 1;
    end

    // BREQ-to-BREQ spacing probe (grant-latency diagnostics for the fetch-
    // bound anchors; P-23 is unmeasured, M-4)
    reg     ba_breq_d = 1'b1;
    longint ba_breq_t = 0, ba_breq_n = 0, ba_breq_sum = 0, ba_breq_max = 0;
    always @(posedge i_CLK) if (ba_on != 0) begin
        ba_breq_d <= dut.blit_breq_n;
        if (ba_breq_d && !dut.blit_breq_n) begin
            if (ba_breq_t != 0) begin
                ba_breq_n   = ba_breq_n + 1;
                ba_breq_sum = ba_breq_sum + (ba_ckio - ba_breq_t);
                if (ba_ckio - ba_breq_t > ba_breq_max) ba_breq_max = ba_ckio - ba_breq_t;
            end
            ba_breq_t = ba_ckio;
        end
    end

    // write one 16-bit op word into the U1 SDRAM model (bd_word's inverse)
    task automatic ba_wr16(input [31:0] a, input [15:0] w);
`ifdef MISTER_SDRAM
        integer ix;
        begin
            ix = (a[20:10] * 1024) + (a[9:2] * 2) + a[1];
            case (a[22:21])
                2'd0: dut.u_sdram.chip0.Bank0[ix] = w;
                2'd1: dut.u_sdram.chip0.Bank1[ix] = w;
                2'd2: dut.u_sdram.chip0.Bank2[ix] = w;
                2'd3: dut.u_sdram.chip0.Bank3[ix] = w;
            endcase
        end
`else
        reg [31:0] lw;
        begin
            case (a[22:21])
                2'd0: lw = dut.u_u1_sdram.Bank0[a[20:2]];
                2'd1: lw = dut.u_u1_sdram.Bank1[a[20:2]];
                2'd2: lw = dut.u_u1_sdram.Bank2[a[20:2]];
                2'd3: lw = dut.u_u1_sdram.Bank3[a[20:2]];
            endcase
            if (a[1]) lw[15:0]  = w;
            else      lw[31:16] = w;
            case (a[22:21])
                2'd0: dut.u_u1_sdram.Bank0[a[20:2]] = lw;
                2'd1: dut.u_u1_sdram.Bank1[a[20:2]] = lw;
                2'd2: dut.u_u1_sdram.Bank2[a[20:2]] = lw;
                2'd3: dut.u_u1_sdram.Bank3[a[20:2]] = lw;
            endcase
        end
`endif
    endtask

    reg [31:0] ba_cur;                   // sequential list-writer cursor
    task automatic ba_w(input [15:0] w);
        begin ba_wr16(ba_cur, w); ba_cur = ba_cur + 32'd2; end
    endtask
    task automatic ba_draw(input [15:0] sx, input [15:0] sy,
                           input [15:0] dx, input [15:0] dy,
                           input int dimx, input int dimy);
        begin
            ba_w(16'h1000); ba_w(16'h0000);
            ba_w(sx); ba_w(sy); ba_w(dx); ba_w(dy);
            ba_w(16'(dimx - 1)); ba_w(16'(dimy - 1));
            ba_w(16'h0080); ba_w(16'h8080);
        end
    endtask

    // inject one EXEC through the blit_regs backdoor, wait for the governed
    // retire, return busy_end(model) in us (half-VCLK tick = 1/153.6 us/1000)
    task automatic ba_run(input [28:0] list, input [15:0] cx, input [15:0] cy,
                          output real end_us);
        begin
            ba_kind.delete();
            ba_cost.delete();
            // wait for the WHOLE blitter to idle (gov+fetch+draw = the
            // composite STATUS busy the real game polls before EXEC; the
            // governed deassert can precede the fetch's END-chunk drain)
            wait (dut.u_blit.u_blit_gov.o_busy   == 1'b0 &&
                  dut.u_blit.u_blit_fetch.o_busy == 1'b0 &&
                  dut.u_blit.u_blit_draw.o_busy  == 1'b0);
            @(negedge i_CLK);
            dut.u_blit.u_blit_regs.bd_list     = list;
            dut.u_blit.u_blit_regs.bd_clip_x   = cx;
            dut.u_blit.u_blit_regs.bd_clip_y   = cy;
            dut.u_blit.u_blit_regs.bd_exec_req = 1'b1;
            wait (dut.u_blit.u_blit_gov.o_busy == 1'b1);
            ba_breq_t = 0; ba_breq_n = 0; ba_breq_sum = 0; ba_breq_max = 0;
            wait (dut.u_blit.u_blit_gov.o_busy == 1'b0);
            @(negedge i_CLK);
            end_us = real'(dut.u_blit.u_blit_gov.r_busy_end) / 153.6;
            if (ba_breq_n != 0)
                $display("[blitanchor]   chunk spacing: n=%0d avg=%.1f max=%0d CKIO (pace 36)",
                         ba_breq_n, real'(ba_breq_sum) / real'(ba_breq_n), ba_breq_max);
        end
    endtask

    task automatic ba_check(input string tag, input real got_us,
                            input real want_us);
        begin
            if (got_us >= want_us * 0.97 && got_us <= want_us * 1.03)
                $display("[blitanchor] %s: %.2f us (want %.2f +-3%%)  PASS",
                         tag, got_us, want_us);
            else begin
                $display("[blitanchor] %s: %.2f us (want %.2f +-3%%)  FAIL",
                         tag, got_us, want_us);
                ba_fail = ba_fail + 1;
            end
        end
    endtask

    // H5: like ba_run but fired at a controlled scanline phase - wait for
    // the video hline (= steal point / governor boundary anchor), then
    // phase_ckio CKIO, then EXEC.  Returns the ENGINE-CHAIN time
    // busy_end - first_op_start (lead-in-free, so the expected value is an
    // exact cost-model number, checkable to <1%).
    task automatic ba_run_phase(input [28:0] list, input [15:0] cx,
                                input [15:0] cy, input longint phase_ckio,
                                output real end_us);
        longint t0;
        begin
            ba_kind.delete();
            ba_cost.delete();
            wait (dut.u_blit.u_blit_gov.o_busy   == 1'b0 &&
                  dut.u_blit.u_blit_fetch.o_busy == 1'b0 &&
                  dut.u_blit.u_blit_draw.o_busy  == 1'b0);
            wait (dut.u_blit.blit_hline == 1'b0);
            wait (dut.u_blit.blit_hline == 1'b1);
            t0 = ba_ckio;
            while (ba_ckio < t0 + phase_ckio) @(posedge i_CLK);
            @(negedge i_CLK);
            dut.u_blit.u_blit_regs.bd_list     = list;
            dut.u_blit.u_blit_regs.bd_clip_x   = cx;
            dut.u_blit.u_blit_regs.bd_clip_y   = cy;
            dut.u_blit.u_blit_regs.bd_exec_req = 1'b1;
            wait (dut.u_blit.u_blit_gov.o_busy == 1'b1);
            wait (dut.u_blit.u_blit_gov.o_busy == 1'b0);
            @(negedge i_CLK);
            end_us = real'(dut.u_blit.u_blit_gov.r_busy_end
                           - dut.u_blit.u_blit_gov.r_first_start) / 153.6;
        end
    endtask

    initial begin
        #10;                                       // after plusarg initials
        if (ba_on != 0) begin
            real t_us;
            int  want_cost[3];
            want_cost[0] = 93; want_cost[1] = 189; want_cost[2] = 12090;

            wait (ba_ckio >= ba_at);
            wait (dut.u_blit.u_blit_fetch.o_busy == 1'b0 && dut.u_blit.u_blit_gov.o_busy == 1'b0);
            begin : quiet_seek
                longint deadline;
                deadline = ba_ckio + 64'd8_000_000;
                while (ba_quiet < 128 && ba_ckio < deadline) @(posedge i_CLK);
                if (ba_quiet < 128)
                    $display("[blitanchor] WARNING: no quiet-bus window by CKIO %0d - injecting anyway", ba_ckio);
            end
            $display("[blitanchor] injecting at CKIO %0d (bus quiet for %0d CKIO)",
                     ba_ckio, ba_quiet);

            // list A: the three cost anchors, unclipped at clip=(32,32)
            ba_cur = 32'h0C7C0000;
            ba_draw(16'd0, 16'd0, 16'd0, 16'd0, 8, 8);
            ba_draw(16'd0, 16'd0, 16'd0, 16'd0, 16, 12);
            ba_draw(16'd64, 16'd0, 16'd64, 16'd0, 240, 64);
            ba_w(16'h0000);
            // list B: 80x 1x324, dst_x=1000 -> fully outside the window
            ba_cur = 32'h0C7C1000;
            for (int i = 0; i < 80; i++)
                ba_draw(16'd0, 16'd0, 16'd1000, 16'd0, 1, 324);
            ba_w(16'h0000);
            // list C: 256x5 UPLOAD (payload zeros), dst off-screen
            ba_cur = 32'h0C7C2000;
            ba_w(16'h2000); ba_w(16'h0000); ba_w(16'h9999); ba_w(16'h9999);
            ba_w(16'd0);    ba_w(16'd3800); ba_w(16'd255);  ba_w(16'd4);
            for (int i = 0; i < 256 * 5; i++) ba_w(16'h0000);
            ba_w(16'h0000);

            // A: per-draw VCLK costs
            ba_run(29'h0C7C0000, 16'd32, 16'd32, t_us);
            for (int i = 0; i < 3; i++) begin
                if (i < ba_cost.size() && ba_kind[i] == 1 && ba_cost[i] == want_cost[i])
                    $display("[blitanchor] draw anchor %0d: %0d VCLK  PASS", i, ba_cost[i]);
                else begin
                    $display("[blitanchor] draw anchor %0d: kind=%0d cost=%0d (want draw/%0d)  FAIL",
                             i, (i < ba_kind.size()) ? ba_kind[i] : -1,
                             (i < ba_cost.size()) ? ba_cost[i] : -1, want_cost[i]);
                    ba_fail = ba_fail + 1;
                end
            end

            // B: fetch-bound clipped list (busy_end model vs 17.5 us)
            ba_run(29'h0C7C1000, 16'd32, 16'd32, t_us);
            ba_check("80x clipped 1x324", t_us, 17.5);

            // C: bus-bound upload (vs 58.77 us)
            ba_run(29'h0C7C2000, 16'd32, 16'd32, t_us);
            ba_check("256x5 upload", t_us, 58.77);

            // D: governed-window bind smoke.  10 engine-bound 240x64 draws
            // (12,090 VCLK each) with the window pinched to 2 chunks: the
            // fetch must stall on o_fetch_hold until governed op STARTS
            // release slot-chunks (fifo_study drainB), then the governed
            // busy_end must still land on the pure cost-model prediction
            // ~10 x (12,090 VCLK + ~2.5 hline steals) ~= 1628 us (checked
            // +-2%: steal count is phase-sensitive).  Exercises the window
            // and the real-time steal loop, which A-C never bind.
            begin
                ba_cur = 32'h0C7C4000;
                for (int i = 0; i < 10; i++)
                    ba_draw(16'd64, 16'd0, 16'd64, 16'd0, 240, 64);
                ba_w(16'h0000);
                dut.u_blit.u_blit_gov.t_window = 16'd2;          // sim-only pinch
                ba_run(29'h0C7C4000, 16'd32, 16'd32, t_us);
                dut.u_blit.u_blit_gov.t_window = 16'd512;        // restore
                if (ba_breq_max > 1000)
                    $display("[blitanchor] window bind: max chunk gap %0d CKIO  PASS", ba_breq_max);
                else begin
                    $display("[blitanchor] window bind: max chunk gap %0d CKIO (want >1000: hold never engaged)  FAIL", ba_breq_max);
                    ba_fail = ba_fail + 1;
                end
                if (t_us >= 1628.0 * 0.98 && t_us <= 1628.0 * 1.02)
                    $display("[blitanchor] window-bound busy_end: %.1f us (want ~1628 +-2%%)  PASS", t_us);
                else begin
                    $display("[blitanchor] window-bound busy_end: %.1f us (want ~1628 +-2%%)  FAIL", t_us);
                    ba_fail = ba_fail + 1;
                end
            end

            // Non-tile-aligned scroll for the E/F scanout window: the frame
            // capture (+blitframe alongside +blitanchor) then exercises the
            // line fetcher's intra-tile x offset and y wrap against the
            // golden crop - (0,0)/32-multiple scrolls never touch that path.
            // Set BEFORE test E so the last logged EXEC carries it (golden
            // crops at the last exec's scroll).
            dut.u_blit.u_blit_regs.o_scroll_x = 16'd13;
            dut.u_blit.u_blit_regs.o_scroll_y = 16'd7;

            // E (H5): 240x64 fired 2000 CKIO (= 3000 VCLK) past a real hline
            // - the draw's op_start phase lands in (2230, 4630) VCLK after a
            // boundary, where exactly 3 steal boundaries fall inside the op:
            // busy_end - first_op_start = 12,090 + 3x166 = 12,588 VCLK =
            // 163.91 us EXACTLY (the [PDF] "240x64 total incl. steals"
            // number).  Validates the governor's steal phase being anchored
            // on the real scanline instead of reset at EXEC.
            begin
                ba_cur = 32'h0C7C5000;
                ba_draw(16'd64, 16'd0, 16'd64, 16'd0, 240, 64);
                ba_w(16'h0000);
                ba_run_phase(29'h0C7C5000, 16'd32, 16'd32, 64'd2000, t_us);
                if (ba_cost.size() >= 1 && ba_kind[0] == 1 && ba_cost[0] == 12090)
                    $display("[blitanchor] phased 240x64: cost %0d VCLK  PASS", ba_cost[0]);
                else begin
                    $display("[blitanchor] phased 240x64: bad cost record  FAIL");
                    ba_fail = ba_fail + 1;
                end
                if (t_us >= 163.91 * 0.99 && t_us <= 163.91 * 1.01)
                    $display("[blitanchor] 3-steal engine chain: %.2f us (want 163.91 +-1%%)  PASS", t_us);
                else begin
                    $display("[blitanchor] 3-steal engine chain: %.2f us (want 163.91 +-1%%)  FAIL", t_us);
                    ba_fail = ba_fail + 1;
                end
            end

            // G (H5): visible nonzero content for the frame capture - all
            // prior tests paint black-on-black (zero src, copy blend), which
            // makes the +blitframe golden diff vacuous.  UPLOAD a 64x32
            // gradient at (20,10) and DRAW a copy of it at (150,100): both
            // land inside the scroll-(13,7) window, so the captured frame
            // checks real pixels through the non-aligned fetch path.
            begin
                ba_cur = 32'h0C7C6000;
                ba_w(16'h2000); ba_w(16'h0000); ba_w(16'h9999); ba_w(16'h9999);
                ba_w(16'd20);   ba_w(16'd10);   ba_w(16'd63);   ba_w(16'd31);
                for (int y = 0; y < 32; y++)
                    for (int x = 0; x < 64; x++)
                        ba_w(16'h8000 | 16'((x * 7 + y * 13 + 'h1234) & 'h7FFF));
                ba_draw(16'd20, 16'd10, 16'd150, 16'd100, 64, 32);
                ba_w(16'h0000);
                ba_run(29'h0C7C6000, 16'd32, 16'd32, t_us);
            end

            // F (H5): sync generator exactness - hline spacing 3256 CKIO,
            // vsync spacing 853,072 CKIO (= 60.0184 Hz)
            begin
                longint t_a, t_b;
                wait (dut.u_blit.blit_hline == 1'b0); wait (dut.u_blit.blit_hline == 1'b1);
                t_a = ba_ckio;
                wait (dut.u_blit.blit_hline == 1'b0); wait (dut.u_blit.blit_hline == 1'b1);
                t_b = ba_ckio;
                if (t_b - t_a == 3256)
                    $display("[blitanchor] hline period: %0d CKIO  PASS", t_b - t_a);
                else begin
                    $display("[blitanchor] hline period: %0d CKIO (want 3256)  FAIL", t_b - t_a);
                    ba_fail = ba_fail + 1;
                end
                wait (dut.u_blit.blit_vsync == 1'b0); wait (dut.u_blit.blit_vsync == 1'b1);
                t_a = ba_ckio;
                wait (dut.u_blit.blit_vsync == 1'b0); wait (dut.u_blit.blit_vsync == 1'b1);
                t_b = ba_ckio;
                if (t_b - t_a == 853_072)
                    $display("[blitanchor] frame period: %0d CKIO (60.0184 Hz)  PASS", t_b - t_a);
                else begin
                    $display("[blitanchor] frame period: %0d CKIO (want 853,072)  FAIL", t_b - t_a);
                    ba_fail = ba_fail + 1;
                end
                // let +blitframe's snapshot at this vsync land before $finish
                // (the frame scanned between these two vsyncs is the clean,
                // post-G capture; finishing in the same timestep loses it)
                @(negedge i_CLK); @(negedge i_CLK);
            end

            $display("[blitanchor] %s", (ba_fail == 0) ? "ALL PASS" : "FAILURES");
            $finish;
        end
    end

    //========================================================================
    // H5 - scanout frame capture: +blitframe=<file> taps blit_video's pixel
    // stream continuously and keeps the most recent COMPLETE 320x240 frame
    // (vsync-to-vsync with all 76,800 px_de strobes); at end of sim it is
    // written raw little-endian ARGB1555 row-major, with the scroll sampled
    // at the frame's first visible pixel.  Diff against the golden model's
    // visible crop with `blitgold --boardtrace <fifo log> --frame <file>`
    // (MAME copyscrollbitmap wrap semantics) - the in-system H5 accept.
    // A TB-side self-check against the behavioral VRAM (same wrap math) runs
    // too; it can false-fail only if a blit landed after the last vsync.
    //========================================================================
    string  bfr_file;
    //========================================================================
    // H7 descriptor-sideband footprint checker: always on (hard $fatal on a
    // beat outside the descriptor-predicted footprint).  Taps blit_top's
    // exported sideband + beat channels hierarchically.
    //========================================================================
    blit_dsc_check u_dsc_check (
        .i_CLK          (i_CLK),
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

    integer bfr_on = 0;
    reg [15:0] bfr_cur  [0:76799];
    reg [15:0] bfr_last [0:76799];
    integer bfr_idx    = 0;
    integer bfr_frames = 0;
    reg [15:0] bfr_sx_c, bfr_sy_c, bfr_sx_l, bfr_sy_l;

    initial if ($value$plusargs("blitframe=%s", bfr_file)) bfr_on = 1;

    always @(posedge i_CLK) if (bfr_on != 0) begin
        if (dut.u_blit.blit_vsync) begin
            if (bfr_idx == 76800) begin
                for (int i = 0; i < 76800; i++) bfr_last[i] = bfr_cur[i];
                bfr_sx_l   = bfr_sx_c;
                bfr_sy_l   = bfr_sy_c;
                bfr_frames = bfr_frames + 1;
            end
            bfr_idx = 0;
        end
        else if (dut.u_blit.u_blit_video.o_px_de && bfr_idx < 76800) begin
            if (bfr_idx == 0) begin      // scroll in effect at line 0
                bfr_sx_c = dut.u_blit.blit_scroll_x;
                bfr_sy_c = dut.u_blit.blit_scroll_y;
            end
            bfr_cur[bfr_idx] = dut.u_blit.u_blit_video.o_px;
            bfr_idx = bfr_idx + 1;
        end
    end

    final begin
        if (bfr_on != 0) begin
            if (bfr_frames == 0)
                $display("[blitframe] no complete frame captured");
            else begin
                automatic integer bfr_fd = $fopen(bfr_file, "wb");
                automatic longint bad = 0;
                if (bfr_fd != 0) begin
                    for (int i = 0; i < 76800; i++)
                        $fwrite(bfr_fd, "%c%c", bfr_last[i][7:0], bfr_last[i][15:8]);
                    $fclose(bfr_fd);
                end
                for (int y = 0; y < 240; y++)
                    for (int x = 0; x < 320; x++) begin
                        automatic logic [15:0] want = dut.u_blit_vram.mem[
                            {(12'(bfr_sy_l) + 12'(y)), (13'(bfr_sx_l) + 13'(x))}];
                        if (bfr_last[y * 320 + x] !== want) bad = bad + 1;
                    end
                $display("[blitframe] wrote %s (frame %0d, scroll=(%0d,%0d)); VRAM self-check: %s (%0d bad px)",
                         bfr_file, bfr_frames, bfr_sx_l, bfr_sy_l,
                         (bad == 0) ? "PIXEL-EXACT" : "MISMATCH", bad);
            end
        end
    end

    //========================================================================
    // H3 - blitter VRAM dump: +blitvram=<file> writes the draw engine's
    // 64 MB behavioral VRAM as raw little-endian ARGB1555 (one u16 per px,
    // 8192x4096 row-major) at end of sim.  Diff against the golden model
    // with `blitgold --boardtrace build/board_blit.txt --raw <file>` - the
    // in-system half of the H3 accept.
    //========================================================================
    string bv_file;
    final begin
        if ($value$plusargs("blitvram=%s", bv_file)) begin
            automatic integer bv_fd = $fopen(bv_file, "wb");
            if (bv_fd != 0) begin
                for (int unsigned i = 0; i < 33554432; i++) begin
                    automatic logic [15:0] px = dut.u_blit_vram.mem[i];
                    $fwrite(bv_fd, "%c%c", px[7:0], px[15:8]);
                end
                $fclose(bv_fd);
                $display("[blitvram] wrote %s", bv_file);
            end
        end
    end

endmodule

// attach the retired-instruction probe inside the (read-only) core
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
