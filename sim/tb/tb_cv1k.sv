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
        @(posedge i_CLK) i_POR_n = 1'b1;
        repeat (8) @(posedge i_CLK);
        i_RST_n = 1'b1;
        $display("[tb] reset released @ %0t (flash powered up)", $time);
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
    );

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
