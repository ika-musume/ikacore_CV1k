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
        .ROM_FILE("rom/ibara_u4_4M.hex")
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
