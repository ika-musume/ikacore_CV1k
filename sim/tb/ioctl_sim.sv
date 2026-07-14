`default_nettype none
`timescale 1ns/1ps
//============================================================================
// ioctl_sim - MiSTer HPS ioctl download emulator (testbench stimulus)
//
// Modeled on ika-musume's BubSysROM_ioctl_test.v (ikacore_BubSysROM): 8-bit
// data beats, 27-bit byte address, paced writes, WAIT honored.  Reads a raw
// binary (the MAME u4 dump) and streams it as one download with INDEX=0 -
// the CV1k core treats the first 32 Mbit of the stream as the U4 NOR image
// (NOR -> NAND -> YMZ order is prepared on the ARM/MRA side).
//
// Plusargs:  +ioctl_bin=<raw file>   (default roms/ibara/u4)
//            +ioctl_bytes=<n>        (0 / absent = whole file)
//============================================================================
module ioctl_sim #(
    parameter INTERVAL = 4                   // idle clocks between byte writes
) (
    input  wire         i_CLK,
    input  wire         i_START,             // rising edge starts the download
    output reg          o_DOWNLOAD,
    output reg  [26:0]  o_ADDR,
    output reg  [7:0]   o_DATA,
    output reg          o_WR,
    output reg  [15:0]  o_INDEX,
    input  wire         i_WAIT,
    output reg          o_DONE
);

string  path = "roms/ibara/u4";
longint limit = 0;
longint n;
integer fd, b;

initial begin
    o_DOWNLOAD = 1'b0; o_ADDR = 27'h0; o_DATA = 8'h0;
    o_WR = 1'b0; o_INDEX = 16'h0; o_DONE = 1'b0;

    void'($value$plusargs("ioctl_bin=%s", path));
    void'($value$plusargs("ioctl_bytes=%d", limit));

    @(posedge i_START);
    fd = $fopen(path, "rb");
    if (fd == 0) begin
        $display("[ioctl] ERROR: cannot open %s", path);
        $finish;
    end

    @(posedge i_CLK);
    o_DOWNLOAD <= 1'b1;
    n = 0;
    forever begin
        b = $fgetc(fd);
        if (b == -1 || (limit > 0 && n >= limit)) break;
        repeat (INTERVAL) @(posedge i_CLK);
        while (i_WAIT) @(posedge i_CLK);
        o_ADDR <= n[26:0];
        o_DATA <= b[7:0];
        o_WR   <= 1'b1;
        @(posedge i_CLK);
        o_WR   <= 1'b0;
        n = n + 1;
    end
    $fclose(fd);

    repeat (INTERVAL) @(posedge i_CLK);
    while (i_WAIT) @(posedge i_CLK);     // let the last halfword drain
    o_DOWNLOAD <= 1'b0;
    o_DONE     <= 1'b1;
    $display("[ioctl] streamed %0d bytes from %s", n, path);
end

endmodule
`default_nettype wire
