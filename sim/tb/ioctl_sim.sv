`default_nettype none
`timescale 1ns/1ps
//============================================================================
// ioctl_sim - MiSTer HPS ioctl download emulator (testbench stimulus)
//
// Modeled on ika-musume's BubSysROM_ioctl_test.v (ikacore_BubSysROM): 8-bit
// data beats, 27-bit byte address, paced writes, WAIT honored.  H7b.4: the
// stream is the FIXED game-agnostic MRA layout the CV1k_ioctl decoder
// expects (short parts zero-padded, exactly what the MRA <rom> assembles):
//
//   [0,          0x840_0000)  u2  NAND dump (132 MiB, exact size)
//   [0x840_0000, 0x8C0_0000)  u23 YMZ chip 0, zero-padded to 8 MiB
//   [0x8C0_0000, 0x940_0000)  u24 YMZ chip 1, zero-padded to 8 MiB
//   [0x940_0000, 0x980_0000)  u4  program NOR "x2": a <= 2 MiB file is
//                             streamed twice (the MRA lists u4 twice =
//                             the vendor-arm ROM_RELOAD mirror); a larger
//                             file is streamed once, zero-padded to 4 MiB
//
// The o_ADDR presented wraps mod 2^27 exactly like hps_io's ioctl_addr -
// the DUT decoder must key on its own byte counter (that is the point).
//
// Plusargs:  +ioctl_u2=<f> +ioctl_u23=<f> +ioctl_u24=<f> +ioctl_u4=<f>
//              (defaults roms/ibara/{u2,u23,u24,u4})
//            +ioctl_bytes=<n>   stop after n total stream bytes
//                               (0 / absent = the whole 152 MiB layout)
//            +ioctl_ival=<n>    idle clocks between byte writes
//                               (default = INTERVAL parameter)
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

localparam longint U2_SZ  = 64'h840_0000;    // exact dump size incl. spare
localparam longint YMZ_SZ = 64'h080_0000;    // 8 MiB chip slot
localparam longint U4_SZ  = 64'h040_0000;    // 4 MiB NOR window (u4 x2)

string  p_u2  = "roms/ibara/u2";
string  p_u23 = "roms/ibara/u23";
string  p_u24 = "roms/ibara/u24";
string  p_u4  = "roms/ibara/u4";
longint limit = 0;
integer ival;
longint n;                                   // global stream byte counter

// stream one part: file bytes (tiled n_tile times if tile, else once),
// zero-padded to part_sz.  Honors the global limit; sets done_all when hit.
reg done_all;
task automatic stream_part(input string path, input longint part_sz,
                           input bit tile2);
    integer fd, b;
    longint k, half;
    begin
        fd = $fopen(path, "rb");
        if (fd == 0) begin
            $display("[ioctl] WARNING: cannot open %s - part streamed as zeros", path);
        end
        // u4 "x2": file <= 2 MiB -> two 2 MiB sub-slots of the same file;
        // bigger file -> one 4 MiB slot
        half = part_sz / 2;
        for (k = 0; k < part_sz && !done_all; k = k + 1) begin
            if (tile2 && (k == half) && fd != 0)
                void'($fseek(fd, 0, 0));
            b = (fd != 0) ? $fgetc(fd) : -1;
            if (b == -1) b = 0;              // zero-pad past EOF
            repeat (ival) @(posedge i_CLK);
            while (i_WAIT) @(posedge i_CLK);
            o_ADDR <= n[26:0];               // wraps at 128 MiB like hps_io
            o_DATA <= b[7:0];
            o_WR   <= 1'b1;
            @(posedge i_CLK);
            o_WR   <= 1'b0;
            n = n + 1;
            if (limit > 0 && n >= limit) done_all = 1;
        end
        if (fd != 0) $fclose(fd);
    end
endtask

initial begin
    o_DOWNLOAD = 1'b0; o_ADDR = 27'h0; o_DATA = 8'h0;
    o_WR = 1'b0; o_INDEX = 16'h0; o_DONE = 1'b0;
    done_all = 0;
    ival = INTERVAL;

    void'($value$plusargs("ioctl_u2=%s",  p_u2));
    void'($value$plusargs("ioctl_u23=%s", p_u23));
    void'($value$plusargs("ioctl_u24=%s", p_u24));
    void'($value$plusargs("ioctl_u4=%s",  p_u4));
    void'($value$plusargs("ioctl_bytes=%d", limit));
    void'($value$plusargs("ioctl_ival=%d",  ival));

    @(posedge i_START);
    @(posedge i_CLK);
    o_DOWNLOAD <= 1'b1;
    n = 0;

    stream_part(p_u2,  U2_SZ,  1'b0);
    if (!done_all) stream_part(p_u23, YMZ_SZ, 1'b0);
    if (!done_all) stream_part(p_u24, YMZ_SZ, 1'b0);
    if (!done_all) begin : u4_part
        // u4 x2 decision needs the file size up front
        integer fd4;
        longint fsz4;
        fd4 = $fopen(p_u4, "rb");
        fsz4 = 0;
        if (fd4 != 0) begin
            void'($fseek(fd4, 0, 2));
            fsz4 = $ftell(fd4);
            $fclose(fd4);
        end
        stream_part(p_u4, U4_SZ, fsz4 <= 64'h20_0000);
    end

    repeat (ival + 1) @(posedge i_CLK);
    while (i_WAIT) @(posedge i_CLK);     // let the last halfword/word drain
    o_DOWNLOAD <= 1'b0;
    o_DONE     <= 1'b1;
    $display("[ioctl] streamed %0d bytes (u2=%s u23=%s u24=%s u4=%s)",
             n, p_u2, p_u23, p_u24, p_u4);
end

endmodule
`default_nettype wire
