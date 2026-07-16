`default_nettype none
//============================================================================
// ddr3_beh.sv - behavioural MiSTer f2sdram (DDRAM) slave for simulation
//
// Models the HPS-side SDRAM that CV1k_ddr3_harness drives.  Two content
// planes:
//   * file plane: word W >= BASE_W returns image bytes [(W-BASE_W)*8 ..+7]
//     little-endian off the raw dump on demand (no multi-MB preload) -
//     the DDR3-resident U2 NAND image, matching CV1k_nand's page layout;
//   * write-back overlay (H7b.2): writes land in an associative array
//     (byte-enable merged over the file/zero background) and shadow the
//     file plane on reads - this is the VRAM region (and any other RAM
//     use) of the full in-system blit stack.  Unwritten non-file words
//     read 0 = black VRAM at power-up, the blit_vram_beh convention.
// In-order burst return with a small command/data gap, exactly what the
// harness's tag-routed read path and the calibrated port model expect.
// The TB reads the end-of-run VRAM image through peek() (hierarchical
// function call - +blitvram / +blitframe self-check).
//============================================================================
module ddr3_beh #(
    parameter        IMAGE  = "roms/ibara/u2",
    parameter [28:0] BASE_W = 29'd0,      // DDRAM word base of the image
    parameter integer GAP   = 4           // command->data / inter-burst gap
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rd,
    input  wire [28:0] addr,
    input  wire [7:0]  burstcnt,
    input  wire        we,
    input  wire [63:0] din,
    input  wire [7:0]  be,
    output reg         busy,
    output reg         dout_ready,
    output reg  [63:0] dout
);
    integer fd;
    initial begin
        busy = 1'b0; dout_ready = 1'b0; dout = 64'd0;
        fd = 0;
        if (IMAGE != "") begin               // "" = no file plane (VRAM only)
            fd = $fopen(IMAGE, "rb");
            if (fd == 0)
                $display("[ddr3_beh] WARNING: cannot open %s (reads return 0)", IMAGE);
            else
                $display("[ddr3_beh] serving %s on-demand at word base 0x%07x", IMAGE, BASE_W);
        end
    end

    function [63:0] word_at(input [28:0] wa);
        integer base, k, ch;
        reg [63:0] w;
        begin
            w = 64'd0;
            if (fd != 0 && wa >= BASE_W) begin
                base = (wa - BASE_W) * 8;
                void'($fseek(fd, base, 0));
                for (k = 0; k < 8; k = k + 1) begin
                    ch = $fgetc(fd);
                    w[k*8 +: 8] = (ch < 0) ? 8'h00 : ch[7:0];
                end
            end
            word_at = w;
        end
    endfunction

    // write-back overlay (H7b.2): written words shadow the file plane
    reg [63:0] mem [longint];

    function [63:0] peek(input [28:0] wa);
        peek = mem.exists(longint'(wa)) ? mem[longint'(wa)] : word_at(wa);
    endfunction

    task automatic poke(input [28:0] wa, input [63:0] d, input [7:0] be);
        reg [63:0] w;
        integer k;
        begin
            w = peek(wa);
            for (k = 0; k < 8; k = k + 1)
                if (be[k]) w[k*8 +: 8] = d[k*8 +: 8];
            mem[longint'(wa)] = w;
        end
    endtask

    // in-order pending-burst queue
    reg [28:0] q_addr [0:63];
    reg [8:0]  q_len  [0:63];
    reg [5:0]  q_wp, q_rp;
    reg        active;
    reg [28:0] cur_addr;
    reg [8:0]  cur_left;
    reg [3:0]  gap;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_wp <= 6'd0; q_rp <= 6'd0; active <= 1'b0;
            dout_ready <= 1'b0; dout <= 64'd0; gap <= 4'd0;
            busy <= 1'b0; cur_addr <= 29'd0; cur_left <= 9'd0;
        end
        else begin
            busy <= 1'b0;                        // deep queue: always accept
            if (rd) begin
                q_addr[q_wp] <= addr;
                q_len[q_wp]  <= {1'b0, burstcnt};
                q_wp <= q_wp + 6'd1;
            end
            if (we)                              // H7b.2: write-back overlay
                poke(addr, din, be);

            dout_ready <= 1'b0;
            if (!active) begin
                if (q_wp != q_rp) begin
                    cur_addr <= q_addr[q_rp];
                    cur_left <= q_len[q_rp];
                    q_rp     <= q_rp + 6'd1;
                    active   <= 1'b1;
                    gap      <= GAP[3:0];
                end
            end
            else begin
                if (gap != 4'd0) gap <= gap - 4'd1;
                else begin
                    dout       <= peek(cur_addr);
                    dout_ready <= 1'b1;
                    cur_addr   <= cur_addr + 29'd1;
                    cur_left   <= cur_left - 9'd1;
                    if (cur_left == 9'd1) active <= 1'b0;
                end
            end
        end
    end
endmodule
`default_nettype none
