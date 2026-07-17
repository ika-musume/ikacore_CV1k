`timescale 1ns/1ps
`default_nettype none
//============================================================================
// tb_nand.sv - Step-5 unit accept for CV1k_nand + CV1k_ddr3_harness NAND path
//
// Drives the raw CV1000-B NAND command protocol (reset / read-ID / page read)
// into CV1k_nand, which serves page data from a behavioural DDR3 slave through
// the real CV1k_ddr3_harness (video + batch clients tied off).  Every streamed
// byte is checked against the raw U2 image (roms/ibara/u2), independently
// $fread into this TB - so a pass proves the whole harness-served path
// (command decode -> page-fill train -> RE# streaming) is byte-for-byte the
// data the physical chip would return.  The read-ID is checked against the
// K9F1G08U0M constants EC F1 00 95 40.
//============================================================================

// --- behavioural DDR3 slave: MiSTer f2sdram face, holds the U2 image ---
module ddr_slave_beh #(
    parameter integer PAGES = 256,      // pages of the image staged at word 0
    parameter integer GAP   = 4         // command->data / inter-burst gap
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
    localparam integer NBYTES = PAGES*2112;
    reg [7:0] bmem [0:NBYTES-1];

    integer fd, nread, ib;
    initial begin
        busy = 1'b0; dout_ready = 1'b0; dout = 64'd0;
        for (ib = 0; ib < NBYTES; ib = ib + 1) bmem[ib] = 8'h00;
        fd = $fopen("roms/ibara/u2", "rb");
        if (fd == 0) begin $display("[ddr_slave] cannot open roms/ibara/u2"); $fatal; end
        nread = $fread(bmem, fd);
        $fclose(fd);
        $display("[ddr_slave] loaded %0d bytes of U2 image (%0d pages)", nread, PAGES);
    end

    function [63:0] word_at(input [28:0] wa);
        integer base, k;
        reg [63:0] w;
        begin
            base = wa * 8;
            w = 64'd0;
            for (k = 0; k < 8; k = k + 1)
                if (base + k < NBYTES) w[k*8 +: 8] = bmem[base + k];
            word_at = w;
        end
    endfunction

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
                    dout       <= word_at(cur_addr);
                    dout_ready <= 1'b1;
                    cur_addr   <= cur_addr + 29'd1;
                    cur_left   <= cur_left - 9'd1;
                    if (cur_left == 9'd1) active <= 1'b0;
                end
            end
        end
    end
endmodule


module tb_nand;
    localparam integer PAGES = 256;

    reg clk = 1'b0;
    always #5 clk = ~clk;                        // 100 MHz TB clock
    reg rst_n;

    // NAND chip face (split DQ)
    reg  [7:0] dq_in;
    wire [7:0] dq_out;
    wire       dq_oe;
    reg        cle, ale, ce_n, we_n, re_n, wp_n;
    wire       rb_n;

    // CV1k_nand <-> harness NAND client
    wire        nd_req;
    wire [28:0] nd_addr;
    wire [10:0] nd_len;
    wire        nd_rdy, nd_dvld;
    wire [63:0] nd_data;

    // DDRAM face
    wire        ddr_busy, ddr_dout_ready;
    wire [63:0] ddr_dout;
    wire [7:0]  ddr_burstcnt;
    wire [28:0] ddr_addr;
    wire        ddr_rd, ddr_we, ddr_clk;
    wire [63:0] ddr_din;
    wire [7:0]  ddr_be;

    CV1k_nand #(.NAND_BASE_W(29'd0)) dut (
        .i_CLK(clk), .i_RST_n(rst_n),
        .i_Dq(dq_in), .o_Dq(dq_out), .o_Dq_oe(dq_oe),
        .i_Cle(cle), .i_Ale(ale), .i_Ce_n(ce_n),
        .i_We_n(we_n), .i_Re_n(re_n), .i_Wp_n(wp_n), .o_Rb_n(rb_n),
        .o_nd_req(nd_req), .o_nd_addr(nd_addr), .o_nd_len(nd_len),
        .i_nd_rdy(nd_rdy), .i_nd_dvld(nd_dvld), .i_nd_data(nd_data)
    );

    CV1k_ddr3_harness u_harness (
        .i_CLK(clk), .i_RST_n(rst_n),
        .i_lf_req(1'b0), .i_lf_y(12'd0), .i_lf_x0(13'd0),
        .o_lf_dvld(), .o_lf_data(),
        .i_prd_req(1'b0), .i_prd_addr(23'd0), .i_prd_len(11'd0),
        .o_prd_rdy(), .o_prd_dvld(), .o_prd_data(),
        .i_pwr_req(1'b0), .i_pwr_addr(23'd0), .i_pwr_data(64'd0),
        .i_pwr_be(4'd0), .o_pwr_rdy(),
        .i_rd_train(1'b0), .i_wr_train(1'b0),
        .i_nd_req(nd_req), .i_nd_addr(nd_addr), .i_nd_len(nd_len),
        .o_nd_rdy(nd_rdy), .o_nd_dvld(nd_dvld), .o_nd_data(nd_data),
        .i_ym_req(1'b0), .i_ym_addr(29'd0), .i_ym_len(11'd0),
        .o_ym_rdy(), .o_ym_dvld(), .o_ym_data(),
        .DDRAM_CLK(ddr_clk), .DDRAM_BUSY(ddr_busy),
        .DDRAM_BURSTCNT(ddr_burstcnt), .DDRAM_ADDR(ddr_addr),
        .DDRAM_DOUT(ddr_dout), .DDRAM_DOUT_READY(ddr_dout_ready),
        .DDRAM_RD(ddr_rd), .DDRAM_DIN(ddr_din), .DDRAM_BE(ddr_be), .DDRAM_WE(ddr_we)
    );

    ddr_slave_beh #(.PAGES(PAGES)) u_ddr (
        .clk(clk), .rst_n(rst_n),
        .rd(ddr_rd), .addr(ddr_addr), .burstcnt(ddr_burstcnt),
        .we(ddr_we), .din(ddr_din), .be(ddr_be),
        .busy(ddr_busy), .dout_ready(ddr_dout_ready), .dout(ddr_dout)
    );

    // independent reference copy of the image
    reg [7:0] ref_b [0:PAGES*2112-1];
    integer rfd, rn;
    initial begin
        rfd = $fopen("roms/ibara/u2", "rb");
        rn  = $fread(ref_b, rfd);
        $fclose(rfd);
    end

    integer errors, checks;
    reg [7:0] rb;

    function [7:0] exp_id(input integer i);
        case (i)
            0: exp_id = 8'hEC;
            1: exp_id = 8'hF1;
            2: exp_id = 8'h00;
            3: exp_id = 8'h95;
            default: exp_id = 8'h40;
        endcase
    endfunction

    task nand_cmd(input [7:0] c);
        begin
            @(posedge clk); cle <= 1'b1; ale <= 1'b0; dq_in <= c; we_n <= 1'b0;
            repeat (3) @(posedge clk);
            we_n <= 1'b1;                          // rising edge latches
            repeat (3) @(posedge clk);
            cle <= 1'b0;
        end
    endtask

    task nand_addr(input [7:0] a);
        begin
            @(posedge clk); ale <= 1'b1; cle <= 1'b0; dq_in <= a; we_n <= 1'b0;
            repeat (3) @(posedge clk);
            we_n <= 1'b1;
            repeat (3) @(posedge clk);
            ale <= 1'b0;
        end
    endtask

    task nand_rd(output [7:0] d);
        begin
            @(posedge clk); re_n <= 1'b0;
            repeat (3) @(posedge clk);
            d = dq_out;                            // stable during the low phase
            re_n <= 1'b1;                          // rising edge advances pointer
            repeat (3) @(posedge clk);
        end
    endtask

    task wait_ready;
        begin
            @(posedge clk);
            while (rb_n == 1'b0) @(posedge clk);
        end
    endtask

    task read_id_check;
        integer i;
        begin
            nand_cmd(8'h90);
            nand_addr(8'h00);
            for (i = 0; i < 5; i = i + 1) begin
                nand_rd(rb);
                checks = checks + 1;
                if (rb !== exp_id(i)) begin
                    errors = errors + 1;
                    if (errors < 20)
                        $display("  ID[%0d] got %02x exp %02x", i, rb, exp_id(i));
                end
            end
        end
    endtask

    task read_page_check(input [15:0] row, input [11:0] col0, input integer nbytes);
        integer i, off;
        begin
            nand_cmd(8'h00);
            nand_addr(col0[7:0]);
            nand_addr({4'h0, col0[11:8]});
            nand_addr(row[7:0]);
            nand_addr(row[15:8]);
            nand_cmd(8'h30);
            wait_ready;
            for (i = 0; i < nbytes; i = i + 1) begin
                nand_rd(rb);
                off = row * 2112 + col0 + i;
                checks = checks + 1;
                if (rb !== ref_b[off]) begin
                    errors = errors + 1;
                    if (errors < 20)
                        $display("  row %0d col %0d got %02x exp %02x",
                                 row, col0 + i, rb, ref_b[off]);
                end
            end
        end
    endtask

    // watchdog
    initial begin
        repeat (4_000_000) @(posedge clk);
        $display("=== tb_nand: TIMEOUT ===");
        $finish;
    end

    initial begin
        errors = 0; checks = 0;
        cle = 0; ale = 0; ce_n = 1; we_n = 1; re_n = 1; wp_n = 1; dq_in = 8'h00;
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);
        ce_n = 1'b0;                               // U13 0x10C00003 d0: enable chip
        repeat (4) @(posedge clk);

        nand_cmd(8'hFF);                           // reset
        wait_ready;

        read_id_check;                             // EC F1 00 95 40

        // full-page reads across the early-boot slice (boot reads rows 0-95)
        read_page_check(16'd0,   12'd0, 2112);
        read_page_check(16'd1,   12'd0, 2112);
        read_page_check(16'd2,   12'd0, 2112);
        read_page_check(16'd3,   12'd0, 2112);
        read_page_check(16'd32,  12'd0, 2112);
        read_page_check(16'd63,  12'd0, 2112);
        read_page_check(16'd64,  12'd0, 2112);
        read_page_check(16'd95,  12'd0, 2112);
        read_page_check(16'd255, 12'd0, 2112);
        // column-offset stream (random data start) + spare-area read
        read_page_check(16'd10,  12'd1000, 512);
        read_page_check(16'd200, 12'd2048, 64);

        if (errors == 0)
            $display("=== tb_nand: PASS  (%0d bytes byte-exact vs U2 image; ID EC F1 00 95 40) ===", checks);
        else
            $display("=== tb_nand: FAIL  (%0d errors / %0d checks) ===", errors, checks);
        $finish;
    end
endmodule
`default_nettype none
