`default_nettype none
//============================================================================
// CV1k_nand.sv - MiSTer NAND read-path frontend (U2 graphics/asset flash)
//
// On the CV1000-B the SH-3 bit-bangs the U2 NAND (Samsung K9F1G08U0M) through
// the U13 CPLD: CLE=A0, ALE=A1, CE#/RE#/WE# from the CPLD, DQ on D[7:0],
// R/B# -> Port E bit 5.  There is no NAND controller - the CPU issues the raw
// 00h -> col,col,row,row -> 30h read sequence and clocks bytes out on RE#.
//
// On the MiSTer target there is no physical NAND chip; the 128 MB U2 image is
// staged into DDR3 by the HPS loader.  This module sits in the socket the
// vendor nand_model occupied - it presents the identical pin roles to the
// CPU/CPLD side, but instead of a $fread-backed array it decodes the command
// stream and serves page data from DDR3 through the CV1k_ddr3_harness NAND
// client (i_nd_* / o_nd_*), the lowest-priority train (FINDINGS.md 5.1).  This
// is deliberately NOT embedded in the harness: the harness only moves trains;
// the NAND protocol (command decode, page register, RE# streaming, R/B#) lives
// here so the harness stays a pure platform-agnostic arbiter.
//
// Read-only frontend: the boot path only reads NAND (manufacturer-ID check,
// then the DMAC copies pages into work RAM).  Program/erase opcodes are
// accepted-and-ignored; WP# only colours the status byte.
//
// Chip geometry (K9F1G08U0M / MT29F1G08 x8): page = 2048 main + 64 spare =
// 2112 B = 264 x 64-bit DDR3 words; 4 address cycles (2 col + 2 row); column 0
// is the low byte.  The staged image is linear page order, page N main+spare
// at byte NAND_BASE + N*2112 (= word NAND_BASE_W + N*264), byte c in word
// c>>3 lane c[2:0] (little-endian) - identical to the vendor on-demand loader
// (nand_die_model.v: off = row*NUM_COL, column 0 = LSB) so a page served here
// is byte-for-byte the page the physical chip would return.
//
// Single i_CLK domain (2xCKIO): CE#/RE#/WE# are combinational CPLD outputs of
// i_CLK-synchronous CPU strobes, so they are edge-detected on i_CLK - no gated
// or derived clocks (the board-wide rule).
//============================================================================
module CV1k_nand #(
    // DDR3 word base of the staged U2 image (byte 0x0400_0000 >> 3).  The HPS
    // loader stages the 128 MB dump here; overridable per integration.
    parameter [28:0] NAND_BASE_W = 29'h0080_0000
)(
    input  wire        i_CLK,
    input  wire        i_RST_n,

    //------------------------------------------------------------------
    // NAND chip face - pin roles match models/MT29F1G08ABAFA/nand_model.v.
    // DQ is split (write view / drive / OE) exactly as the CPLD and vendor
    // memory models do for Verilator; the board top resolves the tristate:
    //   assign D[7:0] = o_Dq_oe ? o_Dq : 8'hzz;
    //------------------------------------------------------------------
    input  wire [7:0]  i_Dq,      // DQ write-data view (CPU drives cmd/addr in)
    output wire [7:0]  o_Dq,      // DQ drive value (read data out)
    output wire        o_Dq_oe,   // DQ drive enable
    input  wire        i_Cle,     // CLE (= CPU A0)
    input  wire        i_Ale,     // ALE (= CPU A1)
    input  wire        i_Ce_n,    // CE#  (from U13, 0x10C00003 d0)  - the chip gate
    input  wire        i_We_n,    // WE#  - latch cmd/addr on rising edge
    input  wire        i_Re_n,    // RE#  - advance the data pointer on rising edge
    input  wire        i_Wp_n,    // WP#  - read-only frontend: status colour only
    output wire        o_Rb_n,    // R/B# - low while a page/reset is in flight

    //------------------------------------------------------------------
    // CV1k_ddr3_harness NAND client (lowest-priority, single train/page)
    //------------------------------------------------------------------
    output reg         o_nd_req,
    output reg  [28:0] o_nd_addr, // pre-mapped DDRAM word address (page base)
    output reg  [10:0] o_nd_len,
    input  wire        i_nd_rdy,  // harness latched the request this cycle
    input  wire        i_nd_dvld, // one returned word
    input  wire [63:0] i_nd_data
);

    //------------------------------------------------------------------
    // geometry
    //------------------------------------------------------------------
    localparam integer PAGE_WORDS = 264;          // 2112 B / 8
    localparam [10:0]  PAGE_LEN   = 11'd264;
    localparam [11:0]  PAGE_BYTES = 12'd2112;
    localparam [15:0]  RST_BUSY   = 16'd16;        // short R/B# low after FFh

    // chip identity (K9F1G08U0M): EC F1 00 95 40 - constants, not image data
    function automatic [7:0] id_byte(input [2:0] i);
        case (i)
            3'd0:    id_byte = 8'hEC;   // manufacturer (Samsung; ID-patched)
            3'd1:    id_byte = 8'hF1;   // device (1 Gbit, 3.3 V)
            3'd2:    id_byte = 8'h00;
            3'd3:    id_byte = 8'h95;
            default: id_byte = 8'h40;
        endcase
    endfunction

    //------------------------------------------------------------------
    // strobe edge detection (single i_CLK domain)
    //------------------------------------------------------------------
    reg we_d, re_d;
    wire we_rise = ~we_d &  i_We_n;   // WE# 0->1 : latch command / address
    wire re_fall =  re_d & ~i_Re_n;   // RE# 1->0 : clock the next byte onto DQ
    wire re_rise = ~re_d &  i_Re_n;   // RE# 0->1 : end of strobe (reader samples here)

    //------------------------------------------------------------------
    // decode / streaming state
    //------------------------------------------------------------------
    localparam [2:0] M_IDLE=3'd0, M_ADDR=3'd1, M_BUSY=3'd2, M_STREAM=3'd3,
                     M_ID=3'd4, M_STATUS=3'd5, M_RDCOL=3'd6;
    reg [2:0]  mode   /*verilator public_flat_rd*/;

    reg [11:0] col_addr, col_ctr;
    reg [15:0] row_addr;
    reg [1:0]  addr_cnt;              // 0..3 address cycles
    reg [2:0]  id_ptr;               // 0..4 read-ID pointer
    reg [15:0] rst_cnt;

    // page register (fetched whole; streamed from col_addr)
    localparam [1:0] F_REQ=2'd0, F_RECV=2'd1;
    reg [1:0]  fstate;
    reg [8:0]  recv_cnt /*verilator public_flat_rd*/;   // 0..264
    reg [63:0] page_buf [0:PAGE_WORDS-1];

    // page base word: NAND_BASE_W + row*264  (264 = 256+8 -> shift-add, no DSP)
    wire [28:0] page_base_w = NAND_BASE_W + {row_addr, 8'd0} + {row_addr, 3'd0};

    wire busy = (mode == M_BUSY) || (rst_cnt != 16'd0);
    assign o_Rb_n = ~busy;

    //------------------------------------------------------------------
    // output register: the chip clocks a byte onto DQ on each RE# fall and
    // drives it through RE# low PLUS a short hold past the rising edge (the
    // tRHOH output hold) - so a reader sampling at RE# rise (the board /
    // DMA_MON probe) sees valid data - then RELEASES.  The hold DECAYS, so
    // between accesses DQ floats and never fights the CPU on the shared bus
    // (CLE=A0/ALE=A1 mean the chip stays selected across non-NAND cycles).
    //------------------------------------------------------------------
    wire [63:0] cur_word  = page_buf[col_ctr[11:3]];
    wire [7:0]  page_byte = cur_word[{col_ctr[2:0], 3'b000} +: 8];
    wire [7:0]  status_byte = busy ? 8'h00 : {i_Wp_n, 2'b11, 5'b0_0000};
    wire        out_mode  = (mode == M_STREAM || mode == M_ID || mode == M_STATUS);

    reg [7:0] dout_reg;
    reg [1:0] oe_hold;                // cycles of tRHOH hold left past RE# rise
    assign o_Dq    = dout_reg;
    assign o_Dq_oe = ~i_Ce_n & i_We_n & out_mode
                     & (~i_Re_n | re_rise | (oe_hold != 2'd0));

    //------------------------------------------------------------------
    // sequential control
    //------------------------------------------------------------------
    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            mode <= M_IDLE; addr_cnt <= 2'd0; id_ptr <= 3'd0;
            col_addr <= 12'd0; row_addr <= 16'd0; col_ctr <= 12'd0;
            rst_cnt <= 16'd0; fstate <= F_REQ; recv_cnt <= 9'd0;
            o_nd_req <= 1'b0; o_nd_addr <= 29'd0; o_nd_len <= 11'd0;
            we_d <= 1'b1; re_d <= 1'b1;
            dout_reg <= 8'h00; oe_hold <= 2'd0;
        end
        else begin
            we_d <= i_We_n;
            re_d <= i_Re_n;
            if (rst_cnt != 16'd0) rst_cnt <= rst_cnt - 16'd1;

            // ---- command / address latch on WE# rising (chip enabled) ----
            if (we_rise && !i_Ce_n) begin
                if (i_Cle) begin
                    case (i_Dq)
                        8'hFF: begin mode <= M_IDLE; rst_cnt <= RST_BUSY; addr_cnt <= 2'd0; end
                        8'h90: begin mode <= M_ID;   id_ptr <= 3'd0; addr_cnt <= 2'd0; end
                        8'h70: begin mode <= M_STATUS; end
                        8'h00: begin mode <= M_ADDR; addr_cnt <= 2'd0; end
                        8'h30: begin mode <= M_BUSY; fstate <= F_REQ; recv_cnt <= 9'd0; end
                        8'h05: begin mode <= M_RDCOL; addr_cnt <= 2'd0; end
                        8'hE0: begin mode <= M_STREAM; col_ctr <= col_addr; end
                        default: ; // program/erase & friends: read-only, ignore
                    endcase
                end
                else if (i_Ale) begin
                    if (mode == M_RDCOL) begin
                        if (addr_cnt == 2'd0) col_addr[7:0]  <= i_Dq;
                        else                  col_addr[11:8] <= i_Dq[3:0];
                        addr_cnt <= addr_cnt + 2'd1;
                    end
                    else if (mode == M_ADDR) begin
                        case (addr_cnt)
                            2'd0: col_addr[7:0]   <= i_Dq;
                            2'd1: col_addr[11:8]  <= i_Dq[3:0];
                            2'd2: row_addr[7:0]   <= i_Dq;
                            2'd3: row_addr[15:8]  <= i_Dq;
                        endcase
                        addr_cnt <= addr_cnt + 2'd1;
                    end
                    // M_ID: single 00h address cycle, ignored (ID starts at 0)
                end
            end

            // ---- clock a byte onto DQ on RE# fall; short hold past RE# rise ----
            if (re_fall && !i_Ce_n && out_mode) begin
                case (mode)
                    M_STREAM: begin dout_reg <= page_byte; col_ctr <= col_ctr + 12'd1; end
                    M_ID:     begin dout_reg <= id_byte(id_ptr);
                                    if (id_ptr != 3'd4) id_ptr <= id_ptr + 3'd1; end
                    default:  dout_reg <= status_byte;         // M_STATUS (no advance)
                endcase
            end
            if (re_rise)              oe_hold <= 2'd2;          // arm tRHOH hold
            else if (oe_hold != 2'd0) oe_hold <= oe_hold - 2'd1; // then decay -> float

            // ---- page fetch: DDR3 harness NAND client ----
            if (mode == M_BUSY) begin
                case (fstate)
                    F_REQ: begin
                        o_nd_req  <= 1'b1;
                        o_nd_addr <= page_base_w;
                        o_nd_len  <= PAGE_LEN;
                        if (i_nd_rdy) begin
                            o_nd_req <= 1'b0;
                            fstate   <= F_RECV;
                            recv_cnt <= 9'd0;
                        end
                    end
                    F_RECV: begin
                        if (i_nd_dvld) begin
                            page_buf[recv_cnt] <= i_nd_data;
                            if (recv_cnt == 9'd263) begin
                                mode    <= M_STREAM;
                                col_ctr <= col_addr;
                                fstate  <= F_REQ;
                            end
                            recv_cnt <= recv_cnt + 9'd1;
                        end
                    end
                    default: fstate <= F_REQ;
                endcase
            end
            else begin
                o_nd_req <= 1'b0;
            end
        end
    end

`ifndef SYNTHESIS
    // read past the page register (2112 B) is undefined on the real chip
    always @(posedge i_CLK)
        if (i_RST_n && mode == M_STREAM && re_fall && !i_Ce_n
            && col_ctr >= PAGE_BYTES)
            $display("[CV1k_nand] WARNING: read past page register col=%0d t=%0t",
                     col_ctr, $time);

    // bring-up trace (+ndtrace): command stream + page-fetch lifecycle
    reg ndtrace = 1'b0;
    reg [2:0] mode_dbg;
    initial ndtrace = $test$plusargs("ndtrace");
    always @(posedge i_CLK) if (i_RST_n && ndtrace) begin
        mode_dbg <= mode;
        if (we_rise && !i_Ce_n && i_Cle)
            $display("[nd] cmd %02x (row=%0d col=%0d) t=%0t", i_Dq, row_addr, col_addr, $time);
        if (mode == M_BUSY && mode_dbg != M_BUSY)
            $display("[nd] page fetch start row=%0d base=%07x t=%0t", row_addr, page_base_w, $time);
        if (o_nd_req && i_nd_rdy)
            $display("[nd] fetch GRANTED len=%0d t=%0t", o_nd_len, $time);
        if (mode == M_STREAM && mode_dbg == M_BUSY)
            $display("[nd] fetch DONE -> streaming row=%0d rb_n=%0b t=%0t", row_addr, o_Rb_n, $time);
    end
`endif

endmodule
`default_nettype none
