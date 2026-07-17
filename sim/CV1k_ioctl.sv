`default_nettype none
//============================================================================
// CV1k_ioctl.sv - HPS ioctl download decoder + DDR3 packer          [H7b.4]
//
// Splits the ONE concatenated MRA download stream into the CV1k image
// stores, keyed on its OWN byte counter - hps_io's ioctl_addr is [26:0]
// = 128 MiB and the u2 part alone is 132 MiB, so ioctl_addr WRAPS
// mid-NAND and must never be trusted (i_ADDR is accepted only for its
// documented role in the port list; nothing here reads it).
//
// Fixed game-agnostic layout (plan of record 2026-07-16; the MRA
// zero-pads short parts - see tb/ioctl_sim.sv for the sim streamer):
//
//   stream bytes                  target                       word base
//   [0,          0x840_0000)     NAND u2 132 MB   -> DDR3     0x0680_0000
//   [0x840_0000, 0x8C0_0000)     YMZ u23 8 MB slot-> DDR3     0x07A0_0000
//   [0x8C0_0000, 0x940_0000)     YMZ u24 8 MB slot-> DDR3     0x07B0_0000
//   [0x940_0000, 0x980_0000)     u4 x2 (4 MiB)    -> pump NOR window,
//                                                     address rebased to 0
//   >= 0x980_0000                ignored (one-shot note)
//
// DDR3 lane rule: stream byte c -> 64-bit lane c%8 little-endian (= the
// raw dump order = CV1k_nand's page layout).  All region boundaries are
// 8-byte aligned, so the packer never straddles regions.
//
// NOR byte order: the pump (NOR_BSWAP=0) assembles {odd,even} halfwords
// itself; this decoder only rebases the byte address (0x940_0000 is even,
// so byte-pair parity is preserved by construction).  Stream "3d,df" ->
// bus halfword 0xdf3d - the accept probe.
//
// Clocking: the ioctl/pump side lives on i_CLK (102.4 MHz, = hps_io
// clk_sys); the DDR3 write face lives on i_CLK_DDR (153.6 MHz, the DDRAM
// face domain).  One packed word crosses per toggle handshake (req_t /
// ack_t, 2-FF synced both ways); o_WAIT throttles the HPS while a word
// is in flight, so the crossing is strictly single-outstanding.  The
// board top muxes the DDRAM command pins to this module while o_DDR_OWN
// (DDR domain) - the core is held in reset for the whole download
// (o_HOLD keeps the reset sequencer's dl_hold up until the last word has
// drained), so the harness never contends for the face.
//============================================================================
module CV1k_ioctl (
    //------ ioctl / pump domain ------
    input  wire         i_CLK,           // 102.4 MHz (hps_io clk_sys)
    input  wire         i_RST_n,         // memory-subsystem reset (INITRST)

    // HPS ioctl in (hps_io WIDE=0: 8-bit data)
    input  wire         i_DOWNLOAD,
    input  wire         i_WR,
    input  wire [26:0]  i_ADDR,          // wraps at 128 MiB - documentation only
    input  wire [7:0]   i_DATA,
    input  wire [15:0]  i_INDEX,         // only index 0 (the MRA ROM set) decoded
    output wire         o_WAIT,

    // CPU hold: download in flight OR packer draining (reset sequencer)
    output wire         o_HOLD,

    // u4 sub-stream -> CV1k_sdram_control ioctl port (rebased to 0)
    output wire         o_NOR_DOWNLOAD,
    output reg          o_NOR_WR,
    output reg  [26:0]  o_NOR_ADDR,
    output reg  [7:0]   o_NOR_DATA,
    input  wire         i_NOR_WAIT,      // pump's o_IOCTL_WAIT back in

    //------ DDR3 face domain ------
    input  wire         i_CLK_DDR,       // 153.6 MHz (DDRAM face clock)
    output wire         o_DDR_OWN,       // board top: mux the DDRAM face here
    output reg          o_DDR_WE,        // held until !i_DDR_BUSY (f2sdram rule)
    output reg  [28:0]  o_DDR_ADDR,
    output reg  [63:0]  o_DDR_DIN,
    output reg  [7:0]   o_DDR_BE,
    input  wire         i_DDR_BUSY
);

// stream layout (byte counter units)
localparam [27:0] NAND_END = 28'h840_0000;
localparam [27:0] U23_END  = 28'h8C0_0000;
localparam [27:0] YMZ_END  = 28'h940_0000;
localparam [27:0] U4_END   = 28'h980_0000;

// DDR3 word bases (64-bit-word addresses on DDRAM_ADDR)
localparam [28:0] NAND_BASE_W = 29'h0680_0000;   // byte 0x3400_0000
localparam [28:0] U23_BASE_W  = 29'h07A0_0000;   // byte 0x3D00_0000
localparam [28:0] U24_BASE_W  = 29'h07B0_0000;   // u23 + 8 MB chip slot

//------------------------------------------------------------------
// byte counter + region decode (102.4 domain)
//------------------------------------------------------------------
reg  [27:0] cnt;                         // accepted index-0 bytes this download
reg         dl_d;
wire        beat = i_DOWNLOAD && i_WR && (i_INDEX == 16'h0000);

wire in_ddr3 = (cnt < YMZ_END);
wire in_u4   = (cnt >= YMZ_END) && (cnt < U4_END);

// stream byte -> DDR3 word address (region-relative offset >> 3; the
// region starts are 8-byte aligned so the word subtraction is exact)
wire [24:0] cw = cnt[27:3];
wire [28:0] ddr3_word =
    (cnt < NAND_END) ? NAND_BASE_W + {4'd0, cw} :
    (cnt < U23_END)  ? U23_BASE_W  + {4'd0, cw - 25'(NAND_END >> 3)} :
                       U24_BASE_W  + {4'd0, cw - 25'(U23_END  >> 3)};

//------------------------------------------------------------------
// 8-byte packer + single-outstanding toggle handshake into the DDR
// domain.  pk_* stay latched while pk_busy, so the DDR side reads a
// stable payload after its 2-FF sync of pk_req_t.
//------------------------------------------------------------------
reg  [63:0] pk_data;
reg  [7:0]  pk_be_acc;                   // lanes filled so far (tail flush)
reg  [28:0] pk_addr;
reg  [63:0] pk_word;
reg  [7:0]  pk_be;
reg         pk_req_t;
reg  [1:0]  pk_ack_s;                    // ack_t synced back (2-FF)
wire        pk_busy = (pk_req_t != pk_ack_s[1]);
reg  [28:0] pk_cur_word;                 // word the bytes in pk_data belong to

reg         over_note;
reg         flush_pend;                  // download ended mid-word

always @(posedge i_CLK or negedge i_RST_n) begin
    if (!i_RST_n) begin
        cnt <= 28'd0;   dl_d <= 1'b0;
        pk_data <= 64'd0; pk_be_acc <= 8'd0; pk_cur_word <= 29'd0;
        pk_addr <= 29'd0; pk_word <= 64'd0; pk_be <= 8'd0; pk_req_t <= 1'b0;
        pk_ack_s <= 2'b00;
        o_NOR_WR <= 1'b0; o_NOR_ADDR <= 27'd0; o_NOR_DATA <= 8'd0;
        over_note <= 1'b0; flush_pend <= 1'b0;
    end
    else begin
        pk_ack_s <= {pk_ack_s[0], ack_t_ddr};
        dl_d     <= i_DOWNLOAD;
        if (i_DOWNLOAD && !dl_d) begin
            cnt        <= 28'd0;         // new download: restart the layout
            pk_be_acc  <= 8'd0;
            over_note  <= 1'b0;
            flush_pend <= 1'b0;
        end

        o_NOR_WR <= 1'b0;                // 1-cycle pulse per u4 byte
        if (beat) begin
            cnt <= cnt + 28'd1;
            if (in_ddr3) begin
                pk_data[{cnt[2:0], 3'b000} +: 8] <= i_DATA;
                pk_be_acc                        <= pk_be_acc | (8'd1 << cnt[2:0]);
                pk_cur_word                      <= ddr3_word;
                if (cnt[2:0] == 3'd7) begin      // word complete -> launch
                    pk_addr   <= ddr3_word;
                    pk_word   <= {i_DATA, pk_data[55:0]};
                    pk_be     <= 8'hFF;
                    pk_req_t  <= ~pk_req_t;
                    pk_be_acc <= 8'd0;
                end
            end
            else if (in_u4) begin
                o_NOR_WR   <= 1'b1;
                o_NOR_ADDR <= {5'd0, cnt[21:0]};     // rebased to 0 (parity kept)
                o_NOR_DATA <= i_DATA;
            end
`ifndef SYNTHESIS
            else if (!over_note) begin
                $display("[CV1k_ioctl] note: stream bytes past 0x%07x ignored @%0t",
                         U4_END, $time);
                over_note <= 1'b1;
            end
`endif
        end
        // download ended mid-word (truncated stream): flush the partial
        // word with the lanes actually received, waiting out any word
        // still crossing (flush_pend keeps the request armed)
        if (!i_DOWNLOAD && dl_d && (pk_be_acc != 8'd0))
            flush_pend <= 1'b1;
        if (flush_pend && !pk_busy) begin
            pk_addr    <= pk_cur_word;
            pk_word    <= pk_data;
            pk_be      <= pk_be_acc;
            pk_req_t   <= ~pk_req_t;
            pk_be_acc  <= 8'd0;
            flush_pend <= 1'b0;
        end
    end
end

assign o_NOR_DOWNLOAD = i_DOWNLOAD;
assign o_WAIT = pk_busy | i_NOR_WAIT;
assign o_HOLD = i_DOWNLOAD | pk_busy | flush_pend | (pk_be_acc != 8'd0)
              | i_NOR_WAIT;

//------------------------------------------------------------------
// DDR3 write side (153.6 domain): sync the request toggle, present the
// word, hold WE until the face takes it (!BUSY), toggle the ack back.
// o_DDR_OWN covers the whole download plus any in-flight word, so the
// board top's mux only ever hands a quiet face back to the harness
// (which is in reset throughout - o_HOLD keeps the CPU POR down).
//------------------------------------------------------------------
reg  [1:0] req_s;                        // pk_req_t synced (2-FF)
reg        ack_t_ddr;
reg  [2:0] dl_s;                         // i_DOWNLOAD synced + hold

always @(posedge i_CLK_DDR or negedge i_RST_n) begin
    if (!i_RST_n) begin
        req_s <= 2'b00; ack_t_ddr <= 1'b0; dl_s <= 3'b000;
        o_DDR_WE <= 1'b0; o_DDR_ADDR <= 29'd0;
        o_DDR_DIN <= 64'd0; o_DDR_BE <= 8'd0;
    end
    else begin
        req_s <= {req_s[0], pk_req_t};
        dl_s  <= {dl_s[1:0], i_DOWNLOAD};
        if (!o_DDR_WE && (req_s[1] != ack_t_ddr)) begin
            o_DDR_WE   <= 1'b1;          // payload is stable while pk_busy
            o_DDR_ADDR <= pk_addr;
            o_DDR_DIN  <= pk_word;
            o_DDR_BE   <= pk_be;
        end
        else if (o_DDR_WE && !i_DDR_BUSY) begin
            o_DDR_WE  <= 1'b0;
            ack_t_ddr <= req_s[1];
        end
    end
end

assign o_DDR_OWN = (|dl_s) | o_DDR_WE | (req_s[1] != ack_t_ddr);

wire _unused = &{1'b0, i_ADDR, i_INDEX[15:1], 1'b0};

endmodule
`default_nettype wire
