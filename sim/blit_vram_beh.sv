`default_nettype none
//============================================================================
// blit_vram_beh.sv - behavioral blitter VRAM backend            [H3 / I-1.3]
//
// The ideal stand-in for the 2x MT46V16M16 blitter DDR (one logical
// 8192x4096 ARGB1555 image = 64 MB), first backend of the swappable-VRAM
// plan (ideal behavioral -> DDR3-stat -> MiSTer DDR3, I-2.6/I-4.3).
//
// Flat pixel addressing: mem[p] = pixel at (x = p % 8192, y = p / 8192),
// matching the golden model's linear framebuffer (the DDR tile swizzle of
// BD §6.1 is pixel-invisible and belongs to the real backend's address
// stage).  Three independent channels at full rate - reads serve a 4-pixel
// beat {px3..px0} = mem[a+3..a] one cycle after the request cycle and HOLD
// the data until the next request (the contract blit_draw's pipe relies
// on); writes commit the masked lanes at the request edge.  Lane addresses
// wrap mod 2^25, reproducing golden's flat-index row spill.
//
// Sim-only (the mem array is 64 MB); the TB reaches in via /*verilator
// public*/ for init/compare/dump.
//============================================================================
module blit_vram_beh (
    input  wire        i_CLK,

    // src read channel
    input  wire        i_srd_req,
    input  wire [24:0] i_srd_addr,
    output reg  [63:0] o_srd_data,

    // dst read channel
    input  wire        i_drd_req,
    input  wire [24:0] i_drd_addr,
    output reg  [63:0] o_drd_data,

    // video scanout read channel (H5 line fetcher; same hold contract)
    input  wire        i_vrd_req,
    input  wire [24:0] i_vrd_addr,
    output reg  [63:0] o_vrd_data,

    // write channel (per-pixel lane enables - no read-modify-write)
    input  wire        i_wr_req,
    input  wire [24:0] i_wr_addr,
    input  wire [63:0] i_wr_data,
    input  wire [3:0]  i_wr_mask,
    output wire        o_wr_rdy
);

    reg [15:0] mem [0:33554431] /*verilator public_flat_rw*/;

    assign o_wr_rdy = 1'b1;               // ideal backend never backpressures

    initial begin : blk_init              // power-up = all-black VRAM (golden)
        for (int unsigned i = 0; i < 33554432; i++) mem[i] = 16'h0000;
    end

    always @(posedge i_CLK) begin
        if (i_srd_req)
            o_srd_data <= {mem[(i_srd_addr + 25'd3) & 25'h1ffffff],
                           mem[(i_srd_addr + 25'd2) & 25'h1ffffff],
                           mem[(i_srd_addr + 25'd1) & 25'h1ffffff],
                           mem[ i_srd_addr]};
        if (i_drd_req)
            o_drd_data <= {mem[(i_drd_addr + 25'd3) & 25'h1ffffff],
                           mem[(i_drd_addr + 25'd2) & 25'h1ffffff],
                           mem[(i_drd_addr + 25'd1) & 25'h1ffffff],
                           mem[ i_drd_addr]};
        if (i_vrd_req)
            o_vrd_data <= {mem[(i_vrd_addr + 25'd3) & 25'h1ffffff],
                           mem[(i_vrd_addr + 25'd2) & 25'h1ffffff],
                           mem[(i_vrd_addr + 25'd1) & 25'h1ffffff],
                           mem[ i_vrd_addr]};
        if (i_wr_req) begin
            if (i_wr_mask[0]) mem[ i_wr_addr]                        <= i_wr_data[15:0];
            if (i_wr_mask[1]) mem[(i_wr_addr + 25'd1) & 25'h1ffffff] <= i_wr_data[31:16];
            if (i_wr_mask[2]) mem[(i_wr_addr + 25'd2) & 25'h1ffffff] <= i_wr_data[47:32];
            if (i_wr_mask[3]) mem[(i_wr_addr + 25'd3) & 25'h1ffffff] <= i_wr_data[63:48];
        end
    end

endmodule
`default_nettype none
