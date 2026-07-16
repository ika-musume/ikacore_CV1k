`default_nettype none
//============================================================================
// blit_port_beh.sv - behavioral train port for blit_batch        [H7a step 3]
//
// The "perfect fake port" of the step-3 accept: same 64 MB flat-pixel image
// as blit_vram_beh (mem[] public at the same rank so tb_blit_main.cpp's
// init/compare code is backend-agnostic), served through blit_batch's
// train-level interface instead of the fixed-latency beat channels.
//
// Semantics the RTL relies on (and that CV1k_ddr3_harness must also provide):
//   * strictly in-order: read commands are FIFO'd and their data returned
//     in acceptance order; a write accepted before a read command is
//     visible to it (here: writes commit at acceptance).
//   * read data words stream back-to-back (perfect) or with seeded gaps
//     (+portjit=SEED) - blit_batch must be timing-agnostic, so a jittered
//     run must stay pixel-exact.  Jitter also stalls command/write accepts.
//
// The batch layer never opens a write train while a read train is in
// flight; violating that would reorder R/W visibility on real hardware,
// so it is checked here ($fatal).
//============================================================================
module blit_port_beh (
    input  wire        i_CLK,
    input  wire        i_RST_n,

    input  wire        i_prd_req,
    input  wire [22:0] i_prd_addr,
    input  wire [10:0] i_prd_len,
    output wire        o_prd_rdy,
    output reg         o_prd_dvld,
    output reg  [63:0] o_prd_data,

    input  wire        i_pwr_req,
    input  wire [22:0] i_pwr_addr,
    input  wire [63:0] i_pwr_data,
    input  wire [3:0]  i_pwr_be,
    output wire        o_pwr_rdy,

    input  wire        i_rd_train,
    input  wire        i_wr_train
);

    reg [15:0] mem [0:33554431] /*verilator public_flat_rw*/;

    initial begin : blk_init              // power-up = all-black VRAM (golden)
        for (int unsigned i = 0; i < 33554432; i++) mem[i] = 16'h0000;
    end

    // seeded jitter (0 = perfect port)
    reg [31:0] jit;
    int unsigned wt_lo = 1, wt_hi = 0;    // temporary bring-up write trace
    initial begin
        jit = 32'd0;
        void'($value$plusargs("portjit=%d", jit));
        void'($value$plusargs("wtrace_lo=%d", wt_lo));
        void'($value$plusargs("wtrace_hi=%d", wt_hi));
        if (jit != 0)
            $display("[blit_port_beh] jitter seed %0d", jit);
    end
    reg [31:0] lfsr = 32'h1;
    always_ff @(posedge i_CLK)
        if (jit != 0)
            lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]} ^ jit;
    wire j_cmd  = (jit != 0) && (lfsr[3:0]  == 4'h3);   // stall cmd accept
    wire j_dat  = (jit != 0) && (lfsr[7:4]  <  4'h6);   // gap in read data
    wire j_wr   = (jit != 0) && (lfsr[11:8] == 4'h5);   // stall write accept

    // ---------------------------------------------------------------------
    // command FIFO (16 deep) + in-order data streamer
    // ---------------------------------------------------------------------
    reg [33:0] cq [0:15];                 // {addr[22:0], len[10:0]}
    reg [3:0]  cq_wp, cq_rp;
    reg [4:0]  cq_cnt;

    assign o_prd_rdy = (cq_cnt < 5'd16) && !j_cmd;
    assign o_pwr_rdy = !j_wr;

    reg        run_v;
    reg [22:0] run_a;
    reg [10:0] run_n;

    wire cmd_acc = i_prd_req && o_prd_rdy;
    wire pop_cmd = !run_v && (cq_cnt != 5'd0);

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            cq_wp <= '0; cq_rp <= '0; cq_cnt <= '0;
            run_v <= 1'b0; run_a <= '0; run_n <= '0;
            o_prd_dvld <= 1'b0;
        end
        else begin
            o_prd_dvld <= 1'b0;

            if (cmd_acc) begin
                cq[cq_wp] <= {i_prd_addr, i_prd_len};
                cq_wp <= cq_wp + 4'd1;
            end

            if (pop_cmd) begin
                run_v <= 1'b1;
                run_a <= cq[cq_rp][33:11];
                run_n <= cq[cq_rp][10:0];
                cq_rp <= cq_rp + 4'd1;
            end
            cq_cnt <= cq_cnt + (cmd_acc ? 5'd1 : 5'd0)
                             - (pop_cmd ? 5'd1 : 5'd0);

            if (run_v && !j_dat) begin
                o_prd_dvld <= 1'b1;
                o_prd_data <= {mem[{run_a, 2'd3}], mem[{run_a, 2'd2}],
                               mem[{run_a, 2'd1}], mem[{run_a, 2'd0}]};
                run_a <= run_a + 23'd1;
                run_n <= run_n - 11'd1;
                if (run_n == 11'd1) run_v <= 1'b0;
            end

            if (i_pwr_req && o_pwr_rdy) begin
`ifndef SYNTHESIS
                if (run_v || cq_cnt != 5'd0 || i_rd_train)
                    $fatal(2, "[blit_port_beh] write during open read train t=%0t", $time);
                if ({i_pwr_addr, 2'd0} >= 25'(wt_lo) && {i_pwr_addr, 2'd0} <= 25'(wt_hi))
                    $display("[pwr] w=%06x be=%b d=%016x t=%0t",
                             i_pwr_addr, i_pwr_be, i_pwr_data, $time);
`endif
                if (i_pwr_be[0]) mem[{i_pwr_addr, 2'd0}] <= i_pwr_data[15:0];
                if (i_pwr_be[1]) mem[{i_pwr_addr, 2'd1}] <= i_pwr_data[31:16];
                if (i_pwr_be[2]) mem[{i_pwr_addr, 2'd2}] <= i_pwr_data[47:32];
                if (i_pwr_be[3]) mem[{i_pwr_addr, 2'd3}] <= i_pwr_data[63:48];
            end
        end
    end

    wire unused = i_wr_train;

endmodule
`default_nettype none
