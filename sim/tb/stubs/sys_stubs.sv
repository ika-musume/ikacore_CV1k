`default_nettype none
//============================================================================
// sys_stubs.sv - LINT-ONLY stubs for the MiSTer sys/ modules  [H7b.1]
//
// ikacore_CV1k_emu.sv (module emu) is Quartus-only: on target it links
// against the real `srcs/sys/` (hps_io.sv contains cyclonev_hps_interface_*
// atoms, pll is generated Quartus IP - neither elaborates under Verilator).
// These stubs carry ONLY the port shapes so `./build_emu_lint.sh` can
// elaborate the wrapper and catch port/name drift early.  NEVER simulate
// with these; they are not models.
//============================================================================

module pll (
    input  wire        refclk,
    input  wire        rst,
    output wire        outclk_0,          // 153.6 MHz  blit/DDR3 domain
    output wire        outclk_1,          // 102.4 MHz  CPU/board/SDRAM domain
    output wire        outclk_2,          // 102.4 MHz  SDRAM_CLK (phase-tunable)
    output wire        locked,
    input  wire [63:0] reconfig_to_pll,   // MGMT: OSD SDRAM-phase nudge only
    output wire [63:0] reconfig_from_pll
);
    assign outclk_0 = 1'b0;
    assign outclk_1 = 1'b0;
    assign outclk_2 = 1'b0;
    assign locked   = 1'b1;
    assign reconfig_from_pll = 64'd0;
    wire _unused = &{1'b0, refclk, rst, reconfig_to_pll, 1'b0};
endmodule

// Altera PLL Reconfig wrapper (sys/pll_cfg/pll_cfg.v) - MGMT-port face only
module pll_cfg (
    input  wire        mgmt_clk,
    input  wire        mgmt_reset,
    output wire        mgmt_waitrequest,
    input  wire        mgmt_read,
    output wire [31:0] mgmt_readdata,
    input  wire        mgmt_write,
    input  wire  [5:0] mgmt_address,
    input  wire [31:0] mgmt_writedata,
    output wire [63:0] reconfig_to_pll,
    input  wire [63:0] reconfig_from_pll
);
    assign mgmt_waitrequest = 1'b0;
    assign mgmt_readdata    = 32'd0;
    assign reconfig_to_pll  = 64'd0;
    wire _unused = &{1'b0, mgmt_clk, mgmt_reset, mgmt_read, mgmt_write,
                     mgmt_address, mgmt_writedata, reconfig_from_pll, 1'b0};
endmodule

module hps_io #(
    parameter CONF_STR = "",
    parameter [5:0] WIDE = 0
) (
    input  wire         clk_sys,
    inout  wire [45:0]  HPS_BUS,
    inout  wire [35:0]  EXT_BUS,
    inout  wire [21:0]  gamma_bus,

    output wire         forced_scandoubler,
    output wire [1:0]   buttons,
    output wire [127:0] status,
    input  wire [15:0]  status_menumask,

    output wire         ioctl_download,
    output wire [15:0]  ioctl_index,
    output wire         ioctl_wr,
    output wire [26:0]  ioctl_addr,
    output wire [7:0]   ioctl_dout,
    input  wire         ioctl_wait,

    output wire [31:0]  joystick_0,
    output wire [31:0]  joystick_1
);
    assign forced_scandoubler = 1'b0;
    assign buttons        = 2'd0;
    assign status         = 128'd0;
    assign ioctl_download = 1'b0;
    assign ioctl_index    = 16'd0;
    assign ioctl_wr       = 1'b0;
    assign ioctl_addr     = 27'd0;
    assign ioctl_dout     = 8'd0;
    assign joystick_0     = 32'd0;
    assign joystick_1     = 32'd0;
    wire _unused = &{1'b0, clk_sys, status_menumask, ioctl_wait, 1'b0};
endmodule
`default_nettype wire
