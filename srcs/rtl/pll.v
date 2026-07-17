// ikacore_CV1k PLL wrapper.  [H7b.8]
// The physical plan (exact-ratio 153.6 / 102.4 / SDRAM_CLK off ONE
// 1228.8 MHz fractional VCO) lives in pll/pll_0002.v - see its header.
// reconfig_to/from_pll go to the emu-level pll_cfg instance, which is
// used ONLY for the OSD SDRAM-phase nudge (dynamic phase shift on C2);
// the frequency plan is static.

`timescale 1 ps / 1 ps
module pll (
		input  wire        refclk,            //            refclk.clk
		input  wire        rst,               //             reset.reset
		output wire        outclk_0,          //           outclk0.clk  153.6 MHz
		output wire        outclk_1,          //           outclk1.clk  102.4 MHz
		output wire        outclk_2,          //           outclk2.clk  SDRAM_CLK
		output wire        locked,            //            locked.export
		input  wire [63:0] reconfig_to_pll,   //   reconfig_to_pll.reconfig_to_pll
		output wire [63:0] reconfig_from_pll  // reconfig_from_pll.reconfig_from_pll
	);

	pll_0002 pll_inst (
		.refclk            (refclk),
		.rst               (rst),
		.outclk_0          (outclk_0),
		.outclk_1          (outclk_1),
		.outclk_2          (outclk_2),
		.locked            (locked),
		.reconfig_to_pll   (reconfig_to_pll),
		.reconfig_from_pll (reconfig_from_pll)
	);

endmodule
