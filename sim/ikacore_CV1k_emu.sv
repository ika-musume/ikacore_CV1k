//============================================================================
// ikacore_CV1k_emu.sv - MiSTer framework wrapper (module emu)  [H7b.1]
//
// The MiSTer-ONLY half of the two-file split (plan of record 2026-07-16,
// the Arcade-Psychic5 pattern): framework port list (`srcs/sys/emu_ports.vh`
// verbatim - the include IS the port-parity guarantee), hps_io, the PLL,
// CONF_STR (DIP S2 / key map / OSD), the reset policy, and the video output
// stage.  Everything CV1000 lives in the portable core top `ikacore_CV1k.sv`.
//
// This file is QUARTUS-ONLY: simulation drives ikacore_CV1k directly
// (tb_cv1k / ikacore_CV1k_tb).  It still elaborates under Verilator with
// the lint stubs in tb/stubs/ (`./build_emu_lint.sh`) so port/name drift is
// caught long before the H7b.8 Quartus pass.
//
// Clocking (H7b.8, built): ONE fractional PLL VCO at (exact - 6 mHz)
// 1228.8 MHz, STATIC plan hand-authored in srcs/rtl/pll/pll_0002.v via
// the advanced physical parameters - outclk_0 = 153.6 MHz (blit/DDR3
// domain), outclk_1 = 102.4 MHz (CPU/board/SDRAM domain), outclk_2 =
// SDRAM_CLK (102.4 MHz, C2 phase preset +77 VCO/8 taps = +7833 ps ~
// -72 deg lead per the Psychic5/BubSys scaling).  The reconfig MGMT
// port serves ONLY the OSD dynamic-phase nudge (below) - no frequency
// retune ever happens at runtime, so POR determinism (the soft-reset
// CKIO-parity carry-over) has no PLL term beyond pll_locked itself.
//
// Reset policy (MiSTer compliance, recorded 2026-07-16; download term
// refined at H7b.4):
//   hard reset = RESET | ~pll_locked      -> full chain (pump JEDEC
//                re-init + CPU POR)
//   ioctl_download                        -> handled INSIDE the core: the
//                reset sequencer's dl_hold keeps the CPU in POR for the
//                whole download + the decoder's drain, then re-runs the
//                POR/RESETM stagger.  It must NOT drive the hard reset -
//                the pump's loader and JEDEC-init state have to be alive
//                to accept the stream (H7b.4).
//   soft reset = OSD status[0] | buttons[1] -> CPU/blitter reboot only;
//                pump init state and SDRAM/DDR3 contents preserved
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_SL         = 0;
assign VGA_F1         = 0;
assign VGA_SCALER     = 0;
assign VGA_DISABLE    = 0;
assign HDMI_FREEZE    = 0;
assign HDMI_BLACKOUT  = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S   = 1;              // YMZ770C-F emits signed PCM (later phase)
assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

// screen rotation deliberately IGNORED (user decision 2026-07-16 - pivot
// monitor); most CV1k titles are tate.  Post-YMZ idea on record: a test
// build overlaying DDR3 bandwidth measurements on screen.
assign VIDEO_ARX = 13'd4;
assign VIDEO_ARY = 13'd3;

//////////////////////////////////////////////////////////////////

`include "build_id.v"
localparam CONF_STR = {
	"CV1k;;",
	"-;",
	"P1,DIP S2;",
	"P1O[4],S2-1,Off,On;",
	"P1O[5],S2-2,Off,On;",
	"P1O[6],S2-3,Off,On;",
	"P1O[7],S2-4,Off,On;",
	"P2,Timing;",
	"P2O[13:9],SDRAM Phase,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"J1,B1,B2,B3,B4,Start,Coin,Test,Service;",
	"V,v",`BUILD_DATE
};

wire  [1:0] buttons;
wire [127:0] status;
wire        forced_scandoubler;    // consumed by the H7b.6 video face

wire        ioctl_download, ioctl_wr, ioctl_wait;
wire [15:0] ioctl_index;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;

wire [31:0] joystick_0, joystick_1;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_102m4),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask(),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1)
);

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_153m6;                    // blit/DDR3 domain (consumed at H7b.2)
wire clk_102m4;                    // CPU/board/SDRAM domain (= 2x CKIO)
wire clk_sdram;                    // SDRAM_CLK: 102.4 MHz, phase-tunable
wire pll_locked;

wire [63:0] reconfig_to_pll, reconfig_from_pll;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_153m6),
	.outclk_1(clk_102m4),
	.outclk_2(clk_sdram),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll),
	.locked(pll_locked)
);

// SDRAM_CLK leaves through a DDIO output cell (H7b.8): the pad toggles from
// the IOE register pair clocked by clk_sdram, so the chip-clock lag vs the
// fabric is the IOE tCO (~2 ns) plus the C2 phase preset -- fit #2 measured
// ~13.8 ns of PLL->pad clock-network route with a plain assign, which
// swallowed the entire CL2 read window.  The preset is (re)derived against
// the DDIO numbers; the OSD phase nudge remains the per-module trim.
`ifdef SYNTHESIS
altddio_out #(
    .extend_oe_disable("OFF"),
    .intended_device_family("Cyclone V"),
    .invert_output("OFF"),
    .lpm_hint("UNUSED"),
    .lpm_type("altddio_out"),
    .oe_reg("UNREGISTERED"),
    .power_up_high("OFF"),
    .width(1)
) u_sdrclk_ddio (
    .datain_h(1'b1),
    .datain_l(1'b0),
    .outclock(clk_sdram),
    .dataout(SDRAM_CLK),
    .aclr(1'b0), .aset(1'b0), .oe(1'b1),
    .outclocken(1'b1), .sclr(1'b0), .sset(1'b0)
);
`else
assign SDRAM_CLK = clk_sdram;   // sim face: ideal clock, no IOE model
`endif

// OSD SDRAM-phase nudge (H7b.8, PERMANENT module-variance trim).
// The frequency plan is STATIC - rtl/pll/pll_0002.v bakes the exact
// 1228.8 MHz fractional VCO and the C2 (~-72 deg) phase preset - so the
// reconfig MGMT port is used ONLY for dynamic phase steps on C2
// (SDRAM_CLK): one VCO/8 tap = 101.7 ps per step, OSD value = signed
// 5-bit tap offset from the preset, +1 = one tap later (more delay /
// less lead).  DPS reg 6 encoding: [15:0] step count, [20:16] cnt_sel
// (C2 = 2), [21] up/dn.  Runs on CLK_50M, free of the clocks it trims;
// each write self-completes (waitrequest covers the shift, no START).
wire        cfg_waitrequest;
reg         cfg_write = 0;
reg   [5:0] cfg_address;
reg  [31:0] cfg_data;

pll_cfg pll_cfg
(
	.mgmt_clk(CLK_50M),
	.mgmt_reset(0),
	.mgmt_waitrequest(cfg_waitrequest),
	.mgmt_read(0),
	.mgmt_readdata(),
	.mgmt_write(cfg_write),
	.mgmt_address(cfg_address),
	.mgmt_writedata(cfg_data),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll)
);

always @(posedge CLK_50M) begin : sdram_phase_nudge
	reg  [4:0] ph_osd_m  = 0;              // 2-FF sync (OSD value is quasi-static)
	reg  [4:0] ph_osd    = 0;
	reg  [1:0] lock_ok   = 0;
	reg signed [5:0] ph_applied = 0;       // taps applied since config load
	reg signed [5:0] ph_target;

	ph_osd_m  <= status[13:9];
	ph_osd    <= ph_osd_m;
	lock_ok   <= {lock_ok[0], pll_locked};
	ph_target  = $signed({ph_osd[4], ph_osd});

	cfg_write <= 0;
	if ((&lock_ok) && !cfg_waitrequest && !cfg_write && ph_applied != ph_target) begin
		cfg_address <= 6'd6;                            // DPS register
		if (ph_applied < ph_target) begin
			cfg_data   <= 32'h0022_0001;                // C2 +1 tap (up)
			ph_applied <= ph_applied + 1'd1;
		end
		else begin
			cfg_data   <= 32'h0002_0001;                // C2 -1 tap (down)
			ph_applied <= ph_applied - 1'd1;
		end
		cfg_write <= 1;
	end
end

// RTC 32.768 kHz crystal: CLK_AUDIO (24.576 MHz) / 750 = exactly 32,768 Hz
reg [9:0] rtc_div = 10'd0;
reg       rtc_32k = 1'b0;
always @(posedge CLK_AUDIO) begin
	if (rtc_div == 10'd374) begin
		rtc_div <= 10'd0;
		rtc_32k <= ~rtc_32k;
	end
	else rtc_div <= rtc_div + 10'd1;
end

///////////////////////   RESETS   ///////////////////////////////
// ioctl_download deliberately NOT in rst_hard (H7b.4): the core's own
// sequencer holds the CPU during a download (dl_hold), while the pump
// loader - which needs its init state - accepts the stream.

wire rst_hard = RESET | ~pll_locked;
wire rst_soft = status[0] | buttons[1];

///////////////////////   INPUTS   ///////////////////////////////
// MiSTer joystick bits: [3:0] = up/down/left/right? no - [0]=right, [1]=left,
// [2]=down, [3]=up; J1 buttons from bit 4 in CONF_STR order:
//   [4]=B1 [5]=B2 [6]=B3 [7]=B4 [8]=Start [9]=Coin [10]=Test [11]=Service
// The core boundary is PCB truth = ACTIVE LOW (MAME cv1k.cpp PORT_C/D/F/L).

wire [7:0] p1 = {joystick_0[7:4],                       // b4 b3 b2 b1
                 joystick_0[0], joystick_0[1],          // right left
                 joystick_0[2], joystick_0[3]};         // down  up
wire [7:0] p2 = {joystick_1[7:4],
                 joystick_1[0], joystick_1[1],
                 joystick_1[2], joystick_1[3]};
wire [5:0] sys = {joystick_1[8],                        // start2
                  joystick_0[8],                        // start1
                  joystick_1[9],                        // coin2
                  joystick_0[9],                        // coin1
                  joystick_0[10] | joystick_1[10],      // test (JAMMA edge)
                  joystick_0[11] | joystick_1[11]};     // service coin

///////////////////////   CORE   /////////////////////////////////

wire [15:0] px;
wire        px_de, vsync, hline;
wire        ce_pixel, vga_hs, vga_vs, vga_de;
wire [7:0]  vga_r, vga_g, vga_b;
wire [15:0] snd_l, snd_r;
wire        init_done;

wire [12:0] sdram_a;
wire  [1:0] sdram_ba, sdram_dqm;
wire        sdram_ncs, sdram_nras, sdram_ncas, sdram_nwe, sdram_cke;
wire [15:0] sdram_dq_o;
wire        sdram_dq_oe;

ikacore_CV1k core
(
	.i_EMU_CLK102M   (clk_102m4),
	.i_EMU_CLK153M   (clk_153m6),
	.i_EXTAL2        (rtc_32k),

	.i_EMU_INITRST_n (~rst_hard),
	.i_EMU_SOFTRST_n (~rst_soft),

	.i_SYS_n         (~sys),
	.i_P1_n          (~p1),
	.i_P2_n          (~p2),
	.i_S3_TEST_n     (1'b1),           // PCB push button (no MiSTer equivalent)
	.i_DSW_S2        (status[7:4]),

	.o_PX            (px),
	.o_PX_DE         (px_de),
	.o_VSYNC         (vsync),
	.o_HLINE         (hline),
	.o_CE_PIXEL      (ce_pixel),
	.o_VGA_R         (vga_r),
	.o_VGA_G         (vga_g),
	.o_VGA_B         (vga_b),
	.o_VGA_HS        (vga_hs),
	.o_VGA_VS        (vga_vs),
	.o_VGA_DE        (vga_de),

	.o_SND_L         (snd_l),
	.o_SND_R         (snd_r),

	.i_IOCTL_DOWNLOAD(ioctl_download),
	.i_IOCTL_WR      (ioctl_wr),
	.i_IOCTL_ADDR    (ioctl_addr),
	.i_IOCTL_DATA    (ioctl_dout),
	.i_IOCTL_INDEX   (ioctl_index),
	.o_IOCTL_WAIT    (ioctl_wait),

	.o_SDRAM_A       (sdram_a),
	.o_SDRAM_BA      (sdram_ba),
	.o_SDRAM_nCS     (sdram_ncs),
	.o_SDRAM_nRAS    (sdram_nras),
	.o_SDRAM_nCAS    (sdram_ncas),
	.o_SDRAM_nWE     (sdram_nwe),
	.o_SDRAM_DQM     (sdram_dqm),
	.o_SDRAM_CKE     (sdram_cke),
	.o_SDRAM_DQ_O    (sdram_dq_o),
	.o_SDRAM_DQ_OE   (sdram_dq_oe),
	.i_SDRAM_DQ_I    (SDRAM_DQ),

	.o_DDRAM_CLK     (DDRAM_CLK),
	.i_DDRAM_BUSY    (DDRAM_BUSY),
	.o_DDRAM_BURSTCNT(DDRAM_BURSTCNT),
	.o_DDRAM_ADDR    (DDRAM_ADDR),
	.i_DDRAM_DOUT    (DDRAM_DOUT),
	.i_DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.o_DDRAM_RD      (DDRAM_RD),
	.o_DDRAM_DIN     (DDRAM_DIN),
	.o_DDRAM_BE      (DDRAM_BE),
	.o_DDRAM_WE      (DDRAM_WE),

	.o_INIT_DONE     (init_done)
);

// SDRAM module pad tristate (split-DQ recipe, owned here)
assign SDRAM_DQ   = sdram_dq_oe ? sdram_dq_o : 16'hZZZZ;
assign SDRAM_A    = sdram_a;
assign SDRAM_BA   = sdram_ba;
assign SDRAM_nCS  = sdram_ncs;
assign SDRAM_nRAS = sdram_nras;
assign SDRAM_nCAS = sdram_ncas;
assign SDRAM_nWE  = sdram_nwe;
assign SDRAM_DQML = sdram_dqm[0];
assign SDRAM_DQMH = sdram_dqm[1];
assign SDRAM_CKE  = sdram_cke;

///////////////////////   VIDEO   ////////////////////////////////
// H7b.6: the core's MiSTer face goes straight out - 5->8 expanded RGB,
// porch-split syncs and the 6.4 MHz dot CE, all in the 153.6 MHz blit
// domain (= CLK_VIDEO).  320x240 @ 60.0184 Hz (853,072 CKIO exact);
// porch widths provisional until M-1 (blit_video parameters).

assign CLK_VIDEO = clk_153m6;
assign CE_PIXEL  = ce_pixel;
assign VGA_R     = vga_r;
assign VGA_G     = vga_g;
assign VGA_B     = vga_b;
assign VGA_HS    = vga_hs;
assign VGA_VS    = vga_vs;
assign VGA_DE    = vga_de;

wire _unused_video = &{1'b0, px, px_de, hline, vsync, forced_scandoubler,
                       HDMI_WIDTH, HDMI_HEIGHT, 1'b0};

///////////////////////   AUDIO   ////////////////////////////////

assign AUDIO_L = snd_l;
assign AUDIO_R = snd_r;

///////////////////////   STATUS   ///////////////////////////////

assign LED_USER = ioctl_download | ~init_done;

wire _unused_misc = &{1'b0, status[3:1], status[8], status[127:14], OSD_STATUS,
                      UART_CTS, UART_RXD, UART_DSR, USER_IN,
                      SD_MISO, SD_CD, 1'b0};

endmodule
