`default_nettype none
//============================================================================
// ikacore_CV1k.sv - Cave CV1000-B portable core top  [H7b.1]
//
// The PLATFORM-AGNOSTIC game top (the `Psychic5_emu.v` role of the two-file
// split, plan of record 2026-07-16): everything CV1000 lives below this
// boundary; everything MiSTer (framework ports, hps_io, PLL, CONF_STR, OSD)
// lives in the wrapper `ikacore_CV1k_emu.sv` (module emu), which is the only
// file that touches `srcs/`.  Simulation drives THIS module directly
// (tb_cv1k today, ikacore_CV1k_tb at H7b.3) - no sim-only ports exist.
//
// Board-level netlist: the SH7709S (HS3 core, ip_cores/HS3) wired to its two
// shared-bus memories as on the CV1000-B PCB.  Two build arms:
//
//   default (vendor datum): the real NDA device models patched for Verilator
//     - U4 program NOR flash  - Macronix MX29LV320E (area 0 / CS0, word mode)
//     - U1 work-RAM SDRAM     - Micron  MT48LC2M32B2 (area 3 / CS3, 8 MB)
//     kept alive as the frozen regression reference (decision 2026-07-16).
//   +define+MISTER_SDRAM: CV1k_sdram_control (double-pump at 2xCKIO +
//     SDRAM-served NOR window) replaces BOTH; the 128 MB module chip model
//     (models/mister_128mb.sv) attaches OUTSIDE on the o_SDRAM_* pins.
//
// Both device arms ride the SAME physical address/data bus (SH7709S HW manual
// Table 10.3) selected by CSn.  The data bus is a true bidirectional net with
// tristate resolution; vendor models take explicit *_in write-data inputs
// (Dq_in / Q_in) fed from the SH-3 o_D_O, per the model patch recipe.  The
// module's own DQ pins are exported SPLIT (o_SDRAM_DQ_O/OE + i_SDRAM_DQ_I) -
// the wrapper/TB owns the pad tristate, same recipe one level up.
//
// Reset policy (H7b.1, MiSTer compliance):
//   i_EMU_INITRST_n  hard reset (RESET | ~pll_locked | ioctl download start):
//                    memory subsystem re-inits, then the sequencer below
//                    releases the CPU (POR, then RESETM 8 clocks later - the
//                    exact ordering tb_cv1k used to fake with # delays).
//                    MISTER arm: CPU held until pump JEDEC init done and no
//                    ioctl download in flight.
//   i_EMU_SOFTRST_n  soft reset (OSD): CPU + blitter reboot only; pump init
//                    state and SDRAM/DDR3 contents are preserved.
//
// ---------------------------------------------------------------------------
// Verified address map (SH7709S HW manual Table 10.3 / MAME cv1k.cpp):
//   area 0  CS0  0x00000000-0x03FFFFFF  U4 NOR flash            boot @ 0
//   area 3  CS3  0x0C000000-0x0FFFFFFF  U1 SDRAM work RAM (8 MB)
//   reset PC = 0xA0000000 (P2 uncached) -> phys 0x00000000 -> NOR offset 0
//   area 0 bus width = 16 bit  => MD4=1, MD3=0  (Table 10.4)
//   SDRAM AMX 0111 (MT48LC2M32B2), the only mode the HS3 BSC decodes:
//     Addr[10:0] = o_A[12:2],  BA[1:0] = o_A[14:13],  DQM[3:0] = o_WE_n[3:0]
//     Cs=CS3_n, Ras=RAS3L_n, Cas=CASL_n, We=RD_WR, Clk=CKIO, Cke=CKE
//
// JAMMA / cabinet inputs (MAME cv1k.cpp port map, all ACTIVE LOW at this
// boundary = PCB pin truth; the wrapper inverts MiSTer's active-high joys):
//   PORT_C (SH-3 port C) = i_SYS_n : bit0 service coin, bit1 test (JAMMA
//     edge), bit2 coin1, bit3 coin2, bit4 start1, bit5 start2
//   PORT_D (port D) = i_P1_n : bit0 up, 1 down, 2 left, 3 right, 4-7 btn1-4
//   PORT_L (port L) = i_P2_n : same layout, player 2
//   PORT_F bit1     = i_S3_TEST_n : S3 test push button on the PCB
//   DSW (blitter regfile 0x50) = i_DSW_S2
//
// NOTE on the flash part: the physical CV1000-B U4 is a 2 MB MX29LV160D; the
// vendor arm uses its 4 MB Macronix sibling MX29LV320E (the proven-patched
// model) with the 2 MB image mirrored to 4 MB - identical to MAME's
// ROM_RELOAD - which the area-0 (ordinary-memory) controller treats
// identically.
//============================================================================
module ikacore_CV1k #(
    parameter ROM_FILE = "roms/ibara_patched/ibara_u4_4M.hex"  // vendor arm only
) (
    //------------------------------------------------------------------
    // clocks (both from ONE fractional PLL VCO on target - related clocks)
    //------------------------------------------------------------------
    input  wire         i_EMU_CLK102M,   // CPU/board/SDRAM domain (102.4 MHz = 2xCKIO)
    input  wire         i_EMU_CLK153M,   // blit/DDR3 domain (153.6 MHz; consumed at H7b.2)
    input  wire         i_EXTAL2,        // RTC 32.768 kHz (wrapper: CLK_AUDIO / 750)

    //------------------------------------------------------------------
    // resets (see policy above)
    //------------------------------------------------------------------
    input  wire         i_EMU_INITRST_n, // hard: full chain incl. memory subsystem
    input  wire         i_EMU_SOFTRST_n, // soft: CPU/blitter reboot, contents kept

    //------------------------------------------------------------------
    // JAMMA / cabinet inputs (active low) + DIP
    //------------------------------------------------------------------
    input  wire [5:0]   i_SYS_n,         // {start2,start1,coin2,coin1,test,service}
    input  wire [7:0]   i_P1_n,          // {b4,b3,b2,b1,right,left,down,up}
    input  wire [7:0]   i_P2_n,
    input  wire         i_S3_TEST_n,     // PCB S3 push button (PORT_F bit1)
    input  wire [3:0]   i_DSW_S2,        // DIP S2 (blitter regfile 0x50)

    //------------------------------------------------------------------
    // video taps (raw blit_video stream - the sim-accept datum) + the
    // H7b.6 MiSTer face (5->8 expanded RGB, porch-split syncs, 6.4 MHz
    // CE; everything in the 153.6 MHz blit domain = CLK_VIDEO)
    //------------------------------------------------------------------
    output wire [15:0]  o_PX,            // ARGB1555 pixel stream
    output wire         o_PX_DE,
    output wire         o_VSYNC,         // 1-cycle IRQ2-source pulse (not VGA_VS)
    output wire         o_HLINE,
    output wire         o_CE_PIXEL,      // dot CE, one blit-clk cycle per dot
    output wire [7:0]   o_VGA_R,
    output wire [7:0]   o_VGA_G,
    output wire [7:0]   o_VGA_B,
    output wire         o_VGA_HS,
    output wire         o_VGA_VS,
    output wire         o_VGA_DE,        // = o_PX_DE (exact ~(hb|vb) at CE)

    //------------------------------------------------------------------
    // sound (YMZ770 - later phase; tied silent)
    //------------------------------------------------------------------
    output wire [15:0]  o_SND_L,
    output wire [15:0]  o_SND_R,

    //------------------------------------------------------------------
    // HPS ioctl download (CPU is held while i_IOCTL_DOWNLOAD=1)
    //------------------------------------------------------------------
    input  wire         i_IOCTL_DOWNLOAD,
    input  wire         i_IOCTL_WR,
    input  wire [26:0]  i_IOCTL_ADDR,    // NOTE: wraps at 128 MiB - the H7b.4
    input  wire [7:0]   i_IOCTL_DATA,    // decoder keys on its own byte counter
    input  wire [15:0]  i_IOCTL_INDEX,
    output wire         o_IOCTL_WAIT,

    //------------------------------------------------------------------
    // MiSTer SDRAM module pins (MISTER_SDRAM arm; SDRAM_CLK is a PLL
    // output in the wrapper - phase-tunable - not driven from here)
    //------------------------------------------------------------------
    output wire [12:0]  o_SDRAM_A,
    output wire [1:0]   o_SDRAM_BA,
    output wire         o_SDRAM_nCS,
    output wire         o_SDRAM_nRAS,
    output wire         o_SDRAM_nCAS,
    output wire         o_SDRAM_nWE,
    output wire [1:0]   o_SDRAM_DQM,     // connector DQML/H (unrouted on 128MB boards)
    output wire         o_SDRAM_CKE,
    output wire [15:0]  o_SDRAM_DQ_O,    // split DQ: wrapper/TB owns the tristate
    output wire         o_SDRAM_DQ_OE,
    input  wire [15:0]  i_SDRAM_DQ_I,

    //------------------------------------------------------------------
    // MiSTer DDRAM face (H7b.2: the whole blit stack lives behind it -
    // VRAM trains + NAND, arbitrated by the shared CV1k_ddr3_harness;
    // o_DDRAM_CLK = the 153.6 MHz blit domain clock)
    //------------------------------------------------------------------
    output wire         o_DDRAM_CLK,
    input  wire         i_DDRAM_BUSY,
    output wire [7:0]   o_DDRAM_BURSTCNT,
    output wire [28:0]  o_DDRAM_ADDR,
    input  wire [63:0]  i_DDRAM_DOUT,
    input  wire         i_DDRAM_DOUT_READY,
    output wire         o_DDRAM_RD,
    output wire [63:0]  o_DDRAM_DIN,
    output wire [7:0]   o_DDRAM_BE,
    output wire         o_DDRAM_WE,

    //------------------------------------------------------------------
    // status
    //------------------------------------------------------------------
    output wire         o_INIT_DONE      // memory subsystem ready (wrapper LED)
);

//------------------------------------------------------------------
// internal legacy names: the board netlist below predates the H7b.1
// port reshape and is UNCHANGED - it keeps addressing the clock and
// resets by their original names.
//------------------------------------------------------------------
wire i_CLK   = i_EMU_CLK102M;
wire i_CEN   = 1'b1;                         // architectural clock enable (never gated)
wire i_POR_n;                                // CPU power-on reset   (sequencer below)
wire i_RST_n;                                // CPU manual reset     (sequencer below)

// H7b.2: the blit/DDR3 domain clock (blit_top + blit_batch +
// CV1k_ddr3_harness + CV1k_nand live here; enable generation below)
wire blit_clk = i_EMU_CLK153M;

//------------------------------------------------------------------
// H7b.1 reset sequencer (replaces tb_cv1k's ad-hoc `#` ordering, which
// cannot exist on target).  Release order on a hard reset:
//   i_EMU_INITRST_n rises -> memory subsystem runs (pump JEDEC init /
//   vendor-flash Tvcs is the TB's affair) -> cpu_go -> CPU POR released
//   on that edge, RESETM released 8 clocks later (the exact stagger the
//   old TB applied).  A soft reset or an ioctl download drops cpu_go
//   and re-runs only the CPU stagger.
//------------------------------------------------------------------
wire pump_init_done;                         // MISTER arm (tied 1 otherwise)
wire dl_hold;
`ifdef MISTER_SDRAM
// H7b.4: the hold covers the whole download AND the ioctl decoder's
// drain (a packed DDR3 word may still be crossing into the 153.6 domain
// just after ioctl_download falls) - the CPU POR releases only once the
// DDRAM face is definitively back with the harness.
wire ioctl_hold;
assign dl_hold = ioctl_hold;
`else
assign dl_hold = 1'b0;
`endif

// H7b.4 split: the download holds the CPU in MANUAL reset (RESETM), not
// POR.  HS3's CKIO divider (cpg_wdt ckio_ph) is reset by POR ONLY, and
// its phase parity on the 51.2 MHz grid is set by the POR-release edge -
// releasing POR at the download's end (an arbitrary 102.4 edge) put CKIO
// rises on 153.6 FALLING edges (pcen23 misphase, found by H7b.7 cell C).
// So POR releases at the TB/emu-anchored SOFTRST instant (known-good
// parity), CKIO locks its grid phase once, and a download only extends
// the RESETM hold; the blitter/board glue hold on sys_rst_n throughout.
// (On-target note, H7b.8: soft reset still cycles POR - parity there is
// a PLL/sequencer determinism item; and a real seconds-long HPS download
// opens NO refresh windows on the idle grid - see the pump header.)
wire cpu_por_go = i_EMU_INITRST_n & i_EMU_SOFTRST_n & pump_init_done;

// dl_hold release quantizer (H7b.7 cell C determinism): a download ends
// on an arbitrary 102.4 edge, but the RESETM release must land on the
// same CKIO sub-phase as a preload boot's POR release (where ckio_ph=0,
// CKIO high, falling next clock).  Extending the hold to the next
// CKIO_PCEN instant makes cpu_go rise one clock after PCEN = a ph=0
// edge = exactly that alignment.  Inert when dl_hold never rises.
reg dl_hold_q = 1'b0;
always @(posedge i_EMU_CLK102M) begin
    if (dl_hold)        dl_hold_q <= 1'b1;
    else if (CKIO_PCEN) dl_hold_q <= 1'b0;
end
wire cpu_go = cpu_por_go & ~dl_hold & ~dl_hold_q;

reg [3:0] rst_cnt = 4'd0;
always @(posedge i_EMU_CLK102M) begin
    if (!cpu_go)             rst_cnt <= 4'd0;
    else if (rst_cnt != 4'd9) rst_cnt <= rst_cnt + 4'd1;
end
assign i_POR_n = cpu_por_go;                 // combinational: same release edge the TB drove
assign i_RST_n = cpu_go & (rst_cnt >= 4'd8); // 8-clock stagger (old tb repeat(8))

// board glue + blit stack reset: identical to the old i_POR_n in every
// non-download run (dl_hold=0 -> sys_rst_n == i_POR_n, same edge), and
// held through a download so the DDRAM face stays with the ioctl mux
wire sys_rst_n = cpu_go;

assign o_INIT_DONE = pump_init_done;

//------------------------------------------------------------------
// H7b.2 blit-domain CKIO enable (the ÷3 of the two-clock scheme).
//
// 102.4 and 153.6 are RELATED clocks off one PLL VCO with coincident
// rising edges on the 51.2 MHz CKIO grid.  The blit domain regenerates
// its own "CKIO rises at the end of this cycle" enable by sampling the
// CPU domain's registered CKIO_PCEN (= HS3's ckio_ph flop, which is
// high for exactly the one CKIO-half-period ending at each rise):
//
//   blit edges per grid period:   E-13.3ns   E-6.7ns    E (coincident)
//   CKIO_PCEN sampled (pre-edge):    0          1       (1, falls AT E)
//   p3_q & ~p3_qq             :               ----- high -----> update @E
//
// so blit_pcen3-qualified registers update at EXACTLY the same $time
// instants as the CPU domain's PCEN-qualified ones - including the very
// first CKIO rise after POR (PCEN's first half-period gives the one
// observable sample needed), which keeps the video/IRQ2 phase identical
// to the single-clock datum.  Silicon note (H7b.8 SDC): the sample path
// is a 102.4->153.6 mid-grid launch with a 2/12-grid (3.26 ns) setup
// window - one enable bit, constrain explicitly.
//------------------------------------------------------------------
reg p3_q, p3_qq;
always @(posedge blit_clk or negedge i_EMU_INITRST_n) begin
    if (!i_EMU_INITRST_n) begin
        p3_q  <= 1'b0;
        p3_qq <= 1'b0;
    end
    else begin
        p3_q  <= CKIO_PCEN;
        p3_qq <= p3_q;
    end
end
wire blit_pcen3 = p3_q & ~p3_qq;

`ifndef SYNTHESIS
// H7b.2 accept: PCEN2/PCEN3 same-instant assertion.  Each side records
// the $realtime of its last qualified update edge; when the blit side
// fires, the PREVIOUS pair (visible race-free through the NBA old-value
// read) must be equal.  Any grid misphase between the TB's two clocks
// or a mislocked enable trips this on the second fire.
real p23_t2 = -1.0, p23_t3 = -1.0;
integer p23_err = 0, p23_n = 0;
always @(posedge i_CLK) if (CKIO_PCEN) p23_t2 <= $realtime;
always @(posedge blit_clk) if (blit_pcen3) begin
    if (p23_t3 >= 0.0 && p23_t3 != p23_t2) begin
        p23_err <= p23_err + 1;
        if (p23_err < 5)
            $display("[pcen23] MISPHASE: last PCEN2 @%0t vs PCEN3 @%0t (now %0t)",
                     p23_t2, p23_t3, $realtime);
    end
    p23_t3 <= $realtime;
    p23_n  <= p23_n + 1;
end
final begin
    if (p23_n > 0)
        $display("[pcen23] %0d PCEN3 grid edges, %0d misphase errors%s",
                 p23_n, p23_err, (p23_err == 0) ? " - PASS" : " - FAIL");
end
`endif

//------------------------------------------------------------------
// Shared external bus nets (SH7709S BSC pins)
//------------------------------------------------------------------
wire [25:0] A;
wire [31:0] D_O;
wire        D_OE;
wire [31:0] D_I;
wire        BS_n, CS0_n, CS2_n, CS3_n, CS4_n, CS5_n, CS6_n;
wire        RD_WR, RAS3L_n, RAS3U_n, CASL_n, CASU_n, RD_n;
wire [3:0]  WE_n;
wire        CKE, CKIO, BACK_n, BUS_OE;
wire        CKIO_PCEN;                       // i_CLK cycle in which CKIO rises

// U13 CPLD + U2 NAND nets
wire [3:0]  cpld_D;                          // CPLD drive value for D[3:0]
wire        cpld_D_OE;                       // CPLD drives (EEPROM/RTC reads)
wire        u2_ce_n, u2_re_n, u2_we_n;       // U13 -> U2 NAND control strobes
wire        nand_rb_n;                       // U2 ready/busy -> PTE5 (NAND ready)

// blitter (CS6 + IRQ) nets
wire        blit_n;                          // U13 o_BLITTER_n (mirrors CS6_n)
wire [31:0] blit_D;                          // blitter read data onto the shared bus
wire        blit_D_OE;                       // blitter drives the bus (CS6 read)
wire        pth_irq2_n;                      // vblank IRQ2 on PTH[2] (real vsync, H5)
wire        pth_irq1_n;                      // blitter-done IRQ1 on PTH[1] (governed, H4)

// blitter bus-mastering nets (H2: BREQ/BACK tenure, fig 10.41)
wire        blit_breq_n, blit_own;
wire        blit_ref_win;                    // blit_fetch refresh-window sideband
wire [25:0] bf_A;
wire        bf_CS_n, bf_RAS_n, bf_CAS_n, bf_WE;
wire [3:0]  bf_DQM;

// blitter VRAM beat channels (blit_top <-> blit_batch, H7b.2)
wire        bv_srd_req, bv_drd_req, bv_wr_req, bv_wr_rdy, bv_rd_vld;
wire [24:0] bv_srd_addr, bv_drd_addr, bv_wr_addr;
wire [63:0] bv_srd_data, bv_drd_data, bv_wr_data;
wire [3:0]  bv_wr_mask;
wire        blit_steal;                      // scanout-owns-memory tap (debug)

// H7 descriptor sideband (blit_top -> blit_batch train formation)
wire        dsc_vld, dsc_flipy, dsc_blend, dsc_strict, dsc_px1, dsc_wait;
wire [12:0] dsc_sx_lo, dsc_rows;
wire [11:0] dsc_sy0;
wire [13:0] dsc_npx;
wire [31:0] dsc_dst0;

// blit_batch <-> harness train port + blit_video line-train client
wire        prd_req, prd_rdy, prd_dvld;
wire [22:0] prd_addr;
wire [10:0] prd_len;
wire [63:0] prd_data;
wire        pwr_req, pwr_rdy;
wire [22:0] pwr_addr;
wire [63:0] pwr_data;
wire [3:0]  pwr_be;
wire        rd_train, wr_train;
wire        lf_req, lf_dvld;
wire [11:0] lf_y;
wire [12:0] lf_x0;
wire [63:0] lf_data;

// the true bidirectional board data bus (shared A/D bus, Table 10.1)
wire [31:0] D;
assign D   = D_OE ? D_O : 32'hzzzz_zzzz;    // SH-3 drives on writes
assign D_I = D;                             // SH-3 samples the resolved bus
assign D[3:0] = cpld_D_OE ? cpld_D : 4'hz;  // U13 drives the low nibble on EEPROM reads
assign D   = blit_D_OE ? blit_D : 32'hzzzz_zzzz;  // blitter drives on CS6 reads

// area-0 bus width strap = 16 bit (Table 10.4: MD4=1, MD3=0)
localparam MD4 = 1'b1, MD3 = 1'b0;

// sound: YMZ770C-F is a later phase (DDR3 map + loader slots reserved)
assign o_SND_L = 16'h0000;
assign o_SND_R = 16'h0000;

//==================================================================
//  SH7709S  (HS3 subset core, read-only IP)
//==================================================================
HS3 #(
    .RESET_PC   (32'hA000_0000),
    .BIG_ENDIAN (1'b1)
) u_hs3 (
    .i_POR_n (i_POR_n), .i_RST_n (i_RST_n),
    .i_CLK   (i_CLK),   .i_CEN   (i_CEN),
    .o_CKIO  (CKIO),    .i_EXTAL2(i_EXTAL2),
    .o_CKIO_PCEN(CKIO_PCEN), .o_CKIO_NCEN(),    // single-clock enables for board glue

    // shared external bus
    .o_A(A), .o_D_O(D_O), .o_D_OE(D_OE), .i_D_I(D_I),
    .o_BS_n(BS_n),
    .o_CS0_n(CS0_n), .o_CS2_n(CS2_n), .o_CS3_n(CS3_n),
    .o_CS4_n(CS4_n), .o_CS5_n(CS5_n), .o_CS6_n(CS6_n),
    .o_RD_WR(RD_WR),
    .o_RAS3L_n(RAS3L_n), .o_RAS3U_n(RAS3U_n),
    .o_CASL_n(CASL_n),   .o_CASU_n(CASU_n),
    .o_WE_n(WE_n), .o_RD_n(RD_n),
    .i_WAIT_n(1'b1),                       // no external wait insertion
    .i_MD4(MD4), .i_MD3(MD3),
    .o_CKE(CKE),
    .i_BREQ_n(blit_breq_n), .o_BACK_n(BACK_n), .o_BUS_OE(BUS_OE),

    // generic memory port - observation only here (physical pins answer)
    .o_MEM_REQ(), .o_MEM_WRITE(), .o_MEM_BURST(), .o_MEM_SIZE(),
    .o_MEM_ADDR(), .o_MEM_CS_n(), .o_MEM_WSTRB(),
    // NOTE: i_MEM_READY is ORed into the BSC generic-completion term
    // (gen_ext_done = i_MEM_RSP_VALID | i_MEM_READY, bsc.sv l.334). It must
    // be 0 so generic (area-0 NOR) reads complete on the physical wait-state
    // countdown and latch stable i_D_I - tying it 1 finishes every read in
    // one cycle and samples the bus before the flash has driven.
    .i_MEM_READY(1'b0), .i_MEM_RSP_VALID(1'b0), .i_MEM_FAULT(1'b0),
    .o_MEM_RSP_READY(),

    .i_NMI(1'b1),                          // NMI idle high (edge triggered)

    // I/O port pads - JAMMA inputs per the MAME cv1k.cpp map (header),
    // everything else tied to benign idle, outputs open
    .i_PTA_I(8'hFF), .o_PTA_O(), .o_PTA_OE(), .o_PTA_PU(),
    .i_PTB_I(8'hFF), .o_PTB_O(), .o_PTB_OE(), .o_PTB_PU(),
    .i_PTC_I({2'b11, i_SYS_n}),                             // PORT_C: system inputs
    .o_PTC_O(), .o_PTC_OE(), .o_PTC_PU(),
    .i_PTD_I(i_P1_n), .o_PTD_O(), .o_PTD_OE(), .o_PTD_PU(), // PORT_D: player 1
    .i_PTE_I({2'b11, nand_rb_n, 5'b11111}),                 // bit5 = U2 NAND ready/busy
    .o_PTE_O(), .o_PTE_OE(), .o_PTE_PU(),
    .i_PTF_I({6'b111111, i_S3_TEST_n, 1'b1}),               // PORT_F bit1: S3 test button
    .o_PTF_PU(),
    .i_PTG_I(8'hFF), .o_PTG_PU(),
    .i_PTH_I({5'b11111, pth_irq2_n | ~irq2_en, pth_irq1_n, 1'b1}),   // PTH[2]=IRQ2 vblank (H0), PTH[1]=IRQ1 blit done (H3 provisional)
    .o_PTH_O(), .o_PTH_OE(), .o_PTH_PU(),
    .i_PTJ_I(8'hFF), .o_PTJ_O(), .o_PTJ_OE(), .o_PTJ_PU(),
    .i_PTK_I(8'hFF), .o_PTK_O(), .o_PTK_OE(), .o_PTK_PU(),
    .i_PTL_I(i_P2_n),                                       // PORT_L: player 2
    .i_SCPT_I(8'hFF), .o_SCPT_O(), .o_SCPT_OE(), .o_SCPT_PU()
);

`ifndef MISTER_SDRAM
//==================================================================
//  Vendor-datum arm (frozen regression reference, decision 2026-07-16)
//==================================================================
//  U4 - program NOR flash (Macronix MX29LV320E, word mode)
//       area 0 / CS0, 16-bit port on D[15:0]
//==================================================================
MX29LV320E #(
    .Init_File(ROM_FILE)
) u_u4_nor (
    .A       (A[21:1]),               // word address (device A0 = CPU A1)
    .Q       (D[15:0]),               // read data onto shared bus
    .Q_in    (D_O[15:0]),             // write data view (patch: no inout readback)
    .CE_B    (CS0_n),
    .WE_B    (WE_n[1] & WE_n[0]),      // word write strobe (D15-D0 lanes)
    .OE_B    (RD_n),
    .BYTE_B  (1'b1),                  // word mode (x16)
    .RESET_B (i_POR_n),
    .RYBY_B  (),
    .WP_B    (1'b1)
);

//==================================================================
//  U1 - work-RAM SDRAM (Micron MT48LC2M32B2, 8 MB)
//       area 3 / CS3, 32-bit
//==================================================================
//  While the blitter fetch unit holds the bus grant (BACK_n low, SH-3 pins
//  released per fig 10.41), it drives U1's command/address pins in place of
//  the CPU - the sim equivalent of the PCB's tristate handover.
mt48lc2m32b2 u_u1_sdram (
    .Dq   (D[31:0]),                  // read data onto shared bus
    .Dq_in(D_O),                      // write data view (patch: no inout readback)
    .Addr (blit_own ? bf_A[12:2]  : A[12:2]),
    .Ba   (blit_own ? bf_A[14:13] : A[14:13]),
    .Clk  (CKIO),
    .Cke  (CKE),
    .Cs_n (blit_own ? bf_CS_n     : CS3_n),
    .Ras_n(blit_own ? bf_RAS_n    : RAS3L_n),
    .Cas_n(blit_own ? bf_CAS_n    : CASL_n),
    .We_n (blit_own ? bf_WE       : RD_WR),
    .Dqm  (blit_own ? bf_DQM      : WE_n)
);

// no memory subsystem to wait for; MiSTer-facing pins parked inactive
assign pump_init_done = 1'b1;
assign o_IOCTL_WAIT   = 1'b0;
assign o_SDRAM_A      = 13'd0;
assign o_SDRAM_BA     = 2'd0;
assign o_SDRAM_nCS    = 1'b1;
assign o_SDRAM_nRAS   = 1'b1;
assign o_SDRAM_nCAS   = 1'b1;
assign o_SDRAM_nWE    = 1'b1;
assign o_SDRAM_DQM    = 2'b11;
assign o_SDRAM_CKE    = 1'b0;
assign o_SDRAM_DQ_O   = 16'h0000;
assign o_SDRAM_DQ_OE  = 1'b0;
wire _unused_vendor = &{1'b0, i_IOCTL_DOWNLOAD, i_IOCTL_WR, i_IOCTL_ADDR,
                        i_IOCTL_DATA, i_IOCTL_INDEX, i_SDRAM_DQ_I, 1'b0};

`else  // MISTER_SDRAM
//==================================================================
//  MiSTer variant: CV1k_sdram_control + 128 MB dual-chip SDRAM module replace
//  BOTH U4 (NOR served from the SDRAM window, rows 0x1000-0x17FF of
//  chip0 bank0) and U1 (double-pumped at 2xCKIO).  See CV1k_sdram_control.sv and
//  docs/double_pump_sdram.md.  H7b.1: the chip model (models/mister_128mb.sv)
//  moved OUT to the TB/wrapper on the o_SDRAM_* pins; the pad tristate is
//  owned there (split-DQ recipe).
//==================================================================
wire [31:0] pump_g_rdata;
wire        pump_g_oe;
wire [15:0] pump_n_rdata;
wire        pump_n_oe;
wire        pump_ioctl_wait;

// H7b.4 ioctl decoder nets (decoder <-> pump / DDRAM mux)
wire        ic_nor_download, ic_nor_wr;
wire [26:0] ic_nor_addr;
wire [7:0]  ic_nor_data;
wire        ic_ddr_own, ic_ddr_we;
wire [28:0] ic_ddr_addr;
wire [63:0] ic_ddr_din;
wire [7:0]  ic_ddr_be;

assign D        = pump_g_oe ? pump_g_rdata : 32'hzzzz_zzzz;  // CS3 read beats
assign D[15:0]  = pump_n_oe ? pump_n_rdata : 16'hzzzz;       // CS0 (NOR) reads

CV1k_sdram_control u_pump (
    .i_CLK        (i_CLK),
    .i_RST_n      (i_EMU_INITRST_n),         // memory subsystem resets on HARD only
    .i_CKIO_PCEN  (CKIO_PCEN),
    // grid: identical mux expressions the U1 model saw (pin-true handover);
    // row bit 11 is tied 0 on this B board (the CV1000-D top feeds A[13:2]
    // of its 12-bit-row part here instead)
    .i_G_A        (blit_own ? {1'b0, bf_A[12:2]} : {1'b0, A[12:2]}),
    .i_G_BA       (blit_own ? bf_A[14:13] : A[14:13]),
    .i_G_CS_n     (blit_own ? bf_CS_n     : CS3_n),
    .i_G_RAS_n    (blit_own ? bf_RAS_n    : RAS3L_n),
    .i_G_CAS_n    (blit_own ? bf_CAS_n    : CASL_n),
    .i_G_WE_n     (blit_own ? bf_WE       : RD_WR),
    .i_G_DQM      (blit_own ? bf_DQM      : WE_n),
    .i_G_CKE      (CKE),
    .i_G_WDATA    (D_O),
    .o_G_RDATA    (pump_g_rdata),
    .o_G_RDATA_OE (pump_g_oe),
    // NOR: same pins the MX29LV320E saw
    // CS0 qualified by the pad enable: with the bus granted away the SH-3's
    // strobe outputs are meaningless (hi-Z + pull-up on the PCB - the old
    // flash model only ever saw harmless phantom reads, but here a phantom
    // NOR request becomes SDRAM commands colliding with the blitter's open
    // row: "Bank already activated", found by the ddpsdoj +blitreplay census)
    .i_N_CS_n     (CS0_n | ~BUS_OE),
    .i_N_RD_n     (RD_n),
    .i_N_A        (A[21:1]),
    .i_N_WR_n     (WE_n[1] & WE_n[0]),
    .o_N_RDATA    (pump_n_rdata),
    .o_N_RDATA_OE (pump_n_oe),
    // refresh-scheduler windows (doc section 6.2): blit tenures via the
    // fetch sideband, ordinary CS4/5/6 cycles via the strobes.  Same
    // BUS_OE phantom-strobe qualification as CS0 above.
    .i_BACK_n     (BACK_n),
    .i_BLIT_WIN   (blit_ref_win),
    .i_ORD_CS_n   ((CS4_n & CS5_n & CS6_n) | ~BUS_OE),
    // HPS ioctl: since H7b.4 the pump sees only the u4 sub-stream of the
    // MRA layout, rebased to 0 by the CV1k_ioctl decoder below (the pump's
    // own <4 MiB window check + {odd,even} halfword assembly unchanged)
    .i_IOCTL_DOWNLOAD (ic_nor_download),
    .i_IOCTL_WR       (ic_nor_wr),
    .i_IOCTL_ADDR     (ic_nor_addr),
    .i_IOCTL_DATA     (ic_nor_data),
    .i_IOCTL_INDEX    (16'h0000),
    .o_IOCTL_WAIT     (pump_ioctl_wait),
    // status
    .o_INIT_DONE  (pump_init_done),
    // module pins (pad tristate + SDRAM_CLK live in the wrapper/TB)
    .o_S_A(o_SDRAM_A), .o_S_BA(o_SDRAM_BA), .o_S_nCS(o_SDRAM_nCS),
    .o_S_nRAS(o_SDRAM_nRAS), .o_S_nCAS(o_SDRAM_nCAS), .o_S_nWE(o_SDRAM_nWE),
    .o_S_DQM(o_SDRAM_DQM), .o_S_CKE(o_SDRAM_CKE),
    .o_S_DQ_O(o_SDRAM_DQ_O), .o_S_DQ_OE(o_SDRAM_DQ_OE), .i_S_DQ_I(i_SDRAM_DQ_I)
);

//==================================================================
//  H7b.4 - ioctl download decoder: splits the one MRA stream on its own
//  byte counter (ioctl_addr wraps at 128 MiB) into the pump's NOR window
//  (u4 x2, rebased to 0) and DDR3 (NAND u2 + YMZ slots) through the
//  8-byte packer.  The DDRAM face is muxed to the packer while
//  o_DDR_OWN; the core is held in reset the whole time (dl_hold above),
//  so the harness never contends.  See CV1k_ioctl.sv for the layout.
//==================================================================
CV1k_ioctl u_ioctl (
    .i_CLK          (i_CLK),
    .i_RST_n        (i_EMU_INITRST_n),
    .i_DOWNLOAD     (i_IOCTL_DOWNLOAD),
    .i_WR           (i_IOCTL_WR),
    .i_ADDR         (i_IOCTL_ADDR),
    .i_DATA         (i_IOCTL_DATA),
    .i_INDEX        (i_IOCTL_INDEX),
    .o_WAIT         (o_IOCTL_WAIT),
    .o_HOLD         (ioctl_hold),
    .o_NOR_DOWNLOAD (ic_nor_download),
    .o_NOR_WR       (ic_nor_wr),
    .o_NOR_ADDR     (ic_nor_addr),
    .o_NOR_DATA     (ic_nor_data),
    .i_NOR_WAIT     (pump_ioctl_wait),
    .i_CLK_DDR      (blit_clk),
    .o_DDR_OWN      (ic_ddr_own),
    .o_DDR_WE       (ic_ddr_we),
    .o_DDR_ADDR     (ic_ddr_addr),
    .o_DDR_DIN      (ic_ddr_din),
    .o_DDR_BE       (ic_ddr_be),
    .i_DDR_BUSY     (i_DDRAM_BUSY)
);
`endif // MISTER_SDRAM

//==================================================================
//  U13 - EPM7032 address-decoder CPLD
//        Decodes the CS4 window [0x10000000,0x14000000) by {A23,A22}
//        into U2 NAND / YMZ770 / RTC-9701, and passes CS6 to the
//        blitter. Runs in the single i_CLK domain via CKIO_PCEN (no
//        derived clocks). Only the U2 NAND path is wired downstream
//        here; audio + EEPROM/RTC strobes are carried but left open.
//==================================================================
CV1k_cpld u_u13_cpld (
    .i_CLK        (i_CLK),
    .i_CKIO_PCEN  (CKIO_PCEN),
    .i_RST_n      (sys_rst_n),
    .i_CS4_n      (CS4_n),
    .i_CS5_n      (CS5_n),
    .i_CS6_n      (CS6_n),
    .i_RD_n       (RD_n),
    .i_WE_n       (WE_n[0]),           // SH-3 WE0 pin (low byte lane)
    .i_A_HI       ({A[23], A[22]}),    // region select
    .i_A_LO       ({A[1],  A[0]}),     // operation / U2 CLE,ALE
    .i_A2         (A[2]),
    .i_D          (D_O[3:0]),          // SH-3 write-data view of the nibble
    .o_D          (cpld_D),
    .o_D_OE       (cpld_D_OE),
    .o_U2_CE_n    (u2_ce_n),
    .o_U2_RE_n    (u2_re_n),
    .o_U2_WE_n    (u2_we_n),
    .o_AUDIO_CS_n (),                  // YMZ770 not modelled yet
    .o_AUDIO_RESET(),
    .i_EEPROM_DO  (1'b0),              // RTC-9701 not modelled (stub 0)
    .i_EEPROM_TIRQ(1'b0),
    .o_EEPROM_DI  (), .o_EEPROM_CLK(), .o_EEPROM_CE(), .o_EEPROM_FOE(),
    .i_AUDIO_PLAY (1'b1),
    .o_BLITTER_n  (blit_n),            // -> blitter register file (CS6)
    .o_SH3_WAIT   (),
    .o_GLOBAL_CLR (),
    .o_DEVICE_READY()
);

//==================================================================
//  U2 - graphics / asset NAND flash (Samsung K9F1G08U0M). Uses the
//       Micron MT29F1G08 behavioural model, ID-patched to EC/F1 and
//       model patched for Verilator 5 (see models/MT29F1G08ABAFA). x8 on D[7:0];
//       CLE=A0, ALE=A1; CE/RE/WE from U13; R/B -> PTE5.
//
//  Array contents (mutually exclusive, selected by build_sim.sh):
//    +define+NAND_ONDEMAND  BIN_FILE = the raw 128 MB dump, read a page at a
//                           time off disk on first touch. Whole device, no
//                           preprocessing, RAM grows only with pages used.
//    +define+NAND_INIT      INIT_FILE = a $readmemh image (scripts/make_nand_init.py).
//                           Bounded by the model's NUM_ROW; boot slice only.
//==================================================================
`ifdef NO_NAND
assign nand_rb_n = 1'b1;                // +define+NO_NAND: omit U2 (R/B tied ready) - compare boot paths
`elsif CV1K_NAND
//==================================================================
//  MiSTer harness-served NAND (+define+CV1K_NAND): the physical U2
//  chip is replaced by CV1k_nand, which serves the DDR3-resident U2
//  image through the SHARED CV1k_ddr3_harness below (H7b.2: it is the
//  nd client of the one full-stack arbiter; H7a step 5 gave it a
//  private harness instance).  The SH-3/CPLD side is wired
//  bit-identically to the vendor model (CLE=A0, ALE=A1, CE/RE/WE from
//  U13, DQ on D[7:0], R/B -> PTE5); those strobes change only on CKIO
//  protocol edges, so the 153.6 MHz edge-detect samples them exactly
//  as the 102.4 one did.  Image base = the H7b DDR3 byte map: NAND u2
//  at byte 0x3400_0000 = DDRAM word 0x0680_0000.
//==================================================================
wire [7:0]  nd_dq_o;
wire        nd_dq_oe;
assign D[7:0] = nd_dq_oe ? nd_dq_o : 8'hzz;     // U2 drives the low byte on reads

wire        nd_req, nd_rdy, nd_dvld;
wire [28:0] nd_addr;
wire [10:0] nd_len;
wire [63:0] nd_data;

CV1k_nand #(.NAND_BASE_W(29'h0680_0000)) u_u2_nand (
    .i_CLK    (blit_clk),               // H7b.2: harness-client domain
    .i_RST_n  (sys_rst_n),
    .i_Dq     (D_O[7:0]),               // SH-3 write-data view (cmd/addr in)
    .o_Dq     (nd_dq_o),
    .o_Dq_oe  (nd_dq_oe),
    .i_Cle    (A[0]),                   // CLE
    .i_Ale    (A[1]),                   // ALE
    .i_Ce_n   (u2_ce_n),                // CE#  (from U13)
    .i_We_n   (u2_we_n),                // WE#
    .i_Re_n   (u2_re_n),                // RE#
    .i_Wp_n   (1'b1),                   // WP# off
    .o_Rb_n   (nand_rb_n),              // R/B# -> PTE5
    .o_nd_req (nd_req), .o_nd_addr(nd_addr), .o_nd_len(nd_len),
    .i_nd_rdy (nd_rdy), .i_nd_dvld(nd_dvld), .i_nd_data(nd_data)
);
`else
`ifdef NAND_ONDEMAND
localparam NAND_BIN_FILE  = "roms/ibara/u2";                        // pristine dump, never written
localparam NAND_INIT_FILE = "";
`else
// $readmemh must not be handed more lines than the model's NUM_ROW array:
//   FullMem      -> all 65536 rows  (correct, but Verilator is very slow on it)
//   sparse (dflt)-> first NAND_ROWS rows (boot slice; see scripts/make_nand_init.py)
localparam NAND_BIN_FILE  = "";
`ifdef FullMem
localparam NAND_INIT_FILE = "roms/ibara_patched/ibara_u2.8.init";
`else
localparam NAND_INIT_FILE = "roms/ibara_patched/ibara_u2_boot.8.init";
`endif
`endif

nand_model #(
    .INIT_FILE(NAND_INIT_FILE),
    .BIN_FILE (NAND_BIN_FILE)
) u_u2_nand (
    .Lock     (1'b0),
    .Dq_Io    (D[7:0]),                // NAND IO on the low byte of the shared bus
    .Dq_Io_in (D_O[7:0]),              // write-data view (SH-3 drives cmd/addr)
    .Cle      (A[0]),                  // command latch enable
    .Ale      (A[1]),                  // address latch enable
    .Clk_We_n (u2_we_n),               // write enable (active low)
    .Wr_Re_n  (u2_re_n),               // read enable  (active low)
    .Ce_n     (u2_ce_n),               // chip enable  (from U13, 0x10C00003 d0)
    .Wp_n     (1'b1),                  // write protect off
    .Rb_n     (nand_rb_n)              // ready/busy -> PTE5
);
`endif

`ifndef CV1K_NAND
// no CV1k_nand in this arm (vendor chip model / NO_NAND): the shared
// harness's NAND client idles
wire        nd_req  = 1'b0;
wire [28:0] nd_addr = 29'd0;
wire [10:0] nd_len  = 11'd0;
wire        nd_rdy, nd_dvld;
wire [63:0] nd_data;
wire _unused_nd = &{1'b0, nd_rdy, nd_dvld, nd_data, 1'b0};
`endif

//==================================================================
//  Blitter core (sim/CV1k_blit/blit_top.sv) [H0-H6 frozen; H7 refactor]
//  regs + fetch + gov + draw + video + IRQ shapers behind one boundary.
//  H7b.2: the FULL DDR3 stack is in-system - blit_top's beat channels
//  feed blit_batch (K=8-objline trains), blit_video prefetches line
//  trains (PREFETCH=1), and ONE shared CV1k_ddr3_harness arbitrates
//  video > batch > NAND onto the MiSTer DDRAM face.  The whole stack
//  runs in the 153.6 MHz blit domain on blit_pcen3 (the tb_h7-proven
//  configuration).  Board glue here: CS6 tristate drive (above) and
//  the U1 command-pin mux during fetch tenures (blit_own, above).
//
//  CDC audit (H7b.2, the two-clock scheme of the plan of record) - every
//  crossing surface changes only on CKIO protocol edges and is consumed
//  at the next grid edge (multicycle in the H7b.8 SDC; the coincident-
//  edge hold captures old data = Verilator same-timestep NBA semantics):
//    CPU -> blit: CS6 strobes/addr/data (blit_regs samples @pcen3),
//      BACK_n + D bus fetch data (blit_fetch @pcen3), CPLD u2 strobes +
//      D_O[7:0] (CV1k_nand edge-detects CKIO-rate strobes), DSW (static).
//    blit -> CPU: BREQ_n/bf_*/blit_own/REF_WIN (blit_fetch, @pcen3),
//      IRQ1_n/IRQ2_n (shapers update @pcen3 instants), CS6 o_D/o_D_OE
//      (comb off CPU strobes + regs; the STATUS busy bit's draw-floor
//      term can move mid-grid - SDC max-delay note, CKIO-visible value
//      unchanged since the governed window >= the datapath), NAND
//      dq/rb_n (protocol gives whole wait-state windows; SDC note).
//==================================================================
reg          irq1_en   = 1'b1;         // +noirq1 A/B knob (sim-only plusarg)
reg          irq2_en   = 1'b1;         // +noirq2: hold vblank IRQ2 off so a
                                       // parked game never wakes (+blitreplay)
`ifndef SYNTHESIS
initial if ($test$plusargs("noirq1")) irq1_en = 1'b0;
initial if ($test$plusargs("noirq2")) irq2_en = 1'b0;
`endif

blit_top #(
    .PREFETCH    (1'b1)                // H7b.2: 1-hline line-train prefetch
) u_blit (
    .i_DSW_S2    (i_DSW_S2),           // DIP S2 (H7b.1: OSD-fed runtime input)
    .i_CLK       (blit_clk),           // H7b.2: 153.6 MHz domain
    .i_CKIO_PCEN (blit_pcen3),
    .i_RST_n     (sys_rst_n),

    // CS6 slave
    .i_BLIT_n    (blit_n),             // U13 o_BLITTER_n (= CS6_n)
    .i_RD_n      (RD_n),
    .i_WE_n      (WE_n),
    .i_RD_WR     (RD_WR),
    .i_A         (A[6:2]),
    .i_D_CPU     (D_O),                // SH-3 write-data view
    .o_D         (blit_D),
    .o_D_OE      (blit_D_OE),

    // BREQ/BACK bus mastering
    .o_BREQ_n    (blit_breq_n),
    .i_BACK_n    (BACK_n),
    .o_bus_drive (blit_own),
    .o_BF_A      (bf_A),
    .o_BF_CS_n   (bf_CS_n),
    .o_BF_RAS_n  (bf_RAS_n),
    .o_BF_CAS_n  (bf_CAS_n),
    .o_BF_WE     (bf_WE),
    .o_BF_DQM    (bf_DQM),
    .i_D_BUS     (D),                  // resolved shared bus (fetch reads)
    .o_REF_WIN   (blit_ref_win),       // CV1k_sdram_control hidden-refresh window

    // interrupts
    .i_IRQ1_EN   (irq1_en),
    .o_IRQ1_n    (pth_irq1_n),
    .o_IRQ2_n    (pth_irq2_n),

    // governor tables: defaults = the P_PDF set (anchors 93/189/12090
    // VCLK); the MiSTer HPS / TB pokes load measured sets later.
    .i_tbl_we    (1'b0),
    .i_tbl_idx   (4'd0),
    .i_tbl_data  (32'd0),

    // VRAM beat channels -> blit_batch (H7b.2)
    .o_srd_req   (bv_srd_req),
    .o_srd_addr  (bv_srd_addr),
    .i_srd_data  (bv_srd_data),
    .o_drd_req   (bv_drd_req),
    .o_drd_addr  (bv_drd_addr),
    .i_drd_data  (bv_drd_data),
    .o_wr_req    (bv_wr_req),          // steal-gated inside blit_top
    .o_wr_addr   (bv_wr_addr),
    .o_wr_data   (bv_wr_data),
    .o_wr_mask   (bv_wr_mask),
    .i_wr_rdy    (bv_wr_rdy),
    .i_rd_vld    (bv_rd_vld),          // batch read-stall protocol
    .o_vrd_req   (),                   // beat-wise video channel idle
    .o_vrd_addr  (),                   //   (PREFETCH=1)
    .i_vrd_data  (64'd0),
    .o_lf_req    (lf_req),             // line-train client -> harness
    .o_lf_y      (lf_y),
    .o_lf_x0     (lf_x0),
    .i_lf_dvld   (lf_dvld),
    .i_lf_data   (lf_data),
    .o_steal     (blit_steal),

    // H7 descriptor sideband -> blit_batch train formation (the TB
    // footprint checker taps the full set hierarchically)
    .o_dsc_vld   (dsc_vld),      .o_dsc_sx_lo (dsc_sx_lo),
    .o_dsc_sx_hi (),             .o_dsc_sy0   (dsc_sy0),
    .o_dsc_rows  (dsc_rows),     .o_dsc_npx   (dsc_npx),
    .o_dsc_dst0  (dsc_dst0),     .o_dsc_flipx (),
    .o_dsc_flipy (dsc_flipy),    .o_dsc_blend (dsc_blend),
    .o_dsc_strict(dsc_strict),   .o_dsc_px1   (dsc_px1),
    .o_dsc_wait  (dsc_wait),     .o_dsc_upl   (), .o_dsc_upl_addr (),
    .o_dsc_upl_dimx (), .o_dsc_upl_dimy (),

    // video timing + pixel stream (H7b.1: exported at the module boundary;
    // H7b.6: + the porch-split sync face and the 6.4 MHz dot CE)
    .o_hline     (o_HLINE),
    .o_vsync     (o_VSYNC),
    .o_px_de     (o_PX_DE),
    .o_px        (o_PX),
    .o_ce_pix    (o_CE_PIXEL),
    .o_hs        (o_VGA_HS),
    .o_vs        (o_VGA_VS)
);

//==================================================================
//  Train batcher (H7a step 3): strictly serial R/W trains exactly as
//  the DES charges, single-buffered pixel-lane staging, the whole
//  i_rd_vld read-stall protocol.  Replaces blit_vram_beh (H3) as the
//  beat-channel backend - the behavioral VRAM now lives only in the
//  H6 unit rigs.
//==================================================================
blit_batch u_batch (
    .i_CLK        (blit_clk),
    .i_RST_n      (sys_rst_n),
    .i_srd_req    (bv_srd_req),
    .i_srd_addr   (bv_srd_addr),
    .o_srd_data   (bv_srd_data),
    .i_drd_req    (bv_drd_req),
    .i_drd_addr   (bv_drd_addr),
    .o_drd_data   (bv_drd_data),
    .o_rd_vld     (bv_rd_vld),
    .i_wr_req     (bv_wr_req),
    .i_wr_addr    (bv_wr_addr),
    .i_wr_data    (bv_wr_data),
    .i_wr_mask    (bv_wr_mask),
    .o_wr_rdy     (bv_wr_rdy),
    .i_dsc_vld    (dsc_vld),
    .i_dsc_sx_lo  (dsc_sx_lo),
    .i_dsc_sy0    (dsc_sy0),
    .i_dsc_rows   (dsc_rows),
    .i_dsc_npx    (dsc_npx),
    .i_dsc_dst0   (dsc_dst0),
    .i_dsc_flipy  (dsc_flipy),
    .i_dsc_blend  (dsc_blend),
    .i_dsc_strict (dsc_strict),
    .i_dsc_px1    (dsc_px1),
    .i_dsc_wait   (dsc_wait),
    .o_prd_req    (prd_req),
    .o_prd_addr   (prd_addr),
    .o_prd_len    (prd_len),
    .i_prd_rdy    (prd_rdy),
    .i_prd_dvld   (prd_dvld),
    .i_prd_data   (prd_data),
    .o_pwr_req    (pwr_req),
    .o_pwr_addr   (pwr_addr),
    .o_pwr_data   (pwr_data),
    .o_pwr_be     (pwr_be),
    .i_pwr_rdy    (pwr_rdy),
    .o_rd_train   (rd_train),
    .o_wr_train   (wr_train),
    .o_idle       (),
    .o_op_srv     (),
    .o_wr_idle    ()
);

//==================================================================
//  The ONE shared DDR3 harness (H7a step 4): train-level arbiter,
//  video line fetch > batch reads/writes > NAND > YMZ, onto the MiSTer
//  DDRAM face (served by ddr3_beh in tb_cv1k, the C++ stat slave in
//  ikacore_CV1k_tb, the real f2sdram on target).  H7b.4: the face goes
//  through the download mux below - while an ioctl download (or its
//  drain) is in flight the CV1k_ioctl packer owns the command pins and
//  the harness sits in reset (POR held by dl_hold).
//==================================================================
wire        hz_ddram_rd, hz_ddram_we;
wire [7:0]  hz_ddram_burstcnt, hz_ddram_be;
wire [28:0] hz_ddram_addr;
wire [63:0] hz_ddram_din;

// client 3: YMZ sample reads.  The YMZ770 frontend is a later phase
// (post-H7b); the reserved slot is exercised today by the sim-only
// +ymzdump probe below, and idles on target.
wire        ym_rdy, ym_dvld;
wire [63:0] ym_data;
`ifdef SYNTHESIS
wire        ym_req  = 1'b0;
wire [28:0] ym_addr = 29'd0;
wire [10:0] ym_len  = 11'd0;
`else
reg         ym_req  = 1'b0;
reg  [28:0] ym_addr = 29'd0;
reg  [10:0] ym_len  = 11'd0;
`endif

CV1k_ddr3_harness u_harness (
    .i_CLK        (blit_clk),
    .i_RST_n      (sys_rst_n),
    .i_lf_req     (lf_req),
    .i_lf_y       (lf_y),
    .i_lf_x0      (lf_x0),
    .o_lf_dvld    (lf_dvld),
    .o_lf_data    (lf_data),
    .i_prd_req    (prd_req),
    .i_prd_addr   (prd_addr),
    .i_prd_len    (prd_len),
    .o_prd_rdy    (prd_rdy),
    .o_prd_dvld   (prd_dvld),
    .o_prd_data   (prd_data),
    .i_pwr_req    (pwr_req),
    .i_pwr_addr   (pwr_addr),
    .i_pwr_data   (pwr_data),
    .i_pwr_be     (pwr_be),
    .o_pwr_rdy    (pwr_rdy),
    .i_rd_train   (rd_train),
    .i_wr_train   (wr_train),
    .i_nd_req     (nd_req),
    .i_nd_addr    (nd_addr),
    .i_nd_len     (nd_len),
    .o_nd_rdy     (nd_rdy),
    .o_nd_dvld    (nd_dvld),
    .o_nd_data    (nd_data),
    .i_ym_req     (ym_req),
    .i_ym_addr    (ym_addr),
    .i_ym_len     (ym_len),
    .o_ym_rdy     (ym_rdy),
    .o_ym_dvld    (ym_dvld),
    .o_ym_data    (ym_data),
    .DDRAM_CLK    (o_DDRAM_CLK),
    .DDRAM_BUSY   (i_DDRAM_BUSY),
    .DDRAM_BURSTCNT (hz_ddram_burstcnt),
    .DDRAM_ADDR   (hz_ddram_addr),
    .DDRAM_DOUT   (i_DDRAM_DOUT),
    .DDRAM_DOUT_READY (i_DDRAM_DOUT_READY),
    .DDRAM_RD     (hz_ddram_rd),
    .DDRAM_DIN    (hz_ddram_din),
    .DDRAM_BE     (hz_ddram_be),
    .DDRAM_WE     (hz_ddram_we)
);

//------------------------------------------------------------------
// H7b.4 download-gated DDRAM command mux.  ic_ddr_own is a 153.6-domain
// signal that covers the whole download plus the last word's drain; the
// harness is in reset throughout (dl_hold -> POR low), and stays quiet
// for >60 us after POR release (first video line train), so the switch
// itself never races a command.
//------------------------------------------------------------------
`ifdef MISTER_SDRAM
assign o_DDRAM_RD       = ic_ddr_own ? 1'b0       : hz_ddram_rd;
assign o_DDRAM_WE       = ic_ddr_own ? ic_ddr_we  : hz_ddram_we;
assign o_DDRAM_ADDR     = ic_ddr_own ? ic_ddr_addr: hz_ddram_addr;
assign o_DDRAM_DIN      = ic_ddr_own ? ic_ddr_din : hz_ddram_din;
assign o_DDRAM_BE       = ic_ddr_own ? ic_ddr_be  : hz_ddram_be;
assign o_DDRAM_BURSTCNT = ic_ddr_own ? 8'd1       : hz_ddram_burstcnt;
`else
assign o_DDRAM_RD       = hz_ddram_rd;
assign o_DDRAM_WE       = hz_ddram_we;
assign o_DDRAM_ADDR     = hz_ddram_addr;
assign o_DDRAM_DIN      = hz_ddram_din;
assign o_DDRAM_BE       = hz_ddram_be;
assign o_DDRAM_BURSTCNT = hz_ddram_burstcnt;
`endif

//------------------------------------------------------------------
// H7b.6 MiSTer video face: comb 5->8 bit replication off the registered
// ARGB1555 stream (alpha is a blend-plane flag, not scanned out).  All
// face signals update on the blit clock and are sampled at o_CE_PIXEL.
//------------------------------------------------------------------
assign o_VGA_R  = {o_PX[14:10], o_PX[14:12]};
assign o_VGA_G  = {o_PX[9:5],   o_PX[9:7]};
assign o_VGA_B  = {o_PX[4:0],   o_PX[4:2]};
assign o_VGA_DE = o_PX_DE;

`ifndef SYNTHESIS
//------------------------------------------------------------------
// H7b.5 accept probe: +ymzdump=<file> streams bytes out of the YMZ DDR3
// region through the harness's client-3 port while the system runs (the
// arbitration-under-load accept), and writes them little-endian for a
// byte-diff against the u23/u24 images.
//   +ymzoff=<byte off in the 16 MB region>   (default 0 = u23[0];
//                                             8 MiB = u24[0])
//   +ymzlen=<bytes>       (default 65536)
//   +ymztrain=<words>     (default 64  = 512 B per train)
//   +ymzgap=<blit clks>   (default 4096 ~ 26.7 us between trains)
//------------------------------------------------------------------
integer ymz_fd = 0;
longint ymz_off = 0, ymz_len_bytes = 65536;
longint ymz_train_w = 64, ymz_gap = 4096;
longint ymz_left_w = 0, ymz_burst_left = 0, ymz_gap_cnt = 0, ymz_bytes_out = 0;
reg [28:0] ymz_next_w;
reg [1:0]  ymz_st = 2'd0;               // 0 idle/gap, 1 request, 2 collect

initial begin
    string ymz_file;
    if ($value$plusargs("ymzdump=%s", ymz_file)) begin
        void'($value$plusargs("ymzoff=%d",   ymz_off));
        void'($value$plusargs("ymzlen=%d",   ymz_len_bytes));
        void'($value$plusargs("ymztrain=%d", ymz_train_w));
        void'($value$plusargs("ymzgap=%d",   ymz_gap));
        ymz_fd = $fopen(ymz_file, "wb");
        if (ymz_fd == 0) $display("[ymz] ERROR: cannot open %s", ymz_file);
        else $display("[ymz] probe: %0d bytes from region byte 0x%0x, %0d words/train, gap %0d clks",
                      ymz_len_bytes, ymz_off, ymz_train_w, ymz_gap);
    end
end

always @(posedge blit_clk) if (ymz_fd != 0 && sys_rst_n) begin
    case (ymz_st)
    2'd0: begin                          // arm / inter-train gap
        if (ymz_left_w == 0 && ymz_bytes_out == 0) begin
            ymz_left_w  = (ymz_len_bytes + 7) / 8;
            ymz_next_w  = 29'h07A0_0000 + 29'(ymz_off / 8);
            ymz_gap_cnt = ymz_gap;
        end
        if (ymz_left_w == 0) begin
            $display("[ymz] probe done: %0d bytes dumped", ymz_bytes_out);
            $fclose(ymz_fd); ymz_fd = 0;
        end
        else if (ymz_gap_cnt != 0) ymz_gap_cnt = ymz_gap_cnt - 1;
        else begin
            ym_req  <= 1'b1;
            ym_addr <= ymz_next_w;
            ym_len  <= 11'((ymz_left_w > ymz_train_w) ? ymz_train_w : ymz_left_w);
            ymz_st  <= 2'd1;
        end
    end
    2'd1: if (ym_rdy) begin              // accepted this edge
        ym_req         <= 1'b0;
        ymz_burst_left = longint'(ym_len);
        ymz_st         <= 2'd2;
    end
    2'd2: begin
        if (ym_dvld) begin
            for (int k = 0; k < 8; k++)
                if (ymz_bytes_out < ymz_len_bytes) begin
                    $fwrite(ymz_fd, "%c", ym_data[8*k +: 8]);
                    ymz_bytes_out = ymz_bytes_out + 1;
                end
            ymz_next_w     = ymz_next_w + 29'd1;
            ymz_left_w     = ymz_left_w - 1;
            ymz_burst_left = ymz_burst_left - 1;
            if (ymz_burst_left == 0) begin
                ymz_gap_cnt = ymz_gap;
                ymz_st      <= 2'd0;
            end
        end
    end
    default: ymz_st <= 2'd0;
    endcase
end
`endif

endmodule
`default_nettype wire
