`default_nettype none
//============================================================================
// CV1k_sdram_control.sv - double-pumped MiSTer SDRAM hub (docs/double_pump_sdram.md)
//
// One 16-bit MiSTer SDRAM module (2x AS4C32M16SB, models/mister_128mb.sv)
// stands in for BOTH shared-bus devices of the SH-3 external bus:
//
//   * U1 work RAM (CS3, 32-bit SDRAM @ CKIO): every grid command is re-issued
//     at 2xCKIO with a NOP in the filler slot; every 32-bit beat becomes one
//     BL=2 pair of 16-bit beats.  CL3 @ 2x lands the completed word on
//     exactly the same edge as the original CL2 @ 1x, so the CKIO-grid
//     timing (wait states, DMAC cadence, BREQ/BACK) is preserved bit-exactly.
//   * U4 program NOR (CS0, 16-bit async, 10 waits = 12 CKIO/access): served
//     from a dedicated SDRAM window.  Legal because CS0 and CS3 share the
//     physical bus on the real board - they are mutually exclusive in time.
//
// Clocking: single i_CLK domain (= SH-3 architectural clock, 2xCKIO), slot
// phase from i_CKIO_PCEN - no derived clocks, no CDC.  The physical chip is
// clocked with the same i_CLK in sim; on hardware the SDRAM_CLK copy comes
// from a dedicated PLL output whose phase is tuned to the board flight time.
//
// Command timing equivalence (registration-convention exact vs the 1x model):
//   grid pins hold a command during CKIO cycle N = fast cycles [2N, 2N+2).
//   1x path : chip clocked by CKIO registers it at edge 2N+2.
//   pump    : cmd regs load the pins at 2N+1 (mid-CKIO, pins stable), chip
//             registers at 2N+2 - same instant, zero added delay.  The
//             filler slot registers NOP at 2N+3.
//   read    : BL2 READ registered at edge k -> beats capturable at k+3, k+4;
//             with grid CL2 the BSC samples D at k+4 = the identical edge the
//             32-bit part delivered the word.  hi half is registered at k+3,
//             lo half passes combinationally into the BSC capture register.
//
// Memory map (chip 0, CS level 0):
//   work RAM : BA = grid bank, RA = {1'b0, grid row[11:0]},
//              CA = {1'b0, grid col[7:0], beat}
//              (grid row bit 11 is tied 0 by the CV1000-B top - 11-bit rows
//              pass through unchanged; the CV1000-D top feeds the full 12-bit
//              row of its 4M x 32 part, rows 0x000-0xFFF)
//   U4 NOR   : BA = 0, RA = {2'b10, F[20:10]}, CA = F[9:0]   (F = A[21:1],
//              rows 0x1000-0x17FF, 4 MB dense; 16 Mbit images are streamed
//              twice by the loader/MRA so the undecoded-A21 mirror is real)
// Mode register (programmed once at init, grid MRS cycles are acked+dropped):
//   BL=2 sequential, CL=2  (A[11:0] = 0x021)
//
// Module quirks handled (see mister_128mb.sv):
//   * CS_n is a chip-address bit - idle is encoded as NOP, never deselect;
//     chip 1 is never selected, initialized, or refreshed (it holds no data)
//   * chip DQM = A[12:11]: driven 00 on every cycle except write-data beats
//     (byte strobes) and ACT row phases.  An ACT with row[12:11] != 00 masks
//     the read beat driven 2 chip clocks later (DQM read latency 2), so such
//     ACTs are only ever issued where no beat can be in flight: grid/NOR ACTs
//     are covered by their own RCD lead-in, maintenance ACTs by the window
//     rules below.  PRE always drives A[12:11] = 00 (bit 11 of the grid
//     address is dropped - only A10 and BA matter to a precharge).
//
// Refresh: the BSC's CBR refresh commands are forwarded 1:1 (the bus stall
// is CPU-visible timing) but cover only 2048 of the chip's 8192 rows per
// 64 ms.  The remainder is a hidden ACT+PRE row-maintenance scheduler
// (docs/double_pump_sdram.md section 6.2) that walks every used row inside
// 64 ms using provably-safe windows only - it never delays or reorders a
// grid command, so the CKIO-grid A/B equivalence is untouched:
//   * blit tenures (i_BACK_n low + i_BLIT_WIN sideband: blit_fetch guarantees
//     >= 5 CKIO without a PALL/ACT): rows with RA[12:11] = 00 only (read
//     beats are dense mid-train - see the DQM note above)
//   * ordinary bus cycles (CS0 via i_N_CS_n, CS4/5/6 via i_ORD_CS_n): the
//     BSC's SDRAM engine is structurally frozen while an ordinary cycle owns
//     the shared bus (bsc.sv E_IDLE dispatch gate on ord_busy), and WCR2 =
//     0xFDD7 makes every such access >= 8 CKIO - ANY row is safe here,
//     including the DQM-exposed D-board upper half and the NOR window
// CPU SDRAM transactions host nothing BY CHOICE: the BSC dispatches a new
// op's first command with zero pin warning ("no NOP between ops"), so even
// a confirmed burst leaves only ~3 fast edges of tail margin before a
// possible same-bank ACT - and the two classes above already meet demand.
// The BSC's REF lockout (TRAS+TPC = 4 CKIO) has no room after tRFC either.
//
// H7b.8: the download idle-grid window class (w_dl) closes the OPEN ITEM
// found at H7b.4 - during an ioctl DOWNLOAD the CPU is held in RESETM and
// no CS activity opens either window class above, so a real seconds-long
// HPS stream would have decayed early-written NOR-window rows.  While
// i_IOCTL_DOWNLOAD is high every engine-quiet slot is an any-row window
// (the grid is provably silent); the loader engine and the maintenance
// pair interlock both ways (a pair opens only when the loader is idle,
// the loader holds in LS_IDLE - o_IOCTL_WAIT stalls the HPS - while a
// pair is open).  Accept: +refage bound holds THROUGH a download and
// pairs issue while streaming (H7b.8 sim run).
//============================================================================
module CV1k_sdram_control #(
    parameter        NOR_BSWAP = 1'b0,       // 1: swap ioctl byte pairs (MAME dumps need 0)
    parameter [31:0] INIT_WAIT = 32'd200     // NOP cycles before the JEDEC init sequence
) (
    input  wire         i_CLK,               // 2xCKIO architectural clock
    input  wire         i_RST_n,             // memory-subsystem reset (released before CPU POR)
    input  wire         i_CKIO_PCEN,         // i_CLK cycle in which CKIO rises

    //------ grid: CS3 SDRAM bus, post blit_own mux (pin-true, 1xCKIO) ------
    input  wire [11:0]  i_G_A,               // muxed row/col; B board: {1'b0, A[12:2]}
                                             // (AMX 0111), D board: 12-bit row
    input  wire [1:0]   i_G_BA,              // A[14:13]
    input  wire         i_G_CS_n,            // CS3_n / bf_CS_n
    input  wire         i_G_RAS_n,
    input  wire         i_G_CAS_n,
    input  wire         i_G_WE_n,            // RD_WR / bf_WE
    input  wire [3:0]   i_G_DQM,             // WE_n[3:0] / bf_DQM (byte strobes)
    input  wire         i_G_CKE,
    input  wire [31:0]  i_G_WDATA,           // shared-bus write data (o_D_O)
    output wire [31:0]  o_G_RDATA,           // drives D[31:0] on CS3 read beats
    output wire         o_G_RDATA_OE,

    //------ NOR: CS0 async bus (area 0, 16-bit) ------
    input  wire         i_N_CS_n,            // CS0_n
    input  wire         i_N_RD_n,            // RD_n (flash OE)
    input  wire [20:0]  i_N_A,               // A[21:1] halfword address
    input  wire         i_N_WR_n,            // WE_n[1]&WE_n[0] (ignored - see write note)
    output wire [15:0]  o_N_RDATA,           // drives D[15:0] on CS0 reads
    output wire         o_N_RDATA_OE,

    //------ refresh-scheduler observability (section 6.2 windows) ------
    input  wire         i_BACK_n,            // SH-3 bus grant (0 = blit tenure)
    input  wire         i_BLIT_WIN,          // blit_fetch: >= 5 CKIO w/o PALL/ACT
    input  wire         i_ORD_CS_n,          // CS4&CS5&CS6, BUS_OE-qualified

    //------ MiSTer HPS ioctl (active only while the CPU is held in reset) --
    input  wire         i_IOCTL_DOWNLOAD,
    input  wire         i_IOCTL_WR,
    input  wire [26:0]  i_IOCTL_ADDR,        // byte address within the stream
    input  wire [7:0]   i_IOCTL_DATA,
    input  wire [15:0]  i_IOCTL_INDEX,       // unused: NOR = first 32 Mbit of the stream
    output wire         o_IOCTL_WAIT,

    //------ status ------
    output wire         o_INIT_DONE,         // JEDEC init sequence complete (H7b.1:
                                             // gates the CPU POR in the reset sequencer)

    //------ MiSTer SDRAM module pins ------
    output reg  [12:0]  o_S_A,               // A[12:11] double as chip DQM on this module
    output reg  [1:0]   o_S_BA,
    output reg          o_S_nCS,             // chip-address bit: 0 = chip 0
    output reg          o_S_nRAS,
    output reg          o_S_nCAS,
    output reg          o_S_nWE,
    output wire [1:0]   o_S_DQM,             // connector DQML/H (unrouted on 128MB boards)
    output reg          o_S_CKE,
    output reg  [15:0]  o_S_DQ_O,
    output reg          o_S_DQ_OE,
    input  wire [15:0]  i_S_DQ_I
);

wire _unused_ioctl_index = ^i_IOCTL_INDEX;

// {nRAS,nCAS,nWE} command encodings (nCS held 0 = chip 0 for everything)
localparam [2:0] CMD_NOP = 3'b111, CMD_ACT = 3'b011, CMD_RD  = 3'b101,
                 CMD_WR  = 3'b100, CMD_PRE = 3'b010, CMD_REF = 3'b001,
                 CMD_MRS = 3'b000;

localparam [12:0] MRS_BL2_CL2 = 13'h0021;    // BL=2 seq, CL=2, burst writes

assign o_S_DQM = o_S_A[12:11];               // the 128MB module routes DQM on A[12:11]

//------------------------------------------------------------------
// slot phase: pcen_d is high during the FIRST fast half of each CKIO
// cycle, i.e. at the edge where the (just-updated) grid pins are loaded.
//------------------------------------------------------------------
reg pcen_d;
always @(posedge i_CLK) pcen_d <= i_CKIO_PCEN;

//------------------------------------------------------------------
// read-capture pipelines: bit 0 set at the edge a READ is loaded into the
// output regs.  Empirical timing vs the delay-stripped model (blocking
// data-out, evaluated before this block's NBA sampling): READ issued at
// edge k -> chip registers at k+1 -> beat0 on DQ at edge k+3, beat1 at
// k+4, each stable for exactly one edge.  The assembled word is then
// driven for one full CKIO cycle [k+5, k+7) - the identical window the
// 32-bit model drove in the baseline build.
//------------------------------------------------------------------
reg [5:0] rdg_sh;                            // grid BL2 reads
reg [4:0] rdn_sh;                            // NOR/engine BL2 reads (beat 0 only used)
reg [15:0] rd_hi;                            // grid beat-0 (D[31:16]) capture
reg [31:0] rd_word;                          // assembled word, held for the CKIO window

// DQ mid-window capture bank (H7b.8): every falling edge samples the pad.
// Each CL2 beat is stable across the falling edge inside its drive window
// (sim model drives beats for exactly one fast period; on silicon the
// SDRAM_CLK phase preset centers tAC..tOH around this edge), so dq_n holds
// beat k from E+k.5 to E+k+1.5 -- the SAME value every posedge consumer
// previously read live off the pad, but launched from a register.  This is
// the only pad-timed DQ endpoint; everything downstream (rd_hi/rd_word/
// nor_data/the o_G_RDATA arm) is register-to-register.  Value-identical in
// RTL sim by construction (FASTBOOT datum re-proven).
reg [15:0] dq_n;
always @(negedge i_CLK) dq_n <= i_S_DQ_I;

// The BSC latches i_D_I exactly at fast edge E+4 (CKIO E+2: i_BCEN && rd_lat,
// CL2 pipeline in bsc.sv). At that same edge this module commits its NBAs, so
// the sample may resolve pre- or post-commit; both mux arms carry the word
// either way: pre-commit rdg_sh[2]=1 selects {rd_hi, dq_n} (dq_n holds
// beat 1 from E+3.5), post-commit rdg_sh[3]=1 selects the registered rd_word.
assign o_G_RDATA    = rdg_sh[2] ? {rd_hi, dq_n} : rd_word;
assign o_G_RDATA_OE = rdg_sh[2] | rdg_sh[3];

//------------------------------------------------------------------
// init sequencer state
//------------------------------------------------------------------
localparam [2:0] IS_WAIT = 3'd0, IS_PALL = 3'd1, IS_REF = 3'd2,
                 IS_MRS  = 3'd3, IS_DONE = 3'd4;
reg [2:0]  ist;
reg [31:0] icnt;
reg [3:0]  iref;
wire init_done = (ist == IS_DONE);
assign o_INIT_DONE = init_done;

//------------------------------------------------------------------
// NOR fetch engine state
//------------------------------------------------------------------
localparam [2:0] NS_IDLE = 3'd0, NS_ACT = 3'd1, NS_RCD = 3'd2,
                 NS_RD   = 3'd3, NS_CAP = 3'd4;
reg [2:0]  nst;
reg [1:0]  ncnt;
reg [20:0] nor_addr;
reg [15:0] nor_data;
reg        nor_rdy;

wire nor_read_req = !i_N_CS_n && !i_N_RD_n;
assign o_N_RDATA    = nor_data;
assign o_N_RDATA_OE = nor_rdy && nor_read_req && (nor_addr == i_N_A);

// CS0 writes are ignored BY DESIGN (final, not a v1 gap): no CV1k game
// programs the flash - saves live in the RTC-9701 EEPROM - so a write
// strobe here is at most a stray unlock sequence.  One-shot info only.
reg nor_wr_trap = 1'b0;
always @(posedge i_CLK) begin
    if (!i_N_CS_n && !i_N_WR_n) begin
        if (!nor_wr_trap)
            $display("[CV1k_sdram_control] note: CS0 flash write ignored @%0t A=%06x (by design)",
                     $time, {i_N_A, 1'b0});
        nor_wr_trap <= 1'b1;
    end else nor_wr_trap <= 1'b0;
end

//------------------------------------------------------------------
// ioctl loader state (NOR window = stream bytes 0x000000-0x3FFFFF)
//------------------------------------------------------------------
localparam [2:0] LS_IDLE = 3'd0, LS_ACT = 3'd1, LS_RCD = 3'd2,
                 LS_WR   = 3'd3, LS_DAL = 3'd4;
reg [2:0]  lst;
reg [2:0]  lcnt;
reg [7:0]  ld_lo;
reg [15:0] ld_hw;
reg [20:0] ld_a;
reg        ld_go;                            // halfword pending -> engine
reg        ld_beat2;                         // masked second write beat pending

assign o_IOCTL_WAIT = ld_go || (lst != LS_IDLE);

always @(posedge i_CLK) begin
    if (!i_RST_n) begin
        ld_lo <= 8'h0; ld_hw <= 16'h0; ld_a <= 21'h0; ld_go <= 1'b0;
    end else if (i_IOCTL_DOWNLOAD && i_IOCTL_WR) begin
        if (!i_IOCTL_ADDR[0]) begin
            ld_lo <= i_IOCTL_DATA;           // even byte first (MiSTer streams LE)
        end else if (i_IOCTL_ADDR < 27'h040_0000) begin
            // {odd,even} little-endian assembly of a byte-swapped MAME dump
            // IS the SH-3 bus halfword (file 3D DF -> bus 0xDF3D)
            ld_hw <= NOR_BSWAP ? {ld_lo, i_IOCTL_DATA} : {i_IOCTL_DATA, ld_lo};
            ld_a  <= i_IOCTL_ADDR[21:1];
            ld_go <= 1'b1;
        end
    end else if (lst != LS_IDLE) begin
        // engine took the halfword (ld_hw/ld_a stay latched until it is done)
        ld_go <= 1'b0;
    end
end

//------------------------------------------------------------------
// hidden refresh row-maintenance scheduler (doc section 6.2)
//
// Sweep domain, per bank: LO = rows 0x000-0x7FF (chip RA[12:11] = 00,
// DQM-inert, refreshable in blit windows), HI = the DQM-exposed rest
// (bank 0: 0x800-0x17FF incl. the NOR window; banks 1-3: 0x800-0xFFF),
// refreshable only in ordinary-cycle windows.  One ACT+PRE pair per grant:
// ACT at a free slot-B edge, PRE >= 5 edges later (tRAS 42 ns), 3-edge
// cooldown before the bank may be re-opened (tRP/tRC).  Deficit counters
// tick at each region's demand rate (row count / 64 ms), capped at one
// full region pass, so post-drought catch-up runs at window rate and no
// backlog is ever forgotten (see the DEF_CAP note below).
// The B board over-sweeps rows its 11-bit grid never uses - harmless, and
// it keeps one configuration (and one proof) for every board.
//------------------------------------------------------------------
reg        wr_beat2;                         // grid write lo-half pending
reg [15:0] wr_lo;
reg [1:0]  wr_lo_m;

localparam [12:0] HI_TOP_B0 = 13'h17FF;      // bank 0 HI wrap (incl. NOR rows)
localparam [12:0] HI_TOP    = 13'h0FFF;      // banks 1-3 HI wrap
// demand ticks in fast edges: 90% of (64 ms / rows-per-region) @ 102.4 MHz.
// The 10% headroom is load-bearing: at exactly the demand rate the
// steady-state re-visit interval is exactly 64 ms, and any window jitter
// puts a row over the bound (measured: 64.000-64.003 ms ages in the replay
// before the margin).  A full pass now takes 57.6 ms, 6.4 ms of slack.
localparam [11:0] TICK_LO  = 12'd2880;       // 2048 rows -> 28.1 us
localparam [11:0] TICK_HI0 = 12'd1440;       // bank 0: 4096 rows -> 14.1 us

reg  [3:0]  ga_open;                         // banks the grid holds open (ACT/PRE/PALL parse)
reg  [2:0]  cool [0:3];                      // per-bank close cooldown (tRP/tRC)
reg  [3:0]  ref_hold;                        // tRFC guard after a forwarded REF
reg  [1:0]  act_gap;                         // tRRD guard after any ACT

// deficit = rows overdue, capped at one full region pass (owing more than a
// whole pass is meaningless).  Initialized FULL at reset so the first sweep
// pass runs at window rate instead of tick rate - a zero start schedules the
// first pass to finish at exactly t = 64 ms with no margin, and a 7-bit
// saturating counter FORGOT drought backlog (both showed up as ~64.1 ms
// startup-transient ages in the ddpsdoj replay before this shape).
localparam [12:0] DEF_CAP_LO  = 13'd2048;
localparam [12:0] DEF_CAP_HI0 = 13'd4096;
localparam [12:0] DEF_CAP_HI  = 13'd2048;
reg  [10:0] ptr_lo [0:3];                    // next LO sweep row
reg  [12:0] ptr_hi [0:3];                    // next HI sweep row
reg  [12:0] def_lo [0:3] /*verilator public_flat_rd*/;  // rows owed (public:
reg  [12:0] def_hi [0:3] /*verilator public_flat_rd*/;  // +refage monitor)
reg  [11:0] tick_lo_c, tick_hi0_c;
wire        lo_tick  = (tick_lo_c  == TICK_LO  - 12'd1);   // all LO + banks 1-3 HI
wire        hi0_tick = (tick_hi0_c == TICK_HI0 - 12'd1);   // bank 0 HI

reg         m_open;                          // maintenance row open
reg         m_cool;                          // 1 edge after a pair closes
                                             // (loader tRP guard, see LS_IDLE)
reg  [1:0]  m_bank;
reg         m_hi;
reg  [2:0]  m_cnt;                           // edges since our ACT

reg         back_ff, bwin_ff, ordn_ff, cs0n_ff, dl_ff;
reg  [4:0]  ord_cnt;                         // edges since CS4/5/6 strobe fell
reg  [4:0]  cs0_cnt;                         // edges since CS0 strobe fell

// window decode.  Ordinary-cycle placement inside the guaranteed access
// (WCR2 0xFDD7: CS0/CS6 = 12 CKIO, CS5 = 10, CS4 = 8 -> >= 16 fast edges):
// CS0 pairs run after the NOR fetch engine is done (edges 10-14 of 24);
// CS4/5/6 pairs near the head (edges 2-6 of >= 16).  Both leave the PRE +
// tRP complete before the earliest post-access grid dispatch.
wire w_blit = !back_ff && bwin_ff;
wire w_cs0  = !cs0n_ff && (cs0_cnt >= 5'd10) && (cs0_cnt <= 5'd14);
wire w_ord  = !ordn_ff && (ord_cnt >= 5'd2)  && (ord_cnt <= 5'd6);
// any-row windows additionally need a beat-free data bus: an exposed-row ACT
// at edge k masks the beat driven at k+3, i.e. a READ loaded at k+1/k+2 -
// impossible once the engines are idle and the grid is frozen (ord_busy),
// but live tails (rdg_sh/rdn_sh) must have drained.
wire eng_quiet = (nst == NS_IDLE) && (rdn_sh == 5'b0) && (lst == LS_IDLE)
                 && !ld_beat2 && !wr_beat2 && (rdg_sh[2:0] == 3'b0);
// download idle-grid window (H7b.8): the CPU is held in RESETM for the
// whole download, so every engine-quiet slot is any-row safe.  The loader
// engine can want the bus at any time (a new HPS byte) - the LS_IDLE
// dispatch below holds it out while a maintenance row is open, and
// o_IOCTL_WAIT (ld_go) stalls the stream for those few edges.
wire w_dl  = dl_ff && eng_quiet;
wire w_any = (w_cs0 || w_ord || w_dl) && eng_quiet;
wire w_lo  = w_blit || w_any;

wire [3:0] b_elig = {init_done && !ga_open[3] && (cool[3] == 3'd0) && !(m_open && (m_bank == 2'd3)),
                     init_done && !ga_open[2] && (cool[2] == 3'd0) && !(m_open && (m_bank == 2'd2)),
                     init_done && !ga_open[1] && (cool[1] == 3'd0) && !(m_open && (m_bank == 2'd1)),
                     init_done && !ga_open[0] && (cool[0] == 3'd0) && !(m_open && (m_bank == 2'd0))};

// pick: HI region first in any-row windows (it has no other supply), then
// LO; within a region the highest-deficit eligible bank, ties -> low bank.
// H7b.8: restructured from a serial 8-deep compare/mux scan (the fitter
// chained it into a ~25 ns cone ending at o_S_A - the first-fit c102
// worst path at -21 ns) into balanced argmax trees.  Semantics are
// EXACTLY the old scan's: eligibility masks a bank's deficit to 0 (a 0
// deficit was never selectable - the scan's strict > against sel_best
// = 0), and "right wins only on strictly greater" at every tree node
// reproduces argmax-with-ties->lowest-bank.
wire [12:0] dq_hi0 = b_elig[0] ? def_hi[0] : 13'd0;
wire [12:0] dq_hi1 = b_elig[1] ? def_hi[1] : 13'd0;
wire [12:0] dq_hi2 = b_elig[2] ? def_hi[2] : 13'd0;
wire [12:0] dq_hi3 = b_elig[3] ? def_hi[3] : 13'd0;
wire [12:0] dq_lo0 = b_elig[0] ? def_lo[0] : 13'd0;
wire [12:0] dq_lo1 = b_elig[1] ? def_lo[1] : 13'd0;
wire [12:0] dq_lo2 = b_elig[2] ? def_lo[2] : 13'd0;
wire [12:0] dq_lo3 = b_elig[3] ? def_lo[3] : 13'd0;

wire        h01   = (dq_hi1 > dq_hi0);
wire [12:0] h01_d = h01 ? dq_hi1 : dq_hi0;
wire        h23   = (dq_hi3 > dq_hi2);
wire [12:0] h23_d = h23 ? dq_hi3 : dq_hi2;
wire        hfin  = (h23_d > h01_d);
wire [12:0] hi_d  = hfin ? h23_d : h01_d;
wire [1:0]  hi_b  = hfin ? {1'b1, h23} : {1'b0, h01};

wire        l01   = (dq_lo1 > dq_lo0);
wire [12:0] l01_d = l01 ? dq_lo1 : dq_lo0;
wire        l23   = (dq_lo3 > dq_lo2);
wire [12:0] l23_d = l23 ? dq_lo3 : dq_lo2;
wire        lfin  = (l23_d > l01_d);
wire [12:0] lo_d  = lfin ? l23_d : l01_d;
wire [1:0]  lo_b  = lfin ? {1'b1, l23} : {1'b0, l01};

wire        hi_pick  = w_any && (hi_d != 13'd0);
wire        lo_pick  = w_lo  && (lo_d != 13'd0);
wire        sel_v    = hi_pick || lo_pick;
wire        sel_hi   = hi_pick;
wire [1:0]  sel_bank = hi_pick ? hi_b : lo_b;
wire [12:0] sel_row = sel_hi ? ptr_hi[sel_bank] : {2'b00, ptr_lo[sel_bank]};

integer sbi;                                 // scheduler bookkeeping loop var

// retire the open maintenance row: advance its sweep pointer, pay one unit
// of deficit (net 0 if this edge also ticks), start the tRP/tRC cooldown.
// Called from the PRE dispatch and from the external-close paths (a grid
// PALL/PRE that lands on our bank still refreshed the row if tRAS was met).
task m_credit;
begin
    if (m_hi) begin
        def_hi[m_bank] <= def_hi[m_bank]
            - ((((m_bank == 2'd0) && hi0_tick) || ((m_bank != 2'd0) && lo_tick)) ? 13'd0 : 13'd1);
        ptr_hi[m_bank] <= (ptr_hi[m_bank] == ((m_bank == 2'd0) ? HI_TOP_B0 : HI_TOP))
                          ? 13'h0800 : ptr_hi[m_bank] + 13'd1;
    end
    else begin
        def_lo[m_bank] <= def_lo[m_bank] - (lo_tick ? 13'd0 : 13'd1);
        ptr_lo[m_bank] <= ptr_lo[m_bank] + 11'd1;    // wraps at 0x7FF naturally
    end
    cool[m_bank] <= 3'd3;
    m_open <= 1'b0;
end
endtask

//------------------------------------------------------------------
// main output sequencer - single always block, one command per fast edge.
// priority: init > grid slot A > pending write beat 2 > engines (slot B)
// > refresh scheduler (leftover slot-B edges)
//------------------------------------------------------------------
always @(posedge i_CLK) begin
    if (!i_RST_n) begin
        {o_S_nCS, o_S_nRAS, o_S_nCAS, o_S_nWE} <= {1'b0, CMD_NOP};
        o_S_A <= 13'h0; o_S_BA <= 2'b00; o_S_CKE <= 1'b1;
        o_S_DQ_O <= 16'h0; o_S_DQ_OE <= 1'b0;
        rdg_sh <= 6'b0; rdn_sh <= 5'b0;
        rd_hi <= 16'h0; rd_word <= 32'h0;
        wr_beat2 <= 1'b0; wr_lo <= 16'h0; wr_lo_m <= 2'b00;
        ist <= IS_WAIT; icnt <= 32'd0; iref <= 4'd0;
        nst <= NS_IDLE; ncnt <= 2'd0; nor_addr <= 21'h0;
        nor_data <= 16'h0; nor_rdy <= 1'b0;
        lst <= LS_IDLE; lcnt <= 3'd0; ld_beat2 <= 1'b0;
        ga_open <= 4'b0; ref_hold <= 4'd0; act_gap <= 2'd0;
        m_open <= 1'b0; m_cool <= 1'b0; m_bank <= 2'd0; m_hi <= 1'b0; m_cnt <= 3'd0;
        tick_lo_c <= 12'd0; tick_hi0_c <= 12'd0;
        back_ff <= 1'b1; bwin_ff <= 1'b0; ordn_ff <= 1'b1; cs0n_ff <= 1'b1;
        dl_ff <= 1'b0;
        ord_cnt <= 5'd0; cs0_cnt <= 5'd0;
        for (sbi = 0; sbi < 4; sbi = sbi + 1) begin
            cool[sbi]   <= 3'd0;
            ptr_lo[sbi] <= 11'd0;
            ptr_hi[sbi] <= 13'h0800;
            def_lo[sbi] <= DEF_CAP_LO;       // full backlog: first pass runs
            def_hi[sbi] <= (sbi == 0) ? DEF_CAP_HI0 : DEF_CAP_HI;  // at window rate
        end
    end else begin
        // defaults for this edge: NOP, DQM active (A[12:11]=00), bus released
        {o_S_nCS, o_S_nRAS, o_S_nCAS, o_S_nWE} <= {1'b0, CMD_NOP};
        o_S_A <= 13'h0; o_S_BA <= 2'b00;
        o_S_DQ_OE <= 1'b0;
        o_S_CKE <= init_done ? (pcen_d ? i_G_CKE : o_S_CKE) : 1'b1;

        // capture pipelines shift every edge.  With E = the CKIO edge opening
        // the grid READ cycle: issue @E+1, chip reg @E+2, CL2 beats on DQ at
        // their drive edges E+3/E+4; the word reaches o_G_RDATA at E+4 via
        // the consistent-snapshot mux - the BSC's hard capture edge
        // (i_BCEN && rd_lat = CKIO E+2 = fast E+4, see doc section 5.1).
        rdg_sh <= {rdg_sh[4:0], 1'b0};
        rdn_sh <= {rdn_sh[3:0], 1'b0};
        if (rdg_sh[1]) rd_hi   <= dq_n;                   // CL2 beat 0 (dq_n @E+2.5)
        if (rdg_sh[2]) rd_word <= {rd_hi, dq_n};          // CL2 beat 1 (dq_n @E+3.5)

        //--------------------------------------------------------------
        // refresh-scheduler bookkeeping (every edge; guard decrements sit
        // BEFORE the dispatch/parse code so their reloads win the NBA race)
        //--------------------------------------------------------------
        back_ff <= i_BACK_n;
        bwin_ff <= i_BLIT_WIN;
        ordn_ff <= i_ORD_CS_n;
        cs0n_ff <= i_N_CS_n;
        dl_ff   <= i_IOCTL_DOWNLOAD;
        m_cool  <= m_open;
        ord_cnt <= i_ORD_CS_n ? 5'd0 : ((ord_cnt == 5'h1F) ? ord_cnt : ord_cnt + 5'd1);
        cs0_cnt <= i_N_CS_n   ? 5'd0 : ((cs0_cnt == 5'h1F) ? cs0_cnt : cs0_cnt + 5'd1);
        if (ref_hold != 4'd0) ref_hold <= ref_hold - 4'd1;
        if (act_gap  != 2'd0) act_gap  <= act_gap  - 2'd1;
        if (m_open && (m_cnt != 3'd7)) m_cnt <= m_cnt + 3'd1;
        tick_lo_c  <= lo_tick  ? 12'd0 : tick_lo_c  + 12'd1;
        tick_hi0_c <= hi0_tick ? 12'd0 : tick_hi0_c + 12'd1;
        for (sbi = 0; sbi < 4; sbi = sbi + 1) begin
            if (cool[sbi] != 3'd0) cool[sbi] <= cool[sbi] - 3'd1;
            if (lo_tick && (def_lo[sbi] < DEF_CAP_LO)) def_lo[sbi] <= def_lo[sbi] + 13'd1;
        end
        if (lo_tick) begin                   // banks 1-3 HI share the LO period
            if (def_hi[1] < DEF_CAP_HI) def_hi[1] <= def_hi[1] + 13'd1;
            if (def_hi[2] < DEF_CAP_HI) def_hi[2] <= def_hi[2] + 13'd1;
            if (def_hi[3] < DEF_CAP_HI) def_hi[3] <= def_hi[3] + 13'd1;
        end
        if (hi0_tick && (def_hi[0] < DEF_CAP_HI0)) def_hi[0] <= def_hi[0] + 13'd1;

        //--------------------------------------------------------------
        if (!init_done) begin
            // JEDEC init for chip 0 while the whole system is in reset
            icnt <= icnt + 32'd1;
            case (ist)
                IS_WAIT: if (icnt >= INIT_WAIT) begin
                    {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_PRE;
                    o_S_A <= 13'h0400;                        // A10: precharge all
                    icnt <= 32'd0; ist <= IS_PALL;
                end
                IS_PALL: if (icnt >= 32'd3) begin
                    {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_REF;
                    icnt <= 32'd0; iref <= 4'd1; ist <= IS_REF;
                end
                IS_REF: if (icnt >= 32'd8) begin              // tRFC 63 ns < 8 edges
                    if (iref >= 4'd8) begin
                        {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_MRS;
                        o_S_A <= MRS_BL2_CL2;
                        icnt <= 32'd0; ist <= IS_MRS;
                    end else begin
                        {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_REF;
                        icnt <= 32'd0; iref <= iref + 4'd1;
                    end
                end
                IS_MRS: if (icnt >= 32'd3) ist <= IS_DONE;    // tMRD 2
                default: ;
            endcase
        end
        //--------------------------------------------------------------
        else if (wr_beat2) begin
            // grid write lo half: NOP command + data + byte strobes on A[12:11]
            // (never collides with slot A: it always lands on the filler edge)
            o_S_A     <= {wr_lo_m[1], wr_lo_m[0], 11'h0};
            o_S_DQ_O  <= wr_lo;
            o_S_DQ_OE <= 1'b1;
            wr_beat2  <= 1'b0;
        end
        //--------------------------------------------------------------
        else if (ld_beat2) begin
            // loader write beat 2: fully masked (single-halfword write).
            // Outranks slot A: the beat can land on a pcen edge, and the grid
            // is structurally silent during download (CPU held in reset).
            o_S_A     <= {2'b11, 11'h0};
            o_S_DQ_OE <= 1'b1;
            ld_beat2  <= 1'b0;
        end
        //--------------------------------------------------------------
        else if (pcen_d) begin
            // slot A: translate whatever the grid drove this CKIO cycle
            if (!i_G_CS_n) begin
                case ({i_G_RAS_n, i_G_CAS_n, i_G_WE_n})
                    CMD_ACT: begin
                        {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_ACT;
                        o_S_A  <= {1'b0, i_G_A};              // 12-bit row pass-through
                        o_S_BA <= i_G_BA;                     // (B board drives bit 11 = 0)
                        ga_open[i_G_BA] <= 1'b1;
                        act_gap <= 2'd2;
                        if (m_open && (i_G_BA == m_bank))
                            $display("[CV1k_sdram_control] ERROR: grid ACT bank %0d over open maintenance row @%0t",
                                     i_G_BA, $time);
                    end
                    CMD_RD: begin                             // one grid CAS -> one BL2 pair
                        {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_RD;
                        o_S_A  <= {2'b00, i_G_A[10], 1'b0, i_G_A[7:0], 1'b0};
                        o_S_BA <= i_G_BA;
                        // whole-vector re-assign: a bit-select NBA after the
                        // vector-shift NBA is silently dropped by Verilator
                        rdg_sh <= {rdg_sh[4:0], 1'b1};
                        if (i_G_A[10]) begin                  // auto-precharge: the normal
                            ga_open[i_G_BA] <= 1'b0;          // close (MCR 0x543C RASD=0 -
                            cool[i_G_BA] <= 3'd7;             // every op ends with AP)
                        end
                    end
                    CMD_WR: begin
                        {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_WR;
                        o_S_A  <= {i_G_DQM[3], i_G_DQM[2],    // beat-0 byte strobes on A[12:11]
                                   i_G_A[10], 1'b0, i_G_A[7:0], 1'b0};
                        o_S_BA <= i_G_BA;
                        o_S_DQ_O <= i_G_WDATA[31:16];         // hi half rides the command beat
                        o_S_DQ_OE <= 1'b1;
                        wr_beat2 <= 1'b1;
                        wr_lo    <= i_G_WDATA[15:0];
                        wr_lo_m  <= i_G_DQM[1:0];
                        if (i_G_A[10]) begin                  // auto-precharge (defensive)
                            ga_open[i_G_BA] <= 1'b0;
                            cool[i_G_BA] <= 3'd7;
                        end
                    end
                    CMD_PRE: begin                            // PRE / PALL (A10 forwarded)
                        {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_PRE;
                        o_S_A  <= {2'b00, i_G_A[10:0]};       // bit 11 dropped: precharge only
                        o_S_BA <= i_G_BA;                     // reads A10+BA, and A[12:11]=00
                        if (i_G_A[10]) begin                  // keeps the DQM pins quiet
                            ga_open <= 4'b0;                  // PALL closes everything,
                            for (sbi = 0; sbi < 4; sbi = sbi + 1)
                                cool[sbi] <= 3'd3;            // including a live pair row:
                            if (m_open) begin
                                if (m_cnt >= 3'd4) m_credit;  // tRAS met - still counts
                                else begin                    // window rules make this
                                    m_open <= 1'b0;           // unreachable; tripwire only
                                    $display("[CV1k_sdram_control] ERROR: grid PALL under young maintenance row @%0t", $time);
                                end
                            end
                        end
                        else begin
                            ga_open[i_G_BA] <= 1'b0;
                            cool[i_G_BA] <= 3'd3;
                            if (m_open && (i_G_BA == m_bank)) begin
                                if (m_cnt >= 3'd4) m_credit;
                                else begin
                                    m_open <= 1'b0;
                                    $display("[CV1k_sdram_control] ERROR: grid PRE under young maintenance row @%0t", $time);
                                end
                            end
                        end
                    end
                    CMD_REF: begin
                        {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_REF;
                        ref_hold <= 4'd8;                     // tRFC guard (63 ns < 8 edges)
                        if (m_open) begin                     // BSC PALLs before REF, so a
                            m_open <= 1'b0;                   // live pair here is a bug
                            $display("[CV1k_sdram_control] ERROR: grid REF with maintenance row open @%0t", $time);
                        end
                    end
                    CMD_MRS: ;                                // acked, not forwarded (chip runs BL2/CL2)
                    default: ;
                endcase
            end
        end
        //--------------------------------------------------------------
        else begin
            // slot B: engines (mutually exclusive by construction - NOR runs
            // only inside CS0 cycles, the loader only while the CPU is reset)
            case (nst)
                NS_ACT: begin
                    {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_ACT;
                    o_S_A  <= {2'b10, nor_addr[20:10]};       // NOR window rows 0x1000-0x17FF
                    o_S_BA <= 2'b00;
                    ncnt <= 2'd0; nst <= NS_RCD;
                    act_gap <= 2'd2;
                    if (m_open && (m_bank == 2'd0))           // window sizing keeps pairs
                        $display("[CV1k_sdram_control] ERROR: NOR ACT with bank-0 maintenance row open @%0t", $time);
                end
                NS_RD: begin
                    {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_RD;
                    o_S_A  <= {2'b00, 1'b1, nor_addr[9:0]};   // A10: auto-precharge
                    o_S_BA <= 2'b00;
                    rdn_sh <= {rdn_sh[3:0], 1'b1};            // whole-vector (see rdg_sh note)
                    nst <= NS_CAP;
                end
                default: begin
                    case (lst)
                        LS_ACT: begin        // halfword latched by construction once LS_ACT is reached
                            {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_ACT;
                            o_S_A  <= {2'b10, ld_a[20:10]};
                            o_S_BA <= 2'b00;
                            lcnt <= 3'd0; lst <= LS_RCD;
                        end
                        LS_WR: begin
                            {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_WR;
                            o_S_A  <= {2'b00, 1'b1, ld_a[9:0]};   // unmasked beat 0, auto-precharge
                            o_S_BA <= 2'b00;
                            o_S_DQ_O <= ld_hw;
                            o_S_DQ_OE <= 1'b1;
                            ld_beat2 <= 1'b1;                 // beat 1 masked next edge
                            lcnt <= 3'd0; lst <= LS_DAL;
                        end
                        default: begin
                            //------------------------------------------
                            // refresh scheduler (lowest priority): one
                            // hidden ACT+PRE pair at a time on leftover
                            // slot-B edges, section 6.2 window rules
                            //------------------------------------------
                            if (m_open) begin
                                // tRAS: our ACT hit the chip at fire+1; a PRE
                                // here lands >= 5 chip clocks after it once
                                // m_cnt (which starts the edge AFTER the
                                // fire) reads 4.  A10=0, A[12:11]=00.
                                if (m_cnt >= 3'd4) begin
                                    {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_PRE;
                                    o_S_A  <= 13'h0;
                                    o_S_BA <= m_bank;
                                    m_credit;
                                end
                            end
                            else if (sel_v && (ref_hold == 4'd0)
                                           && (act_gap == 2'd0)
                                           && !ld_go) begin
                                // !ld_go: a pending loader halfword wins the
                                // slot (w_dl windows only - ld_go is never
                                // set outside a download); the pair waits
                                {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_ACT;
                                o_S_A  <= sel_row;
                                o_S_BA <= sel_bank;
                                m_open <= 1'b1;
                                m_bank <= sel_bank;
                                m_hi   <= sel_hi;
                                m_cnt  <= 3'd0;
                                act_gap <= 2'd2;
                            end
                        end
                    endcase
                end
            endcase
        end

        //--------------------------------------------------------------
        // NOR engine bookkeeping (counters run on every edge)
        //--------------------------------------------------------------
        if (init_done) begin
            case (nst)
                NS_IDLE: if (nor_read_req && (!nor_rdy || (nor_addr != i_N_A))) begin
                    nor_rdy  <= 1'b0;
                    nor_addr <= i_N_A;
                    nst <= NS_ACT;
                end
                NS_RCD: begin                                 // >= 3 edges ACT->READ (tRCD 21 ns)
                    ncnt <= ncnt + 2'd1;
                    if (ncnt >= 2'd2) nst <= NS_RD;
                end
                NS_CAP: if (rdn_sh[1]) begin                  // CL2 beat 0 = the addressed halfword (dq_n @drive-edge-.5)
                    nor_data <= dq_n;
                    nor_rdy  <= 1'b1;
                    nst <= NS_IDLE;
                end
                default: ;
            endcase
        end

        //--------------------------------------------------------------
        // loader engine bookkeeping
        //--------------------------------------------------------------
        case (lst)
            // !m_open (H7b.8): hold the loader out while a maintenance
            // pair is open (w_dl windows); o_IOCTL_WAIT stalls the HPS
            // for those few edges.  The pair-open side blocks on ld_go,
            // so the two can never start together.
            // !m_cool: one extra edge after a maintenance pair closes -- the
            // pair's PRE and the loader's ACT can hit the same bank, and a
            // release on the very next edge puts them 2 edges (19.5 ns)
            // apart, under tRP 21 ns (H7b.8 matrix cell C find: determin-
            // istic ld_go-held-behind-a-pair collision at the u4 phase)
            LS_IDLE: if (ld_go && init_done && !m_open && !m_cool) lst <= LS_ACT;
            LS_RCD:  begin                                    // >= 3 edges ACT->WRITE
                lcnt <= lcnt + 3'd1;
                if (lcnt >= 3'd2) lst <= LS_WR;
            end
            LS_DAL:  begin                                    // tDAL: data-in to next ACT
                lcnt <= lcnt + 3'd1;
                if (lcnt >= 3'd5) lst <= LS_IDLE;
            end
            default: ;
        endcase
    end
end

//------------------------------------------------------------------
// optional debug (+pumpdbg): init completion + NOR engine trace
//------------------------------------------------------------------
`ifndef SYNTHESIS
integer pdbg = 0;
integer pdbg_n = 0;
reg     pdbg_init_d = 1'b0;
initial if ($test$plusargs("pumpdbg")) pdbg = 1;
always @(posedge i_CLK) if (pdbg != 0) begin
    if (init_done && !pdbg_init_d) $display("[pump] init done @%0t (ist=%0d)", $time, ist);
    pdbg_init_d <= init_done;
    if (!i_G_CS_n && (pdbg_n < 200)) begin
        $display("[pump] t=%0t CS3 cmd(RAS/CAS/WE)=%b%b%b A=%03x BA=%b DQM=%b pcen_d=%b",
                 $time, i_G_RAS_n, i_G_CAS_n, i_G_WE_n, i_G_A, i_G_BA, i_G_DQM, pcen_d);
        pdbg_n = pdbg_n + 1;
    end
    if ((|rdg_sh) && (pdbg_n < 200)) begin
        $display("[pump] t=%0t GRD rdg=%06b dq=%04x hi=%04x w=%08x oe=%b",
                 $time, rdg_sh, i_S_DQ_I, rd_hi, rd_word, o_G_RDATA_OE);
        pdbg_n = pdbg_n + 1;
    end
    if ((nst != NS_IDLE) || (nor_read_req && !nor_rdy)) begin
        if (pdbg_n < 120) begin
            $display("[pump] t=%0t nst=%0d req=%b A=%06x nor_a=%06x rdn=%05b dq=%04x cmd=%b%b%b%b SA=%04x rdy=%b dat=%04x",
                     $time, nst, nor_read_req, {i_N_A,1'b0}, {nor_addr,1'b0},
                     rdn_sh, i_S_DQ_I, o_S_nCS, o_S_nRAS, o_S_nCAS, o_S_nWE,
                     o_S_A, nor_rdy, nor_data);
            pdbg_n = pdbg_n + 1;
        end
    end
end
`endif

endmodule
`default_nettype wire
