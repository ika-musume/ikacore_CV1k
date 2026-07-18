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
//   read    : every grid CAS is GEARED (sect.5.4 of the doc): the module
//             READ fires one fast edge before its pin decode, beats land in
//             dq_n at fire+1.5/fire+2.5 and in rd_hi_e/rd_lo at fire+2/+3,
//             and the serve window spans the BSC capture at pin-CAS+4 = the
//             identical edge the 32-bit part delivered the word.
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
//
// H7b.8d output-stage restructure (timing closure; ZERO behavior change -
// proven by the translate_off shadow oracle at the bottom of this file,
// which replicates the pre-8d priority chain and $fatals on any pad
// mismatch, every edge of every run):
//   * ONE fire-select plane (f_i*/arm_* wires) encodes every arm's fire
//     condition once, one-hot by construction; the old serial else-chain
//     is gone from the pad D-cones.
//   * The pad registers load from a flat AND-OR stage whose data legs
//     are registers (pr_*/nor_addr/ld_*/wr_lo/p_*/m_bank) or the grid
//     pins (slot-A translate + write strobes).  The pin legs are
//     IRREDUCIBLE: the BSC dispatches with zero pin warning, and with
//     tRCD = 21 ns > 2 fast edges (19.5 ns) a slot-A ACT delayed one
//     more edge would violate tRCD against its geared CAS (whose instant
//     is pinned by the CL2 grid serve) - so the translate must consume
//     the pins in the same single-c102-period transfer it always has.
//     That transfer's REAL budget is one c102 period (launch at the
//     CKIO-coincident edge, capture at the mid-CKIO c102 edge); the
//     project SDC carries the proof and the path-specific bound.
//   * All state/bookkeeping updates are byte-for-byte the old ones,
//     keyed on the same conditions via the shared arm wires.
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

    //------ SH-3 early-transaction sideband (docs/sh3_sideband.md 3+11) ----
    input  wire         i_CPU_RST_n,         // CPU manual reset: flush queue (A4)
    input  wire         i_SB_REQ,            // registered 1-cycle pulse per unit
    input  wire         i_SB_WR,
    input  wire [28:0]  i_SB_ADDR,           // [28:26] = CS area
    input  wire [1:0]   i_SB_SIZE,           // unused: reads serve full words
    input  wire         i_SB_BURST,
    input  wire [7:0]   i_BF_SB_COL,         // blit_fetch announce: train col
    input  wire [4:0]   i_BF_SB_LEN,         //   + CAS beats, valid under ACTV

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
reg [31:0] rd_word;                          // assembled word, held for the CKIO window

// sideband early gear (sect.11 / S4-S6): EVERY grid CAS is predicted and
// issues one fast edge BEFORE its pin decode (CPU ops from the strobe
// queue, blit trains from the blit_fetch announce - the oracle $fatals on
// any uncovered CAS), so both beats land in posedge registers a full
// period before the BSC capture edge.  The serve mux launches register-
// to-register only: the old live-DQ consistent-snapshot arm is DELETED,
// and dq_n fans exclusively to the pump-local capture regs below (+
// nor_data) - no half-period dq_n path leaves this module.
reg [2:0]  cap_sh;                           // early-issue capture pipeline
reg [15:0] rd_hi_e;                          // geared beat 0 (dq_n @fire+1.5)
reg [15:0] rd_lo;                            // geared beat 1 (dq_n @fire+2.5)

// DQ mid-window capture bank (H7b.8): every falling edge samples the pad.
// Each CL2 beat is stable across the falling edge inside its drive window
// (sim model drives beats for exactly one fast period; on silicon the
// SDRAM_CLK phase preset centers tAC..tOH around this edge), so dq_n holds
// beat k from E+k.5 to E+k+1.5 -- the SAME value every posedge consumer
// previously read live off the pad, but launched from a register.  This is
// the only pad-timed DQ endpoint; everything downstream (rd_hi_e/rd_lo/
// rd_word/nor_data) is register-to-register.  Value-identical in
// RTL sim by construction (FASTBOOT datum re-proven).
reg [15:0] dq_n;
always @(negedge i_CLK) dq_n <= i_S_DQ_I;

// The BSC latches i_D_I exactly at fast edge E+4 (CKIO E+2: i_BCEN && rd_lat,
// CL2 pipeline in bsc.sv). At that same edge this module commits its NBAs, so
// the sample may resolve pre- or post-commit; both mux arms carry the word
// either way: pre-commit rdg_sh[2]=1 selects the geared beat pair
// {rd_hi_e, rd_lo} (posedge registers loaded at fire+2/fire+3, a full
// period before the capture edge), post-commit rdg_sh[3]=1 selects the
// registered rd_word.  Both arms are register launches - the serve is
// register-to-register by construction, on every consumer.
assign o_G_RDATA    = rdg_sh[2] ? {rd_hi_e, rd_lo} : rd_word;
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
// SH-3 early-transaction sideband: strobe queue + engine-op READ shadow
// (docs/sh3_sideband.md sect.3, sect.6, sect.11 amendments A1-A5)
//
// One o_SB_REQ pulse = one committed external transaction unit (R1); every
// CS3 SDRAM engine op is strobed (R2; exemptions = CBR refresh, self-
// refresh, BREQ row-close PALL, MRS via SDMR - all pin-decoded, none is
// an ACT).  The queue is 2 deep (11.4 Q3: measured outstanding bound 1,
// +1 for a completing tail).  The head pops at the op's pin ACTV on a
// CPU-owned grid (i_BACK_n high - blit-tenure ACTs are bf_*-driven and
// must not pop).  A popped READ op arms the shadow predictor: with
// MCR 0x543C (t_rcd = 2, RASD = 0) the engine's CAS edges are fully
// deterministic after ACTV - READ0 exactly 2 CKIO on, then one CAS per
// CKIO, columns wrapping round the line from the missed word (bsc.sv
// E_RD), READA (A10) on the last beat only.  The shadow tracks those
// edges one fast cycle AHEAD of the pin decode; the translate_off oracle
// compares every predicted field against the pins and $fatals on any
// deviation, so the contract (and this mapping) is re-proven by every
// sim run.  Write ops pop but arm nothing: a yielded drain re-strobes at
// its resume (A3) and its beat count is not promised.  The CPU manual
// reset flushes queued-undispatched strobes (A4).
//
// addr bit map (AMX 0111, 8 MB x32, byte address a[22:2]):
//   bank = a[22:21]   row = a[20:10] (pin i_G_A[10:0] at ACTV)
//   col  = a[9:2]     (pin i_G_A[7:0] at CAS; a[3:2] = wrap pair)
//------------------------------------------------------------------
reg  [1:0]  sbq_v;                           // [0] = head
reg  [22:0] sbq0, sbq1;                      // {wr, burst, a[22:2]}
reg         pr_v;                            // READ op/train in flight (armed)
reg         pr_lin;                          // 1 = blit train (linear col, no AP)
reg  [2:0]  pr_dly;                          // fast edges to the next CAS decode
reg  [4:0]  pr_left;                         // CAS edges remaining (CPU 4/1;
                                             //   blit 1..16)
reg  [5:0]  pr_colh;                         // a[9:4]
reg  [1:0]  pr_beat;                         // a[3:2]: CPU wraps the pair, blit
                                             //   carries into pr_colh
reg  [1:0]  pr_ba;                           // bank captured at the ACTV pop

wire        sb_cs3    = (i_SB_ADDR[28:26] == 3'b011);
wire        sb_push   = i_SB_REQ && sb_cs3;
wire [2:0]  g_cmd     = {i_G_RAS_n, i_G_CAS_n, i_G_WE_n};
wire        slotA_cpu = pcen_d && init_done && i_BACK_n && !i_G_CS_n;
wire        pop_act   = slotA_cpu && (g_cmd == CMD_ACT);
// blit-tenure ACTV: the announce fields ride the same pins (no queue)
wire        slotA_bf  = pcen_d && init_done && !i_BACK_n && !i_G_CS_n;
wire        pop_bact  = slotA_bf && (g_cmd == CMD_ACT);
// CAS decode edge (pr_dly counted out) / early-issue edge (one fast before)
wire        pr_due    = pr_v && (pr_dly == 3'd0) && pcen_d;
wire        pr_fire   = pr_v && (pr_dly == 3'd1) && !pcen_d;
wire [7:0]  pr_col    = {pr_colh, pr_beat};
wire        pr_ap     = !pr_lin && (pr_left == 5'd1);  // READA on the CPU last
                                                       // beat; blit rows close
                                                       // by explicit PALL
// does the slot-A decode issue a module command this edge?  (CAS reads:
// never - the geared issue went out on the previous filler; MRS: acked,
// never forwarded)
wire        slotA_cmd = !i_G_CS_n && ((g_cmd == CMD_ACT) || (g_cmd == CMD_WR) ||
                                      (g_cmd == CMD_PRE) || (g_cmd == CMD_REF));

// queue next-state fold: pop frees the slot the same-edge push may take
// (no mixed bit/vector NBAs - see the rdg_sh note)
reg  [1:0]  sbq_v_nx;
reg  [22:0] sbq0_nx, sbq1_nx;
always @* begin
    sbq_v_nx = sbq_v; sbq0_nx = sbq0; sbq1_nx = sbq1;
    if (pop_act) begin
        sbq_v_nx = {1'b0, sbq_v[1]};
        sbq0_nx  = sbq1;
    end
    if (sb_push) begin
        if (!sbq_v_nx[0])      begin sbq0_nx = {i_SB_WR, i_SB_BURST, i_SB_ADDR[22:2]}; sbq_v_nx[0] = 1'b1; end
        else if (!sbq_v_nx[1]) begin sbq1_nx = {i_SB_WR, i_SB_BURST, i_SB_ADDR[22:2]}; sbq_v_nx[1] = 1'b1; end
        // overflow = A5 bound violated; trapped by the oracle below
    end
end

always @(posedge i_CLK) begin
    if (!i_RST_n || !i_CPU_RST_n) begin      // A4: manual reset drops the queue
        sbq_v <= 2'b00; sbq0 <= 23'h0; sbq1 <= 23'h0;
        pr_v <= 1'b0; pr_lin <= 1'b0; pr_dly <= 3'd0; pr_left <= 5'd0;
        pr_colh <= 6'd0; pr_beat <= 2'd0; pr_ba <= 2'd0;
    end else begin
        sbq_v <= sbq_v_nx; sbq0 <= sbq0_nx; sbq1 <= sbq1_nx;
        if (pr_v && (pr_dly != 3'd0)) pr_dly <= pr_dly - 3'd1;
        if (pr_due) begin                    // one CAS consumed at this decode
            pr_beat <= pr_beat + 2'd1;
            if (pr_lin && (pr_beat == 2'd3)) // blit runs are linear: carry out
                pr_colh <= pr_colh + 6'd1;   //   of the pair (never cross 1KB)
            pr_left <= pr_left - 5'd1;
            pr_dly  <= 3'd1;                 // next CAS one CKIO on
            if (pr_left == 5'd1) pr_v <= 1'b0;
        end
        if (pop_act && sbq_v[0] && !sbq0[22]) begin
            pr_v    <= 1'b1;                 // arm the shadow on a READ op
            pr_lin  <= 1'b0;
            pr_dly  <= 3'd3;                 // CAS decode at ACTV+2 CKIO (t_rcd)
            pr_left <= sbq0[21] ? 5'd4 : 5'd1;
            pr_colh <= sbq0[7:2];
            pr_beat <= sbq0[1:0];
            pr_ba   <= i_G_BA;
        end
        if (pop_bact) begin
            pr_v    <= 1'b1;                 // arm on the announced blit train
            pr_lin  <= 1'b1;
            pr_dly  <= 3'd3;                 // blit_fetch S_RCD = same 2-CKIO gap
            pr_left <= i_BF_SB_LEN;
            pr_colh <= i_BF_SB_COL[7:2];
            pr_beat <= i_BF_SB_COL[1:0];
            pr_ba   <= i_G_BA;
        end
    end
end

wire _unused_sb = &{1'b0, i_SB_SIZE, i_SB_ADDR[25:23], 1'b0};

// synthesis translate_off
// sideband oracle (sect.11.6 shape, our side of the contract): every
// deviation is a $fatal - the FASTBOOT datum, the refage replays and the
// H7b matrix re-prove R1/R2/R3/A2/A3/A5 + the address map on every run.
always @(posedge i_CLK) begin
    if (i_RST_n && i_CPU_RST_n && init_done) begin
        if (sb_push && (sbq_v == 2'b11) && !pop_act)
            $fatal(1, "[sb] strobe queue overflow (A5 bound) @%0t", $time);
        if (pop_act && !sbq_v[0])
            $fatal(1, "[sb] unstrobed CS3 ACTV (R2) @%0t", $time);
        if (pop_act && sbq_v[0] && (i_G_A[10:0] != sbq0[18:8]))
            $fatal(1, "[sb] ACTV row %03x != strobe row %03x @%0t",
                   i_G_A[10:0], sbq0[18:8], $time);
        if (pop_act && sbq_v[0] && (i_G_BA != sbq0[20:19]))
            $fatal(1, "[sb] ACTV bank %0d != strobe bank %0d @%0t",
                   i_G_BA, sbq0[20:19], $time);
        if (pop_act && pr_v)
            $fatal(1, "[sb] ACTV while a READ shadow is live @%0t", $time);
        if (pop_bact && pr_v)
            $fatal(1, "[sb] blit ACTV while a READ shadow is live @%0t", $time);
        if (pop_bact && (i_BF_SB_LEN == 5'd0))
            $fatal(1, "[sb] blit ACTV with a zero-length announce @%0t", $time);
        if (pr_due && !((pr_lin ? slotA_bf : slotA_cpu) && (g_cmd == CMD_RD)))
            $fatal(1, "[sb] predicted CAS edge has no grid READ @%0t", $time);
        if (pr_due && (i_G_A[7:0] != pr_col))
            $fatal(1, "[sb] CAS col %02x != predicted %02x @%0t",
                   i_G_A[7:0], pr_col, $time);
        if (pr_due && (i_G_A[10] != pr_ap))
            $fatal(1, "[sb] CAS A10 %b != predicted %b @%0t",
                   i_G_A[10], pr_ap, $time);
        if (pr_due && (i_G_BA != pr_ba))
            $fatal(1, "[sb] CAS bank %0d != ACTV bank %0d @%0t",
                   i_G_BA, pr_ba, $time);
        if ((slotA_cpu || slotA_bf) && (g_cmd == CMD_RD) && !pr_due)
            $fatal(1, "[sb] grid READ outside the shadow schedule @%0t", $time);
    end
end
// synthesis translate_on

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

// synthesis translate_off
// early-gear edge-ownership assert (companion to the sideband oracle above;
// placed here because wr/ld_beat2 are declared at this point): a predicted
// early CAS must never lose its filler edge to a pending write beat
always @(posedge i_CLK) begin
    if (i_RST_n && i_CPU_RST_n && init_done && pr_fire && (wr_beat2 || ld_beat2))
        $fatal(1, "[sb] early CAS edge stolen by a write beat @%0t", $time);
end
// synthesis translate_on

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

// H7b.8b pad register bank (jtcps CPS2 96 MHz shape): the argmax and its
// sweep-row lookup are PRE-REGISTERED one edge ahead of dispatch, so the
// pad registers' D cones end at p_* instead of walking cool/m_bank ->
// b_elig -> compare trees -> ptr mux in the dispatch edge (fit #4 c102
// worst path, 17.4 ns).  p_* refresh every edge from the same trees.
// The fire edge re-applies the window/guard terms live (same instants as
// before) and re-checks b_elig on the registered bank: the one-edge gap
// can hide a grid ACT (ga_open) or an AP close (cool reload), and the
// pre-reg pick may also be one edge stale after a cool expiry or a fresh
// tick - a fire can only slip later or land on a different owed row,
// never outside the proven windows.  Deficits/pointers of a registered
// pick cannot decay in the gap: m_credit needs m_open, m_open masks the
// bank in b_elig, and its cooldown blocks the re-check.
reg        p_hi_v, p_lo_v;
reg [1:0]  p_hi_b, p_lo_b;
reg [12:0] p_hi_row, p_lo_row;

wire        hi_pick  = w_any && p_hi_v && b_elig[p_hi_b];
wire        lo_pick  = w_lo  && p_lo_v && b_elig[p_lo_b];
wire        sel_v    = hi_pick || lo_pick;
wire        sel_hi   = hi_pick;
wire [1:0]  sel_bank = hi_pick ? p_hi_b : p_lo_b;
wire [12:0] sel_row  = hi_pick ? p_hi_row : p_lo_row;

// H7b.8d step 2: pre-registered pair-dispatch select (see the note at
// the arm_m* wires below)
reg q_mpre, q_mhi, q_mlo;
// H7b.8d step 3: pre-registered init fires - the live f_i* compares
// (32-bit icnt thresholds) fed the pad mux directly and were the last
// deep pad cone (fit-8d2 seed 3: icnt[*] -> o_S_nWE -3.0).  A multicycle
// would be UNSOUND (icnt moves every edge; a mid-transition compare
// sample could double-fire a REF inside tRFC), so the fires get the same
// exact next-value treatment as the pair dispatch.  State transitions
// keep the live f_i* (fabric-local, never the frontier).
reg q_ipall, q_iref, q_imrs;

//------------------------------------------------------------------
// H7b.8d fire-select plane: ONE encoding of every output arm's fire
// condition, shared by the flat pad stage below and the bookkeeping
// block.  These are exactly the old priority chain's arm conditions
// unrolled into mutually-exclusive one-hot terms (the chain's else-
// cascade becomes explicit !higher terms), so every command fires on
// the identical edge with the identical fields - the translate_off
// shadow oracle at the bottom of the file re-proves chain==flat on
// every edge of every sim run.  No decision, guard, or window term
// moved: the restructure is a pure re-expression of the pad D-cones
// so each IOE register sees ONE shallow AND-OR stage (register- or
// grid-pin-sourced data) instead of the serial arm cascade.
//------------------------------------------------------------------
// init sequencer fires (single encoding also drives ist/icnt/iref)
wire f_ipall = (ist == IS_WAIT) && (icnt >= INIT_WAIT);
wire f_iref  = ((ist == IS_PALL) && (icnt >= 32'd3)) ||
               ((ist == IS_REF)  && (icnt >= 32'd8) && (iref < 4'd8));
wire f_imrs  = (ist == IS_REF)  && (icnt >= 32'd8) && (iref >= 4'd8);
// post-init arm ladder (chain order: wr_beat2 > ld_beat2 > pr_fire >
// slot A > slot B engines > pair dispatch)
wire arm_wrb  = init_done && wr_beat2;
wire arm_ldb  = init_done && !wr_beat2 && ld_beat2;
wire arm_prf  = init_done && !wr_beat2 && !ld_beat2 && pr_fire;
wire slotA    = init_done && !wr_beat2 && !ld_beat2 && !pr_fire &&  pcen_d;
wire slotB    = init_done && !wr_beat2 && !ld_beat2 && !pr_fire && !pcen_d;
wire arm_tr     = slotA && slotA_cmd;        // grid translate (ACT/WR/PRE/REF)
wire arm_tr_act = arm_tr && (g_cmd == CMD_ACT);
wire arm_tr_wr  = arm_tr && (g_cmd == CMD_WR);
wire arm_tr_pre = arm_tr && (g_cmd == CMD_PRE);
wire arm_nact = slotB && (nst == NS_ACT);
wire arm_nrd  = slotB && (nst == NS_RD);
wire slotB_e  = slotB && (nst != NS_ACT) && (nst != NS_RD);
wire arm_lact = slotB_e && (lst == LS_ACT);
wire arm_lwr  = slotB_e && (lst == LS_WR);
// pair-dispatch site (both old m_dispatch call sites folded) + guards.
// H7b.8d step 2: the DEEP select cone (windows -> pick -> guards, all
// c102-register functions) is PRE-REGISTERED one edge ahead into
// q_mpre/q_mhi/q_mlo by the exact next-value plane below - NOT the
// H7b.8b stale-pick-plus-recheck shape: every term is computed for the
// fire edge from this edge's inputs (window _ff regs at the fire edge
// are the raw pins now; counters/FSMs/guards are deterministic one
// step ahead), so the fire decision is BIT-EXACT the old live one.
// The fire edge keeps only the single-FF ladder bits and the grid-pin
// term (irreducible, same reason as the translate arm) - one LUT.
// The old live formulas remain below as m_site/m_can_* for the
// translate_off q-vs-live assert and the shadow oracle.
wire m_gate_f = init_done && !wr_beat2 && !ld_beat2 && !pr_fire &&
                (pcen_d ? !slotA_cmd : 1'b1);
wire arm_mpre = q_mpre && m_gate_f;
wire arm_mhi  = q_mhi  && m_gate_f;          // pad data split: p_* regs feed
wire arm_mlo  = q_mlo  && m_gate_f;          //   the AND-OR stage directly
wire arm_mact = arm_mhi || arm_mlo;
// oracle references (translate_off consumers only; pruned in synthesis)
wire m_site   = (slotA && !slotA_cmd) ||
                (slotB_e && (lst != LS_ACT) && (lst != LS_WR));
wire m_can_pre = m_open && (m_cnt >= 3'd4);
wire m_can_act = !m_open && sel_v && (ref_hold == 4'd0)
                         && (act_gap == 2'd0) && !ld_go;

integer sbi;                                 // scheduler bookkeeping loop var

//------------------------------------------------------------------
// H7b.8d step 2 - exact next-value plane for the pair-dispatch select.
// Computes, at this edge, the value every deep select term will hold
// DURING the next edge's evaluation window (i.e. the values the old
// live cone would read at the fire edge):
//   * window _ff regs at the fire edge = the RAW PINS now (back/bwin/
//     ordn/cs0n/dl samplers reload every edge);
//   * counters (ord/cs0, ref_hold, act_gap, m_cnt, cool) and FSMs
//     (nst/lst, beats, rdg/rdn tails) are deterministic one step ahead
//     - their update expressions are replicated 1:1, keyed on the SAME
//     arm wires the bookkeeping block uses this edge;
//   * the argmax trees feed p_* at this edge, so the pick guards pair
//     tree combs (hi_d/hi_b/lo_d/lo_b) with next-edge b_elig - exactly
//     the generation the old cone paired at the fire edge.
// Every mismatch against the live formulas is a translate_off $fatal
// (q-vs-live assert below) on top of the pad shadow oracle.
//------------------------------------------------------------------
wire rd_now      = slotA && !i_G_CS_n && (g_cmd == CMD_RD);
wire ref_now     = slotA && !i_G_CS_n && (g_cmd == CMD_REF);
wire pre_now     = slotA && !i_G_CS_n && (g_cmd == CMD_PRE);
wire ap_close_now= (rd_now || arm_tr_wr) && i_G_A[10];

wire wr_beat2_nx = arm_tr_wr || (wr_beat2 && !arm_wrb);
wire ld_beat2_nx = arm_lwr   || (ld_beat2 && !arm_ldb);

// nst / lst one step ahead (fire-site + bookkeeping-site transitions)
reg [2:0] nst_nx;
always @* begin
    nst_nx = nst;
    if      (arm_nact) nst_nx = NS_RCD;
    else if (arm_nrd)  nst_nx = NS_CAP;
    else if (init_done) begin
        case (nst)
            NS_IDLE: if (nor_read_req && (!nor_rdy || (nor_addr != i_N_A))) nst_nx = NS_ACT;
            NS_RCD:  if (ncnt >= 2'd2)  nst_nx = NS_RD;
            NS_CAP:  if (rdn_sh[1])     nst_nx = NS_IDLE;
            default: ;
        endcase
    end
end
reg [2:0] lst_nx;
always @* begin
    lst_nx = lst;
    if      (arm_lact) lst_nx = LS_RCD;
    else if (arm_lwr)  lst_nx = LS_DAL;
    else begin
        case (lst)
            LS_IDLE: if (ld_go && init_done && !m_open && !m_cool) lst_nx = LS_ACT;
            LS_RCD:  if (lcnt >= 3'd2) lst_nx = LS_WR;
            LS_DAL:  if (lcnt >= 3'd5) lst_nx = LS_IDLE;
            default: ;
        endcase
    end
end

wire init_done_nx = init_done || ((ist == IS_MRS) && (icnt >= 32'd3));
wire ld_go_nx     = (i_IOCTL_DOWNLOAD && i_IOCTL_WR)
                    ? ((i_IOCTL_ADDR[0] && (i_IOCTL_ADDR < 27'h040_0000)) ? 1'b1 : ld_go)
                    : ((lst != LS_IDLE) ? 1'b0 : ld_go);
wire m_open_nx    = arm_mact ? 1'b1
                  : (arm_mpre || ref_now ||
                     (pre_now && (i_G_A[10] ? m_open : (i_G_BA == m_bank)))) ? 1'b0
                  : m_open;
wire [2:0] m_cnt_nx    = arm_mact ? 3'd0
                       : ((m_open && (m_cnt != 3'd7)) ? m_cnt + 3'd1 : m_cnt);
wire [1:0] m_bank_nx   = arm_mact ? (q_mhi ? p_hi_b : p_lo_b) : m_bank;
wire [3:0] ref_hold_nx = ref_now ? 4'd8
                       : ((ref_hold != 4'd0) ? ref_hold - 4'd1 : 4'd0);
wire [1:0] act_gap_nx  = (arm_tr_act || arm_nact || arm_mact) ? 2'd2
                       : ((act_gap != 2'd0) ? act_gap - 2'd1 : 2'd0);

// per-bank ga_open / cool one step ahead (grid parse + credit reloads,
// override order identical to the bookkeeping NBA order)
reg [3:0] ga_open_nx;
reg [2:0] cool_nx [0:3];
always @* begin
    ga_open_nx = ga_open;
    for (sbi = 0; sbi < 4; sbi = sbi + 1)
        cool_nx[sbi] = (cool[sbi] != 3'd0) ? cool[sbi] - 3'd1 : 3'd0;
    if (arm_tr_act)                 ga_open_nx[i_G_BA] = 1'b1;
    if (ap_close_now) begin         // RD/WR auto-precharge close
        ga_open_nx[i_G_BA] = 1'b0;
        cool_nx[i_G_BA]    = 3'd7;
    end
    if (pre_now) begin
        if (i_G_A[10]) begin        // PALL
            ga_open_nx = 4'b0;
            for (sbi = 0; sbi < 4; sbi = sbi + 1) cool_nx[sbi] = 3'd3;
        end else begin
            ga_open_nx[i_G_BA] = 1'b0;
            cool_nx[i_G_BA]    = 3'd3;
        end
    end
    if (arm_mpre) cool_nx[m_bank] = 3'd3;    // pair-close credit
end

wire [3:0] b_elig_nx = {init_done_nx && !ga_open_nx[3] && (cool_nx[3] == 3'd0) && !(m_open_nx && (m_bank_nx == 2'd3)),
                        init_done_nx && !ga_open_nx[2] && (cool_nx[2] == 3'd0) && !(m_open_nx && (m_bank_nx == 2'd2)),
                        init_done_nx && !ga_open_nx[1] && (cool_nx[1] == 3'd0) && !(m_open_nx && (m_bank_nx == 2'd1)),
                        init_done_nx && !ga_open_nx[0] && (cool_nx[0] == 3'd0) && !(m_open_nx && (m_bank_nx == 2'd0))};

// windows one step ahead (fire-edge _ff samplers = raw pins now)
wire [4:0] ord_cnt_nx = i_ORD_CS_n ? 5'd0 : ((ord_cnt == 5'h1F) ? ord_cnt : ord_cnt + 5'd1);
wire [4:0] cs0_cnt_nx = i_N_CS_n   ? 5'd0 : ((cs0_cnt == 5'h1F) ? cs0_cnt : cs0_cnt + 5'd1);
wire eng_quiet_nx = (nst_nx == NS_IDLE) && ({rdn_sh[3:0], arm_nrd} == 5'b0)
                    && (lst_nx == LS_IDLE) && !ld_beat2_nx && !wr_beat2_nx
                    && ({rdg_sh[1:0], rd_now} == 3'b0);
wire w_blit_nx = i_BACK_n ? 1'b0 : i_BLIT_WIN;
wire w_cs0_nx  = !i_N_CS_n   && (cs0_cnt_nx >= 5'd10) && (cs0_cnt_nx <= 5'd14);
wire w_ord_nx  = !i_ORD_CS_n && (ord_cnt_nx >= 5'd2)  && (ord_cnt_nx <= 5'd6);
wire w_dl_nx   = i_IOCTL_DOWNLOAD && eng_quiet_nx;
wire w_any_nx  = (w_cs0_nx || w_ord_nx || w_dl_nx) && eng_quiet_nx;
wire w_lo_nx   = w_blit_nx || w_any_nx;

// pick guards on the SAME tree generation p_* is loading this edge
wire hi_pick_nx = w_any_nx && (hi_d != 13'd0) && b_elig_nx[hi_b];
wire lo_pick_nx = w_lo_nx  && (lo_d != 13'd0) && b_elig_nx[lo_b];
wire sel_v_nx   = hi_pick_nx || lo_pick_nx;

wire mpre_core_nx = m_open_nx && (m_cnt_nx >= 3'd4);
wire mact_core_nx = !m_open_nx && sel_v_nx && (ref_hold_nx == 4'd0)
                    && (act_gap_nx == 2'd0) && !ld_go_nx;
// slot-B parity folds the engine decode; slot-A parity has none (the
// grid-pin term stays live in m_gate_f - it cannot exist one edge early)
wire site_ok_nx = i_CKIO_PCEN ? 1'b1
                : ((nst_nx != NS_ACT) && (nst_nx != NS_RD) &&
                   (lst_nx != LS_ACT) && (lst_nx != LS_WR));

// init sequencer one step ahead (step 3): ist/icnt/iref updates
// replicated 1:1 (the live f_i* chain in the bookkeeping block), fires
// evaluated on the _nx values = the exact next-edge live evaluation
reg [2:0]  ist_nx;
reg [31:0] icnt_nx;
reg [3:0]  iref_nx;
always @* begin
    ist_nx = ist; iref_nx = iref;
    icnt_nx = init_done ? icnt : icnt + 32'd1;
    if (!init_done) begin
        if (f_ipall)     begin icnt_nx = 32'd0; ist_nx = IS_PALL; end
        else if (f_iref) begin
            icnt_nx = 32'd0;
            if (ist == IS_PALL) begin iref_nx = 4'd1; ist_nx = IS_REF; end
            else                iref_nx = iref + 4'd1;
        end
        else if (f_imrs) begin icnt_nx = 32'd0; ist_nx = IS_MRS; end
        else if ((ist == IS_MRS) && (icnt >= 32'd3)) ist_nx = IS_DONE;
    end
end

always @(posedge i_CLK) begin
    if (!i_RST_n) begin
        q_mpre <= 1'b0; q_mhi <= 1'b0; q_mlo <= 1'b0;
        q_ipall <= 1'b0; q_iref <= 1'b0; q_imrs <= 1'b0;
    end else begin
        q_mpre <= site_ok_nx && mpre_core_nx;
        q_mhi  <= site_ok_nx && mact_core_nx &&  hi_pick_nx;
        q_mlo  <= site_ok_nx && mact_core_nx && !hi_pick_nx;
        q_ipall <= (ist_nx == IS_WAIT) && (icnt_nx >= INIT_WAIT);
        q_iref  <= ((ist_nx == IS_PALL) && (icnt_nx >= 32'd3)) ||
                   ((ist_nx == IS_REF)  && (icnt_nx >= 32'd8) && (iref_nx < 4'd8));
        q_imrs  <= (ist_nx == IS_REF)  && (icnt_nx >= 32'd8) && (iref_nx >= 4'd8);
    end
end

// synthesis translate_off
// q-vs-live assert: the pre-registered selects must equal the old live
// cones at every fire evaluation (this is the step-2/3 exactness proof,
// on top of the pad shadow oracle)
always @(posedge i_CLK) if (i_RST_n) begin
    if (arm_mpre != (m_site && m_can_pre))
        $fatal(1, "[pad8d] q_mpre %b != live %b @%0t", arm_mpre, m_site && m_can_pre, $time);
    if (arm_mact != (m_site && m_can_act))
        $fatal(1, "[pad8d] q_mact %b != live %b @%0t", arm_mact, m_site && m_can_act, $time);
    if (arm_mact && (arm_mhi != (m_site && m_can_act && hi_pick)))
        $fatal(1, "[pad8d] q_mhi %b != live %b @%0t", arm_mhi, hi_pick, $time);
    if (q_ipall != f_ipall)
        $fatal(1, "[pad8d] q_ipall %b != live %b @%0t", q_ipall, f_ipall, $time);
    if (q_iref != f_iref)
        $fatal(1, "[pad8d] q_iref %b != live %b @%0t", q_iref, f_iref, $time);
    if (q_imrs != f_imrs)
        $fatal(1, "[pad8d] q_imrs %b != live %b @%0t", q_imrs, f_imrs, $time);
end
// synthesis translate_on


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

// hidden-refresh pair dispatch (section 6.2 window rules): one ACT+PRE at
// a time, guards identical from every call site.  Historically this ran
// on leftover slot-B edges only; the sideband early gear (S5/S6) moved
// mid-train CAS issue onto the fillers, so the dispatch ALSO runs on
// module-idle slot-A edges (geared CAS decodes, MRS acks, CS-quiet
// cells) - the supply edges the gear vacated.  Every window class stays
// CKIO-granular, and every spacing guard (ref_hold, act_gap, m_cnt tRAS,
// cool tRP/tRC) counts fast edges, so the phase move keeps the proofs.
// H7b.8d: the old m_dispatch task is dissolved - both call sites fold
// into the arm_mpre/arm_mact fire-select wires above (guards verbatim:
// PRE = m_open && m_cnt>=4, tRAS met [our ACT hit the chip at fire+1, a
// PRE here lands >= 5 chip clocks after it once m_cnt reads 4]; ACT =
// sel_v && !ref_hold && !act_gap && !ld_go [a pending loader halfword
// wins the slot - ld_go is never set outside a download]).  The pad
// stage issues the commands; the bookkeeping lives in the main block.

//------------------------------------------------------------------
// main state/bookkeeping sequencer (H7b.8d: the pad registers moved to
// the flat one-hot stage below - this block owns every OTHER register).
// One command per fast edge, arm conditions = the fire-select plane
// above (identical priority: init > wr beat 2 > ld beat 2 > geared CAS
// > grid slot A > engines (slot B) > pair dispatch).
//------------------------------------------------------------------
always @(posedge i_CLK) begin
    if (!i_RST_n) begin
        rdg_sh <= 6'b0; rdn_sh <= 5'b0;
        rd_word <= 32'h0;
        cap_sh <= 3'b0;
        rd_hi_e <= 16'h0; rd_lo <= 16'h0;
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
        p_hi_v <= 1'b0; p_lo_v <= 1'b0;
        p_hi_b <= 2'd0; p_lo_b <= 2'd0;
        p_hi_row <= 13'h0; p_lo_row <= 13'h0;
        for (sbi = 0; sbi < 4; sbi = sbi + 1) begin
            cool[sbi]   <= 3'd0;
            ptr_lo[sbi] <= 11'd0;
            ptr_hi[sbi] <= 13'h0800;
            def_lo[sbi] <= DEF_CAP_LO;       // full backlog: first pass runs
            def_hi[sbi] <= (sbi == 0) ? DEF_CAP_HI0 : DEF_CAP_HI;  // at window rate
        end
    end else begin
        // capture pipelines shift every edge.  With E = the CKIO edge opening
        // the grid READ cycle: issue @E+1, chip reg @E+2, CL2 beats on DQ at
        // their drive edges E+3/E+4; the word reaches o_G_RDATA at E+4 via
        // the consistent-snapshot mux - the BSC's hard capture edge
        // (i_BCEN && rd_lat = CKIO E+2 = fast E+4, see doc section 5.1).
        rdg_sh <= {rdg_sh[4:0], 1'b0};
        rdn_sh <= {rdn_sh[3:0], 1'b0};
        cap_sh <= {cap_sh[1:0], 1'b0};
        if (cap_sh[1]) rd_hi_e <= dq_n;                   // geared beat 0 (fire+1.5)
        if (cap_sh[2]) rd_lo   <= dq_n;                   // geared beat 1 (fire+2.5)
        if (rdg_sh[2]) rd_word <= {rd_hi_e, rd_lo};

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

        // pre-registered maintenance pick (see the p_* note above): the
        // trees see this edge's pre-commit state, the fire edge re-checks
        p_hi_v   <= (hi_d != 13'd0);
        p_hi_b   <= hi_b;
        p_hi_row <= ptr_hi[hi_b];
        p_lo_v   <= (lo_d != 13'd0);
        p_lo_b   <= lo_b;
        p_lo_row <= {2'b00, ptr_lo[lo_b]};

        //--------------------------------------------------------------
        if (!init_done) begin
            // JEDEC init for chip 0 while the whole system is in reset
            // (commands issue from the pad stage on the f_i* fires; the
            // icnt-pace/tRFC/tMRD spacing is the f_i* thresholds verbatim)
            icnt <= icnt + 32'd1;
            if (f_ipall)      begin icnt <= 32'd0; ist <= IS_PALL; end
            else if (f_iref)  begin
                icnt <= 32'd0;
                if (ist == IS_PALL) begin iref <= 4'd1; ist <= IS_REF; end
                else                iref <= iref + 4'd1;      // tRFC 63 ns < 8 edges
            end
            else if (f_imrs)  begin icnt <= 32'd0; ist <= IS_MRS; end
            else if ((ist == IS_MRS) && (icnt >= 32'd3)) ist <= IS_DONE;  // tMRD 2
        end
        //--------------------------------------------------------------
        // grid write lo half (arm_wrb: NOP + data + byte strobes on
        // A[12:11] from the pad stage; never collides with slot A - it
        // always lands on the filler edge)
        if (arm_wrb) wr_beat2 <= 1'b0;
        //--------------------------------------------------------------
        // loader write beat 2 (arm_ldb: fully masked single-halfword
        // write from the pad stage).  Outranks slot A: the beat can land
        // on a pcen edge, and the grid is structurally silent during
        // download (CPU held in reset).
        if (arm_ldb) ld_beat2 <= 1'b0;
        //--------------------------------------------------------------
        // sideband early gear (sect.11, arm_prf): the strobed op's next
        // CAS, one fast edge before its pin decode - command from the
        // pad stage.  The filler edge is structurally free mid-op (no
        // window class opens during a CPU SDRAM transaction, wr/ld beats
        // can't reach into a read train - oracle-asserted).  AP
        // bookkeeping stays at the pin decode.
        if (arm_prf) cap_sh <= {cap_sh[1:0], 1'b1};
        //--------------------------------------------------------------
        if (slotA) begin
            // slot A: parse whatever the grid drove this CKIO cycle (the
            // ACT/WR/PRE/REF translate itself issues from the pad stage)
            if (!i_G_CS_n) begin
                case ({i_G_RAS_n, i_G_CAS_n, i_G_WE_n})
                    CMD_ACT: begin                            // 12-bit row pass-through
                        ga_open[i_G_BA] <= 1'b1;              // (B board drives bit 11 = 0)
                        act_gap <= 2'd2;
                        if (m_open && (i_G_BA == m_bank))
                            $display("[CV1k_sdram_control] ERROR: grid ACT bank %0d over open maintenance row @%0t",
                                     i_G_BA, $time);
                    end
                    CMD_RD: begin                             // one grid CAS -> one BL2 pair
                        // sideband-geared (oracle-enforced): the module CAS
                        // went out one fast edge ago on the filler (pr_fire);
                        // this edge runs the serve/OE window + AP bookkeeping
                        // - whole-vector re-assign (a bit-select NBA after
                        // the vector-shift NBA is silently dropped)
                        rdg_sh  <= {rdg_sh[4:0], 1'b1};
                        if (i_G_A[10]) begin                  // auto-precharge: the normal
                            ga_open[i_G_BA] <= 1'b0;          // close (MCR 0x543C RASD=0 -
                            cool[i_G_BA] <= 3'd7;             // every op ends with AP)
                        end
                    end
                    CMD_WR: begin
                        // hi half rides the command beat (pad stage);
                        // beat-0 byte strobes on A[12:11]
                        wr_beat2 <= 1'b1;
                        wr_lo    <= i_G_WDATA[15:0];
                        wr_lo_m  <= i_G_DQM[1:0];
                        if (i_G_A[10]) begin                  // auto-precharge (defensive)
                            ga_open[i_G_BA] <= 1'b0;
                            cool[i_G_BA] <= 3'd7;
                        end
                    end
                    CMD_PRE: begin                            // PRE / PALL (A10 forwarded,
                                                              // bit 11 dropped: precharge
                                                              // reads A10+BA, A[12:11]=00)
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
        // slot B: engines (mutually exclusive by construction - NOR runs
        // only inside CS0 cycles, the loader only while the CPU is reset);
        // commands issue from the pad stage on the arm_* fires
        if (arm_nact) begin                                   // NOR rows 0x1000-0x17FF
            ncnt <= 2'd0; nst <= NS_RCD;
            act_gap <= 2'd2;
            if (m_open && (m_bank == 2'd0))                   // window sizing keeps pairs
                $display("[CV1k_sdram_control] ERROR: NOR ACT with bank-0 maintenance row open @%0t", $time);
        end
        if (arm_nrd) begin                                    // A10: auto-precharge
            rdn_sh <= {rdn_sh[3:0], 1'b1};                    // whole-vector (see rdg_sh note)
            nst <= NS_CAP;
        end
        if (arm_lact) begin  // halfword latched by construction once LS_ACT is reached
            lcnt <= 3'd0; lst <= LS_RCD;
        end
        if (arm_lwr) begin   // unmasked beat 0, auto-precharge
            ld_beat2 <= 1'b1;                                 // beat 1 masked next edge
            lcnt <= 3'd0; lst <= LS_DAL;
        end
        //--------------------------------------------------------------
        // hidden-refresh pair dispatch bookkeeping (arm_mpre/arm_mact =
        // both old call sites: module-idle slot-A edges + leftover slot-B
        // edges, section 6.2 rules; commands from the pad stage)
        //--------------------------------------------------------------
        if (arm_mpre) m_credit;                               // A10=0, A[12:11]=00
        if (arm_mact) begin                                   // bank/region from the
            m_open <= 1'b1;                                   // q-selected p_* pick
            m_bank <= q_mhi ? p_hi_b : p_lo_b;                // (single source with
            m_hi   <= q_mhi;                                  // the pad stage)
            m_cnt  <= 3'd0;
            act_gap <= 2'd2;
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
// H7b.8d flat pad stage: the IOE registers, one shallow AND-OR each.
//
// Every arm's data is a register bank (pr_*/nor_addr/ld_*/wr_lo/p_*/
// m_bank) or the grid pins (arm_tr / arm_wrb strobes - irreducible:
// the BSC dispatches with zero pin warning and tRCD=21 ns pins the
// slot-A ACT instant to pins+1 fast edge, so a pre-registered plan
// stage would either shift the chip schedule [breaks tRCD against the
// geared CAS, proven H7b.8d analysis] or consume a surface that does
// not exist yet.  The pin arcs run launch(CKIO-coincident c153/c102
// edge) -> capture(next c102 edge) = one full c102 period; the SDC
// carries the proof).  The one-hot selects are the fire-select plane -
// no serial arm cascade reaches any pad D input.
// CMD_MRS is 3'b000: its "term" is the absence of 1-contributions
// while cmd_any holds off the NOP fold - do not add a term for it.
//------------------------------------------------------------------
wire cmd_pre_t = q_ipall | arm_mpre;                          // (q_i* = step-3 pre-
wire cmd_ref_t = q_iref;                                      //  registered init fires)
wire cmd_rd_t  = arm_prf | arm_nrd;
wire cmd_act_t = arm_nact | arm_lact | arm_mhi | arm_mlo;
wire cmd_wr_t  = arm_lwr;
wire cmd_any   = cmd_pre_t | cmd_ref_t | q_imrs | cmd_rd_t | cmd_act_t
               | cmd_wr_t | arm_tr;

wire [12:0] pad_a =
      ({13{q_ipall  }} & 13'h0400)                            // A10: precharge all
    | ({13{q_imrs   }} & MRS_BL2_CL2)
    | ({13{arm_wrb  }} & {wr_lo_m[1], wr_lo_m[0], 11'h0})
    | ({13{arm_ldb  }} & {2'b11, 11'h0})
    | ({13{arm_prf  }} & {2'b00, pr_ap, 1'b0, pr_col, 1'b0})
    | ({13{arm_tr_act}} & {1'b0, i_G_A})
    | ({13{arm_tr_wr }} & {i_G_DQM[3], i_G_DQM[2], i_G_A[10], 1'b0, i_G_A[7:0], 1'b0})
    | ({13{arm_tr_pre}} & {2'b00, i_G_A[10:0]})
    | ({13{arm_nact }} & {2'b10, nor_addr[20:10]})
    | ({13{arm_nrd  }} & {2'b00, 1'b1, nor_addr[9:0]})
    | ({13{arm_lact }} & {2'b10, ld_a[20:10]})
    | ({13{arm_lwr  }} & {2'b00, 1'b1, ld_a[9:0]})
    | ({13{arm_mhi  }} & p_hi_row)
    | ({13{arm_mlo  }} & p_lo_row);

wire [1:0] pad_ba =
      ({2{arm_prf}} & pr_ba)
    | ({2{arm_tr_act | arm_tr_wr | arm_tr_pre}} & i_G_BA)
    | ({2{arm_mhi }} & p_hi_b)
    | ({2{arm_mlo }} & p_lo_b)
    | ({2{arm_mpre}} & m_bank);

wire [2:0] pad_cmd =
      ({3{cmd_pre_t}} & CMD_PRE)
    | ({3{cmd_ref_t}} & CMD_REF)
    | ({3{cmd_rd_t }} & CMD_RD)
    | ({3{cmd_act_t}} & CMD_ACT)
    | ({3{cmd_wr_t }} & CMD_WR)
    | ({3{arm_tr   }} & g_cmd)
    | {3{~cmd_any}};                                          // idle = NOP (111)

wire        pad_dq_we = arm_wrb | arm_tr_wr | arm_lwr;
wire [15:0] pad_dq =
      ({16{arm_wrb  }} & wr_lo)
    | ({16{arm_tr_wr}} & i_G_WDATA[31:16])                    // hi half rides the cmd beat
    | ({16{arm_lwr  }} & ld_hw);
wire        pad_oe = arm_wrb | arm_ldb | arm_tr_wr | arm_lwr;

always @(posedge i_CLK) begin
    if (!i_RST_n) begin
        {o_S_nCS, o_S_nRAS, o_S_nCAS, o_S_nWE} <= {1'b0, CMD_NOP};
        o_S_A <= 13'h0; o_S_BA <= 2'b00; o_S_CKE <= 1'b1;
        o_S_DQ_O <= 16'h0; o_S_DQ_OE <= 1'b0;
    end else begin
        o_S_nCS <= 1'b0;                     // chip-address bit: chip 0 always
        {o_S_nRAS, o_S_nCAS, o_S_nWE} <= pad_cmd;
        o_S_A     <= pad_a;                  // defaults fold to 0 = DQM active
        o_S_BA    <= pad_ba;
        o_S_DQ_OE <= pad_oe;
        if (pad_dq_we) o_S_DQ_O <= pad_dq;   // DQ_O holds between drive beats
        o_S_CKE   <= init_done ? (pcen_d ? i_G_CKE : o_S_CKE) : 1'b1;
    end
end

// synthesis translate_off
//------------------------------------------------------------------
// H7b.8d equivalence oracles.
// (1) fire-select one-hot: the flat arms must never overlap (the old
//     else-chain made overlap unreachable; here it is a proven invariant).
// (2) shadow pads: the ORIGINAL priority chain, replicated verbatim as
//     sh_* registers (pad writes only - state is read from the live
//     regs), compared against the flat stage every edge.  Chain==flat
//     is thereby re-proven by every sim run, over every workload.
//------------------------------------------------------------------
always @(posedge i_CLK) if (i_RST_n) begin
    if (!$onehot0({f_ipall, f_iref, f_imrs, arm_wrb, arm_ldb, arm_prf,
                   arm_tr, arm_nact, arm_nrd, arm_lact, arm_lwr,
                   arm_mpre, arm_mact}))
        $fatal(1, "[pad8d] fire-select overlap @%0t: %b", $time,
               {f_ipall, f_iref, f_imrs, arm_wrb, arm_ldb, arm_prf,
                arm_tr, arm_nact, arm_nrd, arm_lact, arm_lwr,
                arm_mpre, arm_mact});
end

reg [12:0] sh_a;
reg [1:0]  sh_ba;
reg        sh_ncs, sh_cke, sh_oe;
reg [2:0]  sh_cmd;
reg [15:0] sh_dqo;
reg        sh_arm;                           // compare enable (post-reset)

always @(posedge i_CLK) begin
    if (!i_RST_n) begin
        sh_ncs <= 1'b0; sh_cmd <= CMD_NOP;
        sh_a <= 13'h0; sh_ba <= 2'b00; sh_cke <= 1'b1;
        sh_dqo <= 16'h0; sh_oe <= 1'b0;
        sh_arm <= 1'b0;
    end else begin
        sh_arm <= 1'b1;
        // ---- the pre-H7b.8d output chain, verbatim ----
        sh_ncs <= 1'b0; sh_cmd <= CMD_NOP;
        sh_a <= 13'h0; sh_ba <= 2'b00;
        sh_oe <= 1'b0;
        sh_cke <= init_done ? (pcen_d ? i_G_CKE : sh_cke) : 1'b1;
        if (!init_done) begin
            case (ist)
                IS_WAIT: if (icnt >= INIT_WAIT) begin
                    sh_cmd <= CMD_PRE; sh_a <= 13'h0400;
                end
                IS_PALL: if (icnt >= 32'd3) sh_cmd <= CMD_REF;
                IS_REF: if (icnt >= 32'd8) begin
                    if (iref >= 4'd8) begin sh_cmd <= CMD_MRS; sh_a <= MRS_BL2_CL2; end
                    else sh_cmd <= CMD_REF;
                end
                default: ;
            endcase
        end
        else if (wr_beat2) begin
            sh_a <= {wr_lo_m[1], wr_lo_m[0], 11'h0};
            sh_dqo <= wr_lo; sh_oe <= 1'b1;
        end
        else if (ld_beat2) begin
            sh_a <= {2'b11, 11'h0}; sh_oe <= 1'b1;
        end
        else if (pr_fire) begin
            sh_cmd <= CMD_RD;
            sh_a <= {2'b00, pr_ap, 1'b0, pr_col, 1'b0};
            sh_ba <= pr_ba;
        end
        else if (pcen_d) begin
            if (!i_G_CS_n) begin
                case (g_cmd)
                    CMD_ACT: begin sh_cmd <= CMD_ACT; sh_a <= {1'b0, i_G_A}; sh_ba <= i_G_BA; end
                    CMD_WR: begin
                        sh_cmd <= CMD_WR;
                        sh_a <= {i_G_DQM[3], i_G_DQM[2], i_G_A[10], 1'b0, i_G_A[7:0], 1'b0};
                        sh_ba <= i_G_BA;
                        sh_dqo <= i_G_WDATA[31:16]; sh_oe <= 1'b1;
                    end
                    CMD_PRE: begin sh_cmd <= CMD_PRE; sh_a <= {2'b00, i_G_A[10:0]}; sh_ba <= i_G_BA; end
                    CMD_REF: sh_cmd <= CMD_REF;
                    default: ;
                endcase
            end
            if (!slotA_cmd) begin
                if (m_open) begin
                    if (m_cnt >= 3'd4) begin
                        sh_cmd <= CMD_PRE; sh_a <= 13'h0; sh_ba <= m_bank;
                    end
                end
                else if (sel_v && (ref_hold == 4'd0) && (act_gap == 2'd0) && !ld_go) begin
                    sh_cmd <= CMD_ACT; sh_a <= sel_row; sh_ba <= sel_bank;
                end
            end
        end
        else begin
            case (nst)
                NS_ACT: begin sh_cmd <= CMD_ACT; sh_a <= {2'b10, nor_addr[20:10]}; sh_ba <= 2'b00; end
                NS_RD:  begin sh_cmd <= CMD_RD;  sh_a <= {2'b00, 1'b1, nor_addr[9:0]}; sh_ba <= 2'b00; end
                default: begin
                    case (lst)
                        LS_ACT: begin sh_cmd <= CMD_ACT; sh_a <= {2'b10, ld_a[20:10]}; sh_ba <= 2'b00; end
                        LS_WR: begin
                            sh_cmd <= CMD_WR; sh_a <= {2'b00, 1'b1, ld_a[9:0]}; sh_ba <= 2'b00;
                            sh_dqo <= ld_hw; sh_oe <= 1'b1;
                        end
                        default: begin
                            if (m_open) begin
                                if (m_cnt >= 3'd4) begin
                                    sh_cmd <= CMD_PRE; sh_a <= 13'h0; sh_ba <= m_bank;
                                end
                            end
                            else if (sel_v && (ref_hold == 4'd0) && (act_gap == 2'd0) && !ld_go) begin
                                sh_cmd <= CMD_ACT; sh_a <= sel_row; sh_ba <= sel_bank;
                            end
                        end
                    endcase
                end
            endcase
        end
    end
end

always @(posedge i_CLK) if (sh_arm) begin
    if ({sh_ncs, sh_cmd, sh_a, sh_ba, sh_oe, sh_cke} !=
        {o_S_nCS, o_S_nRAS, o_S_nCAS, o_S_nWE, o_S_A, o_S_BA, o_S_DQ_OE, o_S_CKE})
        $fatal(1, "[pad8d] flat != chain @%0t: cmd %b%b A %04x/%04x BA %b/%b OE %b/%b",
               $time, sh_cmd, {o_S_nRAS, o_S_nCAS, o_S_nWE},
               sh_a, o_S_A, sh_ba, o_S_BA, sh_oe, o_S_DQ_OE);
    if (sh_oe && (sh_dqo != o_S_DQ_O))
        $fatal(1, "[pad8d] flat != chain DQ_O @%0t: %04x/%04x", $time, sh_dqo, o_S_DQ_O);
end
// synthesis translate_on

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
        $display("[pump] t=%0t GRD rdg=%06b cap=%03b dq=%04x hi=%04x lo=%04x w=%08x oe=%b",
                 $time, rdg_sh, cap_sh, i_S_DQ_I, rd_hi_e, rd_lo, rd_word, o_G_RDATA_OE);
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
