`default_nettype none
//============================================================================
// u1_pump.sv - double-pumped MiSTer SDRAM hub (docs/double_pump_sdram.md)
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
//   work RAM : BA = grid bank, RA = {2'b00, grid row[10:0]},
//              CA = {1'b0, grid col[7:0], beat}    (B-board pass-through;
//              D-board needs a 12-bit grid row - TODO with the CV1000-D top)
//   U4 NOR   : BA = 0, RA = {2'b10, F[20:10]}, CA = F[9:0]   (F = A[21:1],
//              rows 0x1000-0x17FF, 4 MB dense; 16 Mbit images are streamed
//              twice by the loader/MRA so the undecoded-A21 mirror is real)
// Mode register (programmed once at init, grid MRS cycles are acked+dropped):
//   BL=2 sequential, CL=3  (A[11:0] = 0x031)
//
// Module quirks handled (see mister_128mb.sv):
//   * CS_n is a chip-address bit - idle is encoded as NOP, never deselect;
//     chip 1 is never selected, initialized, or refreshed (it holds no data)
//   * chip DQM = A[12:11]: driven 00 on every cycle except write-data beats
//     (byte strobes) and ACT row phases (safe: no data in flight +-2 cycles)
//
// Refresh: the BSC's own CBR refresh commands are forwarded 1:1 (the bus
// stall is CPU-visible timing; the sim models hold data without decay).
// TODO(HW): hidden ACT/PRE row-maintenance scheduler for the 8192-row part
// per docs/double_pump_sdram.md section 6 - the sim A/B equivalence and all
// pin sequencing here are unaffected by it.
//============================================================================
module u1_pump #(
    parameter        NOR_BSWAP = 1'b0,       // 1: swap ioctl byte pairs (MAME dumps need 0)
    parameter [31:0] INIT_WAIT = 32'd200     // NOP cycles before the JEDEC init sequence
) (
    input  wire         i_CLK,               // 2xCKIO architectural clock
    input  wire         i_RST_n,             // memory-subsystem reset (released before CPU POR)
    input  wire         i_CKIO_PCEN,         // i_CLK cycle in which CKIO rises

    //------ grid: CS3 SDRAM bus, post blit_own mux (pin-true, 1xCKIO) ------
    input  wire [10:0]  i_G_A,               // muxed A[12:2] (row/col, AMX 0111)
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
    input  wire         i_N_WR_n,            // WE_n[1]&WE_n[0] (trap only - v1 is read-only)
    output wire [15:0]  o_N_RDATA,           // drives D[15:0] on CS0 reads
    output wire         o_N_RDATA_OE,

    //------ MiSTer HPS ioctl (active only while the CPU is held in reset) --
    input  wire         i_IOCTL_DOWNLOAD,
    input  wire         i_IOCTL_WR,
    input  wire [26:0]  i_IOCTL_ADDR,        // byte address within the stream
    input  wire [7:0]   i_IOCTL_DATA,
    input  wire [15:0]  i_IOCTL_INDEX,       // unused: NOR = first 32 Mbit of the stream
    output wire         o_IOCTL_WAIT,

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

// The BSC latches i_D_I exactly at fast edge E+4 (CKIO E+2: i_BCEN && rd_lat,
// CL2 pipeline in bsc.sv). At that same edge this module commits its NBAs, so
// the sample may resolve pre- or post-commit; both mux arms carry the word
// either way: pre-commit rdg_sh[2]=1 selects {rd_hi, live beat-1 DQ},
// post-commit rdg_sh[3]=1 selects the just-registered rd_word.
assign o_G_RDATA    = rdg_sh[2] ? {rd_hi, i_S_DQ_I} : rd_word;
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

// CS0 write trap (v1 serves reads only; saves live in the RTC-9701 EEPROM)
reg nor_wr_trap = 1'b0;
always @(posedge i_CLK) begin
    if (!i_N_CS_n && !i_N_WR_n) begin
        if (!nor_wr_trap)
            $display("[u1_pump] WARNING: CS0 flash WRITE ignored @%0t A=%06x (program/erase FSM not implemented)",
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
// main output sequencer - single always block, one command per fast edge.
// priority: init > grid slot A > pending write beat 2 > engines (slot B)
//------------------------------------------------------------------
reg        wr_beat2;                         // grid write lo-half pending
reg [15:0] wr_lo;
reg [1:0]  wr_lo_m;

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
    end else begin
        // defaults for this edge: NOP, DQM active (A[12:11]=00), bus released
        {o_S_nCS, o_S_nRAS, o_S_nCAS, o_S_nWE} <= {1'b0, CMD_NOP};
        o_S_A <= 13'h0; o_S_BA <= 2'b00;
        o_S_DQ_OE <= 1'b0;
        o_S_CKE <= init_done ? (pcen_d ? i_G_CKE : o_S_CKE) : 1'b1;

        // capture pipelines shift every edge.  With E = the CKIO edge opening
        // the grid READ cycle: issue @E+1, chip reg @E+2, beat0 on DQ @E+4,
        // beat1 @E+5; the word is assembled at E+5 and driven [E+5, E+7) -
        // covering the BSC's capture edge E+6 exactly as the baseline
        // 32-bit model does (its CL2 window is [E+4, E+6)).
        rdg_sh <= {rdg_sh[4:0], 1'b0};
        rdn_sh <= {rdn_sh[3:0], 1'b0};
        if (rdg_sh[1]) rd_hi   <= i_S_DQ_I;               // CL2 beat 0 at its drive edge (E+3)
        if (rdg_sh[2]) rd_word <= {rd_hi, i_S_DQ_I};      // CL2 beat 1 at its drive edge (E+4)

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
                        o_S_A  <= {2'b00, i_G_A};             // row pass-through, RA[12:11]=00
                        o_S_BA <= i_G_BA;
                    end
                    CMD_RD: begin                             // one grid CAS -> one BL2 pair
                        {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_RD;
                        o_S_A  <= {2'b00, i_G_A[10], 1'b0, i_G_A[7:0], 1'b0};
                        o_S_BA <= i_G_BA;
                        // whole-vector re-assign: a bit-select NBA after the
                        // vector-shift NBA is silently dropped by Verilator
                        rdg_sh <= {rdg_sh[4:0], 1'b1};
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
                    end
                    CMD_PRE: begin                            // PRE / PALL (A10 forwarded)
                        {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_PRE;
                        o_S_A  <= {2'b00, i_G_A};
                        o_S_BA <= i_G_BA;
                    end
                    CMD_REF: {o_S_nRAS, o_S_nCAS, o_S_nWE} <= CMD_REF;
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
                        default: ;
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
                NS_CAP: if (rdn_sh[1]) begin                  // CL2 beat 0 = the addressed halfword, at its drive edge
                    nor_data <= i_S_DQ_I;
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
            LS_IDLE: if (ld_go && init_done) lst <= LS_ACT;
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

endmodule
`default_nettype wire
