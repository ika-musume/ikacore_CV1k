`default_nettype none
//============================================================================
// ikacore_CV1k.sv - Cave CV1000-B PCB-level implementation
//
// Board-level netlist for the SH-3 execution-flow bring-up: the SH7709S (HS3
// core, ip_cores/HS3) wired to its two shared-bus memories as on the CV1000-B
// PCB, using the real vendor (NDA) device models patched only where Verilator
// 5 requires it (sim/models/*.verilator.patch):
//
//   U4 program NOR flash  - Macronix MX29LV320E (area 0 / CS0, word mode)
//   U1 work-RAM SDRAM     - Micron  MT48LC2M32B2 (area 3 / CS3, 8 MB)
//
// Both devices ride the SAME physical address/data bus (SH7709S HW manual
// Table 10.3) selected by CSn - the "shared bus" the task verifies against the
// datasheet.  The data bus is a true bidirectional net with tristate
// resolution; because Verilator cannot read an inout back inside a module, each
// vendor model takes an explicit *_in write-data input (Dq_in / Q_in) fed from
// the SH-3 o_D_O, per the model patch recipe.
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
// NOTE on the flash part: the physical CV1000-B U4 is a 2 MB MX29LV160D; this
// board uses its 4 MB Macronix sibling MX29LV320E (the proven-patched model)
// with the 2 MB image mirrored to 4 MB - identical to MAME's ROM_RELOAD - which
// the area-0 (ordinary-memory) controller treats identically. MX29LV160D is a
// drop-in via the same sibling patch recipe if the exact part is wanted.
//============================================================================
module ikacore_CV1k #(
    parameter ROM_FILE = "rom/ibara_u4_4M.hex"
) (
    input  wire i_CLK,      // SH-3 architectural clock (board = 102.4 MHz domain)
    input  wire i_CEN,      // architectural clock enable (1 in sim)
    input  wire i_POR_n,    // power-on reset  (RESETP)
    input  wire i_RST_n,    // manual reset    (RESETM)
    input  wire i_EXTAL2    // RTC 32.768 kHz crystal input
);

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

// the true bidirectional board data bus (shared A/D bus, Table 10.1)
wire [31:0] D;
assign D   = D_OE ? D_O : 32'hzzzz_zzzz;    // SH-3 drives on writes
assign D_I = D;                             // SH-3 samples the resolved bus

// area-0 bus width strap = 16 bit (Table 10.4: MD4=1, MD3=0)
localparam MD4 = 1'b1, MD3 = 1'b0;

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
    .i_BREQ_n(1'b1), .o_BACK_n(BACK_n), .o_BUS_OE(BUS_OE),

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

    // I/O port pads - inputs tied to benign idle, outputs open
    .i_PTA_I(8'hFF), .o_PTA_O(), .o_PTA_OE(), .o_PTA_PU(),
    .i_PTB_I(8'hFF), .o_PTB_O(), .o_PTB_OE(), .o_PTB_PU(),
    .i_PTC_I(8'hFF), .o_PTC_O(), .o_PTC_OE(), .o_PTC_PU(),
    .i_PTD_I(8'hFF), .o_PTD_O(), .o_PTD_OE(), .o_PTD_PU(),
    .i_PTE_I(8'hFF), .o_PTE_O(), .o_PTE_OE(), .o_PTE_PU(),   // bit5 = NAND ready
    .i_PTF_I(8'hFF), .o_PTF_PU(),
    .i_PTG_I(8'hFF), .o_PTG_PU(),
    .i_PTH_I(8'hFF), .o_PTH_O(), .o_PTH_OE(), .o_PTH_PU(),
    .i_PTJ_I(8'hFF), .o_PTJ_O(), .o_PTJ_OE(), .o_PTJ_PU(),
    .i_PTK_I(8'hFF), .o_PTK_O(), .o_PTK_OE(), .o_PTK_PU(),
    .i_PTL_I(8'hFF),
    .i_SCPT_I(8'hFF), .o_SCPT_O(), .o_SCPT_OE(), .o_SCPT_PU()
);

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
mt48lc2m32b2 u_u1_sdram (
    .Dq   (D[31:0]),                  // read data onto shared bus
    .Dq_in(D_O),                      // write data view (patch: no inout readback)
    .Addr (A[12:2]),
    .Ba   (A[14:13]),
    .Clk  (CKIO),
    .Cke  (CKE),
    .Cs_n (CS3_n),
    .Ras_n(RAS3L_n),
    .Cas_n(CASL_n),
    .We_n (RD_WR),
    .Dqm  (WE_n)
);

endmodule
`default_nettype none
