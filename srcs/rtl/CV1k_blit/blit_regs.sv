`default_nettype none
//============================================================================
// blit_regs.sv - CV1000-B blitter CS6 register file           [H0 / I-1.1]
//
// The CPU-visible register slave of the blitter (blitter_detail.md §3), living
// at CS6 base 0x18000000 (P2 mirror 0xB8000000), decoded by the U13 CPLD whose
// o_BLITTER_n simply mirrors CS6.  Area 6 is a 32-bit ordinary-memory space
// (boot BCR2 = 0x39F0 -> A6SZ = 11 = longword), so every register access is a
// single 32-bit bus cycle - no lane splitting.
//
// H0 scope: this is ONLY the register file that unsticks the CPU.  There is no
// op fetch, no draw engine, no timing governor yet, so `busy` is held 0 (the
// blitter always reads "ready"); an EXEC write latches the LIST_ADDR/CLIP
// shadows and is logged (the per-frame EXEC count is the H0 acceptance signal).
// The fetch/draw/governor consume exec_pulse + the shadows in H2..H4.
//
// Bus discipline (CKIO domain via i_CLK + i_CKIO_PCEN, matching the CPLD - no
// derived/gated clocks): reads are combinational and drive the shared bus while
// selected+RD; a write commits once, on the CKIO_PCEN in which the byte-0 write
// strobe first falls (address + write data are both valid by then).
//
// Register map (blitter_detail.md §3; offset = A[6:2] word index):
//   0x04 W  EXEC       [0]      1 -> latch shadows, kick op fetch (H2+)
//   0x08 W  LIST_ADDR  [28:0]   work-RAM address of the op list (sampled at EXEC)
//   0x10 R  STATUS     [4]      1 = ready / idle, 0 = busy   (see note below)
//   0x14 W  SCROLL_X   [15:0]   scanout window origin X
//   0x18 W  SCROLL_Y   [15:0]   scanout window origin Y
//   0x24 R/W IRQ ack   [0]      write pulses the video-side IRQ ack; reads 1s
//   0x40 W  CLIP_X     [15:0]   clip window origin X (latched at EXEC)
//   0x44 W  CLIP_Y     [15:0]   clip window origin Y (latched at EXEC)
//   0x50 R  DSW        [3:0]    DIP switch S2
//   (all other offsets: writes accepted+ignored, reads 0xFFFFFFFF)
//
// STATUS bit note (reconciles the H0 "boot-poll bit1 vs BD bit4" question):
// the ready flag is bit4 (value 0x10) per MAME/BD.  The FPGA-init poll at U4
// 0c049f88 does `tst #0x2` (bit1) - and bit1 is always 0 in the ready value
// 0x10, so it falls through exactly as it does on silicon / in MAME.  Modeling
// ready on bit4 with bit1 pinned 0 therefore satisfies both consumers.
//============================================================================
module blit_regs (
    input  wire [3:0]  i_DSW_S2,          // DIP switch S2 read at 0x50 (H7b: runtime
                                          // input - MiSTer OSD; was a parameter)
    input  wire        i_CLK,
    input  wire        i_CKIO_PCEN,       // pulses the i_CLK cycle CKIO rises
    input  wire        i_RST_n,

    // CS6 slave taps off the shared SH-3 bus
    input  wire        i_BLIT_n,          // blitter select = U13 o_BLITTER_n (= CS6_n)
    input  wire        i_RD_n,            // SH-3 read strobe (active low)
    input  wire [3:0]  i_WE_n,            // SH-3 byte write strobes (active low)
    input  wire        i_RD_WR,           // bus direction: 1 = read, 0 = write
    input  wire [6:2]  i_A,               // register offset (word index)
    input  wire [31:0] i_D,               // SH-3 write-data view of the bus (D_O)

    output wire [31:0] o_D,               // read data onto the shared bus
    output wire        o_D_OE,            // drive enable (asserted on a CS6 read)

    // busy from the blitter core (H2: fetch busy; H4: governor owns this -
    // the fetch window is a conservative subset of the original BUSY, which
    // always covers at least the list fetch)
    input  wire        i_busy,

    // blitter-core handoff (unused in H0 - consumed by the H2+ fetch unit)
    output reg         o_exec,            // 1-cycle pulse when EXEC is written with bit0
    output reg  [28:0] o_list_addr,       // shadow LIST_ADDR latched at EXEC
    output reg  [15:0] o_clip_x,          // shadow CLIP_X   latched at EXEC
    output reg  [15:0] o_clip_y,          // shadow CLIP_Y   latched at EXEC
    output reg  [15:0] o_scroll_x,        // live SCROLL_X
    output reg  [15:0] o_scroll_y,        // live SCROLL_Y
    output reg         o_irq_ack          // 1-cycle pulse when 0x24 is written
);

    // register offsets (A[6:2] word index)
    localparam [4:0] OFS_EXEC   = 5'h01;  // 0x04
    localparam [4:0] OFS_LIST   = 5'h02;  // 0x08
    localparam [4:0] OFS_STATUS = 5'h04;  // 0x10
    localparam [4:0] OFS_SCRX   = 5'h05;  // 0x14
    localparam [4:0] OFS_SCRY   = 5'h06;  // 0x18
    localparam [4:0] OFS_IRQACK = 5'h09;  // 0x24
    localparam [4:0] OFS_CLIPX  = 5'h10;  // 0x40
    localparam [4:0] OFS_CLIPY  = 5'h11;  // 0x44
    localparam [4:0] OFS_DSW    = 5'h14;  // 0x50

    // H2+: busy tracks the blitter core (fetch unit for now, governor at H4).
    wire busy = i_busy;

    //------------------------------------------------------------------
    // live register storage
    //------------------------------------------------------------------
    reg [28:0] list_addr;                 // 0x08 (live; shadow copied at EXEC)
    reg [15:0] clip_x, clip_y;            // 0x40 / 0x44 (live; shadow at EXEC)

    //------------------------------------------------------------------
    // write-commit detector: one commit per access, on the CKIO_PCEN where the
    // byte-0 write strobe first falls with the blitter selected and the bus in
    // write direction. (Area 6 is 32-bit, so a longword write is one cycle and
    // WE0 is always among the asserted lanes.)
    //------------------------------------------------------------------
    reg  we0_d;                           // WE_n[0] sampled on the previous CKIO_PCEN
    wire wr_commit = i_CKIO_PCEN & ~i_BLIT_n & ~i_RD_WR & we0_d & ~i_WE_n[0];

`ifndef SYNTHESIS
    integer exec_count = 0;               // sim-only: H0 acceptance signal

    // H4 anchor-test backdoor (tb_cv1k +blitanchor): the TB sets bd_exec_req
    // together with bd_list/bd_clip_* hierarchically; consumed below as a
    // real EXEC pulse with the given shadows - exercises the fetch/governor/
    // draw path without a CPU bus write.  Sim-only, cleared when taken.
    reg        bd_exec_req = 1'b0;
    reg [28:0] bd_list     = 29'd0;
    reg [15:0] bd_clip_x   = 16'd0;
    reg [15:0] bd_clip_y   = 16'd0;
`endif

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            we0_d      <= 1'b1;
            list_addr  <= 29'd0;
            clip_x     <= 16'd0;
            clip_y     <= 16'd0;
            o_scroll_x <= 16'd0;
            o_scroll_y <= 16'd0;
            o_list_addr<= 29'd0;
            o_clip_x   <= 16'd0;
            o_clip_y   <= 16'd0;
            o_exec     <= 1'b0;
            o_irq_ack  <= 1'b0;
        end
        else begin
            o_exec    <= 1'b0;            // default: one-cycle pulses
            o_irq_ack <= 1'b0;

            if (i_CKIO_PCEN)
                we0_d <= i_WE_n[0];

            if (wr_commit) begin
                case (i_A)
                    OFS_LIST : list_addr  <= i_D[28:0];
                    OFS_SCRX : o_scroll_x <= i_D[15:0];
                    OFS_SCRY : o_scroll_y <= i_D[15:0];
                    OFS_CLIPX: clip_x     <= i_D[15:0];
                    OFS_CLIPY: clip_y     <= i_D[15:0];
                    OFS_IRQACK: o_irq_ack <= i_D[0];   // video-side IRQ ack pulse
                    OFS_EXEC : if (i_D[0]) begin        // shadow-latch + kick
                        o_exec      <= 1'b1;
                        o_list_addr <= list_addr;
                        o_clip_x    <= clip_x;
                        o_clip_y    <= clip_y;
`ifndef SYNTHESIS
                        exec_count <= exec_count + 1;
                        $display("[blit] EXEC #%0d  list=%08x  clip=(%0d,%0d)  scroll=(%0d,%0d)",
                                 exec_count + 1, {3'b000, list_addr},
                                 clip_x, clip_y, o_scroll_x, o_scroll_y);
`endif
                    end
                    default  : ;          // accepted + ignored (0x10/0x1C/0x34/...)
                endcase
            end

`ifndef SYNTHESIS
            if (bd_exec_req) begin        // TB anchor injection (see above)
                bd_exec_req <= 1'b0;
                o_exec      <= 1'b1;
                o_list_addr <= bd_list;
                o_clip_x    <= bd_clip_x;
                o_clip_y    <= bd_clip_y;
                exec_count  <= exec_count + 1;
                $display("[blit] EXEC #%0d (backdoor)  list=%08x  clip=(%0d,%0d)",
                         exec_count + 1, {3'b000, bd_list}, bd_clip_x, bd_clip_y);
            end
`endif
        end
    end

    //------------------------------------------------------------------
    // read path: combinational, driven onto the shared bus while selected+RD
    //------------------------------------------------------------------
    reg [31:0] rdata;
    always_comb begin
        case (i_A)
            OFS_STATUS: rdata = {27'b0, ~busy, 4'b0};  // bit4 = ready
            OFS_DSW   : rdata = {28'b0, i_DSW_S2};     // 0x50
            default   : rdata = 32'hFFFF_FFFF;         // 0x24/0x28 observed value
        endcase
    end

    assign o_D_OE = ~i_BLIT_n & ~i_RD_n;
    assign o_D    = rdata;

endmodule
`default_nettype none
