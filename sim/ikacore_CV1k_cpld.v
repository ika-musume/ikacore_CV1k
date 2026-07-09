`default_nettype wire

/*
    U13 - Altera EPM7032 address-decoder CPLD (CV1000-B), Verilator model.

    Adapted from the community open-source reproduction bitstream
    (docs/u13_repro_src.v; refs docs/u13_README.md, docs/u13_pins.txt). U13 is
    the only glue logic on the CV1000-B: it decodes the SH-3 CS4 window
    [0x10000000, 0x14000000) into the three CS4 devices by {A23,A22}, passes CS6
    straight through to the blitter, and serialises the RTC-9701 EEPROM/RTC.

      A23 A22   region              device
      0   0     0x10000000          U2 NAND flash  (graphics / assets)
      0   1     0x10400000          YMZ770 audio
      1   1     0x10C00000          RTC-9701 EEPROM/RTC (+ setup / U2-CE side door)

    Operation type is taken from the low address bits {A1,A0}, which on the NAND
    double as U2 ALE (A1) and U2 CLE (A0); the EEPROM/RTC namespace reuses them
    as sub-registers (0x10C00001 serial, 0x10C00002 setup, 0x10C00003 U2 chip
    enable).

    Boot relevance: FUN_0c04bc34 writes 0b110 to 0x10C00001 and spins until the
    read-back nibble shows bit1 or bit2 set. On any EEPROM/RTC read this CPLD
    drives D[3:0] = {1,1,1,eeprom_do}, so bits[2:1] are constant 1 and the poll
    falls through independently of the (stubbed) serial data line.

    FPGA methodology: the reproduction is clocked on CKIO. To keep the whole
    board in the single i_CLK domain (no derived/gated clocks on the FPGA), the
    CKIO-edge registers here advance on i_CLK qualified by i_CKIO_PCEN - the
    cpg_wdt clock-enable that pulses the i_CLK cycle in which CKIO rises.

    Verilator: the 4-bit board data nibble is a shared tristate net, and
    Verilator cannot read an inout back inside a module, so the data port is
    split into i_D (write-data view) / o_D (drive value) / o_D_OE (drive enable),
    exactly as the vendor memory models take *_in ports. The top level resolves
    "assign D[3:0] = o_D_OE ? o_D : 4'hz".
*/

module ikacore_CV1k_cpld (
    /* CLOCK AND RESET - CKIO domain via i_CLK + i_CKIO_PCEN, POR-held */
    input   wire            i_CLK,          //board architectural clock
    input   wire            i_CKIO_PCEN,    //pulses the i_CLK cycle CKIO rises
    input   wire            i_RST_n,        //power-on reset (RESETP)

    /* SH-3 STROBES / CHIP SELECTS (active low) */
    input   wire            i_CS4_n,        //U2 / audio / EEPROM window
    input   wire            i_CS5_n,        //unused (carried for completeness)
    input   wire            i_CS6_n,        //blitter
    input   wire            i_RD_n,         //SH-3 read strobe  (U4 OE)
    input   wire            i_WE_n,         //SH-3 write strobe (U4 WE0)

    /* ADDRESS TAPS */
    input   wire    [1:0]   i_A_HI,         //{A23, A22} - region select
    input   wire    [1:0]   i_A_LO,         //{A1,  A0}  - operation / sub-reg
    input   wire            i_A2,           //A2 - unused

    /* SHARED DATA NIBBLE (split inout, see header) */
    input   wire    [3:0]   i_D,            //SH-3 write-data view of D[3:0]
    output  wire    [3:0]   o_D,            //CPLD drive value for D[3:0]
    output  wire            o_D_OE,         //drive enable (EEPROM/RTC reads only)

    /* U2 NAND CONTROL (active low) */
    output  reg             o_U2_CE_n,      //chip enable  (set from 0x10C00003 d0)
    output  reg             o_U2_RE_n,      //read enable
    output  reg             o_U2_WE_n,      //write enable

    /* YMZ770 AUDIO */
    output  reg             o_AUDIO_CS_n,   //chip select (active low)
    output  wire            o_AUDIO_RESET,  //low until initialised, then high

    /* RTC-9701 EEPROM/RTC SERIAL */
    input   wire            i_EEPROM_DO,    //serial data from RTC-9701
    input   wire            i_EEPROM_TIRQ,  //unused (always low on real HW)
    output  reg             o_EEPROM_DI,    //serial data to RTC-9701
    output  reg             o_EEPROM_CLK,   //serial clock
    output  reg             o_EEPROM_CE,    //high while EEPROM/RTC in use
    output  reg             o_EEPROM_FOE,   //high until first use, then low

    /* MISC PASSTHROUGH / STATUS */
    input   wire            i_AUDIO_PLAY,   //unused
    output  wire            o_BLITTER_n,    //mirrors CS6 to the blitter
    output  wire            o_SH3_WAIT,     //always high (unused)
    output  wire            o_GLOBAL_CLR,   //always low
    output  wire            o_DEVICE_READY  //setup handshake done (debug)
);

    /* region select {A23,A22} */
    localparam [1:0] AHI_U2     = 2'b00;   //0x10000000
    localparam [1:0] AHI_AUDIO  = 2'b01;   //0x10400000
    localparam [1:0] AHI_EEPROM = 2'b11;   //0x10C00000

    /* EEPROM/RTC sub-register {A1,A0} */
    localparam [1:0] ALO_EEPROM = 2'b01;   //0x10C00001 serial link
    localparam [1:0] ALO_SETUP  = 2'b10;   //0x10C00002 setup / device ready
    localparam [1:0] ALO_U2_CS  = 2'b11;   //0x10C00003 U2 chip-enable side door

    //------------------------------------------------------------------
    // Setup handshake: latch "ready" once 4'b1110 is written to 0x10C00002
    // (after the FPGA/blitter bitstream upload). Audio comes out of reset then.
    //------------------------------------------------------------------
    reg device_ready;
    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n)
            device_ready <= 1'b0;
        else if (i_CKIO_PCEN) begin
            if (!i_CS4_n && i_A_HI == AHI_EEPROM &&
                            i_A_LO == ALO_SETUP  && i_D == 4'b1110)
                device_ready <= 1'b1;
        end
    end

    assign o_AUDIO_RESET = device_ready;
    assign o_DEVICE_READY = device_ready;

    //------------------------------------------------------------------
    // Fixed passthroughs (unclocked on real silicon).
    //------------------------------------------------------------------
    assign o_SH3_WAIT   = 1'b1;   //SH-3 WAIT never asserted
    assign o_GLOBAL_CLR = 1'b0;   //global clear tied low
    assign o_BLITTER_n  = i_CS6_n;//blitter select mirrors CS6

    //------------------------------------------------------------------
    // Data-bus drive: the CPLD only drives D[3:0] during EEPROM/RTC reads,
    // presenting {1,1,1,eeprom_do}. o_D_OE is the "eeprom_is_output" term.
    //------------------------------------------------------------------
    reg eeprom_is_output;
    assign o_D    = {3'b111, i_EEPROM_DO};
    assign o_D_OE = eeprom_is_output;

    //------------------------------------------------------------------
    // Combinational decode (unclocked): audio CS, U2 RE/WE, EEPROM output
    // enable - selected by the region bits {A23,A22}. Active-low strobes
    // default high (deselected); assumes RD and WE are never low together.
    //------------------------------------------------------------------
    always_comb begin
        case (i_A_HI)
            AHI_AUDIO: begin        // 0x10400000 : audio
                o_AUDIO_CS_n    = i_CS4_n;
                o_U2_RE_n       = 1'b1;
                o_U2_WE_n       = 1'b1;
                eeprom_is_output = 1'b0;
            end
            AHI_U2: begin           // 0x10000000 : U2 NAND
                o_AUDIO_CS_n    = 1'b1;
                o_U2_RE_n       = i_RD_n | i_CS4_n;
                o_U2_WE_n       = i_WE_n | i_CS4_n;
                eeprom_is_output = 1'b0;
            end
            AHI_EEPROM: begin       // 0x10C00000 : EEPROM/RTC
                o_AUDIO_CS_n    = 1'b1;
                o_U2_RE_n       = 1'b1;
                o_U2_WE_n       = 1'b1;
                eeprom_is_output = !(i_RD_n | i_CS4_n);
            end
            default: begin          // 0x10800000 : nothing
                o_AUDIO_CS_n    = 1'b1;
                o_U2_RE_n       = 1'b1;
                o_U2_WE_n       = 1'b1;
                eeprom_is_output = 1'b0;
            end
        endcase
    end

    //------------------------------------------------------------------
    // Clocked EEPROM/RTC serial link + U2 chip enable (CKIO domain).
    //   0x10C00003 d0 -> U2 CE (active low)
    //   0x10C00001    -> FOE low forever; on write, {ce,clk,di} = d[2:0]
    //------------------------------------------------------------------
    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            o_EEPROM_DI  <= 1'b0;
            o_EEPROM_CE  <= 1'b0;
            o_EEPROM_CLK <= 1'b0;
            o_EEPROM_FOE <= 1'b1;
            o_U2_CE_n    <= 1'b1;
        end else if (i_CKIO_PCEN) begin
            if (!i_CS4_n && i_A_HI == AHI_EEPROM) begin
                case (i_A_LO)
                    ALO_U2_CS: o_U2_CE_n <= !i_D[0];
                    ALO_EEPROM: begin
                        o_EEPROM_FOE <= 1'b0;
                        if (!i_WE_n) begin
                            o_EEPROM_CE  <= i_D[2];
                            o_EEPROM_CLK <= i_D[1];
                            o_EEPROM_DI  <= i_D[0];
                        end
                    end
                    default: ;
                endcase
            end
        end
    end

endmodule

`default_nettype none
