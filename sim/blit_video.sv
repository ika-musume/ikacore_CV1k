`default_nettype none
//============================================================================
// blit_video.sv - CV1000-B video scanout                  [H5 / I-3.1/2/3]
//
// Sync generator + line fetcher + scroll latch + vsync IRQ2 source, per
// blitter_detail.md §9 with the PROVISIONAL parameter set (every value
// below is a working assumption until the M-1/M-2/M-7/M-11 measurements):
//
//   dot clock 6.4 MHz = VCLK/12   -> 1 dot = 12 VCLK = exactly 8 CKIO
//   HTOTAL 407 dots (= 4884 VCLK = 3256 CKIO per line, 63.594 us)
//   VTOTAL 262 lines -> frame = 853,072 CKIO = 60.0184 Hz
//   visible 320x240 at dot HACT_START..+319, line 0..239
//   vsync/IRQ2 at line VSYNC_LINE dot 0 (position = M-2, provisional)
//
// Everything counts on i_CLK + i_CKIO_PCEN (standing rule: no derived
// clocks).  The 12.8 MHz integer clock family makes every video quantity
// an exact CKIO count - the same zero-drift arithmetic the H4 governor
// uses for its half-VCLK time base.
//
// LINE FETCHER (BD §9.3): at every line start (free-running cadence, also
// during vblank and while the blitter idles - matches the cost model and
// the [PDF] capture showing steals before blitter start):
//   1. latch SCROLL_X/Y (BD §9.4 latch point = line start, safest until
//      the mid-frame-write measurement);
//   2. fetch 12 tiles = 384 px covering any 32-px scroll alignment:
//      96 4-px beats from y_v = (scroll_y + line) & 4095,
//      x = ((scroll_x & ~31) + n*4) & 8191 into the line buffer
//      (x wrap per tile = MAME copyscrollbitmap semantics);
//   3. hold o_steal for STEAL_CKIO (111 CKIO = 2.168 us ~ the 166-VCLK
//      hline steal) - the top level gates the draw engine's write channel
//      (i_wr_rdy) with it: scanout owns the memory, the engine stalls on
//      its EXISTING backpressure port.  Real command-level arbitration
//      (absolute priority, non-preemptive mid-train) is the I-4.3 DDR3
//      adapter's job behind these same channels; the behavioral VRAM has
//      infinite bandwidth so only the stall window is modeled here.
//
// o_hline (one pulse per line, at the steal point) re-anchors the H4
// governor's boundary register - the timing plane's steal cadence and the
// execution plane's real steal share one phase (I-2.3 second half).
// Line 0 right after reset has no preceding line-start pulse, so its
// buffer content is stale (all-black VRAM at power-up: benign).
//
// Synthesis notes (target: this module lives in the 153.6 MHz domain):
//   * dot CE becomes a /24 counter (or /12 of a 76.8 CE); all CKIO counts
//     here scale x3 to 153.6-cycles - keep them parameters;
//   * line_buf -> one M9K (384 x 16);
//   * the vrd channel maps onto the DDR3 scanout port (absolute priority);
//   * o_hline/o_vsync cross into the CKIO domain as toggle-syncs;
//   * i_scroll_* are quasi-static CPU-written registers (2-FF sync).
//============================================================================
module blit_video #(
    parameter int unsigned HTOTAL_DOT = 407,   // dots per line (P-30)
    parameter int unsigned HACT_START = 80,    // first visible dot (provisional)
    parameter int unsigned HACT       = 320,
    parameter int unsigned VTOTAL     = 262,
    parameter int unsigned VACT       = 240,   // visible lines 0..239
    parameter int unsigned VSYNC_LINE = 240,   // IRQ2 line (provisional, M-2)
    parameter int unsigned DOT_CKIO   = 8,     // 1 dot = 8 CKIO (P-30)
    parameter int unsigned STEAL_CKIO = 111    // ~166 VCLK engine-stall window
)(
    input  wire        i_CLK,
    input  wire        i_CKIO_PCEN,
    input  wire        i_RST_n,

    // live scroll registers from blit_regs (latched here per line)
    input  wire [15:0] i_scroll_x,
    input  wire [15:0] i_scroll_y,

    // video-side IRQ ack (blit_regs 0x24 write).  Consumed but unused until
    // M-2 answers whether the ack deasserts IRQ2 (ICR1 = falling-edge
    // triggered, so the deassert shape is invisible to the INTC either way).
    input  wire        i_irq_ack,

    // VRAM scanout read channel (blit_vram_beh 3rd port, fixed-latency
    // hold contract, flat 25-bit pixel addresses like the engine's)
    output reg         o_vrd_req,
    output reg  [24:0] o_vrd_addr,
    input  wire [63:0] i_vrd_data,

    // arbitration + timing-plane taps
    output wire        o_steal,          // scanout owns VRAM: stall engine writes
    output reg         o_hline,          // 1-cycle pulse per line (steal point)
    output reg         o_vsync,          // 1-cycle pulse at VSYNC_LINE start

    // pixel stream (o_px valid for the 1-cycle o_px_de strobe, 320x240/frame)
    output reg         o_px_de,
    output reg  [15:0] o_px
);

    // ---------------------------------------------------------------------
    // counters: ckio_div 0..DOT_CKIO-1 / hcnt 0..HTOTAL-1 / vcnt 0..VTOTAL-1
    // ---------------------------------------------------------------------
    reg [2:0]  ckio_div;
    reg [8:0]  hcnt /*verilator public_flat_rd*/;
    reg [8:0]  vcnt /*verilator public_flat_rd*/;

    wire dot_ce    = i_CKIO_PCEN && (ckio_div == 3'(DOT_CKIO - 1));
    wire line_last = (hcnt == 9'(HTOTAL_DOT - 1));

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            ckio_div <= 3'd0;
            hcnt     <= 9'd0;
            vcnt     <= 9'd0;
        end
        else if (i_CKIO_PCEN) begin
            ckio_div <= (ckio_div == 3'(DOT_CKIO - 1)) ? 3'd0 : ckio_div + 3'd1;
            if (dot_ce) begin
                hcnt <= line_last ? 9'd0 : hcnt + 9'd1;
                if (line_last)
                    vcnt <= (vcnt == 9'(VTOTAL - 1)) ? 9'd0 : vcnt + 9'd1;
            end
        end
    end

    wire line_start = dot_ce && line_last;    // next cycle is dot 0 of a line
    wire [8:0] next_line = line_last ? ((vcnt == 9'(VTOTAL - 1)) ? 9'd0
                                                                 : vcnt + 9'd1)
                                     : vcnt;

    // ---------------------------------------------------------------------
    // per-line latch + hline/vsync pulses (all at the line-start instant)
    // ---------------------------------------------------------------------
    reg [12:0] lat_sx;                        // scroll_x latched at line start
    reg [11:0] y_v;                           // fetch row = (scroll_y+line)&4095

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            lat_sx  <= 13'd0;
            y_v     <= 12'd0;
            o_hline <= 1'b0;
            o_vsync <= 1'b0;
        end
        else begin
            o_hline <= line_start;
            o_vsync <= line_start && (next_line == 9'(VSYNC_LINE));
            if (line_start) begin
                lat_sx <= i_scroll_x[12:0];
                y_v    <= i_scroll_y[11:0] + {3'd0, next_line};
            end
        end
    end

    // ---------------------------------------------------------------------
    // steal window: STEAL_CKIO CKIO from each line start
    // ---------------------------------------------------------------------
    reg [7:0] steal_cnt;
    assign o_steal = (steal_cnt != 8'd0);

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n)
            steal_cnt <= 8'd0;
        else if (o_hline)
            steal_cnt <= 8'(STEAL_CKIO);
        else if (i_CKIO_PCEN && steal_cnt != 8'd0)
            steal_cnt <= steal_cnt - 8'd1;
    end

    // ---------------------------------------------------------------------
    // line fetcher: 96 beats (12 tiles x 8) at 1 beat/i_CLK, kicked by
    // o_hline.  Beat n reads x = ((lat_sx & ~31) + n*4) & 8191 - the +n*4
    // never crosses the row edge inside a tile (32-px tiles, 8192-px rows),
    // so the & 8191 per beat IS the per-tile wrap.  Data lands one cycle
    // after the request (hold contract) into line_buf[n*4 +: 4].
    // ---------------------------------------------------------------------
    localparam int unsigned FETCH_BEATS = 96;   // 12 tiles = 384 px

    reg  [15:0] line_buf [0:383];
    reg  [6:0]  f_beat;                        // 0..96; 96 = idle
    reg  [6:0]  f_beat_d;                      // beat whose data lands now
    reg         f_dvalid;

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            f_beat     <= 7'(FETCH_BEATS);
            f_beat_d   <= 7'd0;
            f_dvalid   <= 1'b0;
            o_vrd_req  <= 1'b0;
            o_vrd_addr <= 25'd0;
        end
        else begin
            // request stage
            if (o_hline) begin
                f_beat     <= 7'd0;
                o_vrd_req  <= 1'b0;            // address stage next cycle
            end
            else if (f_beat != 7'(FETCH_BEATS)) begin
                o_vrd_req  <= 1'b1;
                o_vrd_addr <= {y_v, ({lat_sx[12:5], 5'd0} + {4'd0, f_beat, 2'd0}) & 13'h1FFF};
                f_beat     <= f_beat + 7'd1;
            end
            else
                o_vrd_req  <= 1'b0;

            // capture stage (data for the request issued last cycle)
            f_dvalid <= o_vrd_req;
            if (o_vrd_req)
                f_beat_d <= f_beat - 7'd1;
            if (f_dvalid) begin
                line_buf[{f_beat_d, 2'd0} + 9'd0] <= i_vrd_data[15:0];
                line_buf[{f_beat_d, 2'd0} + 9'd1] <= i_vrd_data[31:16];
                line_buf[{f_beat_d, 2'd0} + 9'd2] <= i_vrd_data[47:32];
                line_buf[{f_beat_d, 2'd0} + 9'd3] <= i_vrd_data[63:48];
            end
        end
    end

    // ---------------------------------------------------------------------
    // pixel readout: visible dot d shows line_buf[(scroll_x & 31) + d]
    // ---------------------------------------------------------------------
    wire visible = (vcnt < 9'(VACT)) &&
                   (hcnt >= 9'(HACT_START)) && (hcnt < 9'(HACT_START + HACT));
    wire [8:0] act_x = hcnt - 9'(HACT_START);

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            o_px_de <= 1'b0;
            o_px    <= 16'd0;
        end
        else begin
            o_px_de <= dot_ce && visible;
            if (dot_ce && visible)
                o_px <= line_buf[{4'd0, lat_sx[4:0]} + act_x];
        end
    end

    wire unused = i_irq_ack | (|i_scroll_x[15:13]) | (|i_scroll_y[15:12]);

endmodule
`default_nettype none
