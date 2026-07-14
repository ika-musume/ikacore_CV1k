`default_nettype none
//============================================================================
// ddr3_harness.sv - shared-DDR3 train arbiter + MiSTer DDRAM face  [H7a/I-4.3]
//
// The swappable memory backend of the three-layer stack (blit_top ->
// blit_batch -> HERE).  Multiplexes the port's tenants onto the single
// FPGA->HPS SDRAM bridge, at TRAIN granularity, non-preemptive, priority
//
//     video line fetch  >  blit_batch trains  >  NAND page fill  (> YMZ, later)
//
// exactly the FINDINGS.md §5.1 scheduling rule: every client's occupancy is
// a bounded train, so lower-priority injections look like the latency tails
// the engine's run-ahead already absorbs, and the video's worst wait (one
// whole batch train, ~5-8 us) is what blit_video's 1-hline prefetch buffer
// is sized against.
//
// Clients:
//   * video (blit_video PREFETCH=1 o_lf face): one request = one 96-word
//     line window at (y, x0), x wrapping mod 8192 px inside the row -> at
//     most two DDRAM bursts.  Requests are latched, served between trains.
//   * blit_batch: read burst commands (one per objline segment, pipelined
//     back-to-back so the controller exposes ONE latency per train) and
//     posted write words.  The batch's own o_rd_train/o_wr_train framing
//     delimits its port ownership; the harness never breaks a train.
//   * NAND (step 5): one 264-word page-register fill per request, lowest
//     priority, single train.
//
// DDRAM protocol (MiSTer f2sdram face, 64-bit words): commands accepted
// when !DDRAM_BUSY; reads return BURSTCNT words in order on DDRAM_DOUT_READY
// (commands may pipeline ahead - in-order completion); writes are one WE
// word each (BURSTCNT=1 - write burst formation is an H7b optimization,
// costed as-is by the TB port model so sim and target agree).  Read bursts
// are split at 128 words (the calibrated model's BL_MAX).
//
// Address map: VRAM_BASE_W + flat-pixel-word (23 bit) for video/batch; the
// NAND client passes a pre-mapped word address (its image lives elsewhere
// in DDR3).  All addresses are 64-bit-word units on DDRAM_ADDR.
//============================================================================
module ddr3_harness #(
    parameter [28:0] VRAM_BASE_W = 29'h0600_0000  // byte 0x3000_0000 >> 3
)(
    input  wire        i_CLK,
    input  wire        i_RST_n,

    //------------------------------------------------------------------
    // client 0: video line trains (absolute priority)
    //------------------------------------------------------------------
    input  wire        i_lf_req,
    input  wire [11:0] i_lf_y,
    input  wire [12:0] i_lf_x0,        // 32-px aligned
    output wire        o_lf_dvld,
    output wire [63:0] o_lf_data,

    //------------------------------------------------------------------
    // client 1: blit_batch train port
    //------------------------------------------------------------------
    input  wire        i_prd_req,
    input  wire [22:0] i_prd_addr,
    input  wire [10:0] i_prd_len,
    output wire        o_prd_rdy,
    output wire        o_prd_dvld,
    output wire [63:0] o_prd_data,
    input  wire        i_pwr_req,
    input  wire [22:0] i_pwr_addr,
    input  wire [63:0] i_pwr_data,
    input  wire [3:0]  i_pwr_be,       // pixel lanes -> byte-pair enables
    output wire        o_pwr_rdy,
    input  wire        i_rd_train,
    input  wire        i_wr_train,

    //------------------------------------------------------------------
    // client 2: NAND page fill (step 5; tie i_nd_req=0 until then)
    //------------------------------------------------------------------
    input  wire        i_nd_req,
    input  wire [28:0] i_nd_addr,      // pre-mapped DDRAM word address
    input  wire [10:0] i_nd_len,
    output wire        o_nd_rdy,
    output wire        o_nd_dvld,
    output wire [63:0] o_nd_data,

    //------------------------------------------------------------------
    // MiSTer DDRAM face
    //------------------------------------------------------------------
    output wire        DDRAM_CLK,
    input  wire        DDRAM_BUSY,
    output reg  [7:0]  DDRAM_BURSTCNT,
    output reg  [28:0] DDRAM_ADDR,
    input  wire [63:0] DDRAM_DOUT,
    input  wire        DDRAM_DOUT_READY,
    output reg         DDRAM_RD,
    output reg  [63:0] DDRAM_DIN,
    output reg  [7:0]  DDRAM_BE,
    output reg         DDRAM_WE
);

    assign DDRAM_CLK = i_CLK;

    localparam [1:0] OWN_NONE = 2'd0, OWN_VID = 2'd1,
                     OWN_BAT  = 2'd2, OWN_ND  = 2'd3;
    reg [1:0] own /*verilator public_flat_rd*/;

    // ---------------------------------------------------------------------
    // in-order read-return routing: one {owner} tag + word count per issued
    // DDRAM read burst (f2sdram completes in order).  16 batch segments + 2
    // video + NAND splits fit in 32.  Occupancy is DERIVED from the
    // pointers - a shared up/down counter written from both the pop logic
    // and a same-edge push loses one of the updates (last NBA wins) and
    // desyncs the routing.  The 1-cycle head-reload bubble after each
    // burst is covered by the >=2-cycle inter-burst data gap (G_CMD + the
    // first word's beta).
    // ---------------------------------------------------------------------
    reg [9:0]  oq [0:31];              // {owner[1:0], len[7:0]}
    reg [4:0]  oq_wp, oq_rp;
    reg [7:0]  oq_left /*verilator public_flat_rd*/;
    reg        oq_head_v /*verilator public_flat_rd*/;

    wire [4:0] oq_occ  = oq_wp - oq_rp;          // entries incl. the head
    wire       oq_room = (oq_occ < 5'd30);
    wire [1:0] ret_own = oq[oq_rp][9:8];

    assign o_lf_dvld  = DDRAM_DOUT_READY && oq_head_v && (ret_own == OWN_VID);
    assign o_lf_data  = DDRAM_DOUT;
    assign o_prd_dvld = DDRAM_DOUT_READY && oq_head_v && (ret_own == OWN_BAT);
    assign o_prd_data = DDRAM_DOUT;
    assign o_nd_dvld  = DDRAM_DOUT_READY && oq_head_v && (ret_own == OWN_ND);
    assign o_nd_data  = DDRAM_DOUT;

    // ---------------------------------------------------------------------
    // pending video request (latched; served between trains)
    // ---------------------------------------------------------------------
    reg        vid_pend;
    reg [11:0] vid_y;
    reg [12:0] vid_x0;

    // video window as 1-2 bursts: 96 words from word (y*2048 + x0/4),
    // wrapping mod 2048 words inside the row
    wire [10:0] vid_w0    = {vid_x0[12:2]};          // start word in row
    wire [11:0] vid_rem   = 12'd2048 - {1'b0, vid_w0};
    wire [7:0]  vid_len1  = (vid_rem < 12'd96) ? vid_rem[7:0] : 8'd96;
    wire [7:0]  vid_len2  = 8'd96 - vid_len1;

    // ---------------------------------------------------------------------
    // batch read command splitter (segments may exceed one DDRAM burst)
    // ---------------------------------------------------------------------
    reg        bp_v;                   // batch macro-cmd in flight
    reg [28:0] bp_addr;
    reg [10:0] bp_left;
    localparam [10:0] BL_MAX = 11'd128;

    wire [7:0] bp_burst = (bp_left > BL_MAX) ? 8'(BL_MAX) : bp_left[7:0];

    // NAND macro-cmd splitter (same shape)
    reg        np_v;
    reg [28:0] np_addr;
    reg [10:0] np_left;
    wire [7:0] np_burst = (np_left > BL_MAX) ? 8'(BL_MAX) : np_left[7:0];

    // accept a batch read command only when the splitter register is free
    // (commands still pipeline: the splitter drains at DDRAM accept rate)
    assign o_prd_rdy = (own == OWN_BAT) && !bp_v;
    assign o_nd_rdy  = (own == OWN_ND)  && !np_v;

    // batch writes pass through whenever the batch owns the port and the
    // face can take a word at this edge (a held WE word leaves exactly when
    // !BUSY, so a new one may load the same edge -> full-rate posted words)
    assign o_pwr_rdy = (own == OWN_BAT) && !DDRAM_RD && !bp_v
                       && (!DDRAM_WE || !DDRAM_BUSY);

    // batch activity (train framing + live handshakes): OWN_BAT persists
    // while any of it is up or read returns are outstanding for it
    wire bat_busy = i_rd_train || i_wr_train || i_prd_req || i_pwr_req
                    || bp_v || DDRAM_WE;

    reg [1:0] vid_seg;                 // 0 idle / 1 first burst / 2 second

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            own <= OWN_NONE;
            oq_wp <= '0; oq_rp <= '0;
            oq_left <= '0; oq_head_v <= 1'b0;
            vid_pend <= 1'b0; vid_y <= '0; vid_x0 <= '0;
            bp_v <= 1'b0; bp_addr <= '0; bp_left <= '0;
            np_v <= 1'b0; np_addr <= '0; np_left <= '0;
            vid_seg <= 2'd0;
            DDRAM_RD <= 1'b0; DDRAM_WE <= 1'b0;
            DDRAM_BURSTCNT <= '0; DDRAM_ADDR <= '0;
            DDRAM_DIN <= '0; DDRAM_BE <= '0;
        end
        else begin
            // latch the video request (one outstanding; a second request
            // before service means a dropped line - the prefetch budget is
            // sized so this cannot happen, assert it)
            if (i_lf_req) begin
`ifndef SYNTHESIS
                if (vid_pend || vid_seg != 2'd0)
                    $display("[ddr3_harness] WARNING: video request overrun t=%0t", $time);
`endif
                vid_pend <= 1'b1;
                vid_y    <= i_lf_y;
                vid_x0   <= i_lf_x0;
            end

            // ------------------------------------------------------------
            // read-return queue head bookkeeping (pop side owns oq_rp;
            // pushes only ever move oq_wp)
            // ------------------------------------------------------------
            if (!oq_head_v && (oq_wp != oq_rp)) begin
                oq_left   <= oq[oq_rp][7:0];
                oq_head_v <= 1'b1;
            end
            if (DDRAM_DOUT_READY) begin
`ifndef SYNTHESIS
                if (!oq_head_v || oq_left == 8'd0)
                    $fatal(2, "[ddr3_harness] stray DDRAM read word t=%0t", $time);
`endif
                if (oq_left == 8'd1) begin
                    oq_rp     <= oq_rp + 5'd1;
                    oq_head_v <= 1'b0;           // reload next cycle
                end
                else oq_left <= oq_left - 8'd1;
            end

            // ------------------------------------------------------------
            // DDRAM command face: clear accepted commands (a write accept
            // may be overlaid by a new word in the OWN_BAT branch below)
            // ------------------------------------------------------------
            if (DDRAM_RD && !DDRAM_BUSY) DDRAM_RD <= 1'b0;
            if (DDRAM_WE && !DDRAM_BUSY) DDRAM_WE <= 1'b0;
`ifndef SYNTHESIS
            if (DDRAM_RD && DDRAM_WE)
                $fatal(2, "[ddr3_harness] RD/WE both asserted t=%0t", $time);
`endif

            // ------------------------------------------------------------
            // ownership + command issue
            // ------------------------------------------------------------
            case (own)
            OWN_NONE: begin
                if (vid_pend) begin
                    own      <= OWN_VID;
                    vid_pend <= 1'b0;
                    vid_seg  <= 2'd1;
                end
                else if (bat_busy)  own <= OWN_BAT;
                else if (i_nd_req)  own <= OWN_ND;
            end

            OWN_VID: begin
                // issue burst 1 then (if the window wraps) burst 2; release
                // when both are queued - returns route by tag
                if (!DDRAM_RD || !DDRAM_BUSY) begin
                    if (vid_seg == 2'd1 && !DDRAM_RD) begin
                        DDRAM_RD       <= 1'b1;
                        DDRAM_ADDR     <= VRAM_BASE_W
                                          + {6'd0, vid_y, vid_w0};
                        DDRAM_BURSTCNT <= vid_len1;
                        oq[oq_wp] <= {OWN_VID, vid_len1};
                        oq_wp <= oq_wp + 5'd1;
                        vid_seg <= (vid_len2 != 8'd0) ? 2'd2 : 2'd3;
                    end
                    else if (vid_seg == 2'd2 && !DDRAM_RD) begin
                        DDRAM_RD       <= 1'b1;
                        DDRAM_ADDR     <= VRAM_BASE_W + {6'd0, vid_y, 11'd0};
                        DDRAM_BURSTCNT <= vid_len2;
                        oq[oq_wp] <= {OWN_VID, vid_len2};
                        oq_wp <= oq_wp + 5'd1;
                        vid_seg <= 2'd3;
                    end
                    else if (vid_seg == 2'd3 && !DDRAM_RD) begin
                        vid_seg <= 2'd0;
                        own     <= OWN_NONE;
                    end
                end
            end

            OWN_BAT: begin
                // read command intake (one splitter register; the batch's
                // cmd stream keeps it fed so bursts issue back-to-back)
                if (i_prd_req && o_prd_rdy) begin
                    bp_v    <= 1'b1;
                    bp_addr <= VRAM_BASE_W + {6'd0, i_prd_addr};
                    bp_left <= i_prd_len;
                end
                if (bp_v && !DDRAM_RD && !DDRAM_WE && oq_room) begin
                    DDRAM_RD       <= 1'b1;
                    DDRAM_ADDR     <= bp_addr;
                    DDRAM_BURSTCNT <= bp_burst;
                    oq[oq_wp] <= {OWN_BAT, bp_burst};
                    oq_wp <= oq_wp + 5'd1;
                    bp_addr <= bp_addr + {21'd0, bp_burst};
                    bp_left <= bp_left - {3'd0, bp_burst};
                    if (bp_left <= BL_MAX) bp_v <= 1'b0;
                end
                // posted write words (BURSTCNT=1)
                if (i_pwr_req && o_pwr_rdy) begin
                    DDRAM_WE       <= 1'b1;
                    DDRAM_ADDR     <= VRAM_BASE_W + {6'd0, i_pwr_addr};
                    DDRAM_BURSTCNT <= 8'd1;
                    DDRAM_DIN      <= i_pwr_data;
                    DDRAM_BE       <= {{2{i_pwr_be[3]}}, {2{i_pwr_be[2]}},
                                       {2{i_pwr_be[1]}}, {2{i_pwr_be[0]}}};
                end
                // release between trains (never mid-train)
                if (!bat_busy && !DDRAM_RD && !DDRAM_WE)
                    own <= OWN_NONE;
            end

            OWN_ND: begin
                if (i_nd_req && o_nd_rdy) begin
                    np_v    <= 1'b1;
                    np_addr <= i_nd_addr;
                    np_left <= i_nd_len;
                end
                if (np_v && !DDRAM_RD && oq_room) begin
                    DDRAM_RD       <= 1'b1;
                    DDRAM_ADDR     <= np_addr;
                    DDRAM_BURSTCNT <= np_burst;
                    oq[oq_wp] <= {OWN_ND, np_burst};
                    oq_wp <= oq_wp + 5'd1;
                    np_addr <= np_addr + {21'd0, np_burst};
                    np_left <= np_left - {3'd0, np_burst};
                    if (np_left <= BL_MAX) np_v <= 1'b0;
                end
                if (!np_v && !i_nd_req && !DDRAM_RD)
                    own <= OWN_NONE;
            end

            default: own <= OWN_NONE;
            endcase
        end
    end

endmodule
`default_nettype none
