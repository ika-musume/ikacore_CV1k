`default_nettype none
//============================================================================
// blit_fetch.sv - CV1000-B blitter op-list fetch unit          [H2 / I-4.1+I-2.4]
//
// On EXEC, becomes SH-3 bus master (BREQ/BACK, SH7709S fig 10.41) and DMA-
// fetches the op list from U1 work-RAM in 64-byte chunks, pushing the 16-bit
// op words (big-endian order, exactly the gfx_create_shadow_copy stream) into
// the attribute FIFO.  One BREQ tenure per chunk: request, read 16 longwords,
// precharge-all, release - the CPU breathes between chunks, which is the
// authentic bus-steal pattern.  Chunk start-to-start spacing is the protocol
// cadence, runtime-loadable from the H4 governor's tables (provisional until
// the board rig measures the real gaps):
//   T_EXEC2BRQ ~200 ns   = i_exec2brq_ckio  (P-22)
//   T_CHUNK    ~700 ns   = i_chunk_ckio     attribute chunks
//   T_CHUNK_UP ~1442.5ns = i_upld_ckio      chunks consumed by an UPLOAD op
//
// H4 additions: i_hold stalls the next ATTRIBUTE chunk while the governor's
// window (surviving-draw chunks in flight vs governed op starts) is full --
// upload-fill chunks bypass the hold (payload streams through, never holding
// a virtual FIFO slot; fifo_study drainB semantics).  o_snoop_push/word tap
// the FIFO push stream so the governor sees every op word at its REAL
// arrival time (timing plane = f(op list) only).
//
// The embedded walker parses op framing only (never interprets fields):
//   0x0---/0xF--- END (1w)  0xC--- CLIP (2w)  0x1--- DRAW (10w)
//   0x2--- UPLOAD (8w + dimx*dimy payload;  dimx=(w6&0x1fff)+1, dimy=(w7&0xfff)+1)
// It stops the fetch after the chunk containing END, gates FIFO pushes past
// END, and selects the upload chunk cadence while an UPLOAD is being filled
// (mirrors cost_model.h fetch_ready_times).
//
// SDRAM mastering (U1 MT48LC2M32B2, matches the CPU BSC's boot programming -
// MCR=0x543C: AMX 0111, TRCD=2, TPC=2; SDMR write 0xFFFFE880 -> mode 0x220 =
// CL2 / BL1 / burst-read-single-write):
//   bank = addr[22:21] on pins A[14:13],  row = addr[20:10] on pins A[12:2]
//   column phase: pins A[12:2] = {AP, addr[11:2]}  (device A10 = pin A12)
//   per-word READs issued back-to-back (BL=1), data captured CL=2 CKIO later;
//   rows left fully precharged (PALL) before every release - the BSC precharges
//   all banks before granting (E_BRQ_PALL) and assumes closed rows on regain.
//
// Attribute FIFO: FIFO_WORDS x 16 bit.  Frozen sizing (fifo_study 2026-07-13,
// 8 games / 47 M draws): governed-fetch backlog needs 513 chunks worst-case
// (ddpsdoj slowdown exec, window D=512) -> physical depth 640 chunks
// (20,480 words, 40 KB) = window + engine-lag slack.  A new chunk is only
// scheduled with >= 1 chunk of FIFO headroom (backpressure = safety net; it
// never engages on the trace corpus).
//
// CKIO discipline: all bus-facing state advances on i_CLK @ i_CKIO_PCEN (no
// derived clocks); the walker/FIFO run at full i_CLK rate (internal domain).
//============================================================================
module blit_fetch #(
    parameter int unsigned CAS_LAT       = 2,      // boot SDMR: CL2
    parameter int unsigned FIFO_WORDS    = 20480   // 640 chunks (fifo_study)
)(
    input  wire        i_CLK,
    input  wire        i_CKIO_PCEN,      // pulses the i_CLK cycle CKIO rises
    input  wire        i_RST_n,

    // kick from blit_regs
    input  wire        i_exec,           // 1-cycle pulse (i_CLK domain)
    input  wire [28:0] i_list_addr,      // byte address of the op list (CS3)

    // H4 governor pacing (runtime tables in blit_gov; P-20/21/22)
    input  wire [7:0]  i_exec2brq_ckio,  // EXEC write -> first BREQ, CKIO
    input  wire [7:0]  i_chunk_ckio,     // attribute chunk cadence, CKIO
    input  wire [7:0]  i_upld_ckio,      // upload chunk cadence, CKIO
    input  wire        i_hold,           // governed-window backpressure

    // SH-3 bus arbitration
    output reg         o_BREQ_n,
    input  wire        i_BACK_n,

    // shared-bus master drive (top muxes these onto U1 while o_bus_drive)
    output reg         o_bus_drive,      // 1 = we own the bus and drive pins
    output reg  [25:0] o_A,
    output reg         o_CS_n,
    output reg         o_RAS_n,
    output reg         o_CAS_n,
    output reg         o_WE,             // SDRAM WE_n (the RD_WR pin)
    output reg  [3:0]  o_DQM,
    input  wire [31:0] i_D,              // resolved shared data bus

    // attribute FIFO consumer (i_CLK domain)
    output wire        o_fifo_valid,
    output wire [15:0] o_fifo_word,
    input  wire        i_fifo_pop,

    // H4 governor arrival snoop: every word pushed into the FIFO, at push
    // time (= real bus arrival, +skid latency of a few i_CLK)
    output wire        o_snoop_push,
    output wire [15:0] o_snoop_word,

    // status
    output reg         o_busy,           // EXEC latched .. END chunk done
    output reg         o_done,           // 1-cycle pulse when the fetch retires

    // refresh-scheduler sideband (CV1k_sdram_control hidden row maintenance,
    // docs/double_pump_sdram.md section 6.2): while high at a CKIO edge, this
    // unit guarantees it will issue no PALL and no ACT for >= 5 more CKIO
    // cycles (only CAS reads to its already-open bank).  Derived from the
    // train position, so the guarantee holds for short remainder trains too.
    output wire        o_REF_WIN
);

    //------------------------------------------------------------------
    // fetch FSM (CKIO domain)
    //------------------------------------------------------------------
    typedef enum logic [3:0] {
        S_IDLE, S_REQ, S_ACTV, S_RCD, S_RD, S_DRAIN, S_PALL, S_RP, S_GAP
    } state_e;
    state_e     st;

    reg  [28:0] cur_addr;                // byte address of the next word
    reg  [4:0]  chunk_left;              // longwords left in this chunk
    reg  [4:0]  run_left;                // longwords left before a row crossing
    reg  [7:0]  pace_cnt;                // CKIO since this chunk's BREQ
    reg  [7:0]  pace_tgt;
    reg  [1:0]  gap_cnt;                 // tRCD / tRP spacing counter
    reg         back_z;                  // BACK_n sampled on CKIO
    reg         pend_exec;               // EXEC seen while busy
    reg  [28:0] pend_list;

    // CL landing pipeline: bit i set = data lands in i more CKIO cycles
    reg  [2:0]  rd_pipe;
    wire        rd_issue = (st == S_RD);
    wire        cap_now  = rd_pipe[CAS_LAT-1];

    // walker handshake (i_CLK domain, see below)
    wire        end_seen;                // END parsed: no more chunks
    wire        walk_fault;
    wire [15:0] fifo_level;
    wire        fifo_room = (FIFO_WORDS[15:0] - fifo_level) > 16'd40; // >=1 chunk + skid
    wire        upl_fill;                // walker mid-UPLOAD: next chunk at upload cadence

    // 256-column row: longwords to the 1 KB boundary from cur_addr
    wire [8:0]  col_lw      = {1'b0, cur_addr[9:2]};
    wire [8:0]  row_rem_lw  = 9'd256 - col_lw;

    wire        more_chunks = !end_seen && !walk_fault;
    wire        pace_ok     = pace_cnt >= pace_tgt;

    // refresh-window contract (o_REF_WIN header note): with run_left >= 5 the
    // earliest PALL is 4 more S_RD + >= 2 S_DRAIN = 6 CKIO away, and no ACT
    // can precede that PALL.  S_ACTV is deliberately EXCLUDED: the window
    // must not open before the scheduler has parsed this train's ACT (and so
    // knows which bank is ours) off the grid pins.  All state here advances
    // on CKIO_PCEN, so the value is stable across the CKIO cycle.
    assign o_REF_WIN = ((st == S_RCD) || (st == S_RD)) && (run_left >= 5'd5);

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            st          <= S_IDLE;
            o_BREQ_n    <= 1'b1;
            o_bus_drive <= 1'b0;
            o_CS_n      <= 1'b1;
            o_RAS_n     <= 1'b1;
            o_CAS_n     <= 1'b1;
            o_WE        <= 1'b1;
            o_DQM       <= 4'hF;
            o_A         <= 26'd0;
            o_busy      <= 1'b0;
            o_done      <= 1'b0;
            cur_addr    <= 29'd0;
            chunk_left  <= 5'd0;
            run_left    <= 5'd0;
            pace_cnt    <= 8'd0;
            pace_tgt    <= 8'd0;
            gap_cnt     <= 2'd0;
            back_z      <= 1'b1;
            rd_pipe     <= 3'b000;
            pend_exec   <= 1'b0;
            pend_list   <= 29'd0;
        end
        else begin
            o_done <= 1'b0;

            // EXEC capture (i_CLK-domain pulse from blit_regs)
            if (i_exec) begin
                if (!o_busy) begin
                    o_busy   <= 1'b1;
                    pend_exec<= 1'b1;            // consumed by S_IDLE below
                    pend_list<= i_list_addr;
                end
                else begin
                    pend_exec <= 1'b1;           // queue one; games never do this
                    pend_list <= i_list_addr;
                    $display("[blit_fetch] WARNING: EXEC while fetch busy");
                end
            end

            if (i_CKIO_PCEN) begin
                back_z  <= i_BACK_n;
                rd_pipe <= {rd_issue, rd_pipe[2:1]};
                if (pace_cnt != 8'hFF) pace_cnt <= pace_cnt + 8'd1;

                // NOP by default whenever we own the bus (CS deselect)
                if (o_bus_drive) begin
                    o_CS_n  <= 1'b1;
                    o_RAS_n <= 1'b1;
                    o_CAS_n <= 1'b1;
                    o_WE    <= 1'b1;
                end

                case (st)
                    S_IDLE: if (pend_exec) begin
                        pend_exec <= 1'b0;
                        cur_addr  <= pend_list;
                        pace_cnt  <= 8'd1;       // count THIS cycle (spacing = tgt)
                        pace_tgt  <= i_exec2brq_ckio;
                        st        <= S_GAP;      // wait T_EXEC2BRQ, then request
`ifndef SYNTHESIS
                        $display("[blit_fetch] fetch start  list=%08x", {3'b000, pend_list});
`endif
                    end

                    // common pace/gap state: wait pace_tgt, then request a chunk
                    // (governor hold gates attribute chunks only; upload-fill
                    // chunks stream through the window, fifo_study drainB)
                    // back_z gate: BACK deasserts ~2 CKIO after our release;
                    // re-requesting inside that lag reads the STALE grant in
                    // S_REQ and starts a tenure the BSC never gave us - two
                    // masters drive at once (CPU flash fetch vs our chunk;
                    // caught as "Bank already activated" by the ddpsdoj
                    // +blitreplay census, 2026-07-15).  Request only once the
                    // bus has visibly returned to the CPU.
                    S_GAP: if (pace_ok) begin
                        if (more_chunks && fifo_room && back_z && (upl_fill || !i_hold)) begin
                            o_BREQ_n <= 1'b0;
                            pace_cnt <= 8'd1;    // cadence = BREQ-to-BREQ, exactly
                                                 // pace_tgt CKIO (reset-to-0 was a
                                                 // +1 off-by-one, caught by the H4
                                                 // anchor run)
                            pace_tgt <= upl_fill ? i_upld_ckio : i_chunk_ckio;
                            st       <= S_REQ;
                        end
                        else if (!more_chunks) begin
                            o_busy <= 1'b0;
                            o_done <= 1'b1;
                            st     <= S_IDLE;
`ifndef SYNTHESIS
                            $display("[blit_fetch] fetch done   last=%08x%s",
                                     {3'b000, cur_addr},
                                     walk_fault ? " (WALK FAULT)" : "");
`endif
                        end
                        // else: FIFO backpressure - hold the request
                    end

                    S_REQ: if (!back_z) begin    // grant observed (fig 10.41)
                        o_bus_drive <= 1'b1;
                        o_DQM       <= 4'h0;     // read lanes enabled all tenure
                        chunk_left  <= 5'd16;
                        run_left    <= (row_rem_lw >= 9'd16) ? 5'd16
                                                             : row_rem_lw[4:0];
                        st          <= S_ACTV;
                    end

                    S_ACTV: begin                // ACTV bank/row of cur_addr
                        o_CS_n  <= 1'b0;
                        o_RAS_n <= 1'b0;
                        o_CAS_n <= 1'b1;
                        o_WE    <= 1'b1;
                        o_A     <= 26'd0;
                        o_A[14:13] <= cur_addr[22:21];       // BA
                        o_A[12:2]  <= cur_addr[20:10];       // row
                        gap_cnt <= 2'd1;                     // tRCD = 2
                        st      <= S_RCD;
                    end

                    S_RCD: begin
                        if (gap_cnt == 2'd1) st <= S_RD;
                        gap_cnt <= gap_cnt - 2'd1;
                    end

                    S_RD: begin                  // one READ per CKIO (BL=1)
                        o_CS_n  <= 1'b0;
                        o_RAS_n <= 1'b1;
                        o_CAS_n <= 1'b0;
                        o_WE    <= 1'b1;
                        o_A     <= 26'd0;
                        o_A[14:13] <= cur_addr[22:21];       // BA (held)
                        o_A[12]    <= 1'b0;                  // AP = 0
                        o_A[11:2]  <= cur_addr[11:2];        // column
                        cur_addr   <= cur_addr + 29'd4;
                        chunk_left <= chunk_left - 5'd1;
                        run_left   <= run_left - 5'd1;
                        if (run_left == 5'd1)                // row end or chunk end
                            st <= S_DRAIN;
                    end

                    S_DRAIN: begin               // let the CL pipeline land
                        if (rd_pipe == 3'b000) begin
                            o_CS_n  <= 1'b0;     // PALL
                            o_RAS_n <= 1'b0;
                            o_CAS_n <= 1'b1;
                            o_WE    <= 1'b0;
                            o_A[12] <= 1'b1;     // device A10 = all banks
                            o_A[11] <= 1'b1;
                            gap_cnt <= 2'd1;     // tRP = 2
                            st      <= S_PALL;
                        end
                    end

                    S_PALL: begin                // (state consumes the PALL cycle)
                        if (gap_cnt == 2'd1) st <= S_RP;
                        gap_cnt <= gap_cnt - 2'd1;
                    end

                    S_RP: begin
                        if (chunk_left != 5'd0) begin        // row crossing: reopen
                            run_left <= (chunk_left <= 5'd16) ? chunk_left : 5'd16;
                            st       <= S_ACTV;
                        end
                        else begin                           // chunk done: hand back
                            o_bus_drive <= 1'b0;
                            o_DQM       <= 4'hF;
                            o_BREQ_n    <= 1'b1;
                            st          <= S_GAP;
                        end
                    end

                    default: st <= S_IDLE;
                endcase
            end
        end
    end

    // row-crossing note: run_left counts to the 1 KB row boundary; the
    // mid-chunk PALL/ACTV pair above reopens the next row and continues the
    // same tenure (only possible when LIST_ADDR is not 64 B aligned).

    //------------------------------------------------------------------
    // captured-longword skid queue (CKIO capture -> i_CLK walker)
    //------------------------------------------------------------------
    reg  [31:0] skid    [0:7];
    reg  [2:0]  skid_wp, skid_rp;
    reg  [3:0]  skid_lvl;
    reg         skid_half;               // 0 = next word is [31:16] (BE first)

    wire        skid_pop_word;           // walker consumes one 16-bit word

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            skid_wp  <= 3'd0;
            skid_lvl <= 4'd0;
        end
        else begin
            if (i_CKIO_PCEN && cap_now) begin
                skid[skid_wp] <= i_D;
                skid_wp       <= skid_wp + 3'd1;
                skid_lvl      <= skid_lvl + 4'd1
                                 - ((skid_pop_word && skid_half) ? 4'd1 : 4'd0);
`ifndef SYNTHESIS
                if (skid_lvl >= 4'd8) $fatal(1, "[blit_fetch] skid overflow");
`endif
            end
            else if (skid_pop_word && skid_half)
                skid_lvl <= skid_lvl - 4'd1;
        end
    end

    wire [31:0] skid_lw    = skid[skid_rp];
    wire        skid_avail = (skid_lvl != 4'd0);
    wire [15:0] skid_word  = skid_half ? skid_lw[15:0] : skid_lw[31:16];

    //------------------------------------------------------------------
    // op-framing walker (i_CLK domain): one word per cycle out of the skid,
    // through the framing parser, into the attribute FIFO
    //------------------------------------------------------------------
    typedef enum logic [1:0] { W_HDR, W_BODY, W_END, W_FAULT } wstate_e;
    wstate_e    wst;
    reg  [25:0] w_need;                  // words left in the current op
                                         // (max payload 8192*4096 = 2^25)
    reg  [3:0]  w_idx;                   // header word index (UPLOAD dims)
    reg         w_upl;                   // current op is an UPLOAD
    reg  [12:0] w_dimx;

    assign end_seen   = (wst == W_END);
    assign walk_fault = (wst == W_FAULT);
    assign upl_fill   = w_upl && (wst == W_BODY);

    // FIFO storage
    reg  [15:0] fmem [0:FIFO_WORDS-1];
    reg  [15:0] f_wp, f_rp, f_lvl;
    assign fifo_level   = f_lvl;
    assign o_fifo_valid = (f_lvl != 16'd0);
    assign o_fifo_word  = fmem[f_rp];

    wire fifo_free      = (f_lvl != FIFO_WORDS[15:0]);
    assign skid_pop_word = skid_avail && fifo_free && (wst != W_FAULT);
    wire fifo_push      = skid_pop_word && (wst != W_END);

    // governor arrival snoop (includes the END word - pushed while wst is
    // still W_HDR - and upload payload; nothing after END/fault)
    assign o_snoop_push = fifo_push;
    assign o_snoop_word = skid_word;

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            wst       <= W_HDR;
            w_need    <= 26'd0;
            w_idx     <= 4'd0;
            w_upl     <= 1'b0;
            w_dimx    <= 13'd0;
            skid_half <= 1'b0;
            skid_rp   <= 3'd0;
            f_wp      <= 16'd0;
            f_rp      <= 16'd0;
            f_lvl     <= 16'd0;
        end
        else begin
            // reset parser at fetch start (new list)
            if (i_exec && !o_busy) begin
                wst       <= W_HDR;
                skid_half <= 1'b0;
            end

            if (skid_pop_word) begin
                skid_half <= ~skid_half;
                if (skid_half) skid_rp <= skid_rp + 3'd1;

                // framing parser
                case (wst)
                    W_HDR: begin
                        w_idx <= 4'd1;
                        case (skid_word[15:12])
                            4'h0, 4'hF: wst <= W_END;                 // END word
                            4'hC: begin w_need <= 26'd1;  w_upl <= 1'b0; wst <= W_BODY; end
                            4'h1: begin w_need <= 26'd9;  w_upl <= 1'b0; wst <= W_BODY; end
                            4'h2: begin w_need <= 26'd7;  w_upl <= 1'b1; wst <= W_BODY; end
                            default: begin
                                wst <= W_FAULT;
`ifndef SYNTHESIS
                                $display("[blit_fetch] WALK FAULT: op %04x", skid_word);
`endif
                            end
                        endcase
                    end
                    W_BODY: begin
                        w_idx  <= (w_idx == 4'hF) ? 4'hF : w_idx + 4'd1;
                        w_need <= w_need - 26'd1;
                        if (w_upl && w_idx == 4'd6)       // header word 6: dimx
                            w_dimx <= skid_word[12:0];
                        if (w_upl && w_idx == 4'd7)       // word 7: dimy -> payload
                            w_need <= (26'(w_dimx) + 26'd1) *
                                      (26'({14'd0, skid_word[11:0]}) + 26'd1);
                        else if (w_need == 26'd1)
                            wst <= W_HDR;
                    end
                    W_END, W_FAULT: ;                     // hold (pops drain skid)
                    default: ;
                endcase
            end

            // FIFO push/pop
            if (fifo_push) begin
                fmem[f_wp] <= skid_word;
                f_wp       <= (f_wp == FIFO_WORDS[15:0] - 16'd1) ? 16'd0
                                                                 : f_wp + 16'd1;
            end
            if (i_fifo_pop && o_fifo_valid)
                f_rp <= (f_rp == FIFO_WORDS[15:0] - 16'd1) ? 16'd0
                                                           : f_rp + 16'd1;
            f_lvl <= f_lvl + (fifo_push ? 16'd1 : 16'd0)
                           - ((i_fifo_pop && o_fifo_valid) ? 16'd1 : 16'd0);
        end
    end

endmodule
`default_nettype none
