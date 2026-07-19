`default_nettype none
//============================================================================
// blit_draw.sv - CV1000-B blitter draw engine              [H3 / I-1.5/6/7]
//
// Consumes the 16-bit op-word stream from the attribute FIFO (blit_fetch in
// the board sim, a feeder in the trace TB) and executes it against VRAM:
//   DRAW   src read -> tint -> blend -> trans -> dst write, 4 px/clk
//   UPLOAD payload write path (1 px/clk - FIFO-rate-bound by nature)
//   CLIP   window = clip origin ±32 margin, or full VRAM
//   END    retire, pulse o_done
//
// Pixel semantics are a bit-exact port of the H1 golden model
// (sim/blitgold/golden.h = MAME cv1k_v), including its bug-for-bug quirks:
//   - dmode2 (add_with_clr_square) uses clr0.R as the add's first index for
//     the G and B channels too
//   - flat dst indexing didx = dy*8192+dx: a clip-margin dx<0 with dy>=1
//     wraps into the previous row (MAME bitmap.pix row underflow); any didx
//     outside [0,2^25) is skipped per pixel
//   - the src 0x2000-edge wrap guard rejects the whole draw
//   - blend smode0/dmode4 with s_alpha=d_alpha=0x1f collapses to a copy
// Blend LUTs are computed, not stored: colrtable[x][y]=min(x*y/31,31) with
// the exact floor(/31) as a *2115>>16 reciprocal (exhaustively equal over
// the 0..1953 product range); colrtable_rev = same with x^0x1f;
// colrtable_add = min(x+y,31).  The 64-way s/d-mode switch reduces to one
// regular form: out = add(clr0, dterm) where
//   clr0  = mode[1:0]==3 ? s : mulop(sel{alpha,s,d}, s, rev=mode[2])
//   dterm = mode[1:0]==3 ? d : mulop(sel{alpha,s,d}, d, rev=mode[2])
// (s already tinted; verified case-by-case against golden.h blend_combine).
//
// VRAM interface: three channels in FLAT PIXEL addresses (px = y*8192+x,
// 25 bit), beats of 4 consecutive 16-bit ARGB1555 pixels ({px3..px0} at
// addr+3..addr, addresses wrapping mod 2^25).  Reads have a FIXED contract
// (blit_vram_beh): data appears on i_*rd_data the cycle after the o_*rd_req
// cycle and HOLDS until the next request.  Writes carry a per-pixel lane
// mask - byte-enable semantics, never read-modify-write (pre-H2 punch item
// 2).  The real DDR3 backend (I-4.3) puts an adapter with a ready/valid
// skid behind these channels; the engine itself never throttles - native
// speed, per the two-plane architecture (authentic timing is the H4
// governor's job, never the datapath's).
//
// Performance structure (why not one big sequential FSM): a decode-ahead
// FRONT assembles the next op from the FIFO while the BACK's pixel pipe
// draws the current one, rows chain with zero bubbles (the ±8192 row step
// mod 2^25 implements the &0xfff src-row wrap for free), and consecutive
// ops stream through the pipe back-to-back unless a conservative rect test
// finds a real hazard: next.src or (if blending) next.dst overlapping the
// previous op's dst -> drain once at op start; src overlapping the op's OWN
// dst -> strict mode, one beat at a time write-before-next-read (and if the
// x shift is inside a beat, or flipped, one PIXEL at a time) reproducing
// golden's sequential row-major smear exactly.  Mode state is double-banked
// so two ops' beats can coexist in the pipe.
//============================================================================
module blit_draw (
    input  wire        i_CLK,
    input  wire        i_RST_n,

    // kick + shadow clip origin from blit_regs (i_CLK domain)
    input  wire        i_exec,           // 1-cycle pulse
    input  wire [15:0] i_clip_x,
    input  wire [15:0] i_clip_y,

    // attribute FIFO consumer
    input  wire        i_fifo_valid,
    input  wire [15:0] i_fifo_word,
    output wire        o_fifo_pop,

    // VRAM src read channel (fixed-latency hold contract, see header)
    output wire        o_srd_req,
    output wire [24:0] o_srd_addr,
    input  wire [63:0] i_srd_data,

    // VRAM dst read channel (blend only)
    output wire        o_drd_req,
    output wire [24:0] o_drd_addr,
    input  wire [63:0] i_drd_data,

    // read-stall (H7 / I-4.3): joins the pipe-advance term.  The H3 fixed-
    // latency read contract survives behind blit_batch only if the batch
    // layer can hold the pipe when a read would miss its staging BRAM (or,
    // for strict-mode ops, until the per-beat port round-trip completes).
    // At most ONE read per channel is outstanding (issue at B1, data
    // latched at the next advancing edge), so deasserting i_rd_vld until
    // the last accepted request's data is presented is the whole protocol.
    // Tie 1'b1 for the H0-H6 configuration: the netlist behavior is then
    // bit-identical (H6 re-run is the accept for this port).
    input  wire        i_rd_vld,

    // VRAM write channel (per-pixel lane enables, no RMW)
    output wire        o_wr_req,
    output wire [24:0] o_wr_addr,
    output wire [63:0] o_wr_data,
    output wire [3:0]  o_wr_mask,
    input  wire        i_wr_rdy,

    // -----------------------------------------------------------------
    // descriptor sideband (H7 / I-4.3): OUTPUT-ONLY taps of what the BACK
    // setup already computes - no datapath, state or timing change.  Drives
    // blit_batch's K=8-objline address generation; the beat streams above
    // remain the data path.  o_dsc_vld pulses one cycle when a SURVIVING
    // DRAW is committed to the beat generator (B_S3 -> B_ROW); the draw
    // fields then hold until the next op's setup.  o_dsc_upl pulses at
    // UPLOAD streaming start (fields valid in the pulse cycle).  Ops that
    // are clipped out / wrap-rejected emit nothing (they issue no beats).
    // -----------------------------------------------------------------
    output reg         o_dsc_vld,        // 1-cycle pulse: DRAW committed
    output wire [12:0] o_dsc_sx_lo,      // src x span lo (masked coords)
    output wire [12:0] o_dsc_sx_hi,      // src x span hi
    output wire [11:0] o_dsc_sy0,        // first src row (masked, walk +/-1 mod 4096)
    output wire [12:0] o_dsc_rows,       // surviving rows (1..4096)
    output wire [13:0] o_dsc_npx,        // px per row after clip (1..8192)
    output wire [31:0] o_dsc_dst0,       // dst row-0 flat index (signed; +8192/row)
    output wire        o_dsc_flipx,      // beat walk descends within the row
    output wire        o_dsc_flipy,      // src row walk descends
    output wire        o_dsc_blend,      // dst read beats will issue (q_blend_eff)
    output wire        o_dsc_strict,     // self-overlap: serialized fallback
    output wire        o_dsc_px1,        // 1-px beats (smear inside a 4-px beat)
    output wire        o_dsc_wait,       // cross-op hazard: writes must land first
    output reg         o_dsc_upl,        // 1-cycle pulse: UPLOAD streaming starts
    output wire [24:0] o_dsc_upl_addr,   // upload dst base (flat; +8192/row mod 2^25)
    output wire [13:0] o_dsc_upl_dimx,   // words per row
    output wire [12:0] o_dsc_upl_dimy,   // rows

    output wire        o_busy,           // EXEC accepted .. END retired
    output reg         o_done,           // 1-cycle pulse at END retire

    // r4: raw request legs -- the registered components of o_srd_req /
    // o_drd_req, exported so a backend that owns the other AND legs
    // (wr-fifo almost-full, rd_vld, scanout steal) can rebuild the
    // requests locally instead of timing the two-crossing detour through
    // this module's adv LUT.  o_srd_req == o_rq_v && adv by definition.
    output wire        o_rq_v,           // = b1_v
    output wire        o_rq_wr,          // = draw_wr (b4 write beat pending)
    output wire        o_rq_blend        // = b1_blend (dst read leg)
);

    // ---------------------------------------------------------------------
    // blend arithmetic primitives (== golden.h BlendTables, computed)
    // ---------------------------------------------------------------------
    // colrtable[x][y] (rev=0) / colrtable_rev[x][y] (rev=1), x 5-bit, y up
    // to 6-bit (tint).  floor(p/31) over p<=1953 as (p*2115)>>16 - exact
    // (verified exhaustively); then min(.,31).
    function automatic logic [4:0] f_mulop(input logic [4:0] x,
                                           input logic [5:0] y,
                                           input logic       rev);
        logic [4:0]  xe;
        logic [10:0] p;
        logic [22:0] q;
        xe = rev ? ~x : x;                 // 31-x = x^0x1f
        p  = xe * y;                       // <= 31*63 = 1953
        q  = p * 12'd2115;                 // exact floor(p/31) in q[22:16]
        f_mulop = (|q[22:21]) ? 5'd31 : q[20:16];   // min(q>>16, 31)
    endfunction

    // r4: f_mulop split at the first-product boundary for the ALU2 pipe
    // cut -- f_mulop(x,y,rev) == f_mulop_b(f_mulop_a(x,y,rev)) by
    // construction (same expressions, no truncation between the halves)
    function automatic logic [10:0] f_mulop_a(input logic [4:0] x,
                                              input logic [5:0] y,
                                              input logic       rev);
        logic [4:0] xe;
        xe = rev ? ~x : x;
        f_mulop_a = xe * y;
    endfunction

    function automatic logic [4:0] f_mulop_b(input logic [10:0] p);
        logic [22:0] q;
        q = p * 12'd2115;
        f_mulop_b = (|q[22:21]) ? 5'd31 : q[20:16];
    endfunction

    // r4 iter 2: constant-folded tint form for ALU1 -- the per-op tint y is
    // a bank constant, so y*2115 is precomputed at bank-program time (18b:
    // 63*2115 = 133,245) and the live path is ONE 5x18 multiply + clamp
    // instead of the two serial multiplies of f_mulop.  Exact by integer
    // associativity: (x*y)*2115 == x*(y*2115), both <= 31*133,245 =
    // 4,130,595 < 2^23 -- the elaboration self-check below proves it
    // exhaustively over the full 32x64 domain every sim build.
    function automatic logic [4:0] f_mulop_k(input logic [4:0]  x,
                                             input logic [17:0] k);
        logic [22:0] q;
        q = x * k;
        f_mulop_k = (|q[22:21]) ? 5'd31 : q[20:16];
    endfunction

`ifndef SYNTHESIS
    initial begin
        for (int xi = 0; xi < 32; xi++)
            for (int yi = 0; yi < 64; yi++)
                if (f_mulop(5'(xi), 6'(yi), 1'b0)
                    != f_mulop_k(5'(xi), 18'(yi * 2115)))
                    $fatal(2, "[blit_draw] f_mulop_k mismatch x=%0d y=%0d",
                           xi, yi);
    end
`endif

    function automatic logic [4:0] f_satadd(input logic [4:0] a,
                                            input logic [4:0] b);
        logic [5:0] s;
        s = {1'b0, a} + {1'b0, b};
        f_satadd = s[5] ? 5'd31 : s[4:0];
    endfunction

    // ---------------------------------------------------------------------
    // FRONT / BACK / pipe shared declarations
    // ---------------------------------------------------------------------
    // exec pending latch
    reg        pend_v;
    reg [15:0] pend_cx, pend_cy;
    reg        running;

    // clip window (FRONT-owned; snapshot into the op buffer per DRAW).
    // Signed 18-bit: clip origin is a u16, margins can push past 0/65535.
    reg signed [17:0] fc_min_x, fc_max_x, fc_min_y, fc_max_y;
    reg [15:0]        cw_w1q;    // r5: deferred CLIP w1 (see F_CW)
    reg               cw_pend;
    reg        [15:0] exec_cx, exec_cy;   // origin latched at EXEC (CLIP re-arms from it)

    // op buffer: FRONT commits one decoded DRAW, BACK consumes.
    reg               ob_v;
    reg        [2:0]  ob_smode, ob_dmode;
    reg               ob_trans, ob_blend, ob_flipx, ob_flipy;
    reg        [4:0]  ob_sa, ob_da;
    reg        [5:0]  ob_tr, ob_tg, ob_tb;
    reg        [12:0] ob_src_x;
    reg        [11:0] ob_src_y;
    reg signed [17:0] ob_dst_x, ob_dst_y;
    reg        [13:0] ob_dimx, ob_dimy;
    reg signed [17:0] ob_cminx, ob_cmaxx, ob_cminy, ob_cmaxy;

    // previous-op dst rect (clipped, conservatively row-spill-expanded) for
    // the cross-op hazard test; upload sets it too.
    reg signed [17:0] pv_xlo, pv_xhi, pv_ylo, pv_yhi;
    reg               pv_valid;

    // pixel pipe (B1 issue -> B2 data-on-bus -> B2r raw capture -> B3
    // ALU1 regs -> B4 ALU2/write).  H7b.8c latency step (+1 stage, user
    // latitude 2026-07-17 -- datapath speed unthrottled, write lands one
    // fast cycle later): the read returns are captured RAW into the B2r
    // registers at the same adv edge that used to consume them comb, and
    // ALU1 (tint) now evaluates register->register from B2r into B3 --
    // the staging-RAM Tco + route no longer feeds the tint multiplier
    // cone (fit #8 c153 worst, -6.8).  ALU2 (blend) is unchanged from
    // the B3 registers into the B4 data register that drives the write
    // port directly.  Per-pixel values are identical (same read data,
    // same mode banks -- s3_bank_clear covers the extra stage), only the
    // write instant shifts +1 fast cycle; the batch train protocol is
    // descriptor-counted and order-preserving, so DES behavior is
    // unchanged (proven by datum + matrix).
    reg         b1_v, b2_v, b2r_v, b3_v, b4_v;
    reg         b1_bk, b2_bk, b2r_bk, b3_bk, b4_bk;
    // r6: the 3-deep px1/wa/en staging chains are pure 1:1 delays, which
    // shift-register recognition rebuilt as ONE M10K altshift_taps -- the
    // pipe context bits then LAUNCH from a RAM block's Tco + inter-block
    // route into the ALU1 cone (r5 ship -2.42 family).  preserve pins
    // them as fabric registers beside their consumers; ~60 FFs, free.
    (* preserve *) reg b1_px1, b2_px1;
    reg         b2r_px1;                       // 1-pixel beat (strict smear mode)
    reg  [24:0] b1_sa_;
    (* preserve *) reg [24:0] b1_wa, b2_wa;
    reg  [24:0] b2r_wa, b3_wa, b4_wa;
    (* preserve *) reg [3:0]  b1_en, b2_en;
    reg  [3:0]  b2r_en, b3_mask, b4_mask;
    reg  [63:0] b2r_s, b2r_d;                  // captured src / dst read beats
    // r7 NOTE: a B2m product stage here (ALU1 multiply cut at its own
    // capture bank, permute behind it, pipe +1) was BUILT and REVERTED --
    // the datum stayed byte-identical but the anchor gauntlet caught the
    // gov coupling: pipe_empty gates strict-beat emission and the upload
    // F_UPW/F_END serialization, so the deeper pipe stretches REAL engine
    // completion into the gov-observable timeline (busy_end/deassert/
    // chunk gaps drifted vs the r5i5d baseline; all 11 anchors still
    // PASSed their bands).  A future +1 on this path needs an explicit
    // anchor re-baseline decision, not latitude.
    // H7b.8e: B3i is a plain 1:1 delay stage between ALU1 and ALU2 (values
    // identical, one cycle later) -- the two serial 5x6 / x2115 multiplies
    // of the blend algebra span more than one period, and this register
    // gives the synthesis retimer a boundary to spread them across.  Pipe
    // depth 5 -> 6: hazard windows below carry the extra stage.
    reg         b3i_v;
    reg         b3i_bk;
    reg  [24:0] b3i_wa;
    reg  [3:0]  b3i_mask;
    reg  [63:0] b3i_raw;
    reg  [3:0][4:0] b3i_sr, b3i_sg, b3i_sb;
    reg  [3:0][4:0] b3i_dr, b3i_dg, b3i_db;
    reg  [3:0]      b3i_a;
    reg  [63:0] b3_raw;                        // post-flip raw src px (copy path)
    reg  [3:0][4:0] b3_sr, b3_sg, b3_sb;       // tinted src channels
    reg  [3:0][4:0] b3_dr, b3_dg, b3_db;       // dst channels
    reg  [3:0]      b3_a;                      // src A bits
    reg  [63:0] b4_data;                       // blended write beat

    // double-banked per-op mode state (two ops may coexist in the pipe)
    reg  [1:0]      bk_simple, bk_blend, bk_tint, bk_trans, bk_flip;
    reg  [1:0][2:0] bk_smode, bk_dmode;
    reg  [1:0][4:0] bk_sa, bk_da;
    reg  [1:0][17:0] bk_ktr, bk_ktg, bk_ktb;  // r4 iter2: tint*2115 (K-form)

    // H7b.8e r4: stage-local copies of the mode-bank fields, fetched at the
    // same adv edge that advances the op into the consuming stage (index =
    // the PREVIOUS stage's bank).  s3_bank_clear forbids reprogramming a
    // bank while any pipe stage still references it, so these copies are
    // value-identical to the direct bk_*[stage_bk] reads they replace; the
    // ALU cones become pure register->register and the synthesis retimer
    // can no longer drag the B_S3 bank-write/enable cone (dimx_e/dimy_e
    // compares) into the blend multiplier when it splits ALU2 across B3.
    reg             b1_blend;                   // o_drd_req leg
    reg             b2r_flip, b2r_tint, b2r_trans;
    reg  [17:0]     b2r_ktr, b2r_ktg, b2r_ktb; // K-form tint constants
    reg  [2:0]      b3i_sm, b3i_dm;             // ALU2a selects (B3i stage)
    reg  [4:0]      b3i_sac, b3i_dac;           // alpha constants (B3i stage)
    reg  [2:0]      b3_sm, b3_dm;               // carried for ALU2b
    reg             b3_simple, b3_blendf;

    // H7b.8e r4: ALU2 is hand-split across the existing B3i->B3 boundary
    // instead of leaving the whole blend cone to the synthesis retimer.
    // The retimer's automatic split (fit evidence: Add3~13_NEW_REG inside
    // the soft multiplier) must RECONSTRUCT every moved register's next
    // value -- adv ? D : Q -- which put the adv stall net (o_af arrival
    // from u_batch) and the mode-bank select into the multiplier D-cones
    // as DATA.  With the first multiply registered explicitly (ALU2a:
    // operand select + rev-invert + xe*y products; ALU2b: x2115 const
    // multiply + clamp + saturating add + output select), every B3-stage
    // register is an ordinary enabled register: adv is a clock enable
    // again and no reconstruction LUTs exist.  b4_data values and instants
    // are IDENTICAL (same expressions, cut at a register boundary that
    // was already a pipeline stage).
    reg  [3:0][10:0] b3_ps_r, b3_ps_g, b3_ps_b; // src-leg first products
    reg  [3:0][10:0] b3_pd_r, b3_pd_g, b3_pd_b; // dst-leg first products

    // r5: B3s operand-select stage between B3i and B3 (pipe depth 6 -> 7).
    // The −2.69 residual arc was b3i regs -> operand-select case -> the
    // 5x6 soft multiplier -> b3_ps (the retimer had folded the capture
    // into the lpm padder).  B3s registers the SELECTED AND REV-INVERTED
    // operands (xe/ye) plus every field B3 consumes, so the mult cycle is
    // a pure register-fed xe*y product and the select cycle is two LUTs.
    // Values identical: same expressions, cut at a register; instants +1
    // (the precedented engine-latitude class -- gauntlet-gated).  The
    // bk_simple/bk_blend bank reads stay on the B3i->B3s edge, so the
    // bank-read window is unchanged; the occupancy guards below grow the
    // stage, which extends each bank's protected lifetime accordingly.
    reg             b3s_v;
    reg             b3s_bk;
    reg  [24:0]     b3s_wa;
    reg  [3:0]      b3s_mask;
    reg  [63:0]     b3s_raw;
    reg  [3:0][4:0] b3s_sr, b3s_sg, b3s_sb;
    reg  [3:0][4:0] b3s_dr, b3s_dg, b3s_db;
    reg  [3:0]      b3s_a;
    reg  [2:0]      b3s_sm, b3s_dm;
    reg             b3s_simple, b3s_blendf;
    reg  [3:0][4:0] b3s_xer, b3s_xeg, b3s_xeb;  // ps operand, post-invert
    reg  [3:0][4:0] b3s_yer, b3s_yeg, b3s_yeb;  // pd operand, post-invert

    wire pipe_empty = !b1_v && !b2_v && !b2r_v && !b3i_v && !b3s_v
                      && !b3_v && !b4_v;

    // pipe advance: stall sources are a write beat not accepted and, behind
    // a variable-latency backend (H7), a read not yet served (i_rd_vld=0)
    wire draw_wr    = b4_v && (b4_mask != 4'b0);
    wire adv        = !(draw_wr && !i_wr_rdy) && i_rd_vld;

    // ---------------------------------------------------------------------
    // FRONT: FIFO decode - ops, clip, upload streaming, end
    // ---------------------------------------------------------------------
    localparam [3:0] F_IDLE = 4'd0, F_OP  = 4'd1, F_DW  = 4'd2, F_CW  = 4'd3,
                     F_CMT  = 4'd4, F_UH  = 4'd5, F_UPW = 4'd6, F_UPI = 4'd7,
                     F_UP   = 4'd8, F_END = 4'd9;
    reg [3:0]  fst;
    reg [3:0]  wcnt;
    reg [15:0] w0q, w1q, w2q, w3q, w4q, w5q, w6q, w7q, w8q, w9q;

    // upload streaming regs
    reg [24:0] up_addr, up_row, up_beat;
    reg [13:0] up_x, up_dimx;
    reg [12:0] up_y, up_dimy;
    reg [1:0]  up_lane;
    reg [63:0] up_data;

    wire back_idle;                            // BACK FSM in B_IDLE

    // upload flush: write the beat in the same cycle as its last pop
    wire up_last_in_row = (up_x == up_dimx - 14'd1);
    wire up_flush       = (up_lane == 2'd3) || up_last_in_row;
    wire up_pop_ok      = !up_flush || i_wr_rdy;

    wire pop_want = (fst == F_OP) || (fst == F_DW) || (fst == F_CW) ||
                    (fst == F_UH) || ((fst == F_UP) && up_pop_ok);
    assign o_fifo_pop = pop_want && i_fifo_valid;
    wire   pop_fire   = o_fifo_pop;

    wire up_wr_fire = (fst == F_UP) && pop_fire && up_flush;

    // assembled upload write beat (current word merged at its lane)
    logic [63:0] up_wdata;
    logic [3:0]  up_wmask;
    always_comb begin
        up_wdata = up_data;
        up_wdata[up_lane*16 +: 16] = i_fifo_word;
        case (up_lane)
            2'd0: up_wmask = 4'b0001;
            2'd1: up_wmask = 4'b0011;
            2'd2: up_wmask = 4'b0111;
            2'd3: up_wmask = 4'b1111;
        endcase
    end

    // DRAW field views over the captured words (commit-time decode)
    wire [5:0]  dw_tr    = w8q[7:2];           // tint 8b -> 6b (>>2)  [P-42]
    wire [5:0]  dw_tg    = w9q[15:10];
    wire [5:0]  dw_tb    = w9q[7:2];
    wire [4:0]  dw_sa    = w1q[15:11];         // alpha top-5-of-8     [P-41]
    wire [4:0]  dw_da    = w1q[7:3];

    // upload rect for prev-dst hazard (x spill past 8191 -> expand rows)
    wire signed [17:0] upr_xlo = 18'($signed({5'b0, w4q[12:0]}));
    wire signed [17:0] upr_xhi = upr_xlo + 18'($signed({4'b0, w6q[12:0]})); // +dimx-1
    wire               upr_spill = (upr_xhi > 18'sd8191);
    wire signed [17:0] upr_ylo = 18'($signed({6'b0, w5q[11:0]}));
    wire signed [17:0] upr_yhi = 18'($signed({6'b0, w5q[11:0]}))
                               + 18'($signed({6'b0, w7q[11:0]}))
                               + (upr_spill ? 18'sd1 : 18'sd0);

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            fst      <= F_IDLE;
            pend_v   <= 1'b0;
            running  <= 1'b0;
            ob_v     <= 1'b0;
            o_done   <= 1'b0;
            o_dsc_upl <= 1'b0;
            wcnt     <= 4'd0;
            pend_cx  <= 16'd0;  pend_cy <= 16'd0;
            exec_cx  <= 16'd0;  exec_cy <= 16'd0;
            fc_min_x <= 18'sd0; fc_max_x <= 18'sd8191;
            fc_min_y <= 18'sd0; fc_max_y <= 18'sd4095;
            cw_w1q   <= 16'd0;  cw_pend  <= 1'b0;
            up_addr  <= 25'd0;  up_row  <= 25'd0; up_beat <= 25'd0;
            up_x     <= 14'd0;  up_y    <= 13'd0;
            up_dimx  <= 14'd1;  up_dimy <= 13'd1;
            up_lane  <= 2'd0;   up_data <= 64'd0;
            w0q <= '0; w1q <= '0; w2q <= '0; w3q <= '0; w4q <= '0;
            w5q <= '0; w6q <= '0; w7q <= '0; w8q <= '0; w9q <= '0;
        end
        else begin
            o_done    <= 1'b0;
            o_dsc_upl <= 1'b0;

            if (i_exec) begin
                if (running || pend_v)
                    $display("[blit_draw] WARNING: EXEC while busy (t=%0t)", $time);
                pend_v  <= 1'b1;
                pend_cx <= i_clip_x;
                pend_cy <= i_clip_y;
            end

            // r5: deferred CLIP application (registered compare + enables)
            if (cw_pend) begin
                cw_pend <= 1'b0;
                if (cw_w1q != 16'd0) begin
                    fc_min_x <= 18'($signed({2'b0, exec_cx})) - 18'sd32;
                    fc_max_x <= 18'($signed({2'b0, exec_cx})) + 18'sd351;
                    fc_min_y <= 18'($signed({2'b0, exec_cy})) - 18'sd32;
                    fc_max_y <= 18'($signed({2'b0, exec_cy})) + 18'sd271;
                end
                else begin
                    fc_min_x <= 18'sd0; fc_max_x <= 18'sd8191;
                    fc_min_y <= 18'sd0; fc_max_y <= 18'sd4095;
                end
            end

            case (fst)
            F_IDLE: if (pend_v) begin          // exec start = window clip set
                pend_v   <= 1'b0;
                running  <= 1'b1;
                exec_cx  <= pend_cx;
                exec_cy  <= pend_cy;
                fc_min_x <= 18'($signed({2'b0, pend_cx})) - 18'sd32;
                fc_max_x <= 18'($signed({2'b0, pend_cx})) + 18'sd351;
                fc_min_y <= 18'($signed({2'b0, pend_cy})) - 18'sd32;
                fc_max_y <= 18'($signed({2'b0, pend_cy})) + 18'sd271;
                fst      <= F_OP;
            end

            F_OP: if (pop_fire) begin
                w0q  <= i_fifo_word;
                wcnt <= 4'd1;
                case (i_fifo_word[15:12])
                    4'h0, 4'hf: fst <= F_END;
                    4'hc:       fst <= F_CW;
                    4'h1:       fst <= F_DW;
                    4'h2:       fst <= F_UH;
                    default: begin
                        $display("[blit_draw] FAULT: unknown op %04x (t=%0t)",
                                 i_fifo_word, $time);
                        fst <= F_END;
                    end
                endcase
            end

            F_CW: if (pop_fire) begin          // CLIP: w1!=0 window, ==0 full
                // r5: the fc_* application defers ONE cycle behind a
                // registered copy of w1 -- the popped-word != 0 compare
                // (launched by the fmem RAM/bypass, fit-#3 -2.69) leaves
                // the four 18-bit load-enable fans.  Value-at-consumption
                // is identical: the earliest fc_* read after a CLIP op is
                // the NEXT op's F_CMT capture, >= 11 pops away.
                cw_w1q  <= i_fifo_word;
                cw_pend <= 1'b1;
                fst <= F_OP;
            end

            F_DW: if (pop_fire) begin
                case (wcnt)
                    4'd1: w1q <= i_fifo_word;
                    4'd2: w2q <= i_fifo_word;
                    4'd3: w3q <= i_fifo_word;
                    4'd4: w4q <= i_fifo_word;
                    4'd5: w5q <= i_fifo_word;
                    4'd6: w6q <= i_fifo_word;
                    4'd7: w7q <= i_fifo_word;
                    4'd8: w8q <= i_fifo_word;
                    default: w9q <= i_fifo_word;
                endcase
                wcnt <= wcnt + 4'd1;
                if (wcnt == 4'd9) fst <= F_CMT;
            end

            F_CMT: if (!ob_v) begin            // commit decoded DRAW to BACK
                ob_v     <= 1'b1;
                ob_dmode <= w0q[2:0];
                ob_smode <= w0q[6:4];
                ob_trans <= w0q[8];
                ob_blend <= w0q[9];
                ob_flipy <= w0q[10];
                ob_flipx <= w0q[11];
                ob_sa    <= dw_sa;
                ob_da    <= dw_da;
                ob_tr    <= dw_tr;
                ob_tg    <= dw_tg;
                ob_tb    <= dw_tb;
                ob_src_x <= w2q[12:0];
                ob_src_y <= w3q[11:0];
                ob_dst_x <= 18'($signed(w4q));
                ob_dst_y <= 18'($signed(w5q));
                ob_dimx  <= {1'b0, w6q[12:0]} + 14'd1;
                ob_dimy  <= {2'b0, w7q[11:0]} + 14'd1;
                ob_cminx <= fc_min_x;  ob_cmaxx <= fc_max_x;
                ob_cminy <= fc_min_y;  ob_cmaxy <= fc_max_y;
                fst      <= F_OP;
            end

            F_UH: if (pop_fire) begin
                case (wcnt)
                    4'd1: w1q <= i_fifo_word;
                    4'd2: w2q <= i_fifo_word;
                    4'd3: w3q <= i_fifo_word;
                    4'd4: w4q <= i_fifo_word;
                    4'd5: w5q <= i_fifo_word;
                    4'd6: w6q <= i_fifo_word;
                    default: w7q <= i_fifo_word;
                endcase
                wcnt <= wcnt + 4'd1;
                if (wcnt == 4'd7) fst <= F_UPW;
            end

            F_UPW: if (!ob_v && back_idle && pipe_empty) fst <= F_UPI; // serialize

            F_UPI: begin                       // dst_x &0x1fff, dst_y &0xfff
                up_dimx <= {1'b0, w6q[12:0]} + 14'd1;
                up_dimy <= {1'b0, w7q[11:0]} + 13'd1;
                up_addr <= {w5q[11:0], 13'd0} + {12'd0, w4q[12:0]};
                up_row  <= {w5q[11:0], 13'd0} + {12'd0, w4q[12:0]};
                up_beat <= {w5q[11:0], 13'd0} + {12'd0, w4q[12:0]};
                up_x    <= 14'd0;
                up_y    <= 13'd0;
                up_lane <= 2'd0;
                o_dsc_upl <= 1'b1;             // H7 sideband: upload descriptor
                fst     <= F_UP;
            end

            F_UP: if (pop_fire) begin
                up_data[up_lane*16 +: 16] <= i_fifo_word;
                if (up_flush) up_lane <= 2'd0;
                else          up_lane <= up_lane + 2'd1;
                if (up_last_in_row) begin
                    up_x    <= 14'd0;
                    up_row  <= up_row + 25'd8192;
                    up_addr <= up_row + 25'd8192;
                    up_beat <= up_row + 25'd8192;
                    if (up_y == up_dimy - 13'd1) begin
                        // done: expose the upload rect to the hazard test
                        fst <= F_OP;
                    end
                    else up_y <= up_y + 13'd1;
                end
                else begin
                    up_x    <= up_x + 14'd1;
                    up_addr <= up_addr + 25'd1;
                    if (up_flush) up_beat <= up_addr + 25'd1;
                end
            end

            F_END: if (!ob_v && back_idle && pipe_empty) begin
                o_done  <= 1'b1;
                running <= 1'b0;
                fst     <= F_IDLE;
            end

            default: fst <= F_IDLE;
            endcase

            if (back_ob_take) ob_v <= 1'b0;    // BACK consumed the op
        end
    end

    assign o_busy = running | pend_v;

    // ---------------------------------------------------------------------
    // BACK: per-draw setup + row/beat generator
    // ---------------------------------------------------------------------
    localparam [2:0] B_IDLE = 3'd0, B_S1 = 3'd1, B_S2 = 3'd2, B_S3 = 3'd3,
                     B_ROW  = 3'd4, B_BEAT = 3'd5, B_S3P = 3'd6;
    // r4 iter3: B_S3P precompute registers -- the S3 extent/base/rect
    // automatics land here one cycle ahead (inputs are all S1/S2-or-older
    // registers, stable until the next op), so the S3 commit cycle keeps
    // only the f_ovl compares, the pv_* hazard legs (still read LIVE at
    // the commit edge -- retire updates between S3P and S3 are honored
    // exactly as before), and the bank program.  Engine-side +1 cycle per
    // draw setup (user latitude; datum/anchors/matrix adjudicate).
    reg signed [17:0] p3_npx;
    reg        [12:0] p3_sxb;
    reg        [11:0] p3_ysr;
    reg signed [31:0] p3_drow;
    reg signed [17:0] p3_rows;
    reg signed [17:0] p3_sxlo, p3_sxhi;
    reg signed [17:0] p3_sylo, p3_syhi;
    reg               p3_syfull;
    reg signed [17:0] p3_fxlo, p3_fxhi;
    reg signed [17:0] p3_dylo, p3_dyhi;
    reg               p3_xs_sm;
    reg               p3_rej;   // (starty >= dimy_e) || (startx >= dimx_e)
    reg [2:0] bst;
    assign back_idle = (bst == B_IDLE);
    wire back_ob_take = back_idle && ob_v;

    // latched op (BACK copy; ob_* may be refilled by FRONT immediately)
    reg        [2:0]  q_smode, q_dmode;
    reg               q_trans, q_blend, q_flipx, q_flipy;
    reg        [4:0]  q_sa, q_da;
    reg        [5:0]  q_tr, q_tg, q_tb;
    reg        [12:0] q_src_x;
    reg        [11:0] q_src_y;
    reg signed [17:0] q_dst_x, q_dst_y;
    reg        [13:0] q_dimx, q_dimy;
    reg signed [17:0] q_cminx, q_cmaxx, q_cminy, q_cmaxy;

    // setup intermediates
    reg        [14:0] sx0, sy0;                // flip-adjusted src origin (unmasked)
    reg        [12:0] g_a, g_b;                // wrap-guard masked endpoints
    reg signed [17:0] starty_r, startx_r;      // raw clip starts
    reg signed [17:0] dimy_e, dimx_e;          // trimmed (exclusive) extents
    reg signed [17:0] starty, startx;
    reg signed [17:0] n_px_s;
    reg        [12:0] sx_base;
    reg        [11:0] ysrc0;
    reg signed [31:0] didx_row0;
    reg signed [17:0] rows;

    // effective flags
    reg  q_tint_eff, q_blend_eff, q_simple, q_strict, q_px1, q_waitpipe;
    reg  bk_sel;                               // bank this op will use

    // beat generator regs
    reg  [24:0] src_cur;                       // current beat src read addr
    reg signed [31:0] didx_beat;
    reg signed [15:0] px_left;
    reg  [24:0] srow_sh;                       // next row: src beat-0 addr
    reg signed [31:0] didx_row_sh;
    reg signed [17:0] y_left;

    // per-cycle emit decision
    wire        can_emit_gate = !q_strict || pipe_empty;
    wire        emit_fire = (bst == B_BEAT) && adv && can_emit_gate;
    wire [2:0]  step      = q_px1 ? 3'd1 : 3'd4;
    wire        last_of_row = (px_left <= 16'($signed({13'b0, step})));
    wire        bank_clear = !((b1_v  && (b1_bk  == bk_sel)) ||
                               (b2_v  && (b2_bk  == bk_sel)) ||
                               (b2r_v && (b2r_bk == bk_sel)) ||
                               (b3i_v && (b3i_bk == bk_sel)) ||
                               (b3s_v && (b3s_bk == bk_sel)) ||
                               (b3_v  && (b3_bk  == bk_sel)) ||
                               (b4_v  && (b4_bk  == bk_sel)));
    // S3 writes the mode bank the op is ABOUT to take (~bk_sel): it must
    // not fire while any pipe beat still carries that bank.  Latent since
    // H3 (two fast ops can decode through S3 while an older op's tail
    // beats sit in a stalled pipe - reachable via wr-stall/steal, and
    // readily via H7a rd_vld stalls); B_ROW's bank_clear gate is too late,
    // the programming itself is the clobber.  With an unstalled pipe the
    // target bank drains before any S3 can reach it, so H0-H6 behavior is
    // bit-identical (re-proven by the H6/FASTBOOT reruns).
    wire        s3_bank_clear = !((b1_v  && (b1_bk  == ~bk_sel)) ||
                                  (b2_v  && (b2_bk  == ~bk_sel)) ||
                                  (b2r_v && (b2r_bk == ~bk_sel)) ||
                                  (b3i_v && (b3i_bk == ~bk_sel)) ||
                                  (b3s_v && (b3s_bk == ~bk_sel)) ||
                                  (b3_v  && (b3_bk  == ~bk_sel)) ||
                                  (b4_v  && (b4_bk  == ~bk_sel)));
    wire [2:0]  lane_cnt  = q_px1 ? 3'd1 :
                            (px_left >= 16'sd4) ? 3'd4 : px_left[2:0];

    // per-lane dst flat-index validity (didx in [0, 2^25))
    logic [3:0] lane_ok, lane_en_c;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            automatic logic signed [31:0] di = didx_beat + 32'(l);
            lane_ok[l]   = (di[31:25] == 7'b0);
            lane_en_c[l] = lane_ok[l] && (3'(l) < lane_cnt);
        end
    end

    // rect-overlap helpers computed in setup
    reg signed [17:0] s_xlo, s_xhi, s_ylo, s_yhi;   // src rect (masked coords)
    reg signed [17:0] d_xlo, d_xhi, d_ylo, d_yhi;   // clipped dst rect (spill-expanded)
    reg               s_yfull;

    function automatic logic f_ovl(input logic signed [17:0] alo, ahi, blo, bhi);
        f_ovl = (alo <= bhi) && (blo <= ahi);
    endfunction

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            bst      <= B_IDLE;
            bk_sel   <= 1'b0;
            o_dsc_vld <= 1'b0;
            pv_valid <= 1'b0;
            pv_xlo <= 18'sd0; pv_xhi <= -18'sd1;
            pv_ylo <= 18'sd0; pv_yhi <= -18'sd1;
            {q_smode, q_dmode} <= '0;
            {q_trans, q_blend, q_flipx, q_flipy} <= '0;
            q_sa <= '0; q_da <= '0; q_tr <= '0; q_tg <= '0; q_tb <= '0;
            q_src_x <= '0; q_src_y <= '0; q_dst_x <= '0; q_dst_y <= '0;
            q_dimx <= '0; q_dimy <= '0;
            q_cminx <= '0; q_cmaxx <= '0; q_cminy <= '0; q_cmaxy <= '0;
            q_tint_eff <= 1'b0; q_blend_eff <= 1'b0; q_simple <= 1'b1;
            q_strict <= 1'b0; q_px1 <= 1'b0; q_waitpipe <= 1'b0;
            sx0 <= '0; sy0 <= '0; g_a <= '0; g_b <= '0;
            starty_r <= '0; startx_r <= '0; dimy_e <= '0; dimx_e <= '0;
            starty <= '0; startx <= '0; n_px_s <= '0;
            p3_npx <= '0; p3_sxb <= '0; p3_ysr <= '0; p3_drow <= '0;
            p3_rows <= '0; p3_sxlo <= '0; p3_sxhi <= '0;
            p3_sylo <= '0; p3_syhi <= '0; p3_syfull <= 1'b0;
            p3_fxlo <= '0; p3_fxhi <= '0; p3_dylo <= '0; p3_dyhi <= '0;
            p3_xs_sm <= 1'b0; p3_rej <= 1'b0;
            sx_base <= '0; ysrc0 <= '0; didx_row0 <= '0; rows <= '0;
            src_cur <= '0; didx_beat <= '0; px_left <= '0;
            srow_sh <= '0; didx_row_sh <= '0; y_left <= '0;
            s_xlo <= '0; s_xhi <= '0; s_ylo <= '0; s_yhi <= '0; s_yfull <= 1'b0;
            d_xlo <= '0; d_xhi <= '0; d_ylo <= '0; d_yhi <= '0;
            bk_simple <= '0; bk_blend <= '0; bk_tint <= '0; bk_trans <= '0;
            bk_flip <= '0; bk_smode <= '0; bk_dmode <= '0;
            bk_sa <= '0; bk_da <= '0; bk_ktr <= '0; bk_ktg <= '0; bk_ktb <= '0;
        end
        else begin
            o_dsc_vld <= 1'b0;

            case (bst)
            B_IDLE: if (ob_v) begin
                q_smode <= ob_smode;  q_dmode <= ob_dmode;
                q_trans <= ob_trans;  q_blend <= ob_blend;
                q_flipx <= ob_flipx;  q_flipy <= ob_flipy;
                q_sa <= ob_sa;  q_da <= ob_da;
                q_tr <= ob_tr;  q_tg <= ob_tg;  q_tb <= ob_tb;
                q_src_x <= ob_src_x;  q_src_y <= ob_src_y;
                q_dst_x <= ob_dst_x;  q_dst_y <= ob_dst_y;
                q_dimx  <= ob_dimx;   q_dimy  <= ob_dimy;
                q_cminx <= ob_cminx;  q_cmaxx <= ob_cmaxx;
                q_cminy <= ob_cminy;  q_cmaxy <= ob_cmaxy;
                bst     <= B_S1;
            end

            // S1: flip-adjust src origin, wrap-guard endpoints, raw clip math
            B_S1: begin
                sx0 <= q_flipx ? ({2'b0, q_src_x} + {1'b0, q_dimx} - 15'd1)
                               : {2'b0, q_src_x};
                sy0 <= q_flipy ? ({3'b0, q_src_y} + {1'b0, q_dimy} - 15'd1)
                               : {3'b0, q_src_y};
                // guard compares (a = adjusted origin, b = other end), masked
                g_a <= q_flipx ? 13'(({2'b0, q_src_x} + {1'b0, q_dimx} - 15'd1))
                               : q_src_x;
                g_b <= q_flipx ? q_src_x
                               : 13'(({2'b0, q_src_x} + {1'b0, q_dimx} - 15'd1));
                starty_r <= q_cminy - q_dst_y;
                startx_r <= q_cminx - q_dst_x;
                // trim: if dst_end > max then extent = max - dst + 1
                dimy_e <= ((q_dst_y + 18'($signed({4'b0, q_dimy}))) > q_cmaxy)
                          ? (q_cmaxy - q_dst_y + 18'sd1)
                          : 18'($signed({4'b0, q_dimy}));
                dimx_e <= ((q_dst_x + 18'($signed({4'b0, q_dimx}))) > q_cmaxx)
                          ? (q_cmaxx - q_dst_x + 18'sd1)
                          : 18'($signed({4'b0, q_dimx}));
                bst <= B_S2;
            end

            // S2: wrap-guard reject, clip starts, effective flags
            B_S2: begin
                starty <= (starty_r > 18'sd0) ? starty_r : 18'sd0;
                startx <= (startx_r > 18'sd0) ? startx_r : 18'sd0;
                q_tint_eff  <= (q_tr != 6'h20) || (q_tg != 6'h20) || (q_tb != 6'h20);
                q_blend_eff <= q_blend && !((q_smode == 3'd0) && (q_sa == 5'h1f) &&
                                            (q_dmode == 3'd4) && (q_da == 5'h1f));
                if (q_flipx ? (g_a < g_b) : (g_a > g_b))
                    bst <= B_IDLE;             // src 0x2000-edge wrap: skip draw
                else
                    bst <= B_S3P;
            end

            // S3P (r4 iter3): register the S3 automatics one cycle ahead
            B_S3P: begin
                automatic logic signed [17:0] npx    = dimx_e - startx;
                automatic logic        [12:0] sxb    =
                    q_flipx ? 13'(sx0 - 15'(startx)) : 13'(sx0 + 15'(startx));
                automatic logic signed [17:0] sxlo_c =
                    q_flipx ? (18'($signed({5'b0, sxb})) - npx + 18'sd1)
                            : 18'($signed({5'b0, sxb}));
                automatic logic signed [17:0] sxhi_c =
                    q_flipx ? 18'($signed({5'b0, sxb}))
                            : (18'($signed({5'b0, sxb})) + npx - 18'sd1);
                automatic logic signed [17:0] sylo_u =
                    q_flipy ? (18'($signed({3'b0, sy0})) - (dimy_e - 18'sd1))
                            : (18'($signed({3'b0, sy0})) + starty);
                automatic logic signed [17:0] syhi_u =
                    q_flipy ? (18'($signed({3'b0, sy0})) - starty)
                            : (18'($signed({3'b0, sy0})) + dimy_e - 18'sd1);
                automatic logic               syfull =
                    (sylo_u < 18'sd0) || (sylo_u[17:12] != syhi_u[17:12]);
                automatic logic signed [17:0] dxlo_c = q_dst_x + startx;
                automatic logic signed [17:0] dxhi_c = q_dst_x + dimx_e - 18'sd1;
                automatic logic               dspill =
                    (dxlo_c < 18'sd0) || (dxhi_c > 18'sd8191);
                automatic logic signed [17:0] xshift = dxlo_c - 18'($signed({5'b0, sxb}));
                p3_npx   <= npx;
                p3_sxb   <= sxb;
                p3_ysr   <= q_flipy ? 12'(sy0 - 15'(starty)) : 12'(sy0 + 15'(starty));
                p3_drow  <= ((32'($signed(q_dst_y)) + 32'($signed(starty))) <<< 13)
                            + 32'($signed(q_dst_x)) + 32'($signed(startx));
                p3_rows  <= dimy_e - starty;
                p3_sxlo  <= sxlo_c;
                p3_sxhi  <= sxhi_c;
                p3_sylo  <= {6'b0, sylo_u[11:0]};   // RAW; the syfull mask
                p3_syhi  <= {6'b0, syhi_u[11:0]};   // applies at B_S3 (r4
                p3_syfull<= syfull;                 // iter4: 2 levels off S3P)
                p3_fxlo  <= dspill ? 18'sd0    : dxlo_c;
                p3_fxhi  <= dspill ? 18'sd8191 : dxhi_c;
                p3_dylo  <= (q_dst_y + starty) - (dspill ? 18'sd1 : 18'sd0);
                p3_dyhi  <= (q_dst_y + dimy_e - 18'sd1) + (dspill ? 18'sd1 : 18'sd0);
                p3_xs_sm <= (xshift > -18'sd4) && (xshift < 18'sd4);
                p3_rej   <= (starty >= dimy_e) || (startx >= dimx_e);
                bst      <= B_S3;
            end

            // S3: extents, bases, hazard rects; program the mode bank
            // (held until the target bank's pipe beats have drained)
            B_S3: begin
                if (p3_rej) bst <= B_IDLE;   // registered at S3P (r4 iter4)
                else if (s3_bank_clear) begin
                    // r4 iter3: rects/bases come from the B_S3P registers;
                    // this cycle keeps the f_ovl compares (2 levels off
                    // regs), the LIVE pv_* hazard legs, and the bank
                    // program.  Values identical to the one-cycle form.
                    automatic logic signed [17:0] sylo_m =
                        p3_syfull ? 18'sd0    : p3_sylo;
                    automatic logic signed [17:0] syhi_m =
                        p3_syfull ? 18'sd4095 : p3_syhi;
                    automatic logic ovl_self =
                        f_ovl(p3_sxlo, p3_sxhi, p3_fxlo, p3_fxhi) &&
                        f_ovl(sylo_m, syhi_m, p3_dylo, p3_dyhi);

                    n_px_s   <= p3_npx;
                    sx_base  <= p3_sxb;
                    ysrc0    <= p3_ysr;
                    didx_row0<= p3_drow;
                    rows     <= p3_rows;
                    s_xlo <= p3_sxlo;  s_xhi <= p3_sxhi;
                    s_ylo <= sylo_m;   s_yhi <= syhi_m;   s_yfull <= p3_syfull;
                    d_xlo <= p3_fxlo;  d_xhi <= p3_fxhi;
                    d_ylo <= p3_dylo;  d_yhi <= p3_dyhi;

                    q_strict <= ovl_self;
                    // 1-px beats when the golden sequential smear is visible
                    // inside a 4-px beat: flipped src, or |x shift| < 4
                    q_px1    <= ovl_self && (q_flipx || q_flipy || p3_xs_sm);
                    // cross-op hazard: our src (or, when blending, our dst)
                    // touches the previous op's dst -> drain once at start
                    q_waitpipe <= ovl_self ||
                        (pv_valid && f_ovl(p3_sxlo, p3_sxhi, pv_xlo, pv_xhi)
                                  && f_ovl(sylo_m, syhi_m, pv_ylo, pv_yhi)) ||
                        (pv_valid && q_blend_eff &&
                                     f_ovl(p3_fxlo, p3_fxhi, pv_xlo, pv_xhi) &&
                                     f_ovl(p3_dylo, p3_dyhi, pv_ylo, pv_yhi));

                    q_simple <= !q_blend_eff && !q_tint_eff;
                    // program the (currently unused) mode bank
                    bk_sel               <= ~bk_sel;
                    bk_simple[~bk_sel]   <= !q_blend_eff && !q_tint_eff;
                    bk_blend [~bk_sel]   <= q_blend_eff;
                    bk_tint  [~bk_sel]   <= q_tint_eff;
                    bk_trans [~bk_sel]   <= q_trans;
                    bk_flip  [~bk_sel]   <= q_flipx;
                    bk_smode [~bk_sel]   <= q_smode;
                    bk_dmode [~bk_sel]   <= q_dmode;
                    bk_sa    [~bk_sel]   <= q_sa;
                    bk_da    [~bk_sel]   <= q_da;
                    bk_ktr   [~bk_sel]   <= 18'(q_tr * 12'd2115);
                    bk_ktg   [~bk_sel]   <= 18'(q_tg * 12'd2115);
                    bk_ktb   [~bk_sel]   <= 18'(q_tb * 12'd2115);
                    o_dsc_vld <= 1'b1;         // H7 sideband: surviving DRAW
                    bst      <= B_ROW;
                end
            end

            // ROW: load row 0, arm row-1 shadows; honor start-of-op drain and
            // make sure no stale beat (two ops ago) still carries our bank.
            B_ROW: if ((!q_waitpipe || pipe_empty) && bank_clear) begin
                automatic logic [24:0] srow0 =
                    {ysrc0, 13'd0} + {12'd0, sx_base}
                    - ((q_flipx && !q_px1) ? 25'd3 : 25'd0);
                src_cur     <= srow0;
                didx_beat   <= didx_row0;
                px_left     <= 16'($signed(n_px_s));
                srow_sh     <= srow0 + (q_flipy ? -25'd8192 : 25'd8192);
                didx_row_sh <= didx_row0 + 32'sd8192;
                y_left      <= rows;
                bst         <= B_BEAT;
            end

            B_BEAT: if (emit_fire) begin
                if (last_of_row) begin
                    if (y_left == 18'sd1) begin
                        // op retired: publish clipped dst rect for hazards
                        pv_valid <= 1'b1;
                        pv_xlo   <= d_xlo;  pv_xhi <= d_xhi;
                        pv_ylo   <= d_ylo;  pv_yhi <= d_yhi;
                        bst      <= B_IDLE;
                    end
                    else begin
                        src_cur     <= srow_sh;
                        didx_beat   <= didx_row_sh;
                        px_left     <= 16'($signed(n_px_s));
                        srow_sh     <= srow_sh + (q_flipy ? -25'd8192 : 25'd8192);
                        didx_row_sh <= didx_row_sh + 32'sd8192;
                        y_left      <= y_left - 18'sd1;
                    end
                end
                else begin
                    src_cur   <= q_flipx ? (src_cur - 25'(step)) : (src_cur + 25'(step));
                    didx_beat <= didx_beat + 32'(step);
                    px_left   <= px_left - 16'(step);
                end
            end

            default: bst <= B_IDLE;
            endcase

            // an UPLOAD also becomes "previous dst" for the next draw
            if (up_wr_fire && up_last_in_row && (up_y == up_dimy - 13'd1)) begin
                pv_valid <= 1'b1;
                pv_xlo   <= upr_spill ? 18'sd0    : upr_xlo;
                pv_xhi   <= upr_spill ? 18'sd8191 : upr_xhi;
                pv_ylo   <= upr_ylo;
                pv_yhi   <= upr_yhi;
            end
        end
    end

    // ---------------------------------------------------------------------
    // pixel pipe B1..B4
    // ---------------------------------------------------------------------
    // ALU1 (comb, from the B2r raw-capture registers): channel extract +
    // tint multiply per PHYSICAL lane, then the px1/flip lane permute
    // picks finished results.  r6: the permute used to sit in FRONT of
    // the tint multiply (select + multiply in series, b2r_flip -2.19) --
    // the multiply distributes over the lane mux, so px1/flip enter once,
    // at the last plane.  Same 12 f_mulop_k instances, identical values
    // by substitution (rw of output lane l IS physical lane p's word).
    logic [3:0][15:0] f1_w;                    // per-physical-lane word
    logic [3:0][4:0]  f1_r, f1_g, f1_b;        // tinted channels
    logic [3:0]       f1_a;
    always_comb begin
        for (int p = 0; p < 4; p++) begin
            automatic logic [15:0] sw;
            sw      = b2r_s[p*16 +: 16];
            f1_w[p] = sw;
            f1_a[p] = sw[15];
            f1_r[p] = b2r_tint ? f_mulop_k(sw[14:10], b2r_ktr) : sw[14:10];
            f1_g[p] = b2r_tint ? f_mulop_k(sw[9:5],   b2r_ktg) : sw[9:5];
            f1_b[p] = b2r_tint ? f_mulop_k(sw[4:0],   b2r_ktb) : sw[4:0];
        end
    end
    logic [3:0][15:0] a1_raw;
    logic [3:0][4:0]  a1_sr, a1_sg, a1_sb, a1_dr, a1_dg, a1_db;
    logic [3:0]       a1_a, a1_mask;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            automatic logic [1:0]  p;
            automatic logic [15:0] dw_;
            // src lane un-flip: 4-px beats read ascending, output descending
            p   = b2r_px1 ? 2'd0 : (b2r_flip ? 2'(3 - l) : 2'(l));
            dw_ = b2r_d[l*16 +: 16];
            a1_raw[l] = f1_w[p];
            a1_a[l]   = f1_a[p];
            a1_sr[l]  = f1_r[p];
            a1_sg[l]  = f1_g[p];
            a1_sb[l]  = f1_b[p];
            a1_dr[l]  = dw_[14:10];
            a1_dg[l]  = dw_[9:5];
            a1_db[l]  = dw_[4:0];
            a1_mask[l]= b2r_en[l] && (!b2r_trans || f1_a[p]);
        end
    end

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            b1_v <= 1'b0; b2_v <= 1'b0; b2r_v <= 1'b0; b3_v <= 1'b0; b4_v <= 1'b0;
            b1_bk <= 1'b0; b2_bk <= 1'b0; b2r_bk <= 1'b0; b3_bk <= 1'b0; b4_bk <= 1'b0;
            b1_px1 <= 1'b0; b2_px1 <= 1'b0; b2r_px1 <= 1'b0;
            b1_sa_ <= '0; b1_wa <= '0; b2_wa <= '0; b2r_wa <= '0; b3_wa <= '0; b4_wa <= '0;
            b1_en <= '0; b2_en <= '0; b2r_en <= '0; b3_mask <= '0; b4_mask <= '0;
            b2r_s <= '0; b2r_d <= '0;
            b3i_v <= 1'b0; b3i_bk <= 1'b0; b3i_wa <= '0; b3i_mask <= '0;
            b3i_raw <= '0; b3i_sr <= '0; b3i_sg <= '0; b3i_sb <= '0;
            b3i_dr <= '0; b3i_dg <= '0; b3i_db <= '0; b3i_a <= '0;
            b3s_v <= 1'b0; b3s_bk <= 1'b0; b3s_wa <= '0; b3s_mask <= '0;
            b3s_raw <= '0; b3s_sr <= '0; b3s_sg <= '0; b3s_sb <= '0;
            b3s_dr <= '0; b3s_dg <= '0; b3s_db <= '0; b3s_a <= '0;
            b3s_sm <= '0; b3s_dm <= '0;
            b3s_simple <= 1'b0; b3s_blendf <= 1'b0;
            b3s_xer <= '0; b3s_xeg <= '0; b3s_xeb <= '0;
            b3s_yer <= '0; b3s_yeg <= '0; b3s_yeb <= '0;
            b3_raw <= '0; b4_data <= '0;
            b3_sr <= '0; b3_sg <= '0; b3_sb <= '0;
            b3_dr <= '0; b3_dg <= '0; b3_db <= '0; b3_a <= '0;
            b1_blend <= 1'b0;
            b2r_flip <= 1'b0; b2r_tint <= 1'b0; b2r_trans <= 1'b0;
            b2r_ktr <= '0; b2r_ktg <= '0; b2r_ktb <= '0;
            b3i_sm <= '0; b3i_dm <= '0; b3i_sac <= '0; b3i_dac <= '0;
            b3_sm <= '0; b3_dm <= '0;
            b3_simple <= 1'b0; b3_blendf <= 1'b0;
            b3_ps_r <= '0; b3_ps_g <= '0; b3_ps_b <= '0;
            b3_pd_r <= '0; b3_pd_g <= '0; b3_pd_b <= '0;
        end
        else if (adv) begin
            // B3 -> B4 (ALU2 result lands in the write-data register)
            b4_v    <= b3_v;
            b4_bk   <= b3_bk;
            b4_wa   <= b3_wa;
            b4_mask <= b3_v ? b3_mask : 4'b0;
            b4_data <= {a2_out[3], a2_out[2], a2_out[1], a2_out[0]};
            // B3s -> B3 (r5: the ALU2a xe*y products, now register-fed from
            // the B3s operand banks; everything else is a 1:1 field copy)
            b3_v   <= b3s_v;  b3_bk <= b3s_bk;
            b3_wa  <= b3s_wa; b3_mask <= b3s_mask;
            b3_raw <= b3s_raw;
            b3_sr <= b3s_sr;  b3_sg <= b3s_sg;  b3_sb <= b3s_sb;
            b3_dr <= b3s_dr;  b3_dg <= b3s_dg;  b3_db <= b3s_db;
            b3_a  <= b3s_a;
            b3_sm     <= b3s_sm;
            b3_dm     <= b3s_dm;
            b3_simple <= b3s_simple;
            b3_blendf <= b3s_blendf;
            b3_ps_r <= a2a_ps_r;  b3_ps_g <= a2a_ps_g;  b3_ps_b <= a2a_ps_b;
            b3_pd_r <= a2a_pd_r;  b3_pd_g <= a2a_pd_g;  b3_pd_b <= a2a_pd_b;
            // B3i -> B3s (r5: operand select + rev-invert registered; the
            // bk_simple/bk_blend reads keep their B3i-edge indexing, so the
            // bank-read window is unchanged by the extra stage)
            b3s_v   <= b3i_v;  b3s_bk <= b3i_bk;
            b3s_wa  <= b3i_wa; b3s_mask <= b3i_mask;
            b3s_raw <= b3i_raw;
            b3s_sr <= b3i_sr;  b3s_sg <= b3i_sg;  b3s_sb <= b3i_sb;
            b3s_dr <= b3i_dr;  b3s_dg <= b3i_dg;  b3s_db <= b3i_db;
            b3s_a  <= b3i_a;
            b3s_sm <= b3i_sm;
            b3s_dm <= b3i_dm;
            b3s_simple <= bk_simple[b3i_bk];
            b3s_blendf <= bk_blend[b3i_bk];
            b3s_xer <= a2s_xer;  b3s_xeg <= a2s_xeg;  b3s_xeb <= a2s_xeb;
            b3s_yer <= a2s_yer;  b3s_yeg <= a2s_yeg;  b3s_yeb <= a2s_yeb;
            // B2r -> B3i (ALU1 from the captured read beats.  r4: the
            // ALU2a selects/constants ride this edge from the mode bank --
            // bank stable while the op occupies b2r, per s3_bank_clear)
            b3i_v   <= b2r_v;  b3i_bk <= b2r_bk;
            b3i_wa  <= b2r_wa; b3i_mask <= a1_mask;
            for (int l = 0; l < 4; l++) b3i_raw[l*16 +: 16] <= a1_raw[l];
            b3i_sr <= a1_sr;  b3i_sg <= a1_sg;  b3i_sb <= a1_sb;
            b3i_dr <= a1_dr;  b3i_dg <= a1_dg;  b3i_db <= a1_db;
            b3i_a  <= a1_a;
            b3i_sm  <= bk_smode[b2r_bk];
            b3i_dm  <= bk_dmode[b2r_bk];
            b3i_sac <= bk_sa[b2r_bk];
            b3i_dac <= bk_da[b2r_bk];
            // B2 -> B2r (read beats captured RAW at the edge that used to
            // consume them comb -- same adv gate, same i_rd_vld handshake.
            // r4: ALU1's mode-bank fields ride the same edge)
            b2r_v   <= b2_v;   b2r_bk <= b2_bk;  b2r_px1 <= b2_px1;
            b2r_wa  <= b2_wa;  b2r_en <= b2_en;
            b2r_s   <= i_srd_data;
            b2r_d   <= i_drd_data;
            b2r_flip  <= bk_flip[b2_bk];
            b2r_tint  <= bk_tint[b2_bk];
            b2r_trans <= bk_trans[b2_bk];
            b2r_ktr   <= bk_ktr[b2_bk];
            b2r_ktg   <= bk_ktg[b2_bk];
            b2r_ktb   <= bk_ktb[b2_bk];
            // B1 -> B2
            b2_v  <= b1_v;   b2_bk <= b1_bk;  b2_px1 <= b1_px1;
            b2_wa <= b1_wa;  b2_en <= b1_en;
            // emit -> B1 (r4: blend flag rides along for o_drd_req)
            b1_v   <= emit_fire;
            b1_bk  <= bk_sel;
            b1_blend <= bk_blend[bk_sel];
            b1_px1 <= q_px1;
            b1_sa_ <= src_cur;
            b1_wa  <= didx_beat[24:0];
            b1_en  <= lane_en_c;
        end
    end

    assign o_srd_req  = b1_v && adv;
    assign o_srd_addr = b1_sa_;
    assign o_drd_req  = b1_v && adv && b1_blend;
    assign o_drd_addr = b1_wa;

    // r4 raw request legs (see the port note)
    assign o_rq_v     = b1_v;
    assign o_rq_wr    = draw_wr;
    assign o_rq_blend = b1_blend;

    // ---------------------------------------------------------------------
    // H7 descriptor sideband fields: pure taps of the S1-S3 setup registers
    // (all written by the edge that raises o_dsc_vld; values are in-range
    // for surviving draws, so the width slices below are lossless)
    // ---------------------------------------------------------------------
    assign o_dsc_sx_lo    = s_xlo[12:0];
    assign o_dsc_sx_hi    = s_xhi[12:0];
    assign o_dsc_sy0      = ysrc0;
    assign o_dsc_rows     = rows[12:0];
    assign o_dsc_npx      = n_px_s[13:0];
    assign o_dsc_dst0     = unsigned'(didx_row0);
    assign o_dsc_flipx    = q_flipx;
    assign o_dsc_flipy    = q_flipy;
    assign o_dsc_blend    = q_blend_eff;
    assign o_dsc_strict   = q_strict;
    assign o_dsc_px1      = q_px1;
    assign o_dsc_wait     = q_waitpipe;
    assign o_dsc_upl_addr = up_addr;
    assign o_dsc_upl_dimx = up_dimx;
    assign o_dsc_upl_dimy = up_dimy;

    // ALU2a select half (comb, from B3i): operand select + rev-invert,
    // registered into the B3s xe/ye banks at the B3i->B3s edge.  r5: the
    // xe*y products moved one stage later (see the B3s header note) --
    // f_mulop_a(x,y,rev) == f_xe(x,rev) * y with f_xe registered, the
    // same split f_mulop_a itself performs internally.
    logic [3:0][4:0] a2s_xer, a2s_xeg, a2s_xeb;
    logic [3:0][4:0] a2s_yer, a2s_yeg, a2s_yeb;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            automatic logic [4:0] xr, xg, xb, yr, yg, yb;
            // clr0 = f(smode): operand select {alpha, s, d}, rev = sm[2]
            case (b3i_sm[1:0])
                2'd0: begin xr = b3i_sac;      xg = b3i_sac;      xb = b3i_sac;      end
                2'd1: begin xr = b3i_sr[l];    xg = b3i_sg[l];    xb = b3i_sb[l];    end
                default: begin xr = b3i_dr[l]; xg = b3i_dg[l];    xb = b3i_db[l];    end
            endcase
            a2s_xer[l] = b3i_sm[2] ? ~xr : xr;
            a2s_xeg[l] = b3i_sm[2] ? ~xg : xg;
            a2s_xeb[l] = b3i_sm[2] ? ~xb : xb;
            // dterm = f(dmode)
            case (b3i_dm[1:0])
                2'd0: begin yr = b3i_dac;      yg = b3i_dac;      yb = b3i_dac;      end
                2'd1: begin yr = b3i_sr[l];    yg = b3i_sg[l];    yb = b3i_sb[l];    end
                default: begin yr = b3i_dr[l]; yg = b3i_dg[l];    yb = b3i_db[l];    end
            endcase
            a2s_yer[l] = b3i_dm[2] ? ~yr : yr;
            a2s_yeg[l] = b3i_dm[2] ? ~yg : yg;
            a2s_yeb[l] = b3i_dm[2] ? ~yb : yb;
        end
    end

    // ALU2a product half (comb, from B3s): pure register-fed 5x6 products,
    // registered into b3_ps_*/b3_pd_* at the B3s->B3 edge.
    logic [3:0][10:0] a2a_ps_r, a2a_ps_g, a2a_ps_b;
    logic [3:0][10:0] a2a_pd_r, a2a_pd_g, a2a_pd_b;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            a2a_ps_r[l] = b3s_xer[l] * {1'b0, b3s_sr[l]};
            a2a_ps_g[l] = b3s_xeg[l] * {1'b0, b3s_sg[l]};
            a2a_ps_b[l] = b3s_xeb[l] * {1'b0, b3s_sb[l]};
            a2a_pd_r[l] = b3s_yer[l] * {1'b0, b3s_dr[l]};
            a2a_pd_g[l] = b3s_yeg[l] * {1'b0, b3s_dg[l]};
            a2a_pd_b[l] = b3s_yeb[l] * {1'b0, b3s_db[l]};
        end
    end

    // ALU2b (comb, from B3): x2115 const multiply + clamp + saturating add
    // + output select; copy path passes raw.  Registers into b4_data at
    // the B3->B4 edge.  clr0/dterm bypasses (mode[1:0] == 3) read the
    // carried channel registers, exactly as the unsplit form did.
    logic [3:0][15:0] a2_out;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            automatic logic [4:0] c0r, c0g, c0b, dtr, dtg, dtb, orr, org_, orb;
            if (b3_sm[1:0] == 2'd3) begin
                c0r = b3_sr[l];  c0g = b3_sg[l];  c0b = b3_sb[l];
            end
            else begin
                c0r = f_mulop_b(b3_ps_r[l]);
                c0g = f_mulop_b(b3_ps_g[l]);
                c0b = f_mulop_b(b3_ps_b[l]);
            end
            if (b3_dm[1:0] == 2'd3) begin
                dtr = b3_dr[l];  dtg = b3_dg[l];  dtb = b3_db[l];
            end
            else begin
                dtr = f_mulop_b(b3_pd_r[l]);
                dtg = f_mulop_b(b3_pd_g[l]);
                dtb = f_mulop_b(b3_pd_b[l]);
            end
            // the dmode2 MAME quirk: G/B adds take clr0.R as first index
            orr = f_satadd(c0r, dtr);
            org_= f_satadd((b3_dm == 3'd2) ? c0r : c0g, dtg);
            orb = f_satadd((b3_dm == 3'd2) ? c0r : c0b, dtb);

            if (b3_simple)
                a2_out[l] = b3_raw[l*16 +: 16];
            else if (b3_blendf)
                a2_out[l] = {b3_a[l], orr, org_, orb};
            else  // tint only
                a2_out[l] = {b3_a[l], b3_sr[l], b3_sg[l], b3_sb[l]};
        end
    end

    // ---------------------------------------------------------------------
    // write channel mux (draw pipe / upload streaming - never simultaneous)
    // ---------------------------------------------------------------------
    assign o_wr_req  = up_wr_fire | draw_wr;
    assign o_wr_addr = up_wr_fire ? up_beat  : b4_wa;
    assign o_wr_data = up_wr_fire ? up_wdata : b4_data;
    assign o_wr_mask = up_wr_fire ? up_wmask : b4_mask;

`ifndef SYNTHESIS
    always @(posedge i_CLK)
        if (up_wr_fire && draw_wr)
            $display("[blit_draw] ERROR: upload/draw write collision (t=%0t)", $time);
`endif

endmodule
`default_nettype none
