`default_nettype none
//============================================================================
// blit_dsc_check.sv - H7 descriptor-sideband footprint checker (sim-only)
//
// Asserts, on every beat the draw engine emits, that the address falls
// inside the footprint PREDICTED by the descriptor sideband - the property
// blit_batch's K=8-objline address generation depends on.  A violation
// means the sideband and the beat generator disagree and would corrupt a
// prefetch train, so it is a hard $fatal.
//
// Checked:
//   * src reads  - (row, x) against the CURRENT draw descriptor: row must
//     be sy0 -/+ r (mod 4096, r < rows), x within [sx_lo-3, sx_hi] (the -3
//     covers the flip-mode 4-px beat base adjust; a base 0..2 px below the
//     row start appears as x >= 8189 in the PREVIOUS row's x field, handled
//     as the wrapped interpretation).
//   * writes (per enabled lane) - lane address against the current draw,
//     the two PREVIOUS draws (pipe depth: stalled beats of up to two older
//     ops may retire after the next descriptor strobes), or the last
//     UPLOAD descriptor (flat base + dimx/dimy, row step 8192 mod 2^25).
//   * dst reads are not separately checked: o_drd_addr is the same beat
//     didx stream the write check covers.
//
// Sensitivity is req-level (requests only assert on advancing cycles), so
// no pipe knowledge is needed here.
//============================================================================
module blit_dsc_check (
    input wire        i_CLK,
    input wire        i_RST_n,

    // descriptor sideband (from blit_draw / blit_top)
    input wire        i_dsc_vld,
    input wire [12:0] i_dsc_sx_lo,
    input wire [12:0] i_dsc_sx_hi,
    input wire [11:0] i_dsc_sy0,
    input wire [12:0] i_dsc_rows,
    input wire [13:0] i_dsc_npx,
    input wire [31:0] i_dsc_dst0,
    input wire        i_dsc_flipy,
    input wire        i_dsc_upl,
    input wire [24:0] i_dsc_upl_addr,
    input wire [13:0] i_dsc_upl_dimx,
    input wire [12:0] i_dsc_upl_dimy,

    // beat channels (from blit_draw / blit_top)
    input wire        i_srd_req,
    input wire [24:0] i_srd_addr,
    input wire        i_wr_req,
    input wire [24:0] i_wr_addr,
    input wire [3:0]  i_wr_mask
);

    // descriptor snapshots: current + two previous draws, last upload
    typedef struct packed {
        logic        valid;
        logic [12:0] sx_lo, sx_hi;
        logic [11:0] sy0;
        logic [12:0] rows;
        logic [13:0] npx;
        logic signed [31:0] dst0;
        logic        flipy;
    } dsc_t;

    dsc_t cur, prv, prv2;
    logic        upl_valid;
    logic [24:0] upl_addr;
    logic [13:0] upl_dimx;
    logic [12:0] upl_dimy;

    longint unsigned n_src_checked, n_wr_checked;

    always_ff @(posedge i_CLK or negedge i_RST_n) begin
        if (!i_RST_n) begin
            cur <= '0; prv <= '0; prv2 <= '0;
            upl_valid <= 1'b0;
            upl_addr  <= '0; upl_dimx <= '0; upl_dimy <= '0;
        end
        else begin
            if (i_dsc_vld) begin
                prv2 <= prv;
                prv  <= cur;
                cur  <= '{valid: 1'b1,
                          sx_lo: i_dsc_sx_lo, sx_hi: i_dsc_sx_hi,
                          sy0:   i_dsc_sy0,   rows:  i_dsc_rows,
                          npx:   i_dsc_npx,   dst0:  signed'(i_dsc_dst0),
                          flipy: i_dsc_flipy};
            end
            if (i_dsc_upl) begin
                upl_valid <= 1'b1;
                upl_addr  <= i_dsc_upl_addr;
                upl_dimx  <= i_dsc_upl_dimx;
                upl_dimy  <= i_dsc_upl_dimy;
            end
        end
    end

    // one (row, x) interpretation of a src beat against the current draw.
    // x may be NEGATIVE (wrapped interpretation: a flip-mode beat base up
    // to 3 px below the row start), so the lower bound sx_lo-3 is unclamped.
    function automatic logic f_src_ok(input int y, input int x);
        int dy;
        if (!cur.valid) return 1'b0;
        dy = cur.flipy ? ((int'(cur.sy0) - y) & 4095)
                       : ((y - int'(cur.sy0)) & 4095);
        return (dy < int'(cur.rows)) &&
               (x >= int'(cur.sx_lo) - 3) && (x <= int'(cur.sx_hi));
    endfunction

    // one write-lane flat index against one draw descriptor
    function automatic logic f_wr_draw_ok(input dsc_t d, input longint lane);
        longint diff;
        int r, off;
        if (!d.valid) return 1'b0;
        diff = lane - longint'(d.dst0);
        if (diff < 0) return 1'b0;
        r   = int'(diff >>> 13);
        off = int'(diff & 8191);
        return (r < int'(d.rows)) && (off < int'(d.npx));
    endfunction

    // one write-lane flat index against an upload rect (mod-2^25 rows).
    // A 1-word upload commits its first (only) write in the SAME cycle the
    // o_dsc_upl pulse is high, before the registered capture - so the
    // caller also passes the live sideband values during the pulse.
    function automatic logic f_wr_upl_ok(input logic vld,
                                         input logic [24:0] base,
                                         input logic [13:0] dimx,
                                         input logic [12:0] dimy,
                                         input longint lane);
        longint diff;
        int r, off;
        if (!vld) return 1'b0;
        diff = (lane - longint'(base)) & 33554431;       // mod 2^25
        r   = int'(diff >>> 13);
        off = int'(diff & 8191);
        return (r < int'(dimy)) && (off < int'(dimx));
    endfunction

    always_ff @(posedge i_CLK) begin
        if (i_RST_n) begin
            if (i_srd_req) begin
                automatic int y = int'(i_srd_addr[24:13]);
                automatic int x = int'(i_srd_addr[12:0]);
                automatic logic ok = f_src_ok(y, x) ||
                    // flip beat base 1..3 px below the row start: the x
                    // field wraps to >= 8189 in the previous row's index
                    ((x >= 8189) && f_src_ok((y + 1) & 4095, x - 8192));
                n_src_checked++;
                if (!ok)
                    $fatal(2, "[dsc_check] src beat %07x outside descriptor (sy0=%0d rows=%0d sx=[%0d,%0d] flipy=%0d) t=%0t",
                           i_srd_addr, cur.sy0, cur.rows, cur.sx_lo, cur.sx_hi, cur.flipy, $time);
            end
            if (i_wr_req && (i_wr_mask != 4'b0)) begin
                for (int l = 0; l < 4; l++) begin
                    if (i_wr_mask[l]) begin
                        automatic longint lane =
                            (longint'(i_wr_addr) + longint'(l)) & 33554431;
                        n_wr_checked++;
                        if (!(f_wr_draw_ok(cur, lane) || f_wr_draw_ok(prv, lane) ||
                              f_wr_draw_ok(prv2, lane) ||
                              f_wr_upl_ok(upl_valid, upl_addr, upl_dimx, upl_dimy, lane) ||
                              f_wr_upl_ok(i_dsc_upl, i_dsc_upl_addr,
                                          i_dsc_upl_dimx, i_dsc_upl_dimy, lane)))
                            $fatal(2, "[dsc_check] wr lane %07x outside cur/prv/prv2/upl descriptors (cur dst0=%0d npx=%0d rows=%0d) t=%0t",
                                   25'(lane), cur.dst0, cur.npx, cur.rows, $time);
                    end
                end
            end
        end
    end

    final
        $display("[dsc_check] clean: %0d src beats + %0d wr lanes checked against the sideband",
                 n_src_checked, n_wr_checked);

endmodule
`default_nettype none
