`default_nettype none
//============================================================================
// cpu_tracer - retired-instruction trace probe, bound into cpu_core
//
// The HS3 top leaves the core's dbg_o_RETIRE_* ports open. We reach them with
// a SystemVerilog `bind` (see tb_cv1k.sv) so no edit to the read-only IP is
// needed. One line is emitted per architecturally-retired instruction:
//
//     <pc:8hex> <inst:4hex>[ ; r<n>=<data:8hex>]
//
// which is the canonical stream diffed against the MAME SH-3 trace by
// scripts/compare_flow.py.  A +maxinsn plusarg bounds the run.
//============================================================================
module cpu_tracer (
    input wire        i_CLK,
    input wire        i_CEN,
    input wire        i_RST_n,
    input wire        valid,
    input wire [31:0] pc,
    input wire [15:0] inst,
    input wire        gpr_we,
    input wire [4:0]  gpr,
    input wire [31:0] gpr_data,
    input wire [31:0] fetch_pc
);
    integer  fh;
    longint  count;                 // 64-bit: allow tens/hundreds of millions
    longint  maxinsn;
    longint  tracefrom;             // suppress output until this many retires
    string   tracefile;

    // debug heartbeat: is the fetch PC advancing / is anything retiring?
    integer  dbg = 0;
    integer  hb  = 0;
    always @(posedge i_CLK) if (i_RST_n) begin
        if (dbg) begin
            hb = hb + 1;
            if (hb % 20000 == 0)
                $display("[hb] t=%0t fetch_pc=%08x retired=%0d valid=%b", $time, fetch_pc, count, valid);
        end
    end
    initial void'($value$plusargs("dbg=%d", dbg));

    initial begin
        count     = 0;
        maxinsn   = 200000;                 // default cap
        tracefile = "build/trace_rtl.txt";
        tracefrom = 0;
        void'($value$plusargs("trace=%s",   tracefile));
        void'($value$plusargs("maxinsn=%d", maxinsn));
        void'($value$plusargs("tracefrom=%d", tracefrom));   // window: [tracefrom, maxinsn)
        fh = $fopen(tracefile, "w");
        if (fh == 0) begin
            $display("[tracer] ERROR: cannot open %s", tracefile);
            $finish;
        end
        $display("[tracer] writing RTL trace -> %s (cap %0d insns)", tracefile, maxinsn);
    end

    always @(posedge i_CLK) begin
        if (i_RST_n && i_CEN && valid) begin
            if (count >= tracefrom) begin
                if (gpr_we)
                    $fwrite(fh, "%08x %04x ; r%0d=%08x\n", pc, inst, gpr, gpr_data);
                else
                    $fwrite(fh, "%08x %04x\n", pc, inst);
            end
            count = count + 1;
            if (count >= maxinsn) begin
                $display("[tracer] reached %0d retired instructions - stop", count);
                $fclose(fh);
                $finish;
            end
        end
    end
endmodule
`default_nettype none
