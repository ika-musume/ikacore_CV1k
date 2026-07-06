`default_nettype none
`timescale 1ns/1ps
// standalone MX29LV320E sanity check: load the ibara image, read a few words
// with stable inputs and generous access time. Expect word0 = 0xdf3d.
module tb_flash_only;
    reg  [20:0] A = 0;
    wire [15:0] Q;
    reg  [15:0] Q_in = 0;
    reg  CE_B=1, WE_B=1, OE_B=1, RESET_B=0;

    MX29LV320E #(.Init_File("rom/ibara_u4_4M.hex")) dut (
        .A(A), .Q(Q), .Q_in(Q_in), .CE_B(CE_B), .WE_B(WE_B), .OE_B(OE_B),
        .BYTE_B(1'b1), .RESET_B(RESET_B), .RYBY_B(), .WP_B(1'b1));

    task rd(input [20:0] addr);
        begin
            A = addr; CE_B=0; OE_B=0; #200;      // >> Taa/Tce access time
            $display("[flash] word[%05x] Q=%04x", addr, Q);
            CE_B=1; OE_B=1; #50;
        end
    endtask

    initial begin
        #210_000;                                 // > Tvcs power-up
        RESET_B = 1; #200;
        rd(21'h00000); rd(21'h00001); rd(21'h00002); rd(21'h00003);
        rd(21'h00010);
        $finish;
    end
endmodule
`default_nettype none
