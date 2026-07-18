project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist

puts "########## c153: worst 8 summary (packing A/B vs fit #8 -6.815 batch-RAM->Mult cone) ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -setup -npaths 8 -detail summary -stdout

puts "########## c153: worst into blit_gov (gov Y-stage cone current slack) ##########"
report_timing -to [get_registers {*|blit_gov:u_blit_gov|y_*}] -setup -npaths 4 -detail summary -stdout

puts "########## c153: worst into blit_draw (blender cone) ##########"
report_timing -to [get_registers {*|blit_draw:u_blit_draw|*}] -setup -npaths 4 -detail summary -stdout

puts "########## c102: worst 6 summary ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}] -setup -npaths 6 -detail summary -stdout

puts "########## c102: worst 1 full detail ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}] -setup -npaths 1 -detail full_path -stdout

puts "########## SDRAM_CLK setup worst 2 ##########"
report_timing -to_clock [get_clocks {SDRAM_CLK}] -setup -npaths 2 -detail summary -stdout

project_close
