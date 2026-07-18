project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist

puts "########## c153: worst 8 summary (baseline gov-cone identity for Y-stage job) ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -setup -npaths 8 -detail summary -stdout

puts "########## c153: worst 2 full detail ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -setup -npaths 2 -detail full_path -stdout

puts "########## c102: worst 4 summary ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}] -setup -npaths 4 -detail summary -stdout

project_close
