project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
puts "########## c102 worst 24 summary ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}] -setup -npaths 24 -detail summary -stdout
puts "########## c102 worst FULL ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}] -setup -npaths 1 -detail path_only -stdout
puts "########## c153 worst 10 summary ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -setup -npaths 10 -detail summary -stdout
project_close
