project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
puts "########## clock summary ##########"
qsta_utility::generate_top_failures_per_clock 4
puts "########## c102 worst 30 summary ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}] -setup -npaths 30 -detail summary -stdout
puts "########## c102 worst FULL ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}] -setup -npaths 1 -detail path_only -stdout
puts "########## blit_fetch -> pump family (new 9.76 exception) ##########"
report_timing -from [get_registers {emu|core|u_blit|u_blit_fetch|*}] -to [get_registers {emu|core|u_pump|*}] -setup -npaths 8 -detail summary -stdout
puts "########## pump-internal -> pads ##########"
report_timing -to [get_registers {emu|core|u_pump|o_S_*}] -setup -npaths 8 -detail summary -stdout
puts "########## c153 worst 8 summary ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -setup -npaths 8 -detail summary -stdout
puts "########## recovery worst 6 ##########"
report_timing -recovery -npaths 6 -detail summary -stdout
project_close
