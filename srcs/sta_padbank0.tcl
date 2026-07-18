project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist

puts "########## c102: worst 12 into the pump pad regs (summary) ##########"
report_timing -to [get_registers {*CV1k_sdram_control:u_pump|o_S_*}] -setup -npaths 12 -detail summary -stdout

puts "########## c102: worst 1 into o_S_A (full) ##########"
report_timing -to [get_registers {*CV1k_sdram_control:u_pump|o_S_A[*]}] -setup -npaths 1 -detail full_path -stdout

puts "########## c102: worst 12 overall (who owns the -7.35) ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}] -setup -npaths 12 -detail summary -stdout

puts "########## SDRAM_CLK output: worst 12 summary ##########"
report_timing -to_clock [get_clocks {SDRAM_CLK}] -setup -npaths 12 -detail summary -stdout

puts "########## SDRAM_CLK output: worst 2 full ##########"
report_timing -to_clock [get_clocks {SDRAM_CLK}] -setup -npaths 2 -detail full_path -stdout
project_close
