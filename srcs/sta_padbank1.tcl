project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist

puts "########## c102: worst 8 overall (post pad-bank) ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}] -setup -npaths 8 -detail summary -stdout

puts "########## c102: worst into pump pad regs (did the tail move?) ##########"
report_timing -to [get_registers {*CV1k_sdram_control:u_pump|o_S_*}] -setup -npaths 3 -detail summary -stdout

puts "########## SDRAM_CLK output: worst 8 summary ##########"
report_timing -to_clock [get_clocks {SDRAM_CLK}] -setup -npaths 8 -detail summary -stdout

puts "########## SDRAM_CLK output: worst 1 full ##########"
report_timing -to_clock [get_clocks {SDRAM_CLK}] -setup -npaths 1 -detail full_path -stdout

puts "########## HDMI pll setup worst 3 ##########"
report_timing -to_clock [get_clocks {pll_hdmi|pll_hdmi_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -setup -npaths 3 -detail summary -stdout

puts "########## recovery c153 worst 4 ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -recovery -npaths 4 -detail summary -stdout

puts "########## recovery c102 worst 4 ##########"
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}] -recovery -npaths 4 -detail summary -stdout
project_close
