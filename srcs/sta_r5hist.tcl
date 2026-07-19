project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
set c153 [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}]
set c102 [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}]
puts "########## r5 c153 hist 600 ##########"
report_timing -setup -to_clock $c153 -npaths 600 -detail summary -stdout
puts "########## r5 c102 hist 300 ##########"
report_timing -setup -to_clock $c102 -npaths 300 -detail summary -stdout
project_close
