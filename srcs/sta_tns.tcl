project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
set c153 [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}]
set c102 [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}]
puts "########## c153 all violations ##########"
report_timing -setup -to_clock $c153 -less_than_slack 0 -npaths 4000 -detail summary -stdout
puts "########## c102 all violations ##########"
report_timing -setup -to_clock $c102 -less_than_slack 0 -npaths 4000 -detail summary -stdout
project_close
