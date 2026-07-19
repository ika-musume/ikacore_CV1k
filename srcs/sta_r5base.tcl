project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
set c153 [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}]
set c102 [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}]

# ---- R5 baseline: TRUE domain-worst families, binned by LATCH clock ----
# (the .sta.summary bins this way; the r4 panels missed -2.897 / -3.961)
puts "########## r5 c153-latched worst 20 (summary) ##########"
report_timing -setup -to_clock $c153 -npaths 20 -detail summary -stdout
puts "########## r5 c153-latched worst 3 (full path) ##########"
report_timing -setup -to_clock $c153 -npaths 3 -detail full_path -stdout
puts "########## r5 c102-latched worst 20 (summary) ##########"
report_timing -setup -to_clock $c102 -npaths 20 -detail summary -stdout
puts "########## r5 c102-latched worst 3 (full path) ##########"
report_timing -setup -to_clock $c102 -npaths 3 -detail full_path -stdout
project_close
