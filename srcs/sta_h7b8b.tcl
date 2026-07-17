## H7b.8 respin STA: summaries + the specific questions of this fit
project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist

puts "==== per-domain summaries ===="
report_clock_fmax_summary -stdout
create_timing_summary -setup -stdout
create_timing_summary -hold -stdout
create_timing_summary -recovery -stdout

puts "==== dq_n capture windows (verify edge pair + margins) ===="
report_timing -from [get_ports {SDRAM_DQ[*]}] -to [get_registers {*u_pump|dq_n[*]}] -setup -npaths 2 -detail summary -stdout
report_timing -from [get_ports {SDRAM_DQ[*]}] -to [get_registers {*u_pump|dq_n[*]}] -hold  -npaths 2 -detail summary -stdout

puts "==== any DQ path escaping dq_n? ===="
report_timing -from [get_ports {SDRAM_DQ[*]}] -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -setup -npaths 3 -detail summary -stdout
report_timing -from [get_ports {SDRAM_DQ[*]}] -setup -npaths 3 -detail summary -stdout

puts "==== dq_n fan-out (the new half-period frontier) ===="
report_timing -from [get_registers {*u_pump|dq_n[*]}] -setup -npaths 4 -detail summary -stdout

puts "==== worst per clock: c153 then c102 ===="
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -setup -npaths 6 -detail summary -stdout
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}] -setup -npaths 6 -detail summary -stdout

puts "==== recovery worst ===="
report_timing -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -recovery -npaths 3 -detail summary -stdout

project_close
