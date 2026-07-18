project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist

puts "########## SDRAM_CLK output worst FULL ##########"
report_timing -to_clock [get_clocks {SDRAM_CLK}] -setup -npaths 1 -detail full_path -stdout

puts "########## SDRAM_CLK output worst 12 summary ##########"
report_timing -to_clock [get_clocks {SDRAM_CLK}] -setup -npaths 12 -detail summary -stdout

puts "########## SDRAM_CLK HOLD worst 3 (does mcp move it?) ##########"
report_timing -to_clock [get_clocks {SDRAM_CLK}] -hold -npaths 3 -detail summary -stdout

puts "########## RECOVERY worst FULL ##########"
report_timing -recovery -npaths 1 -detail full_path -stdout

puts "########## RECOVERY worst 12 summary ##########"
report_timing -recovery -npaths 12 -detail summary -stdout
project_close
