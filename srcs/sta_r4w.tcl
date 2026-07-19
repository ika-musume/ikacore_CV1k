project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
puts "########## overall worst 8 ##########"
report_timing -setup -npaths 8 -detail summary -stdout
puts "########## fetch w_need->W_END full ##########"
report_timing -setup -to [get_registers {emu|core|u_blit|u_blit_fetch|wst*}] -npaths 1 -detail full_path -stdout
project_close
