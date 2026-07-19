project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
puts "########## R4-D wfifo o_af fan (full) ##########"
report_timing -setup -from [get_registers {*bb_wfifo*|o_af}] -npaths 2 -detail full_path -stdout
puts "########## R4-D2 wfifo all-from summary ##########"
report_timing -setup -from [get_registers {*bb_wfifo*}] -npaths 6 -detail summary -stdout
puts "########## R4-G video hcnt->y_v (full) ##########"
report_timing -setup -to [get_registers {emu|core|u_blit|u_blit_video|*}] -npaths 1 -detail full_path -stdout
project_close
