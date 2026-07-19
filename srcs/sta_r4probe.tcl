project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist

puts "########## R4-A draw dimx_e -> blend mult (full) ##########"
report_timing -setup -from [get_registers {emu|core|u_blit|u_blit_draw|dimx_e[*]}] -npaths 2 -detail full_path -stdout

puts "########## R4-A2 draw worst overall (full) ##########"
report_timing -setup -to [get_registers {emu|core|u_blit|u_blit_draw|*}] -npaths 2 -detail full_path -stdout

puts "########## R4-B fetch skid_half -> w_need (full) ##########"
report_timing -setup -from [get_registers {emu|core|u_blit|u_blit_fetch|skid_half*}] -npaths 2 -detail full_path -stdout

puts "########## R4-C gov last_mark -> win_f (full) ##########"
report_timing -setup -from [get_registers {emu|core|u_blit|u_blit_gov|last_mark*}] -npaths 2 -detail full_path -stdout

puts "########## R4-D wfifo o_af fan (full) ##########"
report_timing -setup -from [get_registers {emu|core|u_blit|u_bb_wfifo|*}] -npaths 2 -detail full_path -stdout

puts "########## R4-E draw top-10 endpoints summary ##########"
report_timing -setup -to [get_registers {emu|core|u_blit|u_blit_draw|*}] -npaths 10 -detail summary -stdout

puts "########## R4-F fetch top-10 endpoints summary ##########"
report_timing -setup -to [get_registers {emu|core|u_blit|u_blit_fetch|*}] -npaths 10 -detail summary -stdout

project_close
