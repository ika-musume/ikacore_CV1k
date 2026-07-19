project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
puts "########## r5 batch serve-roll full ##########"
report_timing -setup -to [get_registers {*|blit_batch:u_batch|sv_after[*] *|blit_batch:u_batch|sv_slot[*] *|blit_batch:u_batch|sv_blen[*]}] -npaths 3 -detail full_path -stdout
puts "########## r5 batch worst-in 12 ##########"
report_timing -setup -to [get_registers {*|blit_batch:u_batch|*}] -npaths 12 -detail summary -stdout
puts "########## r5 blit_top worst-in 8 ##########"
report_timing -setup -to [get_registers {*|blit_top:u_blit|*}] -npaths 8 -detail summary -stdout
project_close
