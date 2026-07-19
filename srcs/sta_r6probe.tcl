# r6 probe: full anatomy of the surfaced families
project_open ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
puts "########## as2_diff0 capture ##########"
report_timing -setup -to [get_registers {*u_batch|as2_diff0[*]}] -npaths 1 -detail full_path -stdout
puts "########## f2sdram -> bb_wfifo o_af ##########"
report_timing -setup -to [get_registers {*u_wfifo|o_af}] -npaths 1 -detail full_path -stdout
puts "########## ord_wdata -> nand ##########"
report_timing -setup -from [get_registers {*u_bsc|ord_wdata[*]}] -to [get_registers {*u_u2_nand|*}] -npaths 1 -detail full_path -stdout
puts "########## sx0 -> p3_sxlo ##########"
report_timing -setup -to [get_registers {*u_blit_draw|p3_sxlo[*]}] -npaths 1 -detail full_path -stdout
project_close
