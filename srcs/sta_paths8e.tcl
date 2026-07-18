project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
puts "########## batch sch -> sv_blen ##########"
report_timing -setup -from [get_registers {emu|core|u_batch|sch.S_RD}] -to [get_registers {emu|core|u_batch|sv_blen[*]}] -npaths 1 -detail full_path -stdout
puts "########## wfifo cnt -> draw mult ##########"
report_timing -setup -from [get_registers {emu|core|u_batch|u_wfifo|cnt[*]}] -to [get_registers {emu|core|u_blit|u_blit_draw|*}] -npaths 1 -detail full_path -stdout
puts "########## draw mult -> xe ##########"
report_timing -setup -to [get_registers {emu|core|u_blit|u_blit_draw|xe*}] -npaths 1 -detail full_path -stdout
puts "########## f2sdram -> video line_buf ##########"
report_timing -setup -to [get_registers {emu|core|u_blit|u_blit_video|g_fetch_train.line_buf*}] -npaths 1 -detail full_path -stdout
puts "########## harness oq_wp -> oq we ##########"
report_timing -setup -from [get_registers {emu|core|u_harness|oq_wp[*]}] -npaths 1 -detail full_path -stdout
puts "########## nand page_buf -> dout ##########"
report_timing -setup -to [get_registers {emu|core|u_u2_nand|dout_reg[*]}] -npaths 1 -detail full_path -stdout
puts "########## fetch skid -> w_need (pre-fix reference) ##########"
report_timing -setup -to [get_registers {emu|core|u_blit|u_blit_fetch|w_need[*]}] -npaths 1 -detail full_path -stdout
puts "########## gov m_rcs -> q_mem (pre-fix reference) ##########"
report_timing -setup -from [get_registers {emu|core|u_blit|u_blit_gov|m_rcs[*]}] -npaths 1 -detail full_path -stdout
project_close
