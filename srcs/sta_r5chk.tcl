project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
set col [get_registers {ascal:ascal|*i_mem*WRITE_ENABLE* ascal:ascal|*i_mem*DATA_IN*}]
puts "r5chk match count: [get_collection_size $col]"
foreach_in_collection r $col { puts "r5chk: [get_object_info -name $r]" }
set c153 [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}]
puts "########## r5chk c153 worst after ascal fp ##########"
report_timing -setup -to_clock $c153 -npaths 6 -detail summary -stdout
project_close
