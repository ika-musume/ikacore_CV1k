project_open ikacore_CV1k -revision ikacore_CV1k
create_timing_netlist -model slow -voltage 1100 -temperature 85
read_sdc
update_timing_netlist
set c153 [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}]
set c102 [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}]

# ---- c153 cones: worst setup INTO each blit/DDR3-side module ----
foreach {tag pat} {
  gov      {emu|core|u_blit|u_blit_gov|*}
  draw     {emu|core|u_blit|u_blit_draw|*}
  fetch153 {emu|core|u_blit|u_blit_fetch|*}
  video    {emu|core|u_blit|u_blit_video|*}
  batch    {emu|core|u_batch|*}
  harness  {emu|core|u_harness|*}
  nand     {emu|core|u_u2_nand|*}
  ioctl    {emu|core|u_ioctl|*}
} {
  puts "########## c153-to $tag ##########"
  report_timing -setup -to [get_registers $pat] -npaths 3 -detail summary -stdout
}
# from-side (cross-module launches)
foreach {tag pat} {
  govF     {emu|core|u_blit|u_blit_gov|*}
  drawF    {emu|core|u_blit|u_blit_draw|*}
  fetchF   {emu|core|u_blit|u_blit_fetch|*}
  batchF   {emu|core|u_batch|*}
  harnessF {emu|core|u_harness|*}
} {
  puts "########## c153-from $tag ##########"
  report_timing -setup -from [get_registers $pat] -npaths 3 -detail summary -stdout
}

# ---- c102 cones ----
foreach {tag pat} {
  cache    {emu|core|u_hs3|u_cpu|u_cache|*}
  intpipe  {emu|core|u_hs3|u_cpu|u_int_pipe|*}
  bsc      {emu|core|u_hs3|u_bsc|*}
  pump     {emu|core|u_pump|*}
} {
  puts "########## c102-to $tag ##########"
  report_timing -setup -to [get_registers $pat] -npaths 3 -detail summary -stdout
}
foreach {tag pat} {
  bscF     {emu|core|u_hs3|u_bsc|*}
  pumpF    {emu|core|u_pump|*}
} {
  puts "########## c102-from $tag ##########"
  report_timing -setup -from [get_registers $pat] -npaths 3 -detail summary -stdout
}

# ---- cross-domain (6.2 blanket / 9.76 exception families) ----
puts "########## cross c153->c102 ##########"
report_timing -setup -from_clock $c153 -to_clock $c102 -npaths 4 -detail summary -stdout
puts "########## cross c102->c153 ##########"
report_timing -setup -from_clock $c102 -to_clock $c153 -npaths 4 -detail summary -stdout

# ---- SDRAM pin timing ----
puts "########## SDRAM_CLK out ##########"
report_timing -setup -to_clock [get_clocks {SDRAM_CLK}] -npaths 6 -detail summary -stdout
puts "########## SDRAM DQ in (dq_n bank) ##########"
report_timing -setup -from [get_ports {SDRAM_DQ[*]}] -npaths 3 -detail summary -stdout

# ---- recovery families ----
puts "########## recovery c153 ##########"
report_timing -recovery -to_clock $c153 -npaths 20 -detail summary -stdout
puts "########## recovery c102 ##########"
report_timing -recovery -to_clock $c102 -npaths 6 -detail summary -stdout

# ---- hold sanity ----
puts "########## hold worst ##########"
report_timing -hold -npaths 3 -detail summary -stdout
puts "########## removal worst ##########"
report_timing -removal -npaths 3 -detail summary -stdout
project_close
