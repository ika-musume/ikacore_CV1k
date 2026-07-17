# ikacore_CV1k timing constraints.  [H7b.8]
#
# Clock architecture (static, rtl/pll/pll_0002.v - ONE fractional VCO at
# 1228.8 MHz, exact-ratio outputs):
#   counter[0] 153.6 MHz  blit / DDR3 face / video       ("c153")
#   counter[1] 102.4 MHz  CPU / board glue / SDRAM ctrl  ("c102")
#   counter[2] 102.4 MHz  SDRAM_CLK, +7833 ps preset (~ -72 deg lead,
#              OSD-trimmable +/-16 taps of 101.7 ps at runtime)
# c153 and c102 are RELATED (same VCO) with coincident rising edges on
# the 51.2 MHz CKIO grid (period 19.53 ns).  The design contract (CDC
# audit, sim/ikacore_CV1k.sv blitter section): every crossing surface
# changes only on CKIO protocol edges and is consumed at the next grid
# edge - EXCEPT the documented mid-grid movers constrained tighter below.

derive_pll_clocks
derive_clock_uncertainty

# Derived-clock handles (fracn reconfigurable altera_pll atom paths -
# same generated structure as the proven DDR3Test core SDC).
set pin_c153 {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}
set pin_c102 {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}
set pin_csdr {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[2].output_counter|divclk}
set c153 [get_clocks $pin_c153]
set c102 [get_clocks $pin_c102]

#---------------------------------------------------------------------
# SDRAM (MiSTer 128 MB module, AS4C32M16SB-7 class, double-pump at
# 102.4 MHz CL2).  H7b.8 structure (jtframe 96 MHz recipe, calibrated
# against the jtcores sweep - working SDRAM_CLK pad-delay window is
# 3.5-8.25 ns across module types):
#   * SDRAM_CLK leaves through an altddio_out IOE cell clocked by
#     counter[2] (+2950 ps preset) -> chip edge ~ preset + IOE tCO
#     ~ 5.4 ns after the c102 fabric edge (mid-window; OSD DPS nudge
#     +/-16 x 101.7 ps is the per-module trim).
#   * ALL DQ reads land first in the pump's unconditional NEGEDGE
#     capture bank dq_n (IOE-packed via FAST_INPUT_REGISTER): each CL2
#     beat straddles a c102 falling edge with ~3 ns of setup and hold
#     at datasheet tAC 6.0 / tOH 2.5.  Everything downstream of dq_n is
#     register-to-register fabric timing - no other pad-timed DQ path
#     exists (the old live-DQ consistent-snapshot arm is gone).
# I/O numbers are -7 grade datasheet nominals @ CL2: tAC 6.0 / tOH 2.5,
# tDS 1.5 / tDH 0.8 (data), tIS 1.5 / tIH 0.8 (cmd/addr).
#---------------------------------------------------------------------
create_generated_clock -name SDRAM_CLK \
    -source [get_pins -compatibility_mode $pin_csdr] [get_ports {SDRAM_CLK}]

set_input_delay  -clock SDRAM_CLK -max 6.0 [get_ports {SDRAM_DQ[*]}]
set_input_delay  -clock SDRAM_CLK -min 2.5 [get_ports {SDRAM_DQ[*]}]
set_output_delay -clock SDRAM_CLK -max 1.5 [get_ports {SDRAM_DQ[*]}]
set_output_delay -clock SDRAM_CLK -min -0.8 [get_ports {SDRAM_DQ[*]}]
set_output_delay -clock SDRAM_CLK -max 1.5 [get_ports {SDRAM_A[*] SDRAM_BA[*] SDRAM_nCS SDRAM_nRAS SDRAM_nCAS SDRAM_nWE SDRAM_DQML SDRAM_DQMH SDRAM_CKE}]
set_output_delay -clock SDRAM_CLK -min -0.8 [get_ports {SDRAM_A[*] SDRAM_BA[*] SDRAM_nCS SDRAM_nRAS SDRAM_nCAS SDRAM_nWE SDRAM_DQML SDRAM_DQMH SDRAM_CKE}]

# dq_n beat capture: chip rise at +2.95 (ideal waveform) launches; the
# capturing fall is the NEXT one (+14.65), not the same-period 4.88 edge
# - one multicycle to select the physical pair.  Margins are then set by
# the preset/OSD sweep, exactly like jtframe (which closes this window
# empirically rather than by STA).
set_multicycle_path -setup -end 2 -from [get_clocks SDRAM_CLK] -to [get_registers {emu|core|u_pump|dq_n[*]}]
set_multicycle_path -hold  -end 1 -from [get_clocks SDRAM_CLK] -to [get_registers {emu|core|u_pump|dq_n[*]}]

#---------------------------------------------------------------------
# CKIO-grid crossings between c102 and c153 (the H7b.2 two-clock scheme).
# Contract budget is one CKIO period (19.53 ns) for grid-launched
# surfaces, but ONE blit period must cover the documented mid-grid
# movers (CS6 o_D STATUS draw-floor term, CV1k_nand dq/rb_n) - so both
# directions are simply bounded at one 153.6 MHz period.  This is
# conservative for the grid-launched surfaces (all registered, shallow
# comb) and exact for the movers; revisit per-path only if the fitter
# ever struggles here.
#---------------------------------------------------------------------
set_max_delay 6.2 -from $c102 -to $c153
set_min_delay 0   -from $c102 -to $c153
set_max_delay 6.2 -from $c153 -to $c102
set_min_delay 0   -from $c153 -to $c102

# The ONE genuine mid-grid 102.4->153.6 launch: HS3's registered CKIO
# phase flop (ckio_ph, = CKIO_PCEN) sampled by the blit-domain enable
# regenerator p3_q.  2/12-grid window = 3.26 ns - keep it tighter than
# the blanket above (sim/ikacore_CV1k.sv "Silicon note").
set_max_delay 3.0 -from [get_registers {emu|core|u_hs3|u_cpg_wdt|ckio_ph}] -to [get_registers {emu|core|p3_q}]
set_min_delay 0   -from [get_registers {emu|core|u_hs3|u_cpg_wdt|ckio_ph}] -to [get_registers {emu|core|p3_q}]

# Framework clocks (CLK_50M cfg FSM, CLK_AUDIO rtc divider, HDMI/video
# PLLs) are grouped asynchronous to the core PLL by sys/sys_top.sdc;
# the crossings here are 2-FF synchronized or quasi-static (OSD values,
# 32.768 kHz RTC tick).

# RTC tick: emu divides CLK_AUDIO (24.576 MHz) by 750 into rtc_32k, which
# clocks HS3's RTC prescaler (EXTAL2 on silicon).  Declare it (STA flagged
# an undeclared clock) and cut it loose - the RTC treats it as an async
# crystal input exactly like the real chip.
create_generated_clock -name rtc32k -source [get_pins {emu|rtc_32k|clk}] \
    -divide_by 750 [get_pins {emu|rtc_32k|q}]
set_clock_groups -asynchronous -group {rtc32k}

#---------------------------------------------------------------------
# blit_video pixel readout: o_px loads ONLY at dot_ce instants (1 dot =
# DOT_CKIO(8) CKIO = 24 blit clocks), and every input of that load -
# hcnt, the per-line lat_sx latch, and the display-half line_buf cells
# (never written while displayed, double-buffer contract) - also changes
# only on that cadence.  4 periods is a conservative fraction of the
# real 24-cycle window.
#---------------------------------------------------------------------
set_multicycle_path -setup -end 4 -to [get_registers {emu|core|u_blit|u_blit_video|o_px[*]}]
set_multicycle_path -hold  -end 3 -to [get_registers {emu|core|u_blit|u_blit_video|o_px[*]}]

#---------------------------------------------------------------------
# Reset-release recovery: sys_rst_n = cpu_go combs through the pump's
# loader activity (ld_go / lst) into the blit-domain async resets.  The
# arc is quiescent at every deassert instant BY CONSTRUCTION: loader
# activity only exists while i_IOCTL_DOWNLOAD holds dl_hold high (the
# stream is what feeds it), and the actual release edge is launched by
# the dl_hold_q quantizer register one CKIO_PCEN after all of it has
# gone quiet (H7b.7 cell C determinism).  Assertion needs no timing.
# dl_hold_q/cpu_por_go recovery arcs stay constrained (6.2 ns blanket).
#---------------------------------------------------------------------
set_false_path -from [get_registers {emu|core|u_pump|ld_go emu|core|u_pump|lst.*}] -to [get_registers {emu|core|u_blit|*}]
# Same class, ioctl side (fit #4 worst recovery: pk_be_acc -> blit clears):
# the ioctl's stream/hold registers only move while i_IOCTL_DOWNLOAD holds
# cpu_go low, and the release edge is the dl_hold_q quantizer's.  No
# functional ioctl->blit data arcs exist (the stream feeds harness/NAND/
# pump, never u_blit).
set_false_path -from [get_registers {emu|core|u_ioctl|*}] -to [get_registers {emu|core|u_blit|*}]

#---------------------------------------------------------------------
# Pump init sequencer: ist/init_done gate the command/address muxes but
# transition ONCE at boot, and every init-time command is icnt-paced
# many cycles apart - fit #3's c102 worst (-8.5) was ist.IS_DONE fanning
# through the runtime command mux select.  Two cycles is safe for every
# ist-launched transition.
#---------------------------------------------------------------------
set_multicycle_path -setup -end 2 -from [get_registers {emu|core|u_pump|ist.*}]
set_multicycle_path -hold  -end 1 -from [get_registers {emu|core|u_pump|ist.*}]
