# ikacore_CV1k timing constraints.
#
# Clock architecture (static PLL, rtl/pll/pll_0002.v - ONE fractional VCO
# at 1228.8 MHz, exact-ratio outputs):
#   counter[0] 153.6 MHz  blit / DDR3 face / video       ("c153", /8)
#   counter[1] 102.4 MHz  CPU / board glue / SDRAM ctrl  ("c102", /12)
#   counter[2] 102.4 MHz  SDRAM_CLK copy, +1524 ps static lead (phase
#              56.26 deg; OSD DPS trim +/-16 x 101.7 ps at runtime)
#
# c153 and c102 are RELATED (same VCO): 3 c153 = 2 c102 = 1 CKIO period
# (19.53 ns), rising edges coincident on every CKIO boundary.  1 char
# below ~ 0.81 ns:
#
#   ns        0       6.5     13.0    19.5
#   c153      /‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\   6.51 ns
#   c102      /‾‾‾‾‾\_____/‾‾‾‾‾\_____/‾‾‾‾   9.766 ns
#   CKIO      /‾‾‾‾‾‾‾‾‾‾‾\___________/‾‾‾    19.53 ns (enable only,
#             ▲A          ▲B          ▲A           never a fabric clock)
#
#   A = coincident edges (CKIO rises): ALL grid-contract surfaces launch
#       here (CKIO_PCEN-gated registers on both clocks).
#   B = the mid-CKIO c102 edge: the pump's parse/capture edge.
#       A-launch -> B-capture = ONE c102 period, 9.766 ns = the TRUE
#       budget of every PCEN-launched c153->c102 arc (the bf_pump block
#       below).  Default related-clock analysis would instead pick the
#       13.0 -> 19.5 pair (3.26 ns) - launches that never happen.
#
# Design contract (CDC audit, sim/ikacore_CV1k.sv blitter section): every
# crossing surface changes only on CKIO protocol edges and is consumed at
# the next grid edge - EXCEPT the documented mid-grid movers, which get
# one c153 period (the 6.2 ns blanket) or their own line below.

derive_pll_clocks
derive_clock_uncertainty

# Derived-clock handles (same generated atom paths as the DDR3Test SDC).
set pin_c153 {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}
set pin_c102 {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}
set pin_csdr {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[2].output_counter|divclk}
set c153 [get_clocks $pin_c153]
set c102 [get_clocks $pin_c102]

# NOTE: no clock-uncertainty bias.  A fitter-only +1.5 ns intra-c102 bias
# was tried and made c102 WORSE (-5.93 -> -7.92): blanket uncertainty
# marks thousands of easy paths failing and the TNS-driven fitter dilutes
# effort.  Effort-steering that works = shrinking real TNS with RTL.

#---------------------------------------------------------------------
# HS3 M10K mixed-port RDW modeling arcs (the HS3 reference OOC flow
# carries exactly this exception; verified against cache_mem.sv).  A
# write-port we/datain input register has NO real launch arc: the write
# completes inside the following cycle, the read capture launches from
# the read-side registers, and a colliding read value is never consumed
# (the explicit write-through bypass registers supply it).
set_false_path -from [get_registers {*|cache:u_cache|*WRITE_ENABLE* *|cache:u_cache|*we_reg* *|cache:u_cache|*DATA_IN* *|cache:u_cache|*datain_reg* *u_gpr_bram*WRITE_ENABLE* *u_gpr_bram*we_reg* *u_gpr_bram*DATA_IN* *u_gpr_bram*datain_reg*}]

# dq_n fan-out: no exception needed (kept as a design note).  Every grid
# CAS is geared (sideband/announce predictors), so dq_n's complete fabric
# fan-out is the three pump-local capture registers rd_hi_e/rd_lo/
# nor_data beside the IOE column - no dq_n path reaches the D bus, the
# BSC, or the blitter.

#---------------------------------------------------------------------
# SDRAM (MiSTer 128 MB module, AS4C32M16SB-7 class, double-pumped at
# 102.4 MHz CL2; jtframe 96 MHz recipe - the hardware-proven pad-delay
# window across module types is 3.5-8.25 ns, and the OSD DPS trim owns
# the per-module margin; the numbers below are datasheet nominals for
# bookkeeping, not the closure mechanism).
#
# Port phases (1 char ~ 0.81 ns; SDRAM_CLK as fitted: rise +1.52,
# fall +6.40 at the port):
#
#   ns          0        4.88     9.77     14.6     19.5
#   c102 fab    /‾‾‾‾‾‾\______/‾‾‾‾‾‾\______/‾‾‾‾‾‾\___
#                      F1            F2            F3      fabric falls
#   SDRAM_CLK   __/‾‾‾‾‾‾\______/‾‾‾‾‾‾\______/‾‾‾‾‾‾\_
#                 C0            C1            C2           chip rises +1.52
#
#   CMD/A/DQout ==X=========================X============  IOE regs launch at
#                 fabric rise 0; chip registers at C1 (one period later,
#                 setup ~ 9.8 + 1.5 - tCO, ample).  The formal single-
#                 cycle check against C0 is what the SDRAM_CLK "-1.7"
#                 output-summary line reports - the documented OSD-sweep-
#                 owned class, not a real hazard.
#
#   DQ read     ______________<==== beat k ====>_________  chip launches at
#   (CL2 beat)                C1+tAC ...... C2+tOH          its rise; the beat
#   dq_n capture ..............................▲ F2         straddles a fabric
#                                                           FALL; dq_n samples
#                                                           every negedge.
#
# I/O nominals (-7 grade, CL2): tAC 6.0 / tOH 2.5, tDS 1.5 / tDH 0.8,
# tIS 1.5 / tIH 0.8.
#---------------------------------------------------------------------
create_generated_clock -name SDRAM_CLK \
    -source [get_pins -compatibility_mode $pin_csdr] [get_ports {SDRAM_CLK}]

set_input_delay  -clock SDRAM_CLK -max 6.0 [get_ports {SDRAM_DQ[*]}]
set_input_delay  -clock SDRAM_CLK -min 2.5 [get_ports {SDRAM_DQ[*]}]
set_output_delay -clock SDRAM_CLK -max 1.5 [get_ports {SDRAM_DQ[*]}]
set_output_delay -clock SDRAM_CLK -min -0.8 [get_ports {SDRAM_DQ[*]}]
set_output_delay -clock SDRAM_CLK -max 1.5 [get_ports {SDRAM_A[*] SDRAM_BA[*] SDRAM_nCS SDRAM_nRAS SDRAM_nCAS SDRAM_nWE SDRAM_DQML SDRAM_DQMH SDRAM_CKE}]
set_output_delay -clock SDRAM_CLK -min -0.8 [get_ports {SDRAM_A[*] SDRAM_BA[*] SDRAM_nCS SDRAM_nRAS SDRAM_nCAS SDRAM_nWE SDRAM_DQML SDRAM_DQMH SDRAM_CKE}]

# dq_n beat capture: the chip rise at +1.52 launches a beat; the fabric
# fall that samples it is the NEXT one (+14.6), not the same-period +4.88
# - one multicycle selects the physical pair (see the DQ-read waveform
# above).  Margins are then owned by the preset/OSD sweep, exactly like
# jtframe (which closes this window empirically rather than by STA).
set_multicycle_path -setup -end 2 -from [get_clocks SDRAM_CLK] -to [get_registers {emu|core|u_pump|dq_n[*]}]
set_multicycle_path -hold  -end 1 -from [get_clocks SDRAM_CLK] -to [get_registers {emu|core|u_pump|dq_n[*]}]

#---------------------------------------------------------------------
# CKIO-grid crossings between c102 and c153.  Grid-launched surfaces get
# a full CKIO by contract, but ONE c153 period must cover the documented
# mid-grid movers (CS6 o_D STATUS draw-floor term, CV1k_nand dq/rb_n) -
# so both directions are bounded at one 153.6 MHz period.  Conservative
# for everything registered on the grid; exact for the movers.
#---------------------------------------------------------------------
set_max_delay 6.2 -from $c102 -to $c153
set_min_delay 0   -from $c102 -to $c153
set_max_delay 6.2 -from $c153 -to $c102
set_min_delay 0   -from $c153 -to $c102

#---------------------------------------------------------------------
# blit_fetch -> pump: the TRUE single-c102 budget (register-level
# exceptions outrank the clock-level blanket above).  See the A/B edge
# waveform in the header.  Proof:
#   * every register in this launch set advances ONLY under i_CKIO_PCEN
#     (blit_fetch bus/announce regs are assigned exclusively inside the
#     `if (i_CKIO_PCEN)` block; o_REF_WIN is comb off st/run_left which
#     carry the same gate) => launches happen only at coincident edges;
#   * the pump is all-c102 and its earliest capture of these surfaces is
#     the mid-CKIO parse edge, one full c102 period (9.766 ns) later; a
#     same-instant coincident-edge sample is the HOLD side of the same
#     transfer (set_min_delay 0, as the blanket).
# The pin-through arcs (slot-A translate / write strobes into the IOE
# bank) fit one c102 period but NOT 6.2 ns once the ~4.6 ns fabric->IOE
# route is paid - which is why this exception exists.
#---------------------------------------------------------------------
set bf_pump_from [get_registers {emu|core|u_blit|u_blit_fetch|o_bus_drive \
                                 emu|core|u_blit|u_blit_fetch|o_A[*] \
                                 emu|core|u_blit|u_blit_fetch|o_CS_n \
                                 emu|core|u_blit|u_blit_fetch|o_RAS_n \
                                 emu|core|u_blit|u_blit_fetch|o_CAS_n \
                                 emu|core|u_blit|u_blit_fetch|o_WE \
                                 emu|core|u_blit|u_blit_fetch|o_DQM[*] \
                                 emu|core|u_blit|u_blit_fetch|o_SB_COL[*] \
                                 emu|core|u_blit|u_blit_fetch|o_SB_LEN[*] \
                                 emu|core|u_blit|u_blit_fetch|st.* \
                                 emu|core|u_blit|u_blit_fetch|run_left[*]}]
set_max_delay 9.76 -from $bf_pump_from -to [get_registers {emu|core|u_pump|*}]
set_min_delay 0    -from $bf_pump_from -to [get_registers {emu|core|u_pump|*}]

# The ONE genuine mid-grid 102.4->153.6 launch: HS3's registered CKIO
# phase flop (ckio_ph = CKIO_PCEN) sampled by the blit-domain enable
# regenerator p3_q.  2/12-grid window = 3.26 ns - keep it tighter than
# the blanket (sim/ikacore_CV1k.sv "Silicon note").
set_max_delay 3.0 -from [get_registers {emu|core|u_hs3|u_cpg_wdt|ckio_ph}] -to [get_registers {emu|core|p3_q}]
set_min_delay 0   -from [get_registers {emu|core|u_hs3|u_cpg_wdt|ckio_ph}] -to [get_registers {emu|core|p3_q}]

# Framework clocks (CLK_50M cfg FSM, CLK_AUDIO, HDMI/video PLLs) are
# grouped asynchronous by sys/sys_top.sdc; those crossings are 2-FF
# synchronized or quasi-static.

# RTC tick: emu divides CLK_AUDIO (24.576 MHz) by 750 into rtc_32k, which
# clocks HS3's RTC prescaler (EXTAL2 on silicon).  Declared and cut loose
# - the RTC treats it as an async crystal, exactly like the real chip.
create_generated_clock -name rtc32k -source [get_pins {emu|rtc_32k|clk}] \
    -divide_by 750 [get_pins {emu|rtc_32k|q}]
set_clock_groups -asynchronous -group {rtc32k}

#---------------------------------------------------------------------
# blit_video pixel readout: o_px loads only at dot_ce instants (1 dot =
# 24 blit clocks) and every input of that load changes on the same
# cadence (per-line latch; display-half line_buf never written while
# displayed).  4 periods is a conservative fraction of the 24.
#---------------------------------------------------------------------
set_multicycle_path -setup -end 4 -to [get_registers {emu|core|u_blit|u_blit_video|o_px[*]}]
set_multicycle_path -hold  -end 3 -to [get_registers {emu|core|u_blit|u_blit_video|o_px[*]}]

#---------------------------------------------------------------------
# Reset-release recovery (sys_rst_n = cpu_go = cpu_por_go & ~dl_hold &
# ~dl_hold_q, sim/ikacore_CV1k.sv reset sequencer).  Three proven cuts;
# dl_hold_q / cpu_por_go release arcs stay constrained on purpose.
#
# (1) Pump loader activity (ld_go/lst) combs into cpu_go.  Quiescent at
#     every deassert BY CONSTRUCTION: the loader only moves while a
#     download holds dl_hold, and the release edge is launched by the
#     dl_hold_q quantizer one CKIO_PCEN after everything is quiet.
#     -to covers every sys_rst_n async endpoint (ld_go/lst reach them
#     ONLY through the reset comb; their functional consumers are the
#     SDRAM pads and o_IOCTL_WAIT).
set_false_path -from [get_registers {emu|core|u_pump|ld_go emu|core|u_pump|lst.*}] -to [get_registers {emu|core|u_blit|* emu|core|u_batch|* emu|core|u_harness|* emu|core|u_u2_nand|* emu|core|u_u13_cpld|*}]
# (2) ioctl stream/hold registers: same quiescence, same release edge.
#     Scoped to the blit core ONLY - the stream has REAL data arcs into
#     the harness DDR3 mux and the NAND image path, which stay timed.
set_false_path -from [get_registers {emu|core|u_ioctl|*}] -to [get_registers {emu|core|u_blit|* emu|core|u_batch|*}]
# (3) The raw hps_io ioctl_download register: its fall can never be the
#     release edge - dl_hold_q sets the first blit clock after dl_hold
#     rises and clears only at a CKIO_PCEN edge, so cpu_go is still held
#     low by the quantizer when the raw bit falls.  Assertion needs no
#     timing.  Scoped to the blit core; the functional consumers
#     (CV1k_ioctl, pump dl_ff/w_dl, emu DDRAM download mux) stay timed.
set_false_path -from [get_registers {emu|hps_io|ioctl_download*}] -to [get_registers {emu|core|u_blit|* emu|core|u_batch|*}]

#---------------------------------------------------------------------
# Pump init sequencer: ist/init_done gate the runtime muxes but
# transition ONCE at boot, and every init-time command is icnt-paced
# many cycles apart.  Two cycles is safe for every ist-launched
# transition.  (The icnt compare values themselves are pre-registered in
# RTL - q_ipall/q_iref/q_imrs - NOT multicycled: icnt moves every edge,
# so a relaxed compare could sample a mid-transition value.)
#---------------------------------------------------------------------
set_multicycle_path -setup -end 2 -from [get_registers {emu|core|u_pump|ist.*}]
set_multicycle_path -hold  -end 1 -from [get_registers {emu|core|u_pump|ist.*}]
