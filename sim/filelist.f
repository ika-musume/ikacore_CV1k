// ikacore_CV1k board simulation - source manifest (Verilator / any SV sim)
// ip_cores/HS3 is a symlink to the read-only SH-3 IP; files are READ only,
// all build output goes to sim/build (see build_sim.sh --Mdir).

// --- packages + interfaces first (dependency order) ---
ip_cores/HS3/cpu_core/cpu_bus_if.sv
ip_cores/HS3/cpu_core/cache_pkg.sv
ip_cores/HS3/cpu_core/int_pipe_pkg.sv
ip_cores/HS3/peri/peri_bus_if.sv

// --- SH-3 CPU core ---
ip_cores/HS3/cpu_core/cache_mem.sv
ip_cores/HS3/cpu_core/cache.sv
ip_cores/HS3/cpu_core/agu.sv
ip_cores/HS3/cpu_core/int_pipe_mem.sv
ip_cores/HS3/cpu_core/int_pipe.sv
ip_cores/HS3/cpu_core/ctrl_reg.sv
ip_cores/HS3/cpu_core/exc_handler.sv
ip_cores/HS3/cpu_core/cpu_core.sv

// --- SH7709S peripherals + chip top ---
ip_cores/HS3/peri/ibus_splitter.sv
ip_cores/HS3/peri/ibus_bridge.sv
ip_cores/HS3/peri/ibus_arb.sv
ip_cores/HS3/peri/bsc.sv
ip_cores/HS3/peri/cpg_wdt.sv
ip_cores/HS3/peri/dmac_channel.sv
ip_cores/HS3/peri/dmac.sv
ip_cores/HS3/peri/intc.sv
ip_cores/HS3/peri/ioport.sv
ip_cores/HS3/peri/rtc.sv
ip_cores/HS3/peri/tmu.sv
ip_cores/HS3/HS3.sv

// --- board: vendor memory models (patched for Verilator, see *.verilator.patch) ---
models/mt48lc2m32b2.v
models/MX29LV320E.v
// MiSTer 128MB SDRAM module (MISTER_SDRAM variant; compiles in both builds)
models/mt48lc16m16a2.v
models/mister_128mb.sv
// U2 NAND: Micron MT29F1G08 model (ID-patched to Samsung K9F1G08U0M, EC/F1)
+incdir+models/MT29F1G08ABAFA
models/MT29F1G08ABAFA/nand_die_model.v
models/MT29F1G08ABAFA/nand_model.v

// --- blitter core (sim/CV1k_blit: platform-agnostic, ships to MiSTer) ---
// (H7b.2: the board top instantiates the FULL stack - blit_top ->
//  blit_batch -> CV1k_ddr3_harness -> DDRAM face - in the 153.6 MHz
//  blit domain, every build arm)
CV1k_blit/blit_regs.sv
CV1k_blit/blit_fetch.sv
CV1k_blit/blit_gov.sv
CV1k_blit/blit_draw.sv
CV1k_blit/blit_batch.sv
CV1k_blit/blit_video.sv
CV1k_blit/blit_top.sv

// --- PCB top + board glue (CV1k_* = platform / board-integration layer) ---
// (CV1k_ddr3_harness is unconditional since H7b.2 - the one shared
//  train arbiter behind the DDRAM face; CV1k_nand is its nd client under
//  +define+CV1K_NAND, else the vendor chip model serves U2.
//  blit_vram_beh no longer instantiates on the board - it remains for
//  the H6 unit rigs)
CV1k_cpld.v
CV1k_sdram_control.sv
CV1k_ddr3_harness.sv
CV1k_nand.sv
CV1k_ioctl.sv
ddr3_beh.sv
blit_vram_beh.sv
// H7b.1 two-file split: ikacore_CV1k.sv = the portable core top (the TB
// instantiates it directly).  The MiSTer framework wrapper
// ikacore_CV1k_emu.sv (module emu) is deliberately NOT in this manifest -
// it is Quartus-only; ./build_emu_lint.sh elaborates it against the
// tb/stubs/ sys stand-ins + srcs/sys/emu_ports.vh for port parity.
ikacore_CV1k.sv
tb/cpu_tracer.sv
tb/ioctl_sim.sv
tb/blit_dsc_check.sv
tb/tb_cv1k.sv
