#!/usr/bin/env bash
# Lint-elaborate the MiSTer framework wrapper (module emu) under Verilator
# with the sys/ stubs (tb/stubs/), then run the port-parity check against
# srcs/sys/emu_ports.vh.  [H7b.1]
#
# The wrapper is Quartus-only in production (real hps_io/pll are Quartus
# atoms/IP); this lint pass exists so port or name drift dies here instead
# of at the H7b.8 Quartus pass.  Config = the target build: MISTER_SDRAM +
# CV1K_NAND (vendor datum models are not part of the emu build).
set -euo pipefail
cd "$(dirname "$0")"

echo "== Linting module emu (Verilator $(verilator --version | awk '{print $2}')) =="
verilator --lint-only --timing --sv -Wno-fatal \
    +define+MISTER_SDRAM +define+CV1K_NAND \
    +incdir+../srcs +incdir+tb/stubs \
    verilator_waivers.vlt \
    --top-module emu \
    tb/stubs/sys_stubs.sv \
    ip_cores/HS3/cpu_core/cpu_bus_if.sv \
    ip_cores/HS3/cpu_core/cache_pkg.sv \
    ip_cores/HS3/cpu_core/int_pipe_pkg.sv \
    ip_cores/HS3/peri/peri_bus_if.sv \
    ip_cores/HS3/cpu_core/cache_mem.sv \
    ip_cores/HS3/cpu_core/cache.sv \
    ip_cores/HS3/cpu_core/agu.sv \
    ip_cores/HS3/cpu_core/int_pipe_mem.sv \
    ip_cores/HS3/cpu_core/int_pipe.sv \
    ip_cores/HS3/cpu_core/ctrl_reg.sv \
    ip_cores/HS3/cpu_core/exc_handler.sv \
    ip_cores/HS3/cpu_core/cpu_core.sv \
    ip_cores/HS3/peri/ibus_splitter.sv \
    ip_cores/HS3/peri/ibus_bridge.sv \
    ip_cores/HS3/peri/ibus_arb.sv \
    ip_cores/HS3/peri/bsc.sv \
    ip_cores/HS3/peri/cpg_wdt.sv \
    ip_cores/HS3/peri/dmac_channel.sv \
    ip_cores/HS3/peri/dmac.sv \
    ip_cores/HS3/peri/intc.sv \
    ip_cores/HS3/peri/ioport.sv \
    ip_cores/HS3/peri/rtc.sv \
    ip_cores/HS3/peri/tmu.sv \
    ip_cores/HS3/HS3.sv \
    CV1k_blit/blit_regs.sv \
    CV1k_blit/blit_fetch.sv \
    CV1k_blit/blit_gov.sv \
    CV1k_blit/blit_draw.sv \
    CV1k_blit/blit_batch.sv \
    CV1k_blit/blit_video.sv \
    CV1k_blit/blit_top.sv \
    CV1k_cpld.v \
    CV1k_sdram_control.sv \
    CV1k_ddr3_harness.sv \
    CV1k_nand.sv \
    blit_vram_beh.sv \
    ikacore_CV1k.sv \
    ikacore_CV1k_emu.sv

echo "== Port parity vs srcs/sys/emu_ports.vh =="
python3 scripts/check_emu_ports.py

echo "== emu lint + port parity PASS =="
