#!/usr/bin/env bash
# Build + run the H7b.3 MiSTer top-level testbench (ikacore_CV1k_tb) with
# Verilator 5: the portable core top in its final MISTER_SDRAM + CV1K_NAND
# configuration + the 128 MB module SDRAM model, clocked by the C++
# dual-clock scheduler against the region-mapped ddr3_stat slave.
#
#   FASTBOOT=1 ./build_cv1k_tb.sh --seed 1 --frame build/tbfb_frame.bin \
#       --vram build/tbfb_vram.bin +maxinsn=8000000 +blitdump=build/tbfb_blit.txt
#   BUILD_ONLY=1 [FASTBOOT=1] ./build_cv1k_tb.sh
#
# No vendor NDA models compile into this build (the mt48lc16m16a2 inside
# mister_128mb is the jtframe-adapted model; its residual error-branch
# delays are why --timing is on).
set -euo pipefail
cd "$(dirname "$0")"

MDIR=build/obj_cv1k_tb
TOP=ikacore_CV1k_tb
BIN=V$TOP

FBDEF=""
if [ "${FASTBOOT:-0}" = "1" ]; then
    FBDEF="+define+IBARA_FASTBOOT"
    MDIR=build/obj_cv1k_tb_fb
    echo "== FASTBOOT enabled (patched ROM + SDRAM preload) =="
fi

echo "== Verilating $TOP (Verilator $(verilator --version | awk '{print $2}')) =="
verilator --cc --timing -j 0 -O3 --sv \
    -Wno-fatal \
    +define+MISTER_SDRAM +define+CV1K_NAND \
    $FBDEF \
    verilator_waivers.vlt \
    --Mdir "$MDIR" \
    --top-module $TOP \
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
    models/mt48lc16m16a2.v \
    models/mister_128mb.sv \
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
    CV1k_ioctl.sv \
    ikacore_CV1k.sv \
    tb/cpu_tracer.sv \
    tb/blit_dsc_check.sv \
    tb/ioctl_sim.sv \
    tb/ikacore_CV1k_tb.sv \
    --exe ../../tb/ikacore_CV1k_tb_main.cpp \
    -CFLAGS "-O2 -std=c++20 -I$(pwd)" \
    -o $BIN

make -C "$MDIR" -f V$TOP.mk -j"$(nproc)" > /dev/null

if [ "${BUILD_ONLY:-0}" = "1" ]; then
    echo "== Build only; binary: $MDIR/$BIN =="
    exit 0
fi

echo "== Running =="
"./$MDIR/$BIN" "$@"
