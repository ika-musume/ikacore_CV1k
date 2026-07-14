#!/usr/bin/env bash
# Build + run the H7a step-4 stack testbench (tb_h7) with Verilator 5:
# blit_draw + blit_batch + blit_video[PREFETCH] + ddr3_harness against the
# C++ DDRAM stat slave (ddr3_stat.h timing) at target clock ratios.
#
#   ./build_h7_tb.sh --trace blitstudy/traces/ibarao_attract.blit --execs 40 --seed 0
#   BUILD_ONLY=1 ./build_h7_tb.sh
set -euo pipefail
cd "$(dirname "$0")"

MDIR=build/obj_h7
TOP=tb_h7
BIN=V$TOP

echo "== Verilating $TOP (Verilator $(verilator --version | awk '{print $2}')) =="
verilator --cc -j 0 -O3 --sv \
    -Wno-fatal \
    --Mdir "$MDIR" \
    --top-module $TOP \
    CV1k_blit/blit_draw.sv CV1k_blit/blit_batch.sv CV1k_blit/blit_video.sv \
    CV1k_blit/ddr3_harness.sv \
    tb/blit_dsc_check.sv tb/tb_h7.sv \
    --exe ../../tb/tb_h7_main.cpp \
    -CFLAGS "-O2 -std=c++17 -I$(pwd)" \
    -o $BIN

make -C "$MDIR" -f V$TOP.mk -j"$(nproc)" >/dev/null

if [ "${BUILD_ONLY:-0}" = "1" ]; then
    echo "== Build only; binary: $MDIR/$BIN =="
    exit 0
fi

echo "== Running =="
"./$MDIR/$BIN" "$@"
