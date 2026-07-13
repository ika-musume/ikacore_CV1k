#!/usr/bin/env bash
# Build + run the H3 draw-engine trace testbench (tb_blit) with Verilator 5.
# Standalone from the board sim: no HS3, no --timing - just blit_draw +
# blit_vram_beh + the C++ harness that links the H1 golden model and diffs
# the full VRAM after every EXEC.
#
#   ./build_blit_tb.sh                         # build + selftest
#   ./build_blit_tb.sh --trace blitstudy/traces/ibarao.blit --execs 40
#   BUILD_ONLY=1 ./build_blit_tb.sh
set -euo pipefail
cd "$(dirname "$0")"

MDIR=build/obj_blit
TOP=tb_blit
BIN=V$TOP

echo "== Verilating $TOP (Verilator $(verilator --version | awk '{print $2}')) =="
verilator --cc -j 0 -O3 --sv \
    -Wno-fatal \
    --Mdir "$MDIR" \
    --top-module $TOP \
    blit_draw.sv blit_vram_beh.sv blit_gov.sv tb/tb_blit.sv \
    --exe ../../tb/tb_blit_main.cpp \
    -CFLAGS "-O2 -std=c++17 -I$(pwd)" \
    -o $BIN

make -C "$MDIR" -f V$TOP.mk -j"$(nproc)" >/dev/null

if [ "${BUILD_ONLY:-0}" = "1" ]; then
    echo "== Build only; binary: $MDIR/$BIN =="
    exit 0
fi

echo "== Running =="
if [ $# -eq 0 ]; then
    "./$MDIR/$BIN" --selftest
else
    "./$MDIR/$BIN" "$@"
fi
