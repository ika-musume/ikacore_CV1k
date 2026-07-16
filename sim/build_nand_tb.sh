#!/usr/bin/env bash
# Build + run the Step-5 CV1k_nand unit accept (tb_nand) with Verilator 5:
# CV1k_nand + CV1k_ddr3_harness + a behavioural DDR3 slave holding the U2
# image; checks every streamed byte against roms/ibara/u2 (byte-exact) and the
# K9F1G08U0M read-ID.
#
#   ./build_nand_tb.sh
#   BUILD_ONLY=1 ./build_nand_tb.sh
set -euo pipefail
cd "$(dirname "$0")"

MDIR=build/obj_nand
TOP=tb_nand
BIN=V$TOP

echo "== Verilating $TOP (Verilator $(verilator --version | awk '{print $2}')) =="
verilator --binary --timing -j 0 -O3 --sv \
    -Wno-fatal \
    --Mdir "$MDIR" \
    CV1k_ddr3_harness.sv CV1k_nand.sv tb/tb_nand.sv \
    --top-module $TOP -o $BIN

if [ "${BUILD_ONLY:-0}" = "1" ]; then
    echo "== Build only; binary: $MDIR/$BIN =="
    exit 0
fi

echo "== Running =="
"./$MDIR/$BIN" "$@"
