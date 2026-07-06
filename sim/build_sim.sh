#!/usr/bin/env bash
# Build + run the ikacore_CV1k board simulation with Verilator 5 (--timing).
# All generated artifacts go to sim/build/ ; nothing is written to ip_cores/.
#
#   ./build_sim.sh                         # build + run ibara, cap 20k insns
#   ./build_sim.sh +maxinsn=200000         # forward plusargs to the run
#   ./build_sim.sh +rom=rom/other_4M.hex   # different program image
#   BUILD_ONLY=1 ./build_sim.sh            # elaborate only, do not run
set -euo pipefail
cd "$(dirname "$0")"

MDIR=build/obj_dir
TOP=tb_cv1k
BIN=V$TOP

echo "== Verilating (Verilator $(verilator --version | awk '{print $2}')) =="
verilator --binary --timing -j 0 -O3 --sv \
    -Wno-fatal \
    verilator_waivers.vlt \
    --Mdir "$MDIR" \
    -f filelist.f \
    --top-module $TOP -o $BIN

if [ "${BUILD_ONLY:-0}" = "1" ]; then
    echo "== Build only (BUILD_ONLY=1); binary: $MDIR/$BIN =="
    exit 0
fi

echo "== Running =="
mkdir -p build
# user plusargs first: Verilator's $value$plusargs takes the first match, so
# anything passed here overrides the defaults that follow.
"./$MDIR/$BIN" "$@" +maxinsn=20000 +trace=build/trace_rtl.txt

echo "== Done. RTL trace: sim/build/trace_rtl.txt =="
head -20 build/trace_rtl.txt 2>/dev/null || true
