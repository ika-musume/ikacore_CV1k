#!/usr/bin/env bash
# H7b.D diag-ROM accept driver (vendor-datum board sim).
#
#   ./run_diag.sh mini   [maxinsn]     - boot-path smoke (default 1.2 M insns)
#   ./run_diag.sh 1k     [maxinsn]     - 1,024-circle + wave soak (default 3 M)
#
# Flow per the H3 acceptance loop: run the board sim with the diag image
# served as U4 (+norhex), capture the backdoor op-stream (+blitdump) and
# the final behavioral VRAM (+blitvram), then let blitgold replay the
# stream and pixel-diff the dump.  PASS = blitgold reports 0 bad pixels.
# The frame PNGs land in sim/build/diag_<n>_frames/ for eyeballing.
#
# NOTE: needs the PLAIN vendor build in build/obj_dir (no FASTBOOT /
# MISTER / CV1K_NAND defines):  BUILD_ONLY=1 ./build_sim.sh
set -euo pipefail
cd "$(dirname "$0")"

ROM="${1:-mini}"
case "$ROM" in
    mini) MAXI="${2:-1200000}" ;;
    1k)   MAXI="${2:-3000000}" ;;
    *)    echo "usage: $0 {mini|1k} [maxinsn]   (ARM=mister for the pump arm)"; exit 1 ;;
esac

# ARM=mister runs the MISTER_SDRAM build (build/obj_dir_mister, plain -
# no FASTBOOT); default is the vendor-datum build (build/obj_dir)
OBJ="obj_dir"
if [ "${ARM:-vendor}" = "mister" ]; then OBJ="obj_dir_mister"; fi

make "build/diag_${ROM}.hex"

cd ..
if [ ! -x "build/$OBJ/Vtb_cv1k" ]; then
    echo "== building plain board sim ($OBJ) =="
    if [ "$OBJ" = "obj_dir_mister" ]; then MISTER=1 BUILD_ONLY=1 ./build_sim.sh
    else BUILD_ONLY=1 ./build_sim.sh; fi
fi

echo "== running diag_${ROM} ($OBJ, maxinsn=$MAXI) =="
"./build/$OBJ/Vtb_cv1k" \
    +norhex="diag/build/diag_${ROM}.hex" \
    +maxinsn="$MAXI" \
    +trace="build/diag_${ROM}_trace.txt" \
    +blitdump="build/diag_${ROM}_blit.txt" +blitdumpmax=999 \
    +blitvram="build/diag_${ROM}_vram.bin" \
    +irq2log="build/diag_${ROM}_irq2.txt" \
    | tee "build/diag_${ROM}_run.log" | grep -E "\[tb\]|\[blit|\[trace" || true

echo "== progress mailbox (from the retired-insn trace) =="
grep -o "d1a6....\b" "build/diag_${ROM}_trace.txt" | sort | uniq -c | head || true

echo "== blitgold replay + VRAM diff =="
( cd blitgold && make -s )
mkdir -p "build/diag_${ROM}_frames"
./blitgold/blitgold \
    --boardtrace "build/diag_${ROM}_blit.txt" \
    --raw "build/diag_${ROM}_vram.bin" \
    --out "build/diag_${ROM}_frames"
