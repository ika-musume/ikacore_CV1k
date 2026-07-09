#!/usr/bin/env bash
# Build + run the ikacore_CV1k board simulation with Verilator 5 (--timing).
# All generated artifacts go to sim/build/ ; nothing is written to ip_cores/.
#
#   ./build_sim.sh                         # build + run ibara, cap 20k insns
#   ./build_sim.sh +maxinsn=200000         # forward plusargs to the run
#   ./build_sim.sh +rom=roms/ibara_patched/other_4M.hex   # different program image
#   BUILD_ONLY=1 ./build_sim.sh            # elaborate only, do not run
set -euo pipefail
cd "$(dirname "$0")"

MDIR=build/obj_dir
TOP=tb_cv1k
BIN=V$TOP

# FASTBOOT=1: use the patched ROM (copy/FPGA/delay loops NOP'd) + preload SDRAM,
# so the sim reaches the blitter/game code in a few k insns instead of ~1M.
#   FASTBOOT=1 ./build_sim.sh +maxinsn=200000
FBDEF=""
if [ "${FASTBOOT:-0}" = "1" ]; then
    FBDEF="+define+IBARA_FASTBOOT"
    echo "== FASTBOOT enabled (patched ROM + SDRAM preload) =="
fi

# U2 NAND model config (Micron MT29F1G08, x8, 3.3 V, short power-on reset).
#
# The array MUST hold real data: once the SH-3 DMAC copies NAND pages into work
# RAM the boot parses them, and an erased (0xFF) array yields a garbage size
# (calloc(0xFFFFFFF0) -> ~1e9-iteration zero-fill).
#
# Default (NAND_ONDEMAND): mem_array is a MODEL_SV associative array backed by
# the raw dump roms/ibara/u2. A page is $fread off disk the first time it is
# touched, so the whole 128 MB device is visible, startup is instant, and RAM
# grows only with the pages the game actually reads. Writes/erases stay in the
# associative array; the dump is opened read-only and never modified.
#
# NAND_HEX=1 falls back to the old $readmemh preload (scripts/make_nand_init.py):
#   default   -> first NAND_ROWS rows only (the boot itself reads rows 0-95)
#   FULLMEM=1 -> all 65536 rows; correct, but Verilator scales badly on the
#                resulting 138 MB static array and the 277 MB text image.
NANDDEF="+define+x8 +define+V33 +define+SHORT_RESET"
if [ "${NAND_HEX:-0}" = "1" ]; then
    NAND_ROWS="${NAND_ROWS:-1024}"      # must equal the boot slice's line count
    NANDDEF="$NANDDEF +define+NAND_INIT"
    if [ "${NAND_FULLMEM:-0}" = "1" ]; then
        NANDDEF="$NANDDEF +define+FullMem"
        echo "== NAND_HEX + FULLMEM: \$readmemh of the complete 65536-page U2 image (slow) =="
    else
        NANDDEF="$NANDDEF +define+NAND_ROWS=$NAND_ROWS"
        echo "== NAND_HEX: \$readmemh of the ${NAND_ROWS}-row boot slice =="
    fi
else
    # MODEL_SV switches mem_array/pp_counter/seq_page to associative arrays.
    NANDDEF="$NANDDEF +define+MODEL_SV +define+NAND_ONDEMAND +define+NAND_ROWS=65536"
fi

echo "== Verilating (Verilator $(verilator --version | awk '{print $2}')) =="
verilator --binary --timing -j 0 -O3 --sv \
    -Wno-fatal \
    $FBDEF \
    $NANDDEF \
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
