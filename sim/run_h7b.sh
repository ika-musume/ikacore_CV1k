#!/usr/bin/env bash
# run_h7b.sh - H7b.7 full regression + wall-clock snapshot.
#
# Matrix (all cells on ikacore_CV1k_tb, the FINAL MiSTer stack, with the
# ddr3_stat-calibrated slave LIVE at seed 1):
#
#   cell A  fastboot x ibara   boot -> attract EXECs, blitgold --raw AND the
#                              scanout-frame capture vs blitgold --frame;
#                              +nandbytes stream vs the vendor-arm datum;
#                              VGA face self-check; fetch-FIFO high-water
#   cell B  fastboot x diag-1k soak (+norhex), blitgold pixel diff, with the
#                              H7b.5 YMZ probe streaming u23 reads under load
#                              (byte-exact vs the file) - arbitration accept
#   cell C  ioctl    x ibara   +mra: the FULL 152 MiB MRA stream through
#                              CV1k_ioctl (NO file planes, NO NOR preload),
#                              then the same boot - pixel-exact vs golden on
#                              its OWN stream + first-EXEC prefix + IRQ2
#                              cadence vs cell A (see the in-function note:
#                              whole-run byte-equality is structurally
#                              unstable across load modes)
#   cell D  ioctl    x diag-1k +mra with the diag ROM as the u4 slot
#                              (build/diag_1k.raw), short soak, blitgold diff
#
# plus cell A0 (datum, tb_cv1k vendor arm): regenerates the NAND-stream
# datum if build/nand_bytes_datum.bin is missing.
#
# Accepts greppable in build/h7b/<cell>.log; per-cell walltime recorded in
# build/h7b/walltime.txt (the data for ever revisiting the SDRAM-model-
# language question).  "0 ops > 1 hline" is enforced in-system as: zero
# blit_video line-fetch underruns + zero harness video-request overruns
# (the per-op lateness datum itself is tb_h7 / run_h7a_step4.sh).
#
#   ./run_h7b.sh            all cells (C+D are the slow ones: the 152 MiB
#                           stream is ~35-45 min each at +ioctl_ival=1)
#   CELLS="A B" ./run_h7b.sh   subset
#   SEED=2 ./run_h7b.sh        different stat seed (default 1)
set -uo pipefail
cd "$(dirname "$0")"

SEED="${SEED:-1}"
CELLS="${CELLS:-A B C D}"
IBARA_INSNS="${IBARA_INSNS:-8000000}"
DIAG_INSNS="${DIAG_INSNS:-3000000}"
OUT=build/h7b
mkdir -p "$OUT"
: > "$OUT/walltime.txt"
FAIL=0

note()  { echo "[run_h7b] $*"; }
tmark() { date +%s.%N; }
twall() { awk "BEGIN{printf \"%.1f\", $2-$1}"; }

record_walltime() {  # cell t0 t1
    local w; w=$(twall "$2" "$3")
    printf "%-28s %10s s\n" "$1" "$w" >> "$OUT/walltime.txt"
    note "$1 walltime: ${w}s"
}

ck() {  # cell description grep-pattern file [invert]
    local cell="$1" desc="$2" pat="$3" file="$4" inv="${5:-}"
    if [ "$inv" = "absent" ]; then
        if grep -qE "$pat" "$file"; then
            note "FAIL [$cell] $desc (found: $(grep -m1 -E "$pat" "$file"))"; FAIL=$((FAIL+1))
        else
            note "PASS [$cell] $desc"
        fi
    else
        if grep -qE "$pat" "$file"; then
            note "PASS [$cell] $desc"
        else
            note "FAIL [$cell] $desc (missing /$pat/)"; FAIL=$((FAIL+1))
        fi
    fi
}

common_checks() {  # cell log
    ck "$1" "pcen23 same-instant"        "\[pcen23\].* 0 misphase errors - PASS" "$2"
    ck "$1" "no line-fetch underrun"     "line fetch underrun"                   "$2" absent
    ck "$1" "no video request overrun"   "video request overrun"                 "$2" absent
    ck "$1" "no stray DDRAM word"        "stray DDRAM read word"                 "$2" absent
    ck "$1" "fetch FIFO depth holds"     "fetch FIFO high-water.*PASS"           "$2"
    ck "$1" "VGA face exact"             "FACE-EXACT"                            "$2"
}

build_needed() {
    [ -x build/obj_cv1k_tb_fb/Vikacore_CV1k_tb ] || FASTBOOT=1 BUILD_ONLY=1 ./build_cv1k_tb.sh
    [ -x build/obj_cv1k_tb/Vikacore_CV1k_tb ]    ||            BUILD_ONLY=1 ./build_cv1k_tb.sh
    [ -f diag/build/diag_1k.hex ] || ( cd diag && make -s )
    ( cd blitgold && make -s )
}

# --- cell A0: NAND-stream datum off the frozen vendor arm (once) ---------
nand_datum() {
    if [ ! -f build/nand_bytes_datum.bin ]; then
        note "A0: regenerating NAND datum (vendor tb_cv1k FASTBOOT + CV1K_NAND)"
        local t0 t1; t0=$(tmark)
        FASTBOOT=1 CV1K_NAND=1 DMA_MON=1 BUILD_ONLY=1 ./build_sim.sh > "$OUT/A0_build.log" 2>&1
        ./build/obj_dir/Vtb_cv1k +maxinsn=2000000 +nandbytes=65536 \
            > "$OUT/A0.log" 2>&1
        cp build/nand_bytes.bin build/nand_bytes_datum.bin
        t1=$(tmark); record_walltime "A0 nand-datum (tb_cv1k)" "$t0" "$t1"
        note "A0: datum = build/nand_bytes_datum.bin ($(stat -c%s build/nand_bytes_datum.bin) bytes)"
        note "A0: NOTE build/obj_dir now holds the CV1K_NAND arm (rebuild for run_diag.sh)"
    fi
}

cell_A() {
    local t0 t1; t0=$(tmark)
    note "cell A: fastboot ibara, seed $SEED, $IBARA_INSNS insns"
    ./build/obj_cv1k_tb_fb/Vikacore_CV1k_tb --seed "$SEED" \
        --vram "$OUT/A_vram.bin" --frame "$OUT/A_frame.bin" \
        +maxinsn="$IBARA_INSNS" \
        +blitdump="$OUT/A_blit.txt" +blitdumpmax=99 \
        +nandbytes=65536 +nandfile="$OUT/A_nand.bin" \
        +irq2log="$OUT/A_irq2.txt" \
        > "$OUT/A.log" 2>&1
    t1=$(tmark); record_walltime "A fastboot-ibara" "$t0" "$t1"
    common_checks A "$OUT/A.log"
    ./blitgold/blitgold --boardtrace "$OUT/A_blit.txt" --raw "$OUT/A_vram.bin" \
        --out "$OUT/A_frames" > "$OUT/A_gold.log" 2>&1
    ck A "blitgold --raw pixel-exact" "PIXEL-EXACT" "$OUT/A_gold.log"
    ./blitgold/blitgold --boardtrace "$OUT/A_blit.txt" --frame "$OUT/A_frame.bin" \
        --out "$OUT/A_frames" > "$OUT/A_goldframe.log" 2>&1
    ck A "blitgold --frame pixel-exact" "frame diff: PIXEL-EXACT" "$OUT/A_goldframe.log"
    if cmp -s "$OUT/A_nand.bin" build/nand_bytes_datum.bin; then
        note "PASS [A] NAND stream byte-identical to the vendor datum"
    else
        note "FAIL [A] NAND stream differs from datum"; FAIL=$((FAIL+1))
    fi
}

cell_B() {
    local t0 t1; t0=$(tmark)
    note "cell B: fastboot diag-1k soak + YMZ probe, seed $SEED, $DIAG_INSNS insns"
    ./build/obj_cv1k_tb/Vikacore_CV1k_tb --seed "$SEED" \
        --vram "$OUT/B_vram.bin" \
        +norhex=diag/build/diag_1k.hex +maxinsn="$DIAG_INSNS" \
        +blitdump="$OUT/B_blit.txt" +blitdumpmax=999 \
        +ymzdump="$OUT/B_ymz.bin" +ymzoff=0 +ymzlen=262144 \
        > "$OUT/B.log" 2>&1
    t1=$(tmark); record_walltime "B fastboot-diag1k" "$t0" "$t1"
    common_checks B "$OUT/B.log"
    ck B "YMZ probe completed" "\[ymz\] probe done: 262144 bytes" "$OUT/B.log"
    if cmp -s <(head -c 262144 roms/ibara/u23) "$OUT/B_ymz.bin"; then
        note "PASS [B] YMZ u23 readback byte-exact under diag-1k load"
    else
        note "FAIL [B] YMZ u23 readback differs"; FAIL=$((FAIL+1))
    fi
    ./blitgold/blitgold --boardtrace "$OUT/B_blit.txt" --raw "$OUT/B_vram.bin" \
        --out "$OUT/B_frames" > "$OUT/B_gold.log" 2>&1
    ck B "blitgold --raw pixel-exact" "PIXEL-EXACT" "$OUT/B_gold.log"
}

cell_C() {
    local t0 t1; t0=$(tmark)
    note "cell C: +mra ioctl-load ibara (fastboot u4 image), seed $SEED - slow (152 MiB stream)"
    ./build/obj_cv1k_tb_fb/Vikacore_CV1k_tb --seed "$SEED" +mra \
        --vram "$OUT/C_vram.bin" --frame "$OUT/C_frame.bin" \
        +ioctl_u4=roms/ibara_patched/u4_fastboot +ioctl_ival=1 \
        +maxinsn="$IBARA_INSNS" \
        +blitdump="$OUT/C_blit.txt" +blitdumpmax=99 \
        +irq2log="$OUT/C_irq2.txt" \
        > "$OUT/C.log" 2>&1
    t1=$(tmark); record_walltime "C mra-ibara (incl stream)" "$t0" "$t1"
    common_checks C "$OUT/C.log"
    ck C "MRA stream complete (159,383,552 B)" "\[ioctl\] streamed 159383552 bytes" "$OUT/C.log"
    # Load-mode equivalence.  NOT a byte-compare against cell A: the two
    # boots run ~3.4 s apart in sim time, and (H7b.7 finding, trace-diffed
    # to a single extra PTE5/NAND-R/B# poll iteration) the stat slave's
    # real-ns double bookkeeping is not translation-invariant at the exact
    # TICK_NS(625/96) x CLK_NS(2.5) coincidences every 312.5 ns - one poll
    # count flips, and from there the (authentic, board-real) list-builder
    # vs vsync race and the insn-budget cutoff make whole-run equality an
    # unstable comparator.  The honest equivalence set: the ioctl-loaded
    # boot must render pixel-exactly against the golden model on its OWN
    # stream (content equality is separately proven byte-exact by the
    # full-152-MiB tb_cv1k +ioctl_test accept), the deterministic prefix
    # (first EXEC) must equal cell A's, and the IRQ2 cadence must match.
    ./blitgold/blitgold --boardtrace "$OUT/C_blit.txt" --raw "$OUT/C_vram.bin" \
        --out "$OUT/C_frames" > "$OUT/C_gold.log" 2>&1
    ck C "blitgold --raw pixel-exact (own boot)" "PIXEL-EXACT" "$OUT/C_gold.log"
    ./blitgold/blitgold --boardtrace "$OUT/C_blit.txt" --frame "$OUT/C_frame.bin" \
        --out "$OUT/C_frames" > "$OUT/C_goldframe.log" 2>&1
    ck C "blitgold --frame pixel-exact (own boot)" "frame diff: PIXEL-EXACT" "$OUT/C_goldframe.log"
    if cmp -s <(head -5 "$OUT/A_blit.txt") <(head -5 "$OUT/C_blit.txt"); then
        note "PASS [C] first EXEC identical to cell A (deterministic prefix)"
    else
        note "FAIL [C] first EXEC differs from cell A"; FAIL=$((FAIL+1))
    fi
    local da dc
    da=$(grep IRQ2 "$OUT/A_irq2.txt" | awk '{print $2}' | awk 'NR>1{printf "%d ", $1-p} {p=$1}')
    dc=$(grep IRQ2 "$OUT/C_irq2.txt" | awk '{print $2}' | awk 'NR>1{printf "%d ", $1-p} {p=$1}')
    if [ -n "$da" ] && [ "${dc#"$da"}" != "$dc" ] || [ "${da#"$dc"}" != "$da" ]; then
        note "PASS [C] IRQ2 cadence matches cell A (deltas prefix-equal)"
    else
        note "FAIL [C] IRQ2 cadence differs (A: $da / C: $dc)"; FAIL=$((FAIL+1))
    fi
}

cell_D() {
    local t0 t1; t0=$(tmark)
    note "cell D: +mra ioctl-load diag-1k (u4 slot = diag raw), seed $SEED - slow (152 MiB stream)"
    ./build/obj_cv1k_tb/Vikacore_CV1k_tb --seed "$SEED" +mra \
        --vram "$OUT/D_vram.bin" \
        +ioctl_u4=diag/build/diag_1k.raw +ioctl_ival=1 \
        +maxinsn="$DIAG_INSNS" \
        +blitdump="$OUT/D_blit.txt" +blitdumpmax=999 \
        > "$OUT/D.log" 2>&1
    t1=$(tmark); record_walltime "D mra-diag1k (incl stream)" "$t0" "$t1"
    common_checks D "$OUT/D.log"
    ck D "MRA stream complete (159,383,552 B)" "\[ioctl\] streamed 159383552 bytes" "$OUT/D.log"
    ./blitgold/blitgold --boardtrace "$OUT/D_blit.txt" --raw "$OUT/D_vram.bin" \
        --out "$OUT/D_frames" > "$OUT/D_gold.log" 2>&1
    ck D "blitgold --raw pixel-exact" "PIXEL-EXACT" "$OUT/D_gold.log"
}

T0_ALL=$(tmark)
build_needed
nand_datum
for c in $CELLS; do "cell_$c"; done
T1_ALL=$(tmark)
record_walltime "TOTAL ($CELLS)" "$T0_ALL" "$T1_ALL"

echo
note "==== walltime snapshot ===="
cat "$OUT/walltime.txt"
echo
if [ "$FAIL" -eq 0 ]; then note "==== H7b.7 regression: ALL PASS ===="
else note "==== H7b.7 regression: $FAIL FAILURE(S) ===="; exit 1; fi
