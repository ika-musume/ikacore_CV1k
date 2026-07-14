#!/usr/bin/env bash
# ============================================================================
# H7a step-4 accept  (docs/blitter_todo.md, H-stage H7).
#
# The full execution-plane stack - blit_draw -> blit_batch -> ddr3_harness ->
# DDRAM stat slave, with blit_video PREFETCH line trains arbitrated on the
# same port - at target clock ratios (153.6 MHz / CKIO enable /3), fed at
# the modeled fetch cadence.  Per (trace x seed), tb_h7 itself enforces:
#
#   (1) every exec pixel-exact vs the H1 golden model,
#   (2) descriptor-footprint checker clean,
#   (3) per-op lateness: ZERO draws or uploads later than one hline
#       (63.586 us) vs the cost_model.h golden timeline,
#   (4) final video frame == C++ render of the DDRAM image at final SCROLL.
#
# Runs are bounded (--execs, default 300 per trace: full traces at paced
# feed are hours each; 300 execs cover attract action incl. the deathsml
# column-draw stressor geometry) across the seeded HPS-load latency tails.
#
#   ./run_h7a_step4.sh                    # 8 traces x seeds {1,2}, 300 execs
#   ./run_h7a_step4.sh --execs 100 --seeds "1"
# ============================================================================
set -uo pipefail
cd "$(dirname "$0")"

EXECS=300
SEEDS="1 2"
while [ $# -gt 0 ]; do
  case "$1" in
    --execs) EXECS=$2; shift 2;;
    --seeds) SEEDS=$2; shift 2;;
    *) echo "unknown arg: $1"; exit 2;;
  esac
done

TRACES=(ibarao_attract ddpdfk_attract ddpsdoj_attract futaribl_attract
        futari15_attract akatana_attract deathsml_attract espgal2a_attract)

echo "== building tb_h7 =="
BUILD_ONLY=1 ./build_h7_tb.sh >/dev/null || { echo "build FAILED"; exit 1; }

OUT=build/h7a_s4; mkdir -p "$OUT"

echo "== launching $((${#TRACES[@]} * $(wc -w <<<"$SEEDS"))) runs in parallel =="
for t in "${TRACES[@]}"; do
  for s in $SEEDS; do
    (
      ./build/obj_h7/Vtb_h7 --trace "blitstudy/traces/$t.blit" \
          --execs "$EXECS" --seed "$s" > "$OUT/$t.s$s.log" 2>&1
      echo $? > "$OUT/$t.s$s.rc"
    ) &
  done
done
wait

fails=0
printf "\n%-18s %-4s %-6s %-8s %-12s %-9s %-8s %s\n" \
       TRACE SEED RC DRAWS "MAXLATE_us" ">hline" UPL_us FRAME
printf '%.0s-' {1..80}; echo
for t in "${TRACES[@]}"; do
  for s in $SEEDS; do
    L="$OUT/$t.s$s.log"
    rc=$(cat "$OUT/$t.s$s.rc" 2>/dev/null); rc=${rc:-?}
    [ "$rc" = 0 ] || fails=$((fails+1))
    nd=$(grep -oP 'DRAW lateness:\s+n=\K[0-9]+' "$L");        nd=${nd:-?}
    ml=$(grep -oP 'DRAW lateness:.*max=\K[+-][0-9.]+' "$L");  ml=${ml:-?}
    nh=$(grep -oP 'DRAW lateness:.*n>hline=\K[0-9]+' "$L");   nh=${nh:-?}
    mu=$(grep -oP 'UPLOAD lateness:.*max=\K[+-][0-9.]+' "$L");mu=${mu:-?}
    fr=$(grep -oP 'video frame vs DDRAM render: \K[A-Z-]+' "$L"); fr=${fr:-?}
    printf "%-18s %-4s %-6s %-8s %-12s %-9s %-8s %s\n" \
           "$t" "$s" "$rc" "$nd" "$ml" "$nh" "$mu" "$fr"
  done
done
printf '%.0s-' {1..80}; echo
if [ "$fails" -eq 0 ]; then
  echo "H7a STEP 4: PASS  (execs=$EXECS, seeds={$SEEDS}, stat DDR3 w/ HPS tail)"
  exit 0
else
  echo "H7a STEP 4: FAIL  ($fails failing runs) - see $OUT/*.log"
  exit 1
fi
