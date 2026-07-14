#!/usr/bin/env bash
# ============================================================================
# H6 conformance gate  (docs/blitter_todo.md, H-stage H6).
#
# Pure conformance: NO RTL beyond the frozen H5 core is exercised or modified.
# This drives the .blit-trace testbench (tb_blit = blit_draw + blit_gov +
# blit_vram_beh) and the P-stage C++ jitter engine (blitstudy/blit_study), and
# asserts the four H6 properties over all eight attract traces:
#
#  (1) UNIFIED RTL REGRESSION - every EXEC is pixel-exact vs the H1 golden
#      (blitgold) AND per-op cost-exact vs cost_model.h.  tb_blit --trace
#      exits nonzero on the first divergence.
#  (2) TIMELINE BINDING - the RTL governor taps are element-identical to
#      workload::build_work(rec).gov, the EXACT per-op cost array that
#      blit_study's execution engine consumes via cost::governor().  Reported
#      as "gov-bound"; tb_blit exits nonzero otherwise.  (The harness never
#      calls engine::run_exec - the execution-plane DES lives only in
#      blit_study, so the jitter sweep stays in C++.)
#  (3) GOVERNOR INVARIANCE - a seeded --jitter feed perturbs ONLY execution-
#      plane pacing (draw-engine backpressure); the CPU-visible governor
#      timeline (gov_hash) and the rendered pixels (rtl_vram_hash) must be
#      BIT-IDENTICAL to the un-jittered run.  Only cycle counts may move.
#      Invariance is a STRUCTURAL property, so it is checked on a bounded
#      like-for-like slice (JITEXECS) - a capped-plain baseline vs a capped-
#      jitter run (the two hashes must match); the FULL trace carries the
#      conformance (1)(2) accept.
#  (4) JITTER SWEEP (C++) - blit_study in the frozen K=8 / 1-thread config over
#      N DDR3-latency seeds: 0 ops late by > 1 hline (the FINDINGS bound).
#
# Usage:
#   ./run_h6_conformance.sh                 # full-trace conformance + capped
#                                           # invariance (the H6 accept, ~3 h)
#   ./run_h6_conformance.sh --execs 300     # bounded smoke (pipeline check)
#   ./run_h6_conformance.sh --jitexecs 4000 --seeds 16 --jitter 0xC0FFEE
# ============================================================================
set -uo pipefail
cd "$(dirname "$0")"

EXECS=-1                 # -1 = full trace (the conformance accept)
JITEXECS=2000            # bounded slice for the governor-invariance check
SEEDS=8                  # DDR3-latency seeds for the C++ jitter sweep
JIT=0x0BADCAFE           # feed-jitter seed for the RTL invariance run
while [ $# -gt 0 ]; do
  case "$1" in
    --execs)    EXECS=$2;    shift 2;;
    --jitexecs) JITEXECS=$2; shift 2;;
    --seeds)    SEEDS=$2;    shift 2;;
    --jitter)   JIT=$2;      shift 2;;
    *) echo "unknown arg: $1"; exit 2;;
  esac
done
# a bounded conformance smoke also bounds the invariance slice
[ "$EXECS" -ge 0 ] && [ "$EXECS" -lt "$JITEXECS" ] && JITEXECS=$EXECS

TB=build/obj_blit/Vtb_blit
STUDY=blitstudy/blit_study
TRACES=(ibarao_attract ddpdfk_attract ddpsdoj_attract futaribl_attract
        futari15_attract akatana_attract deathsml_attract espgal2a_attract)

echo "== building tb_blit + blit_study =="
BUILD_ONLY=1 ./build_blit_tb.sh >/dev/null || { echo "tb_blit build FAILED"; exit 1; }
make -C blitstudy blit_study >/dev/null   || { echo "blit_study build FAILED"; exit 1; }

execflag=""; [ "$EXECS" -ge 0 ] && execflag="--execs $EXECS"
OUT=build/h6; mkdir -p "$OUT"
fails=0

hx() { grep -oP "$1=\K[0-9a-f]+" "$2" | tail -1; }   # last key=hex in a log

printf "\n%-18s %-8s %-6s %-6s %-17s %-7s %-11s %s\n" \
       TRACE EXECS CONF INVAR GOV_HASH ">hline" MAXLATE_us STUDY
printf '%.0s-' {1..96}; echo

for t in "${TRACES[@]}"; do
  trace=blitstudy/traces/$t.blit
  if [ ! -f "$trace" ]; then
    printf "%-18s  MISSING TRACE\n" "$t"; fails=$((fails+1)); continue
  fi

  # (1)(2) CONFORMANCE - full trace: every EXEC pixel-exact + cost-exact +
  #        RTL-taps-bound-to-build_work.  tb_blit exits nonzero on divergence.
  conf=OK
  if ! "./$TB" --trace "$trace" $execflag > "$OUT/$t.conf.log" 2>&1; then
    conf=FAIL; fails=$((fails+1))
  fi
  nex=$(grep -oP ': \K[0-9]+(?=/)' "$OUT/$t.conf.log" | tail -1); nex=${nex:-?}

  # (3) GOVERNOR INVARIANCE - capped like-for-like pair: plain baseline vs a
  #     jitter run over the SAME slice; rtl_vram_hash + gov_hash must match.
  "./$TB" --trace "$trace" --execs "$JITEXECS"                > "$OUT/$t.base.log" 2>&1
  "./$TB" --trace "$trace" --execs "$JITEXECS" --jitter "$JIT" > "$OUT/$t.jit.log"  2>&1
  bv=$(hx rtl_vram_hash "$OUT/$t.base.log"); bg=$(hx gov_hash "$OUT/$t.base.log")
  jv=$(hx rtl_vram_hash "$OUT/$t.jit.log");  jg=$(hx gov_hash "$OUT/$t.jit.log")
  invar=OK
  [ -n "$bv" ] && [ "$bv" = "$jv" ] && [ "$bg" = "$jg" ] || { invar=FAIL; fails=$((fails+1)); }

  # (4) C++ jitter sweep, frozen K=8 / 1 thread, SEEDS DDR3-latency seeds
  BLIT_OBJLINE_BATCH=8 "./$STUDY" "$trace" 1 "$SEEDS" > "$OUT/$t.study.log" 2>&1
  read -r maxlate nhline <<<"$(awk -v s="$SEEDS" '$1==1 && $2==s {print $3, $6}' "$OUT/$t.study.log")"
  nhline=${nhline:-?}; maxlate=${maxlate:-?}
  study=OK
  [ "$nhline" = "0" ] || { study="FAIL($nhline)"; fails=$((fails+1)); }

  printf "%-18s %-8s %-6s %-6s %-17s %-7s %-11s %s\n" \
         "$t" "$nex" "$conf" "$invar" "$bg" "$nhline" "$maxlate" "$study"
done

printf '%.0s-' {1..96}; echo
echo "CONF = full-trace pixel+cost+binding | INVAR = plain vs jitter @${JITEXECS} execs (bit-identical)"
if [ "$fails" -eq 0 ]; then
  echo "H6 CONFORMANCE: PASS  (conf execs=${EXECS/-1/full}, invar slice=$JITEXECS, sweep seeds=$SEEDS, feed-jitter=$JIT)"
  echo "  logs: $OUT/*.{conf,base,jit,study}.log"
  exit 0
else
  echo "H6 CONFORMANCE: FAIL  ($fails failing checks) - see $OUT/*.log"
  exit 1
fi
