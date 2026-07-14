#!/usr/bin/env bash
# ============================================================================
# H7a step-3 accept  (docs/blitter_todo.md, H-stage H7).
#
# blit_batch (K=8-objline train batcher) + blit_port_beh (perfect train port)
# replace blit_vram_beh behind the SAME engine channels (BATCH=1 build of
# tb_blit).  Accept, per trace:
#
#  (1) FULL-TRACE CONFORMANCE - every EXEC pixel-exact vs golden, cost-exact,
#      gov-bound (tb_blit exits nonzero on divergence), with the descriptor
#      footprint checker armed.
#  (2) A/B TRANSPARENCY - gold_hash, rtl_vram_hash AND gov_hash of the full
#      batch-build run must be bit-identical to the frozen H6 reference logs
#      (build/h6/<t>.conf.log).  The batch layer may only move cycle counts.
#  (3) PORT-JITTER INVARIANCE - a +portjit run (seeded stalls on read
#      commands, read data and write accepts) over the JITEXECS slice must
#      reproduce the H6 baseline hashes (build/h6/<t>.base.log) bit for bit:
#      the batch layer's stall handling is timing-closed.
#
# Traces run in parallel (independent binaries, one core each).
#
#   ./run_h7a_step3.sh                  # full accept (needs build/h6 logs)
#   ./run_h7a_step3.sh --execs 300      # bounded smoke
# ============================================================================
set -uo pipefail
cd "$(dirname "$0")"

EXECS=-1
JITEXECS=2000
PJIT=305419896        # 0x12345678
while [ $# -gt 0 ]; do
  case "$1" in
    --execs)    EXECS=$2;    shift 2;;
    --jitexecs) JITEXECS=$2; shift 2;;
    --portjit)  PJIT=$2;     shift 2;;
    *) echo "unknown arg: $1"; exit 2;;
  esac
done
[ "$EXECS" -ge 0 ] && [ "$EXECS" -lt "$JITEXECS" ] && JITEXECS=$EXECS

TB=build/obj_blit_batch/Vtb_blit
REF=build/h6
TRACES=(ibarao_attract ddpdfk_attract ddpsdoj_attract futaribl_attract
        futari15_attract akatana_attract deathsml_attract espgal2a_attract)

echo "== building tb_blit [BLIT_BATCH] =="
BUILD_ONLY=1 BATCH=1 ./build_blit_tb.sh >/dev/null || { echo "build FAILED"; exit 1; }

execflag=""; [ "$EXECS" -ge 0 ] && execflag="--execs $EXECS"
OUT=build/h7a_s3; mkdir -p "$OUT"

hx() { grep -oP "$1=\K[0-9a-f]+" "$2" 2>/dev/null | tail -1; }

echo "== launching ${#TRACES[@]} traces in parallel =="
for t in "${TRACES[@]}"; do
  (
    "./$TB" --trace "blitstudy/traces/$t.blit" $execflag \
        > "$OUT/$t.conf.log" 2>&1
    echo $? > "$OUT/$t.conf.rc"
    "./$TB" --trace "blitstudy/traces/$t.blit" --execs "$JITEXECS" \
        +portjit="$PJIT" > "$OUT/$t.pjit.log" 2>&1
    echo $? > "$OUT/$t.pjit.rc"
  ) &
done
wait

fails=0
printf "\n%-18s %-8s %-6s %-5s %-6s %-17s\n" TRACE EXECS CONF A/B PJIT GOV_HASH
printf '%.0s-' {1..66}; echo
for t in "${TRACES[@]}"; do
  conf=OK; [ "$(cat "$OUT/$t.conf.rc")" = 0 ] || { conf=FAIL; fails=$((fails+1)); }
  nex=$(grep -oP ': \K[0-9]+(?=/)' "$OUT/$t.conf.log" | tail -1); nex=${nex:-?}

  # (2) full-run hashes vs the frozen H6 reference (only when full-length)
  ab=OK
  if [ "$EXECS" -lt 0 ] && [ -f "$REF/$t.conf.log" ]; then
    for k in gold_hash rtl_vram_hash gov_hash; do
      [ "$(hx $k "$OUT/$t.conf.log")" = "$(hx $k "$REF/$t.conf.log")" ] || ab=FAIL
    done
  else
    ab=n/a
  fi
  [ "$ab" = FAIL ] && fails=$((fails+1))

  # (3) port-jitter slice vs the H6 @JITEXECS baseline
  pj=OK
  ref="$REF/$t.base.log"; [ -f "$ref" ] || ref=""
  if [ -n "$ref" ] && [ "$JITEXECS" = 2000 ]; then
    for k in rtl_vram_hash gov_hash; do
      [ "$(hx $k "$OUT/$t.pjit.log")" = "$(hx $k "$ref")" ] || pj=FAIL
    done
  else
    # no like-for-like reference: at least require a clean exit
    [ "$(cat "$OUT/$t.pjit.rc")" = 0 ] || pj=FAIL
  fi
  [ "$pj" = FAIL ] && fails=$((fails+1))

  printf "%-18s %-8s %-6s %-5s %-6s %-17s\n" \
         "$t" "$nex" "$conf" "$ab" "$pj" "$(hx gov_hash "$OUT/$t.conf.log")"
done
printf '%.0s-' {1..66}; echo
if [ "$fails" -eq 0 ]; then
  echo "H7a STEP 3: PASS  (execs=${EXECS/-1/full}, portjit=$PJIT @$JITEXECS execs)"
  exit 0
else
  echo "H7a STEP 3: FAIL  ($fails failing checks) - see $OUT/*.log"
  exit 1
fi
