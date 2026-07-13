# blitstudy — P-stage workload harvesting + performance study

Plan of record: `docs/blitter_ddr3_sched.md` §10/§10.1/§10.1.1; build
order H0 → H1 → **P** → H2… in `docs/blitter_todo.md` Part 0b.
**Results + rationale (what K means, why K=8/1-thread is frozen):
`FINDINGS.md`.**

## MAME trace hook

`mame/` carries a +72-line patch to `src/mame/cave/cv1k_v.{cpp,h}`
(mirrored here as `mame_blit_trace.patch` — reapply with
`git apply sim/blitstudy/mame_blit_trace.patch` after a MAME update).
It emits one `.blit` record per EXEC: machine time, frame number,
LIST_ADDR, shadow-latched CLIP, live SCROLL, **MAME's built-in Buffi
delay estimate** (free cross-check for our cost-model port), and the
exact op-word stream in shadow-walk order (duplicates from the
dispatch rewind are removed). Enabled by `CV1K_BLIT_TRACE=<path>`;
zero overhead when unset.

## Build (one-time deps, then ~minutes on 48 cores)

```sh
sudo apt install libsdl2-dev libsdl2-ttf-dev   # the only missing deps
cd mame
# the SOURCES scanner misses the nine cv1k_v_blit*.cpp files — list them:
BLITSRC=$(ls src/mame/cave/cv1k_v_blit*.cpp | tr '\n' ',' | sed 's/,$//')
make SOURCES=src/mame/cave/cv1k.cpp,$BLITSRC REGENIE=1 NOWERROR=1 -j48
```

The `SOURCES=` subset build produces a CV1k-only `./mame` binary.
REGENIE=1 is required whenever `src/mame/mame.lst` changes (the driver
list is generated from mame.lst, not from GAME macros).

## Capture

```sh
cd mame
mkdir -p ../sim/blitstudy/traces
CV1K_BLIT_TRACE=../sim/blitstudy/traces/ibarao_attract.blit \
  ./mame ibarao -rompath <romdir> -str 180 -nothrottle \
  -video none -sound none
```

ROM dirs must carry MAME set names/filenames — verify with
`./mame -rompath <romdir> -verifyroms <set>` first; our dumps are the
sets ibarao and espgal2a (dump-collection names are shifted for those
two). `-str 180` = 180 emulated seconds (whole attract loop incl.
gameplay demo) then exit; `-nothrottle` runs at host speed. For scene captures:
run windowed, use save states / cheats / slow-motion (§10.1.1 — no
play skill required), then re-run with the trace env set.

## Validate

```sh
cd sim/blitstudy
make && ./trace_dump -v traces/ibara_attract.blit
```

Hard-fails on header/magic/op-walk/monotonic-time violations. The
"worst MAME delay" line vs the 16.68 ms frame budget immediately shows
whether the captured scene contains real slowdown.

`scroll_check.cpp` (build: `g++ -O2 -std=c++17 -o scroll_check
scroll_check.cpp`) verifies the double-buffering claim in
FINDINGS.md §1: clip-clamped dst pixels per EXEC vs the live SCROLL
window — expected result is zero writes into the displayed window.

## Status / next

DONE: `cost_model.h` (golden timing — C++ port of
`sim/scripts/blit_cost_model.py`, anchors revalidated at every
blit_study startup, cross-checked vs `mame_delay_ns`), `ddr3_stat.h`
(measured port model incl. HPS-load tail), `engine.h` (two-plane DES;
sweeps froze K=8 / 1 thread — see FINDINGS.md). The .py remains the
derivation/provenance document; it is not executed by the pipeline.

Next: `golden_pixel.*` — draw/blend port from `cv1k_v_blit*.cpp`
(BSD-3) for function-side checks; blend=false traffic refinement;
then RTL H2 with this code as the trace-equivalence checker.
