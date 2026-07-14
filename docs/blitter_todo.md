# CV1000 Blitter — Implementation & Measurement Tracker

Living document. Spec lives in `blitter_detail.md` (referenced as [BD §n]);
this file tracks **state**: what is implemented, what isn't, every tunable
clock-level constant and its confirmation status, the human-measurement queue,
and results as they come in.

## How to maintain this file (rules for every session, human or Claude)

1. Never delete rows or history — flip `Status` and append to the Update Log.
2. A parameter value (Part II) may only change with a reference to a
   Measurement Log entry (Part IV) or a cited source. "It simulates nicer" is
   not a source.
3. Every implementation item flips to `DONE` only when its **acceptance test**
   passes in `sim/` (name the test in the log entry).
4. New unknowns discovered while coding → add a row (I-x / P-xx / M-xx), never
   a loose TODO comment in RTL.
5. Update Log entries: newest first, one line per change, format
   `YYYY-MM-DD  [who]  what changed (IDs)`.

Status legend: `DONE` · `WIP` · `TODO` · `BLOCKED(id)` — blocked on a
measurement or another item · `N/A`.

---

## Part 0 — Current snapshot

| Area | State |
|---|---|
| Board-level sim (SH-3 HS3 core + U4 NOR + U1 SDRAM + U2 NAND + U13 CPLD, shared bus, Verilator) | **DONE** — boots Ibara U4, executes from flash, CKIO/BSC verified vs SH7709S manual; **H0 done: CPU now clears the VBLANK loop, runs the attract sequencer, and issues double-buffered blitter EXECs** |
| Blitter RTL | **H0+H2+H3+H4+H5+H6 done** — CS6 register file `sim/CV1k_blit/blit_regs.sv` (I-1.1) + BREQ/BACK fetch unit `sim/CV1k_blit/blit_fetch.sv` (I-4.1, 40 KB attribute FIFO, fifo_study-frozen depths, runtime-paced) + draw engine `sim/CV1k_blit/blit_draw.sv` (I-1.5/6/7, 4 px/clk native speed, pixel-exact vs golden over all 8 attract traces end-to-end) + timing governor `sim/CV1k_blit/blit_gov.sv` (I-2.1/2.2/2.3/2.5: runtime-loadable cost tables, governed BUSY + IRQ1, 512-chunk fetch window, steal phase anchored on the real scanline; anchors 93/189/12,090 VCLK + 17.5 µs + 58.77 µs + 163.91 µs incl. 3 steals all hit in the board sim) + video scanout `sim/CV1k_blit/blit_video.sv` (I-3.1/2/3: 60.0184 Hz sync gen, per-line scroll latch, real line-fetch steal, vsync IRQ2, 240p capture pixel-exact vs the golden crop) live; **H6 conformance DONE 2026-07-14** (`sim/run_h6_conformance.sh`, NO RTL change — RTL trace-equivalent to the P-stage C++ engine over all 8 traces / 80,859 execs: pixel+cost+timeline-binding, and the governor timeline proven BIT-IDENTICAL under execution-plane feed jitter); **H7 steps 1+2 DONE 2026-07-15** — core repackaged into `sim/CV1k_blit/` behind ONE instance `blit_top.sv` (pure code motion; video px stream + gov table port now real boundary ports) and `blit_draw` grew the OUTPUT-ONLY descriptor sideband + `i_rd_vld` read-stall port (tied 1 = bit-identical; full-H6 + FASTBOOT-22M re-accepted, footprint checker `tb/blit_dsc_check.sv` clean over 9.13 G src beats / 26.1 G wr lanes); NEXT = `blit_batch.sv` vs a fake port, then `ddr3_harness` + ddr3_stat port model |
| VRAM (MT46V16M16 DDR) model in sim | **behavioral backend live (H3)** — `sim/blit_vram_beh.sv` (64 MB flat-pixel, 3 channels, per-pixel write lanes) serves the draw engine in board sim + trace TB; the vendor DDR model `sim/models/mt46v16m16.v` stays for I-4.2, the MiSTer DDR3 adapter (I-4.3) respins the same channels to ready/valid |
| Golden pixel model (MAME port) | **DONE 2026-07-13 (H1+H1b)** — `sim/blitgold/` C++ port of MAME `cv1k_v`; 7 unit vectors pass + pixel-correct attract frames from ibarao/futaribl/ddpdfk `.blit` traces (3 games). H1b closed the loop: a `+blitdump` testbench emitter backdoor-walks the op list from the U1 SDRAM on each EXEC; replaying **our own** board-sim output renders a coherent Ibara boot/loading frame |
| PCB measurements | **none taken** — flex PCB probe plan frozen Rev A (`docs/pcb_probe_plan.md`); **long-lead** (sourcing + PCB design), will arrive late — plan is to finish the RTL prototype (H0–H6) before the rig exists; board data later lands in the runtime-loadable governor tables |
| DDR3 scheduling study (P-stage) | **DONE 2026-07-13** — `sim/blitstudy/` (FINDINGS.md); frozen: K=8 objlines, 1 thread, ~24 KB ping-pong staging |

---

## Part 0b — H-stage build order (plan of record, agreed 2026-07-12)

Authoritative copy (was: memory/blitter-phase1-plan). The prototype is a
synthesizable RTL blitter — real BREQ/BACK bus mastering, real 240p
scanout, timing governor pacing op start/BUSY/IRQ by the cost model
regardless of native datapath speed. C++ collateral = checker harness,
not implementation. Sim uses pseudo-CDC (phase offset) between CKIO and
76.8 MHz domains; real CDC FIFO at MiSTer integration.

- **H0 — unstick CPU** (I-1.1) [**DONE 2026-07-13**]: CS6 regfile
  `sim/blit_regs.sv` per BD §3 (EXEC 0x04 latch, LIST_ADDR 0x08,
  STATUS 0x10, SCROLL/CLIP, 0x24 ack, DSW 0x50) + provisional 60 Hz
  IRQ2 tick in `ikacore_CV1k.sv` (853,333-CKIO default, `+irq2period`
  override; both retired at H5 — IRQ2 comes from the real vsync now).
  **bit1-vs-bit4 resolved**: ready = bit4 (0x10); boot-poll
  `tst #0x2` (bit1) passes because bit1 is always 0 in 0x10 — no
  conflict. **IRQ ack semantics resolved from the ISR** (0c00222c):
  vblank = IRQ2 = PTH[2] (INTC `i_IRQ[2]`), ICR1=0x8000 ⇒ falling-edge,
  IPRC|=0x0430 ⇒ priority 4; ISR clears IRR0.2 (INTC ack) and pulses
  0x24 bit0 1→0 (video-side ack); a 2nd handler clears IRR0.1 =
  IRQ1 = blitter-done (wired in H4). Accept **PASSED** (FASTBOOT board
  sim, `build/run_h0_long.log`): CPU clears the VBLANK loop, 1,694 IRQ2
  ticks (frame counter → 0x69a), and issues **double-buffered** EXECs
  (lists 0c395100/0c435200 ping-pong, scroll/clip 32↔416 — the 384-px
  page pitch corroborates P-37).
- **H1 — checker harness** [**mostly DONE 2026-07-13**]: `sim/blitgold/`
  — `golden.h` (MAME `cv1k_v` port: 3 blend LUTs, 64-way s/d-mode blend,
  tint/trans/flip, UPLOAD, DRAW field parse, `gfx_exec` decode with
  CLIP±32), `vram.h` (linear 64 MB u32 VRAM), `png.h` (dependency-free
  PNG: full-VRAM + scroll/clip crop + FNV hash), `gold_main.cpp`
  (7 unit vectors + `.blit` replay). **Validated**: unit vectors pass;
  replaying ibarao/futaribl/ddpdfk attract traces renders pixel-correct
  frames (gameplay, logos, ranking screens, alpha fog) matching MAME —
  the standalone MAME cross-check. Decoder/VRAM/golden/capture =
  I-1.2/1.3/1.4/1.8. **H1b DONE 2026-07-13**: `tb_cv1k.sv` `+blitdump`
  emitter — on each `blit_regs` EXEC pulse, backdoor-walks the op list
  straight out of the U1 SDRAM model at LIST_ADDR (bank=P[22:21],
  index=P[20:2], big-endian longword; no bus fetch — that's H2) and
  writes a text op-word trace; `blitgold --boardtrace` replays it.
  Booting the real Ibara ROM to ~20 M insns captured 8 EXECs (208 k
  words, all op lengths correct, EXIT-terminated) that render a coherent
  boot/loading frame → our own HS3+`blit_regs` output is now
  render-validated end-to-end, closing H0↔H1.
- **P — performance study** [**DONE 2026-07-13**, sched doc §10–10.4.1,
  `sim/blitstudy/FINDINGS.md`]: froze K=8 objlines / 1 thread / ~24 KB
  ping-pong staging / 2-chunk attribute lookahead / streaming uploads;
  verified on 8 games, 47 M draws, worst lateness +9.83 µs, 0 ops
  > 1 hline. The C++ engine is retained as the RTL trace-equivalence
  checker (same .blit workloads + jitter seeds).
- **H2 — fetch unit** (I-4.1 + I-2.4) [**DONE 2026-07-13**]:
  `sim/blit_fetch.sv` — per-chunk BREQ/BACK tenures vs HS3 `bsc.sv`
  (fig 10.41), CL2/BL1 SDRAM mastering (boot MCR/SDMR), op-framing
  walker, 40 KB attribute FIFO. Depths pinned by `fifo_study` FIRST
  (the "~2 chunks from cadence math" was off 100×): governor window
  512 chunks / phys FIFO 640 chunks / 8 KB upload skid (FINDINGS §5a,
  sched §10.5). **Accept PASSED**: `+blitfifo` drain log byte-identical
  to the `+blitdump` backdoor walk (84,293 lines / 5 EXECs incl. the
  132 KB boot upload), zero SDRAM-model protocol errors; STATUS busy
  now = fetch busy, so the game's ready-poll paces EXECs.
- **H3 — draw engine** (I-1.2/5/6/7, native speed) [**DONE 2026-07-13**]:
  `sim/blit_draw.sv` + `sim/blit_vram_beh.sv` — decode-ahead front end
  (10-word DRAW decode hides under the previous draw), banked 4 px/clk
  pixel pipe (src read → tint → blend → trans → masked dst write),
  flips, signed coords, CLIP±32, UPLOAD path; hazards by conservative
  rect tests (drain at op start on src/dst-vs-prev-dst overlap; strict
  write-before-next-read for self-overlap, 1-px beats inside the smear
  window). Blend LUTs computed (exact /31 as ×2115>>16), MAME dmode2
  clr0.r quirk + s0d4 full-alpha collapse + flat-didx row-underflow
  wrap reproduced bit-exact. **Accept PASSED**: trace TB
  (`build_blit_tb.sh`) diffs the full 64 MB VRAM vs H1 golden after
  EVERY exec — **all 8 attract traces end-to-end = 80,859 execs
  pixel-exact**, final scroll-window PNGs hash-identical, + 1000-exec
  random fuzz + 64-way blend grid; board sim (FASTBOOT Ibara) renders
  the boot lists in-system, `+blitvram` dump = 0 bad pixels, and
  `+blitfifo` (now the decoder's real pops) stays byte-identical to
  `+blitdump`. STATUS busy = fetch|draw until H4 owns it.
- **H4 — timing governor** (I-2.1/2.2/2.5) [**DONE 2026-07-13**]:
  `sim/blit_gov.sv` — the timing plane: arrival parser snoops the fetch
  unit's push stream (fetch_ready = real bus arrival), prices ops from
  13 runtime-loadable table entries (defaults = P_PDF), timeline FSM in
  half-VCLK ticks paces op_start = max(engine_free, fetch_ready) + hline
  steals, owns modeled BUSY + IRQ1 retirement, and implements the
  fifo_study 512-chunk governed fetch window (drainB: surviving-draw
  chunks only, upload payload exempt).  **Accept PASSED**: anchors
  93/189/12,090 VCLK exact + 17.66 µs (80× clipped) + 57.56 µs (256×5
  upload) in the board sim via the new `+blitanchor` backdoor injection;
  window-bind smoke (window=2) stalls the fetch and still lands
  busy_end on the model; all 8 traces re-run pixel-exact AND per-op
  cost-exact vs the C++ golden cost model (80,859 execs); board
  `+blitvram` vs golden replay of the FIFO stream = 0 bad pixels with
  governed BUSY + IRQ1 live.  `+blitfifo` vs `+blitdump` now differ by
  24 words in one boot exec: the game pokes 16-bit fields into the
  live list between EXEC and the fetch reaching them — an authentic
  race surfaced by H4's game-visible timing (reproduces with +noirq1;
  when they differ, golden must replay the FIFO log, the stream the
  hardware actually consumed).
  Found+fixed: H2 chunk pacing off-by-one (spacing was pace+1 CKIO).
- **H5 — video** (I-3.1/2/3, provisional params) [**DONE 2026-07-13**]:
  `sim/blit_video.sv` — sync gen 407 dots × 262 lines counted on CKIO
  enables (1 dot = 12 VCLK = exactly 8 CKIO, the same zero-drift
  integer-clock-family arithmetic as the governor's half-VCLK base);
  per-line scroll latch; 12-tile/384-px line fetch on a 3rd
  `blit_vram_beh` read channel behind a real 111-CKIO (≈166-VCLK)
  steal window that stalls the draw engine through its EXISTING
  `i_wr_rdy` backpressure port (first time that path is exercised —
  the I-4.3 DDR3 adapter uses the same mechanism; command-level read
  arbitration stays with I-4.3); vsync → IRQ2 (853,072 CKIO =
  60.0184 Hz, H0 tick + `+irq2period` retired); `o_hline` re-anchors
  the governor's steal boundary → steal phase = the real free-running
  scanline (I-2.3 closed; per-EXEC phase reset gone; `now` free-runs,
  busy_end reported as delta since EXEC).  **Accept PASSED**:
  `+blitanchor` grew tests E–G — 240×64 fired at a controlled
  scanline phase lands busy_end−first_op_start = 12,090+3×166 =
  12,588 VCLK = **163.91 µs exactly** (the [PDF] incl.-steals
  number); hline/frame periods 3256 / 853,072 CKIO exact; a 64×32
  gradient UPLOAD + DRAW copy captured off the live pixel stream
  (`+blitframe`, non-tile-aligned scroll (13,7)) is PIXEL-EXACT vs
  both the behavioral VRAM and `blitgold --frame` (MAME
  copyscrollbitmap wrap); FASTBOOT boot regression re-run with
  per-line engine stalls live: `+blitvram` still 0 bad pixels, game
  boots and paces on the real-vsync IRQ2 (double-buffered EXECs,
  alternating clip/scroll); all 8 attract traces re-run pixel-exact
  with the rebased governor.  Frame-capture caveat learned: a capture
  is only comparable when no EXEC lands between its scan window and
  the compare point (torn frames are authentic, not bugs);
  `+blitfifo` caps at 4 execs by default — pass `+blitfifomax` on
  longer runs or the golden replay silently under-paints.
- **H6 — conformance** [**DONE 2026-07-14**]: RTL vs the P-stage C++
  engine, same workloads + jitter seeds, trace-equivalent; governor
  invariance re-confirmed in RTL.  Accept = `sim/run_h6_conformance.sh`
  (**NO RTL change** — core frozen at H5; `tb_blit.sv` / `blit_vram_beh.sv`
  untouched; all new logic in the C++ harness `tb/tb_blit_main.cpp` + the
  driver; the governor taps read — o_dbg_kind/cost, o_gov_busy/retire —
  already existed).  Three gated properties, all PASS on all 8 attract
  traces (80,859 execs):
  (1) **conformance** — every EXEC pixel-exact vs the H1 golden AND
      per-op cost-exact vs cost_model.h (H3+H4 unified into one regression);
  (2) **timeline binding** — `gov_bind_check` asserts the RTL governor
      taps are element-identical to `workload::build_work(rec).gov`, the
      EXACT per-op cost array the study's `engine::run_exec` consumes via
      `cost::governor()` — so the C++ jitter sweep applies verbatim to the
      RTL.  The harness never calls `engine::run_exec` (execution-plane DES
      stays in `blit_study`), so the jitter sweep is C++-only by
      construction;
  (3) **governor invariance in RTL** — a seeded `--jitter` feed perturbs
      ONLY execution-plane pacing (draw-engine backpressure via
      `i_fifo_valid` gaps); over a 2000-exec like-for-like slice the
      CPU-visible `gov_hash` AND `rtl_vram_hash` are BIT-IDENTICAL to the
      un-jittered baseline while execution wall-clock moves (ibarao
      12.4M→14.0M CKIO) — the two-plane decoupling shown in RTL, the lean
      cousin of H7a's with/without-DDR3 test.
  Scope call (user): full-trace CONFORMANCE + capped-2000-exec INVARIANCE
  (invariance is structural).  Evidence: the frozen C++ jitter sweep
  (`blit_study`, K=8/1-thread, 8 DDR3-latency seeds) re-run per trace = 0
  ops late by > 1 hline, worst 9.83 µs (deathsml, matches FINDINGS §1) ≪
  63.586 µs.  H7 green-lit next.
- **H7 — MiSTer platform integration** (I-4.3, defined 2026-07-14):
  the blitter core (`blit_regs/fetch/draw/gov/video`) is **frozen
  as-is** — portable, platform-agnostic RTL; the algorithm does not
  change for the target.  **Three-layer decomposition** (design of
  record, 2026-07-14):
  - **`blit_draw` + descriptor sideband** — the only core touch:
    OUTPUT-ONLY ports exporting what the FRONT decode already
    computes (src base / width-in-beats / rows / row-stride, dst
    rect, blend flag, self-overlap hazard flag).  No datapath, state,
    or timing change, so H0–H5 results stay valid.  Needed because
    the beat-level address stream alone cannot drive K=8 batching: a
    snooping prefetcher only learns geometry after ~2 objlines
    (useless for the small ops that made K=8 necessary, FINDINGS §3)
    and speculative prefetch is unsafe under self-overlap hazards.
  - **`blit_batch.sv`** — NEW, platform-AGNOSTIC (joins the frozen
    side once H7a passes): realizes the FINDINGS §5 frozen contract
    in RTL — descriptor-driven address generation, K=8-objline read
    trains into the ~24 KB ping-pong staging BRAM, write-train
    assembly, serialized fallback for hazard ops.  Speaks core
    semantics upward (beat channels + descriptors), train-level
    requests downward.  Named `blit_batch`, not `blit_schedule`: it
    *forms* trains; *when* trains go on the port is arbitration (the
    harness's job), and "DDR3 scheduler" already names the P-stage
    feasibility model in `cost_model.h`.
  - **`ddr3_harness`** — the ONLY swappable module (plus its clocking
    shell): multi-client TRAIN-level arbiter in front of the platform
    RAM port (MiSTer HPS-DDR3 via f2sdram first; an Agilex 5 or
    other-part port reimplements just this module against the same
    client-side interface).  Clients, priority high→low per the
    pre-H2 contract: video line fetch DIRECT from `blit_video`
    (non-preemptive trains, ≥1-hline prefetch — never queues behind
    draw staging) > `blit_batch` draw R/W > NAND/NOR flash image
    reads > YMZ; i.e. the harness also serves U4/U2 flash requests
    from the HPS-loaded images in DDR3 (low-priority single-train
    grants).

  `blit_fetch` stays out of this stack entirely: the attribute-FIFO
  fetch rides the SH-3 bus (BREQ/BACK, authentic), so the
  fifo_study-frozen depths and the governor timeline are untouched by
  construction.  Split:
  - **H7a — sim first**: descriptor sideband on `blit_draw`, then
    `blit_batch` between the core's channels (respun to ready/valid +
    skid) and the port; behavioral HPS-DDR3 port model in the
    Verilator TB, latencies sampled from
    `blitstudy/ddr3_stat.h` (M-DDR3-calibrated histogram, PIPE
    command-FIFO, per-hline scanout preemption).  The layers test in
    isolation: `blit_batch` pixel-exact across latency seeds against
    a fake port; `ddr3_harness` arbitration/underrun with synthetic
    clients.  Accepts:
    (1) pixel-exact across latency seeds; (2) scanout never underruns;
    (3) the CPU-visible timeline (governor) is BIT-IDENTICAL with and
    without the DDR3 model — only execution-plane wall-clock may
    move; (4) fifo_study-frozen depths hold (no BREQ-side stall
    propagation); (5) FASTBOOT boot regression with NAND served
    through the harness still passes.
  - **H7b — on-target**: same harness on real Cyclone V HPS f2sdram
    (port config == the M-DDR3 benchmarked config); HPS loads the
    U4/U2 images.  Accept: on-target frame capture matches sim.

Trace-driven vs board-sim: H3/H4/H6 testbenches are `.blit`-trace-driven;
board sim is reserved for H2/H5/integration accepts — so H2 RTL can be
*built* immediately and H0 only gates its *acceptance*. Pre-H2 punch
list (2026-07-13): (1) **DONE** — `fifo_study` occupancy metrics
(FINDINGS §5a; replaced max_wait; froze window 512 ch / FIFO 640 ch /
8 KB upload skid); (2) RTL interface contract — the H3 half is **honored** (dst writes
carry per-pixel lane enables, never RMW; the H3 read channels use a
documented fixed-latency contract that the I-4.3 DDR3 adapter respins
to ready/valid + skid); the scanout-priority half: H5 delivered the
steal-window stall (scanout owns the memory for 111 CKIO/line, engine
writes gated via `i_wr_rdy`); absolute command-level priority between
trains, non-preemptive mid-train, ≥1-hline line-buffer prefetch,
NAND/YMZ low-priority single-train grants, port config == benchmarked
f2sdram config land with H7 (I-4.3); (3) optional blend=false decode in
the C++ checker — open.

---

## Part I — Implementation tracker

Order = agreed build order (function first → timing layer → video → board).
`Accept:` names the pass condition.

### Phase 1 — functional blitter (no timing)

| ID | Item | Spec | Depends | Status | Accept |
|---|---|---|---|---|---|
| I-1.1 | Register file @ CS6 0x18000000–57 (EXEC/LIST_ADDR/STATUS/SCROLL/CLIP/DSW, shadow-latch at EXEC) | [BD §3] | — | **DONE** (2026-07-13, H0) | `sim/blit_regs.sv`; exercised live by the game in the board sim — EXEC/LIST/SCROLL/CLIP latched from real CS6 writes, shadow-latch verified (double-buffered EXEC stream, `run_h0_long.log`) |
| I-1.2 | Op-list decoder (DRAW/UPLOAD/CLIP/EXIT, bit-exact fields, BE word order) | [BD §4] | I-1.1 | **DONE** (2026-07-13, H1) | `blitgold/golden.h` `Engine::exec`; validated by rendering full attract op-streams of 3 games pixel-correctly (stronger than a decode-compare) |
| I-1.3 | Behavioral VRAM model (64 MB, px2addr map; functional, no DDR timing) | [BD §6.1] | — | **DONE (functional)** (2026-07-13, H1) | `blitgold/vram.h` — **linear** 64 MB u32 VRAM (matches MAME framebuffer). px2addr DDR swizzle is pixel-invisible → deferred to the H3 RTL VRAM backend (diffed vs this golden); tile-span counting for timing already in `blitstudy/cost_model.h` `vram_tile_spans()` |
| I-1.4 | Golden pixel model: port MAME `cv1k_v` blend/draw to TB-linkable C++ | [BD §7.4] | — | **DONE** (2026-07-13, H1) | `blitgold/golden.h`; 7 unit vectors + pixel-correct frames for ibarao/futaribl/ddpdfk (3 games) vs MAME |
| I-1.5 | Draw engine datapath: src read → tint → blend ALU (4 px/clk) → trans → dst write; flipX/Y; signed dst coords | [BD §7.1, §7.4] | I-1.2/3/4 | **DONE** (2026-07-13, H3) | `sim/blit_draw.sv`; stronger than the named accept: full-VRAM (not just window) diff vs golden after EVERY exec, all 8 attract traces end-to-end (80,859 execs) + 1000-exec fuzz — zero pixel diffs; scroll-window PNG hashes match |
| I-1.6 | CLIP op + window±32 margin, exact 4-px-grid clipping at decode | [BD §8] | I-1.5 | **DONE** (2026-07-13, H3) | window/full CLIP toggles + edge/corner/underflow-wrap cases in the TB selftest and throughout the 8 game traces, pixel-exact (PinkSweets list superseded by full-corpus replay) |
| I-1.7 | UPLOAD op write path | [BD §4.2] | I-1.3 | **DONE** (2026-07-13, H3) | streams payload at FIFO rate with 4-px write packing; boot 132 KB upload + trace uploads + upload→draw readback pixel-exact; off-bottom-edge case is MAME-UB — defined as flat wrap mod 2^25 in both golden and RTL |
| I-1.8 | Frame-capture: **full-VRAM dump** (8192×4096 ARGB1555 → PNG) plus visible-window crop, at EXIT/VBLANK | — | I-1.3 | **DONE** (2026-07-13, H1) | `blitgold/png.h` — dependency-free PNG (full 8192×4096 validated + scroll/clip crop) + FNV hash for regression; produced per-frame from trace replay |

### Phase 2 — cycle-accurate timing layer

| ID | Item | Spec | Depends | Status | Accept |
|---|---|---|---|---|---|
| I-2.1 | Event/latency-table stall unit (timing decoupled from function) | [BD §7.6] | I-1.5 | **DONE** (2026-07-13, H4) | `sim/blit_gov.sv` — pure timing plane (datapath untouched, never throttled); 13 runtime-loadable table entries, reload proven live in the TB (P_PDF→P_MAME); all 8 traces + board dump zero pixel diffs with the governor active |
| I-2.2 | Draw cost scoreboard (golden `draw_cost_vclk`) as TB assertion | [BD §6.5] | I-2.1 | **DONE** (2026-07-13, H4) | anchors 93/189/12,090 VCLK exact in TB selftest AND board `+blitanchor`; stronger: EVERY op of 80,859 trace execs cost-compared vs cost_model.h — zero diffs |
| I-2.3 | hline-steal cadence: free-running line counter (4884 VCLK), 2.16 µs stall | [BD §9.2] | I-2.1 | **DONE** (2026-07-13, H4 timeline + H5 rebase — period/duration/enable runtime-loadable; `blit_gov` boundary register re-anchored each line by `blit_video.o_hline`, per-EXEC phase reset gone) | accepted: phased 240×64 = 12,090+3×166 VCLK = 163.91 µs exact in board sim; engine-side stall is real (wr_rdy gate, 111 CKIO/line) |
| I-2.4 | Op-fetch cadence model (chunk FIFO, T_CHUNK_IDLE/UPLD) | [BD §5] | I-1.2 | **DONE** (2026-07-13, H2 RTL + H4 anchors — cadences now runtime table entries; H4 fixed a pace+1 off-by-one, spacing now exactly 36/74 CKIO) | PASSED: 80 clipped draws = 17.66 µs (≈17.5 ±3%); 256×5 upload = 57.56 µs (≈58.77 ±3%) |
| I-2.5 | BUSY/READY + IRQ1 retirement timing | [BD §3, §10] | I-2.4 | **DONE** (2026-07-13, H4) | STATUS busy = governed BUSY (fetch/draw OR'd as floor); IRQ1 fires at governed retirement (replaces H3 draw-done provisional); game boots + renders with both live, busy windows = the anchor values |
| I-2.6 | DDR command-stream generator (ACT/RD/WR/PRE/(AREF)) behind the same scheduler — LA-comparable output | [BD §6.2–6.4] | I-2.2 | BLOCKED(M-6,M-10) | RTL cmd-gap trace diffs clean vs PCB capture |

### Phase 3 — real video

| ID | Item | Spec | Depends | Status | Accept |
|---|---|---|---|---|---|
| I-3.1 | Sync generator: hcnt 0..4883 @76.8, ÷12 pixel CE, 262 lines, provisional porches | [BD §9.1] | I-2.3 | **DONE 2026-07-13** (H5, `blit_video.sv` — 407×8-CKIO dots × 262 lines, parameters swappable; refine on M-1/M-2) | accepted: hline 3256 / frame 853,072 CKIO exact = 60.0184 Hz |
| I-3.2 | Line fetcher + line buffer replacing steal placeholder; scroll latch (per-line provisional) | [BD §9.3, §9.4] | I-3.1 | **DONE 2026-07-13** (H5 — 12-tile fetch, per-line latch, wrap = MAME copyscrollbitmap; latch point still M-11, fetch width M-7) | accepted: `+blitframe` capture PIXEL-EXACT vs golden crop at non-aligned scroll (13,7) |
| I-3.3 | IRQ2 generation at vsync (position per M-2) | [BD §9.1] | I-3.1 | **DONE 2026-07-13** (H5 — vsync at line 240 provisional until M-2; 0x24 ack consumed, no-op until M-2) | accepted: FASTBOOT boot paces on real-vsync IRQ2, double-buffered EXECs alternate clip/scroll |

### Phase 4 — board integration

| ID | Item | Spec | Depends | Status | Accept |
|---|---|---|---|---|---|
| I-4.1 | Wire blitter into `ikacore_CV1k.sv` CS6 + BREQ/BACK on HS3 (**pulled forward to prototype stage H2** — HS3 bsc.sv already implements BREQ/BACK) | [BD §2, §5] | I-1.1 | **DONE** (2026-07-13, H2) | game writes op lists, blitter fetches via bus mastering in board sim — PASSED: `+blitfifo` == `+blitdump` byte-identical, 5 EXECs / 84 k lines, real Ibara boot |
| I-4.2 | MT46V16M16 vendor model (or DDR-faithful behavioral) in sim | [BD §6] | I-2.6 | TODO | model passes init/LMR sequence of I-2.6 |
| I-4.3 | **= H7** MiSTer platform integration, three layers: `blit_draw` + output-only descriptor sideband (core otherwise frozen) → NEW platform-agnostic `blit_batch` (K=8-objline trains, ~24 KB ping-pong staging, hazard fallback) → ONE swappable `ddr3_harness` train-level arbiter (video > draw > NAND/NOR flash > YMZ) to HPS-DDR3; sim-first with the `ddr3_stat.h` statistical port model (H7a), then on-target f2sdram (H7b); Agilex 5 (or other) port swaps only the harness | [BD §13] | H6 | TODO | H7a: pixel-exact across latency seeds, no scanout underrun, governor timeline bit-identical w/ and w/o DDR3 model, FIFO depths hold, boot w/ harness-served NAND; H7b: on-target frame capture matches sim |

---

## Part II — Parameter registry (every clock-level constant)

One row per tunable. `Conf` = confirmed by: `[C]` cross-verified source,
`[M-nn]` our measurement, blank = unconfirmed guess.

### VRAM-domain latencies (unit: VRAM CLK, 13.0208 ns)

| ID | Param | Value | Level | Source | Conf | Notes |
|---|---|---|---|---|---|---|
| P-01 | `P_SRC_ROW_SW` read→read row switch | 5 | B | [PDF]=5 / [MAME]=6 | | fit via M-8 |
| P-02 | `P_RW_TURN` dst read→write | 20 | B | [PDF]=[MAME]=20 | | M-8 |
| P-03 | `P_WR_TURN` dst write→read | 10 | B | [PDF]=10 / [MAME]=11 | | M-8 |
| P-04 | `P_SPRITE_END` draw→draw switch | 10 | B | [PDF]=10 / [MAME]=12 | | lumped; isolate via M-8 |
| P-05 | draw→upload / upload→draw switch | **?** | A | none | | M-8 (add cases) |
| P-06 | CLIP/EXIT decode cost | 0 | B | [CLIP] "no direct latency" | | |
| P-07 | EXEC kick → first VRAM op | **?** | A | none | | M-3 |
| P-08 | hline steal duration | 166 (2.16 µs) | C | [PDF] | [C] | composition open → M-6 |
| P-09 | hline preemption granularity | row-segment (assumed) | A | inferred | | M-7 |
| P-10 | AREF per line / placement | 8, batched (assumed) | A | derived need 8.14/line | | M-6 |
| P-11 | DDR CAS latency | 2.0 (assumed) | B | LMR capture undecoded | | M-10 |
| P-12 | dst 4-px padding law (misaligned cost) | pad-to-grid model | B | [CLIP] 652 vs 1068 ns | | M-9 |
| P-13 | `CLIP_MARGIN` | 32 px | B | [MAME]/MMP observation | | M-9 variant |

### CKIO-domain / fetch (unit: ns unless noted)

| ID | Param | Value | Level | Source | Conf | Notes |
|---|---|---|---|---|---|---|
| P-20 | `T_CHUNK_IDLE` clipped-op chunk cadence | 700 | B | [MAME] fit of [CLIP] 17.5 µs | | M-5 |
| P-21 | `T_CHUNK_UPLD` upload chunk gap | 1130 | B | [PDF] | | M-5; why ≠ P-20? |
| P-22 | EXEC write → BREQ# (CKIO cycles) | **?** | A | none | | M-3 |
| P-23 | BREQ→BACK grant latency distribution | **?** | A | none | | M-4 (histogram) |
| P-24 | chunk burst beats | 16 × 32 bit | C | [PDF] | [C] | |
| P-25 | op FIFO depth (fetch-ahead) | governed window 512 chunks / phys FIFO 640 chunks (was: 2 chunks — refuted) | B/D | fifo_study 2026-07-13 (FINDINGS §5a: D=2–4 shifts the golden timeline ~3 ms; 512 = zero shift; runtime-loadable, gov table 9) | | M-5 corroborates; implemented H4 (`blit_gov` window, drainB) |

### Video timing (unit: as stated)

| ID | Param | Value | Level | Source | Conf | Notes |
|---|---|---|---|---|---|---|
| P-30 | dot clock | 6.4 MHz (÷12 CE) | B | derivation [BD §9.1] | | M-1 kills/confirms |
| P-31 | HTOTAL | 407 dots = 4884 VCLK | B | ditto | | expect 60.0184 Hz |
| P-32 | VTOTAL / visible | 262 / 240 | C | [MAME] | [C] | |
| P-33 | frame rate | 60.0184 Hz predicted (60.024 quoted) | B | [MAME] meas. of unknown precision | | M-1 |
| P-34 | H/V sync widths, porches | **?** | A | none | | M-1 |
| P-35 | IRQ2 edge vs vsync, pulse width, 0x24-ack? | **?** | A | ISR (0c00222c) | | vblank=IRQ2=PTH[2]; ICR1=0x8000⇒falling-edge, IPRC⇒pri4; ISR clears IRR0.2 + pulses 0x24 bit0 1→0. Edge-vs-vsync/width still M-2 |
| P-38 | STATUS ready bit | bit4 (0x10) | C | [MAME]/U4 boot | [C] | boot-poll `tst #0x2` (bit1) passes ∵ bit1=0 in 0x10 — reconciled, no conflict |
| P-39 | provisional IRQ2 tick period | ~~853,333 CKIO~~ retired at H5 | B | derivation | | replaced by `blit_video` vsync (853,072 CKIO); `+irq2period` gone; refine on M-1 |
| P-36 | SCROLL latch point | per-line (design assumption) | A | none | | M-11 |
| P-37 | scanout fetch width | 384 px / 12 tiles (hypothesis) | B | 166-CLK fit [BD §9.2] | | M-6 |

### Blend datapath

| ID | Param | Value | Level | Source | Conf | Notes |
|---|---|---|---|---|---|---|
| P-40 | mul/add rounding law | MAME LUT (`min(31,a·b/31)`) | B | [MAME] | | M-12 bit-compare |
| P-41 | alpha effective bits | top 5 of 8 | B | [MAME] `>>3` | | M-12 |
| P-42 | tint grid / unity | 6-bit, 0x20 = 1.0, clamp ×1.97 | B | [MAME] | | M-12 |

---

## Part III — Human measurement queue (PCB work)

Priority: ★★★ architectural (changes RTL structure) · ★★ constants ·
★ confirmation. Probe points per [PDF] "Method of gathering data".

| ID | ★ | Measurement | Probes | Resolves | Status | Result |
|---|---|---|---|---|---|---|
| M-1 | ★★★ | Frame/line rate to ppm (long-gate counter on vsync or IRQ2); then hsync width, porches, dot clock | JAMMA sync, RGB | P-30..34 | TODO | |
| M-2 | ★★ | IRQ2 edge vs vsync; pulse width; does 0x24 write deassert it | SH-3 IRQ pins, sync | P-35, I-3.3 | TODO | |
| M-3 | ★★ | CS6 EXEC write → BREQ# (CKIO cycles); → first VRAM op | CS6, BREQ, VRAM CS | P-07, P-22 | TODO | |
| M-4 | ★ | BREQ→BACK latency histogram in-game | BREQ, BACK | P-23 | TODO | |
| M-5 | ★★ | Chunk cadence: idle-op stream vs upload stream, same session | BREQ, SRAM CS | P-20, P-21, P-25 | TODO | |
| M-6 | ★★★ | Command-decode the 2.16 µs hline window: tile count, AREF count, per-tile gaps; also AREF during long idle | VRAM CS/RAS/CAS/WE/BA/A | P-08, P-10, P-37 | TODO | |
| M-7 | ★★★ | Does hline steal ever split a tile burst? | VRAM cmd + timing | P-09 | TODO | |
| M-8 | ★★ | Re-capture 8×8 / 16×12 / 240×64 draws + draw→upload→draw list; fit P-01..05 | VRAM cmd | P-01..05 | TODO | |
| M-9 | ★★ | Misalignment sweep X%4=0..3, W=4..64; clip-margin edge probe | VRAM cmd | P-12, P-13 | TODO | |
| M-10 | ★★ | Boot LMR/EMR decode: CL, burst type, init sequence | VRAM cmd + A/BA | P-11, I-2.6 | TODO | |
| M-11 | ★★ | SCROLL write mid-frame → tear position | CS6 + video out | P-36 | TODO | |
| M-12 | ★ | Photograph test-menu blend screens; bit-compare vs P-40..42 | camera/capture | P-40..42 | TODO | |
| M-13 | ★ | If -D board or FW A/D access possible: repeat M-8 per firmware | VRAM cmd | FW param sets | TODO | |

> **CPU-side queues live elsewhere:** `docs/opus_measurement_manual.md`
> (the board-measurement operator manual) owns **MC-xx** (CPU bus / U1
> SDRAM / BREQ arbitration), **MS-xx** (SH-3 pipeline via marker
> bracketing) and the **H-xx** HS3 parameter registry — same maintenance
> rules as this file. M-3/M-4/M-5 are executed as part of the MC-4/MC-5
> sessions (same captures, two consumers); their results are still logged
> here.

## Part IV — Measurement results log

Template — append one block per session, never edit old blocks:

```
### YYYY-MM-DD  M-nn  <title>
Setup: <instrument, sample rate, probe points, game/screen used>
Raw:   <numbers verbatim; capture file path under measurements/>
Derived: <fitted values>
Params updated: P-xx old→new (Part II row edited, Conf set to [M-nn])
```

*(no entries yet)*

---

## Part V — Update log (newest first)

- 2026-07-15  [claude]  **H7 steps 1+2 — blitter core repackaged behind
  `blit_top`; descriptor sideband + read-stall port landed (tied off,
  proven bit-identical).**  Two commits.  (1) `69aa128`: the five frozen
  modules moved to `sim/CV1k_blit/` and are instantiated by NEW
  `blit_top.sv` — regs/fetch/gov/draw/video + both IRQ pulse shapers
  (`+noirq1` stays in the board top as an `i_IRQ1_EN` input so blit_top
  is synthesizable); H5 steal gating moved inside (inert: behavioral
  wr_rdy = const 1); video pixel stream + governor table-load port are
  now real boundary ports (MiSTer video pipe / HPS table pokes hang off
  them); board top keeps only glue (tristates, U1 pin mux, blit_vram_beh
  on the exported beat channels); TB taps rewired `dut.u_blit.*`.
  (2) `00c1020`: `blit_draw` exports the H7 descriptor sideband as pure
  taps of the EXISTING S1-S3 setup regs — src span (s_xlo/s_xhi) + first
  row + flip walk, rows, npx, dst row-0 flat index (didx_row0, signed),
  q_blend_eff / q_strict / q_px1 / q_waitpipe, one strobe at surviving-
  DRAW commit (B_S3→B_ROW) + an UPLOAD strobe at F_UPI with base/dims —
  the exact information timing engine.h's DES was validated under (op
  start, no cross-op memory prefetch modeled), so K=8 train formation
  needs no lookahead the core doesn't have.  The ONE core-RTL touch:
  `i_rd_vld` joins the pipe-advance term `adv` (variable-latency reads
  under the H3 fixed-latency contract are physically impossible to serve
  otherwise — wr_rdy engages too late to stop the ≤3 op-start read beats;
  the same stall also IS the strict-op serialized fallback).  Tied 1'b1
  in every current build: full H6 re-run PASS (all 8 traces / 80,859
  execs conformance + 2000-exec invariance + 8-seed sweep, worst late
  9.83 µs deathsml, 0 > hline) and the 300-exec smoke gov_hashes are
  IDENTICAL pre/post-sideband; FASTBOOT 22M board regression PASS
  (5 EXECs incl. the 132 KB boot upload, blitgold FIFO-log replay
  PIXEL-EXACT, 0 bad px).  NEW `tb/blit_dsc_check.sv` (both TBs, always
  on, $fatal on miss) asserts every src beat / masked write lane lands
  inside the descriptor-predicted footprint — the property blit_batch
  relies on: clean over 9.13 G src beats + 26.1 G wr lanes (full corpus)
  + 31 k/166 k on the board boot; it caught two of its own bring-up bugs
  (same-cycle 1-word-upload descriptor consumption; wrapped flip beat
  base with sx_lo<3 — checker fixes, not RTL).  tb_blit_main gained
  `Rig::~Rig{tb.final();}` so the checker's final report prints.
  Disclosed for later steps: the SECOND core touch (agreed) is
  blit_video's ≥1-hline prefetch at H7a step 4 — a K=8 320-px blended
  train (~8.5 µs) exceeds the 2.17 µs steal window, so just-in-time line
  fetch underruns by construction; sync/steal/IRQ2 timing stays
  untouched (execution-plane fetch moves one line early).  Remaining
  H7a: blit_batch vs fake port → ddr3_harness + ddr3_stat.h C++ port
  model + lateness monitor → NAND bridge + boot-through-harness; then
  H7b on-target.
- 2026-07-14  [claude]  **H6 conformance DONE — RTL trace-equivalent to
  the P-stage C++ engine; governor invariance re-confirmed in RTL (NO RTL
  change).**  New accept `sim/run_h6_conformance.sh` + H6 additions to the
  C++ harness `tb/tb_blit_main.cpp` ONLY (frozen H5 core, `tb_blit.sv`,
  `blit_vram_beh.sv` untouched; the governor taps read — o_dbg_kind/cost,
  o_gov_busy/retire — already existed).  Three gated properties, all PASS
  on all 8 attract traces / **80,859 execs**:
  (1) full-trace **pixel-exact** (vs H1 golden) AND **per-op cost-exact**
  (vs cost_model.h) — the H3/H4 checks unified into one regression;
  (2) NEW **timeline binding** — `gov_bind_check` asserts the RTL
  governor's per-op {kind,cost} taps are element-identical to
  `workload::build_work(rec).gov`, the EXACT array the study's
  `engine::run_exec` consumes via `cost::governor()`, certifying the C++
  jitter sweep applies verbatim to the RTL; the harness never runs the
  execution-plane DES (two-plane split at the functional level — the
  jitter sweep stays in C++, `engine::run_exec`/`ddr3::Port` never called);
  (3) NEW **governor invariance in RTL** — `--jitter SEED` inserts seeded
  FIFO-feed stalls perturbing ONLY execution-plane pacing (draw-engine
  backpressure); across a 2000-exec like-for-like slice the CPU-visible
  `gov_hash` AND `rtl_vram_hash` are BIT-IDENTICAL to the un-jittered
  baseline while execution wall-clock moves (ibarao 12.4M→14.0M CKIO) —
  the lean cousin of H7a's with/without-DDR3 test (user directive: jitter
  sweep stays in C++, no in-RTL DDR3 model at H6).  Scope call (user):
  full-trace CONFORMANCE + capped-2000-exec INVARIANCE (invariance is
  structural, and already bit-identical across all 8 traces).  Evidence:
  the frozen C++ jitter sweep (`blit_study`, K=8/1-thread, 8 DDR3-latency
  seeds) re-run per trace = 0 ops late by > 1 hline, worst **9.83 µs**
  (deathsml — matches FINDINGS §1) ≪ 63.586 µs (1 hline).  New harness
  knobs: `--jitter` (feed perturbation), `gov_hash`/`rtl_vram_hash`
  (invariance witnesses), `gov-bound` tag; found no bugs (draw-engine
  backpressure path — first gapped feed in the trace TB — stays
  pixel-exact).  Sequence now H7a → H7b.
- 2026-07-14  [claude]  **H7 defined — MiSTer platform integration via a
  swappable harness (user directive).**  Rationale: a later port to
  Agilex 5 (or another part) must not touch the blitter algorithm, so
  the core RTL is frozen at its current channel contracts and every
  platform-specific concern (HPS-DDR3 access, f2sdram config, port
  clocking) is concentrated in ONE new harness module — a
  multi-client arbiter (video line fetch > blitter draw R/W >
  NAND/NOR flash image reads > YMZ, per the pre-H2 priority contract)
  that also serves U4/U2 flash requests from HPS-loaded images.
  Two sub-stages: H7a = Verilator sim with the harness backed by the
  `blitstudy/ddr3_stat.h` statistical HPS-DDR3 model (M-DDR3
  calibrated) — accepts include pixel-exactness across latency seeds,
  no scanout underrun, and a BIT-IDENTICAL governor timeline with and
  without the model (two-plane invariant); H7b = on-target Cyclone V
  f2sdram, accept = on-target frame capture matches sim.  I-4.3
  updated to point at H7; sequence is H6 → H7a → H7b.
  **Same-day refinement (user): three-layer decomposition** — the
  batching moved OUT of the harness into its own platform-agnostic
  module.  (1) `blit_draw` gains an output-only descriptor sideband
  (src/dst geometry + blend + hazard flags the FRONT decode already
  computes; no datapath/state/timing change) because the beat-level
  address stream alone cannot drive K=8 — a snooping prefetcher
  learns geometry too late for small ops and speculates unsafely
  under self-overlap hazards.  (2) NEW `blit_batch.sv` realizes the
  FINDINGS §5 contract (K=8-objline trains, ~24 KB ping-pong staging,
  hazard fallback); named batch not schedule — it forms trains,
  arbitration is the harness's job, and "DDR3 scheduler" already
  names the P-stage model.  (3) `ddr3_harness` shrinks to a
  train-level arbiter, the only per-platform module; `blit_video`
  is a DIRECT client (never queues behind draw staging);
  `blit_fetch` stays on BREQ/BACK untouched.  H6 green-lit to start
  next session (user, 2026-07-14).
- 2026-07-13  [claude]  **H5 video scanout DONE — `sim/blit_video.sv`
  (I-3.1/2/3) + governor steal-phase rebase (I-2.3 closed).**
  Sync gen: 407-dot × 262-line frame counted on CKIO enables (1 dot =
  12 VCLK = exactly 8 CKIO — same zero-drift integer arithmetic as the
  governor's half-VCLK base; no derived clocks); vsync (line 240,
  provisional til M-2) now sources IRQ2 — the H0 853,333-CKIO tick and
  `+irq2period` are retired; frame = 853,072 CKIO = 60.0184 Hz exact.
  Line fetcher: per-line scroll latch, 12 tiles/384 px per line off a
  3rd `blit_vram_beh` read channel, wrap = MAME copyscrollbitmap; the
  steal became REAL — a 111-CKIO window per line gates the draw
  engine's writes through its existing `i_wr_rdy` port (first
  exercise of that stall path; same mechanism the I-4.3 DDR3 adapter
  will drive — command-level read arbitration stays there).  Governor:
  `now` free-runs since reset, `next_bnd` re-anchors on
  `blit_video.o_hline` (exact + idempotent, both count the same CKIO
  grid), per-EXEC steal-phase reset gone, busy_end reported as delta
  (+ new `r_first_start`).  **Accepts**: `+blitanchor` A–D unchanged
  (in-tolerance shifts from the now-real phase) + new E: 240×64 fired
  at a controlled phase → busy_end−first_start = 12,090+3×166 =
  12,588 VCLK = 163.91 µs EXACT; F: hline/frame periods exact; G:
  gradient UPLOAD+DRAW captured off the live pixel stream
  (`+blitframe`, scroll (13,7) non-tile-aligned) PIXEL-EXACT vs
  behavioral VRAM and vs `blitgold --frame`; FASTBOOT 22M-insn boot:
  `+blitvram` 0 bad px WITH per-line engine stalls live, game paces
  on real-vsync IRQ2; all 8 attract traces pixel-exact with the
  rebased governor.  Gotchas recorded: (1) `+blitfifo` defaults to a
  4-exec cap — the 5th exec's absence made golden under-paint 76,800
  px until `+blitfifomax` was passed (log-cap artifact, not RTL);
  (2) a frame capture is comparable only if no EXEC lands between its
  scan window and the compare point — a torn frame (upload region
  black, draw region painted) is authentic scanout of mid-frame VRAM,
  and the first two "PIXEL-EXACT" frame accepts were vacuous
  (all-black windows: at 22M insns the game hasn't flipped scroll to
  the drawn buffer; anchor draws paint zero-src copies) — hence test
  G's nonzero content requirement; (3) `$finish` in the same timestep
  as vsync loses that vsync's frame snapshot — the anchor TB now
  delays two negedges.  `blitgold` grew `--frame` (+`--framescroll`
  for post-EXEC flips).  Confirmatory 80M-insn run (91 min wall,
  frame 610 ≈ 10.16 s sim at a steady 60.0184 Hz): VRAM diff and
  scanned frame both PIXEL-EXACT vs golden; all 5 EXECs fire in the
  first few M insns and the game then parks scroll at (0,0) over an
  unpainted region for the whole NAND-load stretch, so the visible
  window is authentically black even at 80M — the NOW-LOADING screen
  (flash ID + hex-dump table, TATE) sits fully painted in the
  (416,136) clip window (`board_clip.png`) waiting for the game to
  flip scroll onto it, which happens beyond 80M insns.  Catching it
  through the live scanout is not worth more sim-hours; test G is the
  binding I-3.2 accept.
- 2026-07-13  [claude]  **"Broken blue background" triaged — sim
  harness pacing, not a blitter bug; loader now progresses.**  User
  report: `blitgold --boardtrace` of the board dump shows black
  vertical bars in the NOW-LOADING screen.  Finding chain: (1) RTL ==
  golden == op stream (0-pixel `+blitvram` diff), and MAME's own boot
  stream has the identical exec structure (3/3/65,955/3/…, same list
  addresses) — the bars are UNPAINTED CELLS of the progressive
  loading/check screen, truthfully rendered; (2) but the game issued
  NO EXECs from 22 M to 70 M insns — stall, not slowness.  Busy probe
  (`+blitbusy`): fetch/draw/gov busy all rise+fall cleanly, STATUS
  reads ready during the stall — blitter RTL exonerated.  Windowed
  trace (new `+tracefrom` in cpu_tracer): mainline asleep, only the
  IRQ2 ISR runs (its 3 vblank-callback slots all zero), an 8-slot ×
  56 B job scheduler at 0c04deb0 finds every slot inactive — the
  loader PARKED itself.  IRQ1 was chased and eliminated: the ISR shell
  0c0021c8 acks blitter 0x24 bit1 + clears IRR0.1 and dispatches slot
  0c002224 (registered: 0c0503f0), `+blitirq1` shows the governor's
  IRQ1 delivered per exec — but MAME never implements IRQ1 ("nothing
  depends on it", cv1k.cpp TODO; the registered routine is likely its
  'profiling' handler) so it cannot be the producer.  ROOT CAUSE: the
  bring-up vblank `+irq2period=8000` CKIO (107× real) outruns the
  real-speed NAND/CPU loading work and the frame-paced loader gives
  up after ~5 EXECs.  At `+irq2period=85333` (10.7×) the game tracks
  MAME's exec sequence exactly (19,513 / 19,503 / **85,455** words =
  MAME's first asset-upload batch) and the loading screen fills solid
  blue — verified render `build/h3_slow/`.  **Rule of thumb: 8000 is
  for pre-loading boot bring-up only; use ≥85,333 (or the real
  853,333) for any run that must progress through NAND loading.**
  Kept: `+blitbusy` / `+blitirq1` probes, `+tracefrom` trace window,
  `+blitvram` dump + `blitgold --raw` diff.  IRQ1 tick stays (real
  board has it; now governor-retire-timed, H4).

- 2026-07-13  [claude]  **H4 DONE — timing governor live in the board sim**
  (I-2.1 + I-2.2 + I-2.5 + I-2.4 anchors + the engine half of I-2.3).
  New `sim/blit_gov.sv`: the TIMING PLANE of the two-plane architecture —
  never touches the execution plane (user directive: max-speed datapath,
  only the Buffi cost model stays tunable).  An ARRIVAL PARSER snoops the
  fetch unit's FIFO-push stream (fetch_ready = REAL BREQ/BACK arrival
  times ⇒ timing = f(op list) only, the draw engine's speed never enters),
  re-frames ops, applies the u16 clip test bug-for-bug (workload.h /
  gfx_draw_shadow_copy: src origin kept, dims shrunk, u16 wrap), and
  prices every surviving DRAW from RUNTIME-LOADABLE tables — BD §6.5:
  src_px/4·C_SRC4 + dst_px/4·C_DST4 + spans_src·P_SRC +
  spans_dst·(P_RW+P_WR) + P_SPR, with the 32×32 tile-span slice loop
  reproduced bug-for-bug (partial-tail double count) in closed form.  A
  TIMELINE FSM counts half-VCLK ticks (76.8 MHz = exactly 1.5× CKIO ⇒ +3
  per CKIO enable, no derived clocks), pops one queued {kind, cost,
  nslot} entry per op at op_start = max(engine_free, fetch_ready)
  (pop-when-now≥engine_free ⇒ only a few-tick non-cumulative error), adds
  hline steals incrementally against a running boundary register (no
  division; phase reset at EXEC = cost_model.h add_steals), owns the
  modeled STATUS BUSY (top OR's fetch/draw as a floor) and fires IRQ1 at
  governed retirement — replacing the H3 draw-done provisional, which
  fired unauthentically early.  13 table entries (P_SRC/RW/WR/SPR,
  per-4px rates, hline period/steal/enable, window, EXEC2BRQ/CHUNK/UPLD
  cadences — the fetch unit now paces from the tables) default to P_PDF;
  the board rig refines them later without RTL changes.  GOVERNED FETCH
  WINDOW (fifo_study drainB): only surviving-draw chunks hold virtual
  original-FIFO slots (arrived F vs governed-started R); `blit_fetch`
  holds the next attribute chunk while F−R ≥ 512, upload payload sails
  through.  **Accept PASSED** three ways: (a) trace TB — the governor
  rides every exec in warp mode and EVERY op is cost-compared against the
  C++ golden cost model: all 8 attract traces end-to-end (80,859 execs)
  pixel-exact AND cost-exact, + selftest/fuzz-1000, + anchors
  93/189/12,090 VCLK, + live table reload P_PDF→P_MAME (8×8: 93→97);
  (b) board sim `+blitanchor` (new: op lists backdoor-written to unused
  work RAM, EXEC'd via a sim-only blit_regs backdoor, timed by the real
  fetch+governor): per-draw costs 93/189/12,090 VCLK exact, 80× clipped
  = 17.66 µs (17.5 ±3 %), 256×5 upload = 57.56 µs (58.77 ±3 %; low side
  = the documented first-chunk-at-CHUNK-cadence nit), window-bind smoke
  (window pinched to 2, ten 240×64 draws) stalls the fetch 24,899 CKIO
  and still lands busy_end(model) at 1628.8 µs vs ~1628 predicted (25
  real-time steals included); (c) board FASTBOOT regression with
  governed BUSY + IRQ1 live: `+blitvram` vs `blitgold --boardtrace
  build/board_fifo_h4.txt --raw` = **0 bad pixels** (golden replaying
  the words the fetch REALLY read).  `+blitfifo` vs `+blitdump` differ
  by 24 words inside one boot exec — alternating 16-bit halves of
  longwords in four consecutive DRAWs (fields alpha/src_y/dst_y/dimy/
  tint, values like df3d = fade-alpha bytes), i.e. the game's own mov.w
  field pokes into the live list buffer landing between the EXEC-time
  backdoor snapshot and the fetch's arrival at that chunk (~140 µs at
  the real cadence).  A `+noirq1` A/B reproduces the same 24-word/same-
  value delta, so it is not IRQ1-specific: H4's game-visible timing
  (governed BUSY + IRQ1 retiming + exact cadence) shifts the game's
  poke schedule across the fetch's read point — an authentic race the
  real deep-buffered blitter has too (H3's timing happened to read
  pre-poke).  The half-word granularity itself rules out a fetch bug
  (the 32-bit capture path cannot corrupt one 16-bit half).  Rule going
  forward: when dump ≠ fifo, the FIFO log is ground truth — it is what
  silicon would consume.  **H2
  cadence off-by-one found by the anchor run**: the pace counter reset
  to 0 at BREQ made chunk spacing pace+1 (37/75 CKIO ≠ 36/74 ≈
  700/1442.5 ns) — fixed (reset to 1); H2's content-only accept could
  not see it.  Anchor methodology: fetch-bound anchors are measured in a
  bus-quiet window (the cached STATUS-poll scenario Buffi measured);
  injecting mid-boot makes spacing tenure-limited by BREQ→BACK grant
  latency — that is P-23 (M-4), not the governor (the +blitanchor probe
  prints per-run chunk-spacing stats for exactly this).  busy_end(model)
  (= engine_free at END pop, the C++-comparable number) is reported
  separately from the honest BUSY deassert, which also waits for the END
  chunk's arrival.  Known limits: EXEC while governor busy is
  warned+ignored (games poll ready first — same class as H2's queued-exec
  walker limitation); steal phase is per-EXEC until H5 rebases it on the
  real scanline; synthesis notes in-file (1-cycle comb cost cone ≥10
  cycles of slack for pipelining, comb-read 4096×32 cost queue → BRAM
  respin at the MiSTer pass).

- 2026-07-13  [claude]  **H3 DONE — draw engine pixel-exact everywhere**
  (I-1.5/6/7). New `sim/blit_draw.sv` (decoder + pixel pipe) +
  `sim/blit_vram_beh.sv` (behavioral 64 MB VRAM, flat pixel addressing,
  3 channels, per-pixel write lanes — punch-item-2 no-RMW honored) +
  trace TB `sim/tb/tb_blit.sv` / `tb_blit_main.cpp` (links the H1
  golden model, diffs the FULL 64 MB VRAM after every exec) built by
  `sim/build_blit_tb.sh`.  **Architecture** (user directive
  re-affirmed today: datapath at maximum speed, only the Buffi cost
  model stays runtime-tunable — never throttle the engine): decode-ahead
  FRONT assembles the next op from the FIFO while the BACK draws
  (10-word decode hidden), rows chain bubble-free (the ±8192 row step
  mod 2^25 gives the &0xfff src-row wrap for free), consecutive ops
  stream through the 4-deep pipe with double-banked mode state; drains
  only on real hazards via conservative rect tests (next.src — or, when
  blending, next.dst — vs prev op's dst).  Self-overlapping draws run
  strict (write committed before next read) and drop to 1-px beats when
  the sequential smear is visible inside a 4-px beat (|xshift|<4 or
  flipped) — reproducing golden's row-major feedback exactly.  Blend
  LUTs are computed, not stored: exact floor(/31) = ×2115>>16 (verified
  exhaustively); the 64-way s/d-mode switch reduces to one regular
  mulop/select form incl. MAME's dmode2 clr0.r bug and the s0d4
  full-alpha copy collapse; flat didx = dy·8192+dx reproduces the
  bitmap.pix row-underflow wrap, out-of-[0,2^25) didx lanes mask off.
  **Accept PASSED** three ways: (a) TB selftest — golden's 7 unit
  vectors, 64 smode×dmode grid ×4 alpha/tint passes, flip/trans/tint
  matrix, clip corners (dx<0 underflow wrap, x>8191 row spill, rows
  past y=4095, big clip origins), wrap-guard rejects, 37 self-overlap
  smear cases, upload corners, **1000-exec random fuzz** — all
  pixel-exact; (b) **all 8 attract traces end-to-end: 80,859 execs,
  full-VRAM diff after every one, zero mismatches**, final
  scroll-window PNGs hash-identical (build/h3_accept/); (c) board sim
  FASTBOOT Ibara 22 M insns — draw engine replaces the H2 pop=valid
  placeholder as the FIFO consumer, `+blitfifo` (real pops) still
  byte-identical to `+blitdump` (84,293 lines / 5 EXECs), new
  `+blitvram` raw dump vs `blitgold --boardtrace --raw` = **0 bad
  pixels**, NOW-LOADING frame rendered by RTL in-system.  Golden-model
  fix found by fuzz: `do_upload` past the bottom edge is raw-pointer UB
  in MAME (and was a silent heap overflow in golden.h) — now defined as
  flat wrap mod 2^25 in golden, matching what the RTL's 25-bit
  addressing does for free (games never hit it).  STATUS busy =
  fetch|draw (H4 governor takes ownership).  Native-speed sanity:
  0.8–1.7 G engine cycles per 180 s trace ≈ 5–9 % duty at 100 MHz —
  lots of headroom over the golden timeline, as the two-plane rule
  requires.  Synthesis note for the MiSTer pass: B_S3 uses in-block
  `automatic` temporaries and wide setup adders — may want a
  Quartus-friendly refactor/multi-cycling when Fmax data exists.

- 2026-07-13  [claude]  **H2 DONE — fetch unit accepted in the board sim**
  (I-4.1 + the RTL half of I-2.4), preceded by **pre-H2 punch item 1**:
  `sim/blitstudy/fifo_study.cpp` (replaces the broken `max_wait` column)
  measured the attribute-fetch depths over all 8 traces × 5 seeds and
  **refuted the "~2-chunk lookahead" estimate** — a 2–4 chunk governed
  window shifts the golden timeline ~3 ms (engine restarts at the
  700 ns/chunk refill rate after every engine-bound stretch; the
  validated open-loop formula implies the original buffers deeply, and
  the measured backlog lands at the EP1C12's 234 Kbit M4K budget).
  **Frozen: H4 governor fetch window = 512 chunks** (zero shift, zero
  lateness impact everywhere; binds only on ddpsdoj slowdown execs,
  harmlessly, capping the backlog 680→506 ch), **physical attribute
  FIFO = 640 chunks = 20,480×16 b = 40 KB** (worst need 513 ch =
  32.8 KB), **8 KB upload skid** (payload never transits the FIFO; the
  ~140 KB "parking" was a run_exec 1-thread-port artifact — precise
  rect fencing shows ibarao-only, max 2.7 KB), no separate op queue
  (decode at FIFO pop). Details: FINDINGS.md §5a, sched doc §10.5.
  **RTL**: new `sim/blit_fetch.sv` — per-chunk BREQ/BACK tenures vs the
  HS3 BSC (CPU breathes between chunks = real bus steal; grant per
  fig 10.41 with BSC's E_BRQ_PALL row-close), CL2/BL1 SDRAM mastering
  matching boot programming (MCR=0x543C, SDMR@0xFFFFE880 → mode 0x220;
  ACTV/tRCD=2/16 pipelined READs/PALL/tRP=2 per tenure, rows left
  precharged), embedded op-framing walker (END stop, UPLOAD payload
  sizing from w6/w7, upload cadence UPLD_CKIO=74 vs CHUNK_CKIO=36,
  EXEC2BRQ=10 — all provisional P-22/23/24 pending the rig's MS-6).
  Top: U1 pin mux on grant, `i_BREQ_n` wired, FIFO self-drains until
  the H3 decoder exists; `blit_regs` STATUS busy now = fetch busy
  (conservative subset of the original BUSY — the game's ready-poll
  paces EXECs authentically). **Accept PASSED**: FASTBOOT Ibara 22 M
  insns, `+blitfifo` drain log vs `+blitdump` backdoor walk =
  **byte-identical, 84,293 lines / 5 EXECs** incl. the 132 KB
  boot-upload list, zero SDRAM-model protocol errors. Known H4 nit:
  the first chunk of an upload paces at CHUNK not UPLD cadence
  (hardware can't know a header is coming; the C++ model charges UPLD
  — one-chunk difference, inside anchor tolerance).
- 2026-07-13  [claude]  **H1b DONE — board↔golden loop closed.** Added a
  `+blitdump=<file> [+blitdumpmax=N]` emitter to `tb/tb_cv1k.sv`: on each
  `blit_regs` `o_exec` rising edge it walks the op list backdoor out of
  the U1 SDRAM model (`bd_word`: bank=P[22:21], index=P[20:2],
  word=P[1]?LW[15:0]:LW[31:16] — the FASTBOOT-verified map) and writes a
  text op-word trace (hierarchical `dut.u_u1_sdram.Bank{0..3}` reads
  Verilate cleanly). `blitgold` gained `--boardtrace` to replay it.
  Booting the real Ibara ROM (FASTBOOT, ~22 M insns, no instruction
  tracer, `+irq2period=8000`) captured 8 EXECs / 208 k words that decode
  with zero errors (all op lengths correct, each list EXIT-terminated)
  and render a coherent Ibara **boot/loading** diagnostic frame (differs
  from the MAME attract trace because our sim boots from scratch — an
  authentic render of OUR pipeline, not a MAME replay). clip/scroll
  in the dump = (32,136)/(416,136), matching the H0 EXEC log exactly.
  This validates the whole H0 path (EXEC capture → LIST_ADDR → real op
  list in SDRAM) end-to-end against the H1 golden model. No RTL changes;
  `blit_regs` `o_exec/o_list_addr/o_clip/o_scroll` are the taps.
- 2026-07-13  [claude]  **H1 checker harness — golden pixel model DONE**
  (I-1.2/1.3/1.4/1.8). New `sim/blitgold/`: `vram.h` (linear 64 MB u32
  VRAM = MAME framebuffer layout; the §6.1 px2addr DDR swizzle is
  pixel-invisible and deferred to the H3 RTL backend), `golden.h`
  (faithful MAME `cv1k_v` port — colrtable/rev/add LUTs, `clr_t` ops,
  `pen_to_clr`/`tint_to_clr`, the 64-way s/d-mode `blend_combine`,
  `draw_sprite` with flip/tint/trans + src-wrap guard + flat-index dst
  guard for clip-margin underflow, UPLOAD, DRAW parse incl. the
  smode0/dmode4 blend→copy collapse, `gfx_exec` opcode loop with
  CLIP±32), `png.h` (dependency-free stored-deflate PNG + scroll/clip
  crop + FNV hash), `gold_main.cpp` (7 unit vectors + `.blit` replay).
  Provisional constants tagged in-code with P-rows (CLIP_MARGIN=32 P-13,
  alpha>>3 P-41, tint>>2 P-42, blend LUT P-40). **Validated**: unit
  vectors pass; replaying the P-stage `.blit` traces (which carry full
  op streams incl. UPLOAD payloads + per-EXEC clip/scroll) renders
  pixel-correct attract frames for ibarao (gameplay+HUD), futaribl
  (Original Ranking, heavy alpha fog), ddpdfk (ver1.5 ranking) — the
  standalone MAME cross-check, no board sim needed. Full 8192×4096 VRAM
  PNG verified valid. Golden model is timing-free (timing =
  `blitstudy/cost_model.h`); it is the pixel-exact reference for RTL H3
  and the trace-equivalence checker for H6. **Remaining H1b**: live
  board-sim backdoor `.blit` emitter (read U1 SDRAM at LIST_ADDR on
  EXEC) to render/diff our own HS3+`blit_regs` output, closing H0↔H1.
- 2026-07-13  [claude]  **H0 DONE — CPU unstuck** (I-1.1 + provisional
  IRQ2). New `sim/blit_regs.sv`: CS6 register file (BD §3), CKIO-domain
  via i_CKIO_PCEN, one-commit-per-write edge detect (area 6 = 32-bit,
  confirmed BCR2=0x39F0→A6SZ=11), combinational read drive onto the
  shared bus; STATUS ready=bit4, busy pinned 0 (no engine yet), EXEC
  latches LIST/CLIP shadows + logs. Wired into `ikacore_CV1k.sv` off the
  U13 `o_BLITTER_n` (=CS6); added provisional 60 Hz IRQ2 tick (853,333
  CKIO, `+irq2period` override) driving PTH[2]. **Disasm resolved the H0
  unknowns**: vblank=IRQ2=PTH[2] (INTC i_IRQ[2]), ICR1=0x8000⇒falling
  edge, IPRC|=0x0430⇒pri4, VBR set at 0c000008; ISR 0c00222c clears
  IRR0.2 + pulses 0x24 bit0 1→0; a 2nd handler clears IRR0.1 = IRQ1 =
  blitter-done (for H4). bit1-vs-bit4 reconciled (ready=bit4 0x10;
  boot-poll tests bit1 which is 0 in 0x10). **Accept PASSED** (FASTBOOT,
  15 M-insn run): VBLANK loop cleared, 1,694 IRQ2 ticks (frame ctr →
  0x69a), tick counter 0c0022f0 increments 1→1690, and the game issues
  **double-buffered** EXECs — lists 0c395100/0c435200 ping-pong,
  scroll/clip 32↔416 (384-px page pitch ⇒ corroborates P-37). Params:
  I-1.1 DONE; P-35 ack semantics + P-38 STATUS-bit + P-39 tick period
  added. Left open for H2+: `o_exec/list/clip/scroll` outputs (fetch
  unit), 0x24 `o_irq_ack` (H5 video). filelist.f updated.
- 2026-07-13  [claude]  H-stage build order materialized into this file
  as **Part 0b** (was only in session memory); P-stage marked DONE in
  the Part 0 snapshot; pre-H2 punch list recorded (occupancy metrics,
  RTL interface contract, optional blend=false decode). Terminology
  convention adopted project-wide: **objline** = one horizontal line of
  a blit rect in VRAM space (the K-batched unit; code:
  `g_objline_batch`, env `BLIT_OBJLINE_BATCH`); **hline** reserved for
  the scanout line; golden formula's penalty counter renamed
  `vram_tile_spans()` (32×32-px tile = 2 KiB = one DDR1 page — NOT
  sprite lines; confirmed vs MAME `calculate_vram_accesses`).
  Reviewer-facing rationale doc: `sim/blitstudy/FINDINGS.md` (what the
  study asked, DDR1 background, DDR3 command mapping, K diagram,
  double-buffering evidence via `scroll_check.cpp` — zero front-window
  writes across all 8 games, jitter-absorption bounds, NAND/YMZ port
  tenants).
- 2026-07-13  [claude]  **K = 8 re-confirmed on four more games — eight
  total, 47 M draws** (sched doc §10.4.1): deathsml / espgal2a /
  futaribl / ddpsdoj (ddpsdoj re-enabled like akatana; our Espgaluda II
  dump is set espgal2a, naming shifted like ibarao). ddpsdoj attract
  has real slowdown too (29.35 ms = 1.76× budget); espgal2a runs at
  98 % of budget (at-the-edge regression case). deathsml (only
  horizontal game; tall 1–4 px-wide 216–240-row column draws) is the
  sharpest K=4 refutation: +1.56 ms / 119 k ops > hline; K=8 holds it
  at +9.83 µs ≈ hline/6.5 — new global worst case, still 0 > hline
  everywhere. Max row staging seen = 1,288 B < 1,416 B budget.
  **Frozen params unchanged: K=8, 1 thread, ~24 KB ping-pong BRAM.**
- 2026-07-13  [claude]  **K = 8 rows confirmed across four games** (sched
  doc §10.3/§10.4): geometry + K-sweeps on ibarao / futari15 / ddpdfk /
  akatana (27 M draws; akatana re-enabled locally — GAME + #if +
  mame.lst + REGENIE). ddpdfk attract contains real slowdown frames
  (29.95 ms = 1.8× budget) — north-star-grade workload with zero play
  input. K=8 ⇒ worst lateness ≤ 3.8 µs, 0 ops > 1 hline everywhere
  (K=4 fails on ibara: +1.5 ms); staging ≈ 12 KB (24 KB ping-pong),
  1 thread. Upload model fixed (stream-during-fetch; the +111 µs boot
  UPLOAD artifact). **Frozen RTL engine params: K=8, 1 thread, ~24 KB
  staging BRAM.**
- 2026-07-13  [claude]  **P-stage first results** (sched doc §10.2): full
  pipeline ran end-to-end — CV1k-only MAME built (subset build needs the
  nine `cv1k_v_blit*.cpp` listed in SOURCES explicitly; our dump = MAME
  set **ibarao**, 2005/03/22 MASTER), 180 s attract captured (10,118
  EXECs / 10.17 M draws / worst frame 77 % of budget), C++ study built
  (`sim/blitstudy/`: cost_model.h anchors all pass; ddr3_stat.h from
  measured M-DDR3 data; engine.h two-plane DES). Cross-check
  ours/MAME delay = 0.965–0.970. **Finding: the design hinge is the
  DDR3 turnaround/batching policy, not thread count** — per-op read
  batching → 1 thread holds golden timeline (max +2.5 µs, zero > hline);
  naive per-row interleave → milliseconds late at any thread count.
  Next: K-row batching sweep → BRAM budget; denser traces (Futari).
- 2026-07-12  [claude]  **P-stage kicked off.** MAME trace hook
  implemented (`mame/src/mame/cave/cv1k_v.{cpp,h}`, +72 lines, mirrored
  as `sim/blitstudy/mame_blit_trace.patch`): one `.blit` record per
  EXEC — exact shadow-walk op stream (dispatch-rewind duplicate
  removed), CLIP/SCROLL/LIST_ADDR, machine time, frame, and MAME's
  built-in Buffi delay estimate (upstream MAME now carries Buffi's cost
  model incl. per-row 6/20+11/12 CLK constants and hline steal — free
  cross-validation for our port). `sim/blitstudy/` scaffolded:
  `blit_trace.h` reader with strict op-walker, `trace_dump` validator
  (compiles clean), runbook in README. Blocked only on
  `libsdl2-dev`/`libsdl2-ttf-dev` (user install) before the CV1k-only
  subset build (`make SOURCES=src/mame/cave/cv1k.cpp`). Confirmed in
  MAME source: DRAW = 10 words, UPLOAD = 8 + w×h, CLIP = 2; blitter regs
  0x04 EXEC / 0x08 LIST / 0x14,18 SCROLL / 0x40,44 CLIP; ready bit =
  0x10 (bit 4). cv1k_v license BSD-3-Clause → blend math reusable in
  our golden model.
- 2026-07-12  [claude]  Prototype architecture refined (user):
  **two-plane design** — golden-model timing governor generates the
  CPU-visible timeline open-loop (unconditional DMA fetch on attribute-
  FIFO drain / UPLOAD; BREQ/BUSY/IRQ1 = f(op list) only), while N
  **blitter threads** execute opportunistically against DDR3 (early →
  NOP-idle; jitter-delayed → next op goes to another thread). And
  **model-first**: a C++ performance study (DDR3-stat + cost-model port
  + threaded discrete-event engine, real harvested EXEC lists + stress
  workloads) freezes N_threads / FIFO depths / hazard rule / stability
  verdict BEFORE RTL; the same C++ engine becomes the RTL
  trace-equivalence checker. Spec: `blitter_ddr3_sched.md` **§10**.
  Build order now H0 → H1 → **P (study)** → H2..H5 → H6 conformance.
- 2026-07-12  [claude]  **Hardware-prototype pivot** (user decision,
  supersedes the same-day pure-C++ Phase-1 order): the prototype is a
  synthesizable RTL blitter that (a) really bus-masters op lists over
  BREQ/BACK (HS3 bsc.sv already implements the grant path), (b) really
  scans out 240p video (I-3.1/2/3 unblocked with provisional parameters
  per BD §7.6 — hline steal becomes *real arbitration*, not a counter),
  and (c) is paced by a **timing governor** implementing the cost-model
  formula (`op_start = max(engine_free, fetch_ready)` + runtime-loadable
  latency tables, I-2.1..2.5) regardless of native datapath speed. The
  C++ collateral (golden pixel model, VRAM dumper/PNG+hash, statistical
  DDR3 mimic, blit_cost_model.py) is demoted to the verification
  harness: per-frame pixel-exact diff + anchor assertions. Build order
  H0–H6 in memory/blitter-phase1-plan: H0 CS6+IRQ2 tick → H1 C++ checker
  harness → H2 BREQ fetch unit → H3 decoder+draw engine (native speed) →
  H4 timing governor (anchors 93/189/12090) → H5 video scanout (IRQ2
  moves tick→vsync; steal appears naturally, 240×64 ≈163.9 µs) → H6
  DDR3-stat backend swap + governor-invariance jitter sweeps. I-2.6
  stays hardware-blocked. Effort: ~4–6 sessions vs ~2, nothing discarded.
- 2026-07-12  [claude]  Rig revised to **ROM-simulator pair** (manual §2a
  Rev 3, user decision): FPGA bitstreams are versioned and live *inside*
  U4, so version campaigns need U4 swaps → both U4 NOR and U2 NAND are now
  served by RP2350B ROM simulators. Zero permanent board modification (the
  hook becomes a served image variant, never flashed); mailbox returns to
  CS0 with a stream window (RF-13 WE#-continuity check gates the variant);
  M-13 firmware A/D diffs become batch {boot × bitstream} matrices with
  bitstream bisection. Feasibility rests on the verified CS0 profile: only
  the copy loop reads flash; the bitstream uploads from the SDRAM copy.
  Passive/acceptance sessions (MC-1/6/8/9, M-10) stay all-real silicon.
- 2026-07-12  [claude]  Injection rig architecture decided (operator manual
  §2a, Rev 2): the RP2350B cannot emulate the U4 NOR (pin budget + async
  random-access timing), so the Pico emulates **U2 NAND** behind the U13
  CPLD and code injection rides a **one-time U4 hook** reflash — the stock
  reset stub (BSC init, copy loop, cache flush) runs bit-identical, then
  the hook loads bench payloads from the emulated NAND (stock game boots
  unmodified when the real NAND is fitted). Mailbox/MARKQ moved CS0 → CS4;
  Pico adds CKIO-referenced PICO-TS timestamps, LA trigger-out, reset +
  watchdog → unattended kernel batches. MS-6 promoted to ★★★/P2: the
  fetch-unit discrimination family (32-bit fetch = 2 instructions vs
  per-instruction cache access; unified-cache port contention) — manual
  §5.5. No new flex taps (manual §10).
- 2026-07-12  [claude]  Board-measurement operator manual created
  (`docs/opus_measurement_manual.md`, for the Claude Opus sessions that will
  run the rig). It owns the new CPU-side queues MC-1..9 / MS-0..8 and the
  H-01..15 HS3 registry — key targets per user: SH-3 SDRAM access timing
  under blitter BREQ and SH-3 pipeline CPI (the danmaku-slowdown terms).
  Pointer added after Part III. M-1..13 mapping to labs/benches/decoders is
  tabulated in that manual §4.1; definitions stay here.
- 2026-07-12  [claude]  Direction agreed with user: rapid prototyping first —
  Phase 1 golden model built from Buffi's research + MAME math starts next
  session, no hardware prerequisite (flex PCB proceeds in parallel; captures
  later refine flagged P-rows without restructuring, per BD §7.6 decoupling).
  I-1.8 rescoped to FULL-VRAM dump (8192×4096) + visible-window crop, Depends
  relaxed I-1.5→I-1.3 so capture lands before the draw engine. Agreed build
  order: I-1.1 (+provisional 60 Hz IRQ2 tick) → I-1.3 → I-1.8 → I-1.2/I-1.4
  (C++ golden model fed by TB backdoor list reads before BREQ RTL exists) →
  I-1.5/6/7. Part 0 snapshot refreshed (U2 NAND + U13 CPLD in sim, DDR1 model
  present, boot parks in VBLANK loop).
- 2026-07-06  [claude]  File created. Seeded from blitter_detail.md state:
  board sim (HS3 + U4 + U1) DONE, all blitter items TODO, P-01..42 registered,
  M-1..13 queued. No measurements taken yet.
