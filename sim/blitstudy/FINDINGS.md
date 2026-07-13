# P-stage findings — what we asked, and what the answers force on the RTL

Status 2026-07-13. Full derivations live in `docs/blitter_ddr3_sched.md`
§10–§10.4.1; this file is the distilled rationale so the RTL sessions
don't have to re-derive why the frozen numbers are what they are.

## 1. The problem this study exists to solve

The blitter has two contradictory obligations:

* **Toward the CPU it must be a 2005 EP1C12 over DDR1.** Bus-request
  cadence, BUSY duration, IRQ timing — all of it must match the original
  formula, because danmaku slowdown *is* that timing. This is the
  **timing plane**: the cost-model governor generates the CPU-visible
  timeline open-loop (`op_start = max(engine_free, fetch_ready)`), never
  waiting on memory. Attribute fetch happens unconditionally via DMA at
  the governed cadence.
* **Toward memory it runs on a 2015 shared DDR3 port** (153.6 MHz,
  64-bit, shared with HPS/scanout) whose latency is *statistical*:
  93 % of accesses land in 17–24 clk, but there is a measured tail to
  165 clk under HPS load. This is the **execution plane**: worker
  engine(s) that do the actual pixel work opportunistically — allowed
  to run *ahead* of the golden timeline (that headroom absorbs jitter),
  never allowed to start before an op's attributes have "arrived".

The governed timeline cannot slip by construction. So the single
quantity the whole study measures is per-op

```
lateness = ddr3_finish − golden_finish
```

and the design requirement is: **lateness stays ≪ 1 hline (63.586 µs)
for every op of every real workload, at tail-latency jitter.**

A note on what that bar actually protects, because it is stricter than
the visibility deadline. CV1k games are **double-buffered**: SCROLL
ping-pongs between two 320×240 windows on the surface (x=32 ↔ x=416 in
all eight traces), the blitter draws the *next* frame into the window
not being scanned, and the flip is just a SCROLL move at vblank —
delayed by the CPU when STATUS is still busy, which is precisely the
slowdown mechanism. This is not assumed but measured:
`scroll_check.cpp` classifies every clip-clamped dst pixel of all 47 M
ops against the live SCROLL window — **zero pixels and zero EXECs
touch the displayed window in any of the eight games** (66–91 % of dst
pixels go to the alternate window, the rest to off-window staging
during full-surface CLIP periods and uploads). So a late op is *not*
scanned out mid-frame; the true visibility deadline is the next flip.

We keep the ≪ 1 hline bar anyway, for three reasons. (1) It closes the
BUSY-integrity gap: during slowdown the CPU polls STATUS and flips at
the first vblank after ready — the governor clears BUSY at *golden*
time, so the real DDR3 work must be behind it by much less than the
ready-to-flip distance, which can be short; µs-scale lateness makes it
unconditionally safe. (2) It keeps EXECs decoupled: the study models
each EXEC's port timeline independently, which is only valid if
lateness cannot accumulate across EXEC boundaries — a µs-scale bound
guarantees convergence to an idle port between EXECs. (3) Draws may
*read* the front window (previous frame) as blend source; cross-EXEC
RAW ordering is trivially satisfied when the previous EXEC's writes
landed µs after their golden times. Since K=8 meets the strict bar
anyway (§6), we get all three properties for free rather than having
to bound flip distances and cross-frame dependencies precisely.

## 2. The questions we needed answered before writing any Verilog

1. **Is the architecture feasible at all** — can a DDR3 port shared
   with scanout/HPS hold a DDR1-era timeline through real 180 s
   workloads, including over-budget slowdown frames?
2. **What is the design hinge?** Candidate knobs: number of engine
   threads, FIFO/queue depths, hazard policy, memory-access batching.
   Which one actually moves the lateness distribution?
3. **How big must on-chip staging be** (the only BRAM commitment the
   fetch/draw datapath forces)?
4. **What lookahead does the attribute fetch path need** (FIFO depth in
   ops/chunks)?
5. **Do we own workloads that genuinely stress this** — real slowdown,
   not synthetic worst cases?

Method: trace-driven, CPU-decoupled. A +75-line MAME hook records every
EXEC's exact op-word stream (`.blit` traces); a C++ port of the cost
model (validated bit-level against all five Buffi anchors, and 0.95–0.97
against MAME's own delay estimate on 60 k+ EXECs) gives the golden
timeline; a statistical DDR3 port model (parameters measured on the real
DE10-Nano with `MiSTerDDR3Test-CV1k`, including the HPS-load tail) gives
the execution plane; `engine.h` is the discrete-event model of the
worker side. 8 games, 47 M draws, 2005–2012, both board generations.

## 3. Background for reviewers — the original DDR1 blitter, and what the model's transactions mean

Readers auditing the source need three pieces of context we otherwise
carry in our heads: what the 2005 hardware actually is, how it draws,
and what a `port.read()` / `port.write()` call in this code claims to
represent on the DDR3 side. (Primary sources: Buffi's CV1000 blitter
research and MAME's `cv1k_v.cpp` device, which now carries his measured
cost model; our board docs `docs/blitter_*.md`.)

### 3.1 The original machine

The blitter is U8, an Altera Cyclone **EP1C12** (bitstream uploaded by
the CPU at boot out of the U4 program image). Its private VRAM is
**2× MT46V16M16 DDR1** — a 32-bit bus at 76.8 MHz ("VCLK" in the cost
model), 64 MiB total, organized as one flat **8192×4096 ARGB1555
surface**. *Everything* lives on that one surface: the two 320×240
frame windows the CPU ping-pongs between (selected by SCROLL), and all
sprite/tile source graphics, which the CPU must first UPLOAD into
spare regions. There is no separate texture memory — a DRAW is a
VRAM→VRAM rectangle operation.

The CPU never touches VRAM directly. It builds a display list in its
own SDRAM, writes the list address to LIST_ADDR (CS6 +0x08) and kicks
EXEC (+0x04), then polls STATUS (+0x10). On EXEC the blitter becomes
**SH-3 bus master** (BREQ/BACK) and DMA-fetches the list in **64-byte
chunks** — the chunk cadence is fixed by the bus protocol
(≈700 ns/chunk for attributes, ≈1442.5 ns/chunk for upload payload)
and is *independent of drawing progress*. The op stream: CLIP
(2 words), UPLOAD (8 header words + dimx·dimy pixel words), DRAW
(10 words), END. One 64 B chunk holds ≈3 DRAW ops.

### 3.2 How a DRAW executes, and its measured cost

A DRAW decodes to: attribute flags (h/v flip, blend enable + blend-mode
pair, tint), src x/y, dst x/y, dimensions (to 13/12-bit fields), and
two alpha values. The hardware clips it against the CLIP window plus a
fixed 32 px margin; a fully-clipped-out draw costs only its 20-byte
list fetch. A surviving draw is executed **row by row**: read the
source row; when blending, also read the destination row; combine
(tint multiply, alpha blend, bit-15 transparency); write the
destination row back.

Buffi measured the timing on real boards, and the formula (per draw,
in 76.8 MHz VCLK) is

```
cost = src_px/4 + dst_px/2 + spans_src·6 + spans_dst·(20+11) + 12
```

The streaming terms are 4 source px/clk and 2 destination px/clk (dst
extended to 4-px alignment), plus 12 clk setup. `spans_*` count the
**32×32-pixel VRAM tiles the rectangle touches** — *not* sprite lines
(MAME: "VRAM data is laid out in 32x32 pixel rows",
`calculate_vram_accesses()` in `cv1k_v.cpp`; `vram_rows()` in
`blit_cost_model.py`, renamed `vram_tile_spans()` in our C++ port to
break the collision). A 32×32 tile at 2 B/px
is 2 KiB — exactly one DDR1 page across the chip pair — so the
original VRAM is *page-tiled* and the penalty terms count page
activations directly: ~6 clk per source page touched, ~31 clk per
destination page (its read-modify-write). Beware the name collision
when auditing: these tile spans are unrelated to the sprite rows that
K batches. Two constant sets exist — P_PDF {5,20,10,10} reproduces
Buffi's PDF anchors exactly, P_MAME {6,20,11,12} is what MAME ships;
the trace cross-check uses P_MAME. Anchors reproduced by our port:
8×8 = 93, 16×12 = 189, 240×64 = 12,090 VCLK. Scanout shares the
same DDR1 bus and steals ~166 VCLK per hline (63.586 µs). This formula
*is* the golden timeline — `cost_model.h` is its direct port, checked
bit-level on the anchors and to 0.95–0.97 against MAME's own delay
estimate over the ~65 k draw-only EXECs of the eight captured traces.

### 3.3 How ops map onto the DDR3 command pipeline in this model

The replacement memory is the DE10-Nano's FPGA→HPS SDRAM bridge:
64-bit port at 153.6 MHz into the HPS DDR3 controller, **shared with
Linux and with our scanout**. Every constant in `ddr3_stat.h` is
fitted from hardware measurements (`benchmarks/MiSTerDDR3Test-CV1k`),
not from datasheets.

* **Address map**: linear, `addr = (Y·8192 + X)·2`. A sprite row at
  (x, w) covers the contiguous 8-byte words returned by
  `words_linear()`; rows are *not* contiguous with each other, so each
  row is its own address segment → its own burst command.
* **A read train** (`Port::read(t, words, nbursts)`) models the
  measured PIPE behavior of the controller (test M-DDR3-1): commands
  are FIFO'd ahead of data, so a train of independent reads exposes
  **one** latency, not one per burst. Charge = one sampled latency L
  (histogram under HPS load: 93.4 % in 17–24 clk, tail to 165 clk)
  + `words·β_R` (β_R = 1.016 clk/word, measured stream efficiency)
  + `nbursts·G_CMD` (1 clk command slot per burst, ≤128 words each).
* **A write train** is posted — no exposed latency:
  `words·β_W + nbursts·C_W`.
* **Direction turnaround**: 14 clk whenever the port flips R↔W
  (measured). This is the quantity K amortizes.
* **Engine mapping** (`engine.h`): one K-row batch of a blended draw
  becomes exactly *one* read train (`words = K·(src_row + dst_row)`,
  `nbursts = 2K`) followed by *one* write train (`K·dst_row` words,
  K bursts) — so a batch pair costs one R→W→R turnaround pair, one
  exposed read latency, and stream-rate data. An UPLOAD becomes a
  single posted write stream whose start is tied to payload arrival
  over the (10× slower) CPU-bus fetch and whose completion cannot
  precede the last fetched byte.
* **Scanout** occupies the port for an 80-word read at the top of every
  hline; engine requests landing inside that window are deferred past
  it.
* **Deliberately not modeled**: DDR3 bank/row state and refresh (both
  are already inside the measured L/β/T_TURN distributions), and
  fine-grained scanout interleave (start-defer only). Pixel *compute*
  is also not charged: at 153.6 MHz against BRAM-staged rows, the
  original's 4 px/clk rate is trivially exceeded, so the engine is
  memory-bound by construction and the model charges memory time only.

## 4. The meaning of K — the answer to "what is the hinge"

**K is the objline-batching granularity of the draw engine**: the
engine stages K objlines' worth of data (source line + destination
line for blending) in on-chip BRAM, issues them to DDR3 as *one* read
train, draws, then writes the K destination lines back as one write
train.

**Terminology** (project convention, matching the tileline/objline
naming used in our other cores): an **objline** is one horizontal line
of a blit rectangle in VRAM space. Where this document or the sweep
tables say "row", read objline — the code now says `objline`
(`g_objline_batch`, env `BLIT_OBJLINE_BATCH`; legacy `BLIT_ROW_BATCH`
still accepted). Three near-collisions to keep apart: an objline is
not a scanout **hline** (the 63.586 µs display line — "hline" is
reserved for that throughout), not a DRAM row/page, and not the golden
formula's 32×32 **tile span**. DRAM page behavior is present only
implicitly: on the DDR1 side it is counted by the golden formula's
32×32-tile spans — the DDR1-page terms of §3.2, distinct from the
sprite rows K batches; on the DDR3 side it is folded into the measured
latency histogram / β / T_TURN parameters. The model does assume a
**linear VRAM mapping in DDR3** — each sprite row is one contiguous,
burst-able segment (`words_linear()`); the RTL must keep that layout
for these results to transfer.)

Axes, for the orientation-wary: a row runs along VRAM **X** — the
direction of contiguous addresses and of scanout lines. Screen
orientation never enters:

```
VRAM (linear, addr = (Y·8192 + X)·2)
      X →                                     a "row" = dimx px along X
  Y ┌────────────────────────────────┐        = contiguous DDR3 bytes
  ↓ │ ┌───────┐┌───────┐             │
    │ │ win A ││ win B │  sprite/    │        scanout reads 320-px lines
    │ │320×240││320×240│  tile art…  │        along X (80 × 64-bit words)
    │ └───────┘└───────┘             │
    └────────────────────────────────┘
```

Vertical (tate) games rotate the **monitor**, not the machine: VRAM,
the op stream, and scanout are identical; the player's vertical axis is
simply VRAM X, and sprite art is authored pre-rotated. Consequence,
visible in our data: in tate games a player-vertical bullet stream lies
along VRAM X (wide, few rows — batching-friendly), while the one
horizontal (ROT0) game in the corpus, deathsml, is exactly where
player-vertical columns become many-rows-of-few-pixels — the measured
K stressor (§4, 1×240 / 4×216 draws).

Why this is the hinge and threads are not: the DDR3 port charges a
turnaround penalty (~14 clk + a fresh ~17–24 clk latency) every time
the access direction flips R→W or W→R. A blit is intrinsically
read-modify-write, so *some* flipping is unavoidable — K decides how
often:

* **K=1 (naive per-row interleave)**: read src row, read dst row, write
  dst row, repeat. Two turnaround pairs *per row*. For danmaku sprites
  (p50 width 16–24 px, i.e. a handful of DDR3 words per row) the port
  spends more time turning around than transferring. Result: ~7 ms of
  lateness on Ibara **at any thread count** — the port itself is
  saturated, so adding threads adds nothing. This measurement is what
  demoted "N_threads" from presumed-hinge to non-issue.
* **K=∞ (whole-op batching)**: one turnaround pair per op — but staging
  must hold the whole sprite: up to 264–307 KB per op (p99.9 across the
  corpus). Not a BRAM budget we want, and unnecessary.
* **The knee is at K=8.** K=4 already fails two games; K=8 is
  empirically indistinguishable from unbounded staging on all eight.

The knee has a physical reading. The golden cost is mostly
*pixel*-rate (px/4 + px/2) plus per-2-KiB-page penalties (§3.2), so
for a narrow sprite the golden model charges only a few VCLK per row —
while the DDR3 side pays a fixed ~(L + 2·T_TURN + cmd slots) per batch
no matter how few words the batch moves. At K=8 that fixed cost,
spread over 8 rows, sits safely below the golden rate for essentially
all real geometry (and only marginally above it for degenerate
few-px-wide columns — see §4.1); at K=4 it re-crosses the line for
tall narrow sprites broadly, and the engine falls behind faster than
inter-op slack can repay.

**The stressor geometry is tall-and-narrow, and we have it in-corpus:**
Deathsmiles (the one horizontal/ROT0 game) draws 1×240 and 4×216
column strips. At K=4 a 216-row draw pays 54 turnaround pairs →
+1.56 ms late, 119,308 ops > 1 hline. At K=8 the same trace's worst op
is +9.83 µs ≈ hline/6.5 with zero ops over. That single game is the
sharpest proof that K=8 is *necessary*; the other seven (including both
over-budget workloads) prove it *sufficient*.

What one op looks like on the port at K=8 — a blended DRAW with
dimy = 20 → ⌈20/8⌉ = 3 batches (8+8+4 rows):

```
port ─[READ train b0]─T─[WRITE train b0]─T─[READ b1]─T─[WRITE b1]─T─[READ b2·4rows]─T─[WRITE b2]→
       │                  │
       │ L + words·β_R    │ words·β_W + 8·C_W      (posted — no latency)
       │   + 16·G_CMD     │ burst order: w0 w1 … w7
       │ burst order:     │
       │ s0 d0 s1 d1 … s7 d7                        T = 14-clk R↔W turnaround
       │ (each row = its own burst command —
       │  rows are not address-contiguous;
       │  one exposed latency L for the whole
       │  train: the cmd FIFO pipelines the rest)
```

Turnaround/latency *density* is the entire design question:

```
K=1: [L·s0·d0]T[w0]T [L·s1·d1]T[w1]T …   → 1 latency + 2 turnarounds per ROW
K=8: [L·s0·d0 … s7·d7]T[w0 … w7]T …      → 1 latency + 2 turnarounds per 8 ROWS
```

(In RTL the ~24 KB staging is ping-ponged so the blend datapath is
never on the port's critical path: batch N's compute overlaps the tail
of its own read train and pipelines into its write train, and the port
can proceed straight to the next train the moment the current one ends
— which is what lets the model charge memory time only, §3.3. The port
itself stays strictly serial, exactly as drawn.)

### 4.1 How DDR3 jitter is actually absorbed — run-ahead, and its two bounds

The governor fetches attributes at the protocol cadence *regardless* of
DDR3 state, and the engine only ever sees ops that have physically
arrived. So the absorption mechanism is: the engine drains the
attribute FIFO faster than the golden cadence, banks run-ahead
headroom, and a latency-tail event (≤ 1.07 µs) or scanout window
(~0.7 µs) merely spends some of it. The sweeps show the bank is real —
the p99 of lateness is *negative* (−10.8 µs deathsml, −30.3 µs
ddpsdoj): 99 % of ops finish tens of µs before their golden time.

This works iff two conditions hold:

1. **Sustained throughput margin** — DDR3 time must run below golden
   time over any window of the op mix. At K=8 nearly every geometry
   has per-op surplus (an 8×8 draw: ~0.8 µs DDR3 vs 1.21 µs golden).
   The exception is degenerate few-px-wide columns: their golden cost
   is nearly all pixel-rate while DDR3 still pays per-batch
   latency+turnaround, so a 1×240 column carries a µs-scale
   *intrinsic* deficit even at K=8 — and that is exactly the measured
   +9.83 µs worst case (deathsml, op #2, before any headroom is
   banked). It stays absorbed because such ops are rare, the deficit
   is bounded, and surplus neighbors regenerate the slack. K=4 breaks
   the condition wholesale: on deathsml's column-heavy mix each op is
   intrinsically slower than golden, "draw the next one faster" has no
   faster next op to offer, and the deficit compounds to +1.56 ms.
   Jitter absorption and the K knee are one requirement seen from two
   sides.
2. **The run-ahead cap** — the engine cannot start ahead of
   `fetch_ready` (RTL: FIFO-empty), so headroom is bounded by the
   governed fetch lookahead plus one staged batch. Visible in the data:
   worst-lateness ops cluster at EXEC start (deathsml's +9.83 µs worst
   is op #2, futaribl's op #0) where no headroom has accumulated yet
   and a first-op tail lands undamped — still only µs, because a
   single op's excess is itself bounded.

Two RTL corollaries: the attribute FIFO must never overflow (the
open-loop governor cannot be backpressured) — its required depth was
**measured, not hand-bounded** (`fifo_study`, §5a below; the "~2 µs
stall ≈ 3 chunks" estimate that stood here originally was wrong by two
orders of magnitude). And uploads are the one op class with no "draw
faster" available — the engine cannot out-run payload arriving over
the 10× slower CPU bus — which is exactly why the streaming-upload
rule in §5 exists.

## 5. What K=8 costs, and the rest of the frozen contract

* **Staging BRAM = K × worst row (src+dst)**. Measured worst row across
  47 M draws: 1,288 B (320-px-wide sprite, src+dst, 2 B/px) — the
  1,416 B/row budget (8192-px wrap-safe) covers it. So one buffer is
  K × 1,416 ≈ 12 KB, **~24 KB as ping-pong** so batch N+1's read train
  overlaps batch N's draw/write-back.
* **One engine thread.** With per-op/K=8 batching a single thread holds
  the golden timeline everywhere; the multi-thread machinery (rect
  hazard scoreboard across threads, per-thread staging) is not needed
  in RTL. The hazard rule survives only as a *correctness* interlock
  (overlapping-rect RAW/WAW ordering), not a performance feature.
* **Attribute fetch: protocol-fixed 64 B chunks ≈ 3 DRAW ops per
  chunk**, governed cadence T_CHUNK ≈ 700 ns. Depth: see §5a — the
  original "~2 chunks suffices" cadence estimate did not survive
  measurement. The governor's *unconditional* fetch (never gated on
  engine/DDR3 state) is what guarantees `fetch_ready` marches on
  schedule.
* **Uploads must stream, not store-and-forward.** The CPU-side fetch of
  upload payload (SDRAM, ~1442.5 ns/64 B) is ~10× slower than the DDR3
  write. v1 of the engine issued the whole VRAM write after the last
  fetched byte and produced a +111 µs artifact on the boot-time 256×256
  UPLOAD in every game that has one. The RTL upload path must write
  chunks to VRAM as they arrive; completion = last chunk. With that,
  uploads are never the critical path.

**Frozen RTL engine parameters: K = 8 rows, 1 thread, ~24 KB ping-pong
staging, attribute FIFO per §5a, streaming uploads.**

### 5a. Attribute-FIFO sizing — measured, and the 2-chunk myth (fifo_study, 2026-07-13)

The pre-H2 punch item "occupancy metrics over the traces" (`fifo_study.cpp`,
replacing the broken `max_wait` column) produced a qualitatively different
answer than the cadence math above, and two of this document's original
claims are hereby corrected:

1. **A 2–4 chunk fetch lookahead is not viable.** Simulating the governed
   fetch (chunk c waits for golden consumption of chunk c−D, the virtual
   original-FIFO model) at D = 2–4 shifts the golden timeline by ~3 ms and
   puts millions of ops over an hline: after any engine-bound stretch a
   small-lookahead fetch has long stalled, and the engine then starves at
   the 700 ns/chunk refill rate. Since the open-loop formula is validated
   on real hardware, the real EP1C12 must buffer *deeply* — the measured
   backlog (below) lands right at its 234 Kbit M4K budget, which is
   corroborating, not coincidental.
2. **The FIFO is tens of KB, not hundreds of bytes.** The surviving-draw
   attribute backlog (fetch at protocol cadence, consumption at golden op
   starts) peaks per game at 334–690 chunks; the global worst is
   **ddpsdoj's 29.1 ms slowdown exec: 690 chunks = 44.2 KB** outstanding.

Measured over all 8 traces × 5 jitter seeds (K=8, 1 thread):

| quantity | worst case | frozen RTL value |
|---|---|---|
| governed-fetch window D (zero timeline shift, zero lateness impact) | 512 chunks OK everywhere; 256 fails (992 µs shift) | **D = 512 chunks** (H4 governor; drains decode-discard: clip/clipped ops leave at arrival) |
| physical attribute-FIFO occupancy at D=512 | 513 chunks / 32.8 KB (ddpsdoj capped by the window; 506–518 elsewhere) | **640 chunks = 20,480 words = 40 KB** (window + engine-lag slack) |
| decoded-op backlog (if a separate op queue were used) | 2,030 ops | none — decode at FIFO pop |
| upload-fence park (payload arriving while a prior overlapping draw runs) | ibarao only: 10k fenced / 43k uploads, max **2.7 KB** parked (4.3 ms) | 8 KB skid buffer on the upload write path |

Notes: (a) the D=512 window binds only on ddpsdoj slowdown execs (3,059
chunks over 180 s) and binds *harmlessly* — zero shift, baseline-identical
lateness, and it caps the physical backlog; (b) upload payload never
transits the attribute FIFO (it streams down the write path), so payload
size does not enter FIFO sizing; (c) the real fetch pacing of the original
is a rig P-row (MS-6 K-family) — until then the RTL fetches open-loop at
protocol cadence with the D=512 window as the deterministic bound, which
reproduces the validated model timing exactly on the whole corpus.

### 5.1 The other tenants on the DDR3 port — NAND and YMZ770

In the full core the same DDR3 also backs the U2 NAND image (1 Gbit +
spare ≈ 132 MB) and the YMZ770's sample ROMs (U23/U24, 8 MB); with
64 MB VRAM that is ~204 MB of address space — no contention there. The
bandwidth question reduces to two more port claimants:

* **NAND**: the chip's own page architecture batches for us — only the
  2,112 B page-register fill touches DDR3 (264 words ≈ one ~2.1 µs read
  train); the byte-wise drain is throttled by the SH-3's 8-bit CS4 bus
  (~8–9 MB/s ceiling), so worst-case streaming is one ~2.1 µs event per
  ~240 µs (<1 % occupancy), and gameplay scenes see ~zero NAND traffic
  at all (assets already resident in SDRAM/VRAM).
* **YMZ770**: continuous (music plays through slowdown) but tiny —
  ~0.4 MB/s of compressed AMM across all channels. With a 512 B
  per-channel prefetch FIFO the port sees a 64-word (~0.7 µs) read per
  ~1.4 ms: one-twentieth of scanout's own interference rate.

Scheduling rule: both ride at low priority, transactions capped at one
burst train, never preempting a blitter train mid-burst. Under that
rule each injection is a bounded sub-couple-µs occupancy —
structurally the same disturbance class as a latency tail or a scanout
window, which §4.1 shows the engine absorbing with ~6× margin. Their
combined effect is statistically indistinguishable from the HPS-load
tail already baked into the calibrated latency histogram (measured
under deliberately hostile HPS traffic the MiSTer steady state won't
reach), so they do not move the frozen parameters.

## 6. Feasibility verdict, and the workloads that certify it

With the frozen parameters, worst-case lateness over the entire corpus
is **+9.83 µs, and zero ops exceed one hline** (5 jitter seeds per
trace, HPS-load latency tails on). The corpus is not soft:

* **ddpdfk** and **ddpsdoj** attract demos contain *real* slowdown —
  worst frames 29.95 ms and 29.35 ms ≈ 1.8× the 16.68 ms budget, i.e.
  the exact regime the whole project exists to reproduce — captured
  with zero play input (they are permanent, repeatable regression
  workloads).
* **espgal2a** runs at 98 % of frame budget — the at-the-edge case
  where a timing regression flips frames over budget first.
* **deathsml** supplies the adversarial geometry (§4).

## 7. What this code becomes now

The study is not throwaway: `engine.h` + `cost_model.h` + `ddr3_stat.h`
are the **RTL trace-equivalence checker** for build stage H2+ — same
`.blit` workloads, same jitter seeds, op-for-op comparison against the
Verilog engine. The governor/cost model additionally ships *in* the FPGA
(runtime-loadable latency tables), so board-trace feedback later adjusts
tables, not architecture.

Known model simplifications (kept deliberately conservative; revisit
only if RTL margins get tight): `blend=true` forced on every draw (dst
always read — overstates read traffic), and scanout modeled as a
start-defer rather than fine-grained interleave. The `max_wait` column
in the sweep output is a broken metric — ignore it; `fifo_study` (§5a)
is its replacement.
