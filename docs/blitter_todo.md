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
| Board-level sim (SH-3 HS3 core + U4 NOR + U1 SDRAM, shared bus, Verilator) | **DONE** — boots Ibara U4, executes from flash, CKIO/BSC verified vs SH7709S manual |
| Blitter RTL | **not started** — CS6 decoded by nothing yet |
| VRAM (MT46V16M16 DDR) model in sim | **not present** (datasheet in docs/) |
| Golden pixel model (MAME port) | **not started** |
| PCB measurements | **none taken** |

---

## Part I — Implementation tracker

Order = agreed build order (function first → timing layer → video → board).
`Accept:` names the pass condition.

### Phase 1 — functional blitter (no timing)

| ID | Item | Spec | Depends | Status | Accept |
|---|---|---|---|---|---|
| I-1.1 | Register file @ CS6 0x18000000–57 (EXEC/LIST_ADDR/STATUS/SCROLL/CLIP/DSW, shadow-latch at EXEC) | [BD §3] | — | TODO | TB register r/w test; EXEC latching semantics |
| I-1.2 | Op-list decoder (DRAW/UPLOAD/CLIP/EXIT, bit-exact fields, BE word order) | [BD §4] | I-1.1 | TODO | decode-compare vs reference parser on MAME `wpset` dumps |
| I-1.3 | Behavioral VRAM model (64 MB, px2addr map; functional, no DDR timing) | [BD §6.1] | — | TODO | address-map unit test (bank/row/col vectors incl. X%4≠0) |
| I-1.4 | Golden pixel model: port MAME `cv1k_v` blend/draw to TB-linkable C++ | [BD §7.4] | — | TODO | self-check vs MAME screenshots (3 games) |
| I-1.5 | Draw engine datapath: src read → tint → blend ALU (4 px/clk) → trans → dst write; flipX/Y; signed dst coords | [BD §7.1, §7.4] | I-1.2/3/4 | TODO | **frame capture of 320×240 @ SCROLL vs golden model, pixel-exact**, full game op lists |
| I-1.6 | CLIP op + window±32 margin, exact 4-px-grid clipping at decode | [BD §8] | I-1.5 | TODO | clipped-list capture vs golden model (PinkSweets waves list) |
| I-1.7 | UPLOAD op write path | [BD §4.2] | I-1.3 | TODO | uploaded tiles readable by subsequent draws, pixel-exact |
| I-1.8 | Frame-capture "virtual scanout" (dump visible window to PNG/PPM at EXIT) | — | I-1.5 | TODO | images produced per frame; used by all Accepts above |

### Phase 2 — cycle-accurate timing layer

| ID | Item | Spec | Depends | Status | Accept |
|---|---|---|---|---|---|
| I-2.1 | Event/latency-table stall unit (timing decoupled from function) | [BD §7.6] | I-1.5 | TODO | latencies runtime-loadable; zero pixel diffs vs Phase 1 |
| I-2.2 | Draw cost scoreboard (golden `draw_cost_vclk`) as TB assertion | [BD §6.5] | I-2.1 | TODO | anchors: 8×8=93 CLK, 16×12=189, 240×64=12 090 (±P-fit) |
| I-2.3 | hline-steal cadence: free-running line counter (4884 VCLK), 2.16 µs stall | [BD §9.2] | I-2.1 | TODO | 240×64 total ≈163.9 µs incl. 3 steals |
| I-2.4 | Op-fetch cadence model (chunk FIFO, T_CHUNK_IDLE/UPLD) | [BD §5] | I-1.2 | TODO | 80 clipped draws ≈17.5 µs; 256×5 upload ≈58 µs |
| I-2.5 | BUSY/READY + IRQ1 retirement timing | [BD §3, §10] | I-2.4 | TODO | ready-poll loop in TB sees PCB-plausible busy windows |
| I-2.6 | DDR command-stream generator (ACT/RD/WR/PRE/(AREF)) behind the same scheduler — LA-comparable output | [BD §6.2–6.4] | I-2.2 | BLOCKED(M-6,M-10) | RTL cmd-gap trace diffs clean vs PCB capture |

### Phase 3 — real video

| ID | Item | Spec | Depends | Status | Accept |
|---|---|---|---|---|---|
| I-3.1 | Sync generator: hcnt 0..4883 @76.8, ÷12 pixel CE, 262 lines, provisional porches | [BD §9.1] | I-2.3 | BLOCKED(M-1,M-2) | 60.0184 Hz frame in sim; porch params swappable |
| I-3.2 | Line fetcher + line buffer replacing steal placeholder; scroll latch (per-line provisional) | [BD §9.3, §9.4] | I-3.1 | BLOCKED(M-7,M-11) | pixel stream matches frame capture |
| I-3.3 | IRQ2 generation at vsync (position per M-2) | [BD §9.1] | I-3.1 | BLOCKED(M-2) | game boots and paces at 60 Hz in board sim |

### Phase 4 — board integration

| ID | Item | Spec | Depends | Status | Accept |
|---|---|---|---|---|---|
| I-4.1 | Wire blitter into `ikacore_CV1k.sv` CS6 + BREQ/BACK on HS3 | [BD §2, §5] | I-2.5 | TODO | game writes op lists, blitter fetches via bus mastering in board sim |
| I-4.2 | MT46V16M16 vendor model (or DDR-faithful behavioral) in sim | [BD §6] | I-2.6 | TODO | model passes init/LMR sequence of I-2.6 |
| I-4.3 | MiSTer DDR3-backed VRAM with timing counters gating datapath | [BD §13] | I-2.x | TODO | on-target frame capture matches sim |

---

## Part II — Parameter registry (every clock-level constant)

One row per tunable. `Conf` = confirmed by: `[C]` cross-verified source,
`[M-nn]` our measurement, blank = unconfirmed guess.

### VRAM-domain latencies (unit: VRAM CLK, 13.0208 ns)

| ID | Param | Value | Level | Source | Conf | Notes |
|---|---|---|---|---|---|---|
| P-01 | `P_SRC_ROW_SW` read→read row switch | 6 | B | [PDF]=5 / [MAME]=6 | | fit via M-8 |
| P-02 | `P_RW_TURN` dst read→write | 20 | B | [PDF]=[MAME]=20 | | M-8 |
| P-03 | `P_WR_TURN` dst write→read | 11 | B | [PDF]=10 / [MAME]=11 | | M-8 |
| P-04 | `P_SPRITE_END` draw→draw switch | 12 | B | [PDF]=10 / [MAME]=12 | | lumped; isolate via M-8 |
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
| P-25 | op FIFO depth (fetch-ahead) | 2 chunks (design choice) | B/D | inferred from cadence | | M-5 corroborates |

### Video timing (unit: as stated)

| ID | Param | Value | Level | Source | Conf | Notes |
|---|---|---|---|---|---|---|
| P-30 | dot clock | 6.4 MHz (÷12 CE) | B | derivation [BD §9.1] | | M-1 kills/confirms |
| P-31 | HTOTAL | 407 dots = 4884 VCLK | B | ditto | | expect 60.0184 Hz |
| P-32 | VTOTAL / visible | 262 / 240 | C | [MAME] | [C] | |
| P-33 | frame rate | 60.0184 Hz predicted (60.024 quoted) | B | [MAME] meas. of unknown precision | | M-1 |
| P-34 | H/V sync widths, porches | **?** | A | none | | M-1 |
| P-35 | IRQ2 edge vs vsync, pulse width, 0x24-ack? | **?** | A | none | | M-2 |
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

- 2026-07-06  [claude]  File created. Seeded from blitter_detail.md state:
  board sim (HS3 + U4 + U1) DONE, all blitter items TODO, P-01..42 registered,
  M-1..13 queued. No measurements taken yet.
