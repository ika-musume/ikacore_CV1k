# ikacore_CV1k — CV1000 Core for MiSTer: Project Outline

Goal: a board-level FPGA re-implementation of the Cave CV1000 (CV1000-B / CV1000-D)
arcade platform for MiSTer, with a cycle-approximate blitter so that slowdown
behavior matches the real PCB. This document consolidates what is currently known
from MAME, Buffi's research, and public board documentation, and lists what still
has to be measured (we have a real PCB available).

---

## 1. Reference material

| Source | Location / URL | What it gives us |
|---|---|---|
| Buffi, *CV1000 Blitter performance and behavior* v1.04 (2022-12-30) | `docs/CV1000_Blitter_Research_by_buffi.pdf` | VRAM layout, DDR access pattern, op formats, measured cycle costs, latency formulas, logic-analyzer methodology |
| MAME `cave/cv1k.cpp` + `cv1k_v.*` (blitter, ex-`epic12`) | `docs/mame_cv1k_src/` | Board description, memory map, register map, op decoding, pixel-exact blend math, timing model constants |
| Buffi's research repo | https://github.com/buffis/cv1k_research | U13 CPLD replacement (Verilog!), U2/U4 tools, JTAG notes, datasheet archive links |
| Buffi's blog (CPU slowdown, MAME PR) | https://buffis.com/research/cv1000-cpu-slowdown-investigated/ , https://github.com/mamedev/mame/pull/10849 | SH-3 wait-state findings; discussion of timing-model accuracy |
| JAMMArcade CV1000 page | https://jammarcade.net/cave-cv1000/ | Board revisions, part locations, common failures (video amps, C109, U11) |

## 2. Board hardware summary

Two revisions: **CV1000-B** (Mushihime-Sama … Deathsmiles) and **CV1000-D**
(“SH3B”, DDP DaiFukkatsu and later: 2× work RAM, 2× program flash, no RTC battery).

| Block | Part | Notes |
|---|---|---|
| CPU | Hitachi SH7709S (SH-3) @ **102.4 MHz** (12.8 MHz X1 × 8 PLL) | big-endian; on-chip cache, INTC, BSC, ports |
| Blitter / GPU | **Altera Cyclone EP1C12F324C8** (U8) | no config flash — SH-3 bit-bangs 2,323,240-bit bitstream via Port J at boot (4 known firmware versions A–D, distinguished in MAME by byte checksum 03/3e/f9/e1) |
| Glue CPLD | Altera EPM7032 (U13) | CS6 decode to blitter, RTC/EEPROM interface; labeled with game ID; Verilog replacement exists in buffi's repo |
| VRAM | 2× MT46V16M16 **DDR** SDRAM (U6/U7), 16-bit each, lockstep → 32-bit bus, 64 MB total | **76.8 MHz (13 ns), DDR ⇒ 4 px/clk**; burst length 2 |
| Work RAM | U1 SDR SDRAM: 8 MB (B: MT48LC2M32) / 16 MB (D: IS42S32400) | SH-3 bus, **50 MHz measured (20 ns)**; shared with blitter via BREQ/BACK |
| Program flash | U4: 2 MB 29LV160BB (B) / 4 MB S29JL032H (D) | boot device: SH-3 vector + program + FPGA bitstream |
| GFX flash | U2: Samsung K9F1G08U0M **NAND, 128 MB** | graphics assets; games do manufacturer-ID check (0x98/76, 0x98/79, 0xEC/F1); notoriously failure-prone |
| Audio | Yamaha **YMZ770C-F** @ 16.384 MHz (X3) + U23/U24 sound flash (2× 4 MB) | plays compressed “AMM” streams (MPEG-1 Layer-2 class decoder needed); only right output connected |
| RTC/EEPROM | RTC9701 (U10) + CR2450 (B only) | settings storage; Ibara Black Label renders clock from it |
| Misc | MAX690S supervisor, AD8061 RGB amps (U14–16), LX240A (U11, vsync buffer) | AD8061 degradation and C109 leakage are the classic repair items |

### SH-3 memory map (from MAME)

| Range | Device |
|---|---|
| `0x00000000–0x003FFFFF` | program flash (mirrored 2 MB on -B) |
| `0x0C000000–0x0C7FFFFF` (-B) / `–0x0CFFFFFF` (-D) | work RAM |
| `0x10000000–07` | NAND flash I/O (data/cmd/addr) |
| `0x10400000–07` | YMZ770 |
| `0x10C00000–07` | serial RTC/EEPROM + NAND CE |
| `0x18000000–57` | **blitter registers (CS6, via U13)** |
| Port C/D/F/L | inputs; Port E bit 5 = NAND ready; Port J = FPGA config bit-bang |

### Blitter register map (base 0x18000000)

| Offset | R/W | Function |
|---|---|---|
| 0x04 | W | exec: write 1 → start processing op list |
| 0x08 | W | op-list address in work RAM |
| 0x10 | R | status: `0x10` = ready/idle, `0x00` = busy |
| 0x14 / 0x18 | W | scroll X / Y (display window origin in VRAM) |
| 0x24 / 0x28 | R/W | IRQ handshake / coin-related (reads return 0xFFFFFFFF in MAME) |
| 0x30 / 0x34 | W | contrast / brightness (test menu) |
| 0x38 / 0x3C | W | V / H display offset (test menu) |
| 0x40 / 0x44 | W | clip window X / Y |
| 0x50 | R | DIP switches (S2) |

IRQs: **IRQ2 = vsync pulse** (MAME notes it is the sync pulse, not vblank start),
**IRQ1 = blitter done** (games don't rely on it; one game has a profiling handler —
MAME doesn't emulate it yet, we should).

## 3. Blitter behavior (from Buffi's PDF — the core of this project)

### VRAM organization
- Addressed as an 8192×4096 ARGB1555 bitmap = exactly 64 MB.
- 4 DRAM banks × 8192 rows × 512 columns × 32 bit. One DRAM row = one **32×32-px
  tile**; tiles raster-ordered, 256 per tile-row, bank = `Y/1024`.
- Burst length 2, all accesses 4-pixel aligned ⇒ **4 px per 13 ns VRAM clock**
  (32-bit DDR). Unaligned draws pay one extra CLK per row edge.

### Op list protocol (SH-3 ↔ FPGA)
1. CPU polls `0x18000010` until `0x10` (ready).
2. CPU builds op list in work RAM, waits for IRQ2 (vsync).
3. CPU writes list address → `0x08`, writes 1 → `0x04`.
4. FPGA asserts **BREQ**; CPU grants with **BACK**; FPGA reads the list in
   **64-byte chunks** (16 SRAM clocks @ 50 MHz), releases the bus between chunks.
   Measured inter-chunk cadence ≈ **1.13 µs** (MAME models a 700 ns interval for
   idle/clipped stretches). A 64-byte chunk holds up to three 20-byte Draw ops,
   which are FIFO-queued; fetch runs **concurrently** with execution.
5. When the list is done: IRQ1, status returns to ready.

### Operations
| Op | Code (word 0 hi-nibble) | Size | Notes |
|---|---|---|---|
| Draw | 0x1 | 20 B | src/dst X,Y, W,H, flip X/Y, transparency, blend enable, s/d blend mode (3 bit each), s/d alpha (8 bit), RGB tint multipliers (0x80 = 100 %, 0x20 = neutral in 5-bit) |
| Upload | 0x2 | 16 B + W·H·2 | direct host→VRAM pixel upload |
| Clip | 0xC | 4 B | clip window on/off; **no measured latency effect** (see open Q) |
| Exit | 0x0 / 0xF | 4 B | end of list |

Blend math is 5-bit-per-channel with clamped add/multiply LUT semantics
(see `colrtable*` in `cv1k_v.cpp`); blending/transparency **do not change timing**.
Effective clip = 320×240 window ± **32 px margin** (verified via MMP test mode).

### Measured cycle costs (VRAM CLK = 13 ns unless noted)

Draw, per sprite: read src px/4 + read dst px/4 + write dst px/4, plus per
VRAM-row-switch overhead — Buffi PDF v1.04: **read→read 5, read→write 20,
write→read 10, +10 per new sprite**; MAME after re-measurement uses
**src-row 6, dst-row 20+11, +12 per sprite** (Buffi noted v1.04 numbers were a
few CLK low). 8×8 draw ≈ 93 CLK ≈ 1.21 µs; 240×64 ≈ 12 090 CLK ≈ 157 µs.

Upload: `(16 + W·H·2)/4` SRAM clocks × 20 ns, plus one ~1.13 µs BREQ gap per
64-byte chunk (fetch and VRAM write overlap).

Display refresh steals the VRAM bus: every **63.6 µs** a scanline fetch blocks the
blitter for **2.16 µs** (≈ 3.4 % of bandwidth).

MAME's `HLINE_DELAY` model: `floor(total_blit_time / 63.6 µs) × 2.16 µs` added on top.

### Slowdown model (why this matters)
Per-frame sequence: game logic → wait IRQ2 → kick blitter → next frame's logic runs
in parallel. If the blitter (or the CPU) isn't ready at the next IRQ2, the game
drops to spin-wait: 1 frame of slowdown. Some games *also* add deliberate
slowdown by waiting multiple IRQ2s (program-controlled, free for us).
Buffi's conclusion: much residual inaccuracy in MAME is **SH-3 side** (uncached /
cache-miss wait states, BREQ-induced CPU stalls), not blitter-side. For the core
this means the SH-3 bus state controller and cache timing matter as much as the
blitter datapath.

## 4. Video timing — known vs. unknown

Known / derived:
- Refresh **60.024 Hz**, **262 total lines** (measured from Ibara PCB, in MAME).
- ⇒ line rate ≈ 60.024 × 262 = **15.726 kHz** (period 63.59 µs — consistent with
  Buffi's 63.6 µs scanline-fetch cadence). 15 kHz-class RGB on JAMMA, 320×240 visible.
- Scanline fetch occupies VRAM 2.16 µs/line (≈ 166 VRAM CLK ⇒ roughly 512 px + overhead,
  suggesting the scanout buffer fetches a full 512-px window).

Unknown (MAME TODO: "requires measuring screen raw params"; MAME uses a
placeholder 512×512 container):
- Pixel clock and its derivation from X1 12.8 MHz; htotal in dots.
- H/V sync pulse widths, porches, vsync position relative to IRQ2, IRQ2 pulse width.
- Whether scroll/H-V offset registers shift sync or only the fetch window.

**→ Measurement task #1 on the real PCB** (scope/LA on JAMMA sync + VRAM CS):
capture hsync period & width, vsync, dot clock (if exposed), IRQ2 vs sync edges.

## 5. Blit-command acceptance latency — known vs. unknown

The question: cycles from *presenting* two blit commands until they are *accepted*.

Known:
- Acceptance unit is the 64-byte list chunk (up to 3 Draws at once), 16 clocks
  @ 50 MHz on the bus, ~1.13 µs cycle-to-cycle cadence including re-arbitration
  (varies with how fast the SH-3 acks BREQ).
- Between two queued Draws inside the FPGA: 10–12 VRAM CLK sprite-switch overhead
  on top of the write→read row switch; fetch of the next chunk overlaps execution,
  so back-to-back small sprites are execution-bound, not fetch-bound.
- Kick-off path: write `0x18000004` → first BREQ. **Exact cycle count is not
  published anywhere.**

**→ Measurement task #2 on the real PCB**: LA on CS6 (command write), BREQ/BACK,
SRAM CS, VRAM CS — measure (a) `0x04`-write → first BREQ, (b) BREQ → BACK → first
SRAM read, (c) end-of-chunk → first VRAM op of first draw, (d) spacing of two
consecutive small draws in one chunk vs. across a chunk boundary. Buffi's PDF §
"Method of gathering data" documents the exact probe points.

## 6. Proposed core architecture (MiSTer, DE10-Nano)

| Subsystem | Approach | Risk |
|---|---|---|
| SH7709S | New/adapted SH-3 core: full user ISA, exceptions, INTC, TMU, **cache with real line-fill wait states**, BSC with per-area wait states, BREQ/BACK bus release. (No off-the-shelf SH-3 core exists; J-Core J2 is SH-2 and would need MMU-less SH-3 extensions.) | **Highest** |
| Blitter | Behavioral re-implementation from Buffi's timing model + MAME pixel semantics. Do **not** attempt to run the original EP1C12 bitstream. Implement all 4 firmware checksums initially as one behavior (differences unknown). | Medium |
| VRAM | Needs ≈ 615 MB/s peak (32-bit @ 76.8 MHz DDR). MiSTer SDR SDRAM (16-bit ≈ 130 MHz) is insufficient → **DDR3 via HPS bridge for VRAM**, with the blitter's cycle-cost model enforced by counters (decouple functional storage from timing emulation). Work RAM + ROMs in SDRAM module. | Medium |
| Audio | YMZ770C-F: sequencer + **MPEG-1 Layer-2 (AMM) decoder** in fabric. | Medium-high |
| NAND U2 | 128 MB image staged to DDR3 by HPS loader; emulate NAND command interface incl. ID check and bad-block semantics. | Low |
| U13 CPLD | Port buffi's reverse-engineered Verilog. | Low |
| RTC9701, inputs, DIPs | Straightforward. | Low |
| Video out | Framebuffer scanout from VRAM with scroll regs, 320×240 @ 60.024 Hz / 15.726 kHz through MiSTer video pipeline. | Low |

## 7. Board-level simulation & verification plan

1. **Golden-model harness**: Verilator testbench instantiating CPU + blitter +
   memory models; feed real op lists. Two comparison sources:
   - MAME: `wpset 18000008,1,w,1,{dump blitter_ops.txt,(wpdata-0xA0000000),20000}`
     (Buffi's technique) to dump per-frame op lists + parse script from his repo.
   - The software blitter in `cv1k_v.cpp` as pixel-exact reference (port to a
     C++ checker linked into the TB).
2. **Timing regression**: implement Buffi's latency formulas as an independent
   scoreboard; assert per-op and per-frame cycle counts within a tolerance band;
   sweep alignment cases (X%4, 32-px row crossings, clip edge cases).
3. **PCB captures as ground truth**: Saleae captures of the probe list in §5
   replayed against the RTL (same op lists → compare BREQ/VRAM signal schedules).
4. **Game-level acceptance**: known slowdown scenes (Futari stage 1 fog,
   Pink Sweets, DFK bee stages) frame-counted vs. PCB video.

## 8. Open questions (prioritized)

1. Raw video parameters (pixel clock, sync widths, porches) — measure. (§4)
2. Kick→BREQ and inter-draw acceptance latency — measure. (§5)
3. Firmware A–D behavioral differences (early boards got FPGA updates; nobody
   knows what changed). Affects which games need which timing set.
4. Does Clip reduce draw time of clipped sprites? (Buffi: probably yes —
   MAME models clipped-area timing; verify numbers.)
5. Blend precision: source alpha is 8-bit in the op but VRAM is 1555 and tint
   "neutral" is 0x20 — what does the FPGA actually compute internally? (MAME TODO.)
6. IRQ2 pulse timing/width relative to vsync; IRQ1 exact assertion point.
7. SH-3 uncached/cache-miss wait-state table for our BSC settings (SH7709S
   datasheet + PCB measurement; biggest driver of CPU-bound slowdown parity).
8. mushisam 1-frame sprite/background lag seen in MAME — real or emulation bug?

## 9. Game matrix (CV1000-B/D, from MAME driver)

Mushihime-Sama (+1.5, Tama), Ibara (+Kuro), Espgaluda II, Pink Sweets,
Mushihime-Sama Futari (+1.5/BL), Muchi Muchi Pork!, Deathsmiles (+MBL),
Deathsmiles BL (D), DDP DaiFukkatsu (+BL) (D), Akai Katana (D), SaiDaiOuJou (D),
Medal Mahjong Moukari Bancho (touchscreen, needs SCIF serial).
Note: Akai Katana / SDOJ were removed from MAME upstream (exA-Arcadia request);
sets differ per-region mostly in U4 only. -D board doubles RAM/flash sizes.
