# CV1000 Blitter — HDL-Level Detail (for core implementation)

Working document for implementing the EP1C12 blitter in HDL. Written from the
perspective of: *"we are about to write SystemVerilog; every number here is either
a synthesis parameter or a testbench assertion."*

Every fact is tagged with a confidence level:

| Level | Meaning | Action |
|---|---|---|
| **A** | Not documented anywhere. | Must be measured on PCB before/while implementing. |
| **B** | Cycle numbers exist but sources disagree (PDF v1.04 vs. clipping blog vs. MAME). | Implement as `parameter`, reconfirm on PCB. |
| **C** | Cross-verified (≥2 independent sources, or LA capture matches formula). | Safe to hard-code. |
| **D** | Internal implementation detail, externally unobservable. | Free choice; note rationale. |

Sources: `docs/CV1000_Blitter_Research_by_buffi.pdf` (v1.04, 2022-12-30) — cited
as **[PDF]**; https://buffis.com/research/more-cv1000-research-now-featuring-clipping/
— cited as **[CLIP]** (supersedes parts of [PDF]); `docs/mame_cv1k_src/cv1k_v.cpp/.h`
— cited as **[MAME]**; DDR datasheet MT46V16M16 — **[DDR-DS]**.

---

## 1. Clock tree and domains

```
X1 12.8000 MHz
 ├─ ×8   → 102.4  MHz  SH-3 core clock (Iφ)
 ├─ ÷2 of core → 51.2 MHz  CKIO = external bus clock (Bφ)  ← op fetch domain
 └─ ×6   → 76.8  MHz  VRAM DDR clock                        ← blitter datapath domain
```

| Item | Value | Level | Note |
|---|---|---|---|
| CKIO period | 19.531 ns | **C** | Buffi measured "50 MHz / 20 ns" with a 500 MS/s LA — quantization; nominal 51.2 MHz is the correct number. [MAME] uses 20 ns. All CKIO-derived times in [PDF] are ~2.3 % pessimistic. |
| VRAM CLK period | 13.021 ns | **C** | [PDF] uses 13 ns. 93 CLK × 13.021 = 1211 ns vs. measured 1212 ns — better fit than 13 ns flat. |
| DDR data rate | 153.6 MT/s × 32 bit = 614.4 MB/s = **4 px / VRAM CLK** | **C** | Two ×16 chips in lockstep (CS/RAS/CAS/WE/A/BA tied). |
| FPGA-internal clock | probably 76.8 MHz core + DDR I/O regs; Cyclone I has 2 PLLs, a 153.6 MHz capture clock is possible | **D** | Externally unobservable. Our core: run blitter datapath at one clock, enforce external cadence with counters. |
| Video dot clock | 6.4 MHz (12.8/2), ÷12 enable in VRAM domain | **B** | Working assumption, see §9.1; confirm via ppm frame-rate measurement. |

> Note: 12.8 × 8 = **102.4** MHz (not 102.8); CKIO = 102.4/2 = **51.2** MHz (not
> 51.4). If a frequency counter on the PCB really reads 51.4, X1 is off-nominal —
> re-measure X1 directly (that would rescale *every* number in this file).

**HDL consequence**: two clock domains matter — CKIO (SH-3 bus slave + master) and
VRAM (datapath + scanout). One dual-clock FIFO between them (op bytes downstream,
status upstream).

---

## 2. Top-level block diagram (target structure)

```
              CKIO domain (51.2)                 VRAM domain (76.8)
        ┌──────────────────────────┐      ┌──────────────────────────────┐
SH-3 ──►│ reg_slave (CS6 via U13)  │      │  op_decode / draw_engine     │
 bus    │ 0x18000000..57           │      │  ┌────────┐   ┌───────────┐  │
        ├──────────────────────────┤      │  │ 64-bit │──►│ blend ALU │  │
        │ op_fetch (BREQ/BACK      │─FIFO─┼─►│ op FIFO│   │ 4 px/clk  │  │
SH-3 ◄──│ bus master, 64 B bursts) │      │  └────────┘   └───────────┘  │
 BREQ   └──────────────────────────┘      │        │            │        │
                                          │        ▼            ▼        │
                                          │  ┌──────────────────────┐    │
                    IRQ1/IRQ2 ◄───────────┤  │ vram_scheduler       │◄─┐ │
                                          │  │ (DDR cmd generator)  │  │ │
                                          │  └──────────┬───────────┘  │ │
                                          │             ▼              │ │
                                          │        DDR 32-bit          │ │
                                          │             ▲              │ │
                                          │  ┌──────────┴───────────┐  │ │
                                          │  │ scanout line fetcher │──┘ │
                                          │  │ (priority master)    │    │
                                          │  └──────────┬───────────┘    │
                                          └─────────────┼────────────────┘
                                                   line buffer → DAC, syncs
```

---

## 3. Register file (base 0x18000000, CS6, decoded by U13 CPLD)

All facts **C** ([MAME] `blitter_r/w`, confirmed by game behavior), except where noted.

| Ofs | Dir | Name | Bits used | Behavior |
|---|---|---|---|---|
| 0x04 | W | `EXEC` | [0] | 1 → latch `LIST_ADDR`, `CLIP_X/Y` into shadow copies, start op fetch. Write while busy: behavior **A** (games always poll first; MAME serializes). |
| 0x08 | W | `LIST_ADDR` | [28:0] | Physical work-RAM address of op list. Sampled at EXEC, not live. **C** |
| 0x10 | R | `STATUS` | [4] | 1 = idle/ready, 0 = busy. Busy window = EXEC until last op retired (incl. hline steals). Sub-cycle deassert timing **A**. |
| 0x14 | W | `SCROLL_X` | [15:0]? | Scanout window origin X. Latch point (immediate / per line / per frame) **A** — see §9.4. |
| 0x18 | W | `SCROLL_Y` | | ditto |
| 0x24 | R/W | IRQ handshake | | reads 0xFFFFFFFF in MAME; written at start of IRQ routines. Real semantics **A** (candidate: IRQ ack/clear register — probe: does IRQ2 line stay asserted until 0x24 write?). |
| 0x28 | R/W | coin-counter related | | reads 0xFFFFFFFF. **D** |
| 0x30 / 0x34 | W | contrast / brightness | | analog-side (test menu). Video DAC path, not blitter. **D** for timing |
| 0x38 / 0x3C | W | V / H display offset | | shifts raster (test menu). Interaction with sync **A**. |
| 0x40 / 0x44 | W | `CLIP_X` / `CLIP_Y` | | clip window origin; effective window = X..X+319, Y..Y+239 **plus 32 px margin on all sides** ([MAME] `CV1K_CLIP_MARGIN`, from MMP VRAM-viewer observation — **B**, margin value re-confirmable). Latched at EXEC. **C** |
| 0x50 | R | DSW | [3:0] | DIP switch S2. **C** |

Bus width note: SH-3 area 6 is set up as 32-bit; MAME installs 64-bit handlers but
the underlying accesses are 32-bit writes (`u32` regs). **C**

```systemverilog
// reg_slave.sv — CKIO domain
typedef struct packed {
  logic [28:0] list_addr;
  logic [15:0] scroll_x, scroll_y;
  logic [15:0] clip_x,   clip_y;
} blit_regs_t;

always_ff @(posedge clk_ckio) begin
  exec_pulse <= 1'b0;
  if (sel_cs6 && we) begin
    unique case (addr[6:2])
      5'h01: exec_pulse       <= wdata[0];        // 0x04
      5'h02: regs.list_addr   <= wdata[28:0];     // 0x08
      5'h05: regs.scroll_x    <= wdata[15:0];     // 0x14  (latch point: LEVEL A)
      5'h06: regs.scroll_y    <= wdata[15:0];     // 0x18
      5'h10: regs.clip_x      <= wdata[15:0];     // 0x40
      5'h11: regs.clip_y      <= wdata[15:0];     // 0x44
      default: ;
    endcase
  end
  if (exec_pulse) begin      // shadow-latch semantics per [MAME]
    sh.list_addr <= regs.list_addr;
    sh.clip_x    <= regs.clip_x;
    sh.clip_y    <= regs.clip_y;
  end
end
assign rdata = (addr[6:2]==5'h04) ? {27'b0, ~busy, 4'b0}   // 0x10 ready bit
             : (addr[6:2]==5'h14) ? {28'b0, dsw}           // 0x50
             : 32'hFFFF_FFFF;                              // 0x24/0x28 observed value
```

---

## 4. Op list — bit-exact formats (**C**, [MAME] comments + [PDF])

Ops are a byte stream of big-endian 16-bit words in work RAM. Opcode = top nibble
of word 0. In HDL, after a 32-bit fetch, word order = MSB half first (the `^3`
address XOR in MAME is a host-endianness artifact — ignore it).

### 4.1 DRAW — 0x1, 20 bytes
```
word 0  [15:12]=0001  [11]=flipX  [10]=flipY  [9]=blend_en  [8]=trans_en
        [6:4]=s_mode  [2:0]=d_mode
word 1  [15:8]=s_alpha  [7:0]=d_alpha          (8-bit fields, HW uses top 5 bits)
word 2  [12:0]=src_x                            (0..8191)
word 3  [11:0]=src_y                            (0..4095)
word 4  [15:0]=dst_x   signed 16-bit
word 5  [15:0]=dst_y   signed 16-bit
word 6  [12:0]=width-1
word 7  [11:0]=height-1
word 8  [7:0]=tint_R                            (0x80 = 100 %)
word 9  [15:8]=tint_G  [7:0]=tint_B
```
Blend mode encoding (test-mode label): 0=+alpha 1=+source 2=+dest 4=−alpha
5=−source 6=−dest (3/7 reserved). See §8 for datapath semantics.

### 4.2 UPLOAD — 0x2, 16 bytes + W·H·2 payload
```
word 0..1 = 0x2000, 0x0000    word 2..3 = 0x9999, 0x9999  (fixed)
word 4 [12:0]=dst_x  word 5 [11:0]=dst_y
word 6 [12:0]=w-1    word 7 [11:0]=h-1
then W·H ARGB1555 pixels, row-major
```

### 4.3 CLIP — 0xC, 4 bytes
```
word 0 = 0xC000
word 1 = cliptype:  !=0 → window mode (CLIP_X/Y ± margin);  0 → whole VRAM (8192×4096)
```

### 4.4 EXIT — 4 bytes: word0 top nibble 0x0 **or** 0xF → end of list, assert IRQ1.

---

## 5. Op fetch engine (SH-3 bus mastering, CKIO domain)

### 5.1 Protocol (**C**, [PDF] §Overall sequence + LA captures)

```
CKIO      ─┐_┌─┐_┌─┐_┌─┐_┌─┐_┌─┐_┌─┐_┌─┐_┌─  (51.2 MHz)
BREQ#  ────╲_______________________________╱────   FPGA requests bus
BACK#  ────────╲___________________________╱────   SH-3 grants (delay = CPU-dependent!)
SRAM_CS# ────────╲_╱╲_╱ ... 16 beats ... ╲_╱────   64 B = 16 × 32-bit reads
D[31:28]           op nibbles visible here (LA trick to identify op types)
```

| Quantity | Value | Level | Source |
|---|---|---|---|
| Chunk size | 64 bytes, always | **C** | [PDF] |
| Bus beats per chunk | 16 × 32-bit @ CKIO | **C** | [PDF] "16 SRAM CLK pulses" |
| Chunk-to-chunk cadence during **uploads** | ≈ 1.13 µs (variance: depends on how fast SH-3 acks BREQ) | **B** | [PDF]; = 312 ns of bus + ~0.8 µs re-arbitration/idle |
| Chunk cadence when ops are **all-clipped / idle** | ≈ 700 ns | **B** | [MAME] `OPERATION_READ_CHUNK_INTERVAL_NS`, calibrated so 80 clipped draws (1600 B = 25 chunks) ≈ 17.5 µs, matching [CLIP] LA measurement |
| EXEC write → first BREQ# assertion | **unknown** | **A** | Measure: CS6 write strobe → BREQ# edge, in CKIO cycles |
| BREQ→BACK grant latency distribution | CPU-state dependent; unmeasured distribution | **A** | affects worst-case fetch stall; capture histogram while game runs |
| FIFO depth in FPGA | ≥ 64 B; exact depth unknown | **B** | Draw fetch overlaps execution [PDF]; sustained 3 draws/chunk never starves. Choose 2×64 B in HDL; verify with back-to-back clipped-op cadence (700 ns implies fetch-ahead of ≥1 chunk). |

### 5.2 Command acceptance — the "two draws" question

Two consecutive DRAW ops (20 B each) can sit in **one** 64-byte chunk (3 fit, 4 B
spare). Consequences (**C** unless noted):

- Both are queued in the same 312 ns bus burst. The second draw's *acceptance*
  costs zero extra fetch time; its *start* is `end_of_draw1 + sprite_switch`
  (10–12 VRAM CLK, §7.3).
- If the second draw is in the next chunk and the FIFO has drained, worst case
  adds one chunk cadence (0.7–1.13 µs, **B**).
- Fetch never interrupts a running draw; it only competes for the *SH-3* bus,
  not the VRAM bus. Upload is the exception: payload beats occupy the SH-3 bus
  for the whole payload (§7.5).

### 5.3 Fetch FSM skeleton

```systemverilog
typedef enum logic [2:0] {F_IDLE, F_REQ, F_WAIT_GRANT, F_BURST, F_GAP, F_DONE} fetch_st_t;

always_ff @(posedge clk_ckio) begin
  case (st)
    F_IDLE:  if (exec_latched)                     st <= F_REQ;
    F_REQ:   begin breq_n <= 1'b0;                 st <= F_WAIT_GRANT; end
    F_WAIT_GRANT: if (!back_n)                     st <= F_BURST;      // latency: PCB-measured
    F_BURST: if (beat == 15) begin                 // 16 × 32-bit reads, addr += 4
               breq_n <= 1'b1;
               st <= fifo_has_exit ? F_DONE
                   : fifo_afull    ? F_GAP : F_REQ;
             end
    F_GAP:   if (gap_cnt == GAP_CYCLES)            st <= F_REQ;        // param, LEVEL B
    F_DONE:  if (engine_idle) begin irq1 <= 1'b1;  st <= F_IDLE; end
  endcase
end
```

---

## 6. VRAM subsystem (DDR, 76.8 MHz domain)

### 6.1 Geometry and address mapping (**C**, [PDF] cross-checked against chip org)

2 × MT46V16M16 (4 banks × 8192 rows × 512 cols × 16 bit), lockstep → logical
device: 4 banks × 8192 rows × 512 cols × **32 bit**. One DRAM row = one **32×32-px
tile** (512 × 32 b = 2 KB = 32·32·2 B). Whole VRAM = 8192×4096 px ARGB1555 = 64 MB,
no holes.

Pixel (X, Y) → DDR address:
```
BA[1:0] = Y[11:10]                        // bank   = Y / 1024
ROW[12:0] = {Y[9:5], X[12:5]}             // row    = (X/32) + 256*((Y%1024)/32)
COL[8:0]  = {Y[4:0], X[4:1]}              // column = (X%32)/2 + 16*(Y%32)
// X[0] resolved inside the 32-bit word (2 px per column)
// Blitter only issues 4-px-aligned accesses → COL[0] always 0 ("A0 always low", [PDF])
```

```systemverilog
function automatic ddr_addr_t px2addr(input logic [12:0] x, input logic [11:0] y);
  px2addr.ba  = y[11:10];
  px2addr.row = {y[9:5], x[12:5]};
  px2addr.col = {y[4:0], x[4:1]};   // col[0]==x[1]; blitter keeps x[1:0]==0 → col[0]=0
endfunction
```

### 6.2 Mode register / burst configuration

| Item | Value | Level |
|---|---|---|
| Burst length | **2** (LMR with low bits = 0x1) → one READ/WRITE command transfers 2×32 bit = 4 px in **one** clock (DDR both edges) | **C** [PDF LA capture] |
| CAS latency | unknown; CL=2 or 2.5 plausible at 76.8 MHz | **B** — decode A[6:4] from the boot-time LMR capture (Buffi's init trace already contains it: A-bus values 0x0000/0x0001/0x000F visible) |
| Burst type | sequential (A3=0 implied) | **B** same capture |
| DDR command truth table (CS,RAS,CAS,WE) | ACT=LLHH READ=LHLH WRITE=LHLL PRE=LLHL LMR=LLLL AREF=LLLH NOP=LHHH | **C** [DDR-DS] — use to decode any LA capture |

### 6.3 Steady-state streams (what the LA should show)

Aligned read of one full tile row region, BL=2, gapless (4 px/clk):

```
VRAM CLK   1   2   3   4   5   6   7   8   9  ...
CMD       ACT NOP RD  RD  RD  RD  RD  RD  RD  ... (256 RDs for 1024 px)
ADDR      row      c0  c2  c4  c6  c8  ...
DQ (DDR)          ......≪CL≫ d0 d0 d1 d1 d2 d2 ...   2 beats/clk = 4 px/clk
```

8×8 draw (64 px, src and dst each within one tile), total **93 CLK ≈ 1.211 µs**
(**C** — [PDF] formula matches LA measurement 1.212 µs):

```
|◄16►|◄5►|◄16 ►|◄ 20 ►|◄ 16 ►|◄10►|◄ 10 ►|
 RD    rr  RD     r→w    WR     w→r  sprite
 src  sw   dst   turn    dst    turn  switch      (all numbers = VRAM CLKs)
```

### 6.4 Inter-operation penalties — THE Level-B table

Three sources, three slightly different constant sets. Implement as parameters;
the *structure* (which penalties exist and when they apply) is **C**, the *values*
are **B**:

| Penalty | [PDF] v1.04 | [CLIP] revision | [MAME] as merged | apply when |
|---|---|---|---|---|
| src row switch (read→read) | 5 | (5, inside lump) | 6 | per **source** VRAM-row access |
| read→write turnaround | 20 | (20, inside lump) | 20 | per **destination** row: after dst read, before dst write |
| write→read turnaround | 10 | (10, inside lump) | 11 | per destination row, after write |
| lumped per-dst-row overhead | — | **35** = 5+20+10 | 31 (=20+11) +6 on src side | [CLIP]: count *destination accesses* only |
| per-sprite end overhead | 10 | 10 | 12 | once per draw op |
| fully-clipped draw | n/a | **0 VRAM work** — only FIFO fetch cadence | `idle_blitter()`: 700 ns per accumulated 64 B | sprite entirely outside clip window |
| partial clip | n/a | cost computed on **clipped** area; edge rows still pay row overheads; writes still occur up to 32 px outside window (margin) | same model | clipped draws |

Notes for the HDL scheduler:
- Buffi flagged v1.04 totals as "very slightly below measured — off by a few CLK";
  MAME's +1s (6/11/12) are that correction. Re-measure and settle. (**B**)
- The dst read and dst write hit the **same open row**, so 20/10 are *bus/pipeline
  turnarounds*, not PRE+ACT costs; the src 5–6 CLK *is* a PRE+ACT-class row switch
  (tRP+tRCD ≈ 2+2 CLK @ 76.8 MHz + command slot ≈ matches). Decomposition **D**,
  totals **B**.
- Alignment: X not ≡ 0 (mod 4) adds one 4-px column on both dst read and dst
  write per row (**C**, measured 32 px: 652 ns aligned vs 1068 ns misaligned-by-2
  [CLIP] — note 1068−652 = 416 ns = 32 CLK, i.e. *not* just +2 CLK: misalignment
  can double the column count for narrow draws. Model: pad dst span to 4-px grid,
  cost = padded_pixels, and re-verify on PCB. **B**)

### 6.5 Draw cost formula (scoreboard / golden model)

```systemverilog
// Testbench golden model. LEVELS: structure C, constants B (parameters!).
function automatic longint draw_cost_vclk(
    int src_x, src_y, dst_x, dst_y, w, h,   // AFTER clipping to window+margin
    int P_SRC_SW = 6, P_RW = 20, P_WR = 11, P_SPR = 12);
  int src_rows = vram_rows(src_x, src_y, w, h);       // 32×32 tile spans
  int dst_rows = vram_rows(dst_x, dst_y, w, h);
  int dst_x0 = dst_x & ~3;
  int dst_x1 = (dst_x + w - 1) | 3;
  longint src_px = longint'(w) * h;                    // src always 4-aligned in VRAM
  longint dst_px = longint'(dst_x1 - dst_x0 + 1) * h;  // padded to 4-px grid
  return src_px/4 + 2*(dst_px/4)                       // read src + read dst + write dst
       + src_rows*P_SRC_SW + dst_rows*(P_RW + P_WR) + P_SPR;
endfunction

function automatic int vram_rows(int x, int y, int w, int h);  // [MAME] logic, C
  int xr = 0, n = 0;
  for (int xp = w; xp > 0; xp -= 32) begin
    xr++;  if (((x & 31) + (xp < 32 ? xp : 32)) > 32) xr++;
  end
  for (int yp = h; yp > 0; yp -= 32) begin
    n += xr; if (((y & 31) + (yp < 32 ? yp : 32)) > 32) n += xr;
  end
  return n;
endfunction
```

Cross-checks this must reproduce (**C** anchors):
- 8×8 aligned, 1 row each: 16+5+16+20+16+10+10 = 93 CLK = 1.211 µs (meas. 1.212).
- 16×12: 189 CLK = 2.457 µs (meas. 2.484 — note ~1 % gap, part of the +1 story).
- 240×64 @ (768,0): 14×803 + 2×419 + 10 = 12 090 CLK = 157.4 µs; +3 hline steals
  → 163.9 µs (meas. 165.0 µs).
- 80 fully-clipped 1×324 draws: 25 chunks ≈ 17.5 µs total, zero VRAM traffic.

### 6.6 Refresh — open problem (**A**)

DDR needs 8192 AREF / 64 ms ⇒ average interval 7.81 µs ⇒ **8.14 refreshes per
63.6 µs line period**. No source mentions refresh at all. Hypotheses:

- (a) batched inside the per-line 2.16 µs bus steal (8×tRFC(≈80 ns→7 CLK)=56 CLK
  — fits: 166 CLK slot − 56 = 110 CLK ≈ 11 tiles × (8 data + 2 sw));
- (b) distributed AREF between ops (would show as isolated LLLH commands);
- (c) row activity from scanout+blit deemed sufficient (risky, unlikely for a
  Cave production board).

**Measurement**: decode CS/RAS/CAS/WE during (i) the hline window and (ii) long
idle (attract-mode static screen). Count LLLH. This decides scheduler design.

---

## 7. Draw / upload engine sequencing

### 7.1 Per-draw sequence (**C**, [PDF])
For each destination VRAM row (32×32 tile intersection) of the sprite:
```
1. READ  source pixels        (4 px/CLK, src rows interleaved as needed, 5–6 CLK per src row switch)
2. READ  destination pixels   (always, even when blend/trans disabled — C)
3. 20 CLK read→write
4. WRITE merged pixels        (4 px/CLK)
5. 10–11 CLK write→read
```
then +10–12 CLK to switch to the next sprite. Blend/transparency settings change
**nothing** in timing (**C**, verified via MMP test menu A/T toggles).

### 7.2 Execution vs. fetch concurrency (**C**)
Draw payloads come from VRAM only; op fetch (SH-3 bus) runs in parallel and can
be ignored in draw timing. Upload is the opposite: payload comes over the SH-3
bus and VRAM writes hide behind it.

### 7.3 Draw FSM sketch

```systemverilog
typedef enum logic [3:0] {
  D_IDLE, D_DECODE, D_CLIPCHK,
  D_RD_SRC, D_SW_SRC,           // src tile stream, penalty P_SRC_SW between rows
  D_RD_DST, D_TURN_RW,          // dst tile read, then 20-CLK turnaround
  D_WR_DST, D_TURN_WR,          // dst tile write, then 10/11-CLK turnaround
  D_NEXT_ROW, D_SPRITE_END      // +12 CLK, pop next op
} draw_st_t;

// The penalty states are pure down-counters:
always_ff @(posedge clk_vram) begin
  case (st)
    D_TURN_RW: if (--pen == 0) st <= D_WR_DST;   // pen loaded with P_RW
    D_TURN_WR: if (--pen == 0) st <= (rows_left ? D_RD_SRC : D_SPRITE_END);
    ...
  endcase
  if (hline_req) begin saved_st <= st; st <= D_HLINE_STALL; end // §9.3, LEVEL B granularity
end
```

### 7.4 Blend datapath (semantics **C** from [MAME]; internal layout **D**)

All arithmetic is 5 bit/channel. Required throughput: **4 px/CLK** (matches
read/write rate; a 4-lane combinational unit at 76.8 MHz is trivial timing).

```
factor selection (per channel c ∈ {r,g,b}, values are 5-bit):
  s_mode 0: Fs = s_alpha            4: Fs = ~s_alpha (1-x)
         1: Fs = src.c              5: Fs = ~src.c
         2: Fs = dst.c              6: Fs = ~dst.c
  d_mode analogous with d_alpha / src / dst reversed roles
out.c = clamp31( mul5(src.c*, Fs) + mul5(dst.c, Fd) )      // when blend_en
src.c* = tint applied first: mul5t(src.c, tint.c)          // tint 6-bit, 0x20 = 1.0,
                                                           // range → ×0..×1.97, clamped
trans_en: pixels with A=0 in source are skipped (dst untouched);
          with trans_en=0, A bit is copied through.
mul5(a,b)   = min(31, a*b/31)        // MAME colrtable
mul5t(a,t)  = min(31, a*t/31)        // t up to 0x3F
```
Open precision question (**B**): ops carry 8-bit alphas, HW keeps top 5 bits
(MAME `>>3`) — but "normal" tint = 0x20 (6-bit unity) proves the multiplier grid
is finer than 5 bits at least for tint. Exact rounding: bit-compare candidate
formulas against PCB screenshots (test-menu blending screens are ideal).

### 7.5 Upload timing (**C** structure, **B** constants)

```
bus_clks   = (16 + W*H*2) / 4                 // CKIO beats (header + payload)
gaps       = ceil(bus_clks / 16) - 1          // one BREQ/BACK cycle per 64 B
T_upload   ≈ bus_clks * 19.53 ns + gaps * T_GAP,   T_GAP ≈ 1.13 µs (B — variance!)
```
Anchors: 8×8 → 36 beats, 2 gaps ≈ 2.98 µs; 256×5 → 644 beats, 40 gaps ≈ 58.08 µs
(measured 58.77 µs). VRAM writes are concurrent and hidden. Note the gap
formula assumes a chunk-aligned start; a command starting at offset *k* in
the 16-word chunk grid ([PDF] "encountered at offset 5") crosses
`ceil((k + bus_clks)/16) − 1` boundaries — up to one more gap. The RTL needs
no correction (chunking is by stream position, so the offset emerges); the
closed form is a cost-table estimate only. The difference between
T_GAP≈1.13 µs (upload) and ≈0.7 µs (idle op stream) is unexplained —
candidates: BACK-ack latency under different CPU load, or upload↔VRAM-insert
coupling ([PDF] calls them "coupled": the next chunk may wait on the 64 B
drain into a VRAM also serving scanout/queued ops, which clipped-op streams
never touch) — **A/B: measure both cases, correlate gap length vs VRAM load.**

### 7.6 Timing-decoupled FSM pattern (design contract for latency tuning)

Inter-instruction switch latencies are **not isolated anywhere**: the per-sprite
+10/+12 CLK is a lumped fit-constant (draw→draw only); draw↔upload, clip/exit
decode cost, and cross-chunk FIFO-drain transitions are Level A. Therefore the
RTL must keep function and timing separate so these can be tuned post-measurement
without FSM surgery:

- The datapath FSM emits **events** (src-row done, rd→wr turn, wr done, op
  retired, op-type switch, kick); a separate timing unit inserts stalls from a
  **latency table** indexed by event — the single source of truth, shared with
  the TB scoreboard, runtime-loadable in sim (`$plusargs`/backdoor) so PCB
  fitting is a sweep, not a recompile.
- **Knobs map 1:1 to observables**: every table entry corresponds to exactly one
  measurable gap on the DDR command bus (CAS-to-CAS spacing / CS idle) or BREQ
  timing. Datapath is fully pipelined (0-cycle nominal); *all* dead time lives
  in the table — never smear latency into pipeline-fill of the blend ALU.
- **hline steal and fetch starvation are orthogonal stall inputs** that gate the
  penalty counter; they are not table entries (tuning must not interact).
- **Nondeterminism is quarantined**: only BREQ→BACK grant latency varies; it
  stays behind the op FIFO in the fetch unit, so the execution side is
  cycle-reproducible (same op list in → same cycle count out).

```systemverilog
typedef enum logic [2:0] {EV_SRC_ROW, EV_RD2WR, EV_WR2RD, EV_OP_DRAW,
                          EV_OP_UPLD, EV_OP_CLIP, EV_KICK} evt_t;
logic [7:0] lat [7] = '{6, 20, 11, 12, /*A:*/0, /*A:*/0, /*A:*/0};

always_ff @(posedge clk_vram)
  if (evt_valid)                                stall_cnt <= lat[evt];
  else if (!hline_steal && stall_cnt != 0)      stall_cnt <= stall_cnt - 1;
assign engine_go = (stall_cnt == 0) && !hline_steal;
```

Tuning loop later: PCB LA capture → diff observed vs. RTL command-bus gaps →
edit table → re-run scoreboard. Pixel correctness is never re-verified because
function and timing share no logic.

### 7.7 Tile buffer (existence **C** — implied by sequencing; layout **D**)

The op FIFO (§5) carries *commands only*; draw pixel payloads come from VRAM
(§7.2) and need their own on-chip storage. The per-tile sequence (§7.1) is
strictly serial — all source pixels of the dst-tile intersection are read
*before* the dst read/write phases, and the src DRAM row is closed by then —
so the source data must be buffered locally. The real chip provably does this:
LA captures show uninterrupted src-read bursts (up to 256 CLK), never
interleaved with dst accesses.

**Load unit = sprite ∩ dst-tile intersection, not a fixed tile.** The buffer is
*sized* for the worst case — full 32×32 coverage = 1024 px × 16 b = 2 KB (one
DRAM row) — but each fill reads only the intersection's actual pixels:

| Draw | intersection px | src read CLK | anchor |
|---|---|---|---|
| 8×8, 1 row | 64 | 16 | [PDF] 93-CLK trace |
| 16×12, 1 row | 192 | 48 | [PDF] 2.484 µs trace |
| 240×64 full tile | 1024 | 256 | [PDF] 803-CLK/row |

Src reads are exact (`src_px = w·h`, always 4-px aligned in VRAM — §6.5); only
the **dst** span is padded to the 4-px grid.

**Datapath integration:**
- **D_RD_SRC** fills the buffer, addressed in *dst-tile-relative* coordinates.
  Src and dst tile grids don't align in general (`src_x%32 ≠ dst_x%32`), so one
  fill may stream from up to 4 src rows (the D_RD_SRC/D_SW_SRC interleave, 5–6
  CLK per src row switch).
- **D_RD_DST** blends on the fly: incoming dst px + buffered src px → §7.4 ALU
  (4 px/CLK) → merged result written back into the buffer (dual-port BRAM, or a
  second 2 KB buffer for a simpler pipeline). Zero added time — consistent with
  blend settings changing nothing in timing.
- **D_WR_DST** streams the merged buffer out at 4 px/CLK. Buffer locations in
  the padded 4-px dst columns but outside the sprite (or trans-skipped, A=0)
  hold the pass-through dst value from D_RD_DST — they are rewritten unchanged,
  not skipped (matches writes landing up to 32 px outside the clip window, §8).

**Sizing**: minimum one dual-port 2 KB BRAM (1024 × 16); 2 × 2 KB (src +
merged) is cheap on EP1C12-class BRAM budgets (§9.4) and recommended.

---

## 8. CLIP operation semantics (updated by [CLIP] — supersedes [PDF] "unknown")

- CLIP op itself: no direct latency beyond its 4 bytes of fetch (**C**).
- Window mode: draws clipped to `[clip_x−32 .. clip_x+320+31] × [clip_y−32 .. clip_y+240+31]`
  (the ±32 margin is real, observed as garbage drawn outside window in MMP; **B** on exact margin).
- Fully-outside draw: zero VRAM work; only contributes 20 B toward the 64 B fetch
  cadence (700 ns per chunk) (**C** — Pink Sweets 80×(1×324) ⇒ ≈17.5 µs).
- Partially clipped: engine processes only surviving area (plus 4-px padding and
  row-edge overheads). Exact per-edge behavior (does src read still fetch full
  rows?) **B** — MAME clips both src and dst counts identically; verify.

**HDL**: clip evaluation must happen at *decode*, before any VRAM commands are
issued, and must be exact to the padded-4-px grid or slowdown accuracy drifts.

---

## 9. Video scanout and its interaction with the blitter

### 9.1 Frame-level facts

| Item | Value | Level |
|---|---|---|
| Refresh | 60.024 Hz (Ibara PCB measurement) | **C** (as measured; re-confirm on our board) |
| Total lines | 262 | **C** |
| Line rate | 60.024 × 262 = 15.7263 kHz → 63.586 µs | **C** (derived; matches [PDF] "every 63.6 µs") |
| Visible | 320 × 240 | **C** |
| Hsync width / porches / vsync width / dot clock | — | **A** — measure JAMMA sync with scope + LA |
| IRQ2 position = "V-sync pulse, not V-blank" | [MAME] comment | **B** — pin down edge vs. sync, and pulse width (**A**) |

**Working assumption (B, upgraded from A): dot clock = 6.4 MHz, HTOTAL = 407 dots.**
Rationale:
- (a) only candidate that keeps the whole design in the 12.8 MHz integer clock
  family: 6.4 = 12.8/2 = 76.8/12 = 153.6/24. The pixel "clock" is a ÷12 clock
  enable inside the VRAM domain — **zero CDC** between scanout counter, hline
  steal, and blitter datapath;
- (b) htotal = 6.4 MHz/(60.024 Hz × 262) = 406.96 → integer within 0.01 %;
- (c) active fractions come out NTSC-like: 320/407 = 78.6 % H, 240/262 = 91.6 % V.

Derived timing set (with HTOTAL = 407 exact):
line = 6.4 MHz/407 = 15,724.8 Hz (63.594 µs); frame = 60.0184 Hz;
**4884 VRAM CLK per line**; 106,634 dot clocks (1,279,608 VRAM CLK) per frame.
Residual vs. MAME's "60.024 Hz" is 93 ppm — inside plausible measurement error;
the ppm-grade IRQ2/vsync frequency measurement (checklist #1/#2) confirms or
kills this with one probe (expect ≈60.018 Hz if true).

Rejected candidates: 7.68 MHz (htotal 488.4, non-integer), 8.533 MHz (542.6),
12.8 MHz (813.9, implausible 39 % active).
The only other integer-VRAM-CLK line length inside the 60.024 error band is
4883 CLK/line → 60.031 Hz, but 4883 = 19 × 257 admits no sensible dot clock —
so the frequency measurement discriminates cleanly: **60.0184 Hz ⇒ 407 × 12
confirmed; 60.031 Hz ⇒ hypothesis dead, re-derive.**

### 9.2 The per-line VRAM steal (**C** existence/values, **B** composition)

Every 63.6 µs the scanout fetcher takes the DDR bus for **2.16 µs ≈ 166 VRAM CLK**,
stalling the draw engine ([PDF] "Rendering output video latency"; MAME adds
`floor(t/63.6µs)·2.16µs`). Composition hypotheses (decide by LA command decode):

- 12 tiles × (8 CLK data + ~6 CLK row switch) = 168 CLK ≈ 2.19 µs → fetches
  384 px = 320 + margins (12-tile window covers any scroll alignment) — best fit;
- 11 tiles × (8+6) = 154 CLK = 2.01 µs — fits if switch cost ≈ 7;
- 11–12 tiles + ~8 batched AREF (§6.6) also lands on ≈166 CLK.

A scanline (Y fixed) walks horizontally consecutive DRAM **rows** within one
bank (row = f(X/32)); each tile contributes 32 px = 8 READ commands from one row.
Consecutive tiles = different rows, same bank ⇒ PRE/ACT between each ⇒ the
read→read penalty applies per tile. (**C** reasoning from address map.)

### 9.3 Arbitration rules for HDL (**B** — behavior inferred, granularity unmeasured)

```
priority: scanout line fetch  >  draw/upload engine  >  (refresh: unknown slot)
```
- The steal happens on a free-running 63.6 µs cadence (captures show it before
  blitter start, i.e. also while idle) (**C**).
- Preemption granularity: does a steal wait for the current 4-px burst? for the
  current row segment? Buffi's 240×64 capture shows clean thick hline blocks
  *between* row segments — suggests row-segment granularity, but **A: measure**
  (probe: does an hline ever split a 256-CLK tile read?).

```systemverilog
// scanout cadence, VRAM domain. Under the §9.1 working assumption (6.4 MHz dot
// = ÷12 enable, HTOTAL = 407) one line = exactly 4884 VRAM CLK: derive the hline
// pulse from the video hcounter itself (same domain, no free divider, no drift).
// If the dot clock hypothesis changes, re-derive; never use an async divider.
// → CDC into VRAM domain. (Getting this wrong accumulates 0.5 CLK/line drift.)
always_ff @(posedge clk_vram) begin
  hline_req <= hline_toggle_sync ^ hline_toggle_sync_d;   // one pulse per line
  if (hline_req) grant_scanout <= 1'b1;                   // engine sees stall
  if (scanout_done) grant_scanout <= 1'b0;
end
```

### 9.4 Scroll registers / double buffering (**C** concept, **A** latch point)

No hardware double buffering: EP1C12 has only ~29 KB of BRAM (a 320×240×16 frame
is 150 KB) and the 8192×4096 map accounts for all 64 MB — scanout reads the same
DDR the blitter writes. Games flip by rewriting SCROLL_X/Y each frame and drawing
into the off-screen buffer; buffers are laid out ≥32 px apart (clip margin).

**A**: when does a SCROLL write take effect — immediately (mid-line tearing
possible), next line, or next frame? Test on PCB: write scroll mid-frame from a
test program (or find a game that does) and observe. Until measured, HDL latches
scroll at line start (safest for artifact-free display, matches typical design).

---

## 10. End-to-end frame timeline (what the whole board does, **C**)

```
IRQ2 (vsync)                                              IRQ2
  │                                                         │
CPU: ──ack──[write 0x08=listN, 0x04=1]──game logic frame N+1──[poll 0x10]──wait──
  │              │                                          │
BLT:   idle      ├─BREQ/BACK 64B chunks (concurrent)        │
                 ├─draw draw draw ... upload ... draw──EXIT─┤→ IRQ1, ready=1
DDR:   ▓ hline every 63.6 µs ▓ interleaved with draw bursts ▓
```
Slowdown rule: if ready=0 (or CPU logic unfinished) at IRQ2 → game waits a whole
extra frame. Our core reproduces PCB slowdown iff (a) draw cost model §6.5,
(b) fetch cadence §5, (c) hline steal §9.2, and (d) SH-3 wait states (outside
this doc) are all right.

---

## 11. Consolidated parameter package

```systemverilog
package cv1k_blit_params;
  // ---- Level C (hard) ----
  localparam real T_VCLK_NS       = 13.0208;   // 76.8 MHz
  localparam real T_CKIO_NS       = 19.5313;   // 51.2 MHz
  localparam int  PX_PER_VCLK     = 4;
  localparam int  CHUNK_BYTES     = 64;
  localparam int  CHUNK_BEATS     = 16;
  localparam int  TILE            = 32;        // px, = 1 DDR row
  // ---- Level B (re-confirm on PCB) ----
  localparam int  P_SRC_ROW_SW    = 6;         // 5 [PDF] / 6 [MAME]
  localparam int  P_RW_TURN       = 20;
  localparam int  P_WR_TURN       = 11;        // 10 [PDF] / 11 [MAME]
  localparam int  P_SPRITE_END    = 12;        // 10 [PDF] / 12 [MAME]
  localparam int  CLIP_MARGIN     = 32;
  localparam int  T_CHUNK_IDLE_NS = 700;       // clipped-op cadence
  localparam int  T_CHUNK_UPLD_NS = 1130;      // upload gap cadence
  localparam int  HLINE_PERIOD_NS = 63586;     // 63600 in [PDF]/[MAME]
  localparam int  HLINE_STEAL_NS  = 2160;
  localparam int  DDR_CL_X2       = 4;         // CL=2.0; decode LMR capture!
  // ---- Level A (placeholders until measured) ----
  localparam int  EXEC_TO_BREQ_CK = 8;         // ??? CKIO cycles
  localparam int  AREF_PER_HLINE  = 8;         // ??? see §6.6
  localparam bit  SCROLL_LATCH_PER_LINE = 1;   // ??? see §9.4
  localparam int  DOT_CLK_HZ      = 6_400_000; // working assumption §9.1 (B)
  localparam int  HTOTAL_DOTS     = 407;       // → 4884 VRAM CLK/line (B)
endpackage
```

---

## 12. PCB measurement checklist (fills every A, settles every B)

Probe points per [PDF] "Method of gathering data": SH3 BREQ/BACK, CS6, D[31:28],
VRAM CLK/CS/RAS/CAS/WE/BA/A, SRAM CS/CLK; plus JAMMA H/V sync and RGB.

| # | Measurement | Resolves |
|---|---|---|
| 1 | Hsync period, width, porches; vsync width; dot clock (RGB pixel edges vs sync) | §9.1 (A) |
| 2 | IRQ2 edge vs. vsync edge; IRQ2 pulse width; effect of 0x24 write on IRQ line | §3, §9.1 (A/B) |
| 3 | CS6 write of 0x04 → BREQ# edge (CKIO cycles) | §5.1 (A) |
| 4 | BREQ→BACK latency histogram in-game | §5.1 (A) |
| 5 | Chunk cadence: idle-op stream vs. upload stream (same LA session) | 700 vs 1130 ns (B) |
| 6 | Command-decode the 2.16 µs hline window: tile count, AREF count, per-tile switch cost | §6.6, §9.2 (A/B) |
| 7 | Does hline steal ever split a tile read/write? | §9.3 granularity (A) |
| 8 | Re-run 8×8 / 16×12 / 240×64 draw captures; fit P_* constants (5/20/10/10 vs 6/20/11/12) | §6.4 (B) |
| 9 | Misaligned-draw sweep X%4 = 0..3, widths 4..64: exact padded-cost law | §6.4 alignment (B) |
| 10 | LMR/EMR decode at boot: CL, burst type; full init sequence | §6.2 (B) |
| 11 | Scroll write mid-frame → observe tear line position | §9.4 (A) |
| 12 | Blend rounding: photograph test-menu blend screens, bit-compare vs §7.4 formulas | §7.4 (B) |
| 13 | Firmware A vs D board (if both available): repeat #8 — do constants differ per firmware? | open question |

---

## 13. Explicit non-goals / D-level notes

- Running the original EP1C12 bitstream: no. We re-implement behavior; firmware
  A–D differences are handled (if ever found) by parameter sets keyed off the
  bitstream checksum the game uploads via Port J (03/3e/f9/e1).
- Internal FPGA pipelining, PLL usage, FIFO sizes beyond observable cadence: free.
- On MiSTer, VRAM lives in DDR3 behind a cache/scheduler; **functional storage is
  decoupled from timing** — the §6.4/§9.2 counters gate the datapath so external
  cadence matches the PCB even though the physical memory is faster.
