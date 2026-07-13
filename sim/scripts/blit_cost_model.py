#!/usr/bin/env python3
"""
blit_cost_model.py — CV1000 blitter: original (DDR1/EP1C12) cost model vs.
MiSTer HPS-DDR3 scheduler cost model.

Two implementations, one comparison:

  1. ORIGINAL — Buffi's draw cost formula (blitter_detail.md §6.5), PDF v1.04
     constants (5/20/10/10). Validated against the three LA anchors:
     8x8 = 93 VCLK, 16x12 = 189 VCLK, 240x64@(768,0) = 12090 VCLK.
     This is the same model the RTL pacing FSM implements (§7.6 latency table).

  2. DDR3 SCHEDULER — cost of the same op on the MiSTer DDRAM port (64-bit
     Avalon-MM via f2sdram), calibrated from DE10-nano measurements @125 MHz:
         read  BL=128 random : delay min 15 / avg 17 / max 99 clk, 860 MB/s
         read  BL=128 seq    : delay min 15 / avg 16 / max 85 clk, 872 MB/s
         write BL=1   random : delay min 0  / avg 1  / max 74 clk, 490 MB/s
         write BL=1   seq    : delay min 0  / avg 1  / max 62 clk, 740 MB/s
     Bursts can be stalled mid-flight by waitrequest; the MB/s figures are
     effective rates *including* those stalls. Calibration @125 MHz:
         BL=128 read: 1024 B / 860 MB/s = 1.191 us = 148.8 clk
                    = L_R(17) + 128 * BETA_R  ->  BETA_R = 1.03 clk/word
         BL=1 write seq: 1.35 clk/word accepted (posted).

Clock profiles:
  125.0 MHz — as measured (2026-07 baseline).
  153.6 MHz — 2x the original 76.8 MHz VRAM clock (integer 12.8 MHz family,
              1 VCLK = exactly 2 port clk). MEASURED 2026-07-11 with
              benchmarks/MiSTerDDR3Test-CV1k (M-DDR3-0..4):
        read  BL=128 rnd ser : dly 17/21.4/116, 1031 MB/s (83.9% peak)
        read  BL=8  rnd Pipe8: 1187 MB/s = 96.6% peak (Pipe4: 94.7%)
        write BL>=2 any addr : 1183-1210 MB/s = 96-98% peak, beta_W 1.017
        turnaround (R3)      : ~14 clk per R<->W switch
        tail under HPS load  : 93.4% in 17-24 clk, 2.56% in 65-96,
                               0.0035% >=97, max 165 clk (1.07 us)
              The constant-in-ns latency scaling assumption was confirmed
              (21.4 clk avg vs 21 predicted; max 116 clk ~ 755 ns).

Port-behavior modes (M-DDR3-1 measurement decides which is real):
  SERIAL — one outstanding command: every burst pays full L_R (conservative).
  PIPE   — f2sdram command FIFO queues bursts; one L_R per read train,
           G_CMD per burst (optimistic).

VRAM-in-DDR3 layouts:
  linear  — addr = (Y*8192 + X)*2.  One sprite row = one burst, words ~ W/4.
  stripe4 — addr = ((Y//4)*8192 + X)*8; one 64-bit word = 4 vertically
            adjacent px of one column. One burst covers 4 sprite rows
            (words = W): 4x fewer, 4x longer bursts, same total traffic.
            Partial stripes handled with byte enables (no RMW).

Usage: python3 blit_cost_model.py
"""

import math

# ------------------------------------------------------------------ constants
V_NS = 13.0208          # VRAM CLK (76.8 MHz)

# original per-row penalties: PDF v1.04 set — hits all three anchors exactly
P_PDF = dict(src=5, rw=20, wr=10, spr=10)
P_MAME = dict(src=6, rw=20, wr=11, spr=12)   # alt set, kept for reference

HLINE_PERIOD_NS = 63586.0
HLINE_STEAL_VCLK = 166

PROFILES = {
    "125.0MHz": dict(t_ns=8.0,    l_avg=17, l_max=99,  beta_r=1.03),
    "153.6MHz": dict(t_ns=6.5104, l_avg=21, l_max=165, beta_r=1.016),  # MEASURED
}

BETA_W = 1.017          # clk/word write in a burst (MEASURED, R2)
BETA_W_MEAS = 1.35      # BL=1 sequential rate (legacy sensitivity bound)
C_W = 1                 # per write-burst cmd overhead (measured 0.04; margin)
G_CMD = 1               # per read-burst cmd slot in a PIPE train (meas 0.15-1.6)
T_TURN = 14             # R<->W direction switch (MEASURED, R3: 13.6-14.0)
BL_MAX = 128

DERATE = 1.03           # scanout line fetch + NAND window share of the port


# ------------------------------------------------------- original blitter ---
def vram_rows(x, y, w, h):
    """32x32-tile (=DDR1 row) spans touched — MAME logic, blitter_detail §6.5."""
    xr = 0
    xp = w
    while xp > 0:
        xr += 1
        if (x & 31) + min(xp, 32) > 32:
            xr += 1
        xp -= 32
    n = 0
    yp = h
    while yp > 0:
        n += xr
        if (y & 31) + min(yp, 32) > 32:
            n += xr
        yp -= 32
    return n


def orig_draw_vclk(sx, sy, dx, dy, w, h, P=P_PDF):
    """blitter_detail.md §6.5 draw_cost_vclk. Blend/trans do NOT change cost."""
    src_px = w * h
    dx0 = dx & ~3
    dx1 = (dx + w - 1) | 3
    dst_px = (dx1 - dx0 + 1) * h
    return (src_px // 4 + 2 * (dst_px // 4)
            + vram_rows(sx, sy, w, h) * P['src']
            + vram_rows(dx, dy, w, h) * (P['rw'] + P['wr'])
            + P['spr'])


# ---- op-list fetch model (BREQ/BACK chunk cadence, blitter_detail §5) ----
# The original OVERLAPS fetch and execution: 64-B chunks stream into a FIFO
# (fetch-ahead >= 1 chunk, up to 3 DRAWs per chunk) while the engine draws.
#   op_start = max(engine_free, op_bytes_fetched)
# Fetch binds only for clipped-heavy lists and uploads; draw payload comes
# from VRAM and hides fetch entirely (Level C, [PDF]/[CLIP]).
T_CHUNK_NS   = 700.0     # chunk period, draw/clip stream ([MAME] fit, 17.5us anchor)
T_CHUNK_UPLD = 1442.5    # chunk period during upload payload (312.5ns bus + 1130ns gap)
T_EXEC2BRQ   = 200.0     # EXEC write -> first chunk started (P-22, Level A placeholder)
CHUNK        = 64

def op_stream(ops):
    """ops: list of (kind, w, h, cost_vclk) with kind in draw/clip/upload.
    Returns per-op (bytes, upload_flag, cost_ns)."""
    out = []
    for kind, w, h, cost_vclk in ops:
        if   kind == 'draw':   out.append((20,           False, cost_vclk * V_NS))
        elif kind == 'clip':   out.append((20,           False, 0.0))  # clipped DRAW
        elif kind == 'upload': out.append((16 + 2*w*h,   True,  0.0))  # engine hides
        else: raise ValueError(kind)
    return out

def fetch_ready_times(stream):
    """absolute time each op's last byte is in the FIFO."""
    t, filled, ready = T_EXEC2BRQ, 0, []
    pos = 0
    for nbytes, upld, _ in stream:
        pos += nbytes
        while filled < pos:                      # fetch chunks until op resident
            t += T_CHUNK_UPLD if upld else T_CHUNK_NS
            filled += CHUNK
        ready.append(t)
    return ready

def add_steals(t0, dur_ns):
    """engine-side hline steal: +2.16us at each free-running 63.586us boundary."""
    end = t0 + dur_ns
    k = int(t0 // HLINE_PERIOD_NS) + 1
    while k * HLINE_PERIOD_NS < end:
        end += HLINE_STEAL_VCLK * V_NS
        k += 1
    return end

def frame_ns(stream, ready, engine_costs_ns, steals=True):
    """two-process coupling: op_start = max(engine_free, fetch_ready)."""
    engine_free = 0.0
    for (nbytes, upld, _), rdy, cost in zip(stream, ready, engine_costs_ns):
        start = max(engine_free, rdy)
        engine_free = add_steals(start, cost) if steals else start + cost
    return engine_free

def orig_frame_ns(ops):
    """ops as in op_stream(). Original: engine costs + steals + fetch coupling."""
    stream = op_stream(ops)
    return frame_ns(stream, fetch_ready_times(stream),
                    [c for _, _, c in stream], steals=True)


# --------------------------------------------------------- DDR3 scheduler ---
def words_linear(x, w):
    """64-bit words covering pixel span [x, x+w) on a linear row."""
    b0 = (x * 2) // 8
    b1 = ((x + w) * 2 - 1) // 8
    return b1 - b0 + 1


def stripes(y, h):
    """number of 4-row stripes covering [y, y+h)."""
    return (y + h - 1) // 4 - y // 4 + 1


def burst_split(n):
    """split an n-word run into BL<=BL_MAX bursts -> list of burst lengths."""
    out = []
    while n > 0:
        out.append(min(n, BL_MAX))
        n -= BL_MAX
    return out


def op_bursts(sx, sy, dx, dy, w, h, blend, layout):
    """(read_bursts, write_bursts) word-count lists for one draw op."""
    rd, wr = [], []
    if layout == 'linear':
        ws, wd = words_linear(sx, w), words_linear(dx, w)
        for _ in range(h):
            rd += burst_split(ws)
            if blend:
                rd += burst_split(wd)
            wr += burst_split(wd)
    elif layout == 'stripe4':
        for _ in range(stripes(sy, h)):
            rd += burst_split(w)
        for _ in range(stripes(dy, h)):
            if blend:
                rd += burst_split(w)
            wr += burst_split(w)
    else:
        raise ValueError(layout)
    return rd, wr


def ddr3_draw_clk(prof, sx, sy, dx, dy, w, h, blend, mode, layout,
                  beta_w=BETA_W, l_override=None):
    """DDR3 port occupancy (port clk) for one draw op.

    blend=True  : src read + dst read + write   (original semantics)
    blend=False : src read + BE-masked write    (dst-read skip fast path)
    """
    l_r = PROFILES[prof]['l_avg'] if l_override is None else l_override
    rd, wr = op_bursts(sx, sy, dx, dy, w, h, blend, layout)

    beta_r = PROFILES[prof]['beta_r']
    if mode == 'SERIAL':
        t_rd = sum(l_r + math.ceil(n * beta_r) for n in rd)
    elif mode == 'PIPE':
        t_rd = l_r + math.ceil(sum(rd) * beta_r) + len(rd) * G_CMD
    else:
        raise ValueError(mode)

    t_wr = math.ceil(sum(wr) * beta_w) + len(wr) * C_W
    return t_rd + t_wr + 2 * T_TURN


# ------------------------------------------------------------- validation ---
def validate():
    anchors = [
        ("8x8 aligned",   orig_draw_vclk(0, 0, 0, 0, 8, 8),        93),
        ("16x12 aligned", orig_draw_vclk(0, 0, 0, 0, 16, 12),      189),
        ("240x64@768,0",  orig_draw_vclk(768, 0, 768, 0, 240, 64), 12090),
    ]
    print("== ORIGINAL model validation (PDF v1.04 constants) ==")
    ok = True
    for name, got, want in anchors:
        mark = "OK " if got == want else "FAIL"
        ok &= (got == want)
        print(f"  [{mark}] {name:16s} model={got:6d} VCLK  anchor={want}"
              f"  ({got*V_NS/1000:.3f} us)")
    if not ok:
        raise SystemExit("anchor mismatch — model broken")

    # fetch-bound anchors (the pure engine-sum model cannot reproduce these)
    t_clip = orig_frame_ns([('clip', 1, 324, 0)] * 80)
    t_upld = orig_frame_ns([('upload', 256, 5, 0)])
    for name, got_us, want_us, tol in (
            ("80x clipped 1x324 (fetch-bound)", t_clip/1000, 17.5, 0.03),
            ("upload 256x5 (bus-bound)",        t_upld/1000, 58.77, 0.03)):
        mark = "OK " if abs(got_us - want_us) <= want_us * tol else "FAIL"
        ok &= (mark == "OK ")
        print(f"  [{mark}] {name:32s} model={got_us:6.2f} us  anchor={want_us} us")
    if not ok:
        raise SystemExit("fetch anchor mismatch — model broken")
    print()


# ------------------------------------------------------------- comparison ---
CASES = [
    # (label, w, h, sx, sy, dx, dy)   unaligned = +1 px x, +2 px y offsets
    ("8x8    al", 8,   8,   0, 0, 0, 0),
    ("8x8    un", 8,   8,   1, 2, 1, 2),
    ("16x16  al", 16,  16,  0, 0, 0, 0),
    ("16x16  un", 16,  16,  1, 2, 1, 2),
    ("32x32  al", 32,  32,  0, 0, 0, 0),
    ("32x32  un", 32,  32,  1, 2, 1, 2),
    ("64x64  un", 64,  64,  1, 2, 1, 2),
    ("240x64 al", 240, 64,  768, 0, 768, 0),
    ("320x240un", 320, 240, 1, 2, 1, 2),
]

CONFIGS = [
    ("SERIAL", "linear"),
    ("SERIAL", "stripe4"),
    ("PIPE",   "linear"),
    ("PIPE",   "stripe4"),
]


def v2d(prof):
    return V_NS / PROFILES[prof]['t_ns']


def per_op_table(prof, blend):
    tag = 'ON (src+dst rd+wr)' if blend else 'OFF (dst-read skip, BE writes)'
    print(f"== [{prof}] per-op cost, blend={tag} ==")
    print("   margin = (orig - ddr3)/orig ; negative = DDR3 SLOWER than original\n")
    hdr = f"  {'case':10s} {'orig clk':>9s}"
    for m, l in CONFIGS:
        hdr += f"  {m[0]}-{l[:3]:>4s}"
    print(hdr)
    worst = {c: 1.0 for c in CONFIGS}
    for label, w, h, sx, sy, dx, dy in CASES:
        oc = orig_draw_vclk(sx, sy, dx, dy, w, h) * v2d(prof)
        row = f"  {label:10s} {oc:9.0f}"
        for cfg in CONFIGS:
            m, l = cfg
            dc = ddr3_draw_clk(prof, sx, sy, dx, dy, w, h, blend, m, l)
            marg = (oc - dc) / oc
            worst[cfg] = min(worst[cfg], marg)
            row += f"  {marg:+6.0%}"
        print(row)
    print("  " + "-" * 60)
    row = f"  {'WORST':10s} {'':9s}"
    for cfg in CONFIGS:
        row += f"  {worst[cfg]:+6.0%}"
    print(row + "\n")
    return worst


def spike_check(prof):
    """Can one worst-case latency spike eat an op's margin?"""
    p = PROFILES[prof]
    print(f"== [{prof}] single max-latency spike (+{p['l_max']-p['l_avg']} clk) absorption, blend ON ==")
    for label, w, h, sx, sy, dx, dy in [CASES[1], CASES[3]]:   # 8x8un, 16x16un
        oc = orig_draw_vclk(sx, sy, dx, dy, w, h) * v2d(prof)
        for m, l in [("PIPE", "linear"), ("PIPE", "stripe4")]:
            base = ddr3_draw_clk(prof, sx, sy, dx, dy, w, h, True, m, l)
            spike = base + (p['l_max'] - p['l_avg'])
            verdict = 'ABSORBED in-op' if spike <= oc else 'needs accumulated slack'
            print(f"  {label:10s} {m}/{l:7s}: base {base:5.0f}  +spike {spike:5.0f}"
                  f"  orig {oc:5.0f}  -> {verdict}")
    print()


def slow_write_sensitivity(prof):
    """What if burst writes are no better than measured BL=1 (1.35 clk/word)?"""
    print(f"== [{prof}] write-rate sensitivity (BETA_W = {BETA_W_MEAS} = measured BL=1 seq), blend ON ==")
    for label, w, h, sx, sy, dx, dy in [CASES[3], CASES[7]]:
        oc = orig_draw_vclk(sx, sy, dx, dy, w, h) * v2d(prof)
        for m, l in [("PIPE", "linear"), ("PIPE", "stripe4")]:
            dc = ddr3_draw_clk(prof, sx, sy, dx, dy, w, h, True, m, l,
                               beta_w=BETA_W_MEAS)
            print(f"  {label:10s} {m}/{l:7s}: {dc:6.0f} clk vs orig {oc:6.0f}"
                  f"  margin {(oc-dc)/oc:+6.0%}")
    print()


def workload_table(prof):
    print(f"== [{prof}] frame workloads — two-process model: "
          f"op_start = max(engine_free, fetch_ready) ==")
    print(f"   (original incl. hline steals; DDR3 incl. {DERATE:.0%}-100% "
          f"scanout+NAND derate; both consume the same BREQ chunk stream)\n")
    D = lambda w, h, blend: ('draw', 1, 2, 1, 2, w, h, blend)
    workloads = {
        "danmaku: 1 bg 320x240 + 400 bullets 16x16(un,blend) + 60 obj 32x32(un)":
            [D(320, 240, False)] + [D(16, 16, True)] * 400 + [D(32, 32, True)] * 60,
        "heavy: 2 bg 320x240(blend) + 800 bullets 16x16(un,blend)":
            [D(320, 240, True)] * 2 + [D(16, 16, True)] * 800,
        "clip-heavy: 200 bullets 16x16 + 800 fully-clipped (offscreen)":
            [D(16, 16, True)] * 200 + [('clip', 0, 0, 0, 0, 1, 324, False)] * 800,
        "anchor op alone: 240x64":
            [('draw', 768, 0, 768, 0, 240, 64, True)],
    }
    t_ns = PROFILES[prof]['t_ns']
    for name, wl in workloads.items():
        stream_ops = [(k, w, h,
                       orig_draw_vclk(sx, sy, dx, dy, w, h) if k == 'draw' else 0)
                      for k, sx, sy, dx, dy, w, h, b in wl]
        stream = op_stream(stream_ops)
        ready  = fetch_ready_times(stream)
        o_ns = frame_ns(stream, ready, [c for _, _, c in stream], steals=True)
        print(f"  {name}")
        print(f"    original      : {o_ns/1000:9.1f} us"
              f"  ({o_ns/16666666*100:5.1f}% of a frame)")
        for m, l in CONFIGS:
            d_costs = [ddr3_draw_clk(prof, sx, sy, dx, dy, w, h, b, m, l)
                       * t_ns * DERATE if k == 'draw' else 0.0
                       for k, sx, sy, dx, dy, w, h, b in wl]
            d_ns = frame_ns(stream, ready, d_costs, steals=False)
            marg = (o_ns - d_ns) / o_ns
            print(f"    {m:6s}/{l:7s}: {d_ns/1000:9.1f} us   margin {marg:+6.1%}")
        print()


if __name__ == "__main__":
    validate()
    for prof in ("153.6MHz", "125.0MHz"):
        per_op_table(prof, blend=True)
        per_op_table(prof, blend=False)
        spike_check(prof)
        slow_write_sensitivity(prof)
        workload_table(prof)
        print("#" * 72 + "\n")
