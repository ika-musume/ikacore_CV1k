#!/usr/bin/env python3
"""
compare_flow.py - diff the RTL SH-3 execution flow against a MAME SH-3 trace.

  usage: compare_flow.py <rtl_trace.txt> <mame_trace.txt>

RTL trace  (build/trace_rtl.txt, from cpu_tracer.sv):
    a0000000 df3d ; r23=0c800000
    ^PC      ^op
MAME trace (from the debugger `trace` command, see mame_trace.md):
    A0000000: mov.l   @($3D,PC),R15
    ^PC

The common, tool-independent key is the retired-PC stream (the control-flow
path). We align the two PC sequences and report the first divergence - the exact
instruction where the HS3 core and MAME's SH-3 part ways. That point is either an
HS3 bug or (more often early in boot) the first read of a peripheral this board
does not model yet (NAND / YMZ / RTC / blitter / input port); the printed context
tells you which.

Exit status 0 if the streams match for the whole overlap, 1 on divergence.
"""
import re, sys

MAME_RE = re.compile(r'^\s*([0-9A-Fa-f]{2,8}):')     # "A0000000: mov.l ..."

def load_pcs(path, is_mame):
    pcs = []
    for ln in open(path):
        if is_mame:
            m = MAME_RE.match(ln)              # skips loop markers / blank lines
            if m:
                pcs.append(int(m.group(1), 16))
        else:
            p = ln.split()                    # "a0000000 df3d ; ..."
            if p:
                try:
                    pcs.append(int(p[0], 16))
                except ValueError:
                    pass
    return pcs

def main():
    if len(sys.argv) != 3:
        print(__doc__); sys.exit(2)
    rtl = load_pcs(sys.argv[1], is_mame=False)
    mame = load_pcs(sys.argv[2], is_mame=True)
    n = min(len(rtl), len(mame))
    print(f"[compare] RTL={len(rtl)} PCs  MAME={len(mame)} PCs  overlap={n}")

    for i in range(n):
        if rtl[i] != mame[i]:
            lo = max(0, i - 4)
            print(f"\n*** FIRST DIVERGENCE at retired instruction #{i} ***")
            print(f"    {'idx':>6}  {'RTL':>8}  {'MAME':>8}")
            for j in range(lo, min(n, i + 3)):
                mark = "  <-- diverge" if j == i else ""
                print(f"    {j:6d}  {rtl[j]:08x}  {mame[j]:08x}{mark}")
            print(f"\n  {i} instructions matched before divergence.")
            sys.exit(1)

    print(f"[compare] MATCH: all {n} overlapping retired PCs identical.")
    if len(rtl) != len(mame):
        print(f"  (streams differ in length; compare longer/equal caps to extend)")
    sys.exit(0)

if __name__ == "__main__":
    main()
