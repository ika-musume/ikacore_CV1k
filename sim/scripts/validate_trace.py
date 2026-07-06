#!/usr/bin/env python3
"""
validate_trace.py - independent correctness check for the RTL execution flow.

Every instruction the SH-3 retires out of the program flash (P0/P1/P2 view of
area 0) must equal the actual word stored in the U4 ROM at that address. This
proves the board is executing the *real* game program - a golden check that does
not depend on MAME being installed.

  usage: validate_trace.py <trace.txt> <u4_rom_file>

The ROM is the raw MAME u4 image. The board loads it byte-for-byte into the
flash array; a x16 word read returns {rom[2a+1], rom[2a]} and the SH-3 fetches
it big-endian, so the CPU-visible word at byte offset b is rom[b+1]<<8 | rom[b].
"""
import sys

def main():
    if len(sys.argv) != 3:
        print(__doc__); sys.exit(2)
    trace, romfile = sys.argv[1], sys.argv[2]
    rom = open(romfile, "rb").read()
    mask = len(rom) - 1                       # flash mirrors within its size

    def romword(pc):
        b = (pc & 0x1FFFFFFF) & mask           # P0/P1/P2 -> physical area-0 offset
        return (rom[b + 1] << 8) | rom[b]

    tot = bad = 0
    for ln in open(trace):
        p = ln.split()
        if len(p) < 2:
            continue
        try:
            pc, inst = int(p[0], 16), int(p[1], 16)
        except ValueError:
            continue
        if (pc & 0xF0000000) not in (0xA0000000, 0x00000000, 0x80000000):
            continue                           # only flash-resident code (skip RAM/cache)
        tot += 1
        if romword(pc) != inst:
            bad += 1
            if bad <= 10:
                print(f"  MISMATCH pc={pc:08x} trace={inst:04x} rom={romword(pc):04x}")
    verdict = "PASS" if bad == 0 else "FAIL"
    print(f"[validate] {tot} flash-resident retirements checked, {bad} mismatches -> {verdict}")
    sys.exit(0 if bad == 0 else 1)

if __name__ == "__main__":
    main()
