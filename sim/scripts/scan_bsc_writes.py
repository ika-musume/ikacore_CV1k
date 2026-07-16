#!/usr/bin/env python3
"""Scan CV1k u4 program ROMs for references to the SH-3 BSC's SDRAM/bus
config registers, to check whether anything could reprogram the SDRAM
(burst length, CAS latency, refresh, waits) after boot.

SH-3 code can only materialize a 32-bit I/O address via a mov.l @(disp,PC)
literal pool entry, so every reachable register write leaves the register's
address as a 4-byte constant in the image.  We scan for those constants and
report where they sit.

The u4 dumps are 16-bit byte-swapped (MAME convention); the scanner
un-swaps to the CPU's byte-addressable view first.  The CPU is big-endian
and mov.l literal pools are 4-byte aligned (calibrated on the boot block's
SDMR constant 0xFFFFE880 @ 0x13C), so only aligned big-endian matches are
real pool entries.  Register writes can also go through a base pointer
(mov.l Rm,@(disp,Rn), disp <= 0x3C), so any aligned constant landing in
0xFFFFFF44-0xFFFFFF80 is reported as a potential BSC/FRQCR base.

Caveat (static analysis): ibara's u4 is copied 1:1 to SDRAM and executed,
so this scan covers all gameplay code.  ddpsdoj carries its bulk
compressed (~2.6 MB unpacked by a loader), so constants inside the packed
region are invisible here; only its plain stage-1/loader region (first
~0x51000 bytes) is fully covered.

Usage: scan_bsc_writes.py <u4-image> [<u4-image> ...]
"""

import struct
import sys

# SH7709-class BSC / clock register ranges (name, first, last inclusive).
# BSC-base covers reachable-by-displacement pointers: a base B reaches
# B..B+0x3C, so any constant in [MCR-0x3C, FRQCR] can address MCR/RTCSR/
# RTCOR/FRQCR through mov.l Rm,@(disp,Rn).
REGS = [
    ("BSC-base", 0xFFFFFF44, 0xFFFFFF80),  # BCR/WCR/MCR/RTC*/RFCR/FRQCR window
    ("SDMR",     0xFFFFD000, 0xFFFFEFFF),  # mode-register write window:
                                           # the mode word (BL/CL) is in the address
]

# Boot-time BSC init block: everything below this file offset is the known
# reset-vector init path (verified byte-identical across games/boards).
BOOT_END = 0x1000


def unswap(raw: bytes) -> bytes:
    """MAME u4 dumps store each 16-bit unit byte-swapped; undo it."""
    b = bytearray(raw)
    b[0::2], b[1::2] = raw[1::2], raw[0::2]
    return bytes(b)


def scan(mem: bytes):
    """Every 4-aligned big-endian word that falls in a watched range."""
    hits = []
    for off in range(0, len(mem) - 3, 4):
        (word,) = struct.unpack_from(">I", mem, off)
        for name, lo, hi in REGS:
            if lo <= word <= hi:
                hits.append((off, name, word))
    return hits


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for path in sys.argv[1:]:
        with open(path, "rb") as f:
            mem = unswap(f.read())
        hits = scan(mem)
        boot = [h for h in hits if h[0] < BOOT_END]
        late = [h for h in hits if h[0] >= BOOT_END]
        print(f"\n=== {path} ({len(mem)//1024} KB) ===")
        print(f"  boot-block hits (< 0x{BOOT_END:X}): {len(boot)}")
        for off, name, addr in boot:
            print(f"    0x{off:06X}  {name:8s} 0x{addr:08X}")
        print(f"  hits beyond the boot block: {len(late)}")
        for off, name, addr in late:
            print(f"    0x{off:06X}  {name:8s} 0x{addr:08X}"
                  f"  <-- possible runtime reconfig, inspect!")
        if not boot:
            print("  WARNING: no boot-block hits - scanner self-check FAILED"
                  " (wrong image/byte order?)")
        if not late:
            print("  -> no SDRAM/BSC register reference outside boot init")


if __name__ == "__main__":
    main()
