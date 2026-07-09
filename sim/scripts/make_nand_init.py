#!/usr/bin/env python3
"""
Generate the U2 NAND preload image ($readmemh format) for the Verilator sim.

Source:  sim/roms/ibara/u2         - raw K9F1G08U0M dump, 65536 pages of
                                      2112 bytes (2048 main + 64 spare), in
                                      linear page order (block*64 + page).
Output:  sim/roms/ibara_patched/ibara_u2.8.init

The patched Micron model (models/MT29F1G08ABAFA, x8, NUM_COL=2112) stores a
page in one wide reg `mem_array[row]` of PAGE_SIZE = 2112*8 = 16896 bits, with
column `col` occupying bits [col*8 +: 8] - i.e. column 0 is the LSB. Verilog
`$readmemh` parses each token MSB-first and drops one value per memory index,
so each output line must be the page's 2112 bytes in REVERSE order (column
2111 first ... column 0 last), 4224 hex chars, one line per page (row).

  row N in the file  ==  mem_array index N  ==  NAND row/page address N

The model's array holds only NUM_ROW rows, and `$readmemh` must not be handed
more lines than that, so two images are produced:

  ibara_u2.8.init       full 65536-page image (~277 MB) - needs +define+FullMem;
                        correct but Verilator is very slow on the 138 MB array.
  ibara_u2_boot.8.init  first BOOT_PAGES rows - what the sim loads by default
                        (+define+NAND_ROWS=<n> must equal this line count).

Usage:
  scripts/make_nand_init.py                    # both images (boot slice = 1024 pages)
  scripts/make_nand_init.py --boot-pages 4096  # bigger boot slice
  scripts/make_nand_init.py --pages 256 --dst /tmp/x.init   # one-off slice
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM  = os.path.dirname(HERE)
SRC  = os.path.join(SIM, "roms/ibara/u2")
DST  = os.path.join(SIM, "roms/ibara_patched/ibara_u2.8.init")
BOOT = os.path.join(SIM, "roms/ibara_patched/ibara_u2_boot.8.init")

PAGE_BYTES = 2112          # 2048 main + 64 spare (K9F1G08U0M / NUM_COL=2112)
NUM_PAGES  = 65536         # 1 Gbit = 1024 blocks * 64 pages
BOOT_PAGES = 1024          # rows the sim preloads by default (must == NAND_ROWS)


def emit(src, dst, pages):
    """Write `pages` rows of `src` to `dst` in $readmemh form (col0 = LSB)."""
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    n = 0
    with open(src, "rb") as fi, open(dst, "w") as fo:
        for _ in range(pages):
            page = fi.read(PAGE_BYTES)
            if len(page) < PAGE_BYTES:
                break
            # column 0 = LSB -> emit column 2111..0  (reverse byte order)
            fo.write(page[::-1].hex())
            fo.write("\n")
            n += 1
    print(f"wrote {dst}\n  {n} pages x {PAGE_BYTES} B  ->  "
          f"{os.path.getsize(dst)/1e6:.1f} MB ({PAGE_BYTES*2} hex chars/line)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pages", type=int, default=NUM_PAGES,
                    help="rows for the full image (default: all 65536)")
    ap.add_argument("--boot-pages", type=int, default=BOOT_PAGES,
                    help=f"rows for the boot slice (default: {BOOT_PAGES})")
    ap.add_argument("--src", default=SRC)
    ap.add_argument("--dst", default=DST)
    ap.add_argument("--boot-dst", default=BOOT)
    args = ap.parse_args()

    size = os.path.getsize(args.src)
    exp  = NUM_PAGES * PAGE_BYTES
    if size != exp:
        print(f"[warn] {args.src} is {size:#x} bytes, expected {exp:#x} "
              f"({NUM_PAGES} pages x {PAGE_BYTES})", file=sys.stderr)
    avail = size // PAGE_BYTES

    emit(args.src, args.dst, min(args.pages, avail))
    if args.dst == DST:            # only emit the slice for a default full run
        emit(args.src, args.boot_dst, min(args.boot_pages, avail))


if __name__ == "__main__":
    main()
