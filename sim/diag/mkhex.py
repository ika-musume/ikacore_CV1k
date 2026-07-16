#!/usr/bin/env python3
"""mkhex.py - CV1k diag ROM image packer  [H7b.D]

Input: the big-endian flat binary from sh-elf-objcopy.
Output: a U4-shaped hex (one byte per line, 4 MiB) in the MAME-dump byte
order the whole sim stack consumes: file byte order is LOW byte first
(even byte -> flash DQ[7:0]), i.e. instruction 0xDF3D is stored as
"3d\ndf\n" - identical to roms/ibara_patched/*.hex, so ROM_FILE /
+norhex / the pump NOR-window preload all take it unmodified.

usage: mkhex.py in.bin out.hex [out.swapped.bin]
"""
import sys


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    data = bytearray(open(sys.argv[1], "rb").read())
    if len(data) & 1:
        data.append(0xFF)
    if len(data) > 4 * 1024 * 1024:
        print(f"mkhex: image {len(data)} bytes > 4 MiB")
        return 1

    out = bytearray(len(data))
    out[0::2] = data[1::2]          # low byte of each halfword first
    out[1::2] = data[0::2]
    out += b"\xff" * (4 * 1024 * 1024 - len(out))   # erased-flash fill

    with open(sys.argv[2], "w") as f:
        f.write("\n".join(f"{b:02x}" for b in out))
        f.write("\n")
    if len(sys.argv) > 3:
        open(sys.argv[3], "wb").write(out)
    print(f"mkhex: {sys.argv[1]} ({len(data)} bytes) -> {sys.argv[2]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
