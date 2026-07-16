#!/usr/bin/env python3
"""check_emu_ports.py - H7b.1 port-parity check for the MiSTer wrapper.

The wrapper `sim/ikacore_CV1k_emu.sv` takes its port list from
`srcs/sys/emu_ports.vh` via `include - so the LIST cannot drift by
construction.  What can drift:
  * the include line itself getting replaced by a pasted copy;
  * a port the framework expects us to drive being left untouched
    (Verilator flags undriven outputs too, but -Wno-fatal builds scroll
    past warnings - this is the hard gate).

Checks:
  1. the wrapper still does `include "sys/emu_ports.vh"` inside module emu;
  2. every port declared in emu_ports.vh outside inactive `ifdef regions
     (MISTER_FB / MISTER_FB_PALETTE / MISTER_DUAL_SDRAM are NOT defined in
     our build) is referenced somewhere in the wrapper body.

Exit 0 = parity OK.  Run from sim/ (build_emu_lint.sh does).
"""
import re
import sys
from pathlib import Path

SIM = Path(__file__).resolve().parent.parent
PORTS_VH = SIM.parent / "srcs" / "sys" / "emu_ports.vh"
WRAPPER = SIM / "ikacore_CV1k_emu.sv"

# `ifdef groups our Quartus build does NOT define
UNDEFINED = {"MISTER_FB", "MISTER_FB_PALETTE", "MISTER_DUAL_SDRAM"}

PORT_RE = re.compile(
    r"^\s*(input|output|inout)\s+(?:wire|reg|logic)?\s*(?:\[[^\]]+\]\s*)?(\w+)"
)


def parse_ports(text: str):
    """Yield (name, active) for every port; active=False inside an
    `ifdef of an undefined macro."""
    depth_inactive = 0
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("`ifdef"):
            macro = s.split()[1] if len(s.split()) > 1 else ""
            depth_inactive += 1 if (macro in UNDEFINED or depth_inactive) else 0
            continue
        if s.startswith("`ifndef"):
            # none used in emu_ports.vh today; treat as active
            continue
        if s.startswith("`endif"):
            if depth_inactive:
                depth_inactive -= 1
            continue
        m = PORT_RE.match(line)
        if m:
            yield m.group(2), depth_inactive == 0


def main() -> int:
    if not PORTS_VH.exists():
        print(f"[emu-ports] FAIL: {PORTS_VH} not found")
        return 1
    wrapper = WRAPPER.read_text()

    if not re.search(r'`include\s+"sys/emu_ports\.vh"', wrapper):
        print("[emu-ports] FAIL: wrapper does not `include \"sys/emu_ports.vh\"")
        return 1

    ports = list(parse_ports(PORTS_VH.read_text()))
    ident = set(re.findall(r"\w+", wrapper))

    missing = [n for n, active in ports if active and n not in ident]
    guarded = [n for n, active in ports if not active]

    print(f"[emu-ports] {len(ports)} ports in emu_ports.vh "
          f"({len(guarded)} in undefined `ifdef groups: skipped)")
    if missing:
        print("[emu-ports] FAIL: wrapper never references:")
        for n in missing:
            print(f"    {n}")
        return 1
    print("[emu-ports] PASS: every active port is referenced by the wrapper")
    return 0


if __name__ == "__main__":
    sys.exit(main())
