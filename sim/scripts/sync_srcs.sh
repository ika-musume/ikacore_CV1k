#!/usr/bin/env bash
# Sync the Quartus project tree (srcs/) from the sim/ dev tree.  [H7b.8]
#
# sim/ is the source of truth for ALL core RTL - the srcs/rtl copies are
# never edited by hand.  Re-run this after any sim-side RTL change, then
# recompile the Quartus project.  (SH3Test precedent: the HS3 IP ships as
# a dereferenced local copy so the release repo is self-contained.)
#
# Layout produced:
#   srcs/ikacore_CV1k_emu.sv <- sim/ikacore_CV1k_emu.sv (module emu, the
#                             MiSTer glue file; NOT named <core>.sv - that
#                             basename belongs to the portable core top and
#                             the collision confuses tool module searches)
#   srcs/rtl/ikacore_CV1k.sv                            (portable core top)
#   srcs/rtl/CV1k_*.{sv,v}                              (board layer)
#   srcs/rtl/CV1k_blit/*.sv                             (blitter core)
#   srcs/rtl/HS3/**                                     (SH-3 IP, cp -L)
#
# NOT synced (Quartus-owned): srcs/ikacore_CV1k.{qpf,qsf,sdc,srf},
# srcs/files.qip, srcs/rtl/pll* (hand-authored IP), srcs/sys/ (framework),
# srcs/releases/.
set -euo pipefail
cd "$(dirname "$0")/../.."          # repo root

SIM=sim
DST=srcs

mkdir -p "$DST/rtl/CV1k_blit" "$DST/rtl/HS3/cpu_core" "$DST/rtl/HS3/peri"

# emu glue at the project root (same basename as in sim/)
cp "$SIM/ikacore_CV1k_emu.sv" "$DST/ikacore_CV1k_emu.sv"

# portable core top + board layer
for f in ikacore_CV1k.sv CV1k_cpld.v CV1k_sdram_control.sv \
         CV1k_ddr3_harness.sv CV1k_nand.sv CV1k_ioctl.sv; do
    cp "$SIM/$f" "$DST/rtl/$f"
done

# blitter core
cp "$SIM"/CV1k_blit/blit_{regs,fetch,gov,draw,batch,video,top}.sv \
   "$DST/rtl/CV1k_blit/"

# HS3 SH-3 IP (dereference the sim/ip_cores symlink; RTL only, no obj_dir)
cp -L "$SIM"/ip_cores/HS3/HS3.sv                "$DST/rtl/HS3/"
cp -L "$SIM"/ip_cores/HS3/cpu_core/*.sv         "$DST/rtl/HS3/cpu_core/"
cp -L "$SIM"/ip_cores/HS3/peri/*.sv             "$DST/rtl/HS3/peri/"

echo "[sync_srcs] srcs/ synced from sim/ ($(git -C . rev-parse --short HEAD 2>/dev/null || echo no-git))"
