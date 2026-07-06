# Producing the MAME SH-3 reference trace (for compare_flow.py)

The board sim emits the HS3 core's retired-instruction stream to
`build/trace_rtl.txt`. To compare it against MAME's SH-3, capture MAME's
own instruction trace for the same game and diff the two PC streams.

## 1. MAME with the cv1k driver

Any MAME with the `cave/cv1k.cpp` driver works (>= 0.2xx; Debian ships 0.285:
`apt install mame`). CV1000 boots the SH7709S from U4 exactly like this board,
big-endian, reset PC `0xA0000000`, so the retired-PC streams line up 1:1.

## 2. ROM set

MAME loads the same U4/U2/U23/U24 dumps that live in `sim/roms/<game>/`. Zip
them under the MAME set name (here `ibara`) into your MAME `roms/` dir, e.g.:

    cd sim/roms/ibara
    zip ~/mame/roms/ibara.zip u2 u4 u23 u24
    # if MAME rejects a name/CRC, run `mame -listxml ibara` and rename to match

## 3. Capture the trace from the debugger

    mame ibara -debug -window

In the debugger console (CPU 0 is the SH-3 "maincpu"):

    trace mame_ibara.tr,0        ; log every retired instruction of cpu 0
    go                           ; run; let it boot a while
    ... (break: Ctrl-\ or a wp)  ; stop once you have enough
    traceflush                   ; flush the file
    trace off,0

`mame_ibara.tr` now holds lines like:

    A0000000: mov.l   @($3D,PC),R15
    A0000002: mov.l   @($3E,PC),R0
    ...

Headless variant (no window), auto-stop after N seconds:

    mame ibara -debug -debugscript trace.cmd
    # trace.cmd:
    #   trace mame_ibara.tr,0
    #   go
    # then kill after a few seconds; the .tr is written incrementally

## 4. Compare

    # RTL side (this repo):
    ./build_sim.sh +maxinsn=200000
    # diff the two execution flows:
    python3 scripts/compare_flow.py build/trace_rtl.txt mame_ibara.tr

`compare_flow.py` aligns the retired-PC sequences and prints the first PC where
the HS3 core and MAME's SH-3 diverge.

## Interpreting a divergence

- **Early boot, at a load/store PC** whose target is a device this board does
  not model yet (NAND `0x10000000`, YMZ770 `0x10400000`, serial RTC/EEPROM
  `0x10c00000`, blitter `0x18000000`, or an input port): expected. The two cores
  read different values from the unmodeled device and branch apart. Model that
  device (or stub the value MAME returns) to push the match further.
- **A divergence with all operands identical up to that point**: a genuine HS3
  vs MAME difference - an SH-3 core bug on one side. This is the signal we want.
- MAME's timing model is not cycle-accurate to the PCB, so compare the
  *instruction order* (control flow), not cycle counts.
