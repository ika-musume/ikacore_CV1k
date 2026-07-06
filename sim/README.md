# ikacore_CV1k — board-level SH-3 simulation

A PCB-level simulation of the Cave **CV1000-B** arcade board built to **compare
the ROM execution flow against MAME's SH-3 core**. The SH7709S (the `HS3` IP
core in `ip_cores/`) is wired to its two shared-bus memories using the **real
vendor (NDA) device models**, patched only where Verilator 5 requires it. The
board boots a real game ROM out of flash and streams the retired-instruction
flow for diffing against MAME.

Status: **boots ibara and executes the real program** — 200k+ retired
instructions verified byte-for-byte against the U4 ROM (`0` mismatches).

```
   SH7709S (HS3)  ── shared A/D bus (Table 10.3) ──┬── U4  MX29LV320E  NOR  (area 0 / CS0)
   reset PC A0000000, big-endian                   └── U1  MT48LC2M32B2 SDRAM (area 3 / CS3)
```

## Layout

| Path | What |
|---|---|
| `ikacore_CV1k.sv`            | **PCB top**: HS3 + U4 NOR + U1 SDRAM on the shared bus |
| `tb/tb_cv1k.sv`             | testbench: clock/reset, ROM load, `+dbg` bus probe |
| `tb/cpu_tracer.sv`          | retired-instruction probe, `bind`-ed into `cpu_core` |
| `models/*.v`                | vendor memory models, patched for Verilator |
| `models/*.verilator.patch`  | the exact patch recipe vs the pristine `nda_models/` originals |
| `scripts/compare_flow.py`   | diff RTL vs MAME retired-PC streams → first divergence |
| `scripts/validate_trace.py` | check retired opcodes == U4 ROM content (golden, no MAME needed) |
| `scripts/mame_trace.md`     | how to capture the MAME SH-3 reference trace |
| `rom/`, `build/`            | generated ROM hex + Verilator output (git-ignored) |
| `ip_cores/HS3`              | symlink to the read-only SH-3 IP (see note below) |

## Verified address map — the shared bus (SH7709S HW manual + MAME)

The NOR and SDRAM ride the **same physical address/data pins**, decoded by CSn —
confirmed in **Table 10.3, Physical Address Space Map** (`docs/SH7709S…pdf`):

| Area | CSn | Range | Device | Notes |
|---|---|---|---|---|
| 0 | CS0 | `0x00000000–0x03FFFFFF` | U4 program NOR flash | boot @ 0; ordinary memory / burst ROM |
| 3 | CS3 | `0x0C000000–0x0FFFFFFF` | U1 work-RAM SDRAM (8 MB) | ordinary memory / **synchronous DRAM** |

- Reset PC `0xA0000000` (P2 uncached) → phys `0x00000000` → NOR offset 0. Verified:
  the CPU's first retired instruction is `df3d` = `mov.l @(disp,pc),r15`, loading
  **SP = 0x0C800000** (top of the 8 MB work RAM).
- Area-0 bus width = **16-bit** ⇒ straps **MD4=1, MD3=0** (Table 10.4). NOR on `D[15:0]`.
- SDRAM address multiplex = **AMX 0111** for the MT48LC2M32B2 (Table 10.13/10.14 and
  the only mode the HS3 BSC decodes): `Addr[10:0]=A[12:2]`, `BA[1:0]=A[14:13]`,
  `DQM=WE_n[3:0]`, `Cs=CS3_n Ras=RAS3L_n Cas=CASL_n We=RD_WR Clk=CKIO Cke=CKE`.

## Memory models — vendor models, minimal Verilator patches

The `nda_models/` sign-off models use tristate `Z`, `#delays`, `specify` and
cross-process `disable` that Verilator 5 cannot elaborate. `models/` holds
patched copies; each change is **functional-equivalence only** (no timing/logic
behavior altered) and recorded in the matching `.verilator.patch`:

- **MT48LC2M32B2 (SDRAM)** — canonical single tristate driver + explicit `Dq_in`
  write-data port (Verilator can't read an inout back inside a module); internal
  `z` on masked lanes → 0. The BSC reads with DQM all-low, so unaffected.
- **MX29LV320E (NOR)** — `Q_in` write-data port; `specify`/`specparam` timing
  block → plain `parameter`s (so the procedural checks still resolve `Twc/Tah/…`);
  the `Read_Q`/`read_mode` access one-shots (event + cross-process `disable`)
  rebuilt as request-flag servers with identical restart timing; status-read `z`
  → 0; canonical tristate `Q` driver. **Access times (Taa/Tce/Toe) are the
  datasheet 70/70/30 ns — unchanged.**

> The physical −B U4 is the 2 MB **MX29LV160D**; this board uses its proven-patched
> 4 MB sibling **MX29LV320E** with the 2 MB image mirrored to 4 MB (exactly MAME's
> `ROM_RELOAD`). The area-0 ordinary-memory controller is flash-size-agnostic, so
> the execution flow is identical. MX29LV160D drops in via the same sibling recipe.

### The one board-integration fix that mattered

`i_MEM_READY` **must be tied 0**. The BSC folds it into its generic-completion
term (`gen_ext_done = i_MEM_RSP_VALID | i_MEM_READY`, `bsc.sv:334`), so tying it 1
finishes every area-0 read in a single cycle and samples the bus *before* the
flash drives — the CPU then executes zeros and never boots. With it 0, reads run
the programmed wait-state countdown and latch stable flash data.

## Build & run (Verilator 5)

```
./build_sim.sh                 # build + run ibara, cap 20k retired insns
./build_sim.sh +maxinsn=200000 # longer trace
./build_sim.sh +dbg=1 +cycles=206000   # log the first external bus cycles
BUILD_ONLY=1 ./build_sim.sh    # elaborate only
```

Output: `build/trace_rtl.txt` — one line per retired instruction:
`<pc> <opcode> [; r<n>=<data>]`.

**Golden self-check (no MAME):**
```
python3 scripts/validate_trace.py build/trace_rtl.txt roms/ibara/u4
# -> [validate] N flash-resident retirements checked, 0 mismatches -> PASS
```

## Compare against MAME's SH-3

1. Capture a MAME trace for the same game — see `scripts/mame_trace.md`
   (`mame ibara -debug`, then `trace mame_ibara.tr,0; go`).
2. Diff the two execution flows:
   ```
   python3 scripts/compare_flow.py build/trace_rtl.txt mame_ibara.tr
   ```
   It aligns the retired-PC streams and prints the **first divergence** — the PC
   where the HS3 core and MAME's SH-3 part ways. Early on that is typically the
   first read of a device this board does not model yet (NAND `0x10000000`, YMZ
   `0x10400000`, RTC/EEPROM `0x10c00000`, blitter `0x18000000`, input ports); a
   divergence with identical state up to that point is a genuine core difference.

## Note on the `ip_cores/HS3` symlink

`ip_cores/HS3 → /home/raki/Desktop/HS3/src` is **reference-only**. The build only
**reads** those files; reading through a symlink never modifies the target, and
all Verilator output goes to `sim/build/` (`--Mdir`). Nothing is ever written
under `ip_cores/`.
