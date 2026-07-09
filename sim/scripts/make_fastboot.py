#!/usr/bin/env python3
"""
Generate the ibara "fast-boot" simulation artifacts.

Reads the pristine U4 NOR image (roms/ibara/u4, 2 MB, byte-swapped-per-16bit as
the SH-3 fetches it big-endian), applies three NOP patches that remove the boot
loops unnecessary for RTL simulation, and emits:

  roms/ibara_patched/ibara_u4_4M_fastboot.hex   patched flash image, 4 MB mirror, 1 byte/line
  roms/ibara_patched/ibara_sdram_bank0.hex      SDRAM bank0 preload (P = 0x000000..0x1FFFFC)
  roms/ibara_patched/ibara_sdram_bank1.hex      SDRAM bank1 preload (P = 0x200000..0x3950B8)
  roms/ibara_patched/u4_fastboot         patched raw 2 MB binary (reference)

Patches (CPU addr -> NOP 0x0009; file stores each 16-bit word byte-swapped):
  0x000092..0x00009a  flash->SDRAM copy loop  (5 insns)  -> SDRAM is preloaded
  0x049f64            jsr FUN_0c04a250 (FPGA bitstream upload) -> RTL is preconfigured
  0x049f74            bf/s of the 0x40000-iter FPGA settle delay loop
The blitter-ready poll at 0c049f86 is intentionally KEPT (waits for the RTL
blitter to leave NOT-READY after its DDR init).

SDRAM layout (HS3 BSC, AMX=0111, verified from RTL):
  bank  = (P >> 21) & 3          # each 2 MB chunk is one bank
  index = (P >>  2) & 0x7FFFF    # linear 32-bit word index within the bank
  word(P) = big-endian longword the CPU would have copied there
The program spans 0..0x3950BC, so only bank0 (full) + bank1 (partial) are used.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SIM  = os.path.dirname(HERE)
SRC  = os.path.join(SIM, "roms/ibara/u4")

NOP_SITES = [0x92, 0x94, 0x96, 0x98, 0x9a,   # copy loop
             0x49f64,                        # FPGA bitstream upload jsr
             0x49f74]                        # FPGA settle delay bf/s

COPY_BYTES = 0x3950BC          # what the boot copy loop transfers (== program size)

def be_longword(fm, p):
    """CPU big-endian 32-bit longword at byte addr p, from byte-swapped file fm."""
    return (fm[p+1] << 24) | (fm[p] << 16) | (fm[p+3] << 8) | fm[p+2]

def main():
    rom = bytearray(open(SRC, "rb").read())
    assert len(rom) == 0x200000, f"expected 2 MB u4, got {len(rom):#x}"

    # --- verify + apply NOP patches ------------------------------------
    expect = {0x92:0x6016,0x94:0x4310,0x96:0x2202,0x98:0x8ffb,0x9a:0x7204,
              0x49f64:0x400b, 0x49f74:0x8ffc}
    for off in NOP_SITES:
        cpu = (rom[off+1] << 8) | rom[off]
        assert cpu == expect[off], f"@{off:#x}: got {cpu:#06x} want {expect[off]:#06x}"
        rom[off]   = 0x09         # NOP = 0x0009, stored byte-swapped -> 09 00
        rom[off+1] = 0x00
    print(f"patched {len(NOP_SITES)} sites -> NOP")

    # --- patched raw binary -------------------------------------------
    open(os.path.join(SIM, "roms/ibara_patched/u4_fastboot"), "wb").write(rom)

    # --- flash hex: 4 MB (2 MB mirrored), one byte per line -----------
    with open(os.path.join(SIM, "roms/ibara_patched/ibara_u4_4M_fastboot.hex"), "w") as f:
        for i in range(0x400000):
            f.write(f"{rom[i & 0x1FFFFF]:02x}\n")
    print("wrote roms/ibara_patched/ibara_u4_4M_fastboot.hex (4 MB)")

    # --- SDRAM bank preloads ------------------------------------------
    # flash mirror the copy loop reads: fm[P] = rom[P % 0x200000]
    def fmbyte(p): return rom[p & 0x1FFFFF]
    fm = [0]*4  # tiny helper via closure below
    def word(p):
        return ((fmbyte(p+1) << 24) | (fmbyte(p) << 16) |
                (fmbyte(p+3) << 8)  |  fmbyte(p+2))

    bank_words = {0: [], 1: []}
    for p in range(0, COPY_BYTES, 4):
        bank = (p >> 21) & 3
        bank_words[bank].append(word(p))
    for b in (0, 1):
        path = os.path.join(SIM, f"roms/ibara_patched/ibara_sdram_bank{b}.hex")
        with open(path, "w") as f:
            for w in bank_words[b]:
                f.write(f"{w:08x}\n")
        print(f"wrote roms/ibara_patched/ibara_sdram_bank{b}.hex ({len(bank_words[b])} words)")

if __name__ == "__main__":
    main()
