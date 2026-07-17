# CV1k diagnostic ROMs  [H7b.D]
#
# Toolchain split (probed 2026-07-16, blitter_todo H7b.D):
#   compile : sh4-linux-gnu-gcc -m4-nofpu -mb  ->  big-endian SH assembly
#             (the Ubuntu cross package is LE-SH4-Linux, but its compiler
#             stage happily emits -mb code; we never touch its binutils
#             or libgcc - so NO division/modulo in ROM code)
#   assemble: sh-elf-as -isa=sh3  ->  any SH-4-only opcode dies right here
#   link    : sh-elf-ld (big-endian default), diag.ld, -nostdlib
#
# Outputs per ROM: build/<n>.elf/.bin (BE) and build/<n>.hex - the U4
# byte-per-line, pair-swapped, 4 MiB image every sim path consumes
# (vendor ROM_FILE / +norhex / pump NOR-window preload).

CC      = sh4-linux-gnu-gcc
AS      = sh-elf-as
LD      = sh-elf-ld
OBJCOPY = sh-elf-objcopy
OBJDUMP = sh-elf-objdump

CFLAGS  = -m4-nofpu -mb -O2 -ffreestanding -fno-pie -fno-stack-protector \
          -fomit-frame-pointer -Wall -Wextra
ASFLAGS = -isa=sh3
LDFLAGS = -T diag.ld -nostdlib

ROMS = diag_mini diag_1k

all: $(patsubst %,build/%.hex,$(ROMS))

build:
	mkdir -p build

build/%.s: %.c diag.h | build
	$(CC) $(CFLAGS) -S -o $@ $<

build/%.o: build/%.s
	$(AS) $(ASFLAGS) -o $@ $<

build/crt0.o: crt0.s | build
	$(AS) $(ASFLAGS) -o $@ $<

build/%.elf: build/crt0.o build/%.o diag.ld
	$(LD) $(LDFLAGS) -o $@ build/crt0.o build/$*.o
	$(OBJDUMP) -h $@ | sed -n '1,12p'

build/%.bin: build/%.elf
	$(OBJCOPY) -O binary $< $@

# the .raw sibling is the pair-swapped 4 MiB image in raw-dump byte order
# (what the H7b.4 ioctl MRA stream's u4 slot consumes: +ioctl_u4=)
build/%.hex: build/%.bin mkhex.py
	python3 mkhex.py $< $@ build/$*.raw

clean:
	rm -rf build

.PRECIOUS: build/%.s build/%.o build/%.elf build/%.bin
.PHONY: all clean
