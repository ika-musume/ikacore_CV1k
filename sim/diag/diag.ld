/* diag.ld - CV1k diagnostic ROM layout  [H7b.D]
 *
 * Everything is linked at 0xAC000000 (P2 uncached view of the U1 work-RAM
 * SDRAM at phys 0x0C000000).  The image boots from the NOR at 0xA0000000
 * (same bytes, CS0); crt0 is position-independent until its self-copy
 * lands the image here.  Op lists built by C are immediately visible to
 * the blitter's BREQ/BACK fetch (no cache in the loop).
 */
ENTRY(_start)

MEMORY
{
    RAM (rwx) : ORIGIN = 0xAC000000, LENGTH = 4M
}

SECTIONS
{
    .text : {
        KEEP(*(.text.crt0))
        *(.text*)
    } > RAM

    .rodata : { *(.rodata*) } > RAM

    .data : { *(.data*) } > RAM

    __image_start = ORIGIN(RAM);
    __image_end   = .;
    __image_words = (__image_end - __image_start + 3) / 4;

    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    } > RAM
    __bss_words = (__bss_end - __bss_start + 3) / 4;

    _stack_top = ORIGIN(RAM) + 8M;   /* 0xAC800000 = boot SP, P2 view */

    /DISCARD/ : { *(.comment) *(.note*) *(.SH.*) }
}
