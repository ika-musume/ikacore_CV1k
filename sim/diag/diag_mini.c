/* diag_mini.c - CV1k boot-path smoke ROM  [H7b.D]
 *
 * Exercises every leg the real boot uses, in ~1-2 M CKIO instead of the
 * 22 M-insn game boot: CS0 NOR fetch -> BSC init -> NOR->SDRAM self-copy
 * (crt0) -> execute from work RAM -> CS6 register writes -> op-list build
 * in SDRAM -> EXEC -> BREQ/BACK list fetch -> UPLOAD + DRAW through the
 * pixel pipe -> STATUS ready poll -> vblank pacing off the INTC IRR0
 * edge latch (ICR1 falling-edge config, SR-masked polling).
 *
 * Screen: four white 32x32 corner squares + one square sweeping the
 * middle row, on black.  Fully deterministic - the TB accept replays the
 * +blitdump op stream through blitgold and diffs +blitvram (H3 flow).
 *
 * Progress mailbox (0xAC7F0000): [0] 0xD1A60001 = C entered,
 * 0xD1A60002 = first EXEC retired, 0xD1A60003 = frame loop; [1] frames.
 */
#include "diag.h"

#define LIST_VA   ((u16 *)0xAC100000u)
#define LIST_PA   0x0C100000u

/* Page origin (scroll = clip = draw offset).  MUST be >= 32 like the real
 * game's 32/416 pages: a clip origin below the 32-px margin puts the
 * timing governor's u16 clip window into wraparound (unmeasured hardware
 * corner, M-9 family) - the engine still draws but zero ops get priced.
 * Found by this ROM's first run at origin 0. */
#define ORG_X     32
#define ORG_Y     32

/* white 32x32 source tile, uploaded off-screen */
#define SPR_X     0u
#define SPR_Y     3968u

/* gcc -ffreestanding may still emit these for aggregates/loops */
void *memset(void *d, int c, unsigned n)
{
    u8 *p = (u8 *)d;
    while (n--) *p++ = (u8)c;
    return d;
}
void *memcpy(void *d, const void *s, unsigned n)
{
    u8 *p = (u8 *)d; const u8 *q = (const u8 *)s;
    while (n--) *p++ = *q++;
    return d;
}

static u16 *draw_square(u16 *p, s32 x, s32 y)
{
    /* straight copy: no blend, no trans, unity tint */
    return op_draw(p, ATTR(0, 0, 0, 0, 0, 0), ALPHA(0xFF, 0xFF),
                   SPR_X, SPR_Y, ORG_X + x, ORG_Y + y, 32, 32,
                   0x80, 0x80, 0x80);
}

static u16 *draw_clear(u16 *p)
{
    /* tint 0 multiplies any source to black: full-window clear without a
     * dedicated black tile (source RGB beyond the uploaded 32x32 is
     * don't-care, x0 kills it; A copies through - unwritten VRAM = A=0).
     *
     * Source row 3776: a 240-row source starting at SPR_Y=3968 would wrap
     * mod 4096 into the destination rows and the engine then (correctly)
     * flags the op strict/self-overlap and serializes it beat-by-beat -
     * on the real DDR3 stack every 8-px beat pays a full train latency
     * (~0.4 us) and the "clear" takes ~9 ms.  Found by the H7b.3 stat-
     * timed TB; games never wrap a big source into its destination. */
    return op_draw(p, ATTR(0, 0, 0, 0, 0, 0), ALPHA(0xFF, 0xFF),
                   0, 3776, ORG_X, ORG_Y, 320, 240, 0x00, 0x00, 0x00);
}

static u16 *frame_list(u16 *p, u32 frame)
{
    p = draw_clear(p);
    p = draw_square(p,   8,   8);
    p = draw_square(p, 280,   8);
    p = draw_square(p,   8, 200);
    p = draw_square(p, 280, 200);
    p = draw_square(p, (s32)(8 + ((frame << 2) & 0xFFu)), 104);
    return op_end(p);
}

int main(void)
{
    u32 frame = 0;
    u16 *p;
    u32 i;

    MAILBOX[MB_MAGIC] = 0xD1A60001u;

    /* single buffer at the game-convention page origin */
    REG32(BLIT_SCRX)  = ORG_X;
    REG32(BLIT_SCRY)  = ORG_Y;
    REG32(BLIT_CLIPX) = ORG_X;
    REG32(BLIT_CLIPY) = ORG_Y;

    /* first list: upload the white tile, then frame 0 */
    p = op_upload_hdr(LIST_VA, SPR_X, SPR_Y, 32, 32);
    for (i = 0; i < 32u * 32u; i++)
        *p++ = 0xFFFFu;                     /* A=1, white */
    p = frame_list(p, 0);

    blit_ready_wait();
    blit_exec(LIST_PA);
    blit_ready_wait();
    MAILBOX[MB_MAGIC] = 0xD1A60002u;

    for (;;) {
        vsync_wait();
        frame++;
        p = frame_list(LIST_VA, frame);
        (void)p;
        blit_ready_wait();
        blit_exec(LIST_PA);
        MAILBOX[MB_MAGIC] = 0xD1A60003u;
        MAILBOX[MB_FRAME] = frame;
    }
}
