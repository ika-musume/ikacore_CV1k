/* diag.h - CV1k diagnostic ROM common definitions  [H7b.D]
 *
 * Register map facts: blit_regs.sv (CS6 @ phys 0x18000000, P2 view
 * 0xB8000000), HS3 intc.sv (IRR0 @ 0xA4000004, edge-pend, write-0-clear),
 * op-word encodings per blitgold/golden.h (= MAME cv1k_v).
 *
 * Everything runs uncached (P2): op-list stores are immediately visible
 * to the blitter's BREQ/BACK fetch.  No division/modulo anywhere - the
 * sh4-linux-gnu-gcc -mb path has no big-endian libgcc.
 */
#ifndef DIAG_H
#define DIAG_H

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;
typedef signed short   s16;
typedef signed int     s32;

#define REG32(a) (*(volatile u32 *)(a))
#define REG16(a) (*(volatile u16 *)(a))
#define REG8(a)  (*(volatile u8  *)(a))

/* ---- blitter register file (CS6) ---- */
#define BLIT_EXEC   0xB8000004u   /* write 1: shadow-latch list/clip/scroll + kick */
#define BLIT_LIST   0xB8000008u   /* op list PHYSICAL address [28:0]               */
#define BLIT_STATUS 0xB8000010u   /* bit4 = ready (1) / busy (0)                   */
#define BLIT_SCRX   0xB8000014u
#define BLIT_SCRY   0xB8000018u
#define BLIT_ACK    0xB8000024u   /* write 1: video-side vblank IRQ ack pulse      */
#define BLIT_CLIPX  0xB8000040u
#define BLIT_CLIPY  0xB8000044u
#define BLIT_DSW    0xB8000050u   /* [3:0] DIP S2                                  */

/* ---- INTC (polled: SR.IMASK stays F, IRR0 still latches edges) ---- */
#define INTC_IRR0   0xA4000004u   /* bit2 = IRQ2 (vblank) pend; write-0-to-clear   */

/* ---- diag mailbox (top of work RAM, below the stack guard band) ---- */
#define MAILBOX     ((volatile u32 *)0xAC7F0000u)
#define MB_MAGIC    0u            /* 0xD1A6xxxx progress marker  */
#define MB_FRAME    1u            /* frame counter               */
#define MB_LFSR     2u            /* current LFSR state (diag-1k) */

/* ---- op-word builders (16-bit words; the BE CPU stores them in the
 *      exact memory order the fetch walker consumes) ---- */

/* DRAW w0 attribute bits (golden.h do_draw) */
#define ATTR(flipx, flipy, blend, trans, smode, dmode) \
    (u16)(((flipx) << 11) | ((flipy) << 10) | ((blend) << 9) | \
          ((trans) << 8) | ((smode) << 4) | (dmode))

/* alpha word: [15:8] s_alpha, [7:0] d_alpha (top 5 bits used) */
#define ALPHA(s, d)  (u16)(((s) << 8) | (d))

static inline u16 *op_draw(u16 *p, u16 attr, u16 alpha,
                           u32 sx, u32 sy, s32 dx, s32 dy, u32 w, u32 h,
                           u32 tint_r, u32 tint_g, u32 tint_b)
{
    p[0] = (u16)(0x1000u | attr);
    p[1] = alpha;
    p[2] = (u16)sx;                 /* [12:0] */
    p[3] = (u16)sy;                 /* [11:0] */
    p[4] = (u16)dx;                 /* signed 16-bit */
    p[5] = (u16)dy;
    p[6] = (u16)(w - 1);
    p[7] = (u16)(h - 1);
    p[8] = (u16)tint_r;             /* low byte; 0x80 = unity */
    p[9] = (u16)((tint_g << 8) | tint_b);
    return p + 10;
}

/* UPLOAD: 8 header words + w*h ARGB1555 payload words follow */
static inline u16 *op_upload_hdr(u16 *p, u32 dx, u32 dy, u32 w, u32 h)
{
    p[0] = 0x2000u;
    p[1] = 0; p[2] = 0; p[3] = 0;
    p[4] = (u16)dx;
    p[5] = (u16)dy;
    p[6] = (u16)(w - 1);
    p[7] = (u16)(h - 1);
    return p + 8;
}

static inline u16 *op_end(u16 *p) { *p++ = 0xF000u; return p; }

/* ---- pacing ---- */

/* Compiler fence: the op lists are built with PLAIN stores that the
 * blitter's bus-master fetch reads behind the compiler's back.  Without
 * this, gcc may legally sink the list stores past the volatile EXEC
 * register write (observed: the first EXEC fetched a stale END word). */
#define barrier() __asm__ __volatile__("" ::: "memory")

static inline void blit_ready_wait(void)
{
    while (!(REG32(BLIT_STATUS) & 0x10u)) ;
    barrier();
}

static inline void blit_exec(u32 list_phys)
{
    barrier();                      /* every list word lands before the kick */
    REG32(BLIT_LIST) = list_phys;
    REG32(BLIT_EXEC) = 1;
}

/* vblank: wait for the IRR0.2 edge latch, clear it, pulse the video ack */
static inline void vsync_wait(void)
{
    while (!(REG8(INTC_IRR0) & 0x04u)) ;
    REG8(INTC_IRR0) = 0xFBu;        /* write-0-to-clear bit2, hold others */
    REG32(BLIT_ACK) = 1;
    barrier();
}

/* xorshift32 - shifts/xors only (no libgcc) */
static inline u32 lfsr_next(u32 s)
{
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

#endif /* DIAG_H */
