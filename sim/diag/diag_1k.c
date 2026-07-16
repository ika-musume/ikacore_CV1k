/* diag_1k.c - CV1k DDR3/SDRAM soak ROM: 1,024 gradient circles + waves  [H7b.D]
 *
 * Sustained, deterministic per-frame blit load with tiny code space:
 *  - ONE 8x8 circle sprite (radial gradient, A=1 inside), synthesized at
 *    runtime and UPLOADed once; every circle draw colors it through the
 *    DRAW tint fields and randomizes its s/d blend alphas -> 1,024
 *    uniquely colored/translucent gradient circles from 128 sprite bytes.
 *  - ONE 32x8 stripe tile (banded gradient) tiled over the whole screen
 *    with a per-band-row triangle-wave x offset -> waving background
 *    (doubles as the frame clear).
 *  - xorshift32 LFSR seeds positions/velocities/tints/alphas; motion is
 *    add + bounce (shift/mask/mul only - no division anywhere).
 *  - double-buffered op lists + EXEC per vblank, the game's pacing
 *    pattern (IRR0.2 edge poll, STATUS bit4 gate).
 *
 * Per frame: 330 stripe draws + 1,024 blended circle draws ~= 13.6k op
 * words ~= 1.9 ms of modeled draw time - a real load at 60 Hz, still
 * inside the frame budget.
 *
 * Mailbox: [0] 0xD1A61001 init / 0xD1A61002 sprites up / 0xD1A61003
 * frame loop; [1] frame counter; [2] LFSR state.
 */
#include "diag.h"

#define LIST0_VA  ((u16 *)0xAC100000u)
#define LIST0_PA  0x0C100000u
#define LIST1_VA  ((u16 *)0xAC180000u)
#define LIST1_PA  0x0C180000u

/* page origin: MUST be >= 32 (game convention) - see diag_mini.c note on
 * the governor's u16 clip window */
#define ORG_X     32
#define ORG_Y     32

#define CIRC_X    0u          /* sprite atlas row, off-screen */
#define CIRC_Y    3968u
#define STRP_X    32u
#define STRP_Y    3968u

#define NCIRC     1024u

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

static s16 pos_x[NCIRC], pos_y[NCIRC];
static s16 vel_x[NCIRC], vel_y[NCIRC];
static u16 tint_rw[NCIRC];            /* w8: red (low byte)          */
static u16 tint_gb[NCIRC];            /* w9: {g,b}                   */
static u16 alpha_w[NCIRC];            /* w1: {s_alpha, d_alpha}      */
static u8  wave[32];                  /* triangle 0..30              */

/* 8x8 radial-gradient circle, gray (colored per draw via tint) */
static u16 *upload_circle(u16 *p)
{
    u32 dy, dx;
    p = op_upload_hdr(p, CIRC_X, CIRC_Y, 8, 8);
    for (dy = 0; dy < 8; dy++) {
        s32 ay = (s32)(dy << 1) - 7;
        for (dx = 0; dx < 8; dx++) {
            s32 ax = (s32)(dx << 1) - 7;
            u32 r2 = (u32)(ax * ax + ay * ay);      /* 2..98 */
            if (r2 <= 49u) {
                u32 lum = 12u + ((49u - r2) >> 1);  /* 12..35 */
                if (lum > 31u) lum = 31u;
                *p++ = (u16)(0x8000u | (lum << 10) | (lum << 5) | lum);
            } else {
                *p++ = 0x0000u;                     /* A=0: trans-skipped */
            }
        }
    }
    return p;
}

/* 32x8 banded-gradient stripe, gray (tinted per band row) */
static u16 *upload_stripe(u16 *p)
{
    static const u8 band[8] = { 6, 10, 15, 21, 26, 21, 15, 10 };
    u32 dy, dx;
    p = op_upload_hdr(p, STRP_X, STRP_Y, 32, 8);
    for (dy = 0; dy < 8; dy++) {
        u32 lum = band[dy];
        u16 px  = (u16)(0x8000u | (lum << 10) | (lum << 5) | lum);
        for (dx = 0; dx < 32; dx++)
            *p++ = px;
    }
    return p;
}

/* waving background: 30 band rows x 11 tiles, tinted deep blue */
static u16 *draw_waves(u16 *p, u32 frame)
{
    u32 row, k;
    for (row = 0; row < 30; row++) {
        u32 w  = wave[(row + (frame >> 1)) & 31u];
        s32 x0 = (s32)w - 16;
        u32 tb_ = 0x50u + w;                        /* shimmer the blue */
        for (k = 0; k < 11; k++)
            p = op_draw(p, ATTR(0, 0, 0, 0, 0, 0), ALPHA(0xFF, 0xFF),
                        STRP_X, STRP_Y,
                        ORG_X + x0 + (s32)(k << 5), ORG_Y + (s32)(row << 3),
                        32, 8, 0x18, 0x28, tb_);
    }
    return p;
}

static u16 *draw_circles(u16 *p)
{
    u32 i;
    for (i = 0; i < NCIRC; i++) {
        /* blended (smode0/dmode0: s_alpha*src + d_alpha*dst), trans=1 so
         * the A=0 corners stay background */
        p = op_draw(p, ATTR(0, 0, 1, 1, 0, 0), alpha_w[i],
                    CIRC_X, CIRC_Y, ORG_X + pos_x[i], ORG_Y + pos_y[i], 8, 8,
                    tint_rw[i], (tint_gb[i] >> 8), (tint_gb[i] & 0xFFu));
        pos_x[i] = (s16)(pos_x[i] + vel_x[i]);
        pos_y[i] = (s16)(pos_y[i] + vel_y[i]);
        if (pos_x[i] < 0)   { pos_x[i] = 0;   vel_x[i] = (s16)-vel_x[i]; }
        if (pos_x[i] > 312) { pos_x[i] = 312; vel_x[i] = (s16)-vel_x[i]; }
        if (pos_y[i] < 0)   { pos_y[i] = 0;   vel_y[i] = (s16)-vel_y[i]; }
        if (pos_y[i] > 232) { pos_y[i] = 232; vel_y[i] = (s16)-vel_y[i]; }
    }
    return p;
}

int main(void)
{
    u32 l = 0xC0FFEE01u;
    u32 i, frame;
    u16 *p;

    MAILBOX[MB_MAGIC] = 0xD1A61001u;

    for (i = 0; i < 32; i++)
        wave[i] = (u8)((i < 16 ? i : (31u - i)) << 1);

    for (i = 0; i < NCIRC; i++) {
        l = lfsr_next(l);
        pos_x[i] = (s16)(28u + (l & 0xFFu));                  /* 28..283  */
        pos_y[i] = (s16)((l >> 8) & 0xE7u);                   /* 0..231   */
        vel_x[i] = (s16)(1 + ((l >> 16) & 3u));
        vel_y[i] = (s16)(1 + ((l >> 18) & 3u));
        if (l & 0x00100000u) vel_x[i] = (s16)-vel_x[i];
        if (l & 0x00200000u) vel_y[i] = (s16)-vel_y[i];
        l = lfsr_next(l);
        tint_rw[i] = (u16)(0x30u + (l & 0x7Fu));
        tint_gb[i] = (u16)((((0x30u + ((l >> 8) & 0x7Fu)) << 8)) |
                            (0x30u + ((l >> 16) & 0x7Fu)));
        alpha_w[i] = (u16)ALPHA(0x60u + ((l >> 24) & 0x7Fu),
                                0x80u + ((l >> 11) & 0x7Fu));
    }
    MAILBOX[MB_LFSR] = l;

    /* game-convention page origin */
    REG32(BLIT_SCRX)  = ORG_X;
    REG32(BLIT_SCRY)  = ORG_Y;
    REG32(BLIT_CLIPX) = ORG_X;
    REG32(BLIT_CLIPY) = ORG_Y;

    /* frame 0 on list 0: sprite uploads + first field */
    p = upload_circle(LIST0_VA);
    p = upload_stripe(p);
    p = draw_waves(p, 0);
    p = draw_circles(p);
    (void)op_end(p);

    blit_ready_wait();
    blit_exec(LIST0_PA);
    blit_ready_wait();
    MAILBOX[MB_MAGIC] = 0xD1A61002u;

    frame = 0;
    for (;;) {
        vsync_wait();
        frame++;
        /* build the back buffer while the front one scans out */
        if (frame & 1u) {
            p = draw_waves(LIST1_VA, frame);
            p = draw_circles(p);
            (void)op_end(p);
            blit_ready_wait();
            blit_exec(LIST1_PA);
        } else {
            p = draw_waves(LIST0_VA, frame);
            p = draw_circles(p);
            (void)op_end(p);
            blit_ready_wait();
            blit_exec(LIST0_PA);
        }
        MAILBOX[MB_MAGIC] = 0xD1A61003u;
        MAILBOX[MB_FRAME] = frame;
        MAILBOX[MB_LFSR]  = l;
    }
}
