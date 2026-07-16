#!/usr/bin/env python3
"""Extract a frame window from a .blit trace (blitstudy/blit_trace.h format)
into the tb_cv1k +blitreplay file.  Per exec:

  header "t_ckio list_addr cx cy nwords"   (t_ns -> CKIO at 51.2 MHz)
  then nwords lines "hex_byte_addr hex_u16" (real work-RAM addresses, 8 MB
  mask) - words are written by the TB just before the EXEC fires, because
  the game double-buffers lists at a few fixed addresses.

Usage: blit_replay_extract.py <trace.blit> <frame_lo> <frame_hi> <out>
Example (ddpsdoj slowdown window):
  scripts/blit_replay_extract.py blitstudy/traces/ddpsdoj_attract.blit \
      4050 4100 build/ddps_replay.txt
"""
import struct
import sys


def main():
    trace, flo, fhi, outf = sys.argv[1:5]
    flo, fhi = int(flo), int(fhi)
    f = open(trace, 'rb')
    assert f.read(4) == b'BLT1', "not a .blit trace"
    f.read(4)  # version
    out = open(outf, 'w')
    t0 = None
    nrec = nwords = 0
    t_ckio = 0
    while True:
        tag = f.read(4)
        if len(tag) < 4:
            break
        assert tag == b'EXEC', tag
        t_ns, frame = struct.unpack('<QQ', f.read(16))
        gfx, = struct.unpack('<I', f.read(4))
        cx, cy, sx, sy = struct.unpack('<4H', f.read(8))
        f.read(8)  # mame_delay_ns
        nw, = struct.unpack('<I', f.read(4))
        data = f.read(nw * 2)
        if not (flo <= frame <= fhi):
            continue
        if t0 is None:
            t0 = t_ns
        t_ckio = (t_ns - t0) * 512 // 10000
        out.write("%d %07x %d %d %d\n" %
                  (t_ckio, 0x0C000000 | (gfx & 0xFFFFFF), cx, cy, nw))
        base = gfx & 0x7FFFFF
        for i, w in enumerate(struct.unpack('<%dH' % nw, data)):
            out.write("%06x %04x\n" % (base + 2 * i, w))
        nrec += 1
        nwords += nw
    print("execs: %d  words: %d  span: %.1f ms" % (nrec, nwords, t_ckio / 51200.0))


if __name__ == '__main__':
    main()
