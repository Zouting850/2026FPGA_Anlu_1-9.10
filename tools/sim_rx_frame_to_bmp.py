#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# tools/sim_rx_frame_to_bmp.py
#
# Cycle-accurate Python model of
#   boardB_eth_frontend/source_code/rtl/rx_frame_to_bmp.v
# chained end-to-end into the bmp_decode model (tools/sim_bmp_decode.py).
#
# Two things are verified:
#   1. rx_frame_to_bmp in isolation: SOF-magic hunt (with partial-match
#      recovery), transport-length capture, exactly-file_len byte forwarding,
#      single start pulse per image, idle-watchdog abandon + re-hunt.
#   2. The rx_frame_to_bmp -> bmp_decode INTEGRATION contract: start pulses
#      ALONE (never the same cycle bmp_decode samples in_valid), the forwarded
#      bytes are the BMP file starting at 'B', and a full valid image decodes to
#      the exact expected pixel stream -- including when the payload itself
#      contains the magic bytes (no mid-stream re-hunt).
#
# Non-blocking semantics, negative controls, no Verilog simulator -- same
# discipline as sim_bmp_decode.py / sim_spi_master.py.
#
# Run:  python tools/sim_rx_frame_to_bmp.py
# ---------------------------------------------------------------------------

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_bmp_decode import (BmpDecode, make_bmp, expected_pixels,  # noqa: E402
                            pix_gradient, pix_const, MASK32)        # noqa: E402

S_HUNT   = 0
S_LEN    = 1
S_STREAM = 2
DEFAULT_MAGIC = 0xA55A5AA5


# ===========================================================================
# The model: mirror of rx_frame_to_bmp.v
# ===========================================================================
class RxFrameToBmp:
    def __init__(self, sof_magic=DEFAULT_MAGIC, idle_aw=25):
        self.M0 = (sof_magic >> 24) & 0xFF
        self.M1 = (sof_magic >> 16) & 0xFF
        self.M2 = (sof_magic >> 8) & 0xFF
        self.M3 = sof_magic & 0xFF
        self.IDLE_AW = idle_aw
        self.SAT_BIT = idle_aw - 1
        self._reset()

    def _reset(self):
        self.state = S_HUNT
        self.magic_idx = 0
        self.len_idx = 0
        self.file_len_t = 0
        self.byte_remain = 0
        self.idle_cnt = 0
        self.start = 0
        self.out_valid = 0
        self.out_byte = 0
        self.idle_timeout_err = 0

    def clk_edge(self, rst=False, in_valid=False, in_byte=0):
        in_byte &= 0xFF
        idle_sat = (self.idle_cnt >> self.SAT_BIT) & 1

        # ---- idle watchdog counter -- RTL "Idle watchdog" block -----------
        if rst:
            n_idle = 0
        elif self.state == S_HUNT:
            n_idle = 0
        elif in_valid:
            n_idle = 0
        elif not idle_sat:
            n_idle = self.idle_cnt + 1
        else:
            n_idle = self.idle_cnt

        # ---- framing FSM -- RTL main always block -------------------------
        n = {
            'start': 0, 'out_valid': 0, 'idle_timeout_err': 0,
            'state': self.state, 'magic_idx': self.magic_idx,
            'len_idx': self.len_idx, 'file_len_t': self.file_len_t,
            'byte_remain': self.byte_remain, 'out_byte': self.out_byte,
        }
        if rst:
            n.update(state=S_HUNT, magic_idx=0, len_idx=0, file_len_t=0,
                     byte_remain=0, start=0, out_valid=0, out_byte=0,
                     idle_timeout_err=0)
        elif self.state == S_HUNT:
            if in_valid:
                mi = self.magic_idx
                if mi == 0:
                    n['magic_idx'] = 1 if in_byte == self.M0 else 0
                elif mi == 1:
                    n['magic_idx'] = 2 if in_byte == self.M1 else \
                                     (1 if in_byte == self.M0 else 0)
                elif mi == 2:
                    n['magic_idx'] = 3 if in_byte == self.M2 else \
                                     (1 if in_byte == self.M0 else 0)
                elif mi == 3:
                    if in_byte == self.M3:
                        n['state'] = S_LEN
                        n['magic_idx'] = 0
                        n['len_idx'] = 0
                    else:
                        n['magic_idx'] = 1 if in_byte == self.M0 else 0
                else:
                    n['magic_idx'] = 0
        elif self.state == S_LEN:
            if idle_sat:
                n['state'] = S_HUNT
                n['len_idx'] = 0
                n['idle_timeout_err'] = 1
            elif in_valid:
                li = self.len_idx
                if li == 0:
                    n['file_len_t'] = (self.file_len_t & ~0xFF) | in_byte
                    n['len_idx'] = 1
                elif li == 1:
                    n['file_len_t'] = (self.file_len_t & ~0xFF00) | (in_byte << 8)
                    n['len_idx'] = 2
                elif li == 2:
                    n['file_len_t'] = (self.file_len_t & ~0xFF0000) | (in_byte << 16)
                    n['len_idx'] = 3
                elif li == 3:
                    n['file_len_t'] = ((self.file_len_t & ~0xFF000000) |
                                       (in_byte << 24)) & MASK32
                    n['byte_remain'] = ((in_byte << 24) |
                                        (self.file_len_t & 0xFFFFFF)) & MASK32
                    n['len_idx'] = 0
                    n['state'] = S_STREAM
                    n['start'] = 1
                else:
                    n['len_idx'] = 0
        elif self.state == S_STREAM:
            if idle_sat:
                n['state'] = S_HUNT
                n['magic_idx'] = 0
                n['byte_remain'] = 0
                n['idle_timeout_err'] = 1
            elif in_valid:
                n['out_byte'] = in_byte
                n['out_valid'] = 1
                if self.byte_remain == 1:
                    n['byte_remain'] = 0
                    n['state'] = S_HUNT
                    n['magic_idx'] = 0
                else:
                    n['byte_remain'] = self.byte_remain - 1
        else:
            n['state'] = S_HUNT

        # ---- commit -------------------------------------------------------
        self.idle_cnt = n_idle & ((1 << self.IDLE_AW) - 1)
        self.state = n['state']
        self.magic_idx = n['magic_idx']
        self.len_idx = n['len_idx']
        self.file_len_t = n['file_len_t']
        self.byte_remain = n['byte_remain']
        self.start = n['start']
        self.out_valid = n['out_valid']
        self.out_byte = n['out_byte'] & 0xFF
        self.idle_timeout_err = n['idle_timeout_err']
        return {
            'start': self.start, 'out_valid': self.out_valid,
            'out_byte': self.out_byte, 'idle_timeout_err': self.idle_timeout_err,
            'state': self.state, 'byte_remain': self.byte_remain,
        }


# ===========================================================================
# Wire framing + chained driver
# ===========================================================================
def wire_frame(file_bytes, magic=DEFAULT_MAGIC):
    """[4-byte magic MSB-first][4-byte LE length][file bytes]."""
    flen = len(file_bytes)
    hdr = [(magic >> 24) & 0xFF, (magic >> 16) & 0xFF,
           (magic >> 8) & 0xFF, magic & 0xFF,
           flen & 0xFF, (flen >> 8) & 0xFF,
           (flen >> 16) & 0xFF, (flen >> 24) & 0xFF]
    return hdr + list(file_bytes)


def wire_cycles(wire_bytes, gap=None):
    """Turn wire bytes into (in_valid, in_byte) cycles, optional idle gaps."""
    cyc = []
    for i, b in enumerate(wire_bytes):
        if gap is not None and gap(i):
            cyc.append((False, 0))
        cyc.append((True, b))
    return cyc


def run_cycles(cycles, idle_aw=25):
    """Drive rx_frame_to_bmp chained into bmp_decode over the given cycles."""
    rx = RxFrameToBmp(idle_aw=idle_aw)
    bmp = BmpDecode()
    rx.clk_edge(rst=True); bmp.clk_edge(rst=True)
    rx.clk_edge(rst=True); bmp.clk_edge(rst=True)

    s = ov = 0
    ob = 0
    pixels = []
    dim_pulses = 0
    dim_w = dim_h = None
    coincident = 0
    errs = 0
    start_pulses = 0
    out_valid_pulses = 0

    for (iv, ib) in cycles:
        # bmp edge: samples rx's outputs registered last cycle.
        if s and ov:
            coincident += 1                 # CONTRACT VIOLATION if ever > 0
        bo = bmp.clk_edge(start=bool(s), in_valid=bool(ov), in_byte=ob)
        if bo['src_valid']:
            pixels.append(bo['src_pixel'])
        if bo['src_dim_valid']:
            dim_pulses += 1
            dim_w, dim_h = bo['src_width'], bo['src_height']
        # rx edge: samples this cycle's input byte.
        ro = rx.clk_edge(in_valid=iv, in_byte=ib)
        if ro['idle_timeout_err']:
            errs += 1
        s, ov, ob = ro['start'], ro['out_valid'], ro['out_byte']
        if s:
            start_pulses += 1
        if ov:
            out_valid_pulses += 1

    return {
        'pixels': pixels, 'dim_pulses': dim_pulses, 'dim_w': dim_w,
        'dim_h': dim_h, 'coincident': coincident, 'errs': errs,
        'start_pulses': start_pulses, 'out_valid_pulses': out_valid_pulses,
        'end_state': rx.state,
    }


def run_chain(wire_bytes, idle_aw=25, gap=None, drain=8):
    return run_cycles(wire_cycles(wire_bytes, gap) + [(False, 0)] * drain, idle_aw)


# ===========================================================================
# Test framework
# ===========================================================================
PASS = 0
FAIL = 0


def check(cond, msg):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [PASS] {msg}")
    else:
        FAIL += 1
        print(f"  [FAIL] {msg}")


def test_full_decode(name, width, height, gap=None):
    pix = pix_gradient
    data = make_bmp(width, height, pix)
    exp = expected_pixels(width, height, pix)
    r = run_chain(wire_frame(data), gap=gap)
    ok = (r['pixels'] == exp and r['coincident'] == 0 and
          r['start_pulses'] == 1 and r['out_valid_pulses'] == len(data) and
          r['dim_pulses'] == 1 and r['dim_w'] == width and
          r['dim_h'] == height and r['errs'] == 0 and
          r['end_state'] == S_HUNT)
    gnote = " [gapped]" if gap else ""
    check(ok, f"{name} {width}x{height}{gnote}: {len(r['pixels'])}/{len(exp)} px, "
              f"fwd={r['out_valid_pulses']}/{len(data)}B start={r['start_pulses']} "
              f"coincident={r['coincident']} dim={r['dim_w']}x{r['dim_h']} "
              f"errs={r['errs']} end=HUNT")
    return r


def test_partial_magic_recovery():
    pix = pix_gradient
    data = make_bmp(64, 64, pix)
    exp = expected_pixels(64, 64, pix)
    # A5,5A,A5 is a false partial: matches M0,M1 then mismatches M2 (expects 5A).
    prefix = [0xA5, 0x5A, 0xA5]
    r = run_chain(prefix + wire_frame(data))
    check(r['pixels'] == exp and r['start_pulses'] == 1 and r['coincident'] == 0,
          f"partial-magic recovery (A5,5A,A5 then real SOF): {len(r['pixels'])} px, "
          f"start={r['start_pulses']}")


def test_wrong_magic_rejected():
    # Black BMP so the payload cannot accidentally contain the real magic.
    data = make_bmp(64, 64, pix_const(0, 0, 0))
    r = run_chain(wire_frame(data, magic=0xA55A5AA6))   # M3 corrupted
    check(r['pixels'] == [] and r['start_pulses'] == 0 and
          r['out_valid_pulses'] == 0 and r['coincident'] == 0,
          f"wrong SOF magic rejected: pixels={len(r['pixels'])} "
          f"start={r['start_pulses']} fwd={r['out_valid_pulses']}")


def test_magic_in_payload_not_rehunted():
    # Every pixel = (R=5A,G=5A,B=A5) -> file bytes A5,5A,5A repeating, so the
    # 4-byte magic A5,5A,5A,A5 occurs across pixel boundaries throughout the
    # payload. A framer that re-hunted mid-stream would truncate; this must not.
    pix = pix_const(0x5A, 0x5A, 0xA5)
    data = make_bmp(64, 64, pix)
    exp = expected_pixels(64, 64, pix)
    assert exp[0] == 0x5A5AA5
    r = run_chain(wire_frame(data))
    check(r['pixels'] == exp and r['start_pulses'] == 1 and
          r['out_valid_pulses'] == len(data),
          f"magic-in-payload not re-hunted: {len(r['pixels'])}/{len(exp)} px, "
          f"start={r['start_pulses']} (must be 1), fwd={r['out_valid_pulses']}")


def test_two_images_back_to_back():
    pix1 = pix_gradient
    pix2 = lambda r, c: ((r + c) & 0xFF, (r * 2 + c) & 0xFF, (r + c * 3) & 0xFF)
    d1 = make_bmp(64, 64, pix1)
    d2 = make_bmp(65, 64, pix2)
    wire = wire_frame(d1) + wire_frame(d2)
    r = run_chain(wire, drain=16)
    e1 = expected_pixels(64, 64, pix1)
    e2 = expected_pixels(65, 64, pix2)
    ok = (r['pixels'] == e1 + e2 and r['start_pulses'] == 2 and
          r['out_valid_pulses'] == len(d1) + len(d2) and
          r['dim_pulses'] == 2 and r['coincident'] == 0)
    check(ok, f"two images back-to-back: {len(r['pixels'])}/{len(e1)+len(e2)} px, "
              f"start={r['start_pulses']} dim_pulses={r['dim_pulses']} "
              f"coincident={r['coincident']}")


def test_truncation_watchdog_and_recovery():
    pix = pix_gradient
    data = make_bmp(64, 64, pix)
    exp = expected_pixels(64, 64, pix)
    full = wire_frame(data)
    # Feed magic+len+100 file bytes, then 40 idle cycles (watchdog @ idle_aw=5
    # fires at 16), then a COMPLETE valid frame.
    cyc = (wire_cycles(full[:8 + 100]) +
           [(False, 0)] * 40 +
           wire_cycles(full) +
           [(False, 0)] * 16)
    r = run_cycles(cyc, idle_aw=5)
    # The first (truncated) frame emits junk pixels before the watchdog drops
    # it; recovery is proven by the SECOND frame decoding in full and by the
    # watchdog having fired and re-armed (2 start pulses total).
    tail_ok = r['pixels'][-len(exp):] == exp
    check(r['errs'] >= 1 and r['start_pulses'] == 2 and tail_ok and
          r['end_state'] == S_HUNT,
          f"truncation watchdog + recovery: errs={r['errs']} (>=1) "
          f"start={r['start_pulses']} (==2) tail==exp:{tail_ok} end=HUNT")


def test_start_alone_no_gap():
    # Even with zero gaps, start must be a lone pulse the cycle before byte 0.
    pix = pix_gradient
    data = make_bmp(64, 64, pix)
    r = run_chain(wire_frame(data))   # no gap
    check(r['coincident'] == 0 and r['start_pulses'] == 1,
          f"start-alone contract (no gaps): coincident={r['coincident']} "
          f"start={r['start_pulses']}")


def main():
    print("=== rx_frame_to_bmp.v model + rx->bmp_decode integration ===\n")

    print("Full decode through the chain:")
    test_full_decode("no-pad",   64, 64)
    test_full_decode("pad-2",    66, 64)
    test_full_decode("square",   100, 100)
    test_full_decode("gapped",   65, 64, gap=lambda i: i % 3 == 0)
    print()

    print("Framing robustness:")
    test_start_alone_no_gap()
    test_partial_magic_recovery()
    test_two_images_back_to_back()
    print()

    print("Negative controls:")
    test_wrong_magic_rejected()
    test_magic_in_payload_not_rehunted()
    test_truncation_watchdog_and_recovery()
    print()

    total = PASS + FAIL
    print(f"==== {'ALL PASS' if FAIL == 0 else 'FAILURES'} ==== ({PASS}/{total})")
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
