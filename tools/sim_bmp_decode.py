#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# tools/sim_bmp_decode.py
#
# Cycle-accurate Python model of
#   boardB_eth_frontend/source_code/rtl/bmp_decode.v
#
# Mirrors the RTL block-for-block with non-blocking (read-all-then-commit)
# semantics: every clk_edge() reads the OLD register state to compute every
# next-state value, then commits them all at once -- exactly like a Verilog
# `always @(posedge clk)` with `<=`. The model and the RTL therefore advance in
# lockstep, which is the whole point: there is no Verilog simulator on this
# machine (project discipline), so this model IS the verification artefact.
#
# What it proves
#   * header parse + geometry validation lifted from bmp_read.v behaves the same
#   * 4-byte row padding is dropped (widths 65/66/67 carry 1/2/3 pad bytes)
#   * B,G,R file bytes pack to src_pixel = {R,G,B} (R in the high byte)
#   * src_dim_valid is a single pulse, ahead of the first pixel
#   * the decoder survives an idle-gap-riddled stream (UDP reassembly never
#     delivers a byte every cycle)
#   * every out-of-contract file is REJECTED (hdr_ok stays 0, zero pixels)
#
# A model that only confirms the happy path proves nothing, so the negative
# controls below are load-bearing: each one MUST show the decoder rejecting bad
# input, and the byte-order control MUST show a wrong-order expectation
# mismatching. If a negative control ever "passes" by accepting, the suite fails.
#
# Run:  python tools/sim_bmp_decode.py
# ---------------------------------------------------------------------------

import sys

MASK32 = 0xFFFFFFFF
MASK24 = 0xFFFFFF
MASK16 = 0xFFFF

# Accepted geometry -- bmp_decode.v lines 79-82 (== bmp_read.v 71-74).
SRC_W_MIN = 64
SRC_W_MAX = 1920
SRC_H_MIN = 64
SRC_H_MAX = 1080

# byte_cnt at which the REGISTERED header_match_r first reflects a fully parsed
# header (compression, the last field, is final from byte 34; the register lags
# one cycle). Mirrors bmp_decode.v HDR_SETTLE_CNT -- see the long comment there.
HDR_SETTLE_CNT = 35


# ===========================================================================
# The model
# ===========================================================================
class BmpDecode:
    """Cycle-accurate mirror of bmp_decode.v. One clk_edge() == one posedge."""

    def __init__(self):
        self._reset_state()

    def _reset_state(self):
        # Every reg in the RTL, at its reset value.
        self.byte_cnt      = 0
        self.header_0      = 0
        self.header_1      = 0
        self.file_len      = 0
        self.pixel_offset  = 54          # RTL reset value (line 152)
        self.width         = 0
        self.height        = 0
        self.bit_count     = 0
        self.compression   = 0
        self.width_ok_r    = 0
        self.height_ok_r   = 0
        self.src_w3        = 0
        self.src_stride    = 0
        self.src_width     = 0
        self.src_height    = 0
        self.header_match_r = 0
        self.hdr_ok        = 0
        self.src_dim_valid = 0
        self.dim_pulsed    = 0
        self.row_byte_cnt  = 0
        self.bmp_byte_idx  = 0
        self.src_valid     = 0
        self.src_pixel     = 0

    # -- combinational wires, from CURRENT register state (RTL 110-128) -----
    def _comb(self, in_valid):
        w3_calc     = (self.width + ((self.width << 1) & MASK32)) & MASK32
        stride_calc = (w3_calc + 3) & 0xFFFFFFFC
        header_match_c = (self.header_0 == ord('B') and
                          self.header_1 == ord('M') and
                          bool(self.width_ok_r) and bool(self.height_ok_r) and
                          self.bit_count == 24 and
                          self.compression == 0)
        in_pixel_region = (self.byte_cnt >= self.pixel_offset) and \
                          (self.byte_cnt < self.file_len)
        bmp_data_valid = bool(in_valid) and bool(self.hdr_ok) and \
            in_pixel_region and (self.row_byte_cnt < self.src_w3)
        return {
            'w3_calc': w3_calc,
            'stride_calc': stride_calc,
            'header_match_c': header_match_c,
            'in_pixel_region': in_pixel_region,
            'bmp_data_valid': bmp_data_valid,
        }

    def clk_edge(self, rst=False, start=False, in_valid=False, in_byte=0):
        """Advance one clock. Returns the post-edge output snapshot."""
        in_byte &= 0xFF
        c = self._comb(in_valid)
        n = {}   # next-state; committed atomically at the end (non-blocking)

        # ---- byte_cnt -- RTL 133-140 ------------------------------------
        if rst or start:
            n['byte_cnt'] = 0
        elif in_valid:
            n['byte_cnt'] = (self.byte_cnt + 1) & MASK32
        else:
            n['byte_cnt'] = self.byte_cnt

        # ---- header field parse -- RTL 147-195 --------------------------
        if rst or start:
            n['header_0']     = 0
            n['header_1']     = 0
            n['file_len']     = 0
            n['pixel_offset'] = 54
            n['width']        = 0
            n['height']       = 0
            n['bit_count']    = 0
            n['compression']  = 0
        elif in_valid and self.byte_cnt <= 33:
            n['header_0']     = self.header_0
            n['header_1']     = self.header_1
            n['file_len']     = self.file_len
            n['pixel_offset'] = self.pixel_offset
            n['width']        = self.width
            n['height']       = self.height
            n['bit_count']    = self.bit_count
            n['compression']  = self.compression
            bc = self.byte_cnt
            if   bc == 0:  n['header_0'] = in_byte
            elif bc == 1:  n['header_1'] = in_byte
            elif bc == 2:  n['file_len'] = (self.file_len & ~0xFF) | in_byte
            elif bc == 3:  n['file_len'] = (self.file_len & ~0xFF00) | (in_byte << 8)
            elif bc == 4:  n['file_len'] = (self.file_len & ~0xFF0000) | (in_byte << 16)
            elif bc == 5:  n['file_len'] = (self.file_len & ~0xFF000000) | (in_byte << 24)
            elif bc == 10: n['pixel_offset'] = (self.pixel_offset & ~0xFF) | in_byte
            elif bc == 11: n['pixel_offset'] = (self.pixel_offset & ~0xFF00) | (in_byte << 8)
            elif bc == 12: n['pixel_offset'] = (self.pixel_offset & ~0xFF0000) | (in_byte << 16)
            elif bc == 13: n['pixel_offset'] = (self.pixel_offset & ~0xFF000000) | (in_byte << 24)
            elif bc == 18: n['width'] = (self.width & ~0xFF) | in_byte
            elif bc == 19: n['width'] = (self.width & ~0xFF00) | (in_byte << 8)
            elif bc == 20: n['width'] = (self.width & ~0xFF0000) | (in_byte << 16)
            elif bc == 21: n['width'] = (self.width & ~0xFF000000) | (in_byte << 24)
            elif bc == 22: n['height'] = (self.height & ~0xFF) | in_byte
            elif bc == 23: n['height'] = (self.height & ~0xFF00) | (in_byte << 8)
            elif bc == 24: n['height'] = (self.height & ~0xFF0000) | (in_byte << 16)
            elif bc == 25: n['height'] = (self.height & ~0xFF000000) | (in_byte << 24)
            elif bc == 28: n['bit_count'] = (self.bit_count & ~0xFF) | in_byte
            elif bc == 29: n['bit_count'] = (self.bit_count & ~0xFF00) | (in_byte << 8)
            elif bc == 30: n['compression'] = (self.compression & ~0xFF) | in_byte
            elif bc == 31: n['compression'] = (self.compression & ~0xFF00) | (in_byte << 8)
            elif bc == 32: n['compression'] = (self.compression & ~0xFF0000) | (in_byte << 16)
            elif bc == 33: n['compression'] = (self.compression & ~0xFF000000) | (in_byte << 24)
            n['file_len']     &= MASK32
            n['pixel_offset'] &= MASK32
            n['width']        &= MASK32
            n['height']       &= MASK32
            n['bit_count']    &= MASK16
            n['compression']  &= MASK32
        else:
            n['header_0']     = self.header_0
            n['header_1']     = self.header_1
            n['file_len']     = self.file_len
            n['pixel_offset'] = self.pixel_offset
            n['width']        = self.width
            n['height']       = self.height
            n['bit_count']    = self.bit_count
            n['compression']  = self.compression

        # ---- registered geometry validation + row pitch -- RTL 202-229 ----
        if rst or start:
            n['width_ok_r']  = 0
            n['height_ok_r'] = 0
            n['src_w3']      = 0
            n['src_stride']  = 0
            n['src_width']   = 0
            n['src_height']  = 0
        else:
            w = self.width
            h = self.height
            n['width_ok_r']  = 1 if (((w >> 16) & MASK16) == 0 and
                                     SRC_W_MIN <= (w & MASK16) <= SRC_W_MAX) else 0
            n['height_ok_r'] = 1 if (((h >> 16) & MASK16) == 0 and
                                     SRC_H_MIN <= (h & MASK16) <= SRC_H_MAX) else 0
            n['src_w3']      = c['w3_calc'] & MASK16
            n['src_stride']  = c['stride_calc'] & MASK16
            n['src_width']   = w & MASK16
            n['src_height']  = h & MASK16

        # ---- header_match_r -- RTL 231-235 -------------------------------
        if rst or start:
            n['header_match_r'] = 0
        else:
            n['header_match_r'] = 1 if c['header_match_c'] else 0

        # ---- hdr_ok / src_dim_valid / dim_pulsed -- RTL 241-258 ----------
        if rst or start:
            n['hdr_ok']        = 0
            n['src_dim_valid'] = 0
            n['dim_pulsed']    = 0
        else:
            n['src_dim_valid'] = 0
            n['hdr_ok']        = self.hdr_ok
            n['dim_pulsed']    = self.dim_pulsed
            header_settled = (self.byte_cnt >= HDR_SETTLE_CNT)
            if self.header_match_r and header_settled and not self.dim_pulsed:
                n['hdr_ok']        = 1
                n['src_dim_valid'] = 1
                n['dim_pulsed']    = 1

        # ---- row_byte_cnt (padding drop) -- RTL 264-275 ------------------
        if rst or start:
            n['row_byte_cnt'] = 0
        elif in_valid and self.hdr_ok and c['in_pixel_region']:
            if (self.row_byte_cnt + 1) >= self.src_stride:
                n['row_byte_cnt'] = 0
            else:
                n['row_byte_cnt'] = self.row_byte_cnt + 1
        else:
            n['row_byte_cnt'] = self.row_byte_cnt

        # ---- bmp_byte_idx -- RTL 281-288 --------------------------------
        if rst or start:
            n['bmp_byte_idx'] = 0
        elif c['bmp_data_valid']:
            n['bmp_byte_idx'] = 0 if self.bmp_byte_idx == 2 else (self.bmp_byte_idx + 1)
        else:
            n['bmp_byte_idx'] = self.bmp_byte_idx

        # ---- src_valid / src_pixel -- RTL 290-314 -----------------------
        # NOTE: only `rst` zeroes src_pixel; `start` does not (no start branch).
        if rst:
            n['src_valid'] = 0
            n['src_pixel'] = 0
        else:
            n['src_valid'] = 0
            n['src_pixel'] = self.src_pixel
            if c['bmp_data_valid']:
                if self.bmp_byte_idx == 0:
                    n['src_pixel'] = (self.src_pixel & ~0xFF) | in_byte          # B
                elif self.bmp_byte_idx == 1:
                    n['src_pixel'] = (self.src_pixel & ~0xFF00) | (in_byte << 8)  # G
                elif self.bmp_byte_idx == 2:
                    n['src_valid'] = 1
                    n['src_pixel'] = (self.src_pixel & ~0xFF0000) | (in_byte << 16)  # R
            n['src_pixel'] &= MASK24

        # ---- commit (non-blocking) --------------------------------------
        for k, v in n.items():
            setattr(self, k, v)

        # busy is combinational (RTL 128), reported from the post-edge state.
        busy = (self.byte_cnt < self.file_len) if self.hdr_ok else True
        return {
            'src_valid':     self.src_valid,
            'src_pixel':     self.src_pixel,
            'src_width':     self.src_width,
            'src_height':    self.src_height,
            'src_dim_valid': self.src_dim_valid,
            'hdr_ok':        self.hdr_ok,
            'busy':          busy,
        }


# ===========================================================================
# Synthetic BMP construction
# ===========================================================================
def make_bmp(width, height, pix, bit_count=24, compression=0, pixel_offset=54,
             top_down=False, magic=b'BM', hdr_size=40, pad_byte=0xAA):
    """Build a synthetic 24-bit BMP byte array (bottom-up storage, as real BMPs).

    pix(r, c) -> (R, G, B); r is the FILE row index (0 = first row in the file).
    Padding bytes are a distinctive 0xAA so a failure to drop them corrupts the
    pixel stream visibly instead of silently passing.
    """
    stride = ((width * 3) + 3) & ~3
    body = bytearray()
    for r in range(height):
        row = bytearray()
        for cc in range(width):
            R, G, B = pix(r, cc)
            row += bytes([B & 0xFF, G & 0xFF, R & 0xFF])   # BMP stores B,G,R
        while len(row) < stride:
            row.append(pad_byte)
        body += row
    file_len = pixel_offset + len(body)
    h_field = (-height) & MASK32 if top_down else (height & MASK32)

    hdr = bytearray()
    hdr += magic                                       # 0-1
    hdr += (file_len & MASK32).to_bytes(4, 'little')   # 2-5
    hdr += (0).to_bytes(4, 'little')                   # 6-9   reserved
    hdr += (pixel_offset & MASK32).to_bytes(4, 'little')  # 10-13
    hdr += (hdr_size & MASK32).to_bytes(4, 'little')   # 14-17 biSize
    hdr += (width & MASK32).to_bytes(4, 'little')      # 18-21
    hdr += (h_field & MASK32).to_bytes(4, 'little')    # 22-25
    hdr += (1).to_bytes(2, 'little')                   # 26-27 planes
    hdr += (bit_count & MASK16).to_bytes(2, 'little')  # 28-29
    hdr += (compression & MASK32).to_bytes(4, 'little')  # 30-33
    if pixel_offset > len(hdr):
        hdr += bytes(pixel_offset - len(hdr))          # rest of info header
    assert len(hdr) == pixel_offset, (len(hdr), pixel_offset)
    return bytes(hdr) + bytes(body)


def expected_pixels(width, height, pix):
    """src_pixel values the decoder MUST emit, in file order (R high byte)."""
    out = []
    for r in range(height):
        for c in range(width):
            R, G, B = pix(r, c)
            out.append(((R & 0xFF) << 16) | ((G & 0xFF) << 8) | (B & 0xFF))
    return out


def stream_file(dut, data, gap=None, rst_cycles=2):
    """Drive `data` bytes into a fresh dut. gap(i)->bool inserts an idle cycle
    before byte i. Returns the captured output stream and timing markers."""
    for _ in range(rst_cycles):
        dut.clk_edge(rst=True)
    dut.clk_edge(start=True)          # arm; contract: NOT coincident with byte 0

    pixels = []
    dim_pulses = 0
    dim_w = dim_h = None
    dim_at = None
    first_pixel_at = None
    hdr_ok_ever = False
    busy_at_end = None
    last_i = -1

    def observe(o, i):
        nonlocal dim_pulses, dim_w, dim_h, dim_at, first_pixel_at, hdr_ok_ever
        if o['src_dim_valid']:
            dim_pulses += 1
            dim_w, dim_h = o['src_width'], o['src_height']
            if dim_at is None:
                dim_at = i
        if o['hdr_ok']:
            hdr_ok_ever = True
        if o['src_valid']:
            pixels.append(o['src_pixel'])
            if first_pixel_at is None:
                first_pixel_at = i

    for i, b in enumerate(data):
        if gap is not None and gap(i):
            observe(dut.clk_edge(in_valid=False), i)
        o = dut.clk_edge(in_valid=True, in_byte=b)
        observe(o, i)
        busy_at_end = o['busy']
        last_i = i

    return {
        'pixels': pixels,
        'dim_pulses': dim_pulses,
        'dim_w': dim_w, 'dim_h': dim_h, 'dim_at': dim_at,
        'first_pixel_at': first_pixel_at,
        'hdr_ok': hdr_ok_ever,
        'busy_at_end': busy_at_end,
        'bytes': last_i + 1,
    }


# ===========================================================================
# Tiny test framework
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


def pix_gradient(r, c):
    return ((r * 3 + c * 5) & 0xFF, (r * 7 + c) & 0xFF, (r + c * 11) & 0xFF)


def pix_const(R, G, B):
    return lambda r, c: (R, G, B)


# ---------------------------------------------------------------------------
# Positive: a valid file decodes to the exact expected pixel stream
# ---------------------------------------------------------------------------
def test_valid(name, width, height, gap=None):
    pix = pix_gradient
    data = make_bmp(width, height, pix)
    exp = expected_pixels(width, height, pix)
    r = stream_file(BmpDecode(), data, gap=gap)
    ok = (r['hdr_ok'] is True and
          r['pixels'] == exp and
          r['dim_w'] == width and r['dim_h'] == height and
          r['dim_pulses'] == 1 and
          r['busy_at_end'] is False)
    padnote = ""
    stride = ((width * 3) + 3) & ~3
    if stride != width * 3:
        padnote = f" [drops {stride - width*3} pad byte/row]"
    gapnote = " [gapped]" if gap else ""
    check(ok, f"{name} {width}x{height}{padnote}{gapnote}: "
              f"{len(r['pixels'])}/{len(exp)} px, dim={r['dim_w']}x{r['dim_h']} "
              f"pulses={r['dim_pulses']} busy_end={r['busy_at_end']}")
    return r


def test_dim_before_pixel():
    r = test_valid("dim-lead", 65, 64)
    check(r['dim_at'] is not None and r['first_pixel_at'] is not None and
          r['dim_at'] < r['first_pixel_at'],
          f"src_dim_valid (byte {r['dim_at']}) precedes first pixel "
          f"(byte {r['first_pixel_at']})")


def test_byte_order():
    # A single distinctive pixel proves B->low, R->high packing.
    data = make_bmp(64, 64, pix_const(0x11, 0x22, 0x33))
    r = stream_file(BmpDecode(), data)
    check(len(r['pixels']) == 64 * 64 and r['pixels'][0] == 0x112233,
          f"RGB888 pack {{R,G,B}}: src_pixel=0x{r['pixels'][0]:06X} (want 0x112233)")
    # Negative control: the WRONG order (B in the high byte) must NOT match.
    check(r['pixels'][0] != 0x332211,
          "byte-order negative control: src_pixel != 0x332211 (B-high is wrong)")


# ---------------------------------------------------------------------------
# Negative controls: out-of-contract files MUST be rejected (hdr_ok=0, no px)
# ---------------------------------------------------------------------------
def expect_reject(name, data):
    r = stream_file(BmpDecode(), data)
    rejected = (r['hdr_ok'] is False and len(r['pixels']) == 0)
    check(rejected, f"rejects {name}: hdr_ok={r['hdr_ok']} pixels={len(r['pixels'])}")


def test_rejections():
    g = pix_gradient
    expect_reject("top-down (negative biHeight)",
                  make_bmp(64, 64, g, top_down=True))
    expect_reject("16-bit (bit_count=16)",
                  make_bmp(64, 64, g, bit_count=16))
    expect_reject("8-bit (bit_count=8)",
                  make_bmp(64, 64, g, bit_count=8))
    expect_reject("RLE8 compressed (compression=1)",
                  make_bmp(64, 64, g, compression=1))
    expect_reject("RLE4 compressed (compression=2)",
                  make_bmp(64, 64, g, compression=2))
    # The nastiest case: compression's only set bit is in byte 33, the LAST
    # header byte. header_match_r captured one cycle early (before byte 33
    # lands) still reads compression==0, so a byte_cnt>=34 gate would accept
    # this. The >=35 gate (HDR_SETTLE_CNT) is what rejects it.
    expect_reject("compression=0x01000000 (bit only in byte 33)",
                  make_bmp(64, 64, g, compression=0x01000000))
    expect_reject("compression=0x80000000 (bit only in byte 33)",
                  make_bmp(64, 64, g, compression=0x80000000))
    expect_reject("width 32 < SRC_W_MIN",
                  make_bmp(32, 64, g))
    expect_reject("width 2000 > SRC_W_MAX",
                  make_bmp(2000, 64, g))
    expect_reject("height 32 < SRC_H_MIN",
                  make_bmp(64, 32, g))
    expect_reject("height 1200 > SRC_H_MAX",
                  make_bmp(64, 1200, g))
    expect_reject("bad magic header_0 ('X','M')",
                  make_bmp(64, 64, g, magic=b'XM'))
    expect_reject("bad magic header_1 ('B','X')",
                  make_bmp(64, 64, g, magic=b'BX'))


def test_start_contract():
    # Negative control on the stream contract: if `start` is asserted on the
    # SAME edge as the first byte, that byte is consumed but ignored (start has
    # priority over the parse), shifting the whole header by one -> rejection.
    pix = pix_gradient
    data = make_bmp(64, 64, pix)
    dut = BmpDecode()
    dut.clk_edge(rst=True)
    dut.clk_edge(rst=True)
    # start coincident with byte 0:
    dut.clk_edge(start=True, in_valid=True, in_byte=data[0])
    got = []
    for b in data[1:]:
        o = dut.clk_edge(in_valid=True, in_byte=b)
        if o['src_valid']:
            got.append(o['src_pixel'])
        if o['hdr_ok']:
            hdr = True
            break
    else:
        hdr = False
    check(hdr is False and len(got) == 0,
          "start-coincident-with-first-byte loses byte 0 -> rejected "
          f"(hdr_ok={hdr}, pixels={len(got)}); contract = pulse start alone")


# ===========================================================================
def main():
    print("=== bmp_decode.v cycle-accurate model ===\n")

    print("Positive decode (exact pixel-stream match):")
    test_valid("no-pad",   64, 64)
    test_valid("pad-1",    65, 64)
    test_valid("pad-2",    66, 64)
    test_valid("pad-3",    67, 64)
    test_valid("square",   100, 100)
    test_valid("wide",     320, 240)
    test_valid("min-geom", 64, 64)
    print()

    print("Idle-gap robustness (same file, gaps in the byte stream):")
    test_valid("gap-every", 66, 64, gap=lambda i: True)
    test_valid("gap-alt",   67, 64, gap=lambda i: i % 2 == 0)
    test_valid("gap-mod3",  65, 64, gap=lambda i: i % 3 == 0)
    print()

    print("Dimension / timing contract:")
    test_dim_before_pixel()
    test_byte_order()
    print()

    print("Negative controls (must REJECT):")
    test_rejections()
    test_start_contract()
    print()

    total = PASS + FAIL
    print(f"==== {'ALL PASS' if FAIL == 0 else 'FAILURES'} ==== ({PASS}/{total})")
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
