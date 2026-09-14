#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Integration model for boardB_eth_frontend/source_code/rtl/board_b_top.v.

Every other module in the Board B chain has its own cycle-accurate model
(sim_rx_byte_cdc, sim_rx_frame_to_bmp, sim_bmp_decode, sim_scaler_nn,
sim_sdram_to_spi, sim_spi_master). What none of them cover is board_b_top.v
itself -- and a top level is exactly where the silent, image-corrupting mistakes
live, because it is nothing but wiring decisions. This suite pins down the five
decisions that are NEW in this top and that no submodule test can see:

  (1) DECODE CHAIN AT REAL GEOMETRY. rx_frame_to_bmp and bmp_decode were each
      proven on small synthetic files. The competition case is 640x480, and the
      wire framing plus the 4-byte-aligned row padding of a real 640-wide BMP
      (1920 B/row, already a multiple of 4, so no padding) is what actually ships.
      Driven end-to-end here with an early stop, because a full frame is 921654
      wire bytes and the transport is proven separately at reduced N.

  (2) THE IDENTITY LINCHPIN. The user chose "PC downscales to <=640x480", so at
      640x480 scaler_nn must be an exact pass-through: tw=640, th=480, no
      letterbox offset, want_sx == k. Everything downstream (SDRAM round-trip,
      SPI transport) is only pixel-value-preserving if THIS holds; if the scaler
      permuted or duplicated pixels, a passing transport test would prove nothing
      about the image the judge sees.

  (3) THE EXPLICIT [23:0] SLICE. sdram_top.Sdr_rd_dout is 32 bits (DATA_WIDTH=32)
      but the pixel is the low 24. The vendor top let Verilog truncate silently
      onto a 24-bit wire; board_b_top.v slices `Sdr_rd_dout_full[23:0]` on
      purpose. Proven here by putting GARBAGE in [31:24] and requiring the wire
      to carry the decoded pixel, compared against the UNMASKED truth -- using
      check_contract alone would mask both sides and pass a wrong slice.

  (4) THE frame_rst STRETCHER. rx_frame_to_bmp.start is a single clk_50m pulse;
      app_wrrd lives in the sdr_clk domain behind a 2-FF synchroniser and needs a
      multi-cycle reset to re-arm. board_b_top.v widens the pulse to
      FRAME_RST_WIDTH=16. Too short and app_wrrd never re-arms (frame never
      ships); too long and it is still in reset when the scaler's first pixel
      arrives (pixel dropped, image torn). The real margin is MEASURED here
      against the actual header-settle latency, not assumed.

  (5) THE usedw SATURATION. scaler_nn.i_fifo_usedw is 9 bits and STALL_THRESH=384
      was sized for a 512-deep FIFO whose occupancy wraps to 0 when full. The
      FIFO in this design is fifo_sdr_data_2 at 4096 deep / 12-bit wrusedw, so a
      naive `[8:0]` truncation reports 0 at 512 entries -- un-stalling the scaler
      against a full FIFO. board_b_top.v saturates at 511 instead.

Vendor IP (fifo_sdr_data_2, the encrypted SDRAM PHY, app_wrrd's own FSM) is
treated as correct, per house convention: only the glue we wrote is modelled.

Run:  python tools/sim_board_b_integration.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sim_bmp_decode import BmpDecode, make_bmp, expected_pixels        # noqa: E402
from sim_rx_frame_to_bmp import (RxFrameToBmp, wire_frame, wire_cycles,  # noqa: E402
                                 DEFAULT_MAGIC)
from sim_scaler_nn import (geometric_check, DST_W, DST_H, MAX_UPSCALE,  # noqa: E402
                           STALL_THRESH, SRC_W_MIN, SRC_W_MAX,
                           SRC_H_MIN, SRC_H_MAX)
import sim_sdram_to_spi as bridge                                      # noqa: E402
from sim_spi_master import reconstruct, check_contract                 # noqa: E402

# board_b_top.v parameters, mirrored. If the RTL changes, change these too --
# they are asserted against the RTL text by check_board_b_top_transcription().
FRAME_RST_WIDTH = 16          # board_b_top.v:82
USEDW_SAT       = 511         # board_b_top.v:393
SLICE_LO        = 0           # board_b_top.v:445  Sdr_rd_dout_full[23:0]
SPI_MAGIC       = 0xA55A5AA5  # board_b_top.v:81   == DEFAULT_MAGIC == bridge.MAGIC
PIXELS          = 640 * 480   # board_b_top.v:79

# app_wrrd sits in the sdr_clk (125 MHz) domain behind a 2-FF synchroniser and
# then needs a cycle of its own to leave reset. 2 sdr_clk edges = 16 ns < one
# clk_50m cycle (20 ns), so one clk_50m cycle suffices; require four so the
# claim survives a slower-than-modelled release without being a free pass.
SDR_RELEASE_MARGIN = 4


# ---------------------------------------------------------------------------
# board_b_top.v:374-383 -- the frame_rst stretcher, cycle-accurate.
# rst_n is held deasserted: only the fr_start response is under test.
# ---------------------------------------------------------------------------
class FrameRstStretcher:
    def __init__(self, width=FRAME_RST_WIDTH):
        self.width = width & 0xFF
        self.frst_cnt = 0

    @property
    def frame_rst(self):
        return self.frst_cnt != 0

    def edge(self, fr_start):
        """One posedge clk_50m. fr_start has priority, as in the RTL."""
        if fr_start:
            n = self.width
        elif self.frst_cnt != 0:
            n = self.frst_cnt - 1
        else:
            n = 0
        self.frst_cnt = n


# ---------------------------------------------------------------------------
# Co-simulation of the clk_50m chain rx_frame_to_bmp -> bmp_decode, with the
# stretcher riding alongside. Handshake copied verbatim from
# sim_rx_frame_to_bmp.run_cycles: bmp_decode's edge at iteration c samples the
# values rx_frame_to_bmp registered at iteration c-1, and the stretcher's edge at
# iteration c samples the same fr_start. All three therefore share one clock and
# one non-blocking discipline, so their post-edge values are directly comparable.
#
# Early stop: the loop ends once `stop_after` pixels have been decoded.
# ---------------------------------------------------------------------------
def cosim_decode(wire_bytes, stop_after=200, idle_aw=25,
                 frst_width=FRAME_RST_WIDTH):
    rx = RxFrameToBmp(sof_magic=DEFAULT_MAGIC, idle_aw=idle_aw)
    bmp = BmpDecode()
    frst = FrameRstStretcher(width=frst_width)
    for _ in range(2):
        rx.clk_edge(rst=True)
        bmp.clk_edge(rst=True)

    pixels = []
    dim = None
    dim_pulses = 0
    coincident = 0
    errs = 0
    s = ov = 0
    ob = 0
    start_iter = None
    first_valid_iter = None
    release_iter = None
    saw_rst = False

    for cyc, (iv, ib) in enumerate(wire_cycles(wire_bytes)):
        if s and start_iter is None:
            start_iter = cyc

        frst.edge(fr_start=bool(s))
        if frst.frame_rst:
            saw_rst = True
        elif saw_rst and release_iter is None:
            release_iter = cyc

        if s and ov:
            coincident += 1                    # CONTRACT VIOLATION if ever > 0
        bo = bmp.clk_edge(start=bool(s), in_valid=bool(ov), in_byte=ob)
        if bo['src_valid']:
            if first_valid_iter is None:
                first_valid_iter = cyc
            pixels.append(bo['src_pixel'])
        if bo['src_dim_valid']:
            dim_pulses += 1
            dim = (bo['src_width'], bo['src_height'])

        ro = rx.clk_edge(in_valid=iv, in_byte=ib)
        if ro['idle_timeout_err']:
            errs += 1
        s, ov, ob = ro['start'], ro['out_valid'], ro['out_byte']

        if len(pixels) >= stop_after:
            break

    return {
        'pixels': pixels, 'dim': dim, 'dim_pulses': dim_pulses,
        'coincident': coincident, 'errs': errs,
        'start_iter': start_iter, 'first_valid_iter': first_valid_iter,
        'release_iter': release_iter, 'frst_width': frst_width,
    }


# ---------------------------------------------------------------------------
# The 640x480 test image. R is deliberately nonzero and per-pixel distinct so
# that a wrong [23:0] slice cannot land on the right answer by accident.
# ---------------------------------------------------------------------------
def pix_640(r, c):
    return ((0x80 | (r & 0x7F)) & 0xFF, (c & 0xFF), ((r * 3 + c) & 0xFF))


def bmp_640x480():
    return make_bmp(DST_W, DST_H, pix_640)


# ---------------------------------------------------------------------------
# board_b_top.v:445 -- the explicit slice. A 32-bit SDRAM readback word carries
# garbage in [31:24]; the top hands only [23:0] to sdram_to_spi.
# ---------------------------------------------------------------------------
def rtl_slice(word32):
    return (word32 >> SLICE_LO) & 0xFFFFFF


def wrong_slice(word32):
    """The off-by-one-byte slice a rushed port would write: [31:8]."""
    return (word32 >> 8) & 0xFFFFFF


def usedw_sat(u):
    """board_b_top.v:393 -- saturate, do not truncate."""
    return USEDW_SAT if u >= USEDW_SAT else (u & 0x1FF)


def usedw_naive(u):
    """The truncation the 9-bit port width invites."""
    return u & 0x1FF


# ---------------------------------------------------------------------------
# Transcription guard: this model mirrors constants out of board_b_top.v, and a
# mirror that silently drifts from the RTL turns every test below into a lie
# about a design that no longer exists. Read the real file and compare.
# ---------------------------------------------------------------------------
def check_board_b_top_transcription():
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'boardB_eth_frontend', 'source_code', 'rtl', 'board_b_top.v')
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        rtl = f.read()
    checks = [
        ("FRAME_RST_WIDTH = 16", "integer FRAME_RST_WIDTH = 16"),
        ("PIXELS = 307200",      "integer PIXELS         = 307200"),
        ("SPI_MAGIC",            "[31:0]  SPI_MAGIC       = 32'hA55A_5AA5"),
        ("usedw saturate @511",  "(wr_fifo_usedw >= 12'd511) ? 9'd511 : wr_fifo_usedw[8:0]"),
        ("explicit [23:0] slice", ".Sdr_rd_dout  (Sdr_rd_dout_full[23:0])"),
        ("stretcher priority",   "else if(fr_start)"),
        ("STALL_THRESH 384",     ".STALL_THRESH (384)"),
        ("DST 640x480",          ".DST_W        (640)"),
    ]
    missing = [name for name, needle in checks if needle not in rtl]
    assert not missing, ("board_b_top.v no longer contains: %s -- this model has "
                         "drifted from the RTL and its results are meaningless"
                         % ", ".join(missing))


# ------------------------------------------------------------------ harness
def t(name, fn):
    try:
        fn()
        print(f"  PASS  {name}")
        return True
    except AssertionError as ex:
        print(f"  FAIL  {name}: {ex}")
        return False
    except Exception as ex:  # noqa
        print(f"  ERROR {name}: {type(ex).__name__}: {ex}")
        return False


def expect_fail(name, fn):
    """Negative control: fn MUST raise AssertionError. A clean return means the
    wrong behaviour went undetected -> the matching positive test is vacuous."""
    try:
        fn()
    except AssertionError:
        print(f"  PASS  {name} (negative control)")
        return True
    except Exception as ex:  # noqa
        print(f"  FAIL  {name}: negative control raised {type(ex).__name__}, "
              f"not AssertionError")
        return False
    print(f"  FAIL  {name}: negative control did NOT fail -> test is vacuous")
    return False


# ------------------------------------------------------------------ tests
def test_transcription():
    """The mirrored constants still match board_b_top.v on disk."""
    check_board_b_top_transcription()


def test_decode_chain_640x480():
    """(1) The full wire framing + BMP decode at the real competition geometry.
    The first 200 decoded pixels must equal the file's first 200 pixels exactly,
    dimensions must be reported once and correctly, and start must never be
    coincident with out_valid (the documented bmp_decode arm contract)."""
    data = bmp_640x480()
    r = cosim_decode(wire_frame(data), stop_after=200)
    assert r['coincident'] == 0, \
        f"start coincident with out_valid {r['coincident']}x (arm contract broken)"
    assert r['errs'] == 0, f"{r['errs']} idle-timeout errors mid-frame"
    assert r['dim_pulses'] == 1, f"src_dim_valid pulsed {r['dim_pulses']}x (want 1)"
    assert r['dim'] == (DST_W, DST_H), f"reported dims {r['dim']} != {(DST_W, DST_H)}"
    want = expected_pixels(DST_W, DST_H, pix_640)[:200]
    assert len(r['pixels']) == 200, f"decoded {len(r['pixels'])} pixels, wanted 200"
    for i, (g, e) in enumerate(zip(r['pixels'], want)):
        assert g == e, f"pixel[{i}] {g:#08x} != {e:#08x} (640-wide row stride wrong?)"
    # 640*3 = 1920 is already 4-byte aligned, so this BMP has NO row padding.
    # State that explicitly: if a future test image changes width, the padding
    # path becomes live again and this comment is the warning.
    assert ((DST_W * 3) % 4) == 0, "test image unexpectedly exercises row padding"


def test_scaler_is_identity_at_640x480():
    """(2) The linchpin. At the destination geometry the scaler must not move,
    duplicate or drop a single pixel: no letterbox, want_sx == k, want_sy == row,
    exactly DST_W*DST_H emissions."""
    assert SRC_W_MIN <= DST_W <= SRC_W_MAX, "640 outside scaler_nn source-width range"
    assert SRC_H_MIN <= DST_H <= SRC_H_MAX, "480 outside scaler_nn source-height range"
    (problems, tw, th, off_x, off_y, max_sx, max_sy,
     emitted) = geometric_check(DST_W, DST_H)
    assert not problems, "identity geometry reported problems: %s" % problems
    assert (tw, th) == (DST_W, DST_H), f"target {tw}x{th} != {DST_W}x{DST_H}"
    assert (off_x, off_y) == (0, 0), f"letterbox offset ({off_x},{off_y}) != (0,0)"
    assert max_sx == DST_W - 1, f"max sx {max_sx} != {DST_W - 1}"
    assert max_sy == DST_H - 1, f"max sy {max_sy} != {DST_H - 1}"
    assert emitted == PIXELS, f"emitted {emitted} != {PIXELS}"
    # MAX_UPSCALE must not clip a 640x480 source: min(640*4,640)==640 only
    # because the source already equals the destination. Spell out the identity.
    assert min(DST_W * MAX_UPSCALE, DST_W) == DST_W
    assert min(DST_H * MAX_UPSCALE, DST_H) == DST_H


def test_transport_preserves_decoded_pixels():
    """(3) Decoded pixels -> 32-bit SDRAM words with garbage in [31:24] -> the
    explicit [23:0] slice -> sdram_to_spi -> the wire. What Board A samples must
    be the decoded pixels themselves, unmasked and unshifted."""
    n = 64
    r = cosim_decode(wire_frame(bmp_640x480()), stop_after=n)
    decoded = r['pixels']
    assert len(decoded) == n, f"only decoded {len(decoded)} pixels"
    assert len(set(decoded)) == n, "test pixels are not distinct; a wrong slice " \
                                   "could pass by coincidence"

    # sdram_top hands back 32 bits; DATA_WIDTH=32 and the pixel is the low 24.
    words32 = [(((0x5A + i * 7) & 0xFF) << 24) | p for i, p in enumerate(decoded)]
    assert all((w >> 24) != 0 for w in words32), "garbage byte is zero: test is weak"
    sliced = [rtl_slice(w) for w in words32]

    br, rb, fifo = bridge.run(read_pixels=sliced, gate=64, fifo_depth=128,
                              sck_div=8, bridge_pixels=n)
    assert rb.done, "readback did not finish"
    assert fifo.empty, f"FIFO not drained (wrusedw={fifo.wrusedw})"
    assert not fifo.overflowed, "FIFO overflowed"
    assert br.frame_start_count == 1, f"frame_start fired {br.frame_start_count}x"
    assert br.frame_done_count == 1, f"frame_done fired {br.frame_done_count}x"

    # Wire framing, self-consistent with what we handed the bridge.
    check_contract(br.bits, bridge.MAGIC, sliced)
    # The real claim: against the UNMASKED decoded truth. check_contract masks
    # its expectation, so it alone would also pass a wrong slice.
    magic, got = reconstruct(br.bits)
    assert magic == SPI_MAGIC == DEFAULT_MAGIC == bridge.MAGIC, \
        f"MAGIC {magic:#010x} disagrees across board_b_top / rx framing / bridge"
    assert got == decoded, "wire pixels != decoded pixels: the [23:0] slice or " \
                           "the transport moved bits"


def neg_wrong_slice_corrupts():
    """Negative control for (3): the plausible off-by-one-byte slice [31:8] must
    be caught. House idiom -- assert the UNSOUND claim so it is rejected."""
    n = 64
    decoded = cosim_decode(wire_frame(bmp_640x480()), stop_after=n)['pixels']
    words32 = [(((0x5A + i * 7) & 0xFF) << 24) | p for i, p in enumerate(decoded)]
    bad = [wrong_slice(w) for w in words32]
    if bad == decoded:
        raise RuntimeError("the [31:8] slice reproduced the pixels; the control "
                           "cannot exercise the failure mode")
    br, rb, fifo = bridge.run(read_pixels=bad, gate=64, fifo_depth=128,
                              sck_div=8, bridge_pixels=n)
    if not rb.done or not fifo.empty:
        raise RuntimeError(f"transport itself failed (done={rb.done}, "
                           f"empty={fifo.empty}); not testing the slice")
    _, got = reconstruct(br.bits)
    assert got == decoded, "[31:8] slice came out correct -> the positive test " \
                           "cannot distinguish a right slice from a wrong one"


def test_stretcher_releases_before_first_pixel():
    """(4) app_wrrd must be out of reset, with margin for its sdr_clk crossing,
    before the scaler's earliest possible input pixel. Measured, not assumed:
    this uses bmp_decode's first src_valid, which is EARLIER than the scaler's
    first write by the scaler's own park-and-fill latency, so the assertion is
    conservative in the safe direction."""
    r = cosim_decode(wire_frame(bmp_640x480()), stop_after=8,
                     frst_width=FRAME_RST_WIDTH)
    assert r['start_iter'] is not None, "fr_start never fired"
    assert r['release_iter'] is not None, \
        f"frame_rst never deasserted within the window (width={r['frst_width']})"
    assert r['first_valid_iter'] is not None, "bmp_decode never emitted a pixel"
    margin = r['first_valid_iter'] - r['release_iter']
    print(f"        fr_start@{r['start_iter']}  frame_rst released@{r['release_iter']}  "
          f"first src_valid@{r['first_valid_iter']}  margin={margin} cyc "
          f"(need >= {SDR_RELEASE_MARGIN})")
    assert margin >= SDR_RELEASE_MARGIN, \
        (f"app_wrrd released only {margin} clk_50m cycles before the first "
         f"decoded pixel (need >= {SDR_RELEASE_MARGIN} for the sdr_clk 2-FF "
         f"crossing) -- FRAME_RST_WIDTH={FRAME_RST_WIDTH} is too long")
    # And it must actually have been a multi-cycle pulse, or the synchroniser in
    # sdram_top can miss it entirely -- the reason the stretcher exists.
    held = r['release_iter'] - r['start_iter']
    assert held >= FRAME_RST_WIDTH, \
        f"frame_rst held only {held} cycles, want >= {FRAME_RST_WIDTH}"


def neg_stretcher_too_long():
    """Negative control for (4): a stretcher long enough to still be holding
    app_wrrd in reset when the first pixel arrives must be caught."""
    width = 100
    r = cosim_decode(wire_frame(bmp_640x480()), stop_after=8, frst_width=width)
    if r['start_iter'] is None or r['first_valid_iter'] is None:
        raise RuntimeError("model never produced both fr_start and a pixel")
    if r['release_iter'] is not None and \
            r['release_iter'] - r['start_iter'] < width:
        raise RuntimeError(f"width={width} stretcher held only "
                           f"{r['release_iter'] - r['start_iter']} cycles")
    margin = (r['first_valid_iter'] - r['release_iter']) \
        if r['release_iter'] is not None else -10 ** 6
    assert margin >= SDR_RELEASE_MARGIN, \
        f"FRAME_RST_WIDTH={width} still released app_wrrd {margin} cycles before " \
        "the first pixel -> the timing test cannot detect an over-long stretcher"


def test_usedw_saturation():
    """(5) The 12-bit -> 9-bit narrowing must preserve the stall decision across
    the whole real occupancy range, i.e. `saturated >= STALL_THRESH` exactly when
    `true >= STALL_THRESH`. That equivalence -- not the value itself -- is what
    keeps the scaler from overrunning a full FIFO."""
    top = 4096                      # fifo_sdr_data_2 ADDR_WIDTH_W=12
    prev = -1
    for u in range(top):
        s = usedw_sat(u)
        want = min(u, USEDW_SAT)
        assert s == want, f"sat({u}) = {s}, want {want}"
        assert s >= prev, f"sat() not monotonic at u={u}: {s} after {prev}"
        prev = s
        assert (s >= STALL_THRESH) == (u >= STALL_THRESH), \
            f"stall decision diverged at u={u}: saturated {s} vs true {u} " \
            f"(STALL_THRESH={STALL_THRESH})"
    # The saturation ceiling must sit above the threshold, else the scaler would
    # be permanently stalled and no frame would ever complete.
    assert USEDW_SAT >= STALL_THRESH, \
        f"saturating at {USEDW_SAT} <= STALL_THRESH {STALL_THRESH} stalls forever"
    # And a 512-deep-FIFO-era occupancy reading must pass through untouched, so
    # scaler_nn's existing STALL_THRESH=384 sizing note still applies verbatim.
    for u in (0, 1, 383, 384, 385, 510, 511):
        assert usedw_sat(u) == u, f"sat({u}) altered an in-range reading"


def neg_naive_truncation_unstalls():
    """Negative control for (5): a plain [8:0] truncation wraps 512 -> 0 and
    would UN-STALL the scaler against a completely full FIFO. Assert the unsound
    claim so it is rejected."""
    top = 4096
    viol = [u for u in range(top)
            if (usedw_naive(u) >= STALL_THRESH) != (u >= STALL_THRESH)]
    if not viol:
        raise RuntimeError("naive truncation never diverged; the control cannot "
                           "exercise the failure mode")
    print(f"        naive [8:0] truncation diverges at {len(viol)} occupancies, "
          f"first {viol[0]} (reads {usedw_naive(viol[0])} for a true {viol[0]})")
    assert all((usedw_naive(u) >= STALL_THRESH) == (u >= STALL_THRESH)
               for u in range(top)), \
        "naive truncation preserved the stall decision -> the saturation glue is " \
        "untested by the positive case"


def test_magic_agreement():
    """One number is used in three places that must not drift: the SPI frame
    MAGIC Board A hunts for, the SOF magic rx_frame_to_bmp accepts on the wire,
    and board_b_top's SPI_MAGIC parameter. A mismatch anywhere yields a frame
    that is received perfectly and then silently rejected."""
    assert SPI_MAGIC == DEFAULT_MAGIC, \
        f"board_b_top SPI_MAGIC {SPI_MAGIC:#010x} != rx wire SOF {DEFAULT_MAGIC:#010x}"
    assert SPI_MAGIC == bridge.MAGIC, \
        f"board_b_top SPI_MAGIC {SPI_MAGIC:#010x} != bridge frame MAGIC {bridge.MAGIC:#010x}"
    assert SPI_MAGIC == 0xA55A5AA5


# ------------------------------------------------------------------ main
def main():
    print("board_b_top.v integration model   640x480   FRAME_RST_WIDTH=%d   "
          "usedw saturates at %d   Sdr_rd_dout slice [%d:%d]"
          % (FRAME_RST_WIDTH, USEDW_SAT, SLICE_LO + 23, SLICE_LO))
    print("=" * 78)

    results = [
        t("rtl transcription guard",              test_transcription),
        t("magic agrees across the three users",  test_magic_agreement),
        t("decode chain at real 640x480 geometry", test_decode_chain_640x480),
        t("scaler_nn is identity at 640x480",     test_scaler_is_identity_at_640x480),
        t("transport preserves decoded pixels",   test_transport_preserves_decoded_pixels),
        t("frame_rst releases before first pixel", test_stretcher_releases_before_first_pixel),
        t("usedw saturation keeps stall decision", test_usedw_saturation),
        expect_fail("wrong [31:8] slice is caught",    neg_wrong_slice_corrupts),
        expect_fail("over-long stretcher is caught",   neg_stretcher_too_long),
        expect_fail("naive [8:0] truncation is caught", neg_naive_truncation_unstalls),
    ]

    print("=" * 78)
    bad = results.count(False)
    print("%d/%d passed%s" % (len(results) - bad, len(results),
                              "" if not bad else "  -- %d FAILED" % bad))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
