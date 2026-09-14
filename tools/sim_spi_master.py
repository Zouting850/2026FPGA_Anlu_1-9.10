#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cycle-accurate model + testbench for boardB_eth_frontend spi_master_tx.v.

There is no Verilog simulator on this machine, so per project convention the RTL
is verified by a Python model that mirrors it register-for-register with
non-blocking (read-all-then-commit) semantics. The testbench drives the model,
captures the MOSI bitstream at each SCK RISING edge exactly as a Board A SPI
slave (mode 0) would, and checks the frame contract:

    CS_n low -> 32-bit MAGIC -> PIXELS x 24-bit pixels (all MSB-first) -> CS_n high

Negative controls are included so a PASS cannot be a false positive from a
degenerate stimulus: the same captured bitstream is re-interpreted with the
WRONG magic, WRONG bit order (LSB-first) and WRONG bit count, and each wrong
interpretation MUST fail. If a negative control ever passes, the test is vacuous.

Run:  python tools/sim_spi_master.py
"""
import sys

S_IDLE, S_MAGIC, S_PIX_LOAD, S_PIX_SHIFT, S_END = 0, 1, 2, 3, 4


def _clog2(n):
    """Mirror Verilog $clog2: smallest w with 2**w >= n (w>=0)."""
    w = 0
    while (1 << w) < n:
        w += 1
    return w


class SpiMasterTx:
    """Direct mirror of spi_master_tx.v. One clk() == one posedge clk."""

    def __init__(self, sck_div=8, pixels=307200, magic=0xA55A5AA5):
        assert sck_div % 2 == 0 and sck_div >= 4, "SCK_DIV must be even and >=4"
        self.SCK_DIV = sck_div
        self.HALF = sck_div // 2
        self.PIXELS = pixels
        self.MAGIC = magic & 0xFFFFFFFF
        # width localparams mirror the RTL
        self.DIVW = max(1, _clog2(sck_div))
        self.PIXW = max(1, _clog2(pixels + 1))
        self.reset()

    def reset(self):
        self.state = S_IDLE
        self.div = 0
        self.bit_cnt = 0
        self.bit_total = 0
        self.shift = 0
        self.pix_cnt = 0
        self.pixel_ready = 0
        self.spi_sck = 0
        self.spi_mosi = 0
        self.spi_cs_n = 1

    @property
    def busy(self):
        return int(self.state != S_IDLE)

    def clk(self, frame_start=0, pixel_valid=0, pixel=0):
        """Advance one clk. Returns (sck, mosi, cs_n, pixel_ready, busy) AFTER
        the edge (i.e. the newly committed register values)."""
        # ---- compute next state from CURRENT state (non-blocking) ----
        n_state = self.state
        n_div = self.div
        n_bit_cnt = self.bit_cnt
        n_bit_total = self.bit_total
        n_shift = self.shift
        n_pix_cnt = self.pix_cnt
        n_prdy = 0
        n_sck = self.spi_sck
        n_mosi = self.spi_mosi
        n_cs = self.spi_cs_n

        if self.state == S_IDLE:
            n_cs = 1
            n_sck = 0
            n_div = 0
            n_pix_cnt = 0
            if frame_start:
                n_cs = 0
                n_mosi = (self.MAGIC >> 31) & 1
                n_shift = self.MAGIC
                n_bit_total = 32
                n_bit_cnt = 0
                n_div = 0
                n_state = S_MAGIC

        elif self.state in (S_MAGIC, S_PIX_SHIFT):
            if self.div == 0:
                n_mosi = (self.shift >> 31) & 1
            if self.div == self.HALF - 1:
                n_sck = 1
            if self.div == self.SCK_DIV - 1:
                n_sck = 0
                n_shift = (self.shift << 1) & 0xFFFFFFFF
                n_bit_cnt = self.bit_cnt + 1
                n_div = 0
                if self.bit_cnt == self.bit_total - 1:
                    if self.state == S_PIX_SHIFT:
                        n_pix_cnt = self.pix_cnt + 1
                    n_state = S_PIX_LOAD
            else:
                n_div = self.div + 1

        elif self.state == S_PIX_LOAD:
            n_sck = 0
            n_div = 0
            n_bit_cnt = 0
            if self.pix_cnt == self.PIXELS:
                n_state = S_END
            elif pixel_valid:
                n_prdy = 1
                n_shift = ((pixel & 0xFFFFFF) << 8) & 0xFFFFFFFF
                n_bit_total = 24
                n_mosi = (pixel >> 23) & 1
                n_state = S_PIX_SHIFT

        elif self.state == S_END:
            n_cs = 1
            n_sck = 0
            n_mosi = 0
            n_state = S_IDLE

        else:
            n_state = S_IDLE

        # ---- commit (all at once) ----
        self.state = n_state
        self.div = n_div
        self.bit_cnt = n_bit_cnt
        self.bit_total = n_bit_total
        self.shift = n_shift
        self.pix_cnt = n_pix_cnt
        self.pixel_ready = n_prdy
        self.spi_sck = n_sck
        self.spi_mosi = n_mosi
        self.spi_cs_n = n_cs
        return self.spi_sck, self.spi_mosi, self.spi_cs_n, self.pixel_ready, self.busy


def run_frame(dut, pixels, stall_at=None, stall_len=3):
    """Drive one full frame through the DUT.

    pixels    : list of 24-bit ints, len must == dut.PIXELS
    stall_at  : set of pixel indices at which pixel_valid is held low for
                stall_len clks before the pixel is offered (tests backpressure)
    Returns (bits, consumed, cycles) where bits is the MOSI bitstream captured
    at SCK rising edges (what the slave sees), consumed is the pixel count the
    DUT pulled, cycles is the clk count.
    """
    assert len(pixels) == dut.PIXELS, "stimulus length must equal PIXELS"
    stall_at = set(stall_at or [])
    bits = []
    idx = 0
    n = len(pixels)
    prev_sck = dut.spi_sck
    stalled_for = set()
    stall_remaining = 0
    frame_start = 1
    expected_bits = 32 + n * 24
    cycles = 0
    max_cycles = expected_bits * dut.SCK_DIV + 1000 + len(stall_at) * (stall_len + 2) * dut.SCK_DIV

    while cycles < max_cycles:
        fs = frame_start
        frame_start = 0  # one-shot pulse

        if idx < n and idx in stall_at and idx not in stalled_for:
            stall_remaining = stall_len
            stalled_for.add(idx)
        if stall_remaining > 0:
            pv, px = 0, 0
            stall_remaining -= 1
        else:
            pv = 1 if idx < n else 0
            px = pixels[idx] if idx < n else 0

        sck, mosi, cs_n, prdy, busy = dut.clk(fs, pv, px)
        if prdy:
            idx += 1
        if sck == 1 and prev_sck == 0:   # rising edge: slave samples MOSI
            bits.append(mosi)
        prev_sck = sck
        cycles += 1

        if len(bits) >= expected_bits and not busy:
            break

    return bits, idx, cycles


def _bits_to_int(bs):
    return int(''.join(str(b) for b in bs), 2) if bs else 0


def reconstruct(bits):
    """Decode the captured stream per the contract. Returns (magic, pixels)."""
    magic = _bits_to_int(bits[:32])
    pbits = bits[32:]
    pixels = [_bits_to_int(pbits[i * 24:(i + 1) * 24]) for i in range(len(pbits) // 24)]
    return magic, pixels


def check_contract(bits, magic, pixels):
    expected_bits = 32 + len(pixels) * 24
    assert len(bits) == expected_bits, f"bit count {len(bits)} != expected {expected_bits}"
    got_magic, got_pixels = reconstruct(bits)
    assert got_magic == magic, f"MAGIC {got_magic:#010x} != {magic:#010x}"
    assert len(got_pixels) == len(pixels), f"pixel count {len(got_pixels)} != {len(pixels)}"
    for i, (g, e) in enumerate(zip(got_pixels, pixels)):
        assert g == (e & 0xFFFFFF), f"pixel[{i}] {g:#08x} != {e:#08x}"


# ------------------------------------------------------------------ tests
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
    """Negative control: fn MUST raise AssertionError. If it returns cleanly the
    wrong interpretation unexpectedly matched -> the test would be vacuous."""
    try:
        fn()
    except AssertionError:
        print(f"  PASS  {name} (negative control: wrong interpretation rejected)")
        return True
    except Exception as ex:  # noqa
        print(f"  FAIL  {name}: negative control raised {type(ex).__name__} not AssertionError")
        return False
    print(f"  FAIL  {name}: negative control did NOT fail -> test is vacuous")
    return False


def test_basic(div, npix, seed):
    pixels = [((seed + i * 0x123457) & 0xFFFFFF) for i in range(npix)]
    # ensure at least one non-palindromic, non-trivial pixel for bit-order control
    pixels[0] = 0xA5C31E
    dut = SpiMasterTx(sck_div=div, pixels=npix)
    bits, consumed, cycles = run_frame(dut, pixels)
    assert consumed == npix, f"consumed {consumed} != {npix}"
    check_contract(bits, dut.MAGIC, pixels)
    # frame time sanity
    sck_hz = 50_000_000 / div
    frame_s = (32 + npix * 24) / sck_hz
    print(f"        (div={div} -> SCK={sck_hz/1e6:.3f}MHz, {cycles} clks, "
          f"640x480 frame ~{(32+307200*24)/sck_hz:.3f}s)")


def test_backpressure():
    npix = 6
    pixels = [0xA5C31E, 0x000001, 0xFFFFFF, 0x123456, 0x800000, 0x7FFFFF]
    dut = SpiMasterTx(sck_div=8, pixels=npix)
    bits, consumed, cycles = run_frame(dut, pixels, stall_at={1, 3, 5}, stall_len=5)
    assert consumed == npix, f"consumed {consumed} != {npix}"
    check_contract(bits, dut.MAGIC, pixels)


def test_single_pixel():
    dut = SpiMasterTx(sck_div=4, pixels=1)
    bits, consumed, _ = run_frame(dut, [0xA5C31E])
    assert consumed == 1
    check_contract(bits, dut.MAGIC, [0xA5C31E])


def test_cs_frames_one_window():
    """CS_n must be low for exactly one contiguous window covering all bits."""
    npix = 3
    pixels = [0xA5C31E, 0x123456, 0x00FF00]
    dut = SpiMasterTx(sck_div=8, pixels=npix)
    cs_low_clks = 0
    idx = 0
    prev_sck = dut.spi_sck
    transitions = 0
    prev_cs = 1
    frame_start = 1
    expected_bits = 32 + npix * 24
    bits = 0
    guard = expected_bits * dut.SCK_DIV + 500
    c = 0
    while c < guard:
        pv = 1 if idx < npix else 0
        px = pixels[idx] if idx < npix else 0
        sck, mosi, cs_n, prdy, busy = dut.clk(frame_start, pv, px)
        frame_start = 0
        if prdy:
            idx += 1
        if sck == 1 and prev_sck == 0:
            bits += 1
        prev_sck = sck
        if cs_n == 0:
            cs_low_clks += 1
        if cs_n != prev_cs:
            transitions += 1
        prev_cs = cs_n
        c += 1
        if bits >= expected_bits and not busy:
            break
    assert bits == expected_bits, f"captured {bits} != {expected_bits}"
    # exactly one falling + one rising = 2 transitions, and CS low spanned the frame
    assert transitions == 2, f"CS_n transitions {transitions} != 2 (one frame window)"
    assert cs_low_clks > 0


def main():
    print("spi_master_tx.v cycle-accurate model -- self-check\n")
    results = []

    print("positive tests:")
    results.append(t("basic div=8 npix=4", lambda: test_basic(8, 4, 0x1)))
    results.append(t("basic div=4 npix=4 (fast SCK)", lambda: test_basic(4, 4, 0x2)))
    results.append(t("basic div=10 npix=5 (non-pow2 div)", lambda: test_basic(10, 5, 0x3)))
    results.append(t("single pixel (npix=1)", test_single_pixel))
    results.append(t("backpressure stalls (no bit loss)", test_backpressure))
    results.append(t("CS_n frames exactly one window", test_cs_frames_one_window))

    print("\nnegative controls (these MUST fail to be valid):")
    # capture a known-good stream, then assert wrong interpretations are rejected
    pixels = [0xA5C31E, 0x123456, 0x00FF00, 0xDEADBE & 0xFFFFFF]
    dut = SpiMasterTx(sck_div=8, pixels=len(pixels))
    good_bits, _, _ = run_frame(dut, pixels)

    def wrong_magic():
        check_contract(good_bits, 0xDEADBEEF, pixels)   # wrong magic must fail
    results.append(expect_fail("wrong MAGIC rejected", wrong_magic))

    def wrong_order():
        # reinterpret first pixel LSB-first; must NOT equal the MSB-first value
        pbits = good_bits[32:32 + 24]
        lsb = _bits_to_int(list(reversed(pbits)))
        msb = _bits_to_int(pbits)
        assert lsb != msb, "pixel is bit-palindromic; order control is vacuous"
        assert msb == (pixels[0] & 0xFFFFFF), "MSB-first must match stimulus"
        assert lsb == (pixels[0] & 0xFFFFFF), "LSB-first must NOT match stimulus"
    results.append(expect_fail("LSB-first order rejected", wrong_order))

    def wrong_count():
        check_contract(good_bits[:-1], dut.MAGIC, pixels)  # one bit short
    results.append(expect_fail("short bitstream rejected", wrong_count))

    def wrong_pixel():
        bad = list(pixels)
        bad[2] = (bad[2] ^ 0xFF) & 0xFFFFFF
        check_contract(good_bits, dut.MAGIC, bad)
    results.append(expect_fail("corrupted pixel rejected", wrong_pixel))

    print("\nreal-frame analytic (not simulated, 7.37 Mbit):")
    for div in (4, 8):
        sck = 50_000_000 / div
        print(f"  SCK_DIV={div}: SCK={sck/1e6:.2f}MHz  bits={32+307200*24}  "
              f"frame={(32+307200*24)/sck:.3f}s")

    ok = all(results)
    print(f"\n{'==== ALL PASS ====' if ok else '==== FAILURES ===='} "
          f"({sum(results)}/{len(results)})")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
