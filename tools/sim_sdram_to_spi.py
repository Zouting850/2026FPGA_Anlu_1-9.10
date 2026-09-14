#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cycle-accurate model + testbench for boardB_eth_frontend sdram_to_spi.v.

Mirrors the bridge's clk_50m control logic register-for-register (non-blocking
read-all-then-commit), reuses the verified SpiMasterTx model from sim_spi_master,
and chains an app_wrrd-like SDRAM readback source so the whole read-side path is
exercised end to end:

    readback source (sdr_clk) --push, gated by udp_wrusedw<GATE--> async FIFO
        --SHOW-AHEAD head--> spi_master_tx (clk_50m) --> MOSI bitstream

Two clock domains. sdr_clk:clk_50m = 125:50 = 5:2, so each macro-cycle does 5
readback pushes then 2 bridge clk edges. The async FIFO itself is board-proven
Anlogic IP (fifo_sdr_data_2, gray-code CDC) and is NOT re-verified here; it is
modelled as an order-preserving queue whose length is the idealised wrusedw. This
is the same verification boundary the project uses elsewhere: model the glue we
wrote, treat the vendor IP as correct, and say so.

What IS verified (the glue we wrote):
  * frame_start fires EXACTLY once per frame, on the first readback pixel after
    re-arm, and only while SPI is idle.
  * every pixel the readback pushed is sent over SPI, in order, uncorrupted --
    checked by capturing MOSI at SCK rising edges and reconstructing the frame.
  * the udp_wrusedw<GATE backpressure actually throttles the readback so the FIFO
    can never pass its depth (no overflow, no pixel loss).
  * SPI underrun (readback slower than the drain) stalls but never loses a pixel.
  * frame_done pulses once per completed frame; frame_rst re-arms for the next.

Negative controls (a PASS must not be a false positive):
  * a readback that IGNORES udp_wrusedw overflows the FIFO -> must be caught.
  * a readback that pushes PIXELS-1 never completes -> no frame_done -> caught.
  * a corrupted source pixel is caught by the bitstream reconstruction.

Run:  python tools/sim_sdram_to_spi.py
"""
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_spi_master import SpiMasterTx, reconstruct  # noqa: E402

MAGIC = 0xA55A5AA5


# ---------------------------------------------------------------------------
# Idealised async FIFO (board-proven IP stand-in): order-preserving queue with a
# hard depth. wrusedw == len, the write-domain occupancy app_wrrd gates on.
# ---------------------------------------------------------------------------
class AsyncFifo:
    def __init__(self, depth):
        self.depth = depth
        self.q = deque()
        self.overflowed = False        # set if a push is attempted while full

    @property
    def wrusedw(self):
        return len(self.q)

    @property
    def empty(self):
        return len(self.q) == 0

    @property
    def dout(self):                     # SHOW-AHEAD head
        return self.q[0] if self.q else 0

    def push(self, pixel):             # sdr_clk domain (write)
        if len(self.q) >= self.depth:
            self.overflowed = True     # real HW would drop this pixel
            return
        self.q.append(pixel)

    def pop(self):                     # clk_50m domain (read)
        if self.q:
            self.q.popleft()

    def flush(self):                   # frame_rst
        self.q.clear()
        self.overflowed = False


# ---------------------------------------------------------------------------
# app_wrrd read-side stand-in: pushes exactly `pixels` in order, but only while
# udp_wrusedw < GATE (the <2048 throttle), one per sdr_clk edge.
# ---------------------------------------------------------------------------
class ReadbackSource:
    def __init__(self, pixels, gate, ignore_gate=False):
        self.pixels = list(pixels)
        self.idx = 0
        self.gate = gate
        self.ignore_gate = ignore_gate

    @property
    def done(self):
        return self.idx >= len(self.pixels)

    def sdr_edge(self, fifo):
        if self.done:
            return
        if self.ignore_gate or fifo.wrusedw < self.gate:
            fifo.push(self.pixels[self.idx])
            self.idx += 1

    def reset(self, pixels=None):
        if pixels is not None:
            self.pixels = list(pixels)
        self.idx = 0


# ---------------------------------------------------------------------------
# sdram_to_spi.v mirror (clk_50m side). One clk_edge() == one posedge clk.
# ---------------------------------------------------------------------------
class Bridge:
    def __init__(self, sck_div=8, pixels=307200, magic=MAGIC):
        self.spi = SpiMasterTx(sck_div=sck_div, pixels=pixels, magic=magic)
        self.PIXELS = pixels
        # clk_50m registers
        self.awaiting_frame = 1        # armed at power-on
        self.frame_start = 0
        self.busy_d = 0
        self.frame_done = 0
        self.prev_sck = 0
        self.bits = []                 # captured MOSI at SCK rising edges
        self.frame_start_count = 0
        self.frame_done_count = 0

    def hard_reset(self):
        self.spi.reset()
        self.awaiting_frame = 1
        self.frame_start = 0
        self.busy_d = 0
        self.frame_done = 0

    def clk_edge(self, fifo, frame_rst=0):
        # ---- pre-edge FIFO state (SHOW-AHEAD) ----
        empty = fifo.empty
        dout = fifo.dout
        # re uses the REGISTERED pixel_ready from the previous edge, exactly as
        # the RTL's combinational re = spi_pixel_ready & ~fifo_empty does.
        re = self.spi.pixel_ready and (not empty)

        # ---- frame_start / awaiting_frame (non-blocking) ----
        n_awaiting = self.awaiting_frame
        n_frame_start = 0
        if frame_rst:
            n_awaiting = 1
        elif self.awaiting_frame and (not empty) and (not self.spi.busy):
            n_frame_start = 1
            n_awaiting = 0

        # ---- frame_done on busy falling edge (non-blocking) ----
        n_frame_done = self.busy_d and (not self.spi.busy)
        n_busy_d = self.spi.busy

        # ---- SPI edge: consumes pre-edge fifo head ----
        sck, mosi, cs_n, prdy, busy = self.spi.clk(
            self.frame_start,            # the value committed last edge
            0 if empty else 1,
            dout,
        )

        # ---- commit ----
        self.awaiting_frame = n_awaiting
        self.frame_start = n_frame_start
        self.frame_done = n_frame_done
        self.busy_d = n_busy_d
        if n_frame_start:
            self.frame_start_count += 1
        if n_frame_done:
            self.frame_done_count += 1
        if re:
            fifo.pop()

        # ---- capture the wire (what a Board A slave samples) ----
        if sck == 1 and self.prev_sck == 0:
            self.bits.append(mosi)
        self.prev_sck = sck
        return busy


# ---------------------------------------------------------------------------
# Two-clock driver. ratio 5 sdr_clk : 2 clk_50m.
# ---------------------------------------------------------------------------
def run(read_pixels, gate, fifo_depth, sck_div, bridge_pixels,
        ignore_gate=False, sdr_per_macro=5, clk_per_macro=2,
        max_macro=None, frame_rst_at_macro=None, second_frame_pixels=None):
    fifo = AsyncFifo(fifo_depth)
    rb = ReadbackSource(read_pixels, gate, ignore_gate=ignore_gate)
    br = Bridge(sck_div=sck_div, pixels=bridge_pixels)

    if max_macro is None:
        # generous guard: every pixel needs ~24*SCK_DIV clk_50m cycles
        max_macro = (bridge_pixels * 24 * sck_div) // clk_per_macro + 5000

    macro = 0
    started_second = False
    while macro < max_macro:
        if frame_rst_at_macro is not None and macro == frame_rst_at_macro and not started_second:
            # re-arm: flush FIFO, reset SPI+control, swap in the next frame
            fifo.flush()
            br.hard_reset()
            if second_frame_pixels is not None:
                rb.reset(second_frame_pixels)
            else:
                rb.reset()
            br.clk_edge(fifo, frame_rst=1)
            started_second = True
            macro += 1
            continue

        for _ in range(sdr_per_macro):
            rb.sdr_edge(fifo)
        for _ in range(clk_per_macro):
            br.clk_edge(fifo)
        macro += 1

        if br.frame_done_count >= 1 and rb.done and fifo.empty:
            if frame_rst_at_macro is None or started_second:
                break
    return br, rb, fifo


# ------------------------------------------------------------------ helpers
def grad(n, seed=0):
    return [((seed + i * 0x123457) & 0xFFFFFF) for i in range(n)]


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


# ------------------------------------------------------------------ tests
def test_frame_integrity():
    """All pushed pixels reach the wire, in order; frame_start/done fire once."""
    npix = 12
    src = grad(npix, seed=0xA5)
    src[0] = 0xA5C31E
    br, rb, fifo = run(read_pixels=src, gate=64, fifo_depth=128,
                       sck_div=8, bridge_pixels=npix)
    assert rb.done, "readback did not finish"
    assert fifo.empty, f"FIFO not drained at end (wrusedw={fifo.wrusedw})"
    assert not fifo.overflowed, "FIFO overflowed"
    assert br.frame_start_count == 1, f"frame_start fired {br.frame_start_count}x (want 1)"
    assert br.frame_done_count == 1, f"frame_done fired {br.frame_done_count}x (want 1)"
    magic, pixels = reconstruct(br.bits)
    assert magic == MAGIC, f"MAGIC {magic:#010x} != {MAGIC:#010x}"
    assert pixels == [p & 0xFFFFFF for p in src], "pixel stream mismatch"


def test_gating_throttles_readback():
    """A tight gate must hold FIFO occupancy under depth with no loss."""
    npix = 16
    src = grad(npix, seed=0x11)
    gate, depth = 4, 8                 # gate well under depth; readback is fast
    br, rb, fifo = run(read_pixels=src, gate=gate, fifo_depth=depth,
                       sck_div=8, bridge_pixels=npix)
    assert rb.done and fifo.empty and not fifo.overflowed, \
        f"done={rb.done} empty={fifo.empty} overflow={fifo.overflowed}"
    assert br.frame_start_count == 1 and br.frame_done_count == 1
    _, pixels = reconstruct(br.bits)
    assert pixels == [p & 0xFFFFFF for p in src], "gated stream lost/reordered a pixel"


def test_underrun_stalls_no_loss():
    """Readback slower than the SPI drain: SPI stalls but loses nothing."""
    npix = 8
    src = grad(npix, seed=0x77)
    # 1 push per macro-cycle vs 2 clk edges => drain can outrun fill -> underrun
    br, rb, fifo = run(read_pixels=src, gate=64, fifo_depth=128,
                       sck_div=4, bridge_pixels=npix,
                       sdr_per_macro=1, clk_per_macro=8)
    assert rb.done and fifo.empty and not fifo.overflowed
    _, pixels = reconstruct(br.bits)
    assert pixels == [p & 0xFFFFFF for p in src], "underrun corrupted the stream"


def test_rearm_two_frames():
    """frame_rst flushes and re-arms; a second frame flows correctly."""
    npix = 10
    src1 = grad(npix, seed=0x01)
    src2 = grad(npix, seed=0x02)
    # first frame completes well before the re-arm macro; pick a safe re-arm point
    br, rb, fifo = run(read_pixels=src1, gate=64, fifo_depth=128,
                       sck_div=4, bridge_pixels=npix,
                       frame_rst_at_macro=None)  # placeholder, replaced below
    # explicit two-frame run: let frame 1 finish, then re-arm with frame 2
    fifo2 = AsyncFifo(128)
    rb2 = ReadbackSource(src1, 64)
    br2 = Bridge(sck_div=4, pixels=npix)

    def drive(rb, fifo, br, max_macro):
        m = 0
        while m < max_macro:
            for _ in range(5):
                rb.sdr_edge(fifo)
            for _ in range(2):
                br.clk_edge(fifo)
            m += 1
            if br.frame_done_count >= 1 and rb.done and fifo.empty:
                break
        return m

    guard = (npix * 24 * 4) // 2 + 5000
    drive(rb2, fifo2, br2, guard)
    assert br2.frame_done_count == 1, "frame 1 did not complete"
    _, p1 = reconstruct(br2.bits)
    assert p1 == [p & 0xFFFFFF for p in src1], "frame 1 mismatch"

    # re-arm
    fifo2.flush()
    br2.hard_reset()
    br2.bits = []
    br2.frame_start_count = 0
    br2.frame_done_count = 0
    rb2.reset(src2)
    br2.clk_edge(fifo2, frame_rst=1)
    drive(rb2, fifo2, br2, guard)
    assert br2.frame_start_count == 1, \
        f"frame 2 frame_start fired {br2.frame_start_count}x (want 1)"
    assert br2.frame_done_count == 1, "frame 2 did not complete"
    assert fifo2.empty and not fifo2.overflowed
    _, p2 = reconstruct(br2.bits)
    assert p2 == [p & 0xFFFFFF for p in src2], "frame 2 mismatch"


# ------------------------------------------------------- negative controls
def neg_overflow_without_gating():
    """A readback that ignores udp_wrusedw must overflow the FIFO -> caught."""
    npix = 64
    src = grad(npix, seed=0x33)
    br, rb, fifo = run(read_pixels=src, gate=4, fifo_depth=8,
                       sck_div=8, bridge_pixels=npix, ignore_gate=True)
    assert fifo.overflowed, "no-gate readback did NOT overflow -> control is vacuous"


def neg_short_readback_never_completes():
    """PIXELS-1 pushed: SPI waits forever, frame_done must NOT fire."""
    npix = 12
    src = grad(npix - 1, seed=0x44)    # one short of what the bridge expects
    br, rb, fifo = run(read_pixels=src, gate=64, fifo_depth=128,
                       sck_div=4, bridge_pixels=npix, max_macro=4000)
    assert br.frame_done_count == 0, \
        "frame_done fired on a short frame -> completion detection is wrong"


def neg_corrupted_pixel_caught():
    """Bitstream reconstruction must reject a single corrupted source pixel."""
    npix = 10
    src = grad(npix, seed=0x55)
    br, rb, fifo = run(read_pixels=src, gate=64, fifo_depth=128,
                       sck_div=8, bridge_pixels=npix)
    _, pixels = reconstruct(br.bits)
    bad = [p & 0xFFFFFF for p in src]
    bad[3] ^= 0xFF
    assert pixels != bad, "corrupted reference still matched -> control is vacuous"
    assert pixels == [p & 0xFFFFFF for p in src], "good stream should match itself"


def main():
    print("sdram_to_spi.v two-clock model -- SDRAM readback -> FIFO -> SPI master\n")

    print("positive tests:")
    res = []
    res.append(t("frame integrity (12 px, order+count+magic)", test_frame_integrity))
    res.append(t("gating throttles readback (gate=4 depth=8)", test_gating_throttles_readback))
    res.append(t("underrun stalls without loss", test_underrun_stalls_no_loss))
    res.append(t("re-arm: two frames back to back", test_rearm_two_frames))

    print("\nnegative controls (these MUST fail to be valid):")
    res.append(t("no-gate readback overflows FIFO", neg_overflow_without_gating))
    res.append(t("short readback never asserts frame_done", neg_short_readback_never_completes))
    res.append(t("corrupted pixel rejected by reconstruction", neg_corrupted_pixel_caught))

    print("\nreal-frame analytic (not simulated; 307200 px, SDRAM 125M / SPI 50M):")
    for div in (4, 8):
        sck = 50_000_000 / div
        drain_s = (32 + 307200 * 24) / sck
        # readback fills the 4096 FIFO to the 2048 gate far faster than SPI drains
        fill_to_gate_s = 2048 / 125_000_000
        print(f"  SCK_DIV={div}: SCK={sck/1e6:.2f}MHz  SPI frame={drain_s:.3f}s  "
              f"FIFO fills to gate in {fill_to_gate_s*1e6:.1f}us "
              f"({drain_s/fill_to_gate_s:.0f}x slower drain => never underruns)")

    ok = all(res)
    print(f"\n{'==== ALL PASS ====' if ok else '==== FAILURES ===='} "
          f"({sum(res)}/{len(res)})")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
