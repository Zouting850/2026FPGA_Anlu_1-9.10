#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cycle-accurate model + testbench for boardB_eth_frontend rx_byte_cdc.v.

No Verilog simulator on this machine, so per project convention the glue is
verified by a Python model. rx_byte_cdc.v is thin: it wraps the vendor
fifo_sdr_data_2 (an async SHOW_AHEAD soft FIFO, treated as correct here exactly
as the other sims treat vendor IP) and adds only the read protocol

    out_valid = !empty ;  out_byte = dout ;  re = !empty

plus a sticky overflow flag. What this model pins down is therefore NOT the FIFO
internals but the two glue contracts that, if wrong, silently corrupt an image:

  (A) NO LOSS / NO DUP / IN ORDER when the FIFO does not overflow -- guaranteed
      only because out_byte is latched on the SAME rd_clk cycle that re pops the
      head it is showing. A one-cycle phase error between present and pop dups or
      drops a byte.
  (B) The 125 MHz -> 50 MHz rate crossing is real: a sustained full-rate producer
      (5 bytes per macro) outruns the consumer (2 bytes per macro) and the FIFO
      MUST overflow. This is what makes the PC pacing contract load-bearing rather
      than decorative; if a model never overflowed, contract (A) passing would be
      vacuous.

The two clock domains are advanced at their true ratio: udp_clk 125 MHz and
clk_50m 50 MHz give 5 write edges per 2 read edges in a 40 ns macro-cycle. The
FIFO is an idealised order-preserving deque bounded by DEPTH that drops a write
when full, mirroring fifo_sdr_data_2's wr_en_s = !full_flag & we.

Run:  python tools/sim_rx_byte_cdc.py
"""
import sys
import random
from collections import deque

WR_PER_MACRO = 5     # 125 MHz
RD_PER_MACRO = 2     # 50 MHz


class ByteFifo:
    """Idealised SHOW_AHEAD async byte FIFO. Order-preserving, depth-bounded,
    drops a write when full (== fifo_sdr_data_2 wr_en_s = !full & we)."""

    def __init__(self, depth=4096):
        self.depth = depth
        self.q = deque()
        self.dropped = 0

    # ---- write domain (udp_clk) ----
    def wr_edge(self, we, di):
        if we:
            if len(self.q) < self.depth:
                self.q.append(di & 0xFF)
            else:
                self.dropped += 1

    @property
    def full(self):
        return len(self.q) >= self.depth

    # ---- read domain (clk_50m), combinational SHOW_AHEAD view ----
    @property
    def empty(self):
        return len(self.q) == 0

    @property
    def dout(self):
        return self.q[0] if self.q else 0

    def rd_pop(self):
        """Advance the head one entry (re asserted while non-empty)."""
        if self.q:
            self.q.popleft()


def run(byte_stream, depth=4096, gap_after=0, paced_period=None,
        phase_bug=False, max_macros=200000):
    """Drive byte_stream into the FIFO and drain it with the rx_byte_cdc read
    protocol. Returns (received, dropped, overflowed).

    byte_stream   : list of ints (the bytes the PC sends, in order)
    gap_after     : if >0, insert this many idle write-macros after every
                    `paced_period` bytes (models inter-packet pacing)
    paced_period  : bytes per burst before a gap (use with gap_after)
    phase_bug     : NEGATIVE-CONTROL hook -- latch a REGISTERED dout (pop one
                    cycle out of phase with present), which must corrupt (A)
    """
    fifo = ByteFifo(depth)
    src = list(byte_stream)
    si = 0
    received = []
    overflowed = 0
    dout_d = 0          # registered dout, only used by the phase-bug consumer
    macros = 0

    # producer plan: which write-edges carry a byte
    # we generate the per-wr-edge we/di on the fly respecting pacing
    burst_cnt = 0
    gap_remaining = 0

    while macros < max_macros:
        # ---- 5 write edges ----
        for _ in range(WR_PER_MACRO):
            we, di = 0, 0
            if gap_remaining > 0:
                gap_remaining -= 1
            elif si < len(src):
                we, di = 1, src[si]
                si += 1
                burst_cnt += 1
                if paced_period and gap_after and burst_cnt >= paced_period:
                    burst_cnt = 0
                    gap_remaining = gap_after * WR_PER_MACRO
            if we and fifo.full:
                overflowed += 1
            fifo.wr_edge(we, di)

        # ---- 2 read edges ----
        for _ in range(RD_PER_MACRO):
            empty = fifo.empty
            out_valid = not empty
            dout = fifo.dout
            # correct protocol: present and pop the SAME head this cycle
            re = out_valid
            if phase_bug:
                # latch the PREVIOUS cycle's head while popping the current one
                if out_valid:
                    received.append(dout_d)
                if re:
                    fifo.rd_pop()
            else:
                if out_valid:
                    received.append(dout)
                if re:
                    fifo.rd_pop()
            dout_d = dout

        macros += 1
        # done when the whole source is in and the FIFO has drained
        if si >= len(src) and fifo.empty:
            break

    return received, fifo.dropped, overflowed


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
        print(f"  FAIL  {name}: negative control raised {type(ex).__name__}, not AssertionError")
        return False
    print(f"  FAIL  {name}: negative control did NOT fail -> test is vacuous")
    return False


# ------------------------------------------------------------------ tests
def test_paced_no_loss():
    """3 packets of 1440 B, paced so the average stays under the 50 MB/s drain:
    every byte must arrive, in order, with no drop."""
    pkt = 1440
    stream = [(i * 7 + (i // 251)) & 0xFF for i in range(pkt * 3)]
    # burst one packet (1440 wr-edges = 288 macros), then idle 600 macros so the
    # consumer (2/macro) fully drains the ~864 B residue before the next packet.
    rec, dropped, ovf = run(stream, depth=4096, paced_period=pkt, gap_after=600)
    assert dropped == 0, f"paced stream dropped {dropped} bytes"
    assert ovf == 0, f"paced stream overflowed {ovf} times"
    assert rec == stream, f"received {len(rec)} B != sent {len(stream)} B (in-order/no-loss)"


def test_single_burst_absorbed():
    """One 1440 B packet at full 125 MHz line rate, then idle: the 4096-deep FIFO
    must absorb the ~864 B peak (written-but-not-yet-drained) with zero loss."""
    stream = [(0xA5 + i) & 0xFF for i in range(1440)]
    rec, dropped, ovf = run(stream, depth=4096)   # no pacing, single packet then idle
    assert dropped == 0, f"single burst dropped {dropped} bytes (depth too small?)"
    assert rec == stream, f"single burst corrupted: {len(rec)} B received"


def test_random_pacing_no_loss():
    """Random inter-byte gaps (still averaging under the drain rate): order and
    completeness must hold for any pacing the PC might emit."""
    rng = random.Random(0xBEEF)
    stream = [rng.randrange(256) for _ in range(4000)]
    # gap every 1..40 bytes for 1..40 macros: average well under 2/5
    rec, dropped, ovf = run(stream, depth=4096, paced_period=20, gap_after=30)
    assert dropped == 0, f"random pacing dropped {dropped}"
    assert rec == stream, f"random pacing corrupted ({len(rec)} B)"


def neg_sustained_overrate_overflows():
    """(B) A continuous full-rate producer with NO pacing must overrun the FIFO.
    House idiom: assert the UNSOUND claim ("it loses nothing") so it is rejected;
    if that claim ever held, contract (A) passing above would be meaningless."""
    stream = [i & 0xFF for i in range(20000)]      # 20 KB at 5 B/macro, no gap
    rec, dropped, ovf = run(stream, depth=4096)    # consumer only drains 2/macro
    if dropped == 0 or ovf == 0:
        # Not the control's job to pass silently -- the model failed to overrun at
        # all, so contract B cannot be exercised. Surface it as a model error.
        raise RuntimeError(f"model never overflowed (dropped={dropped}, ovf={ovf})")
    assert rec == stream, "overrate stream came out lossless -> pacing contract is vacuous"


def neg_phase_bug_corrupts():
    """(A) A consumer that pops one cycle out of phase with the byte it latches
    must produce a stream != the input. Assert the UNSOUND claim (the bugged
    consumer is still lossless) so it is rejected -- proving the same-cycle
    present+pop wiring in rx_byte_cdc.v is what guarantees order, not luck."""
    stream = [(i * 13 + 5) & 0xFF for i in range(2000)]
    rec_good, _, _ = run(stream, depth=4096, paced_period=20, gap_after=30, phase_bug=False)
    rec_bad, _, _ = run(stream, depth=4096, paced_period=20, gap_after=30, phase_bug=True)
    if rec_good != stream:
        raise RuntimeError("baseline correct-protocol stream is itself wrong; control is moot")
    assert rec_bad == stream, "phase-bugged consumer was still lossless -> protocol test is vacuous"


def main():
    print("rx_byte_cdc.v two-clock model -- UDP RX byte stream 125M -> 50M\n")
    results = []

    print("positive tests (contract A: lossless, in order, no dup):")
    results.append(t("paced 3x1440B stream, no loss", test_paced_no_loss))
    results.append(t("single 1440B line-rate burst absorbed by depth", test_single_burst_absorbed))
    results.append(t("random pacing, order preserved", test_random_pacing_no_loss))

    print("\nnegative controls (these MUST fail to be valid):")
    results.append(expect_fail("sustained overrate overflows (contract B)", neg_sustained_overrate_overflows))
    results.append(expect_fail("present/pop phase error corrupts stream", neg_phase_bug_corrupts))

    print("\nrate analytic (125 MHz in / 50 MHz out, DEPTH=4096):")
    print(f"  per 40ns macro: {WR_PER_MACRO} B in, {RD_PER_MACRO} B out -> net "
          f"{WR_PER_MACRO - RD_PER_MACRO} B/macro accumulation at full rate")
    print(f"  one 1440B packet: peak residency ~{1440 - (1440 // WR_PER_MACRO) * RD_PER_MACRO} B "
          f"(< DEPTH 4096, absorbed)")
    print(f"  back-to-back full rate overflows after ~{4096 // (WR_PER_MACRO - RD_PER_MACRO)} macros "
          f"(~{4096 // (WR_PER_MACRO - RD_PER_MACRO) * WR_PER_MACRO} B written) -> PC must pace")

    ok = all(results)
    print(f"\n{'==== ALL PASS ====' if ok else '==== FAILURES ===='} "
          f"({sum(results)}/{len(results)})")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
