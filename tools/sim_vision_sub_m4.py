#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Frame-level + closed-loop models for the vision sub-board M4 design.

M4 adds a hardware auto-exposure loop on top of M3: the on-sensor AEC/AGC is
switched off (R0xAF = 0) and the FPGA closes the loop itself by rewriting
R0x0B (Coarse Shutter Width Total) and, only as a fallback, R0x35 (Analog
Gain Control).  There is no Verilog simulator on this machine, so this file
mirrors the M4 RTL and asserts the things that decide whether the loop
actually converges on the board:

  1. exp_meter.v accumulates sum / cnt / sat per frame and latches them at
     frame start (checked against a plain per-pixel reference).

  2. The control law is a *proportional* step  dshut = shut*|err| >> PSHIFT,
     clamped to [STEP_MIN, STEP_MAX] and never across the shut/gain rails.
     With a linear sensor this contracts the error quadratically
     (e' = -e^2/T), so it cannot hunt around the deadband the way a fixed
     step does.  A negative control shows exactly that: a fixed step
     oscillates across the deadband forever.

  3. The loop only acts on frame_tick (frame start = vertical blanking),
     respects the shutter's n+2 frame latency (datasheet: a shutter change
     takes effect for the next exposure but shows up in the n+2 image), and
     never writes while cfg_done is low.  The very first transaction after
     cfg_done is the R0xAF = 0 write.

  4. Closed-loop simulation with a linear sensor (with the n+2 latency and
     saturation clipping) converges into the deadband from dark and bright
     starts, then stops writing for 100 frames -- no oscillation -- and
     raises exp_locked.  At the rails (extreme scenes) it parks instead of
     hunting.  A negative control that ignores the spacing guard writes far
     more often.

  5. top_vision_m4.v emits a 164-byte status line with the new E/G/Y/L
     fields, byte-for-byte.

The M4 defines and the fixed-character table are PARSED OUT OF THE RTL, not
restated here, so the model cannot silently drift from the design.

Run from anywhere:  python tools/sim_vision_sub_m4.py
"""
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)
HDL = os.path.join(ROOT, "src", "vision_sub", "user_source", "hdl_source")
DEFS = os.path.join(HDL, "vision_def.v")
F_TOP = os.path.join(HDL, "top_vision_m4.v")
F_AEXP = os.path.join(HDL, "auto_exp.v")

FAILURES = []
CHECKS = [0]


def check(cond, label, detail=""):
    CHECKS[0] += 1
    if cond:
        print("    ok    %s" % label)
        return True
    FAILURES.append("%s%s" % (label, (" -- " + detail) if detail else ""))
    print("    FAIL  %s%s" % (label, (" -- " + detail) if detail else ""))
    return False


def expect_fail(cond, label, detail=""):
    """Negative control: passes when cond is False."""
    CHECKS[0] += 1
    if not cond:
        print("    ok    control bites: %s" % label)
        return True
    FAILURES.append("control did NOT bite: %s" % label)
    print("    FAIL  control did NOT bite: %s%s"
          % (label, (" -- " + detail) if detail else ""))
    return False


# ============================================================
# RTL introspection
# ============================================================
def strip_comments(text):
    return re.sub(r"//[^\n]*", "", text)


def resolve(expr, env):
    expr = expr.strip().strip(";").strip()
    m = re.match(r"^\d+\s*'\s*([hHbBdDoO])\s*(.+)$", expr)
    if m:
        base = {"h": 16, "b": 2, "d": 10, "o": 8}[m.group(1).lower()]
        return int(m.group(2).replace("_", ""), base)
    if expr.startswith("`"):
        return env[expr[1:]]
    if re.match(r"^-?\d+$", expr):
        return int(expr)
    subbed = re.sub(r"`([A-Za-z_]\w*)",
                    lambda mo: str(env.get(mo.group(1), "0")), expr)
    subbed = re.sub(r"([A-Za-z_]\w*)",
                    lambda mo: str(env.get(mo.group(1), "0")), subbed)
    return int(eval(subbed, {"__builtins__": {}}, {}))  # noqa: S307


def parse_defines(path=DEFS):
    env = {}
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = re.sub(r"//[^\n]*", "", line)
            m = re.match(r"\s*`define\s+(\w+)\s+(.+?)\s*$", line)
            if not m:
                continue
            try:
                env[m.group(1)] = resolve(m.group(2), env)
            except Exception:
                pass
    return env


def parse_fixed_char(path=F_TOP):
    with open(path, "r", encoding="utf-8") as fh:
        code = strip_comments(fh.read())
    m = re.search(r"function\[7:0\]\s+fixed_char;(.*?)endfunction", code, re.S)
    if not m:
        raise RuntimeError("fixed_char() not found in %s" % path)
    tbl = {}
    for _wid, idx, hexv in re.findall(
            r"(\d+)'d(\d+)\s*:\s+fixed_char\s*=\s*8'h([0-9A-Fa-f]+)", m.group(1)):
        tbl[int(idx)] = int(hexv, 16)
    tbl["default"] = None
    return tbl


def parse_mean_recip(path=F_AEXP):
    """Parse the display-mean reciprocal out of auto_exp.v.

    The RTL computes the 8-bit display mean as ``(frm_sum * M) >> SH`` rather
    than ``frm_sum / CAM_FRAME_PIX``.  That is not cosmetic: TD does *not*
    turn a division by a constant into a multiply-by-reciprocal, it builds a
    32-step combinational restoring divider -- measured at logic level 48 /
    50.564 ns, which single-handedly broke the whole 50 MHz sys_clk domain
    (slack -30.680 ns).  Parsing M and SH here means the model cannot silently
    drift from whatever the RTL actually does.
    """
    code = strip_comments(open(path, "r", encoding="utf-8").read())
    m = re.search(r"frm_sum\s*\*\s*\d+'d(\d+)", code)
    s = re.search(r"mean_m\s*\[\s*(\d+)\s*:\s*(\d+)\s*\]", code)
    if not m or not s:
        raise RuntimeError("display-mean reciprocal not found in %s" % path)
    return int(m.group(1)), int(s.group(2))


D = parse_defines()
TBL = parse_fixed_char()
MEAN_MUL, MEAN_SHIFT = parse_mean_recip()

PIX = D["CAM_FRAME_PIX"]
T_SUM = D["EXP_TARGET_SUM"]
BAND = D["EXP_DEADBAND_SUM"]
PSHIFT = D["EXP_PSHIFT"]
STEP_MIN = D["EXP_STEP_MIN"]
STEP_MAX = D["EXP_STEP_MAX"]
SHUT_MIN = D["EXP_SHUT_MIN"]
SHUT_MAX = D["EXP_SHUT_MAX"]
SHUT_INIT = D["EXP_SHUT_INIT"]
GAIN_MIN = D["EXP_GAIN_MIN"]
GAIN_MAX = D["EXP_GAIN_MAX"]
GAIN_INIT = D["EXP_GAIN_INIT"]
GAIN_STEP = D["EXP_GAIN_STEP"]
UPDATE_DIV = D["EXP_UPDATE_DIV"]
LOCK_FRAMES = D["EXP_LOCK_FRAMES"]
SAT_LEVEL = D["EXP_SAT_LEVEL"]
SAT_MAX = D["EXP_SAT_MAX"]
REG_SHUT = D["MT_REG_SHUTTER"]
REG_GAIN = D["MT_REG_GAIN"]
REG_AEC = D["MT_REG_AECAGC"]
VAL_AEC_OFF = D["MT_VAL_AECAGC_OFF"]
LINE_LEN = D["M4_LINE_LEN"]
CURVE_N = D["CURVE_N"]


# ============================================================
# exp_meter.v -- frame-level model
# ============================================================
class ExpMeter(object):
    """Mirrors exp_meter.v: accumulate during the frame, latch at frame start."""

    def __init__(self):
        self.acc_sum = 0
        self.acc_cnt = 0
        self.acc_sat = 0
        self.sum_o = 0
        self.cnt_o = 0
        self.sat_o = 0
        self.tog = 0

    def pixel(self, v):
        self.acc_sum += v
        self.acc_cnt += 1
        if v >= SAT_LEVEL:
            self.acc_sat += 1

    def frame_start(self):
        self.sum_o = self.acc_sum
        self.cnt_o = self.acc_cnt
        self.sat_o = self.acc_sat
        self.tog ^= 1
        self.acc_sum = 0
        self.acc_cnt = 0
        self.acc_sat = 0


# ============================================================
# auto_exp.v -- control law (shared by the frame model and the
# closed-loop simulation, so the two cannot drift)
# ============================================================
def control_step(shut, gain, frm_sum, frm_sat, fixed_step=None):
    """Mirror of auto_exp.v's combinational decision.

    Returns None (no write) or ('shut'|'gain', new_value).
    too_bright is checked before too_dark, exactly like the RTL if/else-if
    chain -- a saturated-but-dark-average scene counts as too bright.
    """
    geom_ok = True  # caller guarantees cnt; see AutoExpFrame.frame()
    sat_alarm = frm_sat > SAT_MAX
    too_bright = geom_ok and ((frm_sum > T_SUM + BAND) or sat_alarm)
    too_dark = geom_ok and (frm_sum < T_SUM - BAND)

    if not (too_bright or too_dark):
        return None

    # proportional step: (shut * |err|) >> PSHIFT, clamped
    errmag = abs(frm_sum - T_SUM)          # < 2^25 by construction
    if fixed_step is not None:
        step = fixed_step
    else:
        raw = (shut * errmag) >> PSHIFT
        if raw > STEP_MAX:
            step = STEP_MAX
        elif raw < STEP_MIN:
            step = STEP_MIN
        else:
            step = raw

    if too_bright:
        if shut > SHUT_MIN:
            dec = step if step < (shut - SHUT_MIN) else (shut - SHUT_MIN)
            return ("shut", shut - dec)
        if gain > GAIN_MIN:
            return ("gain", gain - GAIN_STEP)
        return None
    # too dark
    if shut < SHUT_MAX:
        inc = step if step < (SHUT_MAX - shut) else (SHUT_MAX - shut)
        return ("shut", shut + inc)
    if gain < GAIN_MAX:
        return ("gain", gain + GAIN_STEP)
    return None


class AutoExpFrame(object):
    """Frame-granularity model of auto_exp.v's state machine.

    One call to frame() models one frame period.  The SCCB transaction
    itself (~0.5 ms at 100 kHz SCL) completes well inside a 16.7 ms frame,
    so E_WRSET/E_WRBSY/E_WRACK are modelled as instantaneous commits --
    what matters for the loop is only *which frame* a write lands in.
    """

    def __init__(self, fixed_step=None, no_div_guard=False):
        self.st = "INIT"
        self.shut = SHUT_INIT
        self.gain = GAIN_INIT
        self.div = 0
        self.band = 0
        self.locked = 0
        self.mean = 0
        self.wr_cnt = 0
        self.writes = []                   # (frame_no, addr, value)
        self.fixed_step = fixed_step       # negative-control injection
        self.no_div_guard = no_div_guard   # negative-control injection

    def frame(self, n, frm_sum, frm_cnt, frm_sat, cfg_done):
        """One frame period.  Returns the list of (addr, value) written."""
        ws = []
        if self.st == "INIT":
            if cfg_done:
                # first transaction after config: switch off on-sensor AEC/AGC
                ws.append((REG_AEC, VAL_AEC_OFF))
                self.writes.append((n, REG_AEC, VAL_AEC_OFF))
                self.div = 0
                self.st = "RUN"
            return ws

        # ---- E_RUN: everything below happens on frame_tick only ----
        # div_cnt is evaluated with its OLD value (non-blocking semantics)
        div_ok = self.no_div_guard or (self.div >= UPDATE_DIV)
        if self.div != UPDATE_DIV:
            self.div += 1

        geom_ok = (frm_cnt == PIX)
        sat_alarm = frm_sat > SAT_MAX
        in_band = (geom_ok and not (frm_sum > T_SUM + BAND or sat_alarm)
                   and not (frm_sum < T_SUM - BAND))

        # mean display: (sum * M) >> SH ≈ sum / 90240, only meaningful with
        # valid geometry (see parse_mean_recip for why this is not a divide)
        self.mean = (((frm_sum * MEAN_MUL) >> MEAN_SHIFT) & 0xFF) if geom_ok else 0

        # lock judgment
        if in_band:
            if self.band >= LOCK_FRAMES:
                self.locked = 1
            else:
                self.band += 1
        else:
            self.band = 0
            self.locked = 0

        # adjustment decision
        if geom_ok and div_ok:
            d = control_step(self.shut, self.gain, frm_sum, frm_sat,
                             fixed_step=self.fixed_step)
            if d is not None:
                kind, val = d
                addr = REG_SHUT if kind == "shut" else REG_GAIN
                ws.append((addr, val))
                self.writes.append((n, addr, val))
                if kind == "shut":
                    self.shut = val
                else:
                    self.gain = val
                self.wr_cnt += 1
                self.div = 0
        return ws


# ============================================================
# Linear sensor with the shutter's n+2 latency and clipping
# ============================================================
class Sensor(object):
    """sum = N_pix * clamp(k * shut * gain); optional bright-spot fraction.

    Latency model (datasheet wording): a shutter write issued at frame head n
    takes effect for the *next* exposure (frame n+1) and therefore shows up
    in the n+2 image.  head() applies the previous frame's pending write
    before this frame's decision, set_pend() registers this frame's write.
    """

    def __init__(self, k, hi_frac=0.0, k_lo=None):
        self.k = k
        self.hi_frac = hi_frac
        self.k_lo = k if k_lo is None else k_lo
        self.eff = SHUT_INIT
        self.pend = None

    def head(self):
        """Frame start: last frame's write (if any) now drives the exposure."""
        if self.pend is not None:
            self.eff = self.pend
            self.pend = None

    def set_pend(self, v):
        """A shutter write issued this frame -> effective next exposure."""
        self.pend = v

    def measure(self, gain):
        """Return (sum, cnt, sat) for the image produced by this exposure."""
        cnt = PIX
        if self.hi_frac <= 0.0:
            v = int(round(self.k * self.eff * gain))
            v = max(0, min(255, v))
            sat = cnt if v >= SAT_LEVEL else 0
            return (v * cnt, cnt, sat)
        # high-contrast scene: a fraction of the frame is a saturated spot
        n_hi = int(cnt * self.hi_frac)
        n_lo = cnt - n_hi
        v_lo = int(round(self.k_lo * self.eff * gain))
        v_lo = max(0, min(255, v_lo))
        return (n_hi * 255 + n_lo * v_lo, cnt, n_hi)


def run_loop(sensor, frames, cfg_frame=3, fixed_step=None, no_div_guard=False):
    """Drive AutoExpFrame + Sensor together; return the model for inspection."""
    ae = AutoExpFrame(fixed_step=fixed_step, no_div_guard=no_div_guard)
    m_sum, m_cnt, m_sat = 0, 0, 0
    sums = []
    for n in range(frames):
        sensor.head()
        ws = ae.frame(n, m_sum, m_cnt, m_sat, n >= cfg_frame)
        for addr, val in ws:
            if addr == REG_SHUT:
                sensor.set_pend(val)
        m_sum, m_cnt, m_sat = sensor.measure(ae.gain)
        sums.append(m_sum)
    return ae, sums


def in_band(sum_, sat=0):
    return (not (sum_ > T_SUM + BAND or sat > SAT_MAX)
            and not (sum_ < T_SUM - BAND))


# ============================================================
# 1. constants
# ============================================================
def test_constants():
    print("-- 1. M4 constants are mutually consistent --")
    check(T_SUM == D["EXP_TARGET_MEAN"] * PIX,
          "EXP_TARGET_SUM == EXP_TARGET_MEAN x CAM_FRAME_PIX")
    check(T_SUM < (1 << 25), "target sum fits the 25-bit errmag path")
    check(PIX * 255 < (1 << 25),
          "worst-case frame sum 255x90240 fits 25 bits (no errmag overflow)")
    check(STEP_MIN <= STEP_MAX, "STEP_MIN <= STEP_MAX")
    check(SHUT_MIN < SHUT_INIT <= SHUT_MAX, "SHUT_INIT inside the shutter rails")
    check(GAIN_MIN <= GAIN_INIT <= GAIN_MAX, "GAIN_INIT inside the gain rails")
    check(SAT_MAX < PIX, "saturation limit is below one full frame")
    check(LOCK_FRAMES <= 15, "LOCK_FRAMES fits the 4-bit band_cnt")
    check(2 <= UPDATE_DIV <= 15, "UPDATE_DIV is a sane frame spacing")
    check(LINE_LEN == 68 + CURVE_N + 2,
          "line length 164 == 68 header bytes + 94 curve chars + CRLF")
    check(REG_SHUT == 0x0B and REG_GAIN == 0x35 and REG_AEC == 0xAF,
          "register addresses are R0x0B / R0x35 / R0xAF")


# ============================================================
# 1b. display-mean reciprocal (real TD timing bug, fixed)
# ============================================================
def test_mean_reciprocal():
    print("-- 1b. display mean uses a reciprocal multiply, not a divider --")
    M, SH = MEAN_MUL, MEAN_SHIFT
    max_sum = PIX * 255                      # hard upper bound of frm_sum

    exact = 1.0 / PIX
    approx = M / float(1 << SH)

    # This is a *proof* over the whole input range, not a spot check:
    #   |approx*sum - exact*sum| <= |approx - exact| * max_sum  for all sum.
    bound = abs(approx - exact) * max_sum
    check(bound < 1.0,
          "M/2^SH stays within 1 LSB of sum/90240 across the entire range",
          "M=%d SH=%d -> bound %.4f LSB" % (M, SH, bound))

    check(max_sum * M < (1 << 32),
          "the 32-bit intermediate product cannot overflow "
          "(max_sum x %d = %d < 2^32)" % (M, max_sum * M))

    check(((max_sum * M) >> SH) <= 255,
          "display mean never wraps the 8-bit field at full brightness "
          "(max -> %d)" % ((max_sum * M) >> SH))

    check(M * (1 << SH) > 0 and SH > 0 and M > 1,
          "the reciprocal is an actual multiply-shift (M>1, SH>0)")

    # Negative control: the nearest power of two is 2^16, and 1/65536 is 37.7%
    # away from 1/90240 -- i.e. a bare right shift CANNOT do this job, which is
    # exactly why the multiply is required.
    best_pow2 = 1 << (PIX.bit_length() - 1)
    pow2_err = abs(1.0 / best_pow2 - exact) * max_sum
    expect_fail(pow2_err < 1.0,
                "control bites: no pure right shift can replace the division",
                "best 2^%d is off by %.0f LSB at full brightness"
                % (PIX.bit_length() - 1, pow2_err))


# ============================================================
# 2. exp_meter
# ============================================================
def test_meter():
    print("-- 2. exp_meter accumulates and latches per frame --")
    m = ExpMeter()
    frame = [100] * 300 + [255] * 50 + [0] * 26
    for v in frame:
        m.pixel(v)
    m.frame_start()
    check(m.sum_o == sum(frame), "frame sum matches the plain reference")
    check(m.cnt_o == len(frame), "pixel count matches")
    check(m.sat_o == 50, "saturated pixels counted (255 >= %d)" % SAT_LEVEL)
    check(m.acc_sum == 0 and m.acc_cnt == 0 and m.acc_sat == 0,
          "accumulators cleared at frame start")
    check(m.tog == 1, "meter_done_tog flipped once")

    for v in [250] * 10 + [249] * 5:
        m.pixel(v)
    m.frame_start()
    check(m.sat_o == 10 and m.sum_o == 10 * 250 + 5 * 249,
          "threshold is >= (250 counts, 249 does not)")
    check(m.tog == 0, "tog flipped back (one toggle per frame)")

    # cross-check against the Sensor's analytic uniform frame
    s = Sensor(0.02)
    s_sum, s_cnt, s_sat = s.measure(16)
    m2 = ExpMeter()
    v = int(round(0.02 * SHUT_INIT * 16))
    v = max(0, min(255, v))
    for _ in range(PIX):
        m2.pixel(v)
    m2.frame_start()
    check(m2.sum_o == s_sum and m2.cnt_o == s_cnt and m2.sat_o == s_sat,
          "ExpMeter agrees with the analytic uniform frame")


# ============================================================
# 3. control law invariants (exhaustive-ish)
# ============================================================
def test_control_law():
    print("-- 3. control-law invariants --")
    probes = [0,
              T_SUM - 3 * BAND, T_SUM - BAND - 1, T_SUM - BAND,
              T_SUM - BAND // 2, T_SUM, T_SUM + BAND // 2,
              T_SUM + BAND, T_SUM + BAND + 1, T_SUM + 3 * BAND,
              PIX * 255]
    shunts = [SHUT_MIN, SHUT_MIN + 1, 137, SHUT_INIT, 500,
              SHUT_MAX - 1, SHUT_MAX]
    ok_none = True
    ok_rails = True
    ok_dir = True
    ok_sat = True
    for shut in shunts:
        for s in probes:
            for sat in (0, SAT_MAX):
                d = control_step(shut, GAIN_INIT, s, sat)
                inb = (T_SUM - BAND <= s <= T_SUM + BAND) and sat <= SAT_MAX
                if inb and d is not None:
                    ok_none = False
                if d is not None:
                    kind, val = d
                    if kind == "shut":
                        if not (SHUT_MIN <= val <= SHUT_MAX):
                            ok_rails = False
                    else:
                        if not (GAIN_MIN <= val <= GAIN_MAX):
                            ok_rails = False
                    bright = (s > T_SUM + BAND) or (sat > SAT_MAX)
                    if bright:
                        if kind == "shut" and val > shut:
                            ok_dir = False
                        if kind == "gain" and val > GAIN_INIT:
                            ok_dir = False
                    else:
                        if kind == "shut" and val < shut:
                            ok_dir = False
                        if kind == "gain" and val < GAIN_INIT:
                            ok_dir = False
    check(ok_none, "deadband (and sat<=limit) never produces a write")
    check(ok_rails, "every result stays inside the shut/gain rails")
    check(ok_dir, "bright reduces, dark increases -- never the wrong way")

    d = control_step(SHUT_INIT, GAIN_INIT, T_SUM, SAT_MAX + 1)
    ok_sat = d is not None and d[0] in ("shut", "gain") and (
        d[0] == "gain" or d[1] < SHUT_INIT)
    check(ok_sat, "saturation alarm alone forces a reduction")
    expect_fail(control_step(SHUT_INIT, GAIN_INIT, T_SUM, SAT_MAX + 1) is None,
                "the sat alarm is load-bearing "
                "(above-limit sat produces a write)")

    # proportional step is clamped to [STEP_MIN, STEP_MAX]
    big = control_step(SHUT_INIT, GAIN_INIT, PIX * 255, 0)
    check(big is not None and abs(big[1] - SHUT_INIT) <= STEP_MAX,
          "huge error still moves at most STEP_MAX per write")
    tiny = control_step(600, GAIN_INIT, T_SUM + BAND + 1, 0)
    check(tiny is not None and abs(tiny[1] - 600) >= STEP_MIN,
          "tiny error still moves at least STEP_MIN (no stall outside band)")


# ============================================================
# 4. handshake / sequencing
# ============================================================
def test_handshake():
    print("-- 4. sequencing: AEC-off first, frame-aligned, spaced --")
    dark = (3 * 1000 * 1000, PIX, 0)
    ae = AutoExpFrame()

    pre = []
    for n in range(5):
        pre += ae.frame(n, dark[0], dark[1], dark[2], cfg_done=0)
    check(pre == [], "no transaction at all before cfg_done")

    ws = ae.frame(5, dark[0], dark[1], dark[2], cfg_done=1)
    check(ws == [(REG_AEC, VAL_AEC_OFF)],
          "first transaction after cfg_done is R0xAF = 0")

    for n in range(6, 60):
        ae.frame(n, dark[0], dark[1], dark[2], cfg_done=1)
    tail = ae.writes[1:]
    check(all(a in (REG_SHUT, REG_GAIN) for _f, a, _v in tail),
          "later writes only touch R0x0B / R0x35")
    frames_of = [f for f, _a, _v in ae.writes]
    gaps_ok = all(b - a >= 3 for a, b in zip(frames_of, frames_of[1:]))
    check(gaps_ok, "consecutive writes are >= 3 frames apart (n+2 latency guard)")
    per_frame_ok = all(frames_of.count(f) == 1 for f in set(frames_of))
    check(per_frame_ok, "at most one transaction per frame")

    ws_bad = ae.frame(999, 100000, 12345, 0, cfg_done=1)
    check(ws_bad == [], "broken geometry (cnt != 90240) produces no write")

    # locked flag in a static in-band scene
    ae2 = AutoExpFrame()
    ae2.frame(0, T_SUM, PIX, 0, cfg_done=1)
    for n in range(1, 30):
        ae2.frame(n, T_SUM, PIX, 0, cfg_done=1)
    check(ae2.locked == 1, "static in-band scene raises exp_locked")
    check(ae2.writes == [(0, REG_AEC, VAL_AEC_OFF)],
          "in-band scene writes nothing beyond the AEC-off")


# ============================================================
# 5. closed loop with the linear sensor
# ============================================================
def test_closed_loop():
    print("-- 5. closed loop converges and does not oscillate --")

    # (a) ordinary dark scene
    ae, sums = run_loop(Sensor(0.01), 200)
    conv = next((i for i in range(len(sums))
                 if all(in_band(s) for s in sums[i:])), None)
    check(conv is not None and conv < 60,
          "dark start enters the deadband within 60 frames", "conv=%s" % conv)
    check(not [w for w in ae.writes if w[0] > 100],
          "dark start: zero writes in the last 100 frames (no hunting)")
    check(ae.locked == 1, "dark start: exp_locked raised")
    check(ae.shut > SHUT_INIT, "dark start: shutter climbed (it was dark)")

    # (b) ordinary bright scene
    ae, sums = run_loop(Sensor(0.04), 200)
    conv = next((i for i in range(len(sums))
                 if all(in_band(s) for s in sums[i:])), None)
    check(conv is not None and conv < 60,
          "bright start enters the deadband within 60 frames", "conv=%s" % conv)
    check(not [w for w in ae.writes if w[0] > 100],
          "bright start: zero writes in the last 100 frames")
    check(ae.locked == 1, "bright start: exp_locked raised")
    check(ae.shut < SHUT_INIT, "bright start: shutter dropped (it was bright)")

    # (c) extreme bright: parks at both lower rails instead of hunting
    ae, sums = run_loop(Sensor(2.0), 200)
    check(ae.shut == SHUT_MIN and ae.gain == GAIN_MIN,
          "extreme bright parks at SHUT_MIN and GAIN_MIN")
    check(not [w for w in ae.writes if w[0] > 150],
          "extreme bright: stops writing once parked")
    check(ae.locked == 0, "extreme bright: not locked (honestly unachievable)")

    # (d) extreme dark: parks at both upper rails
    ae, sums = run_loop(Sensor(0.001), 500)
    check(ae.shut == SHUT_MAX and ae.gain == GAIN_MAX,
          "extreme dark parks at SHUT_MAX and GAIN_MAX")
    check(not [w for w in ae.writes if w[0] > 450],
          "extreme dark: stops writing once parked")

    # (e) high-contrast scene: saturation alarm forces reduction even though
    #     the average is inside the deadband
    ae, sums = run_loop(Sensor(0.0, hi_frac=0.06, k_lo=0.021), 200)
    shut_writes = [v for _f, a, v in ae.writes if a == REG_SHUT]
    check(all(b < a for a, b in zip(shut_writes, shut_writes[1:])),
          "sat alarm: shutter writes are strictly decreasing")
    check(ae.shut < SHUT_INIT,
          "sat alarm: shutter reduced below the initial value")

    # ---- negative controls ----
    ae, sums = run_loop(Sensor(0.04), 200, fixed_step=32)
    late = [w for w in ae.writes if w[0] > 100]
    expect_fail(not late,
                "a fixed step of 32 keeps writing after 'convergence' "
                "(oscillates across the deadband)")
    ae, sums = run_loop(Sensor(0.04), 200, fixed_step=32)
    conv = next((i for i in range(len(sums))
                 if all(in_band(s) for s in sums[i:])), None)
    expect_fail(conv is not None and conv < 60,
                "fixed-step loop stays out of band (hunting)")

    ae, sums = run_loop(Sensor(0.04), 200, no_div_guard=True)
    late = [w for w in ae.writes if w[0] > 100]
    expect_fail(not late,
                "writing every frame against the n+2 latency keeps "
                "re-adjusting on stale data")
    check(len(late) > 30 if late else True,
          "stale-data loop is writing nearly every frame (documented)",
          "late writes = %d" % len(late))


# ============================================================
# 6. status line
# ============================================================
HEX = "0123456789ABCDEF"


def build_line(ok, cnt, fg, m, ppl, ocs, exp, gain, mean, lock, curve):
    """Python mirror of top_vision_m4.v's line_byte() + tx shift."""
    out = []
    for i in range(LINE_LEN):
        if i in TBL:
            out.append(chr(TBL[i]))
        elif i == 4:
            out.append("K" if ok else "F")
        elif 8 <= i <= 13:
            out.append(HEX[(cnt >> (4 * (5 - (i - 8)))) & 0xF])
        elif 17 <= i <= 22:
            out.append(HEX[(fg >> (4 * (5 - (i - 17)))) & 0xF])
        elif 26 <= i <= 31:
            out.append(HEX[(m >> (4 * (5 - (i - 26)))) & 0xF])
        elif i == 35:
            out.append(HEX[ppl & 0xF])
        elif 39 <= i <= 41:
            out.append(HEX[(ocs >> (4 * (2 - (i - 39)))) & 0xF])
        elif 45 <= i <= 48:
            out.append(HEX[(exp >> (4 * (3 - (i - 45)))) & 0xF])
        elif 52 <= i <= 54:
            out.append(HEX[(gain >> (4 * (2 - (i - 52)))) & 0xF])
        elif 58 <= i <= 60:
            out.append(HEX[(mean >> (4 * (2 - (i - 58)))) & 0xF])
        elif i == 64:
            out.append(HEX[lock & 0xF])
        elif 68 <= i <= 161:
            out.append(HEX[curve[i - 68] & 0xF])
        else:
            out.append(" ")
    return "".join(out)


def test_line():
    print("-- 6. 164-byte status line --")
    curve = [i % 16 for i in range(CURVE_N)]
    curve_str = "".join(HEX[c] for c in curve)
    expected = ("MH4 K N=016080 F=0000A3 M=00001C P=2 S=068 "
                "E=0100 G=010 Y=060 L=1 V=" + curve_str + "\r\n")
    line = build_line(True, 90240, 163, 28, 2, 104, 256, 16, 96, 1, curve)
    check(len(line) == LINE_LEN == 164, "line is exactly 164 bytes")
    check(line == expected, "line matches the expected string byte-for-byte",
          repr(line[:90]))
    check(line[162] == "\r" and line[163] == "\n", "CRLF at the tail")
    check(line[66:68] == "V=" and len(line[68:162]) == CURVE_N,
          "curve field is the last 94 chars before CRLF")

    fixed = {k for k in TBL if k != "default"}
    dyn = ({4} | set(range(8, 14)) | set(range(17, 23)) | set(range(26, 32))
           | {35} | set(range(39, 42)) | set(range(45, 49)) | set(range(52, 55))
           | set(range(58, 61)) | {64} | set(range(68, 162)))
    check(not (fixed & dyn), "no position is both fixed and dynamic")
    check(fixed | dyn == set(range(LINE_LEN)), "every byte position is covered")

    line_f = build_line(False, 90240, 163, 28, 2, 104, 256, 16, 96, 1, curve)
    check(line_f[4] == "F", "self-test flag renders 'F' when cam_ok = 0")

    swapped_curve = list(curve)
    swapped_curve[3], swapped_curve[4] = swapped_curve[4], swapped_curve[3]
    swapped = build_line(True, 90240, 163, 28, 2, 104, 256, 16, 96, 1,
                         swapped_curve)
    expect_fail(swapped == expected, "a nibble swap would change the line")
    mirrored = build_line(True, 90240, 163, 28, 2, 104, 256, 16, 96, 1,
                          curve[::-1])
    expect_fail(mirrored == expected,
                "mirroring the curve packing changes the line")


# ============================================================
# main
# ============================================================
def main():
    print("=" * 72)
    print("vision sub-board M4 model -- auto exposure closed loop")
    print("=" * 72)
    test_constants()
    test_mean_reciprocal()
    test_meter()
    test_control_law()
    test_handshake()
    test_closed_loop()
    test_line()
    print("=" * 72)
    if FAILURES:
        print("FAILED %d of %d CHECKS" % (len(FAILURES), CHECKS[0]))
        for f in FAILURES:
            print("  - %s" % f)
        sys.exit(1)
    print("ALL %d CHECKS PASSED" % CHECKS[0])


if __name__ == "__main__":
    main()
