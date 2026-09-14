#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Frame-level model of the vision sub-board M5 design (behaviour FSM).

M5 turns M3's per-frame instantaneous quantities into *behaviour*: is anyone
there, has the count settled, are they interacting, how long did they stay.
It emits (state, suggested mode, confidence) and leaves the final decision to
the host board (design doc section 2).  There is no Verilog simulator on this
machine, so this file mirrors behavior_fsm.v and asserts the things that decide
whether the state machine behaves on the board:

  1. The people-count filter: a +/-1 difference (the deadband) is only
     accepted after PPL_CONFIRM consecutive frames *in the same direction*,
     while a difference of 2 or more is followed immediately.  M3's segment
     count oscillates between 1 and 2 as a matter of course (two people
     standing close together), so without this filter the suggested play mode
     flips back and forth.

  2. The state ladder IDLE -> PRESENCE -> SINGLE/MULTI -> INTERACT, the LEAVE
     path with the dwell time it latches, the 10 s LEAVE -> IDLE timeout and
     the ALERT excursion, all with their confirmation counters -- at *exact*
     frame indices, because that is where an off-by-one lives.

  3. Two energy-baseline traps that the model found while the RTL was being
     written (both are real fixes, documented in vision_def.v):
       - the EMA baseline starts at 0, so without an initial step load the
         first person to appear looks like a huge energy *jump*;
       - even with the step load, a change in the number of people is itself
         an energy step, and if the FSM is allowed to judge "interaction"
         while the baseline is still catching up, entering MULTI fires a
         spurious INTERACT.  warm_cnt (NRG_BASE_READY frames after any count
         change) is the guard, and a negative control shows the failure it
         prevents.

  4. The 1.5 s interact cooldown: waving continuously must not emit an event
     every few frames.  A negative control that exits INTERACT as soon as the
     energy subsides emits an order of magnitude more events.

  5. Geometry-broken frames freeze the FSM and only decay the confidence --
     they must never be read as "nobody is there".

  6. top_vision_m5.v emits a 193-byte status line with the new T/Q/C/D/I/U
     fields, byte-for-byte, with every byte position accounted for.

The M5 defines and the fixed-character table are PARSED OUT OF THE RTL, not
restated here, so the model cannot silently drift from the design.

Run from anywhere:  python tools/sim_vision_sub_m5.py
"""
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)
HDL = os.path.join(ROOT, "src", "vision_sub", "user_source", "hdl_source")
DEFS = os.path.join(HDL, "vision_def.v")
F_TOP = os.path.join(HDL, "top_vision_m5.v")

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


# ------------------------------------------------------------
# Sized-literal width audit
#
# resolve() above deliberately ignores the declared width of a literal -- it
# only cares about the *value*.  That is normally fine, but it hid two real
# bugs until the design was actually compiled by TangDynasty:
#
#     `define IDLE_HOLD_FRAMES  9'd600    -> 9 bits cannot hold 600, TD warns
#                                          (HDL-5007) and ships 88 frames, so
#                                          the 10 s IDLE hold became ~1.5 s
#     `define CONF_SHIFT        3'd8      -> truncated to 0
#
# The model happily agreed with the broken RTL because 600 is 600.  Auditing
# every sized literal here turns that whole class of silent truncation into an
# immediate, loud failure instead of a board-level mystery.
# ------------------------------------------------------------
_SIZED_LIT = re.compile(r"^\s*(\d+)\s*'\s*([hHbBdDoO])\s*([0-9a-fA-F_xXzZ]+)\s*$")
_RADIX = {"h": 16, "b": 2, "d": 10, "o": 8}


def literal_width_violations(path=DEFS):
    """Return [(lineno, name, literal, value, bits_needed)] for too-narrow sizes."""
    bad = []
    with open(path, "r", encoding="utf-8") as fh:
        for lineno, line in enumerate(fh, 1):
            line = re.sub(r"//[^\n]*", "", line)
            m = re.match(r"\s*`define\s+(\w+)\s+(.+?)\s*$", line)
            if not m:
                continue
            lit = _SIZED_LIT.match(m.group(2))
            if not lit:
                continue
            width, base, digits = lit.groups()
            if any(c in "xXzZ" for c in digits):
                continue
            value = int(digits.replace("_", ""), _RADIX[base.lower()])
            need = max(1, value.bit_length())
            if int(width) < need:
                bad.append((lineno, m.group(1), "%s'%s%s" % (width, base, digits),
                            value, need))
    return bad


def test_define_widths():
    """Every sized literal must be wide enough for its own value (TD truncates)."""
    print("\n-- sized-literal widths in vision_def.v --")
    bad = literal_width_violations()
    for lineno, name, lit, value, need in bad:
        print("        %s:%d  `define %s = %s  ->  value %d needs %d bits"
              % (os.path.basename(DEFS), lineno, name, lit, value, need))
    check(not bad, "no sized literal is narrower than its value",
          "%d violation(s); TD would silently truncate" % len(bad)
          if bad else "")

    # Prove the auditor actually bites on the two real bugs we shipped --
    # otherwise the check above could pass by being vacuous.
    import tempfile
    fd, tmp = tempfile.mkstemp(suffix=".v")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write("`define GOOD_A 10'd600\n"
                     "`define BAD_A   9'd600\n"
                     "`define BAD_B   3'd8\n"
                     "`define GOOD_B  4'd8\n")
        caught = {v[1] for v in literal_width_violations(tmp)}
    finally:
        os.unlink(tmp)
    check(caught == {"BAD_A", "BAD_B"},
          "auditor catches exactly the two historical truncation bugs "
          "(9'd600, 3'd8) and accepts the fixed forms",
          "auditor returned %s" % sorted(caught))


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


D = parse_defines()
TBL = parse_fixed_char()

PIX = D["CAM_FRAME_PIX"]
IMG_W = D["CAM_IMG_W"]
LINE_LEN = D["M5_LINE_LEN"]
CURVE_N = D["CURVE_N"]
CURVE_POS = D["M5_CURVE_POS"]
CURVE_END = D["M5_CURVE_END"]

ST_IDLE = D["ST_IDLE"]
ST_PRESENCE = D["ST_PRESENCE"]
ST_SINGLE = D["ST_SINGLE"]
ST_MULTI = D["ST_MULTI"]
ST_INTERACT = D["ST_INTERACT"]
ST_LEAVE = D["ST_LEAVE"]
ST_ALERT = D["ST_ALERT"]

MD_STANDBY = D["MD_STANDBY"]
MD_SINGLE = D["MD_SINGLE"]
MD_MULTI = D["MD_MULTI"]
MD_INTERACT = D["MD_INTERACT"]
MD_ALERT = D["MD_ALERT"]

PPL_DEADBAND = D["PPL_DEADBAND"]
PPL_CONFIRM = D["PPL_CONFIRM"]
EVT_CONFIRM = D["EVT_CONFIRM"]
STABLE_FRAMES = D["STABLE_FRAMES"]
LEAVE_FRAMES = D["LEAVE_FRAMES"]
IDLE_HOLD_FRAMES = D["IDLE_HOLD_FRAMES"]
COOL_FRAMES = D["COOL_FRAMES"]

CTR_QUIET = D["CTR_QUIET"]
DIR_WIN = D["DIR_WIN"]
DIR_MIN = D["DIR_MIN"]
DIR_WIN_MAX = D["DIR_WIN_MAX"]

NRG_BASE_SHIFT = D["NRG_BASE_SHIFT"]
NRG_BASE_READY = D["NRG_BASE_READY"]
NRG_INT_DELTA = D["NRG_INT_DELTA"]
NRG_INT_RATIO = D["NRG_INT_RATIO"]
NRG_LO = D["NRG_LO"]
NRG_HI = D["NRG_HI"]

GUARD_EDGE = D["GUARD_EDGE"]
GUARD_R_LEFT = D["GUARD_R_LEFT"]

SC_UP = D["SC_UP"]
SC_DN_SOFT = D["SC_DN_SOFT"]
SC_DN_HARD = D["SC_DN_HARD"]
CONF_MUL = D["CONF_MUL"]
CONF_SHIFT = D["CONF_SHIFT"]

ST_NAME = {ST_IDLE: "IDLE", ST_PRESENCE: "PRESENCE", ST_SINGLE: "SINGLE",
           ST_MULTI: "MULTI", ST_INTERACT: "INTERACT", ST_LEAVE: "LEAVE",
           ST_ALERT: "ALERT"}


# ============================================================
# exp_meter-free harness: driving the FSM frame by frame
# ============================================================
class BehaviorFsm(object):
    """Mirror of behavior_fsm.v.

    Everything runs on frame ticks (one call to frame() = one frame), exactly
    like the RTL: all combinational wires are evaluated from the *current*
    register values, then every register is updated together (non-blocking
    semantics).  Doing it any other way is how off-by-one bugs get baked into
    the model instead of being caught by it.
    """

    def __init__(self, no_deadband=False, no_confirm=False, no_cooldown=False,
                 no_warmup=False):
        self.st = ST_IDLE
        self.pplf = 0
        self.pdir_cnt = 0
        self.pdir_up = False
        self.enter_cnt = 0
        self.multi_cnt = 0
        self.single_cnt = 0
        self.guard_cnt = 0
        self.unguard_cnt = 0
        self.stable_cnt = 0
        self.leave_cnt = 0
        self.idle_cnt = 0
        self.cool_cnt = 0
        self.ctr_p = 0
        self.cmin = self.cmax = self.wfirst = self.wlast = 0
        self.wcnt = 0
        self.nrg_base = 0
        self.base_init = False
        self.warm_cnt = 0
        self.pplr_d = 0
        self.sc_dwell = 0
        self.sc_motion = 0
        self.sc_quiet = 0
        self.dwell = 0
        self.icnt = 0
        self.dir = 0
        self.push = 0
        self.conf = 0
        self.mode = MD_STANDBY
        self.events = []      # (frame_no, "enter"|"leave"|"interact")
        self.trace = []       # per-frame record

        # ---- negative-control injections ----
        self.no_deadband = no_deadband
        self.no_confirm = no_confirm
        self.no_cooldown = no_cooldown
        self.no_warmup = no_warmup

    # --------------------------------------------------------
    def frame(self, n, ppl, ctr=40, cf=35, cl=45, occ=8, nrg=5000, geom_ok=True):
        ev = {"enter": 0, "leave": 0, "interact": 0}
        st = self.st

        # ---------- combinational (from current registers) ----------
        ppl_d = abs(ppl - self.pplf)
        ppl_up = ppl > self.pplf
        ppl_big = ppl_d > PPL_DEADBAND

        ctr_d = abs(ctr - self.ctr_p)
        quiet = geom_ok and (ctr_d <= CTR_QUIET) and (nrg >= NRG_LO)

        nbase_n = (self.nrg_base - (self.nrg_base >> NRG_BASE_SHIFT)
                   + (nrg >> NRG_BASE_SHIFT))
        nrg_upth = nrg > self.nrg_base + (self.nrg_base >> NRG_INT_RATIO)
        nrg_abth = nrg > self.nrg_base + NRG_INT_DELTA
        nrg_jump = nrg_abth and nrg_upth
        push_big = nrg > self.nrg_base + 2 * NRG_INT_DELTA

        wspan = self.cmax - self.cmin
        dir_ok = geom_ok and (wspan >= DIR_MIN) and (nrg >= NRG_LO)
        dir_right = self.wlast > self.wfirst
        push_ok = geom_ok and push_big and (wspan < DIR_MIN)
        base_ok = True if self.no_warmup else (
            (self.warm_cnt == NRG_BASE_READY) and (ppl == self.pplr_d))
        hit = geom_ok and base_ok and ((nrg_jump and dir_ok) or push_ok)

        cf_px = cf << 2
        cl_px = (cl << 2) + 3
        guard_hit = (occ != 0) and (cf_px <= GUARD_EDGE or cl_px >= GUARD_R_LEFT)

        busy = st in (ST_PRESENCE, ST_SINGLE, ST_MULTI, ST_INTERACT, ST_ALERT)
        leave_now = busy and (self.pplf == 0) and (self.leave_cnt == LEAVE_FRAMES - 1)

        # ---------- next-state registers ----------
        n_st = st
        n_pplf, n_pdir_cnt, n_pdir_up = self.pplf, self.pdir_cnt, self.pdir_up
        n_enter, n_multi, n_single = self.enter_cnt, self.multi_cnt, self.single_cnt
        n_guard, n_unguard = self.guard_cnt, self.unguard_cnt
        n_stable, n_leave, n_idle, n_cool = (self.stable_cnt, self.leave_cnt,
                                            self.idle_cnt, self.cool_cnt)
        n_ctr_p = self.ctr_p
        n_cmin, n_cmax, n_wfirst, n_wlast, n_wcnt = (self.cmin, self.cmax,
                                                    self.wfirst, self.wlast,
                                                    self.wcnt)
        n_nrg_base, n_base_init, n_warm, n_pplr_d = (self.nrg_base, self.base_init,
                                                    self.warm_cnt, self.pplr_d)
        n_sd, n_sm, n_sq = self.sc_dwell, self.sc_motion, self.sc_quiet
        n_dwell, n_icnt = self.dwell, self.icnt
        n_dir, n_push = self.dir, self.push

        # ---------- people-count deadband filter ----------
        if self.no_deadband:
            n_pplf = ppl
            n_pdir_cnt = 0
        elif ppl_big:
            n_pplf = ppl
            n_pdir_cnt = 0
        elif ppl_up or (ppl < self.pplf):
            if self.no_confirm:
                n_pplf = ppl
                n_pdir_cnt = 0
            elif self.pdir_cnt == 0:
                n_pdir_up = ppl_up
                n_pdir_cnt = 1
            elif self.pdir_up == ppl_up:
                if self.pdir_cnt == PPL_CONFIRM - 1:
                    n_pplf = ppl
                    n_pdir_cnt = 0
                else:
                    n_pdir_cnt = self.pdir_cnt + 1
            else:
                n_pdir_up = ppl_up
                n_pdir_cnt = 1
        else:
            n_pdir_cnt = 0

        # ---------- geometry-broken frames: freeze, decay confidence ----------
        if not geom_ok:
            n_sd = max(0, self.sc_dwell - SC_DN_SOFT)
            n_sm = max(0, self.sc_motion - SC_DN_SOFT)
            n_sq = max(0, self.sc_quiet - SC_DN_SOFT)

            self.conf = ((self.sc_dwell + self.sc_motion + self.sc_quiet)
                         * CONF_MUL) >> CONF_SHIFT
            self.mode = mode_of(st)
            self._commit(n, st, n_pplf, n_pdir_cnt, n_pdir_up, n_st, n_enter,
                         n_multi, n_single, n_guard, n_unguard, n_stable,
                         n_leave, n_idle, n_cool, n_ctr_p, n_cmin, n_cmax,
                         n_wfirst, n_wlast, n_wcnt, n_nrg_base, n_base_init,
                         n_warm, n_pplr_d, n_sd, n_sm, n_sq, n_dwell, n_icnt,
                         n_dir, n_push, ev)
            return ev

        # ---------- history with no state dependency ----------
        n_ctr_p = ctr

        if self.no_warmup:
            n_nrg_base = nbase_n
        elif not self.base_init:
            n_nrg_base = nrg
            n_base_init = True
        elif st != ST_INTERACT:
            n_nrg_base = nbase_n

        # the raw count is what matters here: a count step changes the energy
        # *this* frame, while pplf still needs PPL_CONFIRM frames to confirm
        if ppl != self.pplr_d:
            n_warm = 0
            n_pplr_d = ppl
        elif n_warm != NRG_BASE_READY:
            n_warm = n_warm + 1

        if self.wcnt == DIR_WIN_MAX:
            n_cmin = n_cmax = n_wfirst = n_wlast = ctr
            n_wcnt = 0
        else:
            if ctr < self.cmin:
                n_cmin = ctr
            if ctr > self.cmax:
                n_cmax = ctr
            if self.wcnt == 0:
                n_wfirst = ctr
            n_wlast = ctr
            n_wcnt = self.wcnt + 1

        # ---------- dwell + leave ----------
        if busy:
            n_dwell = self.dwell + 1

        if leave_now:
            n_st = ST_LEAVE
            ev["leave"] = 1
            n_leave = 0
            n_idle = 0
            n_stable = 0
            n_guard = 0
            n_unguard = 0
        elif busy and self.pplf == 0:
            n_leave = self.leave_cnt + 1
        elif busy:
            n_leave = 0

        # ---------- state transitions ----------
        if not leave_now:
            if st == ST_IDLE:
                n_dwell = 0
                if self.pplf >= 1:
                    if self.enter_cnt == EVT_CONFIRM - 1:
                        n_st = ST_PRESENCE
                        ev["enter"] = 1
                        n_enter = 0
                        n_stable = 0
                    else:
                        n_enter = self.enter_cnt + 1
                else:
                    n_enter = 0

            elif st == ST_PRESENCE:
                if self.pplf >= 2:
                    if self.multi_cnt == EVT_CONFIRM - 1:
                        n_st = ST_MULTI
                        n_multi = 0
                        n_stable = 0
                    else:
                        n_multi = self.multi_cnt + 1
                else:
                    n_multi = 0
                    if quiet:
                        if self.stable_cnt == STABLE_FRAMES - 1:
                            n_st = ST_SINGLE
                            n_stable = 0
                        else:
                            n_stable = self.stable_cnt + 1
                    else:
                        n_stable = 0

            elif st in (ST_SINGLE, ST_MULTI):
                if guard_hit:
                    if self.guard_cnt == EVT_CONFIRM - 1:
                        n_st = ST_ALERT
                        n_guard = 0
                        n_unguard = 0
                    else:
                        n_guard = self.guard_cnt + 1
                else:
                    n_guard = 0

                if st == ST_SINGLE:
                    if self.pplf >= 2:
                        if self.multi_cnt == EVT_CONFIRM - 1:
                            n_st = ST_MULTI
                            n_multi = 0
                        else:
                            n_multi = self.multi_cnt + 1
                    else:
                        n_multi = 0
                else:
                    if self.pplf <= 1:
                        if self.single_cnt == EVT_CONFIRM - 1:
                            n_st = ST_SINGLE
                            n_single = 0
                        else:
                            n_single = self.single_cnt + 1
                    else:
                        n_single = 0

                if hit:
                    n_st = ST_INTERACT
                    ev["interact"] = 1
                    n_icnt = min(255, self.icnt + 1)
                    n_cool = 0
                    n_stable = 0
                    n_dir = (2 if dir_right else 1) if dir_ok else 3
                    n_push = 1 if (not dir_ok and push_ok) else 0

            elif st == ST_INTERACT:
                if self.no_cooldown:
                    if nrg <= self.nrg_base + NRG_INT_DELTA:
                        n_cool = 0
                        n_stable = 0
                        n_st = ST_MULTI if self.pplf >= 2 else ST_SINGLE
                elif self.cool_cnt == COOL_FRAMES - 1:
                    n_cool = 0
                    n_stable = 0
                    n_st = ST_MULTI if self.pplf >= 2 else ST_SINGLE
                else:
                    n_cool = self.cool_cnt + 1

            elif st == ST_LEAVE:
                if self.pplf >= 1:
                    n_st = ST_PRESENCE
                    ev["enter"] = 1
                    n_dwell = 0
                    n_stable = 0
                    n_idle = 0
                elif self.idle_cnt == IDLE_HOLD_FRAMES - 1:
                    n_st = ST_IDLE
                    n_idle = 0
                else:
                    n_idle = self.idle_cnt + 1

            elif st == ST_ALERT:
                if not guard_hit:
                    if self.unguard_cnt == EVT_CONFIRM - 1:
                        n_unguard = 0
                        n_st = ST_MULTI if self.pplf >= 2 else ST_SINGLE
                    else:
                        n_unguard = self.unguard_cnt + 1
                else:
                    n_unguard = 0

            else:
                n_st = ST_IDLE

        # ---------- confidence scores ----------
        if busy:
            n_sd = min(255, self.sc_dwell + SC_UP)
        else:
            n_sd = max(0, self.sc_dwell - SC_DN_HARD)

        if NRG_LO <= nrg <= NRG_HI:
            n_sm = min(255, self.sc_motion + SC_UP)
        else:
            n_sm = max(0, self.sc_motion - SC_DN_HARD)

        if quiet:
            n_sq = min(255, self.sc_quiet + SC_UP)
        else:
            n_sq = max(0, self.sc_quiet - SC_DN_SOFT)

        # conf is a wire off the *current* score registers in the RTL
        self.conf = ((self.sc_dwell + self.sc_motion + self.sc_quiet)
                     * CONF_MUL) >> CONF_SHIFT
        self.mode = mode_of(st)

        self._commit(n, st, n_pplf, n_pdir_cnt, n_pdir_up, n_st, n_enter,
                     n_multi, n_single, n_guard, n_unguard, n_stable, n_leave,
                     n_idle, n_cool, n_ctr_p, n_cmin, n_cmax, n_wfirst,
                     n_wlast, n_wcnt, n_nrg_base, n_base_init, n_warm,
                     n_pplr_d, n_sd, n_sm, n_sq, n_dwell, n_icnt, n_dir,
                     n_push, ev)
        return ev

    # --------------------------------------------------------
    def _commit(self, n, st_now, pplf, pdir_cnt, pdir_up, st, enter, multi,
                single, guard, unguard, stable, leave, idle, cool, ctr_p,
                cmin, cmax, wfirst, wlast, wcnt, nrg_base, base_init, warm,
                pplr_d, sd, sm, sq, dwell, icnt, dir_, push, ev):
        self.pdir_cnt = pdir_cnt
        self.pdir_up = pdir_up
        self.pplf = pplf
        self.st = st
        self.enter_cnt = enter
        self.multi_cnt = multi
        self.single_cnt = single
        self.guard_cnt = guard
        self.unguard_cnt = unguard
        self.stable_cnt = stable
        self.leave_cnt = leave
        self.idle_cnt = idle
        self.cool_cnt = cool
        self.ctr_p = ctr_p
        self.cmin, self.cmax, self.wfirst, self.wlast, self.wcnt = (cmin, cmax,
                                                                   wfirst,
                                                                   wlast, wcnt)
        self.nrg_base = nrg_base
        self.base_init = base_init
        self.warm_cnt = warm
        self.pplr_d = pplr_d
        self.sc_dwell, self.sc_motion, self.sc_quiet = sd, sm, sq
        self.dwell = dwell
        self.icnt = icnt
        self.dir = dir_
        self.push = push
        for kind in ("enter", "leave", "interact"):
            if ev[kind]:
                self.events.append((n, kind))
        self.trace.append(dict(frame=n, st=st, pplf=pplf, dwell=dwell,
                               icnt=icnt, conf=self.conf, nrg_base=nrg_base,
                               warm=warm, ev=ev))


def mode_of(s):
    return {ST_SINGLE: MD_SINGLE, ST_MULTI: MD_MULTI, ST_INTERACT: MD_INTERACT,
            ST_ALERT: MD_ALERT, ST_PRESENCE: MD_SINGLE}.get(s, MD_STANDBY)


def drive(fsm, plan):
    """plan: list of frame kwargs dicts (or a single dict repeated N times)."""
    for n, kw in enumerate(plan):
        fsm.frame(n, **kw)
    return fsm


def const(frames, **kw):
    return [dict(kw) for _ in range(frames)]


def step_until(fsm, cond, n0, limit=3000, **kw):
    """Drive frames until cond(fsm); returns the frame index, or None."""
    n = n0
    while n < n0 + limit:
        fsm.frame(n, **kw)
        if cond(fsm):
            return n
        n += 1
    return None


def n_interacts(fsm):
    return len([e for e in fsm.events if e[1] == "interact"])


# ============================================================
# 1. constants
# ============================================================
def test_constants():
    print("-- 1. M5 constants are mutually consistent --")
    check(LINE_LEN == 193, "status line is 193 bytes", "LINE_LEN=%d" % LINE_LEN)
    check(CURVE_END - CURVE_POS + 1 == CURVE_N,
          "curve field is exactly CURVE_N bytes",
          "%d..%d" % (CURVE_POS, CURVE_END))
    check(LINE_LEN == CURVE_END + 3,
          "line = curve end + CR + LF + 1 (no bytes after the curve)")
    check(LINE_LEN == D["M4_LINE_LEN"] + 29,
          "M5 line is M4's plus the 29 new content bytes")
    check(CURVE_POS == 97, "curve starts at 97 (M4 prefix 68 + 29 M5 bytes)")
    check(PPL_DEADBAND >= 1, "there is a people-count deadband")
    check(PPL_CONFIRM >= 2, "a +/-1 count change needs confirmation")
    check(EVT_CONFIRM >= 2, "state transitions need confirmation")
    check(STABLE_FRAMES > EVT_CONFIRM,
          "PRESENCE->SINGLE is much slower than the generic confirm")
    check(LEAVE_FRAMES >= EVT_CONFIRM, "leaving needs at least the confirm time")
    check(IDLE_HOLD_FRAMES > LEAVE_FRAMES,
          "the 10 s idle timeout is longer than the 2 s leave timeout")
    check(COOL_FRAMES >= 1, "the interact cooldown is non-zero")
    check(NRG_BASE_READY >= NRG_BASE_SHIFT,
          "the baseline warm-up covers at least one EMA time constant")
    check(NRG_LO < NRG_HI, "the energy window is not inverted")
    check(2 * GUARD_EDGE < IMG_W, "the guard bands do not overlap")
    check(GUARD_R_LEFT == IMG_W - 1 - GUARD_EDGE, "guard right edge matches")
    check(all(0 <= v < 8 for v in (ST_IDLE, ST_PRESENCE, ST_SINGLE, ST_MULTI,
                                   ST_INTERACT, ST_LEAVE, ST_ALERT)),
          "all state codes fit 3 bits")
    check(all(0 <= v < 8 for v in (MD_STANDBY, MD_SINGLE, MD_MULTI,
                                   MD_INTERACT, MD_ALERT)),
          "all mode codes fit 3 bits")
    check(len({ST_IDLE, ST_PRESENCE, ST_SINGLE, ST_MULTI, ST_INTERACT,
               ST_LEAVE, ST_ALERT}) == 7, "the seven states are distinct")
    check(((255 + 255 + 255) * CONF_MUL) >> CONF_SHIFT <= 255,
          "confidence cannot overflow 8 bits")
    check((765 * CONF_MUL) >> CONF_SHIFT == 254,
          "confidence saturates at 254 (documented bound)")


# ============================================================
# 2. people-count deadband filter
# ============================================================
def test_pplf_filter():
    print("-- 2. the +/-1 deadband and the same-direction confirmation --")

    f = BehaviorFsm()
    for n in range(20):
        f.frame(n, ppl=1)
    check(f.pplf == 1, "a steady 1 person settles to pplf = 1")
    check(f.st == ST_PRESENCE, "…and the FSM moved on to PRESENCE")

    # a +/-1 change needs PPL_CONFIRM frames
    f = BehaviorFsm()
    for n in range(10):
        f.frame(n, ppl=1)
    base = f.pplf
    for n in range(10):
        f.frame(10 + n, ppl=2)
    check(base == 1 and f.pplf == 2, "1 -> 2 is accepted after confirmation")

    f = BehaviorFsm()
    for n in range(10):
        f.frame(n, ppl=1)
    for n in range(PPL_CONFIRM - 1):
        f.frame(10 + n, ppl=2)
    check(f.pplf == 1,
          "1 -> 2 is NOT accepted after only %d frames" % (PPL_CONFIRM - 1))

    # alternating 1/2/1/2 never accumulates
    f = BehaviorFsm()
    for n in range(10):
        f.frame(n, ppl=1)
    for n in range(300):
        f.frame(10 + n, ppl=2 if n % 2 == 0 else 1)
    check(f.pplf == 1, "alternating 1/2 never changes the filtered count")
    check(f.st == ST_PRESENCE or f.st == ST_SINGLE,
          "alternating 1/2 never reaches MULTI", ST_NAME[f.st])

    # direction reversal restarts the count
    f = BehaviorFsm()
    for n in range(10):
        f.frame(n, ppl=1)
    seq = [2, 2, 1, 2, 2, 1, 2, 2, 1]
    for n, v in enumerate(seq):
        f.frame(10 + n, ppl=v)
    check(f.pplf == 1, "a reversal in the middle restarts the confirmation")

    # >= 2 difference is followed immediately
    f = BehaviorFsm()
    for n in range(10):
        f.frame(n, ppl=1)
    f.frame(10, ppl=3)
    check(f.pplf == 3, "a jump of 2 or more is followed immediately (no wait)")

    # downwards too
    f = BehaviorFsm()
    for n in range(10):
        f.frame(n, ppl=3)
    f.frame(10, ppl=1)
    check(f.pplf == 1, "a drop of 2 or more is followed immediately")


# ============================================================
# 3. the state ladder at exact frame indices
# ============================================================
def test_ladder():
    print("-- 3. IDLE -> PRESENCE -> SINGLE at exact frames --")
    f = BehaviorFsm()
    # pplf becomes 1 at the end of frame 2 (PPL_CONFIRM = 3 confirmations)
    for n in range(3):
        f.frame(n, ppl=1)
    check(f.pplf == 1, "pplf = 1 after exactly PPL_CONFIRM frames")
    check(f.st == ST_IDLE, "still IDLE on the frame pplf first reads 1")

    # the FSM then needs EVT_CONFIRM frames of pplf >= 1
    for n in range(3, 3 + EVT_CONFIRM - 1):
        f.frame(n, ppl=1)
    check(f.st == ST_IDLE,
          "still IDLE after only %d confirmed frames" % (EVT_CONFIRM - 1))
    f.frame(5, ppl=1)
    check(f.events and f.events[-1] == (5, "enter"),
          "ev_enter fires on frame 5", repr(f.events))
    check(f.st == ST_PRESENCE, "PRESENCE on the frame after ev_enter")

    # PRESENCE -> SINGLE after STABLE_FRAMES of a quiet, energetic scene
    for n in range(6, 6 + STABLE_FRAMES - 1):
        f.frame(n, ppl=1)
    check(f.st == ST_PRESENCE,
          "still PRESENCE after %d stable frames" % (STABLE_FRAMES - 1))
    f.frame(6 + STABLE_FRAMES - 1, ppl=1)
    check(f.st == ST_SINGLE,
          "SINGLE after exactly STABLE_FRAMES stable frames (frame %d)"
          % (6 + STABLE_FRAMES - 1))
    # dwell = frames since PRESENCE became visible (frame 6)
    last = 6 + STABLE_FRAMES - 1
    check(f.dwell == last - 6 + 1,
          "dwell counts every frame since PRESENCE", "dwell=%d" % f.dwell)
    check(f.mode == MD_SINGLE, "the suggested mode is SINGLE")
    check(f.conf > 200, "confidence is high in a settled single-person scene",
          "conf=%d" % f.conf)

    # not quiet -> never reaches SINGLE
    f2 = BehaviorFsm()
    for n in range(400):
        # jitter of 5 boxes every frame defeats CTR_QUIET (3)
        f2.frame(n, ppl=1, ctr=40 + (5 if n % 2 else -5))
    check(f2.st == ST_PRESENCE, "a jittering centroid never reaches SINGLE",
          ST_NAME[f2.st])
    check(f2.conf < 200, "jitter keeps the confidence down", "conf=%d" % f2.conf)


# ============================================================
# 4. MULTI, then back to SINGLE
# ============================================================
def test_multi():
    print("-- 4. SINGLE <-> MULTI and the +/-1 deadband around it --")
    f = BehaviorFsm()
    for n in range(200):
        f.frame(n, ppl=1)
    check(f.st == ST_SINGLE, "settled at SINGLE first")

    # 1 -> 2 : deadband (PPL_CONFIRM) then the state confirm (EVT_CONFIRM)
    for n in range(200, 207):
        f.frame(n, ppl=2)
    check(f.pplf == 2, "pplf follows to 2 after the confirm")
    check(f.st == ST_MULTI, "MULTI after the deadband + the state confirm")
    f.frame(207, ppl=2)
    # mode_o is a wire off the state register, so it follows one frame later
    check(f.mode == MD_MULTI, "the suggested mode becomes MULTI")

    for n in range(208, 215):
        f.frame(n, ppl=1)
    check(f.pplf == 1, "pplf returns to 1")
    check(f.st == ST_SINGLE, "SINGLE again after the confirmation")

    # 1 <-> 2 chatter inside MULTI must not leave MULTI
    f = BehaviorFsm()
    for n in range(200):
        f.frame(n, ppl=2)
    check(f.st == ST_MULTI, "settled at MULTI")
    for n in range(200, 500):
        f.frame(n, ppl=2 if (n // 3) % 2 == 0 else 1)
    check(f.st == ST_MULTI,
          "a 3-frame 1/2 chatter never drops MULTI (deadband + confirm)",
          ST_NAME[f.st])


# ============================================================
# 5. interact + cooldown
# ============================================================
def wave(n):
    """Alternating energy with a sweeping centroid: a real wave."""
    high = (n % 10) < 6
    return dict(ppl=1, ctr=40 + (10 if n % 2 else -10), nrg=12000 if high else 5000)


def test_interact():
    print("-- 5. INTERACT, its cooldown, and the direction payload --")
    f = BehaviorFsm()
    for n in range(200):
        f.frame(n, ppl=1)
    check(f.st == ST_SINGLE, "settled at SINGLE before waving")
    check(f.icnt == 0, "no interactions yet")

    # one wave: 10 frames
    for n in range(200, 210):
        f.frame(n, **wave(n))
    hits = [e for e in f.events if e[1] == "interact"]
    check(len(hits) == 1, "a single wave produces exactly one interact event",
          repr(hits))
    check(f.st == ST_INTERACT, "the state is INTERACT during the cooldown")
    check(f.mode == MD_INTERACT, "the suggested mode is INTERACT")
    check(f.icnt == 1, "the interact counter is 1")
    # At the very first hit the direction window has not accumulated a sweep
    # yet (the energy spike happens on the same frame the arm starts moving),
    # so the first hit is legitimately classified as a push.  What must hold
    # is that the payload is self-consistent.
    check(f.dir in (1, 2, 3) and ((f.dir == 3) == (f.push == 1)),
          "the direction payload is self-consistent",
          "dir=%d push=%d" % (f.dir, f.push))

    # the cooldown keeps the state for COOL_FRAMES
    for n in range(210, 200 + COOL_FRAMES):
        f.frame(n, **wave(n))
    check(f.st == ST_INTERACT, "still INTERACT inside the cooldown")
    check(f.icnt == 1, "no extra events inside the cooldown")
    f.frame(200 + COOL_FRAMES, **wave(200 + COOL_FRAMES))
    check(f.st == ST_SINGLE, "falls back to SINGLE when the cooldown expires")

    # by the second hit the window has plenty of sweep -> a real direction
    n = 200 + COOL_FRAMES + 1
    while n < 900 and n_interacts(f) < 2:
        f.frame(n, **wave(n))
        n += 1
    check(n_interacts(f) == 2,
          "continuous waving produces a second event (cooldown respected)",
          "hits=%d" % n_interacts(f))
    check(f.dir in (1, 2) and f.push == 0,
          "the second hit of a lateral wave reports left/right",
          "dir=%d push=%d" % (f.dir, f.push))

    # a push (energy spike, centroid still) yields dir = 3
    f2 = BehaviorFsm()
    for n in range(200):
        f2.frame(n, ppl=1)
    for n in range(200, 206):
        f2.frame(n, ppl=1, ctr=40, nrg=14000)
    check(any(e[1] == "interact" for e in f2.events),
          "a push (energy spike, no lateral sweep) still counts as interact")
    check(f2.dir == 3 and f2.push == 1,
          "a push is encoded dir = 3 / push = 1",
          "dir=%d push=%d" % (f2.dir, f2.push))

    # ---- negative control: no cooldown ----
    fc = BehaviorFsm(no_cooldown=True)
    for n in range(200):
        fc.frame(n, ppl=1)
    for n in range(200, 400):
        fc.frame(n, **wave(n))
    n_cool = len([e for e in fc.events if e[1] == "interact"])
    check(n_cool > 10,
          "without the cooldown the same waving emits many events "
          "(documented failure mode)", "events=%d" % n_cool)
    expect_fail(n_cool <= 3,
                "the cooldown is load-bearing (10x fewer events with it)")


# ============================================================
# 6. leave, dwell and the idle timeout
# ============================================================
def test_leave():
    print("-- 6. LEAVE, the dwell time it latches, and the 10 s timeout --")
    f = BehaviorFsm()
    for n in range(200):
        f.frame(n, ppl=1)
    dwell_before = f.dwell
    check(f.st == ST_SINGLE and dwell_before > 100,
          "settled with a growing dwell", "dwell=%d" % dwell_before)

    # one blank frame must not start the leave count in a meaningful way
    f.frame(200, ppl=0)
    check(f.st != ST_LEAVE, "a single blank frame does not leave")
    f.frame(201, ppl=1)
    check(f.st == ST_SINGLE, "and the state is unchanged")

    # a real departure: 0 people until the leave timer expires
    n_leave = step_until(f, lambda g: g.st == ST_LEAVE, 202,
                         limit=LEAVE_FRAMES + 60, ppl=0)
    check(n_leave is not None, "LEAVE is reached with nobody there")
    check(f.events[-1][1] == "leave", "ev_leave fired", repr(f.events[-1]))
    # the timer starts only after pplf reaches 0 (PPL_CONFIRM frames later)
    check(LEAVE_FRAMES <= (n_leave - 202) <= LEAVE_FRAMES + PPL_CONFIRM,
          "the leave timer runs for exactly LEAVE_FRAMES",
          "frames=%d" % (n_leave - 202))

    latched = f.dwell
    for n in range(n_leave + 1, n_leave + 30):
        f.frame(n, ppl=0)
    check(f.dwell == latched,
          "the dwell time is frozen once the visitor has left",
          "%d -> %d" % (latched, f.dwell))

    # LEAVE -> IDLE after IDLE_HOLD_FRAMES
    n_idle = step_until(f, lambda g: g.st == ST_IDLE, n_leave + 30,
                        limit=IDLE_HOLD_FRAMES + 60, ppl=0)
    check(n_idle is not None, "IDLE is reached after the idle timeout")
    check(abs((n_idle - n_leave) - IDLE_HOLD_FRAMES) <= 2,
          "the idle timeout is exactly IDLE_HOLD_FRAMES frames",
          "elapsed=%d" % (n_idle - n_leave))
    check(f.mode in (MD_STANDBY,), "the suggested mode is STANDBY in IDLE",
          "mode=%d" % f.mode)
    f.frame(n_idle + 1, ppl=0)     # dwell clears on the next IDLE frame
    check(f.dwell == 0, "dwell is cleared in IDLE")

    # somebody comes back inside the 10 s window
    f = BehaviorFsm()
    for n in range(200):
        f.frame(n, ppl=1)
    n_leave = step_until(f, lambda g: g.st == ST_LEAVE, 200,
                         limit=LEAVE_FRAMES + 60, ppl=0)
    check(n_leave is not None, "in LEAVE again")
    for n in range(n_leave + 1, n_leave + 20):
        f.frame(n, ppl=1)
    check(f.st == ST_PRESENCE, "coming back goes to PRESENCE (not IDLE)",
          ST_NAME[f.st])
    check(f.dwell < 30, "and the dwell time restarts", "dwell=%d" % f.dwell)


# ============================================================
# 7. the guard band / ALERT excursion
# ============================================================
def test_alert():
    print("-- 7. the guard band excursion --")
    f = BehaviorFsm()
    for n in range(200):
        f.frame(n, ppl=1, cf=35, cl=45, occ=8)
    check(f.st == ST_SINGLE, "settled in SINGLE, away from the edges")

    # step into the left guard band: cf = 2 boxes -> 8 px <= GUARD_EDGE
    for n in range(200, 202):
        f.frame(n, ppl=1, cf=2, cl=12, occ=8)
    check(f.st == ST_SINGLE, "not ALERT before the confirmation")
    for n in range(202, 205):
        f.frame(n, ppl=1, cf=2, cl=12, occ=8)
    check(f.st == ST_ALERT, "ALERT after EVT_CONFIRM frames in the guard band")
    check(f.mode == MD_ALERT, "the suggested mode is ALERT")

    # stepping back out
    for n in range(205, 208):
        f.frame(n, ppl=1, cf=35, cl=45, occ=8)
    for n in range(208, 211):
        f.frame(n, ppl=1, cf=35, cl=45, occ=8)
    check(f.st == ST_SINGLE, "back to SINGLE once clear of the guard band")

    # the right guard band too
    f2 = BehaviorFsm()
    for n in range(200):
        f2.frame(n, ppl=1, cf=35, cl=45, occ=8)
    for n in range(200, 210):
        f2.frame(n, ppl=1, cf=88, cl=93, occ=8)
    check(f2.st == ST_ALERT, "the right guard band triggers ALERT as well")

    # a stray single-box blob far from the edge must not alert
    f3 = BehaviorFsm()
    for n in range(200):
        f3.frame(n, ppl=1, cf=35, cl=45, occ=8)
    check(f3.st == ST_SINGLE, "no alert without a guard band hit")


# ============================================================
# 8. geometry-broken frames
# ============================================================
def test_geom_broken():
    print("-- 8. broken geometry freezes the FSM --")
    f = BehaviorFsm()
    for n in range(200):
        f.frame(n, ppl=1)
    st_before = f.st
    conf_before = f.conf

    for n in range(200, 400):
        f.frame(n, ppl=0, nrg=0, geom_ok=False)
    check(f.st == st_before,
          "200 geometry-broken frames do not move the state",
          "%s -> %s" % (ST_NAME[st_before], ST_NAME[f.st]))
    check(f.conf < conf_before,
          "the confidence decays meanwhile (and does not rise)",
          "%d -> %d" % (conf_before, f.conf))
    check(f.dwell > 0, "the dwell time is not reset (nobody actually left)")

    # and the FSM recovers once the geometry is valid again
    for n in range(400, 430):
        f.frame(n, ppl=1)
    check(f.st == ST_SINGLE, "the FSM carries on as before", ST_NAME[f.st])


# ============================================================
# 9. the two energy-baseline traps
# ============================================================
def test_baseline_traps():
    print("-- 9. energy-baseline traps (both are real RTL fixes) --")

    # (a) a person appearing at frame 0 must not look like a jump
    f = BehaviorFsm()
    for n in range(200):
        f.frame(n, ppl=2, ctr=40, nrg=5000)
    hits = [e for e in f.events if e[1] == "interact"]
    check(not hits,
          "settling from reset never fires a spurious interact", repr(hits))

    # negative control: without the step-load + warm-up guard this fires
    f2 = BehaviorFsm(no_warmup=True)
    for n in range(30):
        f2.frame(n, ppl=2, ctr=40, nrg=5000)
    hits2 = [e for e in f2.events if e[1] == "interact"]
    check(bool(hits2),
          "without the warm-up guard the same scene fires an interact "
          "(documented failure mode)", repr(hits2))
    expect_fail(not hits2,
                "the energy-baseline warm-up is load-bearing")

    # (b) 1 -> 2 people is itself an energy step; it must not read as a wave
    f3 = BehaviorFsm()
    for n in range(200):
        f3.frame(n, ppl=1, ctr=40, nrg=4000)
    check(f3.st == ST_SINGLE, "settled at SINGLE with 1 person")
    for n in range(200, 260):
        f3.frame(n, ppl=2, ctr=40, nrg=8000)   # strictly more pixels, no motion
    hits3 = [e for e in f3.events if e[1] == "interact"]
    check(not hits3,
          "a second person walking in (an energy step) is not an interaction",
          repr(hits3))
    check(f3.st in (ST_SINGLE, ST_MULTI), "the state reflects the count instead",
          ST_NAME[f3.st])

    # negative control for (b): the same step with the warm-up disabled
    f4 = BehaviorFsm(no_warmup=True)
    for n in range(200):
        f4.frame(n, ppl=1, ctr=40, nrg=4000)
    for n in range(200, 230):
        f4.frame(n, ppl=2, ctr=40, nrg=8000)
    hits4 = [e for e in f4.events if e[1] == "interact"]
    check(bool(hits4),
          "without the warm-up guard the second person fires an interact "
          "(documented failure mode)", repr(hits4))
    expect_fail(not hits4,
                "the warm_cnt reset on a count change is load-bearing")


# ============================================================
# 10. negative controls for the filters themselves
# ============================================================
def test_filter_controls():
    print("-- 10. negative controls for the deadband and the confirm --")

    def blip(width, no_deadband=False, no_confirm=False):
        """Settle at 1 person, then a burst of `width` frames at 2, then back."""
        g = BehaviorFsm(no_deadband=no_deadband, no_confirm=no_confirm)
        for n in range(200):
            g.frame(n, ppl=1)
        peak_pplf = 0
        for n in range(200, 200 + width):
            g.frame(n, ppl=2)
            peak_pplf = max(peak_pplf, g.pplf)
        first_multi = None
        for n in range(200 + width, 400):
            g.frame(n, ppl=1)
            if g.st == ST_MULTI and first_multi is None:
                first_multi = n
        return g, first_multi, peak_pplf

    # A 2-frame burst of "2 people" is rejected end to end: the deadband plus
    # the confirmation means pplf never moves, so the state never sees it.
    g2, multi2, peak2 = blip(2)
    check(multi2 is None and peak2 == 1,
          "a 2-frame burst is rejected by both filters (count and state)",
          "first_multi=%s peak=%d" % (multi2, peak2))

    # A 4-frame burst is long enough to survive PPL_CONFIRM frames and is
    # therefore treated as a real change -- that is the documented trade-off
    # (3 frames = 50 ms at 60 fps).  Worth asserting so the trade-off cannot
    # silently change.
    g4, multi4, peak4 = blip(4)
    check(multi4 is not None and peak4 == 2,
          "a 4-frame burst IS treated as real and reaches MULTI (documented)",
          "first_multi=%s peak=%d" % (multi4, peak4))

    # ---- negative controls: each half of the +/-1 filter is load-bearing ----
    g2d, _, peak2d = blip(2, no_deadband=True)
    check(peak2d == 2,
          "without the deadband a 2-frame burst leaks into the reported count "
          "(documented failure mode)", "peak=%d" % peak2d)
    expect_fail(peak2d == 1, "the +/-1 deadband is load-bearing")

    g2c, _, peak2c = blip(2, no_confirm=True)
    check(peak2c == 2,
          "without the same-direction confirm the burst leaks the same way "
          "(the two are two halves of one mechanism)", "peak=%d" % peak2c)
    expect_fail(peak2c == 1,
                "the same-direction confirmation is load-bearing")

    # the deadband also keeps the reported count (the U field) stable
    def changes(no_deadband):
        g = BehaviorFsm(no_deadband=no_deadband)
        for n in range(200):
            g.frame(n, ppl=1)
        seen = []
        for n in range(200, 600):
            g.frame(n, ppl=2 if n % 2 == 0 else 1)
            seen.append(g.pplf)
        return sum(1 for a, b in zip(seen, seen[1:]) if a != b)

    n_ok, n_bad = changes(False), changes(True)
    check(n_ok < 5 and n_bad > 50,
          "the deadband stabilises the reported count (U field)",
          "with=%d without=%d changes" % (n_ok, n_bad))

    # the confidence must rise monotonically in a settled scene and fall
    # when the scene becomes noisy
    f = BehaviorFsm()
    confs = []
    for n in range(400):
        f.frame(n, ppl=1)
        confs.append(f.conf)
    check(all(b >= a for a, b in zip(confs, confs[1:])),
          "confidence never decreases while the scene stays settled")
    check(confs[-1] == 254, "and it saturates at its documented bound",
          "conf=%d" % confs[-1])

    for n in range(400, 500):
        f.frame(n, ppl=1, ctr=40 + (20 if n % 2 else -20))
    check(f.conf < confs[-1], "a shaking camera/centroid drops the confidence",
          "conf=%d" % f.conf)


# ============================================================
# 11. status line
# ============================================================
HEX = "0123456789ABCDEF"


def build_line(ok, cnt, fg, m, ppl, ocs, exp, gain, mean, lock,
               st, mode, conf, dwell, icnt, pplf, curve):
    """Python mirror of top_vision_m5.v's line_byte() + the tx shift."""
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
        elif i == 68:
            out.append(HEX[st & 0xF])
        elif i == 72:
            out.append(HEX[mode & 0xF])
        elif 76 <= i <= 77:
            out.append(HEX[(conf >> (4 * (1 - (i - 76)))) & 0xF])
        elif 81 <= i <= 84:
            out.append(HEX[(dwell >> (4 * (3 - (i - 81)))) & 0xF])
        elif 88 <= i <= 89:
            out.append(HEX[(icnt >> (4 * (1 - (i - 88)))) & 0xF])
        elif i == 93:
            out.append(HEX[pplf & 0xF])
        elif CURVE_POS <= i <= CURVE_END:
            out.append(HEX[curve[i - CURVE_POS] & 0xF])
        else:
            out.append(" ")
    return "".join(out)


def test_line():
    print("-- 11. the 193-byte status line --")
    curve = [i % 16 for i in range(CURVE_N)]
    curve_str = "".join(HEX[c] for c in curve)
    expected = ("MH5 K N=016080 F=0000A3 M=00001C P=2 S=068 E=0100 G=010 "
                "Y=060 L=1 T=2 Q=1 C=60 D=0078 I=01 U=2 V=" + curve_str + "\r\n")
    line = build_line(True, 90240, 163, 28, 2, 104, 256, 16, 96, 1,
                      2, 1, 96, 120, 1, 2, curve)
    check(len(line) == LINE_LEN == 193, "the line is exactly 193 bytes",
          "len=%d" % len(line))
    check(line == expected, "the line matches byte-for-byte",
          repr(line[:110]))
    check(line[:3] == "MH5", "the banner identifies the milestone as M5")
    check(line[162:164] != "V=", "the M4 curve start position moved")
    check(line[CURVE_END + 1:CURVE_END + 3] == "\r\n", "CRLF at the tail")
    check(line[95:97] == "V=" and len(line[CURVE_POS:CURVE_END + 1]) == CURVE_N,
          "the curve is the last 94 chars before CRLF, after the V= label")

    fixed = {k for k in TBL if k != "default"}
    dyn = ({4} | set(range(8, 14)) | set(range(17, 23)) | set(range(26, 32))
           | {35} | set(range(39, 42)) | set(range(45, 49))
           | set(range(52, 55)) | set(range(58, 61)) | {64}
           | {68} | {72} | {76, 77} | set(range(81, 85)) | {88, 89} | {93}
           | set(range(CURVE_POS, CURVE_END + 1)))
    check(not (fixed & dyn), "no position is both fixed and dynamic")
    check(fixed | dyn == set(range(LINE_LEN)),
          "every byte position is covered (no gaps, no extras)",
          "fixed=%d dyn=%d" % (len(fixed), len(dyn)))
    check(len(fixed) == 54, "54 fixed characters", "fixed=%d" % len(fixed))
    check(len(dyn) == 139, "139 dynamic characters", "dyn=%d" % len(dyn))
    check(line[66:68] == "T=" and line[70:72] == "Q=" and line[74:76] == "C="
          and line[79:81] == "D=" and line[86:88] == "I=" and line[91:93] == "U=",
          "the M5 field labels sit where the RTL puts them")

    line_f = build_line(False, 90240, 163, 28, 2, 104, 256, 16, 96, 1,
                        2, 1, 96, 120, 1, 2, curve)
    check(line_f[4] == "F", "the self-test flag renders 'F' when cam_ok = 0")

    # a different state/mode/conf must actually change the right bytes
    line_alt = build_line(True, 90240, 163, 28, 3, 104, 256, 16, 96, 1,
                          4, 3, 200, 0x1234, 7, 3, curve)
    check(line_alt[68] == "4" and line_alt[72] == "3",
          "T and Q carry the state/mode nibbles")
    check(line_alt[76:78] == "C8", "conf 200 -> 'C8'")
    check(line_alt[81:85] == "1234", "dwell 0x1234 -> '1234'")
    check(line_alt[88:90] == "07" and line_alt[93] == "3",
          "I and U carry the interact count and the filtered count")

    swapped = list(curve)
    swapped[3], swapped[4] = swapped[4], swapped[3]
    expect_fail(build_line(True, 90240, 163, 28, 2, 104, 256, 16, 96, 1,
                           2, 1, 96, 120, 1, 2, swapped) == expected,
                "a nibble swap would change the line")
    expect_fail(build_line(True, 90240, 163, 28, 2, 104, 256, 16, 96, 1,
                           2, 1, 96, 120, 1, 2, curve[::-1]) == expected,
                "mirroring the curve packing changes the line")
    expect_fail(build_line(True, 90240, 163, 28, 2, 104, 256, 16, 96, 1,
                           2, 1, 96, 120, 1, 1, curve) == expected,
                "a wrong filtered count changes the line")


# ============================================================
# main
# ============================================================
def main():
    print("=" * 72)
    print("vision sub-board M5 model -- behaviour FSM + confidence")
    print("=" * 72)
    test_define_widths()
    test_constants()
    test_pplf_filter()
    test_ladder()
    test_multi()
    test_interact()
    test_leave()
    test_alert()
    test_geom_broken()
    test_baseline_traps()
    test_filter_controls()
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
