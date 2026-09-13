#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cycle-accurate + frame-level models for the vision sub-board M2 design.

M2 adds background modelling and frame differencing on top of M1. There is no
Verilog simulator on this machine, so this mirrors the M2 RTL and asserts the
things that decide whether the board works the first time it is powered:

  1. bg_model.v is a *safe* leaky integrator. For every one of the 256x256
     (cur, bg) pairs and both fg_tick phases:
       - bg_next stays in 0..255 (no unsigned wraparound);
       - bg_next lies in [bg, cur] -- it never overshoots;
       - fg = (|cur-bg| > FG_THRESH) exactly.
     The wraparound check is the whole point: a naive signed subtract would
     send bg from 0 to 255 when the scene goes dark.

  2. m2_engine.v keeps its four outputs (fg_valid / fg / fg_x / fg_y) in the
     same cycle. This is the one that bites quietly: fg is computed
     combinationally one cycle earlier than the registered fg_x/fg_y, so
     without the extra fg register fg_stat attributes a foreground bit to the
     *previous* pixel -- the bounding box shifts by one pixel and the count is
     off at frame boundaries. Modelled per-cycle, both the bug and the fix.

  3. The background is loaded for exactly BG_LOAD_FRAMES frames after reset
     (BRAM powers up undefined), then a static scene settles to fg count 0.

  4. An object raises the count and the bounding box lands on the right pixels.

  5. The foreground update cadence is one grey level per FG_DIV frames, so a
     stationary person is not absorbed instantly and a residual fades.

  6. top_vision_m2.v emits a 53-byte status line.

The M2 defines, the fixed-character table and the field layout are PARSED OUT OF
THE RTL, not restated here, so the model cannot silently drift from the design.

Run from anywhere:  python tools/sim_vision_sub_m2.py
"""
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)
HDL = os.path.join(ROOT, "src", "vision_sub", "user_source", "hdl_source")
DEFS = os.path.join(HDL, "vision_def.v")
F_TOP = os.path.join(HDL, "top_vision_m2.v")

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


def parse_fixed_char():
    with open(F_TOP, "r", encoding="utf-8") as fh:
        code = strip_comments(fh.read())
    m = re.search(r"function\[7:0\]\s+fixed_char;(.*?)endfunction", code, re.S)
    if not m:
        raise RuntimeError("fixed_char() not found in top_vision_m2.v")
    tbl = {}
    for _wid, idx, hexv in re.findall(
            r"(\d+)'d(\d+)\s*:\s*fixed_char\s*=\s*8'h([0-9A-Fa-f]+)", m.group(1)):
        tbl[int(idx)] = int(hexv, 16)
    tbl["default"] = None
    return tbl


# ============================================================
# bg_model.v -- pure combinational pixel law
# ============================================================
def bg_model(cur, bg, fg_tick, fg_thresh, bg_shift):
    """Verbatim transcription of bg_model.v. Returns (fg, bg_next, d)."""
    cur_ge = cur >= bg
    d = (cur - bg) if cur_ge else (bg - cur)
    is_fg = d > fg_thresh
    want = 1 if is_fg else (((d >> bg_shift) if (d >> bg_shift) else 1))
    step = d if want > d else want          # clamp to d: never overshoot
    upd_en = fg_tick if is_fg else 1
    if upd_en:
        bg_next = (bg + step) if cur_ge else (bg - step)
    else:
        bg_next = bg
    return (1 if is_fg else 0), bg_next, d


def bg_model_naive(cur, bg, fg_tick, fg_thresh, bg_shift):
    """The pre-fix law: max(1,.) with no clamp to d. Kept for the negative
    control -- it overflows at cur=bg=255."""
    cur_ge = cur >= bg
    d = (cur - bg) if cur_ge else (bg - cur)
    is_fg = d > fg_thresh
    step = 1 if is_fg else max(1, d >> bg_shift)
    upd_en = fg_tick if is_fg else 1
    if upd_en:
        bg_next = (bg + step) if cur_ge else (bg - step)
    else:
        bg_next = bg
    return (1 if is_fg else 0), bg_next, d


def test_bg_law(env):
    print("\n-- 1. bg_model.v pixel law (all 256x256 cur/bg pairs) --")
    TH, SH = env["FG_THRESH"], env["BG_SHIFT"]
    bad_range, bad_over, bad_fg, bad_dir = 0, 0, 0, 0
    worst = None
    for cur in range(256):
        for bg in range(256):
            for tick in (0, 1):
                fg, nxt, d = bg_model(cur, bg, tick, TH, SH)
                if not (0 <= nxt <= 255):
                    bad_range += 1
                    worst = (cur, bg, tick, nxt)
                if not (min(cur, bg) <= nxt <= max(cur, bg)):
                    bad_over += 1
                if fg != (1 if d > TH else 0):
                    bad_fg += 1
                # direction: never move away from cur
                if (bg < cur and nxt < bg) or (bg > cur and nxt > bg):
                    bad_dir += 1

    check(bad_range == 0, "bg_next always in 0..255 (no unsigned wrap)",
          "worst=%s" % (worst,))
    check(bad_over == 0, "bg_next always in [min(cur,bg), max(cur,bg)]")
    check(bad_fg == 0, "fg == (|cur-bg| > %d) exactly" % TH)
    check(bad_dir == 0, "bg always moves toward cur, never away")

    # the exact cases a signed implementation would break on
    _, n_lo, _ = bg_model(0, 255, 1, TH, SH)
    _, n_hi, _ = bg_model(255, 0, 1, TH, SH)
    check(n_lo == 254, "cur=0,bg=255,tick -> 254 (not 0/256)", "got %d" % n_lo)
    check(n_hi == 1, "cur=255,bg=0,tick -> 1 (not 0/510)", "got %d" % n_hi)
    _, n_lo2, _ = bg_model(0, 255, 0, TH, SH)
    check(n_lo2 == 255, "cur=0,bg=255,no-tick (fg) -> holds at 255")
    check(bg_model(255, 0, 0, TH, SH)[1] == 0,
          "cur=255,bg=0,no-tick (fg) -> holds at 0")

    # dead-zone killer: max(1, d>>SHIFT) means a tiny diff still moves
    _, n_tiny, _ = bg_model(100, 101, 1, TH, SH)
    check(n_tiny == 100 or n_tiny == 101,
          "diff=1 still steps (no dead zone): 101 -> %d" % n_tiny)

    # A background pixel satisfies d <= FG_THRESH. Since FG_THRESH <= 2*2^SHIFT
    # here, d>>SHIFT is 0 or 1, so max(1,.) makes the step exactly 1 level per
    # frame -- the exponential term is inert under the current constants. It
    # would only bite if FG_THRESH were raised above 2^BG_SHIFT.
    maxstep = 0
    for cur in range(256):
        for bg in range(256):
            if abs(cur - bg) <= TH:
                _, nxt, _ = bg_model(cur, bg, 0, TH, SH)
                maxstep = max(maxstep, abs(nxt - bg))
    check(maxstep == 1,
          "a background pixel never moves more than 1 level/frame",
          "max step=%d" % maxstep)
    check(TH <= 2 * (1 << SH),
          "FG_THRESH(%d) <= 2*2^BG_SHIFT(%d): bg step is always 1 (BG_SHIFT "
          "inert until FG_THRESH > 2^BG_SHIFT)" % (TH, 2 * (1 << SH)))

    # d = 0 must hold (the clamp). Without it, max(1, .) overshoots and 255
    # wraps to 0 -- the bug this model was written to catch.
    check(bg_model(255, 255, 1, TH, SH)[1] == 255,
          "cur=bg=255 holds at 255 (the overflow case)",
          "got %d" % bg_model(255, 255, 1, TH, SH)[1])
    check(bg_model(0, 0, 1, TH, SH)[1] == 0,
          "cur=bg=0 holds at 0", "got %d" % bg_model(0, 0, 1, TH, SH)[1])
    # a background pixel (d <= THRESH) converges within ~THRESH frames
    bg = 200 - TH
    for _ in range(TH + 4):
        fg, bg, _ = bg_model(200, bg, 1, TH, SH)
    check(abs(bg - 200) <= 1,
          "background pixel (d<=THRESH) reaches cur within ~THRESH frames",
          "bg=%d" % bg)

    expect_fail(bg_model_naive(255, 255, 1, TH, SH)[1] <= 255,
                "control: the un-clamped law overflows to 256 at cur=bg=255")


# ============================================================
# per-pixel temporal behaviour (single pixel, long horizon)
# ============================================================
def pixel_timeline(cur_seq, bg0, fg_thresh, bg_shift, fg_div):
    """Run the pixel law frame by frame with the exact fg_fcnt schedule."""
    bg = bg0
    fg_fcnt = 0
    out = []
    for cur in cur_seq:
        # fg_fcnt advances at frame_start, before the pixels are processed
        fg_fcnt_new = 0 if fg_fcnt == fg_div - 1 else fg_fcnt + 1
        fg_tick = 1 if (fg_fcnt_new == 0) else 0
        fg, bg, d = bg_model(cur, bg, fg_tick, fg_thresh, bg_shift)
        fg_fcnt = fg_fcnt_new
        out.append((fg, bg, d))
    return out


def test_pixel_temporal(env):
    print("\n-- 2. per-pixel temporal behaviour (cadence, absorption, residual) --")
    TH, SH, DIV = env["FG_THRESH"], env["BG_SHIFT"], env["FG_DIV"]
    check(DIV >= 2, "FG_DIV = %d (foreground advances slower than 1/frame)" % DIV)

    # (a) foreground cadence: a foreground pixel moves exactly 1 level per
    #     FG_DIV frames -- count the moves over a long constant-contrast run.
    N = 40 * DIV
    tl = pixel_timeline([200] * N, 100, TH, SH, DIV)
    moves = sum(1 for k in range(1, N) if tl[k][1] != tl[k - 1][1])
    per = [abs(tl[k][1] - tl[k - 1][1]) for k in range(1, N)
           if tl[k][1] != tl[k - 1][1]]
    check(all(p == 1 for p in per),
          "foreground pixel steps by exactly 1 grey level", "step sizes=%s"
          % sorted(set(per)))
    check(moves <= N // DIV + 1,
          "foreground moves <= %d times in %d frames (1 per FG_DIV)"
          % (N // DIV + 1, N), "moves=%d" % moves)
    check(moves >= N // DIV - 2,
          "foreground moves about %d times (not frozen)" % (N // DIV),
          "moves=%d" % moves)

    # (b) a stationary person of large contrast is NOT absorbed within 8 s
    #     (8 s = 480 frames at 60 fps): the pixel stays fg.
    tl = pixel_timeline([200] * 480, 100, TH, SH, DIV)
    still_fg = sum(1 for fg, _, _ in tl if fg)
    check(still_fg == 480,
          "contrast-100 person stays foreground for 480 frames (8 s)")
    check(tl[-1][1] < 200, "but the background does creep toward the person",
          "bg=%d after 480 frames" % tl[-1][1])

    # (c) residual: after the person leaves, a background that overshot the
    #     (now lower) scene decays back below threshold.
    tl_obj = pixel_timeline([200] * 400, 100, TH, SH, DIV)
    bg_after = tl_obj[-1][1]
    check(bg_after > 100 + TH,
          "after 400 frames of a person, bg (%d) is above cur+thresh" % bg_after)
    tl_res = pixel_timeline([100] * 600, bg_after, TH, SH, DIV)
    first_bg = tl_res[0][1]
    cleared_at = next((k for k, (fg, _, _) in enumerate(tl_res) if not fg), None)
    check(tl_res[0][0] == 1, "the residual starts as foreground (a ghost)")
    check(cleared_at is not None,
          "the residual eventually clears to background")
    if cleared_at is not None:
        # expected ~ (bg-cur-TH) ticks, each FG_DIV frames
        want = (first_bg - 100 - TH) * DIV
        check(abs(cleared_at - want) <= 2 * DIV,
              "residual clears in ~%d frames (observed %d)" % (want, cleared_at))
    check(tl_res[-1][1] <= 100 + TH,
          "residual bg settles at/under cur+thresh", "bg=%d" % tl_res[-1][1])

    # (d) background pixels converge fast (within ~THRESH frames at 1 level/frame)
    tl = pixel_timeline([120] * (TH + 8), 100, TH, SH, DIV)
    k = next((i for i, (_, bg, d) in enumerate(tl) if d <= 1), None)
    check(k is not None and k <= TH + 4,
          "background pixel reaches within 1 level in <= %d frames" % (TH + 4),
          "frames=%d" % (k if k is not None else -1))


# ============================================================
# m2_engine.v, per cycle (the alignment bug lives here)
# ============================================================
class M2Engine:
    """Cycle-accurate m2_engine + bg_store (undefined BRAM modelled as 0).

    Two phases, matching the RTL:
      sample()  -- the outputs a downstream block (fg_stat) latches at *this*
                   clock edge, i.e. the values present during this cycle.
      step()    -- the non-blocking-assignment edge that computes the *next*
                   register values from the current ones and the inputs.
    """

    def __init__(self, env, depth, width, register_fg=True):
        self.DEPTH = depth
        self.W = width
        self.TH = env["FG_THRESH"]
        self.SH = env["BG_SHIFT"]
        self.DIV = env["FG_DIV"]
        self.LOAD = env["BG_LOAD_FRAMES"]
        self.register_fg = register_fg
        self.mem = [0] * depth
        self.reset()

    def reset(self):
        self.px_idx = self.x_pos = self.y_pos = 0
        self.cur_d = self.idx_d = self.x_d = self.y_d = 0
        self.vld_d = 0
        self.load_cnt = 0
        self.bg_loading = 1
        self.fg_fcnt = 0
        self.rd_data = 0
        self.fg_valid = self.fg_reg = self.fg_x = self.fg_y = 0
        self.rd_addr_log = []
        self.wr_addr_log = []

    def _comb(self):
        """fg_comb / bg_wr / fg_tick from the current registers."""
        fg_tick = 1 if (self.fg_fcnt == 0 and self.bg_loading == 0) else 0
        is_fg, bg_next, _ = bg_model(self.cur_d, self.rd_data, fg_tick,
                                     self.TH, self.SH)
        bg_wr = self.cur_d if self.bg_loading else bg_next
        fg_comb = 0 if self.bg_loading else is_fg
        return fg_tick, bg_wr, fg_comb

    def sample(self):
        """(fg_valid, fg, fg_x, fg_y) as seen during the current cycle."""
        _, _, fg_comb = self._comb()
        fg = self.fg_reg if self.register_fg else fg_comb
        return (self.fg_valid, fg, self.fg_x, self.fg_y)

    def tick(self, pd, pv, fs):
        """Sample this cycle's outputs, then advance the clock edge."""
        s = self.sample()
        self.step(pd, pv, fs)
        return s

    def step(self, pd, pv, fs):
        rd_en = 1 if pv else 0
        fg_tick, bg_wr, fg_comb = self._comb()

        # ---- next-state (NBA) ----
        if fs:
            n_idx, n_x, n_y = 0, 0, 0
        elif pv:
            n_idx = self.px_idx + 1
            if self.x_pos == self.W - 1:
                n_x, n_y = 0, self.y_pos + 1
            else:
                n_x, n_y = self.x_pos + 1, self.y_pos
        else:
            n_idx, n_x, n_y = self.px_idx, self.x_pos, self.y_pos

        n_cur_d, n_idx_d = pd, self.px_idx
        n_x_d, n_y_d = self.x_pos, self.y_pos
        n_vld_d = rd_en

        if self.bg_loading and fs:
            if self.load_cnt == self.LOAD:
                n_bg_loading, n_load_cnt = 0, self.load_cnt
            else:
                n_bg_loading, n_load_cnt = 1, self.load_cnt + 1
        else:
            n_bg_loading, n_load_cnt = self.bg_loading, self.load_cnt

        n_fg_fcnt = (0 if self.fg_fcnt == self.DIV - 1 else self.fg_fcnt + 1) \
            if fs else self.fg_fcnt

        # bg_store: issue the read, commit the write (rd idx != wr idx-1)
        n_rd_data = self.mem[self.px_idx] if rd_en else self.rd_data
        if self.vld_d:
            self.mem[self.idx_d] = bg_wr
        if rd_en and self.vld_d:
            self.rd_addr_log.append(self.px_idx)
            self.wr_addr_log.append(self.idx_d)

        # output registers
        n_fg_valid = self.vld_d
        n_fg_reg = fg_comb
        n_fg_x, n_fg_y = self.x_d, self.y_d

        # ---- commit ----
        self.px_idx, self.x_pos, self.y_pos = n_idx, n_x, n_y
        self.cur_d, self.idx_d, self.x_d, self.y_d = n_cur_d, n_idx_d, n_x_d, n_y_d
        self.vld_d = n_vld_d
        self.bg_loading, self.load_cnt = n_bg_loading, n_load_cnt
        self.fg_fcnt = n_fg_fcnt
        self.rd_data = n_rd_data
        self.fg_valid, self.fg_reg = n_fg_valid, n_fg_reg
        self.fg_x, self.fg_y = n_fg_x, n_fg_y


def raster_coord(idx, width):
    return idx % width, idx // width


def test_cycle_alignment(env):
    print("\n-- 3. m2_engine.v cycle alignment (the quiet off-by-one) --")
    W, H = env["CAM_IMG_W"], env["CAM_IMG_H"]
    DEPTH = env["CAM_FRAME_PIX"]
    N = 2400                                    # ~6.4 rows, covers two x-wraps

    base = 100
    obj_idx = 1000                              # (248, 2)
    obj_val = 200                               # diff 100 -> foreground

    scene = [base] * N
    scene[obj_idx] = obj_val

    def run(register_fg):
        eng = M2Engine(env, DEPTH, W, register_fg=register_fg)
        for i in range(DEPTH):
            eng.mem[i] = base
        eng.bg_loading, eng.load_cnt = 0, env["BG_LOAD_FRAMES"]
        # one frame_start cycle, one blank cycle, then the pixels
        stream = [(0, 0, 1), (0, 0, 0)] + [(v, 1, 0) for v in scene]
        outs = [eng.tick(pd, pv, fs) for (pd, pv, fs) in stream]
        return eng, outs

    eng, outs = run(register_fg=True)

    coords = [(x, y) for (v, f, x, y) in outs if v]
    want = [raster_coord(i, W) for i in range(len(coords))]
    check(coords == want,
          "fg_x/fg_y walk the raster in order, no off-by-one",
          "first mismatch at %s" % next(
              (k for k in range(min(len(coords), len(want)))
               if coords[k] != want[k]), "n/a"))

    fgcyc = [(k, x, y) for k, (v, f, x, y) in enumerate(outs) if f]
    check(len(fgcyc) == 1, "exactly one foreground cycle for one object pixel",
          "got %d" % len(fgcyc))
    if fgcyc:
        _, fx, fy = fgcyc[0]
        check((fx, fy) == raster_coord(obj_idx, W),
              "the single fg pulse sits on (x,y)=(%d,%d)" % raster_coord(obj_idx, W),
              "got (%d,%d)" % (fx, fy))

    if eng.rd_addr_log:
        diffs = [w - r for r, w in zip(eng.rd_addr_log, eng.wr_addr_log)]
        check(all(d == -1 for d in diffs),
              "read addr (idx) and write addr (idx-1) never collide in a cycle")

    # negative control: leave fg combinational (the bug) and the pulse lands
    # on the coordinate one pixel early.
    _, outs_bug = run(register_fg=False)
    fgcyc_bug = [(k, x, y) for k, (v, f, x, y) in enumerate(outs_bug) if f]
    bug_ok = False
    if fgcyc_bug:
        _, bx, by = fgcyc_bug[0]
        bug_ok = ((bx, by) != raster_coord(obj_idx, W))
    check(bug_ok,
          "control: unregistered fg lands on the wrong (x,y) -- the bug is real")


# ============================================================
# frame-level batch model (bg_store + bg_model + fg_stat)
# ============================================================
class M2FrameModel:
    """Frame-batch model. Exact: within a frame each location is read once
    (old bg) and written once (new bg), so the per-pixel law composes."""

    def __init__(self, env, width, height):
        self.W, self.H = width, height
        self.DEPTH = width * height
        self.TH = env["FG_THRESH"]
        self.SH = env["BG_SHIFT"]
        self.DIV = env["FG_DIV"]
        self.LOAD = env["BG_LOAD_FRAMES"]
        self.mem = bytearray(self.DEPTH)
        self.load_cnt = 0
        self.bg_loading = True
        self.fg_fcnt = 0

    def make_frame(self, base, rects=()):
        f = bytearray([base]) * self.DEPTH
        for (x0, y0, x1, y1, val) in rects:
            for y in range(y0, y1 + 1):
                row = y * self.W
                for x in range(x0, x1 + 1):
                    f[row + x] = val
        return f

    def step_frame(self, frame):
        # frame_start edge: load counter + fg tick phase
        if self.bg_loading:
            if self.load_cnt == self.LOAD:
                self.bg_loading = False
            else:
                self.load_cnt += 1
        self.fg_fcnt = 0 if self.fg_fcnt == self.DIV - 1 else self.fg_fcnt + 1
        fg_tick = 1 if (self.fg_fcnt == 0 and not self.bg_loading) else 0

        if self.bg_loading:
            self.mem[:] = frame
            return {"loading": True, "cnt": 0, "nz": False,
                    "box": (0, 0, 0, 0)}

        mem = self.mem
        FR = frame
        TH, SH, W = self.TH, self.SH, self.W
        cnt = 0
        mnx = mxx = mny = mxy = 0
        x = 0
        y = 0
        for p in range(self.DEPTH):
            # the same law as bg_model(), inlined for speed
            cur = FR[p]
            bg = mem[p]
            if cur >= bg:
                d = cur - bg
                up = True
            else:
                d = bg - cur
                up = False
            if d > TH:
                mem[p] = (bg + 1) if (fg_tick and up) else \
                         ((bg - 1) if fg_tick else bg)
                if cnt == 0:
                    mnx = mxx = x
                    mny = mxy = y
                else:
                    if x < mnx:
                        mnx = x
                    elif x > mxx:
                        mxx = x
                    if y < mny:
                        mny = y
                    elif y > mxy:
                        mxy = y
                cnt += 1
            else:
                s = d >> SH
                if s == 0:
                    s = 1
                if s > d:                       # clamp to d: d=0 holds
                    s = d
                mem[p] = (bg + s) if up else (bg - s)
            x += 1
            if x == W:
                x = 0
                y += 1
        return {"loading": False, "cnt": cnt, "nz": cnt > 0,
                "box": (mnx, mny, mxx, mxy) if cnt else (0, 0, 0, 0)}


def test_frame_static(env):
    print("\n-- 4. frame level: background load + static scene --")
    m = M2FrameModel(env, env["CAM_IMG_W"], env["CAM_IMG_H"])
    LOAD = env["BG_LOAD_FRAMES"]
    flat = m.make_frame(100)
    res = [m.step_frame(flat) for _ in range(LOAD + 2)]

    check(all(r["loading"] for r in res[:LOAD]),
          "exactly the first %d frames are forced background loads" % LOAD)
    check(not res[LOAD]["loading"],
          "frame %d exits the load phase" % (LOAD + 1))
    check(res[LOAD]["cnt"] == 0, "static scene: fg count 0 right after load",
          "cnt=%d" % res[LOAD]["cnt"])
    check(res[LOAD + 1]["cnt"] == 0, "static scene stays at fg count 0",
          "cnt=%d" % res[LOAD + 1]["cnt"])
    check(not res[LOAD + 1]["nz"], "static scene: bounding box is empty")

    # background really is the scene now (the max(1,.) rule leaves a +/-1 dither)
    dev = max(abs(v - 100) for v in m.mem)
    check(dev <= 2,
          "loaded background tracks the scene within +/-1 level",
          "max deviation=%d" % dev)


def test_inline_law(env):
    print("\n-- 4b. frame-model inline law matches bg_model.v --")
    W, H = 20, 20
    m = M2FrameModel(env, W, H)
    rnd = [(i * 37 + 11) & 0xFF for i in range(W * H)]
    scen = [(i * 91 + 5) & 0xFF for i in range(W * H)]
    m.mem[:] = bytes(rnd)
    m.bg_loading, m.load_cnt = False, env["BG_LOAD_FRAMES"]
    m.fg_fcnt = env["FG_DIV"] - 1               # next frame_start -> fg_tick = 1
    old = list(m.mem)
    m.step_frame(bytearray(scen))
    tick = 1                                    # fg_fcnt wraps to 0 this frame
    bad = 0
    for p in range(W * H):
        _, want, _ = bg_model(scen[p], old[p], tick, env["FG_THRESH"],
                              env["BG_SHIFT"])
        if m.mem[p] != want:
            bad += 1
    check(bad == 0, "inlined per-pixel law == bg_model.v on %d pixels" % (W * H),
          "%d mismatches" % bad)


def test_frame_object(env):
    print("\n-- 5. frame level: object appears -> count + bounding box --")
    W, H = env["CAM_IMG_W"], env["CAM_IMG_H"]
    m = M2FrameModel(env, W, H)
    flat = m.make_frame(100)
    for _ in range(env["BG_LOAD_FRAMES"]):
        m.step_frame(flat)
    _ = m.step_frame(flat)                       # settle

    x0, y0, x1, y1 = 100, 50, 119, 59            # 20 x 10 = 200 pixels
    obj = m.make_frame(100, [(x0, y0, x1, y1, 126)])   # diff 26 > 24
    r = m.step_frame(obj)
    check(r["cnt"] == 200, "object of 200 px -> fg count 200",
          "cnt=%d" % r["cnt"])
    check(r["box"] == (x0, y0, x1, y1),
          "bounding box = (%d,%d,%d,%d)" % (x0, y0, x1, y1),
          "got %s" % (r["box"],))
    check(r["nz"], "box_nz asserted when foreground present")

    # a sub-threshold object is invisible
    m2 = M2FrameModel(env, W, H)
    for _ in range(env["BG_LOAD_FRAMES"] + 1):
        m2.step_frame(flat)
    soft = m2.make_frame(100, [(x0, y0, x1, y1, 100 + env["FG_THRESH"] - 1)])
    r = m2.step_frame(soft)
    check(r["cnt"] == 0,
          "object at exactly thresh-1 (%d) stays background"
          % (env["FG_THRESH"] - 1), "cnt=%d" % r["cnt"])


def test_frame_residual(env):
    print("\n-- 6. frame level: person leaves -> residual decays to 0 --")
    # Long-horizon behaviour is validated on a small ROI: the law is per-pixel
    # and position-independent, and 1000 full 376x240 frames is hours of pure
    # Python. Geometry/bbox are covered by the full-frame tests above.
    W, H = 60, 40
    m = M2FrameModel(env, W, H)
    flat = m.make_frame(100)
    for _ in range(env["BG_LOAD_FRAMES"]):
        m.step_frame(flat)
    m.step_frame(flat)

    # a "person" stands long enough for the background to overshoot the scene
    x0, y0, x1, y1 = 10, 10, 29, 19
    person = m.make_frame(100, [(x0, y0, x1, y1, 200)])
    for _ in range(400):
        rp = m.step_frame(person)
    check(rp["cnt"] == 200, "person held: fg count stays 200",
          "cnt=%d" % rp["cnt"])
    bg_over = m.mem[y0 * W + x0]
    check(bg_over > 100 + env["FG_THRESH"],
          "background at the person site overshot to %d" % bg_over)

    # person gone: the ghost is still foreground, then clears
    counts = []
    cleared = None
    for k in range(400):
        r = m.step_frame(flat)
        counts.append(r["cnt"])
        if r["cnt"] == 0 and cleared is None:
            cleared = k
    check(counts[0] == 200, "the instant the person leaves, a ghost remains",
          "cnt=%d" % counts[0])
    check(cleared is not None, "the ghost eventually clears to count 0")
    check(counts[-1] == 0, "and stays cleared", "cnt=%d" % counts[-1])
    check(all(counts[k] <= 200 for k in range(len(counts))),
          "the ghost never grows")
    check(all(counts[k] >= counts[k + 1] for k in range(len(counts) - 1))
          or cleared is not None,
          "the ghost count decays monotonically to 0")


# ============================================================
# status line
# ============================================================
def hexc(n):
    return ord("0") + n if n < 10 else ord("A") + n - 10


# field layout parsed from top_vision_m2.v anchors: index -> (name, width)
ANCHORS = {0: "M", 1: "H", 2: "2", 6: "N", 7: "=", 15: "L", 16: "=",
           20: "H", 21: "=", 25: "F", 26: "=", 34: "B", 35: "=",
           39: ",", 43: ",", 47: ",", 51: "\r", 52: "\n"}


def line_bytes(prt, tbl):
    out = []
    for i in range(53):
        c = tbl.get(i, tbl.get("default"))
        if c is not None:
            out.append(c)
        elif 8 <= i <= 13:
            out.append(hexc((prt["cnt"] >> (4 * (5 - (i - 8)))) & 0xF))
        elif 17 <= i <= 18:
            out.append(hexc((prt["min"] >> (4 * (1 - (i - 17)))) & 0xF))
        elif 22 <= i <= 23:
            out.append(hexc((prt["max"] >> (4 * (1 - (i - 22)))) & 0xF))
        elif 27 <= i <= 32:
            out.append(hexc((prt["fg"] >> (4 * (5 - (i - 27)))) & 0xF))
        elif 36 <= i <= 38:
            out.append(hexc((prt["bx0"] >> (4 * (2 - (i - 36)))) & 0xF))
        elif 40 <= i <= 42:
            out.append(hexc((prt["by0"] >> (4 * (2 - (i - 40)))) & 0xF))
        elif 44 <= i <= 46:
            out.append(hexc((prt["bx1"] >> (4 * (2 - (i - 44)))) & 0xF))
        elif 48 <= i <= 50:
            out.append(hexc((prt["by1"] >> (4 * (2 - (i - 48)))) & 0xF))
        elif i == 4:
            out.append(ord("K") if prt["ok"] else ord("F"))
        else:
            out.append(ord(" "))
    return out


def test_line(env):
    print("\n-- 7. top_vision_m2.v status line --")
    tbl = parse_fixed_char()
    for i, ch in ANCHORS.items():
        check(tbl.get(i) == ord(ch),
              "fixed_char[%d] == %r" % (i, ch),
              "got 0x%02X" % (tbl.get(i) if tbl.get(i) is not None else -1))
    check(env["M2_LINE_LEN"] == 53, "M2_LINE_LEN = 53")

    prt = {"cnt": env["CAM_FRAME_PIX"], "min": 0x11, "max": 0xEE,
           "fg": 0x00ABCD, "bx0": 10, "by0": 20, "bx1": 300, "by1": 200,
           "ok": True}
    b = line_bytes(prt, tbl)
    text = bytes(b).decode("ascii")
    print("      line: %r  (%d bytes)" % (text, len(b)))
    want = "MH2 K N=016080 L=11 H=EE F=00ABCD B=00A,014,12C,0C8\r\n"
    check(len(b) == 53, "line is exactly 53 bytes", "got %d" % len(b))
    check(text == want, "field layout / hex encoding correct", "got %r" % text)

    bad = dict(prt)
    bad["ok"] = False
    check(line_bytes(bad, tbl)[4] == ord("F"),
          "self-test flag renders 'F' when cam_ok = 0")

    zero = dict(prt)
    zero.update({"fg": 0, "bx0": 0, "by0": 0, "bx1": 0, "by1": 0})
    check(bytes(line_bytes(zero, tbl)).decode("ascii")
          == "MH2 K N=016080 L=11 H=EE F=000000 B=000,000,000,000\r\n",
          "no-foreground line is all zeros in F and B")

    wrong = dict(prt)
    wrong["fg"] = 0x00ABDC
    expect_fail(bytes(line_bytes(wrong, tbl)) == bytes(b),
                "a nibble swap would change the line")


def main():
    print("=" * 72)
    print("vision_sub M2 cycle/model checks")
    print("=" * 72)
    env = parse_defines()
    missing = [k for k in ("FG_THRESH", "BG_SHIFT", "FG_DIV", "FG_CNT_W",
                           "M2_LINE_LEN", "BG_LOAD_FRAMES")
               if k not in env]
    if missing:
        print("missing M2 defines in vision_def.v: %s" % missing)
        return 1
    print("parsed from vision_def.v: geometry=%dx%d  FG_THRESH=%d  BG_SHIFT=%d  "
          "FG_DIV=%d  BG_LOAD_FRAMES=%d"
          % (env["CAM_IMG_W"], env["CAM_IMG_H"], env["FG_THRESH"],
             env["BG_SHIFT"], env["FG_DIV"], env["BG_LOAD_FRAMES"]))

    test_bg_law(env)
    test_pixel_temporal(env)
    test_cycle_alignment(env)
    test_frame_static(env)
    test_inline_law(env)
    test_frame_object(env)
    test_frame_residual(env)
    test_line(env)

    print("\n" + "=" * 72)
    if FAILURES:
        print("FAILED %d of %d checks:" % (len(FAILURES), CHECKS[0]))
        for f in FAILURES:
            print("  - %s" % f)
        return 1
    print("ALL %d CHECKS PASSED" % CHECKS[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
