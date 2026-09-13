#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cycle-accurate + frame-level models for the vision sub-board M3 design.

M3 adds morphology, row/column projection and people segmentation on top of M2.
There is no Verilog simulator on this machine, so this mirrors the M3 RTL and
asserts the things that decide whether the board works the first time it is
powered:

  1. morph3x3.v's line buffers are *shift registers*, and the alignment holds
     only because every row is exactly W pixels wide. Modelled cycle by cycle,
     and a *negative control* shows what happens if a stage instead gates its
     valid bit at the border (3 shifts missing per row -> the rows drift).

  2. morph.v (erode -> dilate) equals the textbook 3x3 opening on the interior
     region, and it really does delete an isolated 1-pixel speckle.

  3. projection.v's readout is exactly 154 cycles (94 column bins + 60 row
     bins), the curve nibbles come out bin 0 first, and the bins are cleared as
     they are read. Negative control: registering ro_bin reads a stale bin on
     the first cycle and pre-clears bin 59.

  4. Reading out disables the accumulator, and that loses nothing -- the readout
     is shorter than one row, so the skipped cycles are inside the first row,
     which morph discards anyway (y >= 4).

  5. people_seg.v turns a column projection into a head count: threshold, run
     segmentation with a min-width and a min-gap filter.

  6. top_vision_m3.v emits a 141-byte status line.

The M3 defines and the fixed-character table are PARSED OUT OF THE RTL, not
restated here, so the model cannot silently drift from the design.

Run from anywhere:  python tools/sim_vision_sub_m3.py
"""
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)
HDL = os.path.join(ROOT, "src", "vision_sub", "user_source", "hdl_source")
DEFS = os.path.join(HDL, "vision_def.v")
F_TOP = os.path.join(HDL, "top_vision_m3.v")

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
            r"(\d+)'d(\d+)\s*:\s*fixed_char\s*=\s*8'h([0-9A-Fa-f]+)", m.group(1)):
        tbl[int(idx)] = int(hexv, 16)
    tbl["default"] = None
    return tbl


# ============================================================
# morph.v -- raster helpers + reference morphology
# ============================================================
def blank_image(w, h):
    return [[0] * w for _ in range(h)]


def fill_rect(img, x0, y0, x1, y1, val=1):
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            img[y][x] = val


def ref_morph(img, w, h, is_erode):
    """Plain 3x3 erode/dilate, out-of-frame reads as 0.

    Only called on regions where the stencil is fully in frame, so the padding
    convention never actually matters -- see morph.v's comments.
    """
    out = blank_image(w, h)
    for y in range(h):
        for x in range(w):
            acc = 1 if is_erode else 0
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    yy, xx = y + dy, x + dx
                    v = img[yy][xx] if (0 <= yy < h and 0 <= xx < w) else 0
                    acc = (acc & v) if is_erode else (acc | v)
            out[y][x] = acc
    return out


def ref_open(img, w, h):
    return ref_morph(ref_morph(img, w, h, True), w, h, False)


def raster(img, w, h, frame_blank=0):
    """Yield (v, b, x, y, frame_start) once per PCLK cycle.

    Pixels arrive HREF-contiguous (376 per row, exactly what m2_engine and
    morph rely on), then frame_blank idle cycles, then the next frame_start.
    A single frame is returned unless the image is a list of frames.
    """
    def one_frame(f):
        for y in range(h):
            for x in range(w):
                yield (1, f[y][x], x, y, 0)
        for _ in range(frame_blank):
            yield (0, 0, 0, 0, 0)

    if isinstance(img, tuple) and img and isinstance(img[0], list) \
            and isinstance(img[0][0], list):
        frames = list(img)
    else:
        frames = [img]

    first = True
    for f in frames:
        yield (0, 0, 0, 0, 1)          # vsync rise: frame_start pulse
        for cyc in one_frame(f):
            yield cyc
        # the frame_start of the next frame is emitted by the next iteration
    _ = first


# ============================================================
# morph3x3 -- one 3x3 stage, cycle accurate
# ============================================================
class Morph3x3:
    def __init__(self, w, is_erode, gate_valid=False):
        self.w = w
        self.ero = is_erode
        self.gate_valid = gate_valid      # buggy variant, see test_morph_stream
        self.lb0 = 0
        self.lb1 = 0
        self.t0 = self.t1 = self.t2 = 0
        self.vo = 0
        self.mo = 0
        self.xo = 0
        self.yo = 0

    def _window(self):
        return ((self.t2 & 7) << 6) | ((self.t1 & 7) << 3) | (self.t0 & 7)

    def _m_bit(self):
        win = self._window()
        return 1 if (win == 0o777 if self.ero else win != 0) else 0

    def comb(self, v, x, y):
        """Outputs visible during this cycle (registered values, pre-edge)."""
        if not self.vo:
            return (0, 0, self.xo, self.yo)
        return (1, self.mo, self.xo, self.yo)

    def step(self, v, b, x, y):
        m_bit = self._m_bit()
        xo_n = 0 if x < 2 else x - 2
        yo_n = 0 if y < 1 else y - 1
        p1 = self.lb0 & 1
        p2 = self.lb1 & 1
        if v:
            self.lb0 = ((b & 1) << (self.w - 1)) | (self.lb0 >> 1)
            self.lb1 = ((p1 & 1) << (self.w - 1)) | (self.lb1 >> 1)
            self.t0 = ((b & 1) << 2) | ((self.t0 >> 1) & 7)
            self.t1 = ((p1 & 1) << 2) | ((self.t1 >> 1) & 7)
            self.t2 = ((p2 & 1) << 2) | ((self.t2 >> 1) & 7)
            self.mo = m_bit
            self.xo = xo_n
            self.yo = yo_n
            if self.gate_valid:
                self.vo = 1 if (x >= 3 and y >= 2) else 0
            else:
                self.vo = 1
        else:
            self.vo = 0


class Morph:
    """morph.v: erode stage -> dilate stage, border mask at the exit."""

    def __init__(self, w, buggy_variant=False):
        self.e1 = Morph3x3(w, True, gate_valid=buggy_variant)
        self.e2 = Morph3x3(w, False, gate_valid=buggy_variant)

    def step(self, v, b, x, y):
        # stage 1 samples its registered taps, then advances
        v1, b1, x1, y1 = (1, self.e1.mo, self.e1.xo, self.e1.yo) \
            if self.e1.vo else (0, 0, self.e1.xo, self.e1.yo)
        self.e1.step(v, b, x, y)
        # stage 2 consumes stage 1's *current* outputs
        (v2, b2, x2, y2) = (1, self.e2.mo, self.e2.xo, self.e2.yo) \
            if self.e2.vo else (0, 0, self.e2.xo, self.e2.yo)
        self.e2.step(v1, b1, x1, y1)
        in_region = (x2 >= 2) and (y2 >= 2)
        return (1 if (v2 and in_region) else 0,
                1 if (in_region and b2) else 0, x2, y2)


def run_morph(img, w, h, buggy=False, frame_blank=4):
    """Feed an image stream through morph.v; return (cyc, out_img, valid_img).

    frame_blank defaults to 4: the two-stage pipeline needs two idle cycles
    after the last pixel to flush the last row's last two columns. The real
    sensor supplies them as frame blanking; with frame_blank = 0 those two
    pixels simply never come out (harmless, but it would fail the "interior is
    fully covered" check below for the wrong reason).
    """
    m = Morph(w, buggy_variant=buggy)
    out = blank_image(w, h)
    val = blank_image(w, h)
    cyc = 0
    for (v, b, x, y, _fs) in raster(img, w, h, frame_blank):
        mv, mf, mx, my = m.step(v, b, x, y)
        if mv and 0 <= mx < w and 0 <= my < h:
            out[my][mx] = mf
            val[my][mx] = 1
        cyc += 1
    return cyc, out, val


def test_morph_open(env):
    print("\n-- 1. morph.v open operation vs reference --")
    w, h = env["CAM_IMG_W"], env["CAM_IMG_H"]
    img = blank_image(w, h)
    fill_rect(img, 40, 60, 100, 200)          # a "person"
    fill_rect(img, 200, 60, 260, 200)         # another one
    img[100][150] = 1                          # isolated 1-pixel speckle
    img[100][151] = 0

    _cyc, out, val = run_morph(img, w, h)
    ref = ref_open(img, w, h)

    x0, x1, y0, y1 = 2, w - 5, 2, h - 3
    bad = [(x, y) for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)
           if out[y][x] != ref[y][x]]
    check(not bad, "open matches the reference on x=%d..%d y=%d..%d"
          % (x0, x1, y0, y1), "first mismatch %s" % (bad[:3],))

    check(all(val[y][x] == 1 for y in range(y0, y1 + 1)
              for x in range(x0, x1 + 1)),
          "exactly the x>=2 / y>=2 interior is marked valid")

    check(out[100][150] == 0 and ref[100][150] == 0,
          "isolated 1-pixel speckle survives erosion? (must be 0)")
    check(img[100][150] == 1, "speckle really was in the input")

    body_sum = sum(out[y][x] for y in range(60, 200) for x in range(40, 100))
    check(body_sum == 60 * 140,
          "the solid body is untouched by the open (%d px)" % body_sum,
          "expected %d" % (60 * 140))

    # negative control: dilate-then-erode (a close) is NOT an open
    closed = ref_morph(ref_morph(img, w, h, False), w, h, True)
    diff = sum(1 for y in range(h) for x in range(w)
               if closed[y][x] != out[y][x])
    expect_fail(diff == 0, "close differs from open (so the order matters)")


def test_morph_stream(env):
    print("\n-- 2. morph.v stream alignment (why the stages keep v full width) --")
    w, h = env["CAM_IMG_W"], env["CAM_IMG_H"]
    img = blank_image(w, h)
    fill_rect(img, 295, 0, 305, h - 1)          # full-height 11-column bar

    def spans_of(image):
        s = set()
        for y in range(3, h - 3):
            lit = [x for x in range(2, w - 4) if image[y][x] == 1]
            s.add((lit[0], lit[-1]) if lit else (None, None))
        return s

    _c, out, val = run_morph(img, w, h)
    spans = spans_of(out)
    check(len(spans) == 1,
          "the bar's horizontal span is identical on every row (%s)"
          % (sorted(spans)[0],))
    check(sorted(spans)[0] == (295, 305),
          "open() of a solid bar is the bar itself (295..305)")

    _c2, out_bug, _v2 = run_morph(img, w, h, buggy=True)
    ref = ref_open(img, w, h)
    diff = sum(1 for y in range(3, h - 3) for x in range(2, w - 4)
               if out_bug[y][x] != ref[y][x])
    check(diff > 1000,
          "gating v at the stage border (only 373 shifts per row instead of "
          "376) wrecks the output: %d wrong pixels" % diff)

    # the exit mask really does throw away the 2-pixel border
    check(val[1][300] == 0 and val[2][300] == 1,
          "row 1 masked, row 2 valid (exit mask = x>=2 and y>=2)")
    check(val[2][1] == 0 and val[2][2] == 1,
          "column 1 masked, column 2 valid")


# ============================================================
# projection.v -- histogram + readout, cycle accurate
# ============================================================
class Projection:
    def __init__(self, env, register_ro_bin=False, register_ro_phase=False):
        self.col_n = env["PROJ_COL_BINS"]
        self.row_n = env["PROJ_ROW_BINS"]
        self.total = self.col_n + self.row_n
        self.col = [0] * self.col_n
        self.row = [0] * self.row_n
        self.ro_active = 0
        self.ro_cnt = 0
        self.ro_bin = 0
        self.ro_phase_reg = 0
        self.register_ro_bin = register_ro_bin
        self.register_ro_phase = register_ro_phase
        self.m_cnt_o = 0
        self.acc_cnt = 0
        self.curve = 0
        self.viz_tog = 0
        self.readout_cycles = 0
        self.cur_len = 0
        self.last_len = 0
        self.trace = []

    def _combo_bin(self):
        col_ph = self.ro_cnt < self.col_n
        return col_ph, (self.ro_cnt if col_ph else self.ro_cnt - self.col_n)

    def cycle(self, m_valid, m_fg, m_x, m_y, frame_start):
        # ---- combinational ----
        col_ph, nxt_bin = self._combo_bin()
        # rb is always the RAW bin number (0..153) so the row lookup is one
        # expression; obin is the per-phase bin (0..93 / 0..59) that the
        # downstream people_seg model and the trace see.
        if self.register_ro_bin:
            rb = self.ro_bin
            rcol_ph = (rb < self.col_n)
        else:
            rb, rcol_ph = self.ro_cnt, col_ph
        obin = rb if rcol_ph else (rb - self.col_n)
        rval = 0 if not self.ro_active else \
            (self.col[rb] if rcol_ph else self.row[rb - self.col_n])
        # ro_phase must be combinational too (see projection.v): a registered
        # copy lags ro_bin by one cycle, so at ro_cnt = COL_N the bin is already
        # row bin 0 while the phase still says "column".
        rphase = self.ro_phase_reg if self.register_ro_phase \
            else (0 if col_ph else 1)
        acc_en = m_valid and m_fg and not self.ro_active

        # ---- edge ----
        if frame_start:
            self.m_cnt_o = self.acc_cnt
            self.acc_cnt = 0
            self.ro_active = 1
            self.ro_cnt = 0
            self.cur_len = 0
            if not self.register_ro_bin:
                pass                    # combinational: already 0 this cycle
        elif self.ro_active:
            self.readout_cycles += 1
            self.cur_len += 1
            self.trace.append((1 if rcol_ph else 0, obin, rval))
            if rcol_ph:
                # new nibble enters at the TOP, existing content shifts down 4:
                # bin 0 (packed first) ends up in the lowest 4 bits, which is
                # the order the top-level transmitter shifts out.
                nibv = self.col[rb] >> 6
                self.curve = ((nibv << (4 * (self.col_n - 1)))
                              | (self.curve >> 4)) & ((1 << (self.col_n * 4)) - 1)
                self.col[rb] = 0
            else:
                self.row[rb - self.col_n] = 0
            if self.ro_cnt == self.total - 1:
                self.ro_active = 0
                self.viz_tog ^= 1
                self.last_len = self.cur_len
            else:
                self.ro_cnt += 1

        if acc_en:
            self.col[m_x >> 2] += 1
            self.row[m_y >> 2] += 1
        if (m_valid and m_fg) and not frame_start:
            self.acc_cnt += 1

        if self.register_ro_bin:
            self.ro_bin = nxt_bin if self.ro_active else 0
        if self.register_ro_phase:
            if frame_start:
                self.ro_phase_reg = 0
            elif self.ro_active:
                self.ro_phase_reg = 0 if col_ph else 1
        return (self.ro_active, rphase, obin, rval)

    def nibbles(self):
        return [(self.curve >> (4 * k)) & 0xF for k in range(self.col_n)]


def drive(proj, seg, img, mo, val, w, h, blanking=400, extra=1):
    """Run one real frame through projection (and optionally people_seg), then
    run `extra` blank frames so the readout of the real frame goes through.

    The blank frames must feed zeros, NOT the previous frame's foreground bits:
    the readout finished during their first 154 cycles, so anything the caller
    keeps feeding after that is accumulated into the freshly cleared histogram.
    """
    for (v, b, x, y, fs) in raster(img, w, h, frame_blank=blanking):
        st = proj.cycle(val[y][x] if v else 0, mo[y][x] if v else 0, x, y, fs)
        if seg is not None:
            seg.step(st[0], st[1], st[2], st[3], fs)
    for _ in range(extra):
        for (v, b, x, y, fs) in raster(blank_image(w, h), w, h,
                                      frame_blank=blanking):
            st = proj.cycle(0, 0, x, y, fs)
            if seg is not None:
                seg.step(st[0], st[1], st[2], st[3], fs)


def test_projection(env):
    print("\n-- 3. projection.v histogram, readout, curve packing --")
    w, h = env["CAM_IMG_W"], env["CAM_IMG_H"]
    img = blank_image(w, h)
    fill_rect(img, 40, 60, 100, 200)
    fill_rect(img, 200, 60, 260, 200)

    _cyc, mo, val = run_morph(img, w, h)
    p = Projection(env)
    drive(p, None, img, mo, val, w, h)
    tr = p.trace[-p.total:]

    ref_col = [0] * env["PROJ_COL_BINS"]
    ref_row = [0] * env["PROJ_ROW_BINS"]
    for y in range(h):
        for x in range(w):
            if val[y][x] and mo[y][x]:
                ref_col[x >> 2] += 1
                ref_row[y >> 2] += 1
    check(len(tr) == p.total, "the last readout produced %d entries" % p.total,
          "got %d" % len(tr))
    check(p.last_len == env["PROJ_COL_BINS"] + env["PROJ_ROW_BINS"],
          "readout is exactly %d cycles"
          % (env["PROJ_COL_BINS"] + env["PROJ_ROW_BINS"]),
          "got %d" % p.last_len)

    seen_col = [t[1] for t in tr[:env["PROJ_COL_BINS"]]]
    seen_row = [t[1] for t in tr[env["PROJ_COL_BINS"]:]]
    check(seen_col == list(range(env["PROJ_COL_BINS"])),
          "column phase reads bins 0..%d in order" % (env["PROJ_COL_BINS"] - 1))
    check(seen_row == list(range(env["PROJ_ROW_BINS"])),
          "row phase reads bins 0..%d in order" % (env["PROJ_ROW_BINS"] - 1))

    got_col = [t[2] for t in tr[:env["PROJ_COL_BINS"]]]
    got_row = [t[2] for t in tr[env["PROJ_COL_BINS"]:]]
    check(got_col == ref_col, "column histogram matches the reference",
          "first diff %s" % next((i for i, (a, b2) in
                                  enumerate(zip(got_col, ref_col)) if a != b2),
                                 None))
    check(got_row == ref_row, "row histogram matches the reference")

    check(sum(got_col) == p.m_cnt_o,
          "sum(column histogram) == morphed pixel count (%d)" % p.m_cnt_o)
    check(p.m_cnt_o == sum(1 for y in range(h) for x in range(w)
                           if val[y][x] and mo[y][x]),
          "morphed pixel count is the interior foreground count")

    nib = p.nibbles()
    check(nib == [min(15, c >> 6) for c in ref_col],
          "curve nibbles = min(15, bin >> 6) in bin order")
    check(all(0 <= n <= 15 for n in nib), "every curve nibble fits one hex digit")

    # negative control: packing lsb-first instead of msb-first mirrors the curve
    mir = 0
    for c in ref_col:
        mir = ((mir << 4) | (c >> 6)) & ((1 << (env["CURVE_N"] * 4)) - 1)
    got_mir = [(mir >> (4 * k)) & 0xF for k in range(env["CURVE_N"])]
    expect_fail(got_mir == nib,
                "lsb-first packing would mirror the curve "
                "(peak lands on the wrong side)")

    check(all(c == 0 for c in p.col) and all(r == 0 for r in p.row),
          "the histogram is cleared as it is read")

    # negative control: a registered ro_bin reads a stale bin on cycle 1
    p2 = Projection(env, register_ro_bin=True)
    drive(p2, None, img, mo, val, w, h)
    got_col2 = [t[2] for t in p2.trace[-p2.total:][:env["PROJ_COL_BINS"]]]
    expect_fail(got_col2 == ref_col,
                "registering ro_bin corrupts the readout (stale first bin)")


def test_projection_lossless(env):
    print("\n-- 4. readout vs accumulator contention --")
    ro_total = env["PROJ_COL_BINS"] + env["PROJ_ROW_BINS"]
    check(ro_total < env["CAM_IMG_W"],
          "readout (%d cycles) is shorter than one row (%d pixels)"
          % (ro_total, env["CAM_IMG_W"]))
    check(ro_total == 154, "readout is 154 cycles at these constants")

    # the skipped cycles land in rows that morph discards anyway
    w = env["CAM_IMG_W"]
    rows_covered = set()
    for i in range(ro_total):
        rows_covered.add(i // w)
    check(rows_covered == {0},
          "a readout starting at pixel 0 only touches row 0 (rows %s)"
          % sorted(rows_covered))


# ============================================================
# people_seg.v -- column projection -> head count
# ============================================================
class PeopleSeg:
    def __init__(self, env):
        self.col_th = env["SEG_COL_THRESH"]
        self.row_th = env["SEG_ROW_THRESH"]
        self.min_w = env["SEG_MIN_WIDTH"]
        self.min_gap = env["SEG_MIN_GAP"]
        self.reset()

    def reset(self):
        self.run_open = 0
        self.run_ok = 0
        self.run_start = 0
        self.gap = 0
        self.ppl = 0
        self.occ = 0
        self.cf_nz = 0
        self.cf = 0
        self.cl = 0
        self.cp = 0
        self.cpb = 0
        self.row_nz = 0
        self.rf = 0
        self.rl = 0

    def step(self, ro_valid, ro_phase, ro_bin, ro_val, frame_start):
        if frame_start:
            self.reset()
            return
        if not ro_valid:
            return
        col_ph = (ro_phase == 0)
        col_occ = ro_val > self.col_th
        row_occ = ro_val > self.row_th

        # column phase wrap-up: the first row-phase entry (bin 0)
        if (not col_ph) and (ro_bin == 0):
            if self.run_open and self.run_ok:
                self.ppl = min(15, self.ppl + 1)
            self.run_open = 0
            self.run_ok = 0
            self.gap = 0

        if col_ph:
            if col_occ:
                self.occ += 1
                self.cl = ro_bin
                if not self.cf_nz:
                    self.cf_nz = 1
                    self.cf = ro_bin
                if ro_val > self.cp:
                    self.cp = ro_val
                    self.cpb = ro_bin
                if not self.run_open:
                    self.run_open = 1
                    self.run_start = ro_bin
                    self.run_ok = 1 if self.min_w <= 1 else 0
                else:
                    self.gap = 0
                    if (ro_bin - self.run_start + 1) >= self.min_w:
                        self.run_ok = 1
            else:
                if self.run_open:
                    if self.gap == self.min_gap - 1:
                        if self.run_ok:
                            self.ppl = min(15, self.ppl + 1)
                        self.run_open = 0
                        self.run_ok = 0
                        self.gap = 0
                    else:
                        self.gap += 1
        else:
            if row_occ:
                if not self.row_nz:
                    self.rf = ro_bin
                self.row_nz = 1
                self.rl = ro_bin


def ref_segments(vals, thresh, min_w, min_gap):
    """Independent restatement of the run rule: a run ends once min_gap
    consecutive unoccupied bins have gone by."""
    occ = [1 if v > thresh else 0 for v in vals]
    segs = []
    i, n = 0, len(occ)
    while i < n:
        if not occ[i]:
            i += 1
            continue
        j = i
        last = i
        gap = 0
        while j < n and gap < min_gap:
            if occ[j]:
                last = j
                gap = 0
            else:
                gap += 1
            j += 1
        if (last - i + 1) >= min_w:
            segs.append((i, last))
        i = last + 1
    return segs


def synth_bins(rects, bins):
    v = [0] * bins
    for (b0, b1, lvl) in rects:
        for k in range(b0, b1 + 1):
            v[k] = lvl
    return v


def feed_seg(env, col_vals, row_vals):
    s = PeopleSeg(env)
    s.step(0, 0, 0, 0, 1)                       # frame_start resets the FSM
    for k, v in enumerate(col_vals):
        s.step(1, 0, k, v, 0)
    for k, v in enumerate(row_vals):
        s.step(1, 1, k, v, 0)
    return s


def test_people_seg(env):
    print("\n-- 5. people_seg.v column projection -> head count --")
    col_n = env["PROJ_COL_BINS"]
    row_n = env["PROJ_ROW_BINS"]
    lvl = 600                                    # a bin through a standing body
    low = 1000                                   # row bin level, > row thresh

    s = feed_seg(env, [0] * col_n, [0] * row_n)
    check(s.ppl == 0 and s.occ == 0 and s.row_nz == 0,
          "empty projection -> 0 people")

    col = synth_bins([(10, 25, lvl)], col_n)
    col[17] = 0                                  # a 1-bin dent
    s = feed_seg(env, col, synth_bins([(15, 50, low)], row_n))
    check(s.ppl == 1, "1 person with a 1-bin dent stays 1 person",
          "got %d" % s.ppl)
    check(s.occ == 15, "occupied bins = 15 (%d)" % s.occ)
    check(s.cf == 10 and s.cl == 25, "first/last occupied bin = 10 / 25")
    check(s.cp == lvl and s.cpb == 10, "peak value / peak bin")
    check(s.rf == 15 and s.rl == 50, "row extent 15..50")

    col2 = synth_bins([(10, 25, lvl), (50, 65, lvl)], col_n)
    s2 = feed_seg(env, col2, [0] * row_n)
    check(s2.ppl == 2, "two separate blobs -> 2 people", "got %d" % s2.ppl)
    check(s2.occ == 32, "occupied bins = 32 (%d)" % s2.occ)

    col3 = synth_bins([(5, 20, lvl), (40, 55, lvl), (75, 90, lvl)], col_n)
    s3 = feed_seg(env, col3, [0] * row_n)
    check(s3.ppl == 3, "three separate blobs -> 3 people", "got %d" % s3.ppl)

    hole2 = synth_bins([(10, 25, lvl)], col_n)
    hole2[17] = 0
    hole2[18] = 0
    s4 = feed_seg(env, hole2, [0] * row_n)
    check(s4.ppl == 2,
          "a 2-bin hole (= SEG_MIN_GAP) splits one blob into two",
          "got %d" % s4.ppl)
    expect_fail(s4.ppl == 1, "the min-gap filter is what does the splitting")

    narrow = [0] * col_n
    narrow[70] = lvl
    s5 = feed_seg(env, narrow, [0] * row_n)
    check(s5.ppl == 0, "a 1-bin blob is dropped by SEG_MIN_WIDTH",
          "got %d" % s5.ppl)
    check(s5.occ == 1, "but it still shows up as an occupied bin")

    edge = synth_bins([(88, col_n - 1, lvl)], col_n)
    s6 = feed_seg(env, edge, [0] * row_n)
    check(s6.ppl == 1,
          "a blob touching the right image edge is still counted",
          "got %d" % s6.ppl)

    edge0 = synth_bins([(0, 8, lvl)], col_n)
    s7 = feed_seg(env, edge0, [0] * row_n)
    check(s7.ppl == 1, "a blob touching the left image edge is counted")

    bad = 0
    for t in range(400):
        colr = [lvl if ((t * 37 + k * 91) % 101) > 60 else 0
                for k in range(col_n)]
        sr = feed_seg(env, colr, [0] * row_n)
        rr = ref_segments(colr, env["SEG_COL_THRESH"],
                          env["SEG_MIN_WIDTH"], env["SEG_MIN_GAP"])
        if len(rr) != sr.ppl:
            bad += 1
    check(bad == 0,
          "model agrees with the reference on 400 pseudo-random projections",
          "%d disagreements" % bad)


def test_end_to_end(env):
    print("\n-- 6. morph -> projection -> people_seg, end to end --")
    w, h = env["CAM_IMG_W"], env["CAM_IMG_H"]
    img = blank_image(w, h)
    fill_rect(img, 40, 60, 100, 200)          # person A
    fill_rect(img, 200, 60, 260, 200)         # person B

    _c, mo, val = run_morph(img, w, h)
    p = Projection(env)
    seg = PeopleSeg(env)
    drive(p, seg, img, mo, val, w, h)

    check(seg.ppl == 2, "two synthetic people -> P = 2", "got %d" % seg.ppl)
    check(seg.occ == 32, "S = 32 occupied column bins (%d)" % seg.occ)
    check(seg.rf == 15 and seg.rl == 50, "row extent 15..50")

    ref_col = [0] * env["PROJ_COL_BINS"]
    for y in range(h):
        for x in range(w):
            if val[y][x] and mo[y][x]:
                ref_col[x >> 2] += 1
    check(p.nibbles() == [min(15, c >> 6) for c in ref_col],
          "the curve carried by curve_bits matches the column projection")
    check(any(v > 0 for v in p.nibbles()), "the curve is not all zeros")

    # negative control: with a registered ro_phase the phase lags ro_bin by one
    # cycle, so the row-phase wrap-up never fires and row bin 0 gets eaten as a
    # column entry. Need a frame that exercises both: a blob that runs into the
    # right image edge (only the wrap-up can count it) and a blob up at row bin 0.
    img3 = blank_image(w, h)
    fill_rect(img3, 356, 60, 371, 200)        # reaches column bin 92
    fill_rect(img3, 100, 2, 160, 30)          # occupies row bin 0
    _c3, mo3, val3 = run_morph(img3, w, h)
    seg3 = PeopleSeg(env)
    drive(Projection(env), seg3, img3, mo3, val3, w, h)
    check(seg3.ppl == 2, "right-edge blob + top blob -> 2 people",
          "got %d" % seg3.ppl)
    check(seg3.rf == 0, "the top blob starts the row extent at bin 0",
          "got %d" % seg3.rf)

    seg4 = PeopleSeg(env)
    drive(Projection(env, register_ro_phase=True), seg4, img3, mo3, val3, w, h)
    expect_fail((seg4.ppl, seg4.occ, seg4.rf) == (seg3.ppl, seg3.occ, seg3.rf),
                "a registered ro_phase shifts the rows and leaks row bin 0 into "
                "the columns (ppl/occ/rf %d/%d/%d vs %d/%d/%d)"
                % (seg4.ppl, seg4.occ, seg4.rf, seg3.ppl, seg3.occ, seg3.rf))


# ============================================================
# status line
# ============================================================
ANCHORS = {0: "M", 1: "H", 2: "3", 6: "N", 7: "=", 15: "F", 16: "=",
           24: "M", 25: "=", 33: "P", 34: "=", 37: "S", 38: "=",
           43: "V", 44: "=", 139: "\r", 140: "\n"}

V_BEG = 45
V_END = 138


def hexc(n):
    return ord("0") + n if n < 10 else ord("A") + n - 10


def line_bytes(prt, tbl, curve, line_len):
    out = []
    for i in range(line_len):
        c = tbl.get(i, tbl.get("default"))
        if c is not None:
            out.append(c)
        elif 8 <= i <= 13:
            out.append(hexc((prt["cnt"] >> (4 * (5 - (i - 8)))) & 0xF))
        elif 17 <= i <= 22:
            out.append(hexc((prt["fg"] >> (4 * (5 - (i - 17)))) & 0xF))
        elif 26 <= i <= 31:
            out.append(hexc((prt["m"] >> (4 * (5 - (i - 26)))) & 0xF))
        elif i == 35:
            out.append(hexc(prt["ppl"] & 0xF))
        elif 39 <= i <= 41:
            out.append(hexc((prt["ocs"] >> (4 * (2 - (i - 39)))) & 0xF))
        elif V_BEG <= i <= V_END:
            out.append(hexc((curve >> (4 * (i - V_BEG))) & 0xF))
        elif i == 4:
            out.append(ord("K") if prt["ok"] else ord("F"))
        else:
            out.append(ord(" "))
    return out


def test_line(env):
    print("\n-- 7. top_vision_m3.v status line --")
    tbl = parse_fixed_char()
    for i, ch in ANCHORS.items():
        check(tbl.get(i) == ord(ch),
              "fixed_char[%d] == %r" % (i, ch),
              "got 0x%02X" % (tbl.get(i) if tbl.get(i) is not None else -1))
    check(env["M3_LINE_LEN"] == 141, "M3_LINE_LEN = 141")
    check(V_END - V_BEG + 1 == env["CURVE_N"],
          "the curve field is %d characters" % env["CURVE_N"])

    curve = 0
    for k in range(env["CURVE_N"]):
        curve |= (k % 16) << (4 * k)
    prt = {"cnt": env["CAM_FRAME_PIX"], "fg": 0x00ABCD, "m": 0x001234,
           "ppl": 2, "ocs": 128, "ok": True}
    b = line_bytes(prt, tbl, curve, env["M3_LINE_LEN"])
    text = bytes(b).decode("ascii")
    print("      line head: %r   (%d bytes total)" % (text[:48], len(b)))
    want = ("MH3 K N=016080 F=00ABCD M=001234 P=2 S=080 V="
            + ("0123456789ABCDEF" * 6)[:env["CURVE_N"]] + "\r\n")
    check(len(b) == 141, "line is exactly 141 bytes", "got %d" % len(b))
    check(text == want, "field layout / hex encoding correct",
          "got %r" % text[:60])

    bad = dict(prt)
    bad["ok"] = False
    check(line_bytes(bad, tbl, curve, 141)[4] == ord("F"),
          "self-test flag renders 'F' when cam_ok = 0")

    wrong = dict(prt)
    wrong["fg"] = 0x00ABDC
    expect_fail(bytes(line_bytes(wrong, tbl, curve, 141)) == bytes(b),
                "a nibble swap would change the line")


def test_constants(env):
    print("\n-- 8. M3 constants are mutually consistent --")
    shift = env["PROJ_BIN_SHIFT"]
    per = 1 << shift
    check(per * env["PROJ_COL_BINS"] >= env["CAM_IMG_W"],
          "%d bins x %d columns cover %d columns"
          % (env["PROJ_COL_BINS"], per, env["CAM_IMG_W"]))
    check((env["PROJ_COL_BINS"] - 1) * per < env["CAM_IMG_W"],
          "and the last bin is not all padding")
    check(per * env["PROJ_ROW_BINS"] >= env["CAM_IMG_H"],
          "%d row bins cover %d rows" % (env["PROJ_ROW_BINS"], env["CAM_IMG_H"]))
    check(env["CURVE_N"] == env["PROJ_COL_BINS"],
          "CURVE_N == PROJ_COL_BINS")
    check((1 << env["PROJ_CW"]) > per * env["CAM_IMG_H"],
          "column bins are %d bit wide (> %d)" % (env["PROJ_CW"], 4 * 240))
    check((1 << env["PROJ_RW"]) > per * env["CAM_IMG_W"],
          "row bins are %d bit wide (> %d)" % (env["PROJ_RW"], 4 * 376))
    top = per * env["CAM_IMG_H"]
    check((top >> env["CURVE_SHIFT"]) <= 15,
          "bin %d >> %d = %d fits a hex digit"
          % (top, env["CURVE_SHIFT"], top >> env["CURVE_SHIFT"]))
    check((top >> env["CURVE_SHIFT"]) == 15,
          "and it saturates exactly at 15 (full range used)")
    check(env["SEG_COL_THRESH"] > 0 and env["SEG_ROW_THRESH"] > 0,
          "both segmentation thresholds are non-zero")
    check(env["SEG_MIN_GAP"] >= 2,
          "SEG_MIN_GAP >= 2 so a single dent never splits a body")


def main():
    print("=" * 72)
    print("vision_sub M3 cycle/model checks")
    print("=" * 72)
    env = parse_defines()
    need = ("PROJ_BIN_SHIFT", "PROJ_COL_BINS", "PROJ_ROW_BINS", "PROJ_CW",
            "PROJ_RW", "SEG_COL_THRESH", "SEG_ROW_THRESH", "SEG_MIN_WIDTH",
            "SEG_MIN_GAP", "CURVE_SHIFT", "CURVE_N", "M3_LINE_LEN")
    missing = [k for k in need if k not in env]
    if missing:
        print("missing M3 defines in vision_def.v: %s" % missing)
        return 1
    print("parsed from vision_def.v: geometry=%dx%d  col_bins=%d row_bins=%d "
          "bin=4  col_thresh=%d min_width=%d min_gap=%d curve_shift=%d"
          % (env["CAM_IMG_W"], env["CAM_IMG_H"], env["PROJ_COL_BINS"],
             env["PROJ_ROW_BINS"], env["SEG_COL_THRESH"],
             env["SEG_MIN_WIDTH"], env["SEG_MIN_GAP"], env["CURVE_SHIFT"]))

    test_morph_open(env)
    test_morph_stream(env)
    test_projection(env)
    test_projection_lossless(env)
    test_people_seg(env)
    test_end_to_end(env)
    test_line(env)
    test_constants(env)

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
