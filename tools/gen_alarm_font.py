#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render the 24x24 glyphs for the emergency alarm headline.

Emits `src/user_source/hdl_source/alarm_font.vh`, which `alarm_overlay.v` pulls
in with a bare `include inside the module body. A naked `function` cannot be
compiled standalone, so the .vh is deliberately NOT registered in the .al -- the
same arrangement as marquee_font.vh.

Eight glyphs only, and that is the whole point of a separate table: the alarm
must not depend on the marquee's 15-cell slogan table, which is 15x wider and
belongs to a feature the alarm overrides.

  index 0 紧   1 急   2 疏   3 散   4 火   5 警   6 出   7 口

The words those glyphs spell are in alarm_overlay.v, not here -- this file is
glyphs, not messages. check_rtl_words() reads the 12-bit constants back out of
the RTL and proves they decode to the words listed in WORDS below.

Also writes previews into tools/preview/, rendered by a pixel-exact model of
alarm_overlay.v's addressing and output mux, so the screen can be eyeballed
before burning a build:
  alarm_screen_T1.png / _T2.png / _T3.png   full 640x480, bright flash phase
  alarm_sheet.png                           3 types x 2 flash phases, half size

check_alarm_font_transcription.py re-imports build_glyphs() and compares the
.vh on disk against a fresh render bit for bit, so this module is the golden
reference.
"""

from __future__ import annotations

import ast
import os
import re
import sys

from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_marquee_font import render_glyph   # noqa: E402  shared glyph pipeline

# ---------------------------------------------------------------------------
# Geometry. Mirrored as localparams in alarm_overlay.v and cross-checked here by
# reading the RTL text back, so the two cannot drift apart silently.
# ---------------------------------------------------------------------------
GLYPHS = "紧急疏散火警出口"

CELL = 24            # glyph box, rows and columns
HEAD_N = 4           # glyphs on the headline
PITCH = 64           # 2 * (CELL + 8): the x2 scale, and a power of two so the
                     # cell index and the column inside it are bit slices
SCALE = 2
INK = CELL * SCALE   # 48 real px of drawn glyph
GUTTER = (PITCH - INK) // 2   # 8 px of letter spacing each side

H_ACTIVE = 640
V_ACTIVE = 480
HEAD_X = (H_ACTIVE - HEAD_N * PITCH) // 2   # 192
HEAD_Y = 76

BORDER_IN = 12
BORDER_W = 6
BORDER_INNER = BORDER_IN + BORDER_W          # 18: first row/col inside the rule
ARROW_Y, ARROW_H, ARROW_PITCH = 196, 32, 64
ARROW_LOG_ROWS = ARROW_H // SCALE            # 16 logical rows
ARROW_LOG_PERIOD = ARROW_PITCH // SCALE      # 32 logical columns
TAPE_Y, TAPE_H, TAPE_PITCH = 388, 60, 64

# Words per alarm type, and the flash period in frames. Indexes are into GLYPHS.
WORDS = {
    1: ("火", "警", "疏", "散"),
    2: ("紧", "急", "疏", "散"),
    3: ("紧", "急", "出", "口"),
}


def pack_word(chars):
    """SPELLING -> alarm_overlay.v's W_msg constant.

    W_msg is four 3-bit fields with cell 0 in W_msg[11:9], so the hex value is
    not the digit sequence: 火警疏散 is 100_101_010_011 = 0x953, and writing it
    0x4523 -- which is what the eye expects -- silently renders 疏火火散. Derived
    here rather than typed, so the table and the RTL cannot agree with each other
    and both disagree with the screen.
    """
    value = 0
    for ch in chars:
        value = (value << 3) | GLYPHS.index(ch)
    return value


MSG_HEX = {atype: pack_word(word) for atype, word in WORDS.items()}
FLASH_FRAMES = {1: 8, 2: 16, 3: 32}

FONT_PATH = "C:/Windows/Fonts/simhei.ttf"
FONT_SIZE = 24       # same proven point size as the marquee: the largest whose
                     # natural ink bbox fits CELL without a LANCZOS downsample
THRESHOLD = 110

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VH_PATH = os.path.join(REPO, "src", "user_source", "hdl_source", "alarm_font.vh")
RTL_PATH = os.path.join(REPO, "src", "user_source", "hdl_source", "alarm_overlay.v")
PREVIEW_DIR = os.path.join(REPO, "tools", "preview")

FIRE_HI = (0xE0, 0x18, 0x18)
FIRE_LO = (0x88, 0x00, 0x00)
GEN_HI = (0xD0, 0x88, 0x00)
GEN_LO = (0x78, 0x4A, 0x00)
INK_WHITE = (0xFF, 0xFF, 0xFF)
INK_BLACK = (0x10, 0x10, 0x10)
TAPE_YEL = (0xF2, 0xC4, 0x00)


# ---------------------------------------------------------------------------
# Glyph rendering
# ---------------------------------------------------------------------------
def build_glyphs(**kwargs):
    """Render every alarm glyph. Returns a list of (rows, info) pairs."""
    return [render_glyph(ch, font_path=FONT_PATH, font_size=FONT_SIZE, cell=CELL,
                        threshold=THRESHOLD, **kwargs) for ch in GLYPHS]


def row_bits(row):
    """Pack one glyph row: bit CELL-1 is the leftmost column."""
    value = 0
    for x, on in enumerate(row):
        if on:
            value |= 1 << (CELL - 1 - x)
    return value


def table_from_cells(cells):
    return {i: {y: row_bits(r) for y, r in enumerate(rows)}
            for i, rows in enumerate(cells)}


# ---------------------------------------------------------------------------
# Self-checks
# ---------------------------------------------------------------------------
def check_font(cells, infos=None):
    """Return a list of human-readable failures. Empty list means pass."""
    fails = []
    centre = (CELL - 1) / 2.0
    by_bits = {}

    for idx, rows in enumerate(cells):
        ch = GLYPHS[idx]
        if infos is not None and infos[idx]["clipped"]:
            fails.append("cell %d %r: solid ink was clipped by the cell edge" % (idx, ch))
        xs = [x for y in range(CELL) for x in range(CELL) if rows[y][x]]
        ys = [y for y in range(CELL) for x in range(CELL) if rows[y][x]]
        if not xs:
            fails.append("cell %d %r: no ink at all" % (idx, ch))
            continue
        if min(xs) < 0 or max(xs) > CELL - 1 or min(ys) < 0 or max(ys) > CELL - 1:
            fails.append("cell %d %r: ink escapes the %dx%d cell" % (idx, ch, CELL, CELL))

        # Centring, to half a pixel: the headline is laid out by bit slices, so a
        # glyph that is off centre stays off centre in every copy of the build.
        for axis, lo, hi in (("x", min(xs), max(xs)), ("y", min(ys), max(ys))):
            got = (lo + hi) / 2.0
            if abs(got - centre) > 0.5:
                fails.append("cell %d %r: ink centre %s = %.1f, expected %.1f"
                             % (idx, ch, axis, got, centre))

        # Distinct characters must not collide -- 疏/散 and 出/口 are dense enough
        # that an over-aggressive threshold would blob them together.
        key = tuple(row_bits(r) for r in rows)
        if key in by_bits:
            fails.append("cells %d (%r) and %d (%r) rendered identically"
                         % (by_bits[key], GLYPHS[by_bits[key]], idx, ch))
        by_bits.setdefault(key, idx)

    return fails


def check_geometry():
    """The geometry must stay consistent with the bit-slice addressing in the RTL."""
    fails = []
    for name, val in (("PITCH", PITCH), ("ARROW_PITCH", ARROW_PITCH),
                      ("TAPE_PITCH", TAPE_PITCH)):
        if val & (val - 1):
            fails.append("%s %d is not a power of two; cell/col would need a divider"
                         % (name, val))
    if INK != CELL * SCALE:
        fails.append("INK %d is not CELL*SCALE" % INK)
    if GUTTER * 2 + INK != PITCH:
        fails.append("gutter %d + ink %d + gutter %d != pitch %d"
                     % (GUTTER, INK, GUTTER, PITCH))
    # head_cell = (x - HEAD_X)[7:6] only lines a cell boundary up with a glyph box
    # if the block starts on a pitch boundary.
    if HEAD_X % PITCH:
        fails.append("HEAD_X %d is not a multiple of PITCH %d; the cell slice would "
                     "cut glyphs in half" % (HEAD_X, PITCH))
    if HEAD_X * 2 + HEAD_N * PITCH != H_ACTIVE:
        fails.append("headline is not centred: HEAD_X %d, %d * %d = %d px wide in %d"
                     % (HEAD_X, HEAD_N, PITCH, HEAD_N * PITCH, H_ACTIVE))
    if HEAD_X < 0 or HEAD_X + HEAD_N * PITCH - 1 > H_ACTIVE - 1:
        fails.append("headline %d..%d runs off the %d px panel"
                     % (HEAD_X, HEAD_X + HEAD_N * PITCH - 1, H_ACTIVE))
    # The two >>1 slices need even origins, or row 0 of the box is not y % 2 == 0.
    for name, val in (("HEAD_Y", HEAD_Y), ("ARROW_Y", ARROW_Y), ("TAPE_Y", TAPE_Y)):
        if val % SCALE:
            fails.append("%s %d is not a multiple of the %dx scale; the row slice "
                         "would be misaligned" % (name, val, SCALE))
    if HEAD_Y + INK - 1 > ARROW_Y - 1:
        fails.append("headline rows %d..%d collide with the chevron row at %d"
                     % (HEAD_Y, HEAD_Y + INK - 1, ARROW_Y))
    if ARROW_Y + ARROW_H - 1 > TAPE_Y - 1:
        fails.append("chevron rows %d..%d collide with the tape at %d"
                     % (ARROW_Y, ARROW_Y + ARROW_H - 1, TAPE_Y))
    # Everything must stay inside the border rule, which is drawn on top anyway.
    if HEAD_Y < BORDER_INNER:
        fails.append("headline starts at y%d, inside the border rule y%d..%d"
                     % (HEAD_Y, BORDER_IN, BORDER_INNER - 1))
    if TAPE_Y + TAPE_H - 1 > V_ACTIVE - BORDER_IN - BORDER_W - 1:
        fails.append("tape ends at y%d, past the inner border edge y%d"
                     % (TAPE_Y + TAPE_H - 1, V_ACTIVE - BORDER_IN - BORDER_W - 1))
    if H_ACTIVE % ARROW_PITCH:
        fails.append("H_ACTIVE %d is not a multiple of ARROW_PITCH %d; the last "
                     "chevron would be cut" % (H_ACTIVE, ARROW_PITCH))
    if ARROW_LOG_PERIOD - ARROW_LOG_ROWS + 1 < 1:
        fails.append("chevron period %d logical is too narrow for %d rows of set-off"
                     % (ARROW_LOG_PERIOD, ARROW_LOG_ROWS))
    return fails


def check_rtl_words():
    """alarm_overlay.v's W_msg constants must decode to exactly WORDS.

    The glyph table and the message table live in different files, so nothing
    else notices if a nibble is edited into a character nobody asked for.
    """
    fails = []
    try:
        with open(RTL_PATH, "r", encoding="utf-8", errors="replace") as fh:
            src = fh.read()
    except IOError:
        return ["cannot read %s to check the message table" % RTL_PATH]

    body = re.search(r"W_msg\s*=\s*(.*?);\s*\n", src, re.S)
    if not body:
        return ["no W_msg assignment found in alarm_overlay.v"]
    hexes = [int(h, 16) for h in re.findall(r"12'h([0-9A-Fa-f]{1,4})\b", body.group(1))]
    if len(hexes) != len(MSG_HEX):
        return ["found %d W_msg constants (%s), expected %d"
                % (len(hexes), [hex(h) for h in hexes], len(MSG_HEX))]

    for (atype, want_chars), got in zip(sorted(WORDS.items()), hexes):
        idxs = [(got >> shift) & 0x7 for shift in (9, 6, 3, 0)]
        if any(i >= len(GLYPHS) for i in idxs):
            fails.append("type %d: 12'h%04X indexes a glyph beyond %d"
                         % (atype, got, len(GLYPHS) - 1))
            continue
        spelled = "".join(GLYPHS[i] for i in idxs)
        if got != MSG_HEX[atype]:
            fails.append("type %d: RTL has 12'h%04X, generator expects 12'h%04X"
                         % (atype, got, MSG_HEX[atype]))
        if spelled != "".join(want_chars):
            fails.append("type %d: 12'h%04X spells %r, expected %r"
                         % (atype, got, spelled, "".join(want_chars)))
    return fails


_LOCALPARAM_RE = re.compile(r"^\s*localparam\s+(?:\[[^\]]*\]\s+)?([A-Za-z_]\w*)\s*"
                            r"=\s*([^;]+);", re.M)
_SAFE_EXPR_RE = re.compile(r"[\w\s+\-*/()]+")


class _Unresolved(Exception):
    pass


class _Fraction(Exception):
    pass


def _rtl_localparam_values(src):
    """{name: int} for the arithmetic localparams of alarm_overlay.v.

    Verilog resolves these with integer arithmetic, so an expression that divides
    unevenly is a real difference in kind, not just a cosmetic one: Python would
    keep the fraction and TD would throw it away. That is reported as a failure
    rather than rounded, because a fractional HEAD_X would mean the headline is
    not actually centred on the pitch the bit slices assume.

    H_ACTIVE / V_ACTIVE are module parameters, so they never appear among the
    localparams; they are seeded from the constants top instantiates this module
    with, which is the only pairing that makes the derived numbers meaningful.
    """
    raw = {}
    for m in _LOCALPARAM_RE.finditer(src):
        raw.setdefault(m.group(1), m.group(2).strip())

    values = {"H_ACTIVE": H_ACTIVE, "V_ACTIVE": V_ACTIVE}
    pending = dict(raw)
    while pending:
        progressed = False
        for name in sorted(pending):
            expr = pending[name]
            if not _SAFE_EXPR_RE.fullmatch(expr):
                continue
            try:
                values[name] = _eval_int(expr, values)
            except (_Unresolved, _Fraction, SyntaxError, ZeroDivisionError,
                    ValueError, TypeError):
                continue
            del pending[name]
            progressed = True
        if not progressed:
            break
    for name, expr in raw.items():
        if not _SAFE_EXPR_RE.fullmatch(expr):
            values[name] = _verilog_literal(expr)
    return values, pending


def _eval_int(expr, env):
    node = ast.parse(expr, mode="eval").body

    def walk(n):
        if isinstance(n, ast.Constant) and isinstance(n.value, int):
            return n.value
        if isinstance(n, ast.Name):
            if n.id in env:
                return env[n.id]
            raise _Unresolved(n.id)
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, (ast.UAdd, ast.USub)):
            v = walk(n.operand)
            return v if isinstance(n.op, ast.UAdd) else -v
        if isinstance(n, ast.BinOp) and isinstance(n.op,
                                                  (ast.Add, ast.Sub, ast.Mult,
                                                   ast.Div, ast.FloorDiv)):
            a, b = walk(n.left), walk(n.right)
            if isinstance(n.op, ast.Add):
                return a + b
            if isinstance(n.op, ast.Sub):
                return a - b
            if isinstance(n.op, ast.Mult):
                return a * b
            if b == 0:
                raise ZeroDivisionError(expr)
            if a % b:
                raise _Fraction("%d/%d" % (a, b))
            return a // b
        raise SyntaxError("unsupported node in %r" % expr)

    return walk(node)


def _verilog_literal(text):
    """4'd15 -> 15, 24'hE01818 -> 0xE01818; anything else -> None."""
    m = re.fullmatch(r"\d*'[bBhHdDxX]([0-9A-Fa-f_]+)", text)
    if not m:
        return None
    body = m.group(1).replace("_", "")
    base = 2 if text[-2:].lower().endswith("b") else 16 if "x" in text.lower() else 10
    try:
        return int(body, base)
    except ValueError:
        return None


def check_rtl_geometry():
    """The localparams in alarm_overlay.v must still equal the numbers here."""
    fails = []
    try:
        with open(RTL_PATH, "r", encoding="utf-8", errors="replace") as fh:
            src = fh.read()
    except IOError:
        return ["cannot read %s to check geometry" % RTL_PATH]

    values, unresolved = _rtl_localparam_values(src)
    want = {
        "HEAD_N": HEAD_N, "GLYPH": CELL, "HEAD_PITCH": PITCH, "HEAD_INK": INK,
        "HEAD_X": HEAD_X, "HEAD_X_LAST": HEAD_X + HEAD_N * PITCH - 1,
        "HEAD_Y": HEAD_Y, "HEAD_Y_LAST": HEAD_Y + INK - 1, "HEAD_GUT": GUTTER,
        "BORDER_IN": BORDER_IN, "BORDER_W": BORDER_W,
        "ARROW_Y": ARROW_Y, "ARROW_H": ARROW_H, "ARROW_PITCH": ARROW_PITCH,
        "ARROW_Y_LAST": ARROW_Y + ARROW_H - 1,
        "TAPE_Y": TAPE_Y, "TAPE_H": TAPE_H, "TAPE_PITCH": TAPE_PITCH,
        "TAPE_Y_LAST": TAPE_Y + TAPE_H - 1,
    }
    for name, val in sorted(want.items()):
        if name in unresolved:
            fails.append("%s = %r cannot be evaluated from the RTL text"
                         % (name, unresolved[name]))
        elif name not in values:
            fails.append("alarm_overlay.v does not declare localparam %s" % name)
        elif values[name] is None:
            fails.append("%s is declared but its value is not an integer "
                         "expression this check can read" % name)
        elif values[name] != val:
            fails.append("%s = %d in the RTL, %d in the generator"
                         % (name, values[name], val))
    return fails


def check_rtl_blink():
    """alarm_overlay.v picks the flash period by choosing a wire of fc.

    The advertised 134 / 269 / 538 ms is pure bit arithmetic, which is exactly why
    it needs a check: re-pointing one ternary arm keeps the screen looking right in
    a still preview. Parsed positionally -- the chain is
    `type ? fc[a] : (type ? fc[b] : fc[c])`, so the order the fc[] bits appear in
    is the order the W_type tests appear in, and the one type nobody names takes
    the last bit.
    """
    fails = []
    try:
        with open(RTL_PATH, "r", encoding="utf-8", errors="replace") as fh:
            src = fh.read()
    except IOError:
        return ["cannot read %s to check the blink wire" % RTL_PATH]

    m = re.search(r"\bblink\s*=\s*([^;]+);", src)
    if not m:
        return ["no blink wire found in alarm_overlay.v"]
    expr = m.group(1)
    bits = [int(b) for b in re.findall(r"fc\[(\d+)\]", expr)]
    named = [int(t) for t in re.findall(r"W_type\s*==\s*2'd(\d+)", expr)]
    rest = [t for t in FLASH_FRAMES if t not in named]
    if len(bits) != len(FLASH_FRAMES) or len(rest) != 1:
        return ["blink selects %d fc bits for %d types, with %d types unnamed"
                % (len(bits), len(FLASH_FRAMES), len(rest))]

    for atype, bit in zip(named + rest, bits):
        want = FLASH_FRAMES[atype]
        if (1 << bit) != want:
            fails.append("type %d uses fc[%d] = %d frames, expected %d"
                         % (atype, bit, 1 << bit, want))
    return fails


def check_rtl_mux_order(src=None):
    """The preview model's arm order must BE the RTL's, not merely similar.

    screen_pixel() reproduces the hardware picture only while both test the same
    arms in the same priority order. Headline, chevrons and border are all
    INK_WHITE so their relative order is invisible; the one pair that can actually
    collide is the tape band against the border's VERTICAL arms, which run the
    full height straight through it. So the check is specifically ink_on before
    in_tape_y, plus ink_on still meaning exactly the three arms the model ORs.

    Takes the source text as an argument so the negative controls can hand it a
    mutated copy without touching the file on disk.
    """
    fails = []
    if src is None:
        try:
            with open(RTL_PATH, "r", encoding="utf-8", errors="replace") as fh:
                src = fh.read()
        except IOError:
            return ["cannot read %s to check the mux order" % RTL_PATH]

    m = re.search(r"assign\s+O_rgb\s*=\s*([^;]+);", src, re.S)
    if not m:
        return ["no O_rgb assign found in alarm_overlay.v"]
    chain = m.group(1)

    # A name used as a test is the one followed by '?'; the same identifier also
    # appears in its own `wire` definition, which must not count.
    pos = {}
    for term in ("ink_on", "in_tape_y"):
        hit = re.search(r"\b%s\b\s*\?" % term, chain)
        if hit is None:
            fails.append("O_rgb never tests %s, so the model has no matching arm"
                         % term)
        else:
            pos[term] = hit.start()
    if len(pos) == 2 and pos["ink_on"] > pos["in_tape_y"]:
        fails.append("O_rgb tests in_tape_y before ink_on; screen_pixel() puts "
                     "ink_on first, so the border would render striped on the "
                     "board and solid in the preview")

    m = re.search(r"wire\s+ink_on\s*=\s*([^;]+);", src)
    if not m:
        fails.append("no ink_on wire found in alarm_overlay.v")
    else:
        got = set(re.findall(r"\b(head_on|on_border|arrow_on)\b", m.group(1)))
        want = {"head_on", "on_border", "arrow_on"}
        if got != want:
            fails.append("ink_on ORs %s but screen_pixel() ORs %s"
                         % (sorted(got), sorted(want)))
    return fails


def check_negative_controls():
    """The checks above must have teeth. Returns a list of failures."""
    fails = []
    good, good_info = zip(*build_glyphs())
    good, good_info = list(good), list(good_info)

    if check_font(good, good_info):
        fails.append("baseline font fails its own checks: %s" % check_font(good, good_info))
    if check_geometry():
        fails.append("baseline geometry fails its own checks: %s" % check_geometry())
    if check_rtl_words():
        fails.append("baseline message table fails: %s" % check_rtl_words())
    if check_rtl_geometry():
        fails.append("baseline geometry cross-check fails: %s" % check_rtl_geometry())
    if check_rtl_blink():
        fails.append("baseline blink wires fail: %s" % check_rtl_blink())

    # pack_word is the one place the 3-bit field layout is written down. The naive
    # 4-bit packing is what the eye expects and what the RTL briefly had.
    if pack_word(WORDS[1]) == 0x4523:
        fails.append("pack_word is doing the naive 4-bit packing again")
    if pack_word(WORDS[1]) != 0x953:
        fails.append("pack_word known answer: got 0x%03X, expected 0x953"
                     % pack_word(WORDS[1]))

    biased, biased_info = zip(*build_glyphs(center_bias=1))
    if not check_font(list(biased), list(biased_info)):
        fails.append("negative control: center_bias=1 was NOT caught")

    blanked = [list(r) for r in good]
    blanked[2] = [[0] * CELL for _ in range(CELL)]
    if not check_font(blanked, good_info):
        fails.append("negative control: blanked cell was NOT caught")

    collided = [list(r) for r in good]
    collided[6] = [list(r) for r in collided[7]]
    if not check_font(collided, good_info):
        fails.append("negative control: char collision was NOT caught")

    # check_geometry reads its constants out of the module globals at call time.
    for name, bad in (("PITCH", 60), ("HEAD_X", HEAD_X + 2), ("ARROW_Y", ARROW_Y + 1),
                      ("TAPE_Y", TAPE_Y + 40)):
        good_val = globals()[name]
        globals()[name] = bad
        caught = bool(check_geometry())
        globals()[name] = good_val
        if not caught:
            fails.append("negative control: %s=%d was NOT caught" % (name, bad))

    for shift in (1, 0x10):
        bumped = dict(MSG_HEX)
        bumped[2] = bumped[2] + shift
        good_val = globals()["MSG_HEX"]
        globals()["MSG_HEX"] = bumped
        caught = bool(check_rtl_words())
        globals()["MSG_HEX"] = good_val
        if not caught:
            fails.append("negative control: MSG_HEX+0x%X was NOT caught" % shift)

    for frames in ({1: 8, 2: 8, 3: 32}, {1: 16, 2: 16, 3: 32}):
        good_flash = globals()["FLASH_FRAMES"]
        globals()["FLASH_FRAMES"] = frames
        caught = bool(check_rtl_blink())
        globals()["FLASH_FRAMES"] = good_flash
        if not caught:
            fails.append("negative control: FLASH_FRAMES=%s was NOT caught" % frames)

    # The mux-order controls mutate an in-memory copy of the RTL. A check with
    # broken teeth must never be able to damage the file it is auditing.
    if check_rtl_mux_order():
        fails.append("baseline mux order fails: %s" % check_rtl_mux_order())
    try:
        with open(RTL_PATH, "r", encoding="utf-8", errors="replace") as fh:
            rtl = fh.read()
    except IOError:
        rtl = None

    if rtl is not None:
        swapped, n = re.subn(r"\b(ink_on|in_tape_y)\b(?=\s*\?)",
                             lambda m: "in_tape_y" if m.group(1) == "ink_on"
                             else "ink_on", rtl)
        if n != 2:
            fails.append("mux-order control located %d arms, expected 2, so it "
                         "proved nothing" % n)
        elif not check_rtl_mux_order(swapped):
            fails.append("negative control: swapping the ink/tape mux arms was "
                         "NOT caught")

        dropped = re.sub(r"wire\s+ink_on\s*=\s*[^;]+;",
                         "wire ink_on = head_on || on_border;", rtl, count=1)
        if dropped == rtl:
            fails.append("ink_on control did not apply, so it proved nothing")
        elif not check_rtl_mux_order(dropped):
            fails.append("negative control: ink_on dropping arrow_on was NOT caught")

    return fails


# ---------------------------------------------------------------------------
# Verilog emission
# ---------------------------------------------------------------------------
def emit_vh(cells):
    out = [
        "// Generated by tools/gen_alarm_font.py -- DO NOT EDIT BY HAND.",
        "// Re-run the generator instead; tools/check_alarm_font_transcription.py",
        "// compares this file against a fresh render, bit for bit.",
        "//",
        "// Included from inside alarm_overlay.v's module body, so this file holds",
        "// a bare function and must NOT be added to the .al file list.",
        "//",
        "// Glyphs    : %s (index order %s)" % (GLYPHS, " ".join(
            "%d=%s" % (i, c) for i, c in enumerate(GLYPHS))),
        "// Font      : %s @ %d px, threshold %d" % (FONT_PATH, FONT_SIZE, THRESHOLD),
        "// Cells     : %d x %d px, displayed x%d on a %d px pitch (%d px gutter"
        " each side)" % (CELL, CELL, SCALE, PITCH, GUTTER),
        "// Bit order : bit %d is the leftmost column of the row." % (CELL - 1),
        "// The words these glyphs spell are in alarm_overlay.v, not here; see",
        "// gen_alarm_font.py:check_rtl_words for the cross-check.",
        "// Blank rows are folded into the default arm to keep the file short.",
        "// The argument is cell_idx, not cell: `cell` is a Verilog-2001 reserved",
        "// word and TD rejects it with HDL-8007.",
        "",
        "function [23:0] alarm_glyph;",
        "    input [2:0] cell_idx;",
        "    input [4:0] row;",
        "    begin",
        "        case (cell_idx)",
    ]
    for idx, rows in enumerate(cells):
        out.append("            3'd%d: case (row)   // %s" % (idx, GLYPHS[idx]))
        for y in range(CELL):
            bits = row_bits(rows[y])
            if bits == 0:
                continue
            out.append("                5'd%d: alarm_glyph = 24'h%06X;" % (y, bits))
        out.append("                default: alarm_glyph = 24'h000000;")
        out.append("            endcase")
    out.append("            default: alarm_glyph = 24'h000000;")
    out.append("        endcase")
    out.append("    end")
    out.append("endfunction")
    out.append("")
    return "\n".join(out)


_ROW_RE = re.compile(r"5'd(\d+):\s*alarm_glyph\s*=\s*24'h([0-9A-Fa-f]{1,6});")
_CELL_RE = re.compile(r"3'd(\d+):\s*case \(row\)")


def parse_vh(text):
    """Read a .vh back into {cell: {row: bits}}. Used by the transcription check."""
    table = {}
    cell = None
    for line in text.splitlines():
        line = line.strip()
        m = _CELL_RE.match(line)
        if m:
            cell = int(m.group(1))
            table[cell] = {}
            continue
        m = _ROW_RE.match(line)
        if m and cell is not None:
            table[cell][int(m.group(1))] = int(m.group(2), 16)
    return table


def tables_equal(parsed, expected):
    """Compare, treating rows the emitter folded into `default` as zero."""
    diffs = []
    if set(parsed) != set(expected):
        return ["cell set differs: parsed %s vs expected %s"
                % (sorted(parsed), sorted(expected))]
    for cell in sorted(expected):
        for row in range(CELL):
            got = parsed[cell].get(row, 0)
            want = expected[cell].get(row, 0)
            if got != want:
                diffs.append("cell %d row %d: .vh has 24'h%06X, generator has 24'h%06X"
                             % (cell, row, got, want))
    return diffs


# ---------------------------------------------------------------------------
# Preview: a pixel-exact model of alarm_overlay.v's addressing and output mux
# ---------------------------------------------------------------------------
def blink_bit(fc, atype):
    """~fc[k] with k = log2(FLASH_FRAMES), the RTL's one-wire 'modulo'."""
    k = FLASH_FRAMES[atype].bit_length() - 1
    return ((fc >> k) & 1) ^ 1


def screen_pixel(x, y, atype, fc, packed):
    """One pixel of the alarm layer with the signage underneath replaced.

    The three ink terms are computed separately and ORed before the tape is
    consulted, because that is the RTL's mux order: `ink_on` ahead of `in_tape_y`.
    Testing the tape first looks harmless -- the two bands do not touch -- except
    that the border's VERTICAL arms run the full height and cross the tape, so the
    model would paint stripes over the frame and disagree with the hardware on
    2 x 6 x 60 = 720 pixels. check_rtl_mux_order() pins the orders together.
    """
    field = ((GEN_HI if blink_bit(fc, atype) else GEN_LO) if atype == 3
             else (FIRE_HI if blink_bit(fc, atype) else FIRE_LO))

    # headline: cell/row/col are the RTL's bit slices, not a division
    head_on = False
    if HEAD_Y <= y <= HEAD_Y + INK - 1 and HEAD_X <= x <= HEAD_X + HEAD_N * PITCH - 1:
        col = (x - HEAD_X) % PITCH
        if GUTTER <= col <= GUTTER + INK - 1:
            cell = (x - HEAD_X) // PITCH
            nibble = (MSG_HEX[atype] >> (3 * (3 - cell))) & 0x7
            bits = packed[nibble][(y - HEAD_Y) // SCALE]
            head_on = (bits >> (CELL - 1 - (col - GUTTER) // SCALE)) & 1 == 1

    # marching chevrons, the RTL's arrow_row / arrow_rise / arrow_set / arrow_q
    arrow_on = False
    if ARROW_Y <= y <= ARROW_Y + ARROW_H - 1:
        arrow_row = (y - ARROW_Y) // SCALE
        arrow_rise = arrow_row if arrow_row <= 7 else ARROW_LOG_ROWS - 1 - arrow_row
        arrow_set = ARROW_LOG_ROWS - 1 - arrow_rise
        arrow_q = ((x // SCALE) - (fc % ARROW_LOG_PERIOD) + arrow_set) \
            % ARROW_LOG_PERIOD
        arrow_on = arrow_q >= ARROW_LOG_PERIOD - 8

    # inset border, always white so the frame never disappears with the flash
    in_rect = (BORDER_IN <= x <= H_ACTIVE - BORDER_IN - 1 and
               BORDER_IN <= y <= V_ACTIVE - BORDER_IN - 1)
    near = (x <= BORDER_INNER - 1 or x >= H_ACTIVE - BORDER_IN - BORDER_W or
            y <= BORDER_INNER - 1 or y >= V_ACTIVE - BORDER_IN - BORDER_W)
    on_border = in_rect and near

    if head_on or on_border or arrow_on:
        return INK_WHITE

    # hazard tape: yellow when bit 5 of (x + y - tape_phase) is set
    if TAPE_Y <= y <= TAPE_Y + TAPE_H - 1:
        if (x + y - (2 * fc) % TAPE_PITCH) % TAPE_PITCH >= TAPE_PITCH // 2:
            return TAPE_YEL
        return INK_BLACK

    return field


def render_screen(atype, fc, packed, scale=1):
    """scale=2 samples every other pixel, which for a NEAREST 2x downscale is
    the same picture for half the Python calls."""
    w, h = H_ACTIVE // scale, V_ACTIVE // scale
    img = Image.new("RGB", (w, h))
    px = img.load()
    for j in range(h):
        y = j * scale
        for i in range(w):
            px[i, j] = screen_pixel(i * scale, y, atype, fc, packed)
    return img


def write_previews(packed):
    os.makedirs(PREVIEW_DIR, exist_ok=True)
    paths = []

    for atype in sorted(WORDS):
        img = render_screen(atype, 0, packed)
        d = ImageDraw.Draw(img)
        d.text((4, 4), "T%d %s  fc=0" % (atype, "".join(WORDS[atype])),
               fill=INK_WHITE)
        p = os.path.join(PREVIEW_DIR, "alarm_screen_T%d.png" % atype)
        img.save(p)
        paths.append(p)

    tile_w, tile_h = H_ACTIVE // 2, V_ACTIVE // 2
    sheet = Image.new("RGB", (tile_w * 2, tile_h * 3 + 24), (16, 16, 24))
    d = ImageDraw.Draw(sheet)
    d.text((6, 2), "alarm_overlay preview -- half size, one type per row, "
                   "left bright / right dark flash phase", fill=(200, 220, 255))
    for row, atype in enumerate(sorted(WORDS)):
        for col, fc in enumerate((0, FLASH_FRAMES[atype] // 2)):
            tile = render_screen(atype, fc, packed, scale=2)
            td = ImageDraw.Draw(tile)
            td.text((2, 1), "T%d %s fc=%d" % (atype, "".join(WORDS[atype]), fc),
                    fill=INK_BLACK)
            sheet.paste(tile, (col * tile_w, 20 + row * (tile_h + 2)))
    p = os.path.join(PREVIEW_DIR, "alarm_sheet.png")
    sheet.save(p)
    paths.append(p)
    return paths


# ---------------------------------------------------------------------------
def main():
    failures = 0

    print("=== rendering %d glyphs: %s" % (len(GLYPHS), GLYPHS))
    rendered = build_glyphs()
    cells = [rows for rows, _ in rendered]
    infos = [info for _, info in rendered]
    for idx, (ch, info) in enumerate(zip(GLYPHS, infos)):
        flags = ""
        if info["resampled"]:
            flags += "   [RESAMPLED]"
        if info["clipped"]:
            flags += "   [CLIPPED]"
        print("  cell %2d %s  ink %2dx%-2d  pixels %4d%s"
              % (idx, ch, info["ink"][0], info["ink"][1],
                 sum(sum(r) for r in cells[idx]), flags))

    if any(i["resampled"] for i in infos):
        msg = ("at least one glyph was downsampled to fit the %d px cell; resampled "
               "strokes are what makes dense Chinese glyphs muddy" % CELL)
        if "--allow-resample" in sys.argv:
            print("  WARN: " + msg)
        else:
            print("  FAIL: " + msg)
            print("        lower FONT_SIZE, or re-run with --allow-resample to override")
            failures += 1
    if any(i["clipped"] for i in infos):
        print("  FAIL: at least one glyph's solid ink was clipped by the cell edge")
        failures += 1

    packed = table_from_cells(cells)

    print("=== geometry")
    print("  headline = x %d..%d, y %d..%d (%d glyphs on a %d px pitch, %d px gutter)"
          % (HEAD_X, HEAD_X + HEAD_N * PITCH - 1, HEAD_Y, HEAD_Y + INK - 1,
             HEAD_N, PITCH, GUTTER))
    print("  chevrons = y %d..%d, tape = y %d..%d, border = %d px at %d px inset"
          % (ARROW_Y, ARROW_Y + ARROW_H - 1, TAPE_Y, TAPE_Y + TAPE_H - 1,
             BORDER_W, BORDER_IN))
    for f in check_geometry():
        print("  FAIL: " + f)
        failures += 1

    print("=== glyph self-checks")
    glyph_fails = check_font(cells, infos)
    for f in glyph_fails:
        print("  FAIL: " + f)
    failures += len(glyph_fails)
    if not glyph_fails:
        print("  pass: ink non-empty, inside the cell, centred, no collisions")

    print("=== RTL cross-checks")
    for label, check in (("message table", check_rtl_words),
                         ("geometry localparams", check_rtl_geometry),
                         ("blink rate wires", check_rtl_blink),
                         ("mux arm order", check_rtl_mux_order)):
        found = check()
        for f in found:
            print("  FAIL: " + f)
        failures += len(found)
        if not found:
            print("  pass: alarm_overlay.v agrees with this file (%s)" % label)

    print("=== negative controls")
    nc_fails = check_negative_controls()
    for f in nc_fails:
        print("  FAIL: " + f)
    failures += len(nc_fails)
    if not nc_fails:
        print("  pass: center_bias / blank cell / collision / PITCH / HEAD_X / "
              "ARROW_Y / TAPE_Y / MSG_HEX / mux arm swap / ink_on term all caught")

    if failures:
        print("\n%d failure(s); not writing the .vh" % failures)
        return 1

    text = emit_vh(cells)
    os.makedirs(os.path.dirname(VH_PATH), exist_ok=True)
    with open(VH_PATH, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    print("=== wrote %s (%d bytes, %d lines)"
          % (VH_PATH, len(text), text.count("\n")))

    diffs = tables_equal(parse_vh(text), packed)
    if diffs:
        print("=== FAIL: round-trip through the .vh lost data")
        for d in diffs[:10]:
            print("  " + d)
        return 1
    print("=== round-trip parse of the .vh matches the render bit for bit")

    if "--no-preview" not in sys.argv:
        print("=== previews")
        for p in write_previews(packed):
            print("  " + p)

    print("\nOK -- inspect the previews before building.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
