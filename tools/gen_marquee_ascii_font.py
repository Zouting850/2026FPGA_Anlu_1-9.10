#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render the ASCII glyphs for the PC-text scrolling subtitle.

Emits `src/user_source/hdl_source/marquee_ascii_font.vh`, which
`marquee_overlay.v` pulls in with a bare `include inside the module body. A
naked `function` cannot be compiled standalone, so the .vh is deliberately NOT
registered in the .al -- exactly like the slogan's marquee_font.vh.

Why 12x12 scaled x2 (the original plan said 8x8 x3; a probe disproved it):
  * The PC sends only character CODES; the FPGA owns this table and does all
    rendering / scaling / scrolling, so the contest rule that the host must not
    pre-render glyphs is satisfied (same argument as the serial screen being a
    "human input peripheral" -- it sends codes, like a keyboard).
  * 8x8 was the plan's first choice for slice economy, but tools/_probe_ascii_
    native.py showed EVERY system face collides at 8x8 native (I==l==1==!,
    ,==.==`). 8 px is below the legibility floor for the full ASCII set, and a
    clean 8x8 only exists as a hand-designed bitmap font, which cannot be
    reproduced reliably here. 12x12 is the SMALLEST square box that divides the
    24 px cell evenly AND renders all 96 glyphs distinctly with a uniform font
    size and no resampling (Verdana/Tahoma/Consolas/Courier bold all pass).
  * A square box scaled x2 in BOTH axes keeps the aspect correct (the plan's
    "12x24, horizontal x2 only" would have stretched caps), and the addressing
    is free bit slices -- gcol[4:1] and row[4:1] -- with no divide-by-N LUTs at
    all, which is simpler and shorter than 8x8's gcol/3 + row/3.
  * 12x12 over 0x20..0x7E is ~13.8 Kbit of combinational ROM. A full 24x24
    ASCII ROM (~55 Kbit) would not fit the free-slice budget, and BRAM would
    force a 1-px pipeline that breaks the "disabled == bit-identical" retreat.

Bit order: bit 11 of each row word is the LEFTMOST of the 12 columns, matching
osd_overlay's font8x8 convention (bit 7 = leftmost). Row 0 is the top.

Also writes previews into tools/preview/ so the glyphs can be eyeballed before
burning a build:
  marquee_ascii_sheet.png           all 96 glyphs at 4x, labelled char + hex
  marquee_ascii_band_preview_*.png  a sample string scrolling in the real band

check_marquee_ascii_transcription.py re-imports build_font() and compares it
against the .vh on disk bit for bit, so this module is the golden reference.
"""

from __future__ import annotations

import os
import re
import sys

from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------------------
# Geometry. Mirrored as localparams / addressing in marquee_overlay.v's PC arm
# and cross-checked by tools/check_marquee_ascii_transcription.py.
# ---------------------------------------------------------------------------
CW = 12              # glyph box, columns
CH = 12             # glyph box, rows
SCALE = 2           # marquee_overlay scales 12x12 by 2 -> the 24x24 cell
CELL = CW * SCALE   # 24, must equal the slogan CELL
PITCH = 32          # advance per cell; power of two so cell/col are bit slices
GUTTER = (PITCH - CELL) // 2   # 4 px of letter spacing each side

H_ACTIVE = 640
V_ACTIVE = 480
POS_BITS = 11       # width of marq_pos / of x_pos + marq_pos

BAND_H = 32
BAND_Y = (V_ACTIVE - BAND_H) // 2   # 224
BAND_TEXT_Y = BAND_Y + 4            # 228

# Printable ASCII, 0x20..0x7E (96 codes). 0x7F (DEL) and everything below 0x20
# are excluded; the .vh's outer default arm renders any of those as blank, so a
# stray control byte on the wire can never light a glyph.
CHARSET = [chr(c) for c in range(0x20, 0x7F)]

# Display cap. cell_idx = u[9:5] addresses 0..31, but the 11-bit borrow math
# s = x_pos + marq_pos peaks at (H_ACTIVE-1) + (H_ACTIVE + n_cells*PITCH - 1);
# keeping that <= 2**POS_BITS-1 = 2047 forces n_cells <= 24 (the original plan's
# 32 would peak at 2302 and overflow). char_buf stays 32 entries deep so the
# 5-bit cell_idx is always an in-range index; entries 24..31 are never selected
# because in_region (u < n_cells<<5) is false out there.
MAX_PC_CELLS = 24
CHAR_BUF_ENTRIES = 32

# Font preference, most legible-at-small-size first. autofit walks this list and
# takes the first face whose largest native-fit size (every glyph's solid ink
# already inside the box, no resampling) is also COLLISION-FREE. Verdana Bold is
# the gold standard for small-size screen text; the bolds keep stroke weight when
# the banner is viewed from a distance. FONT_PATH/FONT_SIZE are filled by autofit
# and recorded in the .vh header; the transcription check re-derives the identical
# table from the same system fonts.
FONT_CANDIDATES = [
    "C:/Windows/Fonts/verdanab.ttf",
    "C:/Windows/Fonts/tahomabd.ttf",
    "C:/Windows/Fonts/consolab.ttf",
    "C:/Windows/Fonts/courbd.ttf",
    "C:/Windows/Fonts/verdana.ttf",
    "C:/Windows/Fonts/tahoma.ttf",
    "C:/Windows/Fonts/consola.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
    "C:/Windows/Fonts/arial.ttf",
]
FONT_PATH = None     # set by autofit (or pass --font=PATH)
FONT_SIZE = None     # set by autofit (or pass --size=N)
THRESHOLD = 128      # greyscale cut, 0..255

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VH_PATH = os.path.join(REPO, "src", "user_source", "hdl_source", "marquee_ascii_font.vh")
PREVIEW_DIR = os.path.join(REPO, "tools", "preview")

TEXT_RGB = (0xFF, 0xE8, 0x78)   # same warm yellow as the slogan / osd text
EDGE_RGB = (0x60, 0xD8, 0xFF)   # same cyan as the band edge
DIM_SHIFT = 2                   # band background keeps 1/4 of the picture

SAMPLE_STRINGS = ["HELLO FPGA 2026", "ANLOGIC EG4S20 PC TEXT"]


# ---------------------------------------------------------------------------
# Glyph rendering
# ---------------------------------------------------------------------------
def render_glyph(ch, font_path, font_size, cw=CW, ch_h=CH, threshold=THRESHOLD,
                 center_bias=0, allow_resample=False):
    """Return (rows, info): rows is ch_h x cw of 0/1, ordered left to right.

    Drawn at font_size, cropped to the SOLID ink extent, then centred in the box
    -- exactly the slogan generator's discipline. A uniform font size across the
    whole charset is what keeps cap-height consistent; per-glyph resize-to-fill
    would shrink wide glyphs (W) shorter than narrow ones (H), which looks wrong.

    allow_resample defaults to False: autofit only accepts a size where every
    glyph already fits the box natively, so resampling never happens for the
    chosen face. It can be forced True for diagnostics.
    """
    font = ImageFont.truetype(font_path, font_size)
    pad = 4 * font_size + 8
    big = Image.new("L", (pad, pad), 0)
    ImageDraw.Draw(big).text((pad // 4, pad // 4), ch, font=font, fill=255)

    cut = lambda img: img.point(lambda p: 255 if p > threshold else 0)  # noqa: E731

    solid = cut(big).getbbox()
    if solid is None:
        # Genuinely blank (only the space character should reach here).
        return ([[0] * cw for _ in range(ch_h)],
                {"ink": (0, 0), "resampled": False, "clipped": False, "fits": True})

    crop = big.crop(solid)
    resampled = False
    if crop.width > cw or crop.height > ch_h:
        if not allow_resample:
            # Does not fit natively at this size; report so autofit backs the
            # size down. ink holds the OVERSIZE extent that did not fit.
            return ([[0] * cw for _ in range(ch_h)],
                    {"ink": (crop.width, crop.height), "resampled": False,
                     "clipped": False, "fits": False})
        scale = min(cw / crop.width, ch_h / crop.height)
        crop = crop.resize((max(1, round(crop.width * scale)),
                            max(1, round(crop.height * scale))), Image.LANCZOS)
        resampled = True

    final = cut(crop)
    fb = final.getbbox()
    if fb is not None:
        final = final.crop(fb)

    out = Image.new("L", (cw, ch_h), 0)
    off_x = (cw - final.width) // 2 + center_bias
    off_y = (ch_h - final.height) // 2 + center_bias
    out.paste(final, (off_x, off_y))

    clipped = (off_x < 0 or off_y < 0
               or off_x + final.width > cw or off_y + final.height > ch_h)

    rows = [[1 if out.getpixel((x, y)) else 0 for x in range(cw)] for y in range(ch_h)]
    return rows, {"ink": (final.width, final.height), "resampled": resampled,
                  "clipped": clipped, "fits": True}


def _collisions(glyphs):
    """List of "A==B" strings for distinct chars that rendered identically."""
    seen, coll = {}, []
    for ch in CHARSET:
        rows, _ = glyphs[ch]
        if ch == " " or not any(any(r) for r in rows):
            continue
        key = tuple(row_bits(r) for r in rows)
        if key in seen:
            coll.append("%r==%r" % (seen[key], ch))
        else:
            seen[key] = ch
    return coll


def autofit(candidates=FONT_CANDIDATES, sizes=range(40, 5, -1), threshold=THRESHOLD):
    """Pick (font_path, font_size, cap_height).

    For each face, find the LARGEST size where every glyph fits the box natively
    (no resample, no clip), then require that size to be collision-free. The
    first face that satisfies both wins. Larger sizes carry more pixels and so
    fewer collisions, which is why only the largest native-fit size is tested --
    backing the size down would never rescue a colliding face.
    """
    for path in candidates:
        if not os.path.exists(path):
            continue
        for size in sizes:
            glyphs = {ch: render_glyph(ch, path, size, threshold=threshold,
                                       allow_resample=False) for ch in CHARSET}
            if not all(glyphs[ch][1]["fits"] for ch in CHARSET):
                continue                      # some glyph still overflows the box
            if any(glyphs[ch][1]["clipped"] for ch in CHARSET):
                continue
            if _collisions(glyphs):
                break                         # this face collides; try the next
            heights = []
            for ch in CHARSET:
                rows = glyphs[ch][0]
                ys = [y for y in range(CH) for x in range(CW) if rows[y][x]]
                if ch.isupper() and ch.isalpha() and ys:
                    heights.append(max(ys) - min(ys) + 1)
            cap_h = round(sum(heights) / len(heights), 1) if heights else 0
            return path, size, cap_h
    raise RuntimeError("autofit found no collision-free native-fit font among %s"
                       % (candidates,))


def build_glyphs(font_path=None, font_size=None, **kwargs):
    """Render the whole charset. Returns {char: (rows, info)}."""
    if font_path is None:
        font_path = FONT_PATH
    if font_size is None:
        font_size = FONT_SIZE
    if font_path is None or font_size is None:
        font_path, font_size, _ = autofit()
    return {ch: render_glyph(ch, font_path, font_size, **kwargs) for ch in CHARSET}


def build_font(**kwargs):
    """{char_code: rows} for every printable ASCII char."""
    glyphs = build_glyphs(**kwargs)
    return {ord(ch): rows for ch, (rows, _) in glyphs.items()}


def row_bits(row):
    """Pack one glyph row: bit CW-1 (=11) is the leftmost column."""
    value = 0
    for x, on in enumerate(row):
        if on:
            value |= 1 << (CW - 1 - x)
    return value


def table_from_glyphs(glyphs):
    """{char_code: {row: bits}} -- the canonical form both emitters compare in."""
    return {ord(ch): {y: row_bits(r) for y, r in enumerate(rows)}
            for ch, (rows, _) in glyphs.items()}


# ---------------------------------------------------------------------------
# Self-checks
# ---------------------------------------------------------------------------
def check_font(glyphs):
    """Return a list of human-readable failures. Empty list means pass."""
    fails = []
    centre_x = (CW - 1) / 2.0
    centre_y = (CH - 1) / 2.0

    seen_bits = {}
    for ch in CHARSET:
        rows, info = glyphs[ch]
        code = ord(ch)
        xs = [x for y in range(CH) for x in range(CW) if rows[y][x]]
        ys = [y for y in range(CH) for x in range(CW) if rows[y][x]]

        if ch == " ":
            if xs:
                fails.append("0x20 space: expected blank, got %d lit pixels" % len(xs))
            continue

        if info["clipped"]:
            fails.append("0x%02X %r: solid ink was clipped by the box edge" % (code, ch))
        if not info["fits"]:
            fails.append("0x%02X %r: ink %dx%d does not fit the %dx%d box natively"
                         % (code, ch, info["ink"][0], info["ink"][1], CW, CH))
        if not xs:
            fails.append("0x%02X %r: no ink at all" % (code, ch))
            continue

        if min(xs) < 0 or max(xs) > CW - 1 or min(ys) < 0 or max(ys) > CH - 1:
            fails.append("0x%02X %r: ink escapes the %dx%d box" % (code, ch, CW, CH))

        for axis, lo, hi, centre in (("x", min(xs), max(xs), centre_x),
                                     ("y", min(ys), max(ys), centre_y)):
            got = (lo + hi) / 2.0
            if abs(got - centre) > 0.5:
                fails.append("0x%02X %r: ink centre %s = %.1f, expected %.1f"
                             % (code, ch, axis, got, centre))

        key = tuple(row_bits(r) for r in rows)
        if key in seen_bits:
            fails.append("0x%02X %r and 0x%02X %r rendered identically"
                         % (seen_bits[key], chr(seen_bits[key]), code, ch))
        else:
            seen_bits[key] = code

    return fails


def check_geometry():
    """The geometry must stay consistent with the bit-slice addressing."""
    fails = []
    if PITCH & (PITCH - 1):
        fails.append("PITCH %d is not a power of two; cell/col would need a divider" % PITCH)
    if H_ACTIVE % PITCH:
        fails.append("H_ACTIVE %d is not a multiple of PITCH %d" % (H_ACTIVE, PITCH))
    if CW * SCALE != CELL:
        fails.append("CW*SCALE = %d does not equal CELL %d" % (CW * SCALE, CELL))
    if CELL > PITCH:
        fails.append("CELL %d exceeds PITCH %d; glyphs would overlap" % (CELL, PITCH))
    # The cap that the original plan got wrong: s = x_pos + marq_pos must stay
    # inside POS_BITS, and marq_pos peaks at H_ACTIVE + n_cells*PITCH - 1.
    travel_last = H_ACTIVE + MAX_PC_CELLS * PITCH - 1
    if (H_ACTIVE - 1) + travel_last > (1 << POS_BITS) - 1:
        fails.append("x_pos + marq_pos peaks at %d, overflows %d bits (n_cells=%d)"
                     % ((H_ACTIVE - 1) + travel_last, POS_BITS, MAX_PC_CELLS))
    if MAX_PC_CELLS > CHAR_BUF_ENTRIES:
        fails.append("MAX_PC_CELLS %d exceeds the %d-entry char_buf"
                     % (MAX_PC_CELLS, CHAR_BUF_ENTRIES))
    if MAX_PC_CELLS * PITCH > (1 << POS_BITS) - 1:
        fails.append("text_w = %d overflows the %d-bit u compare"
                     % (MAX_PC_CELLS * PITCH, POS_BITS))
    if BAND_TEXT_Y + CELL > BAND_Y + BAND_H - 1:
        fails.append("glyph rows y%d..%d overflow the band y%d..%d"
                     % (BAND_TEXT_Y, BAND_TEXT_Y + CELL - 1, BAND_Y, BAND_Y + BAND_H - 1))
    return fails


def check_negative_controls():
    """The checks above must have teeth. Returns a list of failures."""
    fails = []
    good = build_glyphs()

    baseline = check_font(good)
    if baseline:
        fails.append("baseline font fails its own checks: %s" % baseline)

    blanked = dict(good)
    blanked["A"] = ([[0] * CW for _ in range(CH)], good["A"][1])
    if not check_font(blanked):
        fails.append("negative control: blanked 'A' was NOT caught")

    collided = dict(good)
    collided["B"] = (good["A"][0], good["B"][1])
    if not check_font(collided):
        fails.append("negative control: A==B collision was NOT caught")

    dirty_space = dict(good)
    dirty_space[" "] = ([[1] * CW for _ in range(CH)], good[" "][1])
    if not check_font(dirty_space):
        fails.append("negative control: non-blank space was NOT caught")

    fp = FONT_PATH or autofit()[0]
    fs = FONT_SIZE or autofit()[1]
    biased = {ch: render_glyph(ch, fp, fs, center_bias=1) for ch in CHARSET}
    if not check_font(biased):
        fails.append("negative control: center_bias=1 was NOT caught")

    good_pitch = globals()["PITCH"]
    if check_geometry():
        fails.append("baseline geometry fails its own checks")
    globals()["PITCH"] = 30
    if not check_geometry():
        fails.append("negative control: PITCH=30 was NOT caught")
    globals()["PITCH"] = good_pitch

    # The n_cells cap must have teeth: bumping it past 24 must trip the
    # x_pos+marq_pos overflow guard that the original plan missed.
    good_cap = globals()["MAX_PC_CELLS"]
    globals()["MAX_PC_CELLS"] = 32
    if not check_geometry():
        fails.append("negative control: MAX_PC_CELLS=32 overflow was NOT caught")
    globals()["MAX_PC_CELLS"] = good_cap

    return fails


# ---------------------------------------------------------------------------
# Verilog emission
# ---------------------------------------------------------------------------
def _comment_char(ch):
    if ch == " ":
        return "(space)"
    if ch == "\\":
        return "(backslash)"
    return ch


def emit_vh(glyphs, font_path, font_size):
    table = table_from_glyphs(glyphs)
    n_lit = sum(1 for code in table for r in range(CH) if table[code].get(r, 0))
    out = [
        "// Generated by tools/gen_marquee_ascii_font.py -- DO NOT EDIT BY HAND.",
        "// Re-run the generator instead; tools/check_marquee_ascii_transcription.py",
        "// compares this file against a fresh render, bit for bit.",
        "//",
        "// Included from inside marquee_overlay.v's module body, so this file holds",
        "// a bare function and must NOT be added to the .al file list.",
        "//",
        "// Charset   : printable ASCII 0x20..0x7E (96 glyphs); outer default is blank",
        "//             so any control/DEL byte on the wire can never light a glyph.",
        "// Font      : %s @ %d px, threshold %d" % (font_path, font_size, THRESHOLD),
        "// Box       : %d x %d px; marquee_overlay scales by %d into the %dx%d cell,"
        % (CW, CH, SCALE, CELL, CELL),
        "//             addressing gcol[4:1] / row[4:1] -- free bit slices, no divider.",
        "// Bit order : bit %d is the leftmost column of the row (osd_overlay's"
        % (CW - 1),
        "//             font8x8 convention); row 0 is the top.",
        "// Blank rows are folded into the inner default arm to keep the file short.",
        "// Lit row-words: %d of %d." % (n_lit, len(CHARSET) * CH),
        "",
        "function [11:0] ascii_glyph;",
        "    input [6:0] char_code;",
        "    input [3:0] row;",
        "    begin",
        "        case (char_code)",
    ]
    for ch in CHARSET:
        code = ord(ch)
        rows = table[code]
        out.append("            7'h%02X: case (row)   // %s" % (code, _comment_char(ch)))
        for y in range(CH):
            bits = rows.get(y, 0)
            if bits == 0:
                continue
            out.append("                4'd%d: ascii_glyph = 12'h%03X;" % (y, bits))
        out.append("                default: ascii_glyph = 12'h000;")
        out.append("            endcase")
    out.append("            default: ascii_glyph = 12'h000;")
    out.append("        endcase")
    out.append("    end")
    out.append("endfunction")
    out.append("")
    return "\n".join(out)


_CHAR_RE = re.compile(r"7'h([0-9A-Fa-f]{2}):\s*case \(row\)")
_ROW_RE = re.compile(r"4'd(\d+):\s*ascii_glyph\s*=\s*12'h([0-9A-Fa-f]{1,3});")


def parse_vh(text):
    """Read a .vh back into {char_code: {row: bits}}."""
    table = {}
    code = None
    for line in text.splitlines():
        line = line.strip()
        m = _CHAR_RE.match(line)
        if m:
            code = int(m.group(1), 16)
            table[code] = {}
            continue
        m = _ROW_RE.match(line)
        if m and code is not None:
            table[code][int(m.group(1))] = int(m.group(2), 16)
    return table


def tables_equal(parsed, expected):
    diffs = []
    if set(parsed) != set(expected):
        return ["char set differs: parsed %d codes vs expected %d codes"
                % (len(parsed), len(expected))]
    for code in sorted(expected):
        for row in range(CH):
            got = parsed[code].get(row, 0)
            want = expected[code].get(row, 0)
            if got != want:
                diffs.append("0x%02X row %d: .vh has 12'h%03X, generator has 12'h%03X"
                             % (code, row, got, want))
    return diffs


# ---------------------------------------------------------------------------
# Preview: a pixel-exact model of marquee_overlay.v's PC-text arm
# ---------------------------------------------------------------------------
_GLYPH_CACHE = {}


def _fill_glyph_cache(font):
    """{char_code: [CH row words]} for the preview modeller."""
    global _GLYPH_CACHE
    _GLYPH_CACHE = {}
    for code in range(0x20, 0x7F):
        rows = font.get(code, [[0] * CW for _ in range(CH)])
        _GLYPH_CACHE[code] = [row_bits(r) for r in rows]


def band_pixel_ascii(x, y, marq_pos, bg_rgb, char_buf, n_cells):
    """One pixel of the band, modelling the gated ASCII arm exactly.

    Mirrors the RTL: u = x_pos + marq_pos - H_ACTIVE on an 11-bit wrap;
    in_region = u < (n_cells<<5); cell_idx = u[9:5]; col = u[4:0]; the glyph bit
    is ascii_glyph(char_buf[cell], row[4:1])[11 - gcol[4:1]] (the x2 scale).
    """
    if not (BAND_Y <= y < BAND_Y + BAND_H):
        return bg_rgb
    if y == BAND_Y or y == BAND_Y + BAND_H - 1:
        return EDGE_RGB

    dim = tuple(c >> DIM_SHIFT for c in bg_rgb)

    if not (BAND_TEXT_Y <= y < BAND_TEXT_Y + CELL):
        return dim

    text_w = n_cells << 5
    u = (x + marq_pos - H_ACTIVE) & ((1 << POS_BITS) - 1)
    if u >= text_w:
        return dim

    cell = (u >> 5) & 0x1F
    col = u & 0x1F
    if cell >= n_cells or not (GUTTER <= col < GUTTER + CELL):
        return dim

    gcol = col - GUTTER            # 0..23
    row = y - BAND_TEXT_Y          # 0..23
    char_code = char_buf[cell]
    font_row = row >> 1            # 0..11  (the x2 scale, free slice)
    font_col = gcol >> 1           # 0..11
    bits = _GLYPH_CACHE[char_code][font_row]
    if (bits >> (CW - 1 - font_col)) & 1:
        return TEXT_RGB
    return dim


def render_strip(background, marq_pos, char_buf, n_cells, pad=12):
    y0, y1 = BAND_Y - pad, BAND_Y + BAND_H + pad
    img = Image.new("RGB", (H_ACTIVE, y1 - y0))
    px = img.load()
    src = background.load()
    for y in range(y0, y1):
        for x in range(H_ACTIVE):
            px[x, y - y0] = band_pixel_ascii(x, y, marq_pos, src[x, y], char_buf, n_cells)
    return img


def write_previews(font):
    os.makedirs(PREVIEW_DIR, exist_ok=True)
    paths = []

    # --- contact sheet: all 96 glyphs --------------------------------------
    zoom = 4
    cols = 16
    rows_n = (len(CHARSET) + cols - 1) // cols
    cell_w = CW * zoom + 18
    cell_h = CH * zoom + 16
    sheet = Image.new("RGB", (cols * cell_w + 24, rows_n * cell_h + 60), (16, 16, 24))
    d = ImageDraw.Draw(sheet)
    for i, ch in enumerate(CHARSET):
        code = ord(ch)
        cx = 12 + (i % cols) * cell_w
        cy = 12 + (i // cols) * cell_h
        d.rectangle([cx, cy + 12, cx + CW * zoom - 1, cy + 12 + CH * zoom - 1],
                    outline=(50, 70, 95))
        gr = font[code]
        for y in range(CH):
            for x in range(CW):
                if gr[y][x]:
                    ox = cx + x * zoom
                    oy = cy + 12 + y * zoom
                    d.rectangle([ox, oy, ox + zoom - 1, oy + zoom - 1], fill=TEXT_RGB)
        d.text((cx, cy), "0x%02X %s" % (code, _comment_char(ch)), fill=(200, 220, 255))
    d.text((12, rows_n * cell_h + 16),
           "ASCII %dx%d marquee font -- %d glyphs, %s @ %d px, threshold %d, scaled %dx into %dx%d cell"
           % (CW, CH, len(CHARSET), os.path.basename(FONT_PATH or "?"), FONT_SIZE or 0,
              THRESHOLD, SCALE, CELL, CELL),
           fill=(190, 190, 190))
    d.text((12, rows_n * cell_h + 34),
           "bit %d = leftmost column; row 0 = top; addressing gcol[4:1]/row[4:1]; outer default = blank"
           % (CW - 1),
           fill=(150, 150, 150))
    p = os.path.join(PREVIEW_DIR, "marquee_ascii_sheet.png")
    sheet.save(p)
    paths.append(p)

    # --- band over real demo pictures --------------------------------------
    backgrounds = [
        os.path.join(REPO, "doc", "TF卡图片", "西瓜_640x480_24bit_显示正常_工程适配版.bmp"),
        os.path.join(REPO, "doc", "convert", "3.png"),
    ]
    for bpath in backgrounds:
        if not os.path.exists(bpath):
            print("  (skip preview background, missing: %s)" % bpath)
            continue
        bg = Image.open(bpath).convert("RGB")
        if bg.size != (H_ACTIVE, V_ACTIVE):
            bg = bg.resize((H_ACTIVE, V_ACTIVE))
        strips = []
        for s in SAMPLE_STRINGS:
            char_buf = [ord(c) for c in s][:MAX_PC_CELLS]
            n_cells = len(char_buf)
            char_buf += [0x20] * (CHAR_BUF_ENTRIES - n_cells)
            for off in (700, 880):
                strips.append((render_strip(bg, off, char_buf, n_cells),
                               "%r n=%d marq_pos=%d" % (s[:18], n_cells, off)))
        strip_h = BAND_H + 24
        canvas = Image.new("RGB", (H_ACTIVE, strip_h * len(strips) + 8), (0, 0, 0))
        cd = ImageDraw.Draw(canvas)
        for i, (img, label) in enumerate(strips):
            canvas.paste(img, (0, i * strip_h))
            cd.text((4, i * strip_h + 1), label, fill=(255, 255, 255))
        stem = os.path.splitext(os.path.basename(bpath))[0]
        p = os.path.join(PREVIEW_DIR, "marquee_ascii_band_preview_%s.png" % stem)
        canvas.save(p)
        paths.append(p)
    return paths


# ---------------------------------------------------------------------------
def main():
    global FONT_PATH, FONT_SIZE
    failures = 0

    for arg in sys.argv[1:]:
        if arg.startswith("--font="):
            FONT_PATH = arg.split("=", 1)[1]
        elif arg.startswith("--size="):
            FONT_SIZE = int(arg.split("=", 1)[1])

    if FONT_PATH is None or FONT_SIZE is None:
        path, size, cap_h = autofit()
        if FONT_PATH is None:
            FONT_PATH = path
        if FONT_SIZE is None:
            FONT_SIZE = size
        print("=== autofit chose %s @ %d px (cap height %.1f px, collision-free, native fit)"
              % (os.path.basename(FONT_PATH), FONT_SIZE, cap_h))

    print("=== rendering %d ASCII glyphs (0x20..0x7E) into %dx%d, scale %dx -> %dx%d cell"
          % (len(CHARSET), CW, CH, SCALE, CELL, CELL))
    glyphs = build_glyphs()

    resampled = [ch for ch in CHARSET if glyphs[ch][1]["resampled"]]
    clipped = [ch for ch in CHARSET if glyphs[ch][1]["clipped"]]
    nofit = [ch for ch in CHARSET if not glyphs[ch][1]["fits"]]
    if resampled:
        print("  WARN: %d glyph(s) resampled: %s"
              % (len(resampled), " ".join("%r" % c for c in resampled)))
    if clipped:
        print("  FAIL: %d glyph(s) clipped: %s" % (len(clipped), " ".join("%r" % c for c in clipped)))
        failures += len(clipped)
    if nofit:
        print("  FAIL: %d glyph(s) do not fit natively: %s"
              % (len(nofit), " ".join("%r" % c for c in nofit)))
        failures += len(nofit)

    print("=== geometry  (display cap n_cells <= %d, char_buf %d entries)"
          % (MAX_PC_CELLS, CHAR_BUF_ENTRIES))
    geo = check_geometry()
    for f in geo:
        print("  FAIL: " + f)
    failures += len(geo)
    if not geo:
        travel_last = H_ACTIVE + MAX_PC_CELLS * PITCH - 1
        print("  pass: CW*SCALE==CELL==%d, n_cells<=%d keeps x_pos+marq_pos<=%d inside %d bits"
              % (CELL, MAX_PC_CELLS, (H_ACTIVE - 1) + travel_last, POS_BITS))

    print("=== glyph self-checks")
    glyph_fails = check_font(glyphs)
    for f in glyph_fails:
        print("  FAIL: " + f)
    failures += len(glyph_fails)
    if not glyph_fails:
        print("  pass: space blank, others non-empty/fit/centred, no collisions")

    print("=== negative controls")
    nc_fails = check_negative_controls()
    for f in nc_fails:
        print("  FAIL: " + f)
    failures += len(nc_fails)
    if not nc_fails:
        print("  pass: blank / collision / dirty-space / center_bias / PITCH=30 / n_cells=32 all caught")

    if failures:
        print("\n%d failure(s); not writing the .vh" % failures)
        return 1

    font = build_font()
    text = emit_vh(glyphs, FONT_PATH, FONT_SIZE)
    os.makedirs(os.path.dirname(VH_PATH), exist_ok=True)
    with open(VH_PATH, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    print("=== wrote %s (%d bytes, %d lines)" % (VH_PATH, len(text), text.count("\n")))

    expected = table_from_glyphs(glyphs)
    diffs = tables_equal(parse_vh(text), expected)
    if diffs:
        print("=== FAIL: round-trip through the .vh lost data")
        for dd in diffs[:10]:
            print("  " + dd)
        return 1
    print("=== round-trip parse of the .vh matches the render bit for bit")

    print("=== previews")
    _fill_glyph_cache(font)
    for p in write_previews(font):
        print("  " + p)

    print("\nOK -- inspect marquee_ascii_sheet.png and the band previews before building.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
