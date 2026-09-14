#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cross-check the PC-text subtitle arm: ASCII font, marquee_overlay.v's PC arm,
uart_pc_text.v, the top-level wiring/CDC, the project files and the F12 pin.

sim_marquee.py Pass G proves the PC arm *behaves* (and that pc_en=0 is the
bit-identical retreat); sim_uart_pc_text.py proves the link and the CDC crossing
behave. This file proves the four artefacts are still the design that was
intended and that nothing around them drifted:

  1. marquee_ascii_font.vh on disk is byte-identical to what
     gen_marquee_ascii_font.py emits for the font/size recorded in its own
     header, that fresh render still passes the generator's quality gates
     (space blank, every other glyph non-empty / native-fit / centred / no two
     chars identical), the geometry stays inside the 11-bit borrow budget, and
     the generator's own negative controls still bite.
  2. Every PC-arm constant, bit slice and colour in marquee_overlay.v is
     re-derived from the generator's constants -- char stride 7 (the contract
     uart_pc_text.v writes with), the free [4:1] x2 scale slices, glyph msb
     CW-1, PC_CELLS_MAX == MAX_PC_CELLS, the runtime text_w/travel_last muxes,
     the I_pc_en output-mux branch and its two colours, the slogan face gated
     off under I_pc_en, and band_active gated by I_pc_en. Plus the port widths
     (I_pc_cells 6, I_pc_char_buf 224) and that cell_idx is still u[9:5].
  3. The top wires the marquee's three PC ports to the FRAME registers
     (pc_en_gated / pc_cells_frame / pc_buf_frame, never the staging regs),
     instantiates uart_pc_text once with every port connected, declares the
     uart_pc_rx input and PC_TEXT_ENABLE, and crosses pc_text_en on a bare 2FF /
     char_buf+n_cells on data+toggle / releases frame-atomically.
  4. Build + pin: uart_pc_text.v registered in the .al, marquee_ascii_font.vh
     deliberately NOT registered and reached by a second `include inside
     marquee_overlay.v, ascii_glyph's signature is 12/7/4 bits, no declared
     identifier collides with a Verilog-2001 reserved word, and pin.adc puts
     uart_pc_rx on F12.
  5. In-memory mutations, each of which the checks above must catch.

The RTL parsing is imported from sim_marquee and the structural helpers from
check_marquee_transcription, so the three tools cannot disagree about the Verilog.

Run from anywhere:  python tools/check_marquee_ascii_transcription.py
"""
import contextlib
import io
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import gen_marquee_ascii_font as gena
import sim_marquee as sim
import check_marquee_transcription as base

REPO = os.path.dirname(TOOLS)
HDL = os.path.join(REPO, "src", "user_source", "hdl_source")
RTL = os.path.join(HDL, "marquee_overlay.v")
UART_RTL = os.path.join(HDL, "uart_pc_text.v")
VH = gena.VH_PATH
TOP = os.path.join(HDL, "top_tf_hdmi_audio.v")
AL = os.path.join(REPO, "src", "td_project", "HDMI1.4b_Transmitter_v1.0.al")
ADC = os.path.join(REPO, "src", "user_source", "constraints_source", "pin.adc")

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
    CHECKS[0] += 1
    if not cond:
        print("    ok    control bites: %s" % label)
        return True
    FAILURES.append("control did NOT bite: %s" % label)
    print("    FAIL  control did NOT bite: %s%s"
          % (label, (" -- " + detail) if detail else ""))
    return False


def read(path):
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def packed(rgb):
    return (rgb[0] << 16) | (rgb[1] << 8) | rgb[2]


class Sources(object):
    """The six files this checker reads, so controls can mutate one in memory."""

    def __init__(self, rtl=None, vh=None, top=None, al=None, adc=None, uart=None):
        self.rtl = read(RTL) if rtl is None else rtl
        self.vh = read(VH) if vh is None else vh
        self.top = read(TOP) if top is None else top
        self.al = read(AL) if al is None else al
        self.adc = read(ADC) if adc is None else adc
        self.uart = read(UART_RTL) if uart is None else uart

    def clone(self, **kw):
        return Sources(rtl=kw.get("rtl", self.rtl), vh=kw.get("vh", self.vh),
                       top=kw.get("top", self.top), al=kw.get("al", self.al),
                       adc=kw.get("adc", self.adc), uart=kw.get("uart", self.uart))


# ---------------------------------------------------------------------------
# 1. The ASCII font
# ---------------------------------------------------------------------------
def check_font(src):
    print("\n[1] marquee_ascii_font.vh vs. a fresh render of the recorded font")

    m = re.search(r"//\s*Font\s*:\s*(.+?)\s*@\s*(\d+)\s*px", src.vh)
    if not check(m is not None, "the .vh header records the font and size it was "
                                "rendered from", "no '// Font : PATH @ N px' line"):
        return
    font_path, font_size = m.group(1).strip(), int(m.group(2))
    if not check(os.path.exists(font_path),
                 "the recorded font %s exists, so the table can be re-derived"
                 % font_path):
        return

    # Pin the generator to the recorded face so the comparison is stable across
    # machines and does not re-run autofit (which could pick a different font).
    gena.FONT_PATH, gena.FONT_SIZE = font_path, font_size
    glyphs = gena.build_glyphs()

    quality = gena.check_font(glyphs)
    check(not quality,
          "a fresh render of all %d ASCII glyphs passes the generator's quality "
          "gates (space blank, others fit/centred/distinct)" % len(gena.CHARSET),
          "; ".join(quality[:4]))

    geo = gena.check_geometry()
    check(not geo,
          "the geometry stays consistent: CW*SCALE==CELL==%d, n_cells<=%d keeps "
          "x_pos+marq_pos inside %d bits"
          % (gena.CELL, gena.MAX_PC_CELLS, gena.POS_BITS), "; ".join(geo))

    nc = gena.check_negative_controls()
    check(not nc, "the generator's own negative controls still bite "
                  "(blank / collision / dirty-space / center_bias / PITCH=30 / n_cells=32)",
          "; ".join(nc[:4]))

    expected = gena.emit_vh(glyphs, font_path, font_size)
    check(src.vh.rstrip("\n") == expected.rstrip("\n"),
          "marquee_ascii_font.vh is byte-identical to gen_marquee_ascii_font.py's "
          "output (%d bytes, nobody edited it by hand)" % len(expected))

    diffs = gena.tables_equal(gena.parse_vh(src.vh), gena.table_from_glyphs(glyphs))
    check(not diffs,
          "all %d case items (%d codes x %d rows) match bit for bit"
          % (len(gena.CHARSET) * gena.CH, len(gena.CHARSET), gena.CH),
          "%d differing rows, first: %s" % (len(diffs), diffs[0] if diffs else ""))

    check("printable ASCII 0x20..0x7E" in src.vh,
          "the .vh header records the charset 0x20..0x7E with a blank outer default")
    check("bit %d is the leftmost column" % (gena.CW - 1) in src.vh,
          "the .vh header records bit %d = leftmost column (osd_overlay convention)"
          % (gena.CW - 1))


# ---------------------------------------------------------------------------
# 2. The PC arm in marquee_overlay.v, re-derived from the generator
# ---------------------------------------------------------------------------
def check_pc_arm(src):
    print("\n[2] marquee_overlay.v's PC arm re-derived from the generator")

    cfg, widths, ops = base.parse_rtl_text(src.rtl)

    if not check(ops.get("has_pc_mux"),
                 "marquee_overlay.v has the I_pc_en output-mux branch"):
        return

    # The char stride is a cross-module contract: marquee reads char_base =
    # cell_idx*STRIDE and uart_pc_text writes char_buf[char_count*STRIDE +: 7].
    # Both must agree, and the width must be 7 bits (ASCII).
    um = re.search(r"char_count\s*\*\s*8'd(\d+)", sim.strip_comments(src.uart))
    uart_stride = int(um.group(1)) if um else None
    derived = {
        "pc_char_stride": 7,
        "pc_char_width": 7,
        "pc_glyph_msb": gena.CW - 1,          # 11, bit CW-1 is leftmost
        "pc_cells_max": gena.MAX_PC_CELLS,    # 24
        "pc_cells_shift": base.bits_to_log2(gena.PITCH),   # 5, text_w = n<<5
        "travel_base": gena.H_ACTIVE,
        "travel_sub": 1,
    }
    bad = []
    for name, want in sorted(derived.items()):
        if ops.get(name) != want:
            bad.append("%s: RTL %s, derived %s" % (name, ops.get(name), want))
    check(not bad, "the PC-arm addressing constants are the ones the generator implies",
          "; ".join(bad))

    check(uart_stride == ops["pc_char_stride"] == 7,
          "marquee's char stride and uart_pc_text's write stride are both 7 "
          "(cell_idx*7 +: 7 == char_count*7 +: 7)",
          "marquee=%s uart=%s" % (ops["pc_char_stride"], uart_stride))

    # The x2 scale must be the free [4:1] slices on BOTH axes (12x12 -> 24x24).
    # A [4:0] slice or a divide would alias / stretch the glyph; parse_ops already
    # stales on anything but (4,1), so reaching here means the shape is right --
    # assert it explicitly so the record shows it.
    check(ops["pc_font_row_slice"] == (4, 1) and ops["pc_font_col_slice"] == (4, 1),
          "the x2 scale is the free row[4:1] / gcol[4:1] slices (no divide LUT)",
          "row%s col%s" % (ops["pc_font_row_slice"], ops["pc_font_col_slice"]))
    check(gena.CW * gena.SCALE == gena.CELL == cfg["CELL"],
          "CW(%d) * SCALE(%d) == CELL(%d) == the RTL's CELL(%d)"
          % (gena.CW, gena.SCALE, gena.CELL, cfg["CELL"]))

    # text_w / travel_last are runtime values for the PC string, constants for the
    # slogan. region_lt is the slogan TEXT_W the else-branch falls back to.
    check(ops["text_w_runtime"],
          "text_w is a runtime mux (I_pc_en ? {pc_cells_sat,5'b0} : TEXT_W), not a "
          "single constant")
    check(ops["region_lt"] == cfg["TEXT_W"],
          "the slogan fallback text_w stays TEXT_W=%d" % cfg["TEXT_W"],
          "region_lt=%s" % ops["region_lt"])
    check(ops["travel_last_const"] == ops["travel_base"] + ops["region_lt"] - ops["travel_sub"],
          "travel_last_const(%d) == H_ACTIVE + TEXT_W - 1, consistent with the "
          "runtime travel_last base/sub" % ops["travel_last_const"])

    # Colours: the PC face reuses the slogan's warm yellow and the band's cyan edge.
    check(ops["pc_text_rgb"] == packed(gena.TEXT_RGB)
          and ops["pc_edge_rgb"] == packed(gena.EDGE_RGB),
          "the PC mux uses 24'h%06X for the face and 24'h%06X for the edges "
          "(same colours as the slogan band)"
          % (packed(gena.TEXT_RGB), packed(gena.EDGE_RGB)),
          "RTL has 24'h%06X / 24'h%06X" % (ops["pc_text_rgb"], ops["pc_edge_rgb"]))

    # Gating: the slogan face must be OFF while the PC string is up, the PC face
    # must be gated by I_pc_en, and band_active must light for I_pc_en alone so a
    # PC string shows even when the SW4 slogan mask hides the banner.
    check(ops["slogan_off_under_pc"], "text_on ends with `&& !I_pc_en` (slogan face "
                                      "gated off under the PC string)")
    check(ops["pc_text_on_gated"], "text_on_pc ends with `&& I_pc_en`")
    check(ops["pc_gates_band"] and ops["band_gate"] == "band_active"
          and ops["en_gates_band"],
          "in_band gates on band_active = I_en || I_pc_en, so the PC string can "
          "light the band on its own")

    # Port widths and the cell index slice.
    ports = base.module_ports(src.rtl, "marquee_overlay")
    check(ports.get("I_pc_en") == ("input", 1),
          "I_pc_en is a 1-bit input", "got %s" % (ports.get("I_pc_en"),))
    check(ports.get("I_pc_cells") == ("input", 6),
          "I_pc_cells is 6 bits (holds 0..PC_CELLS_MAX and the saturation guard)",
          "got %s" % (ports.get("I_pc_cells"),))
    check(ports.get("I_pc_char_buf") == ("input", gena.CHAR_BUF_ENTRIES * 7),
          "I_pc_char_buf is %d bits = %d entries x 7"
          % (gena.CHAR_BUF_ENTRIES * 7, gena.CHAR_BUF_ENTRIES),
          "got %s" % (ports.get("I_pc_char_buf"),))
    check(ops["cell_slice"] == (9, 5),
          "cell_idx is still u[9:5] (5 bits address the %d-entry char_buf)"
          % gena.CHAR_BUF_ENTRIES, "got %s" % (ops["cell_slice"],))

    # The cap really is the geometry limit: bumping it past MAX_PC_CELLS would
    # overflow the 11-bit borrow. gena.check_geometry already proved the shipped
    # cap fits; assert the RTL cap equals it (done above) and that 24 is the
    # largest n_cells with (H_ACTIVE-1) + (H_ACTIVE + n*PITCH - 1) <= 2**POS_BITS-1.
    limit = 0
    for n in range(gena.CHAR_BUF_ENTRIES + 1):
        if (gena.H_ACTIVE - 1) + (gena.H_ACTIVE + n * gena.PITCH - 1) <= (1 << gena.POS_BITS) - 1:
            limit = n
    check(ops["pc_cells_max"] == limit == gena.MAX_PC_CELLS,
          "PC_CELLS_MAX=%d is the largest n_cells that keeps x_pos+marq_pos inside "
          "%d bits (next cell would peak at %d)"
          % (limit, gena.POS_BITS,
             (gena.H_ACTIVE - 1) + (gena.H_ACTIVE + (limit + 1) * gena.PITCH - 1)))
    return cfg, widths, ops


# ---------------------------------------------------------------------------
# 3. Top-level wiring + CDC
# ---------------------------------------------------------------------------
def check_wiring(src):
    print("\n[3] the PC channel in top_tf_hdmi_audio.v")

    top = sim.strip_comments(src.top)

    # the marquee's three PC ports must come from the FRAME registers, never the
    # staging regs -- otherwise a string could tear mid-frame.
    _p, inst, body = base.find_instance(top, "marquee_overlay")
    if not check(inst is not None, "top instantiates marquee_overlay"):
        return
    conn, dupes = base.connections(body)
    check(not dupes, "no marquee port is connected twice", "duplicates: %s" % dupes)
    wanted = {"I_pc_en": "pc_en_gated", "I_pc_cells": "pc_cells_frame",
              "I_pc_char_buf": "pc_buf_frame"}
    bad = ["%s is driven by %s, expected %s" % (p, conn.get(p), s)
           for p, s in sorted(wanted.items()) if conn.get(p) != s]
    check(not bad, "the marquee's PC ports read the frame-atomic registers "
                   "(pc_en_gated / pc_cells_frame / pc_buf_frame)", "; ".join(bad))
    # the slogan ports must be untouched
    slogan = {"I_en": "marquee_en", "I_3d": "font_frame",
              "I_rgb": "vout_data_osd", "O_rgb": "vout_data"}
    sbad = ["%s is %s, expected %s" % (p, conn.get(p), s)
            for p, s in sorted(slogan.items()) if conn.get(p) != s]
    check(not sbad, "the slogan ports are unchanged alongside the new PC ports",
          "; ".join(sbad))

    # uart_pc_text instance: every port connected exactly once, to the right nets.
    _p, uinst, ubody = base.find_instance(top, "uart_pc_text")
    if check(uinst is not None, "top instantiates uart_pc_text"):
        uconn, udupes = base.connections(ubody)
        check(not udupes, "no uart_pc_text port is connected twice",
              "duplicates: %s" % udupes)
        uports = base.module_ports(src.uart, "uart_pc_text")
        check(sorted(uconn) == sorted(uports),
              "every uart_pc_text port is connected exactly once (%d connections)"
              % len(uconn),
              "missing %s, unexpected %s"
              % (sorted(set(uports) - set(uconn)), sorted(set(uconn) - set(uports))))
        uwanted = {"clk": "clk", "rst": "rst_all", "uart_pc_rx": "uart_pc_rx",
                   "pc_text_en": "pc_text_en", "char_buf": "pc_char_buf",
                   "n_cells": "pc_n_cells", "text_toggle": "pc_text_toggle"}
        ubad = ["%s is %s, expected %s" % (p, uconn.get(p), s)
                for p, s in sorted(uwanted.items()) if uconn.get(p) != s]
        check(not ubad, "uart_pc_text's clock/reset/rx and committed outputs go "
                        "where the plan says", "; ".join(ubad))

    # the top must expose uart_pc_rx and own PC_TEXT_ENABLE
    check(re.search(r"\binput\s+(?:wire\s+)?uart_pc_rx\b", top) is not None,
          "the top declares uart_pc_rx as an input port")
    check(re.search(r"parameter\s+PC_TEXT_ENABLE\s*=\s*1'b0", top) is not None,
          "the top ships PC_TEXT_ENABLE = 1'b0, gated off to pay for the four-track "
          "audio table and not because the channel is broken; flipping it back to 1 "
          "needs ~376 slices freed first")

    # The parameter is only a *total* retreat because led[3] is gated by it too.
    # Wired straight to dbg_pc_rx_toggle, the observation LED is a live load on
    # u_uart_pc_text and the module survives synthesis whatever the parameter says.
    # led[3] is now a two-way tap shared with the Type-C command source, so the
    # mux form has to keep that property: dbg_pc_rx_toggle may only be reached
    # through the PC_TEXT_ENABLE-true branch.
    check(re.search(r"assign\s+led\s*=\s*\{\s*PC_TEXT_ENABLE\s*\?\s*"
                    r"dbg_pc_rx_toggle\s*:\s*\(\s*PC_CMD_ENABLE\s*&\s*"
                    r"dbg_pc_cmd_toggle\s*\)", top) is not None,
          "led[3] is PC_TEXT_ENABLE ? dbg_pc_rx_toggle : (PC_CMD_ENABLE & "
          "dbg_pc_cmd_toggle) -- each PC-side tap is gated by its own enable, so "
          "clearing either parameter really removes that module instead of "
          "leaving it loaded by the LED")

    # CDC structure: bare 2FF on the level, data+toggle, frame-atomic release.
    check(re.search(r"pc_en_gated\s*=\s*PC_TEXT_ENABLE\s*&\s*pc_en_frame", top) is not None,
          "pc_en_gated = PC_TEXT_ENABLE & pc_en_frame (the gated total retreat)")
    check(re.search(r"pc_en_v0\s*<=\s*pc_text_en;\s*pc_en_v1\s*<=\s*pc_en_v0;", top) is not None,
          "pc_text_en crosses clk -> video_clk on a bare 2FF (pc_en_v0/v1)")
    check(re.search(r"pc_tgl_s0\s*<=\s*pc_text_toggle;\s*pc_tgl_s1\s*<=\s*pc_tgl_s0;\s*"
                    r"pc_tgl_s2\s*<=\s*pc_tgl_s1;", top) is not None
          and re.search(r"pc_tgl_edge\s*=\s*pc_tgl_s1\s*\^\s*pc_tgl_s2", top) is not None,
          "text_toggle crosses on a 3FF chain and pc_tgl_edge = s1 ^ s2")
    check(re.search(r"if\s*\(pc_tgl_edge\)\s*begin\s*pc_buf_stg\s*<=\s*pc_char_buf;\s*"
                    r"pc_cells_stg\s*<=\s*pc_n_cells;", top) is not None,
          "the toggle edge latches char_buf/n_cells into staging (data+toggle)")
    m = re.search(r"if\s*\(\s*video_frame_start\s*\)\s*begin(.*?)end", top, flags=re.S)
    if check(m is not None, "the top has an `if (video_frame_start)` branch"):
        branch = re.sub(r"\s+", " ", m.group(1))
        for frag in ("pc_en_frame <= pc_en_v1;", "pc_buf_frame <= pc_buf_stg;",
                     "pc_cells_frame <= pc_cells_stg;"):
            check(frag in branch,
                  "%s sits inside the frame-atomic branch (no mid-frame tear)" % frag)


# ---------------------------------------------------------------------------
# 4. Build files + pin
# ---------------------------------------------------------------------------
def check_build_files(src):
    print("\n[4] project registration, the second `include, ascii_glyph, the pin")

    # uart_pc_text.v registered, with a unique CompileOrder in the Verilog section.
    m = re.search(r'<File Path="([^"]*uart_pc_text\.v)">(.*?)</File>', src.al, flags=re.S)
    if check(m is not None, "uart_pc_text.v is registered in the .al"):
        body = m.group(2)
        check(m.group(1) == "../user_source/hdl_source/uart_pc_text.v",
              "the .al path is %s" % m.group(1))
        for attr, want in (("UsedInSyn", "true"), ("UsedInP&R", "true"),
                           ("BelongTo", "design_1")):
            check('Name="%s" Val="%s"' % (attr, want) in body,
                  '%s = %s for uart_pc_text.v' % (attr, want))
        section = re.search(r"<Verilog>(.*?)</Verilog>", src.al, flags=re.S)
        if check(section is not None, "the .al has a <Verilog> section"):
            orders = [int(v) for v in
                      re.findall(r'Name="CompileOrder" Val="(\d+)"', section.group(1))]
            dupes = sorted({o for o in orders if orders.count(o) > 1})
            check(not dupes, "CompileOrder is unique across the %d Verilog entries"
                  % len(orders), "duplicates: %s" % dupes)

    check("marquee_ascii_font.vh" not in src.al,
          "marquee_ascii_font.vh is NOT in the .al -- a bare function cannot compile "
          "standalone and would break the build")

    # marquee_overlay.v must `include BOTH font tables inside the module body.
    code = src.rtl
    incs = re.findall(r'`include\s+"([^"]+)"', code)
    check("marquee_font.vh" in incs and "marquee_ascii_font.vh" in incs,
          "marquee_overlay.v includes both marquee_font.vh and marquee_ascii_font.vh",
          "found %s" % incs)
    if "marquee_ascii_font.vh" in incs:
        idx = code.index('`include "marquee_ascii_font.vh"')
        check(idx < code.rindex("endmodule"),
              "the ascii `include sits inside the module body, before endmodule")
        check(os.path.isfile(os.path.join(HDL, "marquee_ascii_font.vh")),
              "marquee_ascii_font.vh exists next to the .v, so the include resolves")

    # ascii_glyph signature: 12-bit return, 7-bit char_code, 4-bit row.
    fn = re.search(r"function\s+\[\s*(\d+)\s*:\s*0\s*\]\s+ascii_glyph\s*;", src.vh)
    if check(fn is not None, "the .vh defines ascii_glyph"):
        check(int(fn.group(1)) + 1 == gena.CW,
              "ascii_glyph returns %d bits, one per glyph column (CW=%d)"
              % (int(fn.group(1)) + 1, gena.CW))
    cc = re.search(r"input\s+\[\s*(\d+)\s*:\s*0\s*\]\s+char_code\s*;", src.vh)
    rw = re.search(r"input\s+\[\s*(\d+)\s*:\s*0\s*\]\s+row\s*;", src.vh)
    check(cc is not None and int(cc.group(1)) + 1 == 7,
          "ascii_glyph's char_code is 7 bits (matches the char_buf cell width)",
          "got %s" % (cc.group(0) if cc else None))
    check(rw is not None and int(rw.group(1)) + 1 == base.bits_to_log2(gena.CH),
          "ascii_glyph's row is %d bits, enough for %d glyph rows"
          % (int(rw.group(1)) + 1 if rw else 0, gena.CH),
          "got %s" % (rw.group(0) if rw else None))
    check("endfunction" in src.vh
          and not re.search(r"\bmodule\b", sim.strip_comments(src.vh)),
          "the ascii .vh holds a bare function and no module of its own")

    # reserved-word collisions across every file this feature touched
    reserved = []
    for tag, text in (("marquee_overlay.v", src.rtl),
                      ("marquee_ascii_font.vh", src.vh),
                      ("uart_pc_text.v", src.uart),
                      ("top_tf_hdmi_audio.v", src.top)):
        hits = sorted(base.declared_names(sim.strip_comments(text)) & base.RESERVED_WORDS)
        reserved += ["%s declares %s" % (tag, h) for h in hits]
    check(not reserved,
          "no declared identifier collides with a Verilog-2001 reserved word "
          "(TD answers that with a syntax error, not a warning)",
          "; ".join(reserved))

    # the F12 pin
    m = re.search(r"set_pin_assignment\s*\{\s*uart_pc_rx\s*\}\s*\{([^}]*)\}", src.adc)
    if check(m is not None, "pin.adc constrains uart_pc_rx"):
        attrs = m.group(1)
        check("LOCATION = F12" in attrs, "uart_pc_rx is on F12 (the Type-C/CH340 host "
                                         "route, separate from the screen's J1 D14/G11)",
              attrs.strip())
        check("IOSTANDARD = LVCMOS33" in attrs, "uart_pc_rx is LVCMOS33", attrs.strip())
        check("PULLTYPE = PULLUP" in attrs, "uart_pc_rx is PULLUP so an idle link reads "
                                            "high (no false start bits)", attrs.strip())


# ---------------------------------------------------------------------------
# 5. Negative controls
# ---------------------------------------------------------------------------
def bites(fn, src, label):
    """Run a check function against a mutated source; it must complain."""
    saved, saved_n = list(FAILURES), CHECKS[0]
    del FAILURES[:]
    CHECKS[0] = 0
    buf = io.StringIO()
    caught = False
    try:
        with contextlib.redirect_stdout(buf):
            fn(src)
        caught = bool(FAILURES)
    except SystemExit:
        caught = True            # the parser refusing to guess counts as a catch
    finally:
        del FAILURES[:]
        FAILURES.extend(saved)
        CHECKS[0] = saved_n
    expect_fail(not caught, label)


def sub_once(text, pattern, repl, what):
    new, n = re.subn(pattern, repl, text, count=1)
    if n != 1:
        raise SystemExit("control setup failed: %s not found" % what)
    return new


def check_controls(src):
    print("\n[5] negative controls: each mutation must be caught")

    # a glyph bit flipped in the .vh
    mutated = sub_once(src.vh, r"12'h([0-9A-Fa-f]{3})",
                       lambda m: "12'h%03X" % ((int(m.group(1), 16) ^ 0x001)),
                       "a glyph row in the ascii .vh")
    bites(check_font, src.clone(vh=mutated),
          "N1 flipping one bit of one glyph row in marquee_ascii_font.vh")

    bites(check_pc_arm,
          src.clone(rtl=sub_once(src.rtl, r"assign char_base\s*= cell_idx \* 8'd7;",
                                 "assign char_base   = cell_idx * 8'd6;",
                                 "the char stride")),
          "N2 a char stride of 6 (not 7) misreads char_buf")

    bites(check_pc_arm,
          src.clone(rtl=sub_once(src.rtl, r"pc_glyph\[\s*4'd11\s*-\s*pc_font_col\s*\]",
                                 "pc_glyph[4'd10 - pc_font_col]", "the glyph msb")),
          "N3 indexing the glyph at 10-col (not 11-col) shifts every glyph")

    bites(check_pc_arm,
          src.clone(rtl=sub_once(src.rtl, r"localparam \[5:0\] PC_CELLS_MAX = 6'd24;",
                                 "localparam [5:0] PC_CELLS_MAX = 6'd32;",
                                 "PC_CELLS_MAX")),
          "N4 raising PC_CELLS_MAX to 32 overflows the 11-bit borrow math")

    bites(check_pc_arm,
          src.clone(rtl=sub_once(src.rtl, r"assign pc_font_row = row\[4:1\];",
                                 "assign pc_font_row = row[4:0];", "the row scale slice")),
          "N5 a [4:0] row slice (no x2 scale) aliases the glyph rows")

    # drop the I_pc_en output-mux branch entirely
    bites(check_pc_arm,
          src.clone(rtl=re.sub(
              r"I_pc_en\s*\?\s*\(text_on_pc[^:]*:\s*band_edge[^:]*:\s*"
              r"\{dim_r, dim_g, dim_b\}\)\s*:",
              "", src.rtl, count=1)),
          "N6 removing the I_pc_en output-mux branch leaves no PC arm at all")

    bites(check_wiring,
          src.clone(top=sub_once(src.top, r"\.I_pc_char_buf \(pc_buf_frame\)",
                                 ".I_pc_char_buf (pc_buf_stg)", "the char_buf source")),
          "N7 feeding the marquee from the staging reg (not the frame reg) could "
          "tear a string mid-frame")

    bites(check_wiring,
          src.clone(top=sub_once(src.top,
                                 r"pc_en_gated = PC_TEXT_ENABLE & pc_en_frame",
                                 "pc_en_gated = pc_en_frame", "the PC_TEXT_ENABLE gate")),
          "N8 dropping PC_TEXT_ENABLE from pc_en_gated removes the one-line retreat")

    bites(check_build_files,
          src.clone(al=src.al.replace("</Verilog>",
                                      '    <File Path="../user_source/hdl_source/'
                                      'marquee_ascii_font.vh">\n        </File>\n'
                                      '        </Verilog>')),
          "N9 registering the bare-function ascii .vh in the .al")

    bites(check_build_files,
          src.clone(al=re.sub(r'\s*<File Path="[^"]*uart_pc_text\.v">.*?</File>',
                              "", src.al, flags=re.S)),
          "N10 dropping uart_pc_text.v from the .al")

    bites(check_build_files,
          src.clone(adc=sub_once(src.adc, r"LOCATION = F12;", "LOCATION = D12;",
                                 "the uart_pc_rx pin")),
          "N11 moving uart_pc_rx off F12 (D12 is the dead device-to-device route)")

    bites(check_build_files,
          src.clone(uart=sub_once(src.uart, r"reg \[5:0\]\s+char_count;",
                                  "reg [5:0]  cell;", "a reserved-word declaration")),
          "N12 declaring the parser counter as `cell`, a Verilog-2001 reserved word")

    bites(check_wiring,
          src.clone(top=sub_once(
              src.top,
              r"assign led = \{PC_TEXT_ENABLE \? dbg_pc_rx_toggle\s*\n\s*"
              r": \(PC_CMD_ENABLE & dbg_pc_cmd_toggle\),",
              "assign led = {dbg_pc_rx_toggle,",
              "the PC_TEXT_ENABLE mux on led[3]")),
          "N13 wiring led[3] straight to dbg_pc_rx_toggle leaves u_uart_pc_text "
          "loaded by the LED, so PC_TEXT_ENABLE=0 stops being a total retreat")

    bites(check_wiring,
          src.clone(top=sub_once(
              src.top,
              r":\s*\(PC_CMD_ENABLE & dbg_pc_cmd_toggle\)",
              ": dbg_pc_cmd_toggle",
              "the PC_CMD_ENABLE gate on led[3]'s command branch")),
          "N14 dropping PC_CMD_ENABLE from the mux's false branch leaves the "
          "Type-C parser loaded by the LED too -- the same trap on the other "
          "side of the switch")


# ---------------------------------------------------------------------------
def main():
    print("=" * 72)
    print("PC-text transcription check: ascii font vs. RTL vs. top vs. project")
    print("=" * 72)
    for path in (RTL, VH, UART_RTL, TOP, AL, ADC):
        if not os.path.isfile(path):
            raise SystemExit("check_marquee_ascii_transcription: %s is missing" % path)
    src = Sources()

    check_font(src)
    check_pc_arm(src)
    check_wiring(src)
    check_build_files(src)
    check_controls(src)

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
