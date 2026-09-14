#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Transcription gate for the adjustable carousel interval (SPED n, n = 1..8 s).

There is no Verilog simulator on this machine. tools/sim_uart_ctrl.py proves the
command is parsed and crossed into sd_card_clk with the right value, and
tools/sim_load_retry.py Pass G proves the interval counter behaves and that its
own negative controls bite. What neither can see is a hand-copy error between
those models and the three .v files. This file closes that gap.

Two dangers are specific to this feature, and both are silent:

  Shared constant.  sd_card_bmp.v spells `CLK_FREQ_HZ - 1` twice: once for
  auto_cnt, the carousel tick, and once for load_stall_cnt, the picture-level
  no-progress watchdog. They look like the same "one second" and it is tempting
  to fold them into a single interval variable now that the interval is
  adjustable. They are not the same second. The watchdog is one term of README
  section 9's invariant -- sector 300 ms < scaler 671 ms < picture 1000 ms --
  and stretching it to 8 s would let a genuinely dead load sit for eight seconds
  before the retry fires. Section 3 asserts both spellings are still there and
  that no third one appeared.

  Timing.  sd_card_clk is the tightest of the four domains. Making auto_tick a
  variable compare would put a 32-bit magnitude comparator against a register
  loaded from another clock domain onto that path. The design keeps auto_tick
  exactly as it was -- a constant compare -- and adds sec_last, two 3-bit
  registers meeting at a single AND, with target-1 precomputed when SPED
  arrives so no subtractor sits in the tick path either. Section 3 locks all
  three shapes as text, because "is it still a constant compare" is precisely
  the question a behavioural model cannot answer.

Sections:
  1  uart_screen_ctrl.v -- port widths, the SPED arm's exact guard (its accepted
     range is read out of sim_uart_ctrl.SPEED_DIGITS, never retyped here), the
     one-clock strobe default and the reset.
  2  top_tf_hdmi_audio.v -- AUTO_SEC_DEFAULT, the two uart_screen_ctrl instances
     and their own j1_/pc_ nets, the one merge point (strobe OR-ed, value
     priority-muxed with J1 winning, and PC_CMD_ENABLE gating both halves so the
     synthesiser can actually delete the second instance), the
     data+toggle latch order, the 3FF sync chain, the s1^s2 pulse, every reset
     value, and cmd_speed_set in cmd_any_set so LED2 still reports the command.
  3  sd_card_bmp.v -- the two `CLK_FREQ_HZ - 1` spellings, sec_last's shape, the
     auto-play block verbatim, the SPED handler, and the pairing rule: every
     place that clears auto_cnt must clear sec_cnt in the same scope, or that
     site hands the next interval a part-elapsed second.
  4  In-memory mutations, each of which the checks above must catch.

Run from anywhere:  python tools/check_uart_speed_transcription.py
"""
import contextlib
import io
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import sim_uart_ctrl as su          # noqa: E402

REPO = os.path.dirname(TOOLS)
HDL = os.path.join(REPO, "src", "user_source", "hdl_source")
UART = os.path.join(HDL, "uart_screen_ctrl.v")
TOP = os.path.join(HDL, "top_tf_hdmi_audio.v")
SDB = os.path.join(HDL, "SD", "sd_card_bmp.v")

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


class Sources(object):
    """The three files, so a control can mutate one of them in memory."""

    def __init__(self, uart=None, top=None, sd=None):
        self.uart = read(UART) if uart is None else uart
        self.top = read(TOP) if top is None else top
        self.sd = read(SDB) if sd is None else sd

    def clone(self, **kw):
        return Sources(uart=kw.get("uart", self.uart),
                       top=kw.get("top", self.top),
                       sd=kw.get("sd", self.sd))


# ---------------------------------------------------------------------------
# Text helpers
# ---------------------------------------------------------------------------
def strip_comments(text):
    """Blank out // and /* */ comments, preserving offsets and newlines.

    Offsets matter: the pairing rule in section 3 walks begin/end tokens by
    position, so a stripper that shortened the text would desynchronise it from
    the match it was handed.
    """
    out = []
    i, n = 0, len(text)
    in_str = False
    while i < n:
        c = text[i]
        two = text[i:i + 2]
        if in_str:
            out.append(c)
            if c == '"':
                in_str = False
            i += 1
        elif c == '"':
            in_str = True
            out.append(c)
            i += 1
        elif two == "//":
            while i < n and text[i] != "\n":
                out.append(" ")
                i += 1
        elif two == "/*":
            while i < n and text[i:i + 2] != "*/":
                out.append("\n" if text[i] == "\n" else " ")
                i += 1
            out.append("  ")
            i += 2
        else:
            out.append(c)
            i += 1
    return "".join(out)


def squash(text):
    """Collapse every whitespace run to one space, so shape can be compared."""
    return re.sub(r"\s+", " ", text).strip()


def match_paren(text, open_idx):
    depth = 0
    for k in range(open_idx, len(text)):
        if text[k] == "(":
            depth += 1
        elif text[k] == ")":
            depth -= 1
            if depth == 0:
                return k
    raise ValueError("unbalanced parentheses at %d" % open_idx)


def find_instance(text, module, inst=None):
    """(param_text or None, instance_name, port_text) for one instantiation.

    `inst` picks which one when a module is instantiated several times -- the
    top has two uart_screen_ctrl instances since the Type-C command source was
    added, and a checker that silently inspects whichever one comes first in
    the file is the kind of drift this tool exists to catch.
    """
    for m in re.finditer(r"\b%s\b\s*" % re.escape(module), text):
        i = m.end()
        params = None
        head = re.match(r"#\s*\(", text[i:])
        if head:
            open_idx = i + head.end() - 1
            close = match_paren(text, open_idx)
            params = text[open_idx + 1:close]
            i = close + 1
        tail = re.match(r"\s*(\w+)\s*\(", text[i:])
        if not tail:
            continue
        name = tail.group(1)
        if inst is not None and name != inst:
            continue
        open_idx = i + tail.end() - 1
        close = match_paren(text, open_idx)
        return params, name, text[open_idx + 1:close]
    return None, None, None


def connections(text):
    pairs = re.findall(r"\.\s*(\w+)\s*\(([^)]*)\)", text or "")
    seen, dupes = {}, []
    for port, sig in pairs:
        if port in seen:
            dupes.append(port)
        seen[port] = sig.strip()
    return seen, dupes


def module_ports(text, module):
    """{name: (direction, width)} from a module header."""
    m = re.search(r"module\s+%s\b.*?\)\s*\(" % re.escape(module), text, flags=re.S)
    if not m:
        return {}
    close = match_paren(text, m.end() - 1)
    body = text[m.end():close]
    ports = {}
    for direction, msb, lsb, name in re.findall(
            r"\b(input|output)\s+(?:wire|reg)?\s*"
            r"(?:\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*)?(\w+)", body):
        width = 1 if not msb else int(msb) - int(lsb) + 1
        ports[name] = (direction, width)
    return ports


def enclosing_block(code, idx):
    """(start, stop) of the begin/end block enclosing idx, or None."""
    stack = []
    for m in re.finditer(r"\b(begin|end)\b", code[:idx]):
        if m.group(1) == "begin":
            stack.append(m.start())
        elif stack:
            stack.pop()
    if not stack:
        return None
    start = stack[-1]
    depth = 0
    for m in re.finditer(r"\b(begin|end)\b", code[start:]):
        if m.group(1) == "begin":
            depth += 1
        else:
            depth -= 1
            if depth == 0:
                return start, start + m.end()
    return None


# ---------------------------------------------------------------------------
# The shapes this feature has to have. Built from sim_uart_ctrl's own accepted
# range so the checker and the model cannot drift apart about what SPED takes.
# ---------------------------------------------------------------------------
LO, HI = su.SPEED_DIGITS[0], su.SPEED_DIGITS[-1]

SPED_ARM = ('"SPED": if (clen == 4\'d6 && c4 == " " && c5 >= "%s" && c5 <= "%s") '
            "begin cmd_speed <= c5 - 8'h30; cmd_speed_set <= 1'b1; end" % (LO, HI))

TOP_PARAM = "parameter AUTO_SEC_DEFAULT = 3'd1;"
TOP_TOGGLE = ("if (cmd_speed_set) begin speed_lat <= cmd_speed; "
              "spd_tgl <= ~spd_tgl; end")
TOP_SYNC = ("spd_tgl_s0 <= spd_tgl; spd_tgl_s1 <= spd_tgl_s0; "
            "spd_tgl_s2 <= spd_tgl_s1; speed_s0 <= speed_lat; "
            "speed_s1 <= speed_s0;")
TOP_PULSE = "wire cmd_speed_pulse_sd = spd_tgl_s1 ^ spd_tgl_s2;"

# The single merge point, SPED's arm of it. Strobe may be OR-ed, value may not.
# Since the emergency mode, the merged *strobe* also carries the freeze level:
# every consumer writes its latch only on a valid strobe, so gating the strobe
# alone is enough to make an interval change during a takeover a no-op. The
# freeze check pins the parentheses too -- & binds tighter than |, so without
# them only pc_speed_set would be frozen and J1 would keep driving the carousel.
MERGE_STROBE = re.compile(r"j1_cmd_speed_set\s*\|\s*pc_speed_set")
MERGE_FREEZE = re.compile(r"^\(.*\)\s*&\s*~emg_hold$")
MERGE_VALUE = re.compile(r"j1_cmd_speed_set\s*\?\s*j1_cmd_speed"
                         r"\s*:\s*pc_speed_v$")
PC_SPEED_GATE = re.compile(r"PC_CMD_ENABLE\s*\?\s*pc_cmd_speed\s*:\s*4'd0$")
TOP_RST_CLK = "spd_tgl <= 1'b0; speed_lat <= AUTO_SEC_DEFAULT;"
TOP_RST_SD = ("spd_tgl_s0 <= 1'b0; spd_tgl_s1 <= 1'b0; spd_tgl_s2 <= 1'b0; "
              "speed_s0 <= AUTO_SEC_DEFAULT; speed_s1 <= AUTO_SEC_DEFAULT;")

SD_PARAM = "parameter [2:0] AUTO_SEC_DEFAULT = 3'd1"
AUTO_TICK = "assign auto_tick = (auto_cnt == (CLK_FREQ_HZ - 1));"
SEC_LAST = "assign sec_last = (sec_cnt == sec_target_m1);"
AUTO_BLOCK = (
    "if (auto_play_en && first_image_committed && (img_loaded_count > 3'd1)) "
    "begin if (auto_tick) begin auto_cnt <= 32'd0; if (sec_last) begin "
    "sec_cnt <= 3'd0; img_idx <= next_from_loaded; "
    "disp_buf_idx <= next_from_loaded; end else begin "
    "sec_cnt <= sec_cnt + 3'd1; end end else begin "
    "auto_cnt <= auto_cnt + 32'd1; end end else begin auto_cnt <= 32'd0; "
    "sec_cnt <= 3'd0; end")
SPED_SD = ("if (cmd_speed_pulse) begin sec_target_m1 <= cmd_speed[2:0] - 3'd1; "
           "sec_cnt <= 3'd0; end")
SD_RST = "sec_target_m1 <= AUTO_SEC_DEFAULT - 3'd1"


# ---------------------------------------------------------------------------
# 1. uart_screen_ctrl.v
# ---------------------------------------------------------------------------
def check_uart(src):
    print("\n[1] uart_screen_ctrl.v -- parse SPED, emit value + strobe")
    code = strip_comments(src.uart)
    flat = squash(code)
    ports = module_ports(code, "uart_screen_ctrl")

    check(ports.get("cmd_speed") == ("output", 4),
          "1.1 cmd_speed is a 4-bit output reg (1..8 does not fit in 3 bits)",
          "got %r" % (ports.get("cmd_speed"),))
    check(ports.get("cmd_speed_set") == ("output", 1),
          "1.2 cmd_speed_set is a 1-bit output reg",
          "got %r" % (ports.get("cmd_speed_set"),))
    check(SPED_ARM in flat,
          "1.3 the SPED arm guards clen==6, a space and '%s'..'%s', and emits "
          "value + strobe" % (LO, HI),
          "arm not found verbatim")
    # Twice, and only twice: once in the async reset, once in the per-cycle
    # default that makes it a one-clock pulse. Losing the default would leave
    # the strobe high for as long as the frame sits in the registers, and the
    # toggle in the top level would flip once per sd_card_clk cycle.
    check(flat.count("cmd_speed_set <= 1'b0;") == 2,
          "1.4 cmd_speed_set is cleared both in reset and in the per-cycle "
          "default (one-clock strobe)",
          "count=%d" % flat.count("cmd_speed_set <= 1'b0;"))
    check("cmd_speed <= 4'd0;" in flat,
          "1.5 cmd_speed resets to 0")


# ---------------------------------------------------------------------------
# 2. top_tf_hdmi_audio.v
# ---------------------------------------------------------------------------
def check_top(src):
    print("\n[2] top_tf_hdmi_audio.v -- retreat parameter, data+toggle CDC, wiring")
    code = strip_comments(src.top)
    flat = squash(code)

    check(TOP_PARAM in flat,
          "2.1 AUTO_SEC_DEFAULT is declared as 3'd1 -- the one-line retreat")

    j1_params, j1_name, j1_ports = find_instance(code, "uart_screen_ctrl",
                                                  "u_uart_screen_ctrl")
    pc_params, pc_name, pc_ports = find_instance(code, "uart_screen_ctrl",
                                                  "u_uart_pc_cmd")
    check(j1_name == "u_uart_screen_ctrl" and pc_name == "u_uart_pc_cmd",
          "2.2 the top instantiates uart_screen_ctrl twice -- a J1 parser and a "
          "Type-C parser; the second command source is a second instance, not a "
          "shared parser",
          "got %r / %r" % (j1_name, pc_name))

    j1conn, j1dupes = connections(j1_ports)
    pcconn, pcdupes = connections(pc_ports)
    check(not j1dupes and not pcdupes
          and j1conn.get("cmd_speed") == "j1_cmd_speed"
          and j1conn.get("cmd_speed_set") == "j1_cmd_speed_set"
          and pcconn.get("cmd_speed") == "pc_cmd_speed"
          and pcconn.get("cmd_speed_set") == "pc_cmd_speed_set",
          "2.3 each parser drives its own j1_cmd_speed* / pc_cmd_speed* nets -- "
          "no net name is shared, so neither instance can quietly drive the "
          "other's strobe",
          "j1=%r/%r dupes=%r pc=%r/%r dupes=%r"
          % (j1conn.get("cmd_speed"), j1conn.get("cmd_speed_set"), j1dupes,
             pcconn.get("cmd_speed"), pcconn.get("cmd_speed_set"), pcdupes))
    check(j1conn.get("uart_rx") == "uart_rx"
          and pcconn.get("uart_rx") == "uart_pc_rx"
          and pcconn.get("uart_tx", "").strip() == "",
          "2.4 the J1 parser listens on uart_rx (D14), the Type-C one on "
          "uart_pc_rx (F12) with uart_tx left unconnected -- the two RX sources "
          "never share a pin",
          "j1=%r pc_rx=%r pc_tx=%r"
          % (j1conn.get("uart_rx"), pcconn.get("uart_rx"),
             pcconn.get("uart_tx")))

    check(TOP_TOGGLE in flat,
          "2.5 clk domain latches speed_lat from cmd_speed in the same block "
          "that flips spd_tgl (data+toggle: the value must be stable before the "
          "edge that samples it)")

    check(TOP_SYNC in flat,
          "2.6 sd_card_clk domain runs the 3FF toggle chain and the 2FF value "
          "chain")
    check(TOP_PULSE in flat,
          "2.7 cmd_speed_pulse_sd is spd_tgl_s1 ^ spd_tgl_s2")

    check(TOP_RST_CLK in flat,
          "2.8 clk-domain reset: spd_tgl low, speed_lat at AUTO_SEC_DEFAULT")
    check(TOP_RST_SD in flat,
          "2.9 sd-domain reset: the whole sync chain low and both value stages "
          "at AUTO_SEC_DEFAULT, so no pulse and no wrong interval come out of "
          "reset")

    m = re.search(r"wire\s+cmd_any_set\s*=\s*([^;]+);", flat)
    check(m is not None and "cmd_speed_set" in m.group(1),
          "2.10 cmd_speed_set is in cmd_any_set, so LED2 still reports that a "
          "frame was accepted",
          "cmd_any_set=%r" % (m.group(1) if m else None,))

    strobe_line = re.search(r"assign cmd_speed_set\s*=\s*([^;]+);", flat)
    check(strobe_line is not None and MERGE_STROBE.search(strobe_line.group(1)),
          "2.11 the two SPED strobes are OR-ed at the single merge point -- a "
          "strobe carries no information to corrupt, so both sources may share "
          "one edge",
          "cmd_speed_set = %r" % (strobe_line.group(1) if strobe_line else None,))
    check(strobe_line is not None and MERGE_FREEZE.match(strobe_line.group(1)),
          "2.12 the merged SPED strobe is AND-ed with ~emg_hold -- the whole "
          "parenthesised OR, not one term, since & binds tighter than | -- so an "
          "interval change that arrives during a takeover cannot restart the "
          "carousel timer behind a screen that is not allowed to move; the freeze "
          "rides the strobe and not the value bus because every consumer writes "
          "its latch only on a valid strobe",
          "cmd_speed_set = %r" % (strobe_line.group(1) if strobe_line else None,))
    value_line = re.search(r"assign cmd_speed\s*=\s*([^;]+);", flat)
    check(value_line is not None and MERGE_VALUE.match(value_line.group(1)),
          "2.13 the two SPED values are a J1-first mux, NOT an OR -- MODE 1 "
          "colliding with MODE E under an OR produces 0xF, a mode nobody sent, "
          "and for SPED it would silently pick an interval no one asked for",
          "cmd_speed = %r" % (value_line.group(1) if value_line else None,))
    gate = re.search(r"pc_speed_v\s*=\s*([^;]+);", flat)
    check(gate is not None and PC_SPEED_GATE.match(gate.group(1)),
          "2.14 PC_CMD_ENABLE gates the SPED *value* bus as well as its strobe; "
          "gating only the strobe leaves cmd_speed reading pc_cmd_speed and the "
          "parser survives synthesis with a live load, which is what makes the "
          "one-line retreat real",
          "pc_speed_v = %r" % (gate.group(1) if gate else None,))

    sparams, sname, sports = find_instance(code, "sd_card_bmp")
    sconn, sdupes = connections(sports)
    pconn, pdupes = connections(sparams)
    check(not sdupes and sconn.get("cmd_speed") == "speed_s1",
          "2.15 sd_card_bmp_m0 takes .cmd_speed from the synchronised speed_s1, "
          "not from the clk-domain wire",
          "got %r dupes=%r" % (sconn.get("cmd_speed"), sdupes))
    check(sconn.get("cmd_speed_pulse") == "cmd_speed_pulse_sd",
          "2.16 sd_card_bmp_m0 takes .cmd_speed_pulse from the reconstructed "
          "pulse",
          "got %r" % (sconn.get("cmd_speed_pulse"),))
    check(not pdupes and pconn.get("AUTO_SEC_DEFAULT") == "AUTO_SEC_DEFAULT",
          "2.17 sd_card_bmp_m0 is handed AUTO_SEC_DEFAULT, so both domains "
          "agree on the interval before any SPED arrives",
          "got %r dupes=%r" % (pconn.get("AUTO_SEC_DEFAULT"), pdupes))


# ---------------------------------------------------------------------------
# 3. sd_card_bmp.v
# ---------------------------------------------------------------------------
def check_sd(src):
    print("\n[3] sd_card_bmp.v -- the interval counter and the two 1-second "
          "constants")
    code = strip_comments(src.sd)
    flat = squash(code)
    ports = module_ports(code, "sd_card_bmp")

    check(ports.get("cmd_speed") == ("input", 4),
          "3.1 cmd_speed is a 4-bit input",
          "got %r" % (ports.get("cmd_speed"),))
    check(ports.get("cmd_speed_pulse") == ("input", 1),
          "3.2 cmd_speed_pulse is a 1-bit input",
          "got %r" % (ports.get("cmd_speed_pulse"),))
    check(SD_PARAM in flat, "3.3 AUTO_SEC_DEFAULT is a 3-bit parameter = 1")
    check("reg [2:0] sec_cnt;" in flat and "reg [2:0] sec_target_m1;" in flat,
          "3.4 sec_cnt and sec_target_m1 are both 3 bits -- 8 seconds is the "
          "whole range, so neither needs to be wider")

    # The two constants. Both must still be spelled out, and there must still be
    # exactly two of them: a third would mean the interval leaked into the
    # watchdog, or the watchdog into the interval.
    ticks = re.findall(r"CLK_FREQ_HZ\s*-\s*1", code)
    check(AUTO_TICK in flat,
          "3.5 auto_tick is still a CONSTANT compare against CLK_FREQ_HZ-1 -- "
          "sd_card_clk is the tightest domain and a variable 32-bit compare "
          "here would cost the slack")
    check("load_stall_cnt >= CLK_FREQ_HZ - 1" in flat,
          "3.6 the picture-level stall watchdog still uses its own "
          "CLK_FREQ_HZ-1, independent of the carousel interval")
    check(len(ticks) == 2,
          "3.7 CLK_FREQ_HZ-1 appears exactly twice (auto_cnt and "
          "load_stall_cnt) -- the two seconds were not merged into one "
          "adjustable interval",
          "count=%d" % len(ticks))

    check(SEC_LAST in flat,
          "3.8 sec_last compares two registers and nothing else -- no "
          "subtractor, no constant, in the tick path")
    check(SPED_SD in flat,
          "3.9 the SPED handler precomputes target-1 and restarts sec_cnt")
    check(AUTO_BLOCK in flat,
          "3.10 the auto-play block advances only on auto_tick && sec_last, and "
          "auto_cnt still restarts on every tick")
    check(SD_RST in flat,
          "3.11 sec_target_m1 resets to AUTO_SEC_DEFAULT-1, so out of reset "
          "sec_last is constantly true and the advance condition is literally "
          "the bare auto_tick this design has always had")

    # The pairing rule. Nine places clear auto_cnt; each one is a "the carousel
    # timing restarts here" event, and each has to clear sec_cnt in the same
    # scope. Miss one and that site hands the next interval a part-elapsed
    # second -- the picture comes up early, once, and nothing in a rotation
    # test would show it.
    auto_sites = [m.start() for m in
                  re.finditer(r"auto_cnt\s*<=\s*32'd0\s*;", code)]
    check(len(auto_sites) >= 8,
          "3.12 found the auto_cnt clear sites to pair",
          "count=%d" % len(auto_sites))
    unpaired = []
    for idx in auto_sites:
        span = enclosing_block(code, idx)
        if span is None:
            unpaired.append("no enclosing begin/end")
            continue
        line = code[:idx].count("\n") + 1
        if "sec_cnt <= 3'd0;" not in squash(code[span[0]:span[1]]):
            unpaired.append("line %d" % line)
    check(not unpaired,
          "3.13 every auto_cnt <= 0 also clears sec_cnt in the same scope "
          "(SPED, NEXT, IMGX, auto-toggle, auto-off, arming, scan kick, "
          "sd_init, async reset)",
          "unpaired at %s" % ", ".join(unpaired))
    check(flat.count("sec_cnt <= 3'd0;") == len(auto_sites) + 1,
          "3.14 sec_cnt is cleared once per auto_cnt site plus once by SPED",
          "sec_cnt clears=%d auto_cnt clears=%d"
          % (flat.count("sec_cnt <= 3'd0;"), len(auto_sites)))


# ---------------------------------------------------------------------------
# 4. Negative controls
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
    except (SystemExit, ValueError):
        caught = True
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
    print("\n[4] negative controls: each mutation must be caught")

    bites(check_uart,
          src.clone(uart=sub_once(src.uart, r'c5 <= "8"', 'c5 <= "9"',
                                  "SPED upper bound")),
          "C1 accepting SPED 9 -- an interval past the 8 s the range allows")
    bites(check_uart,
          src.clone(uart=sub_once(src.uart, r'(?s)("SPED":.*?c5 >= )"1"',
                                  r'\1"0"', "SPED lower bound")),
          "C2 accepting SPED 0 -- the 3-bit speed-1 target wraps 0 to 7, so it "
          "would silently alias onto the slowest 8 s interval")
    bites(check_uart,
          src.clone(uart=sub_once(src.uart, r"cmd_speed\s+<= c5 - 8'h30;",
                                  "cmd_speed <= c5;", "SPED ASCII offset")),
          "C3 forgetting the ASCII offset, so SPED 4 asks for 52 seconds")
    bites(check_uart,
          src.clone(uart=sub_once(src.uart,
                                  r"(\n\s*)cmd_speed_set\s+<= 1'b0;", "",
                                  "one of the cmd_speed_set clears")),
          "C4 dropping one cmd_speed_set clear -- the strobe stops being a "
          "one-clock pulse")

    bites(check_top,
          src.clone(top=sub_once(src.top, r"speed_lat\s*<=\s*cmd_speed;", "",
                                 "the data latch")),
          "C5 flipping spd_tgl without latching the value -- data+toggle "
          "becomes toggle-only and the sd domain samples a stale interval")
    bites(check_top,
          src.clone(top=sub_once(src.top, r"spd_tgl_s2\s*<=\s*spd_tgl_s1;",
                                 "spd_tgl_s2 <= spd_tgl;",
                                 "the third sync flop")),
          "C6 shortening the toggle chain to 2FF -- s1^s2 stops being a "
          "single-cycle pulse")
    bites(check_top,
          src.clone(top=sub_once(src.top, r"speed_s0\s*<=\s*speed_lat;",
                                 "speed_s0 <= cmd_speed;",
                                 "the value synchroniser")),
          "C7 sampling the clk-domain wire straight into sd_card_clk")
    bites(check_top,
          src.clone(top=sub_once(src.top, r"\.cmd_speed\s*\(speed_s1\)",
                                 ".cmd_speed (cmd_speed)",
                                 "the sd_card_bmp value connection")),
          "C8 feeding sd_card_bmp the un-synchronised cmd_speed")
    bites(check_top,
          src.clone(top=sub_once(src.top,
                                 r"\.AUTO_SEC_DEFAULT\s*\(AUTO_SEC_DEFAULT\)",
                                 ".AUTO_SEC_DEFAULT (3'd4)",
                                 "the parameter pass-down")),
          "C9 hard-coding a different default in the pass-down, so the two "
          "domains disagree out of reset")

    bites(check_sd,
          src.clone(sd=sub_once(src.sd,
                                r"assign auto_tick\s*=\s*\(auto_cnt == \(CLK_FREQ_HZ - 1\)\);",
                                "assign auto_tick = (auto_cnt >= (CLK_FREQ_HZ - 1));",
                                "auto_tick")),
          "C10 rewriting auto_tick -- the constant compare is what keeps the "
          "32-bit path out of the tightest domain")
    bites(check_sd,
          src.clone(sd=sub_once(src.sd, r"load_stall_cnt >= CLK_FREQ_HZ - 1",
                                "load_stall_cnt >= sec_target_m1",
                                "the watchdog constant")),
          "C11 pointing the load watchdog at the adjustable interval -- an "
          "8 s carousel would also mean an 8 s dead-load detection")
    bites(check_sd,
          src.clone(sd=sub_once(src.sd,
                                r"assign sec_last\s*=\s*\(sec_cnt == sec_target_m1\);",
                                "assign sec_last = (sec_cnt + 3'd1 == cmd_speed[2:0]);",
                                "sec_last")),
          "C12 building sec_last out of a subtractor and a cross-domain value "
          "instead of two registers")
    bites(check_sd,
          src.clone(sd=sub_once(src.sd, r"sec_target_m1 <= cmd_speed\[2:0\] - 3'd1;",
                                "sec_target_m1 <= cmd_speed[2:0];",
                                "the target precompute")),
          "C13 storing the value instead of value-1 -- every interval one "
          "second long")
    bites(check_sd,
          src.clone(sd=sub_once(src.sd, r"if \(sec_last\) begin",
                                "if (1'b1) begin", "the sec_last gate")),
          "C14 removing the sec_last gate -- SPED is accepted and ignored")

    # The pairing rule, attacked at the two sites a rotation test is least
    # likely to notice: IMGX and the auto-off else branch. Each drops the
    # sec_cnt clear that sits beside an auto_cnt clear, leaving the next
    # interval a part-elapsed second short.
    bites(check_sd,
          src.clone(sd=sub_once(
              src.sd,
              r"(disp_buf_idx\s*<=\s*cmd_img_sel;\s*\n\s*"
              r"auto_cnt\s*<=\s*32'd0;)\s*\n\s*sec_cnt\s*<=\s*3'd0;",
              r"\1",
              "the sec_cnt clear at the IMGX site")),
          "C15 dropping the sec_cnt clear at the IMGX site")
    bites(check_sd,
          src.clone(sd=sub_once(
              src.sd,
              r"(end else begin\s*\n\s*auto_cnt\s*<=\s*32'd0;)\s*\n\s*"
              r"sec_cnt\s*<=\s*3'd0;",
              r"\1",
              "the sec_cnt clear in the auto-off else branch")),
          "C16 dropping the sec_cnt clear in the auto-off else branch")

    # The merge point, attacked on all four halves. These are the checks that
    # keep a second command source from becoming a second bug surface.
    bites(check_top,
          src.clone(top=sub_once(
              src.top,
              r"cmd_speed\s*=\s*j1_cmd_speed_set\s*\?\s*j1_cmd_speed"
              r"\s*:\s*pc_speed_v;",
              "cmd_speed = j1_cmd_speed | pc_speed_v;",
              "the SPED value mux")),
          "C17 merging the SPED *value* by OR -- the same collision that turns "
          "MODE 1 vs MODE E into 0xF turns 4 s vs 8 s into 12 s truncated to a "
          "4 s interval nobody asked for")
    bites(check_top,
          src.clone(top=sub_once(
              src.top,
              r"cmd_speed_set\s*=\s*\(\s*j1_cmd_speed_set\s*\|\s*pc_speed_set"
              r"\s*\)\s*&\s*~emg_hold;",
              "cmd_speed_set = (j1_cmd_speed_set) & ~emg_hold;",
              "the SPED strobe OR")),
          "C18 dropping the PC strobe from the merge -- Type-C SPED is parsed, "
          "counted on LED2, and never reaches sd_card_bmp")
    bites(check_top,
          src.clone(top=sub_once(
              src.top,
              r"cmd_speed_set\s*=\s*\(\s*j1_cmd_speed_set\s*\|\s*pc_speed_set"
              r"\s*\)\s*&\s*~emg_hold;",
              "cmd_speed_set = j1_cmd_speed_set | pc_speed_set;",
              "the SPED strobe freeze")),
          "C19 dropping ~emg_hold from the SPED strobe -- the merge point goes "
          "back to the two-source shape, so a SPED that arrives during a "
          "takeover restarts the carousel timer under a frozen screen and the "
          "first frame after the release is early by an unknown amount")
    bites(check_top,
          src.clone(top=sub_once(
              src.top,
              r"cmd_speed_set\s*=\s*\(\s*j1_cmd_speed_set\s*\|\s*pc_speed_set"
              r"\s*\)\s*&\s*~emg_hold;",
              "cmd_speed_set = j1_cmd_speed_set | pc_speed_set & ~emg_hold;",
              "the parentheses around the merged strobes")),
          "C20 freezing only the PC strobe -- & outranks |, so J1 keeps driving "
          "the carousel timer through a takeover and the OR half still looks fine")
    bites(check_top,
          src.clone(top=sub_once(
              src.top,
              r"pc_speed_v\s*=\s*PC_CMD_ENABLE\s*\?\s*pc_cmd_speed\s*:\s*4'd0;",
              "pc_speed_v = pc_cmd_speed;",
              "the PC_CMD_ENABLE gate on the value bus")),
          "C21 gating only the strobe -- cmd_speed still reads pc_cmd_speed "
          "unconditionally, so the parser keeps a live load and PC_CMD_ENABLE=0 "
          "stops deleting the instance")


def main():
    src = Sources()
    print("=" * 72)
    print("check_uart_speed_transcription -- SPED n, adjustable carousel interval")
    print("=" * 72)
    check_uart(src)
    check_top(src)
    check_sd(src)
    check_controls(src)

    print("\n" + "=" * 72)
    if FAILURES:
        print("FAILED  %d/%d checks" % (len(FAILURES), CHECKS[0]))
        for f in FAILURES:
            print("  - %s" % f)
        return 1
    print("all %d checks passed" % CHECKS[0])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
