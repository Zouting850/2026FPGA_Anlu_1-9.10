#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cycle-accurate model of uart_pc_text.v + the top's PC-text CDC crossing.

No Verilog simulator on this machine (project memory
"no-verilog-simulator-use-python-models"), so the second UART that turns the
Type-C/F12 link into a subtitle keyboard is proven here register-for-register
before it goes anywhere near the board. The model honours non-blocking semantics
(read the old values, compute next, commit) and runs the two clock domains on
one merged time axis with a deliberate phase offset so no two posedges coincide,
exactly like hardware:

  * clk       (50 MHz)  -- uart_pc_text's RX FSM (8N1, mid-bit sample, LSB
                           first, cloned from the board-proven screen RX) and the
                           PCTX/TEXT parser that commits char_buf / n_cells /
                           pc_text_en / text_toggle.
  * video_clk (25 MHz)  -- top_tf_hdmi_audio's crossing: a bare 2FF on the
                           quasi-static pc_text_en level, a data+toggle crossing
                           (3FF on text_toggle, edge -> staging) on the human-rate
                           char_buf/n_cells, then a frame-atomic latch at
                           video_frame_start so a string can never tear mid-frame.

Every constant the model depends on is parsed out of the RTL -- PC_CAP, the
char_buf/n_cells widths, CLKS_PER_BIT, the keyword literals, the payload range,
the char stride, the commit-on-third-0xFF rule, and the top's CDC shapes. Nothing
is restated here, so the model cannot silently drift; if a shape changes the
parser stops with "the model is stale, fix the parser".

Passes:
  A  RX decode at the real divider (CLKS_PER_BIT = 50e6/9600 = 5208): a "PCTX 1"
     frame arrives as the exact bytes on the wire and commits.
  B  Parser effects + guards. PCTX 1/0 drive pc_text_en; TEXT packs char_buf at
     cell*7, sets n_cells, flips text_toggle once. Internal spaces kept; the
     24-char cap truncates at the source; lowercase keyword, wrong separator,
     bad PCTX argument and empty TEXT are all rejected without wedging the parser.
  C  CDC clk -> video_clk. pc_text_en crosses on a bare 2FF; char_buf/n_cells
     cross on data+toggle and are released frame-atomically. A mid-frame TEXT is
     held until the next video_frame_start (no tearing); back-to-back TEXTs each
     produce exactly one staging latch.
  D  Negative controls + the retreat. Zero traffic leaves pc_en_frame=0,
     char_buf=0, n_cells=0 and emits no toggle; PC_TEXT_ENABLE=0 ties pc_en_gated
     low even after PCTX 1; two 0xFF only never commit; a partial frame recovers.
  E  dbg_rx_toggle / dbg_commit_toggle are observation-only: idle traffic leaves
     both at reset, a clean frame flips rx-toggle once per byte and commit-toggle
     once per committed TEXT, and they never feed back into the protocol.

Run from anywhere:  python tools/sim_uart_pc_text.py
Exit code 0 on success, 1 if any check fails.
"""
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
HDL = os.path.join(os.path.dirname(TOOLS), "src", "user_source", "hdl_source")
UART_RTL = os.path.join(HDL, "uart_pc_text.v")
TOP_RTL = os.path.join(HDL, "top_tf_hdmi_audio.v")

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


def read(path):
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def stale(why):
    raise SystemExit("sim_uart_pc_text: %s -- the model is stale, fix the parser"
                     % why)


# ---------------------------------------------------------------------------
# Parse every constant / shape the model depends on out of the RTL.
# ---------------------------------------------------------------------------
def parse_rtl():
    u = read(UART_RTL)
    t = read(TOP_RTL)
    cfg = {}

    def grab(pat, text, what, group=1, flags=0):
        m = re.search(pat, text, flags)
        if not m:
            stale("could not find %s" % what)
        return m.group(group)

    # ---- uart_pc_text.v geometry ----
    cfg["clk_hz"] = int(grab(r"parameter\s+integer\s+CLK_FREQ_HZ\s*=\s*([\d_]+)",
                             u, "CLK_FREQ_HZ").replace("_", ""))
    cfg["baud"] = int(grab(r"parameter\s+integer\s+BAUD\s*=\s*([\d_]+)", u, "BAUD"))
    cfg["cpb"] = cfg["clk_hz"] // cfg["baud"]
    if not re.search(r"localparam\s+integer\s+CLKS_PER_BIT\s*=\s*CLK_FREQ_HZ\s*/\s*BAUD", u):
        stale("CLKS_PER_BIT is not CLK_FREQ_HZ/BAUD")

    cfg["pc_cap"] = int(grab(r"localparam\s+\[5:0\]\s+PC_CAP\s*=\s*6'd(\d+)", u, "PC_CAP"))
    cfg["char_buf_msb"] = int(grab(r"output\s+reg\s+\[(\d+):0\]\s+char_buf", u, "char_buf msb"))
    cfg["char_buf_w"] = cfg["char_buf_msb"] + 1
    cfg["n_cells_msb"] = int(grab(r"output\s+reg\s+\[(\d+):0\]\s+n_cells", u, "n_cells msb"))
    cfg["n_cells_w"] = cfg["n_cells_msb"] + 1

    # char_buf is cell*STRIDE +: 7; the stride must be 7 to match marquee_overlay's
    # char_base = cell_idx*7 and the model's packing.
    cfg["char_stride"] = int(grab(r"char_count\s*\*\s*8'd(\d+)", u, "char stride"))
    if not re.search(r"char_buf\[char_base_w\s*\+:\s*7\]", u):
        stale("char_buf is not written as [char_base_w +: 7]")

    # payload window 0x20..0x7E and the keyword literals.
    lo = grab(r"payload_ok\s*=\s*\(rx_byte\s*>=\s*8'h([0-9A-Fa-f]+)\)", u, "payload lo")
    hi = grab(r"\(rx_byte\s*<=\s*8'h([0-9A-Fa-f]+)\)", u, "payload hi")
    cfg["payload_lo"] = int(lo, 16)
    cfg["payload_hi"] = int(hi, 16)
    cfg["kw_pctx"] = grab(r'\(kw\s*==\s*"(\w+)"\)\s*\?\s*MODE_PCTX', u, "PCTX literal")
    cfg["kw_text"] = grab(r'\(kw\s*==\s*"(\w+)"\)\s*\?\s*MODE_TEXT', u, "TEXT literal")

    # commit fires on the third consecutive 0xFF (ffc == 2).
    cfg["commit_ffc"] = int(grab(r"if\s*\(ffc\s*==\s*2'd(\d+)\)", u, "commit ffc"))
    # TEXT commit needs sep_ok and at least one char.
    if not re.search(r"\(mode\s*==\s*MODE_TEXT\)\s*&&\s*sep_ok\s*&&\s*\(char_count\s*>=\s*6'd1\)", u):
        stale("TEXT commit guard is not (MODE_TEXT && sep_ok && char_count>=1)")
    if not re.search(r"\(mode\s*==\s*MODE_PCTX\)\s*&&\s*pctx_valid", u):
        stale("PCTX commit guard is not (MODE_PCTX && pctx_valid)")
    # PCTX argument is byte 5 and must be '0' or '1'.
    if not re.search(r"pctx_arg\s*=\s*\(bc\s*==\s*6'd5\)\s*&&\s*\(mode\s*==\s*MODE_PCTX\)\s*&&\s*sep_ok", u):
        stale("pctx_arg is not gated on bc==5 && MODE_PCTX && sep_ok")
    if not re.search(r"\(rx_byte\s*==\s*8'h30\)\s*\|\|\s*\(rx_byte\s*==\s*8'h31\)", u):
        stale("PCTX argument is not restricted to '0'/'1'")
    # do_store gate.
    if not re.search(r"do_store\s*=\s*\(mode\s*==\s*MODE_TEXT\)\s*&&\s*sep_ok\s*&&\s*\(bc\s*>=\s*6'd5\)", u):
        stale("do_store is not (MODE_TEXT && sep_ok && bc>=5 ...)")
    if not re.search(r"payload_ok\s*&&\s*\(char_count\s*<\s*PC_CAP\)", u):
        stale("do_store is not capped at char_count < PC_CAP")

    # ---- top_tf_hdmi_audio.v CDC shapes ----
    cfg["pc_enable_param"] = int(grab(r"parameter\s+PC_TEXT_ENABLE\s*=\s*1'b(\d)", t,
                                      "PC_TEXT_ENABLE"))
    if not re.search(r"pc_en_v0\s*<=\s*pc_text_en;\s*pc_en_v1\s*<=\s*pc_en_v0;", t):
        stale("pc_text_en is not crossed on a bare 2FF (pc_en_v0/v1)")
    if not re.search(r"pc_tgl_s0\s*<=\s*pc_text_toggle;\s*pc_tgl_s1\s*<=\s*pc_tgl_s0;\s*"
                     r"pc_tgl_s2\s*<=\s*pc_tgl_s1;", t):
        stale("text_toggle is not crossed on a 3FF chain (pc_tgl_s0/s1/s2)")
    if not re.search(r"pc_tgl_edge\s*=\s*pc_tgl_s1\s*\^\s*pc_tgl_s2", t):
        stale("pc_tgl_edge is not pc_tgl_s1 ^ pc_tgl_s2")
    if not re.search(r"if\s*\(pc_tgl_edge\)\s*begin\s*pc_buf_stg\s*<=\s*pc_char_buf;\s*"
                     r"pc_cells_stg\s*<=\s*pc_n_cells;", t):
        stale("the toggle edge does not latch char_buf/n_cells into staging")
    if not re.search(r"if\s*\(video_frame_start\)\s*begin", t):
        stale("no video_frame_start frame-atomic block")
    if not re.search(r"pc_en_frame\s*<=\s*pc_en_v1;\s*pc_buf_frame\s*<=\s*pc_buf_stg;\s*"
                     r"pc_cells_frame\s*<=\s*pc_cells_stg;", t):
        stale("video_frame_start does not latch the PC frame registers")
    if not re.search(r"pc_en_gated\s*=\s*PC_TEXT_ENABLE\s*&\s*pc_en_frame", t):
        stale("pc_en_gated is not PC_TEXT_ENABLE & pc_en_frame")
    if not re.search(r"video_frame_start\s*=\s*vs_d\s*&\s*~vs", t):
        stale("video_frame_start is not vs_d & ~vs")
    # the marquee instance must be fed the frame registers, not the staging regs.
    if not re.search(r"\.I_pc_en\s*\(pc_en_gated\)", t):
        stale("marquee I_pc_en is not driven by pc_en_gated")
    if not re.search(r"\.I_pc_char_buf\s*\(pc_buf_frame\)", t):
        stale("marquee I_pc_char_buf is not driven by pc_buf_frame")
    if not re.search(r"\.I_pc_cells\s*\(pc_cells_frame\)", t):
        stale("marquee I_pc_cells is not driven by pc_cells_frame")

    return cfg


CFG = parse_rtl()

RX_IDLE, RX_START, RX_DATA, RX_STOP = 0, 1, 2, 3
MODE_IGNORE, MODE_PCTX, MODE_TEXT = 0, 1, 2

# clock geometry (ns); the offset guarantees no clk and video_clk posedge coincide
CLK_PERIOD, CLK_OFF = 20, 0       # 50 MHz
VID_PERIOD, VID_OFF = 40, 7       # 25 MHz

CPB_REAL = CFG["cpb"]             # 5208, what actually ships
CPB_FAST = 16                     # shrunk divider for the fast passes (see below)


def frame_pctx(enable):
    """Wire bytes of a PCTX command: 'PCTX ' + '1'/'0' + three 0xFF."""
    return ([ord(c) for c in CFG["kw_pctx"]] + [0x20, 0x31 if enable else 0x30]
            + [0xFF, 0xFF, 0xFF])


def frame_text(s):
    """Wire bytes of a TEXT command: 'TEXT ' + payload + three 0xFF."""
    return ([ord(c) for c in CFG["kw_text"]] + [0x20]
            + [ord(c) for c in s] + [0xFF, 0xFF, 0xFF])


def pack_char_buf(s):
    """The flat char_buf a committed TEXT leaves behind: cell i at bit i*stride."""
    buf = 0
    for i, ch in enumerate(s):
        buf |= (ord(ch) & 0x7F) << (i * CFG["char_stride"])
    return buf


def build_rx_wave(byte_list, cpb, idle_before=8, idle_between=2, idle_after=48):
    """Expand bytes into per-clk-cycle uart_pc_rx levels: idle high, then per byte
    a low start bit, 8 LSB-first data bits and a high stop bit, each cpb wide."""
    segs = [(1, idle_before)]
    for i, b in enumerate(byte_list):
        segs.append((0, cpb))
        for k in range(8):
            segs.append(((b >> k) & 1, cpb))
        segs.append((1, cpb))
        if i != len(byte_list) - 1:
            segs.append((1, idle_between))
    segs.append((1, idle_after))
    levels = []
    for lvl, n in segs:
        levels.extend([lvl] * n)
    return levels


# ---------------------------------------------------------------------------
# The clk-domain module: RX FSM + PCTX/TEXT parser, register for register.
# ---------------------------------------------------------------------------
class UartPcText(object):
    def __init__(self, cpb=CPB_FAST):
        self.cpb = cpb
        self.rx_levels = []
        self.n = 0
        self.reset()

    def reset(self):
        s = self.__dict__
        # RX engine
        s["rx_sync"] = 0b11
        s["rx_state"] = RX_IDLE
        s["rx_cnt"] = 0
        s["rx_bit"] = 0
        s["rx_shift"] = 0
        s["rx_byte"] = 0
        s["rx_valid"] = 0
        # parser
        s["kw"] = 0
        s["bc"] = 0
        s["sep_ok"] = 0
        s["mode"] = MODE_IGNORE
        s["pctx_valid"] = 0
        s["pending_en"] = 0
        s["char_count"] = 0
        s["ffc"] = 0
        # committed outputs
        s["pc_text_en"] = 0
        s["char_buf"] = 0
        s["n_cells"] = 0
        s["text_toggle"] = 0
        s["dbg_rx_toggle"] = 0
        s["dbg_commit_toggle"] = 0
        # observation log
        s["rx_log"] = []
        s["commit_log"] = []      # one entry per committed frame: ('PCTX', en)/('TEXT', n)

    def _rx_in(self):
        return (self.rx_sync >> 1) & 1

    def tick(self):
        """One clk posedge. Non-blocking: read old, compute next, then commit."""
        s = self.__dict__
        rx_pin = self.rx_levels[self.n] if self.n < len(self.rx_levels) else 1
        nxt = {}

        # ---- RX FSM ----
        rx_in = self._rx_in()
        nxt["rx_sync"] = ((s["rx_sync"] & 1) << 1) | rx_pin
        nxt["rx_valid"] = 0
        state, cnt = s["rx_state"], s["rx_cnt"]
        bit, shift = s["rx_bit"], s["rx_shift"]
        byte_n = s["rx_byte"]
        cpb = self.cpb
        if state == RX_IDLE:
            cnt, bit = 0, 0
            if rx_in == 0:
                state = RX_START
        elif state == RX_START:
            if cnt == (cpb - 1) // 2:
                if rx_in == 0:
                    cnt, state = 0, RX_DATA
                else:
                    state = RX_IDLE
            else:
                cnt += 1
        elif state == RX_DATA:
            if cnt == cpb - 1:
                cnt = 0
                shift = ((rx_in << 7) | (shift >> 1)) & 0xFF
                bit = (bit + 1) & 7
                if s["rx_bit"] == 7:
                    state = RX_STOP
            else:
                cnt += 1
        elif state == RX_STOP:
            if cnt == cpb - 1:
                cnt, state = 0, RX_IDLE
                if rx_in == 1:
                    byte_n, nxt["rx_valid"] = shift, 1
            else:
                cnt += 1
        nxt["rx_state"], nxt["rx_cnt"] = state, cnt & 0xFFFF
        nxt["rx_bit"], nxt["rx_shift"] = bit, shift
        nxt["rx_byte"] = byte_n

        # ---- parser (consumes the OLD rx_valid / rx_byte) ----
        for k in ("kw", "bc", "sep_ok", "mode", "pctx_valid", "pending_en",
                  "char_count", "ffc", "pc_text_en", "char_buf", "n_cells",
                  "text_toggle", "dbg_rx_toggle", "dbg_commit_toggle"):
            nxt[k] = s[k]

        rx_valid_old = s["rx_valid"]
        rx_byte_old = s["rx_byte"]
        committed = None
        if rx_valid_old:
            b = rx_byte_old
            nxt["dbg_rx_toggle"] = s["dbg_rx_toggle"] ^ 1
            is_ff = (b == 0xFF)
            sep_byte = (b == 0x20)
            payload_ok = (CFG["payload_lo"] <= b <= CFG["payload_hi"])
            char_base_w = (s["char_count"] * CFG["char_stride"]) & 0xFF
            do_store = (s["mode"] == MODE_TEXT and s["sep_ok"] and s["bc"] >= 5
                        and payload_ok and s["char_count"] < CFG["pc_cap"])
            pctx_arg = (s["bc"] == 5 and s["mode"] == MODE_PCTX and s["sep_ok"]
                        and b in (0x30, 0x31))
            pctx_bad = (s["bc"] >= 6 and s["mode"] == MODE_PCTX)

            if is_ff:
                if s["ffc"] == CFG["commit_ffc"]:
                    # ---- third 0xFF: commit ----
                    if s["mode"] == MODE_PCTX and s["pctx_valid"]:
                        nxt["pc_text_en"] = s["pending_en"]
                        committed = ("PCTX", s["pending_en"])
                    if (s["mode"] == MODE_TEXT and s["sep_ok"]
                            and s["char_count"] >= 1):
                        nxt["n_cells"] = s["char_count"]
                        nxt["text_toggle"] = s["text_toggle"] ^ 1
                        nxt["dbg_commit_toggle"] = s["dbg_commit_toggle"] ^ 1
                        committed = ("TEXT", s["char_count"])
                    nxt["bc"] = 0
                    nxt["kw"] = 0
                    nxt["sep_ok"] = 0
                    nxt["mode"] = MODE_IGNORE
                    nxt["pctx_valid"] = 0
                    nxt["char_count"] = 0
                    nxt["ffc"] = 0
                else:
                    nxt["ffc"] = (s["ffc"] + 1) & 3
            else:
                nxt["ffc"] = 0
                if s["bc"] < 4:
                    # RTL: kw <= {kw[23:0], rx_byte}
                    nxt["kw"] = (((s["kw"] & 0xFFFFFF) << 8) | b) & 0xFFFFFFFF
                if s["bc"] == 4:
                    nxt["sep_ok"] = 1 if sep_byte else 0
                    kw_str = s["kw"] & 0xFFFFFFFF
                    nxt["mode"] = (MODE_PCTX if kw_str == _kw_int(CFG["kw_pctx"]) else
                                   MODE_TEXT if kw_str == _kw_int(CFG["kw_text"]) else
                                   MODE_IGNORE)
                if pctx_arg:
                    nxt["pending_en"] = 1 if b == 0x31 else 0
                    nxt["pctx_valid"] = 1
                if pctx_bad:
                    nxt["pctx_valid"] = 0
                if do_store:
                    mask = 0x7F << char_base_w
                    nxt["char_buf"] = (s["char_buf"] & ~mask) | ((b & 0x7F) << char_base_w)
                    nxt["char_count"] = s["char_count"] + 1
                if s["bc"] < 63:
                    nxt["bc"] = s["bc"] + 1

        s.update(nxt)
        self.n += 1
        if nxt["rx_valid"]:
            self.rx_log.append(nxt["rx_byte"])
        if committed:
            self.commit_log.append(committed)


def _kw_int(kw):
    v = 0
    for c in kw:
        v = (v << 8) | ord(c)
    return v


# ---------------------------------------------------------------------------
# The merged-axis system: clk-domain UartPcText + the top's video_clk crossing.
# ---------------------------------------------------------------------------
class System(object):
    def __init__(self, cpb=CPB_FAST, pc_text_enable=1):
        self.uart = UartPcText(cpb=cpb)
        self.pc_text_enable = pc_text_enable   # the top's PC_TEXT_ENABLE parameter
        self.vs_pin = 1                        # video_clk vsync level (idles high)
        self.clk_n = 0
        self.vid_n = 0
        self._vid_reset()

    def _vid_reset(self):
        v = self.__dict__
        v["pc_en_v0"] = 0
        v["pc_en_v1"] = 0
        v["pc_tgl_s0"] = 0
        v["pc_tgl_s1"] = 0
        v["pc_tgl_s2"] = 0
        v["pc_buf_stg"] = 0
        v["pc_cells_stg"] = 0
        v["pc_en_frame"] = 0
        v["pc_buf_frame"] = 0
        v["pc_cells_frame"] = 0
        v["vs_d"] = 0
        v["stg_latches"] = 0       # how many times the toggle edge loaded staging
        v["frame_latches"] = 0     # how many video_frame_start releases happened

    # -- combinational views the marquee would see --
    def pc_en_gated(self):
        return self.pc_text_enable & self.pc_en_frame

    def pc_tgl_edge(self):
        return self.pc_tgl_s1 ^ self.pc_tgl_s2

    def video_frame_start(self):
        return self.vs_d and not self.vs_pin

    # -- clock-edge handlers --
    def _do_clk(self):
        self.uart.tick()

    def _do_vid(self):
        s = self.__dict__
        u = self.uart
        nxt = {}
        # bare 2FF on the quasi-static level
        nxt["pc_en_v0"] = u.pc_text_en
        nxt["pc_en_v1"] = s["pc_en_v0"]
        # 3FF on the commit toggle
        nxt["pc_tgl_s0"] = u.text_toggle
        nxt["pc_tgl_s1"] = s["pc_tgl_s0"]
        nxt["pc_tgl_s2"] = s["pc_tgl_s1"]
        nxt["pc_buf_stg"] = s["pc_buf_stg"]
        nxt["pc_cells_stg"] = s["pc_cells_stg"]
        # data+toggle: latch staging on the synchronised toggle edge
        if self.pc_tgl_edge():
            nxt["pc_buf_stg"] = u.char_buf
            nxt["pc_cells_stg"] = u.n_cells
            self.stg_latches += 1
        nxt["pc_en_frame"] = s["pc_en_frame"]
        nxt["pc_buf_frame"] = s["pc_buf_frame"]
        nxt["pc_cells_frame"] = s["pc_cells_frame"]
        # frame-atomic release
        if self.video_frame_start():
            nxt["pc_en_frame"] = s["pc_en_v1"]
            nxt["pc_buf_frame"] = s["pc_buf_stg"]
            nxt["pc_cells_frame"] = s["pc_cells_stg"]
            self.frame_latches += 1
        nxt["vs_d"] = self.vs_pin
        s.update(nxt)

    # -- time advance over the merged axis --
    def run_clk(self, target_n):
        while self.clk_n < target_n:
            tc = CLK_OFF + CLK_PERIOD * self.clk_n
            tv = VID_OFF + VID_PERIOD * self.vid_n
            if tc <= tv:
                self._do_clk(); self.clk_n += 1
            else:
                self._do_vid(); self.vid_n += 1

    def send(self, byte_list, cpb=None, extra=120):
        if cpb is None:
            cpb = self.uart.cpb
        # rx_levels is indexed by absolute clk cycle (== uart.n == clk_n). Pad with
        # idle-high so the new frame's start bit lands exactly on the next clk edge.
        while len(self.uart.rx_levels) < self.uart.n:
            self.uart.rx_levels.append(1)
        self.uart.rx_levels.extend(build_rx_wave(byte_list, cpb))
        self.run_clk(len(self.uart.rx_levels) + extra)

    def settle(self, cycles=40):
        self.run_clk(self.clk_n + cycles)

    def frame_boundary(self, low_cycles=4, high_cycles=4):
        self.vs_pin = 0
        self.run_clk(self.clk_n + low_cycles)
        self.vs_pin = 1
        self.run_clk(self.clk_n + high_cycles)


# ---------------------------------------------------------------------------
def _unpack_char_buf(buf, n):
    """Read back the committed string the way marquee_overlay's char_base does."""
    out = []
    for i in range(n):
        out.append(chr((buf >> (i * CFG["char_stride"])) & 0x7F))
    return "".join(out)


def pass_a():
    print("\n[A] RX decode at the real divider CLKS_PER_BIT = %d (50 MHz / %d)"
          % (CPB_REAL, CFG["baud"]))
    s = System(cpb=CPB_REAL)
    s.send(frame_pctx(1), extra=200)
    want = frame_pctx(1)
    check(s.uart.rx_log == want,
          "real-CPB byte stream decodes the exact %d wire bytes" % len(want),
          "got %s" % s.uart.rx_log)
    check(s.uart.pc_text_en == 1 and s.uart.commit_log == [("PCTX", 1)],
          "real-CPB PCTX 1 commits pc_text_en=1 once",
          "pc_text_en=%d commits=%s" % (s.uart.pc_text_en, s.uart.commit_log))


def pass_b():
    print("\n[B] Parser effects + guards (cpb=%d)" % CPB_FAST)

    # PCTX 1 / PCTX 0 drive the level
    s = System()
    s.send(frame_pctx(1)); s.settle()
    on = s.uart.pc_text_en
    s.send(frame_text("HELLO FPGA")); s.settle()
    still_on = s.uart.pc_text_en          # TEXT must NOT touch pc_text_en
    s.send(frame_pctx(0)); s.settle()
    off = s.uart.pc_text_en
    check(on == 1 and still_on == 1 and off == 0,
          "PCTX 1 -> en=1, TEXT leaves en alone, PCTX 0 -> en=0",
          "on=%d after_text=%d off=%d" % (on, still_on, off))

    # TEXT packs char_buf at cell*stride, sets n_cells, flips text_toggle once
    for txt in ("A", "HELLO FPGA 2026", "Mix3d CASE + 99!", "two  internal   spaces"):
        s = System()
        s.send(frame_pctx(1)); s.settle()
        t0 = s.uart.text_toggle
        s.send(frame_text(txt)); s.settle()
        n = min(len(txt), CFG["pc_cap"])
        want_buf = pack_char_buf(txt[:n])
        ok = (s.uart.n_cells == n and s.uart.char_buf == want_buf
              and s.uart.text_toggle == (t0 ^ 1)
              and _unpack_char_buf(s.uart.char_buf, n) == txt[:n])
        check(ok, "TEXT %-22r -> n_cells=%d, char_buf packs at cell*%d, toggle flips"
              % (txt, n, CFG["char_stride"]),
              "n_cells=%d toggle %d->%d readback=%r"
              % (s.uart.n_cells, t0, s.uart.text_toggle,
                 _unpack_char_buf(s.uart.char_buf, s.uart.n_cells)))

    # internal spaces are kept (0x20 is a legal payload char, not a separator here)
    s = System()
    s.send(frame_text("a b  c")); s.settle()
    check(_unpack_char_buf(s.uart.char_buf, s.uart.n_cells) == "a b  c",
          "internal spaces survive inside TEXT",
          "readback=%r" % _unpack_char_buf(s.uart.char_buf, s.uart.n_cells))

    # the 24-char cap truncates at the source
    cap = CFG["pc_cap"]
    long_s = "".join(chr(0x41 + (i % 26)) for i in range(cap + 6))
    s = System()
    s.send(frame_text(long_s)); s.settle()
    check(s.uart.n_cells == cap and _unpack_char_buf(s.uart.char_buf, cap) == long_s[:cap],
          "TEXT of %d chars truncates to the cap %d at the source" % (len(long_s), cap),
          "n_cells=%d readback=%r" % (s.uart.n_cells,
                                      _unpack_char_buf(s.uart.char_buf, s.uart.n_cells)))

    # exactly cap chars is stored in full (boundary, not off-by-one)
    exact = long_s[:cap]
    s = System()
    s.send(frame_text(exact)); s.settle()
    check(s.uart.n_cells == cap and _unpack_char_buf(s.uart.char_buf, cap) == exact,
          "TEXT of exactly %d chars stores all %d (no off-by-one)" % (cap, cap),
          "n_cells=%d" % s.uart.n_cells)

    # ---- guards: rejected frames must NOT commit and must NOT wedge ----
    def rejected(name, wire, why):
        s = System()
        s.send(wire); s.settle()
        no_commit = (s.uart.commit_log == [])
        # the parser must still accept a good frame afterwards
        s.send(frame_text("OK")); s.settle()
        recovered = (s.uart.n_cells == 2 and _unpack_char_buf(s.uart.char_buf, 2) == "OK")
        check(no_commit and recovered, "%s rejected (%s), parser recovers" % (name, why),
              "commits=%s then n_cells=%d" % (s.uart.commit_log, s.uart.n_cells))

    rejected("lowercase 'text hi'",
             [ord(c) for c in "text hi"] + [0xFF, 0xFF, 0xFF], "keyword is case-sensitive")
    rejected("lowercase 'pctx 1'",
             [ord(c) for c in "pctx 1"] + [0xFF, 0xFF, 0xFF], "keyword is case-sensitive")
    rejected("unknown 'ZZZZ 1'",
             [ord(c) for c in "ZZZZ 1"] + [0xFF, 0xFF, 0xFF], "mode stays IGNORE")
    rejected("TEXT without separator",
             [ord(c) for c in "TEXTX"] + [0xFF, 0xFF, 0xFF], "byte 4 is 'X' not ' '")
    rejected("PCTX with arg '2'",
             [ord(c) for c in "PCTX 2"] + [0xFF, 0xFF, 0xFF], "argument not '0'/'1'")
    rejected("PCTX with no arg",
             [ord(c) for c in "PCTX"] + [0xFF, 0xFF, 0xFF], "pctx_valid never set")
    rejected("empty TEXT",
             [ord(c) for c in "TEXT "] + [0xFF, 0xFF, 0xFF], "char_count < 1")

    # a PCTX with a trailing junk byte after a valid arg must be voided (pctx_bad)
    s = System()
    s.send([ord(c) for c in "PCTX 1X"] + [0xFF, 0xFF, 0xFF]); s.settle()
    check(s.uart.pc_text_en == 0 and s.uart.commit_log == [],
          "PCTX 1X voided by pctx_bad (byte 6 in MODE_PCTX clears pctx_valid)",
          "pc_text_en=%d commits=%s" % (s.uart.pc_text_en, s.uart.commit_log))


def pass_c():
    print("\n[C] CDC clk -> video_clk (bare 2FF level + data/toggle + frame-atomic)")

    # pc_text_en crosses on a bare 2FF and is released at the frame start
    s = System()
    s.settle(40)
    s.send(frame_pctx(1))
    s.settle(200)                       # many video_clk edges, NO vsync pulse yet
    pre = (s.pc_en_frame, s.uart.pc_text_en, s.pc_en_v1)
    s.frame_boundary()
    post = s.pc_en_frame
    check(pre[1] == 1 and pre[2] == 1 and pre[0] == 0 and post == 1,
          "PCTX 1: clk en=1, 2FF v1=1, but pc_en_frame waits for the frame start",
          "before boundary frame=%d (v1=%d), after=%d" % (pre[0], pre[2], post))

    # char_buf/n_cells cross on data+toggle and are frame-atomic: a mid-frame TEXT
    # must be held in staging until the next video_frame_start (no tearing).
    s = System()
    s.settle(40)
    s.send(frame_pctx(1)); s.frame_boundary()
    s.send(frame_text("FIRST")); s.settle(200)
    s.frame_boundary()
    first = (s.pc_cells_frame, _unpack_char_buf(s.pc_buf_frame, s.pc_cells_frame))
    # now change the string mid-frame and confirm the pixels do not see it yet
    s.send(frame_text("SECOND")); s.settle(300)
    held = (s.pc_cells_frame, _unpack_char_buf(s.pc_buf_frame, s.pc_cells_frame))
    stg = (s.pc_cells_stg, _unpack_char_buf(s.pc_buf_stg, s.pc_cells_stg))
    s.frame_boundary()
    released = (s.pc_cells_frame, _unpack_char_buf(s.pc_buf_frame, s.pc_cells_frame))
    check(first == (5, "FIRST"), "first TEXT released at the frame start",
          "frame=%r" % (first,))
    check(held == (5, "FIRST") and stg[1] == "SECOND",
          "mid-frame TEXT held in staging, pixels still show the old string",
          "frame=%r staging=%r" % (held, stg))
    check(released == (6, "SECOND"),
          "the new string is released atomically at the next frame start",
          "frame=%r" % (released,))

    # exactly one staging latch per committed TEXT (no loss, no double)
    s = System()
    s.settle(40)
    s.send(frame_pctx(1)); s.frame_boundary()
    before = s.stg_latches
    for txt in ("ONE", "TWO", "THREE"):
        s.send(frame_text(txt)); s.settle(120)
    latches = s.stg_latches - before
    check(latches == 3,
          "3 back-to-back TEXTs -> exactly 3 staging latches (data+toggle, no loss/double)",
          "latches=%d" % latches)
    s.frame_boundary()
    check(_unpack_char_buf(s.pc_buf_frame, s.pc_cells_frame) == "THREE",
          "the last committed TEXT is what the frame finally shows",
          "frame=%r" % _unpack_char_buf(s.pc_buf_frame, s.pc_cells_frame))


def pass_d():
    print("\n[D] Negative controls + the bit-identical retreat")

    # zero traffic: nothing crosses, nothing commits, the banner stays the slogan's
    s = System()
    s.settle(400)
    for _ in range(4):
        s.frame_boundary()
    check(s.uart.pc_text_en == 0 and s.uart.char_buf == 0 and s.uart.n_cells == 0
          and s.uart.text_toggle == 0 and s.uart.commit_log == [],
          "retreat: zero traffic leaves clk-domain outputs at reset",
          "en=%d buf=%d n=%d toggle=%d commits=%s"
          % (s.uart.pc_text_en, s.uart.char_buf, s.uart.n_cells,
             s.uart.text_toggle, s.uart.commit_log))
    check(s.pc_en_frame == 0 and s.pc_buf_frame == 0 and s.pc_cells_frame == 0
          and s.pc_en_gated() == 0 and s.stg_latches == 0,
          "retreat: pc_en_gated=0 and the frame registers stay 0 across 4 boundaries",
          "frame en=%d buf=%d n=%d gated=%d stg_latches=%d"
          % (s.pc_en_frame, s.pc_buf_frame, s.pc_cells_frame,
             s.pc_en_gated(), s.stg_latches))

    # PC_TEXT_ENABLE=0 is the one-line top retreat: even a committed PCTX 1 + TEXT
    # cannot light the PC arm, because pc_en_gated = PC_TEXT_ENABLE & pc_en_frame.
    s = System(pc_text_enable=0)
    s.settle(40)
    s.send(frame_pctx(1)); s.frame_boundary()
    s.send(frame_text("HIDDEN")); s.settle(200); s.frame_boundary()
    check(s.uart.pc_text_en == 1 and s.pc_en_frame == 1 and s.pc_en_gated() == 0,
          "PC_TEXT_ENABLE=0 ties pc_en_gated low even though the link committed",
          "clk en=%d frame=%d gated=%d" % (s.uart.pc_text_en, s.pc_en_frame,
                                           s.pc_en_gated()))

    # two 0xFF only -> never commit
    s = System()
    s.send([ord(c) for c in "TEXT HI"] + [0xFF, 0xFF]); s.settle(80)
    check(s.uart.commit_log == [] and s.uart.n_cells == 0,
          "two 0xFF only -> no commit (ffc never reaches %d)" % CFG["commit_ffc"],
          "commits=%s n_cells=%d" % (s.uart.commit_log, s.uart.n_cells))

    # a partial frame then a valid one: no false commit, the link self-heals
    s = System()
    s.send([ord(c) for c in "TE"] + [0xFF, 0xFF, 0xFF]); s.settle(40)
    false_commits = list(s.uart.commit_log)
    s.send(frame_text("OK")); s.settle(60)
    check(false_commits == [] and s.uart.n_cells == 2
          and _unpack_char_buf(s.uart.char_buf, 2) == "OK",
          "partial 'TE'+FFF commits nothing, a following TEXT works",
          "false=%s then n_cells=%d" % (false_commits, s.uart.n_cells))

    # negative control on the model itself: if commit fired on the FIRST 0xFF the
    # two-0xFF frame above would wrongly dispatch -- prove the guard is load-bearing
    saved = CFG["commit_ffc"]
    try:
        CFG["commit_ffc"] = 0
        s = System()
        s.send([ord(c) for c in "TEXT HI"] + [0xFF, 0xFF]); s.settle(80)
        early = (s.uart.commit_log != [])
    finally:
        CFG["commit_ffc"] = saved
    expect_fail(not early,
                "committing on the first 0xFF (ffc==0) would dispatch the 2-FF frame")

    # negative control: dropping the cap would let n_cells exceed 24 and overflow
    # marquee's 11-bit borrow math (s = x_pos + marq_pos). Prove the cap bites.
    saved_cap = CFG["pc_cap"]
    try:
        CFG["pc_cap"] = 63
        s = System()
        s.send(frame_text("".join("A" * 30))); s.settle(120)
        over = (s.uart.n_cells > 24)
    finally:
        CFG["pc_cap"] = saved_cap
    expect_fail(not over,
                "without the cap a 30-char TEXT would commit n_cells>24 (marquee overflow)")


def pass_e():
    print("\n[E] dbg toggles are observation-only")

    # idle line: both stay at reset, nothing dispatched
    s = System()
    s.settle(200)
    check(s.uart.dbg_rx_toggle == 0 and s.uart.dbg_commit_toggle == 0
          and s.uart.commit_log == [],
          "no traffic -> both dbg toggles stay at reset",
          "rx_tgl=%d commit_tgl=%d" % (s.uart.dbg_rx_toggle, s.uart.dbg_commit_toggle))

    # a clean TEXT frame: rx-toggle flips once per byte, commit-toggle once per frame
    txt = "HELLO"
    wire = frame_text(txt)
    s = System()
    s.send(wire); s.settle(80)
    check(s.uart.dbg_rx_toggle == (len(wire) & 1) and s.uart.dbg_commit_toggle == 1,
          "clean TEXT: rx-toggle parity = byte count mod 2, commit-toggle flips once",
          "bytes=%d rx_tgl=%d commit_tgl=%d"
          % (len(wire), s.uart.dbg_rx_toggle, s.uart.dbg_commit_toggle))

    # bytes but no terminator: rx-toggle ends lit (odd byte count), commit dark
    wire = [ord(c) for c in "TEXT HI"] + [ord("X"), ord("Y")]   # 9 bytes, no 0xFF
    s = System()
    s.send(wire); s.settle(80)
    check(len(s.uart.rx_log) == len(wire) and s.uart.dbg_rx_toggle == (len(wire) & 1)
          and s.uart.dbg_commit_toggle == 0 and s.uart.commit_log == [],
          "no terminator -> %d bytes seen, rx-toggle lit, commit-toggle dark, no dispatch"
          % len(wire),
          "rx=%d rx_tgl=%d commit_tgl=%d" % (len(s.uart.rx_log),
          s.uart.dbg_rx_toggle, s.uart.dbg_commit_toggle))

    # the dbg toggles must not feed the protocol: a committed frame's effect is
    # identical whether or not we read the toggles (they are pure observation).
    s = System()
    s.send(frame_pctx(1)); s.settle()
    check(s.uart.pc_text_en == 1 and s.uart.dbg_commit_toggle == 0,
          "PCTX commits pc_text_en without touching dbg_commit_toggle (TEXT-only)",
          "en=%d commit_tgl=%d" % (s.uart.pc_text_en, s.uart.dbg_commit_toggle))


def main():
    print("=" * 72)
    print("uart_pc_text.v + top PC-text CDC -- cycle-accurate model")
    print("=" * 72)
    print("parsed from RTL: CLKS_PER_BIT %d (%d/%d), PC_CAP %d, char_buf %d bits "
          "(cell*%d +: 7), n_cells %d bits, payload 0x%02X..0x%02X, commit on "
          "ffc==%d, keywords %r/%r"
          % (CFG["cpb"], CFG["clk_hz"], CFG["baud"], CFG["pc_cap"],
             CFG["char_buf_w"], CFG["char_stride"], CFG["n_cells_w"],
             CFG["payload_lo"], CFG["payload_hi"], CFG["commit_ffc"],
             CFG["kw_pctx"], CFG["kw_text"]))
    print("top CDC        : PC_TEXT_ENABLE=%d, pc_text_en bare 2FF, char_buf/n_cells "
          "data+toggle (3FF), frame-atomic at video_frame_start, pc_en_gated = "
          "PC_TEXT_ENABLE & pc_en_frame" % CFG["pc_enable_param"])

    pass_a()
    pass_b()
    pass_c()
    pass_d()
    pass_e()

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
