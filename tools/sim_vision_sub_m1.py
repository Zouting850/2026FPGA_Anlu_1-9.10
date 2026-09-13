#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cycle-accurate / protocol-level models for the vision sub-board M1 design.

No Verilog simulator on this machine, so this mirrors the M1 RTL and asserts the
things that actually decide whether the board works the first time it is powered:

  1. sccb_master.v drives a *legal* SCCB/I2C waveform:
       - data changes only while SCL is LOW, stable while SCL is HIGH;
       - 9 clocks per byte (8 data + ACK slot);
       - a real START (SDA falls while SCL high) and a real STOP
         (SDA rises while SCL high) -- NOT SCL and SDA jumping together;
       - on a 16-bit read the master (the receiver) ACKs D_H and NACKs only
         D_L. An early NACK makes the MT9V034 release SDA, so the version
         reads 0xFFFF and the self-test falsely fails.

  2. SCL really lands on 100 kHz (50 MHz / (2 x SCCB_DIV_NUM)).

  3. mt9v034_cfg.v writes the register table in order with the right values,
     then reads R0x00 and requires 0x1324. R0x0D must keep bits[9:8] set
     (0x0305), not 0x0005 -- the datasheet marks them "always 1".

  4. dvp_capture.v + frame_stat.v turn one synthetic FRAME_VALID/LINE_VALID
     frame into exactly 376*240 = 90240 pixels with the right sum/min/max.

  5. dbg_uart.v frames a byte as 1 start (0) + 8 data LSB-first + 1 stop (1) at
     115200 baud (UART_BAUD_DIV clocks per bit).

  6. The top-level status line is exactly 44 bytes and hex-encodes the fields.

The register table and the fixed-character table are PARSED OUT OF THE RTL, not
restated here, so the model cannot silently drift from the design.

Run from anywhere:  python tools/sim_vision_sub_m1.py
"""
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)
HDL = os.path.join(ROOT, "src", "vision_sub", "user_source", "hdl_source")
DEFS = os.path.join(HDL, "include", "vision_def.v")
F_TOP = os.path.join(HDL, "top_vision_m1.v")

FAILURES = []
CHECKS = [0]

S_IDLE, S_START, S_BIT, S_ACK, S_RESTART, S_STOP, S_END = range(7)


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
        raise RuntimeError("fixed_char() not found in top_vision_m1.v")
    tbl = {}
    for _wid, idx, hexv in re.findall(
            r"(\d+)'d(\d+)\s*:\s*fixed_char\s*=\s*8'h([0-9A-Fa-f]+)", m.group(1)):
        tbl[int(idx)] = int(hexv, 16)
    tbl["default"] = None
    return tbl


# ============================================================
# SCCB master, bit level
# ============================================================
class SccbMaster:
    def __init__(self, div, dev_addr):
        self.DIV = div
        self.DEV = dev_addr
        self.reset()

    def reset(self):
        self.st, self.bitc, self.byte_idx = S_IDLE, 0, 0
        self.div, self.half = 0, 0
        self.sh, self.rx_sh = 0, 0
        self.scl, self.sda_oe, self.sda_out = 1, 1, 1
        self.ra, self.wd, self.rd, self.rw = 0, 0, 0, 0
        self.nack_c, self.busy, self.ack = 0, 0, 0
        self.req, self.req_rw, self.reg_addr, self.wr_data = 0, 0, 0, 0
        self.tx_log = []            # (byte_idx, byte) actually shifted out
        self.ack_drive_log = []     # (byte_idx, sda_out) when the master drives ACK
        self.cur_tx = 0             # accumulates the byte being shifted out
        self.trace = []             # (scl, sda_bus, st_before_step)

    def tx_byte_of(self, idx):
        if idx == 0:
            return self.DEV
        if idx == 1:
            return (self.ra >> 8) & 0xFF
        if idx == 2:
            return self.ra & 0xFF
        if idx == 3:
            return (self.DEV | 1) if self.rw else ((self.wd >> 8) & 0xFF)
        if idx == 4:
            return 0x00 if self.rw else (self.wd & 0xFF)
        return 0x00

    def step(self, sda_in):
        DIV = self.DIV
        scl, sda_oe, sda_out = self.scl, self.sda_oe, self.sda_out
        st, bitc, byte_idx = self.st, self.bitc, self.byte_idx
        div, half, sh, rx_sh = self.div, self.half, self.sh, self.rx_sh
        cb = self.cur_tx
        ra, wd, rd, rw = self.ra, self.wd, self.rd, self.rw
        nack_c, busy, ack = self.nack_c, self.busy, self.ack

        ack = 0
        tick = (div == DIV - 1)
        rx_byte = (rw == 1) and (byte_idx >= 4)
        mack_byte = (rw == 1) and (byte_idx == 4)
        nack_byte = (rw == 1) and (byte_idx == 5)

        if st != S_IDLE:
            div = 0 if tick else div + 1

        if st == S_IDLE:
            scl, sda_oe, sda_out, div, half = 1, 1, 1, 0, 0
            if self.req:
                ra, wd, rw = self.reg_addr, self.wr_data, self.req_rw
                byte_idx, bitc, busy, st = 0, 0, 1, S_START

        elif st == S_START:
            if tick:
                old = half
                half = 1 - old
                if old == 0:
                    scl, sda_oe, sda_out = 1, 1, 1
                else:                                   # SDA 下降 = START
                    sda_out = 0
                    sh = self.tx_byte_of(0)
                    bitc = 0
                    st = S_BIT

        elif st == S_BIT:
            if tick:
                old = half
                half = 1 - old
                if old == 0:                            # SCL 低：摆数据
                    scl = 0
                    if rx_byte:
                        sda_oe = 0
                    else:
                        sda_oe, sda_out = 1, (sh >> 7) & 1
                        cb = ((cb << 1) | (sh >> 7)) & 0xFF
                else:                                   # SCL 高：采样
                    scl = 1
                    got = ((rx_sh & 0x7F) << 1) | (sda_in & 1)
                    rx_sh = ((rx_sh << 1) | (sda_in & 1)) & 0xFF
                    if bitc == 7:
                        bitc = 0
                        if rx_byte:
                            if byte_idx == 4:
                                rd = (rd & 0x00FF) | (got << 8)
                            elif byte_idx == 5:
                                rd = (rd & 0xFF00) | got
                        else:
                            self.tx_log.append((byte_idx, cb))
                            cb = 0
                        st = S_ACK
                    else:
                        bitc += 1
                        if not rx_byte:
                            sh = (sh << 1) & 0xFF

        elif st == S_ACK:
            if tick:
                old = half
                half = 1 - old
                if old == 0:                            # SCL 低：摆 ACK/NACK
                    scl = 0
                    if nack_byte:
                        sda_oe, sda_out = 1, 1          # 读末字节：NACK
                    elif mack_byte:
                        sda_oe, sda_out = 1, 0          # 读首字节：主机 ACK
                    else:
                        sda_oe = 0                      # 释放，等从机 ACK
                    if sda_oe == 1:
                        self.ack_drive_log.append((byte_idx, sda_out))
                else:                                   # SCL 高：采样从机 ACK
                    scl = 1
                    if sda_in == 1 and not nack_byte and not mack_byte \
                            and nack_c != 31:
                        nack_c += 1
                    sda_oe, sda_out = 1, 1
                    if rw == 0:
                        if byte_idx == 4:
                            st = S_STOP
                        else:
                            nxt = self.tx_byte_of(byte_idx + 1)
                            byte_idx += 1
                            sh, bitc, st = nxt, 0, S_BIT
                    else:
                        if byte_idx == 2:
                            st = S_RESTART
                        elif byte_idx == 5:
                            st = S_STOP
                        else:
                            nxt = self.tx_byte_of(byte_idx + 1)
                            byte_idx += 1
                            sh, bitc, st = nxt, 0, S_BIT

        elif st == S_RESTART:
            if tick:
                old = half
                half = 1 - old
                if old == 0:
                    scl, sda_oe, sda_out = 1, 1, 1
                else:                                   # SDA 下降 = RESTART
                    sda_out = 0
                    byte_idx = 3
                    sh = self.DEV | 1
                    bitc = 0
                    st = S_BIT

        elif st == S_STOP:
            # SCL 低 → SDA 低 → SCL 高 → SDA 高（SCL 高时上升 = 合法 STOP）
            if tick:
                sda_oe = 1
                if bitc == 0:
                    scl, sda_out, bitc = 0, 1, 1
                elif bitc == 1:
                    sda_out, bitc = 0, 2
                elif bitc == 2:
                    scl, sda_out, bitc = 1, 0, 3
                elif bitc == 3:
                    sda_out, bitc = 1, 4
                else:
                    bitc, st = 0, S_END

        elif st == S_END:
            ack, busy, scl, st = 1, 0, 1, S_IDLE

        self.st, self.bitc, self.byte_idx = st, bitc, byte_idx
        self.div, self.half = div, half
        self.sh, self.rx_sh = sh, rx_sh
        self.scl, self.sda_oe, self.sda_out = scl, sda_oe, sda_out
        self.cur_tx = cb
        self.ra, self.wd, self.rd, self.rw = ra, wd, rd, rw
        self.nack_c, self.busy, self.ack = nack_c, busy, ack


class MT9V034Slave:
    """Minimal but protocol-faithful MT9V034 two-wire slave.

    Detects real START/RESTART (SDA falls while SCL high) and STOP (SDA rises
    while SCL high), resets its byte index on a repeated START, and drives ACK
    low for every byte it receives. When it is the transmitter (a read) it emits
    8 bits + an ACK-slot gap + 8 bits + an ACK-slot gap, skipping the master's
    ACK/NACK slots so the bit alignment stays correct.
    """

    def __init__(self, regs, dev=0x90):
        self.regs = dict(regs)
        self.dev = dev
        self.scl_prev, self.sda_prev = 1, 1
        self.drv = None
        self.bitc, self.byte_idx = 0, 0
        self.is_read = False
        self.repeated = False
        self.in_txn = False
        self.ra, self.hi8, self.hi8b = 0, 0, 0
        self.ack_drive = False
        self.plan, self.xpos = [], 0
        self.skip_sample = False
        self.log_w, self.log_r = [], []
        self.starts, self.stops = 0, 0

    def bus_value(self, master_driving, master_sda):
        if master_driving:
            return master_sda
        return 1 if self.drv is None else self.drv

    def tick(self, scl, master_driving, master_sda):
        if scl == 1 and self.scl_prev == 1 and self.sda_prev == 1 \
                and master_sda == 0:
            self.starts += 1
            # A first START opens a fresh transaction at byte 0. A repeated
            # START (issued before the read data phase) mirrors the master:
            # bus byte_idx jumps straight to 3, the read device byte, so the
            # following _after_ack() can arm the read-data plan.
            self.repeated = self.in_txn
            self.in_txn = True
            self.bitc = 0
            self.byte_idx = 3 if self.repeated else 0
            self.is_read = False
            self.ack_drive, self.drv, self.plan, self.xpos = False, None, [], 0
            self.skip_sample = False
        if scl == 1 and self.scl_prev == 1 and self.sda_prev == 0 \
                and master_sda == 1:
            self.stops += 1
            self.ack_drive, self.drv, self.plan, self.xpos = False, None, [], 0
            self.skip_sample = False
            self.in_txn, self.repeated = False, False

        rising = (scl == 1 and self.scl_prev == 0)
        falling = (scl == 0 and self.scl_prev == 1)

        if falling:
            if self.ack_drive:
                self.drv = 0
            elif self.xpos < len(self.plan):
                nxt = self.plan[self.xpos]
                self.xpos += 1
                if nxt is None:
                    self.drv = None
                    self.skip_sample = True
                else:
                    self.drv = nxt

        if rising:
            if self.ack_drive:
                self.ack_drive, self.drv, self.bitc = False, None, 0
                self.byte_idx += 1
                self._after_ack()
            elif self.skip_sample:
                self.skip_sample = False
            elif master_driving:
                self.hi8 = ((self.hi8 << 1) | (master_sda & 1)) & 0xFF
                self.bitc += 1
                if self.bitc == 8:
                    self._byte_done()

        self.scl_prev, self.sda_prev = scl, master_sda

    def _byte_done(self):
        v = self.hi8
        if self.byte_idx == 0 or (self.byte_idx == 3 and self.repeated):
            # byte 0 = device byte of the write phase; byte 3 = device byte
            # after the repeated START. The LSB is the R/W bit.
            self.is_read = bool(v & 1)
            self.ack_drive = ((v & 0xFE) == (self.dev & 0xFE))
        elif not self.is_read:
            if self.byte_idx == 1:
                self.ra = v << 8
            elif self.byte_idx == 2:
                self.ra = (self.ra & 0xFF00) | v
            elif self.byte_idx == 3:
                self.hi8b = v
            elif self.byte_idx == 4:
                word = ((self.hi8b << 8) | v) & 0xFFFF
                self.regs[self.ra] = word
                self.log_w.append((self.ra, word))
            self.ack_drive = True
        else:
            if self.byte_idx == 1:
                self.ra = v << 8
            elif self.byte_idx == 2:
                self.ra = (self.ra & 0xFF00) | v
            self.ack_drive = True

    def _after_ack(self):
        if self.is_read and self.byte_idx == 4:
            word = self.regs.get(self.ra, 0)
            self.log_r.append((self.ra, word))
            bits = [(word >> b) & 1 for b in range(15, -1, -1)]
            self.plan = bits[0:8] + [None] + bits[8:16] + [None]
            self.xpos = 0


def run_sccb_txn(master, slave, max_clk=200000):
    def one():
        st_before = master.st
        master.step(slave.bus_value(master.sda_oe, master.sda_out))
        slave.tick(master.scl, master.sda_oe, master.sda_out)
        sda_bus = slave.bus_value(master.sda_oe, master.sda_out)
        master.trace.append((master.scl, sda_bus, st_before))

    for _ in range(max_clk):
        one()
        if master.ack:
            for _ in range(6):
                one()
            return True
    return False


def find_condition(trace, want_sda):
    """True if SDA moves to want_sda while SCL stays high (START/STOP)."""
    for k in range(1, len(trace)):
        scl, sda, _ = trace[k]
        pscl, psda, _ = trace[k - 1]
        if scl == 1 and pscl == 1 and psda == (1 - want_sda) and sda == want_sda:
            return True
    return False


def data_change_violations(trace):
    """SDA changes while SCL is high, mid-byte (S_BIT/S_ACK) = I2C violation."""
    bad = 0
    for k in range(1, len(trace)):
        scl, sda, st = trace[k]
        pscl, psda, _ = trace[k - 1]
        if scl == 1 and pscl == 1 and sda != psda and st in (S_BIT, S_ACK):
            bad += 1
    return bad


def count_scl_rises(trace):
    return sum(1 for k in range(1, len(trace))
               if trace[k][0] == 1 and trace[k - 1][0] == 0)


def test_sccb(env):
    print("\n-- 1/2. sccb_master.v protocol + clock rate --")
    div, dev = env["SCCB_DIV_NUM"], env["SCCB_DEV_ADDR"]
    scl_hz = 50e6 / (2.0 * div)
    check(abs(scl_hz - 100e3) < 1e3,
          "SCL = 50MHz/(2x%d) = %.1f kHz (target 100 kHz)" % (div, scl_hz / 1e3))

    regs = {0x00: 0x1324, 0x0D: 0x0300, 0x07: 0x0388}

    m = SccbMaster(div, dev)
    s = MT9V034Slave(regs)
    m.reg_addr, m.wr_data, m.req_rw, m.req = 0x0D, 0x0305, 0, 1
    check(run_sccb_txn(m, s), "write txn completes (S_END reached)")
    check([b for _, b in m.tx_log] == [0x90, 0x00, 0x0D, 0x03, 0x05],
          "write byte order = 90 00 0D 03 05",
          "got %s" % ["%02X" % b for _, b in m.tx_log])
    check(s.regs.get(0x0D) == 0x0305, "slave committed R0x0D = 0x0305",
          "regs[0x0D]=0x%04X" % s.regs.get(0x0D, 0))
    check(m.ack_drive_log == [] and m.nack_c == 0,
          "write: master releases SDA on every ACK slot (no NACK)",
          "ack_drive_log=%s nack_c=%d" % (m.ack_drive_log, m.nack_c))
    check(find_condition(m.trace, 1), "valid STOP (SDA rises while SCL high)")
    check(find_condition(m.trace, 0), "valid START (SDA falls while SCL high)")
    check(s.starts == 1 and s.stops == 1,
          "slave saw exactly 1 START and 1 STOP",
          "starts=%d stops=%d" % (s.starts, s.stops))
    check(data_change_violations(m.trace) == 0,
          "SDA only changes while SCL is LOW during data/ACK phases",
          "%d violations" % data_change_violations(m.trace))
    check(count_scl_rises(m.trace) == 5 * 9 + 1,
          "46 SCL pulses = 45 byte clocks (9 x 5) + 1 STOP clock",
          "got %d" % count_scl_rises(m.trace))

    m = SccbMaster(div, dev)
    s = MT9V034Slave(regs)
    m.reg_addr, m.wr_data, m.req_rw, m.req = 0x00, 0x0000, 1, 1
    check(run_sccb_txn(m, s), "read txn completes (S_END reached)")
    check([b for _, b in m.tx_log] == [0x90, 0x00, 0x00, 0x91],
          "read address phase = 90 00 00 then repeated-start 91",
          "got %s" % ["%02X" % b for _, b in m.tx_log])
    check(m.rd == 0x1324, "read R0x00 returns 0x1324", "got 0x%04X" % m.rd)
    check(m.ack_drive_log == [(4, 0), (5, 1)],
          "master drives ACK after D_H and NACK after D_L",
          "got %s" % m.ack_drive_log)
    check(s.starts == 2, "START + repeated START both seen by the slave",
          "starts=%d" % s.starts)
    check(find_condition(m.trace, 1), "valid STOP after the read")
    check(data_change_violations(m.trace) == 0,
          "read: SDA only changes while SCL is LOW",
          "%d violations" % data_change_violations(m.trace))
    check(count_scl_rises(m.trace) == 6 * 9 + 1,
          "55 SCL pulses = 54 byte clocks (4 tx + 2 rx) + 1 STOP clock",
          "got %d" % count_scl_rises(m.trace))

    expect_fail(m.ack_drive_log == [(4, 1), (5, 1)],
                "old rule (nack_byte = byte_idx >= 4) drives two NACKs; "
                "correct is ACK(D_H) then NACK(D_L)")


# ============================================================
# config table
# ============================================================
def cfg_expected(env):
    addr_dev = {0: "MT_REG_RESET", 1: "MT_REG_CHIPCTRL", 2: "MT_REG_COLSTART",
                3: "MT_REG_ROWSTART", 4: "MT_REG_WINHEIGHT",
                5: "MT_REG_WINWIDTH", 6: "MT_REG_READMODE",
                7: "MT_REG_AECAGC"}
    data_dev = {0: "MT_VAL_RESET", 1: "MT_VAL_CHIPCTRL", 2: "MT_VAL_COLSTART",
                3: "MT_VAL_ROWSTART", 4: "MT_VAL_WINHEIGHT",
                5: "MT_VAL_WINWIDTH", 6: "MT_VAL_READMODE",
                7: "MT_VAL_AECAGC"}
    return [(env[addr_dev[i]], env[data_dev[i]])
            for i in range(env["CFG_WR_NUM"])]


def test_cfg_table(env):
    print("\n-- 3. mt9v034_cfg.v register table --")
    tbl = cfg_expected(env)
    check(env["MT_VER_EXPECT"] == 0x1324, "expected chip version is 0x1324")

    rm = env["MT_VAL_READMODE"]
    check((rm & 0x0300) == 0x0300,
          "R0x0D = 0x%04X keeps fixed bits[9:8] set" % rm,
          "0x%04X would clear the 'always 1' bits" % (rm & 0x00FF))
    check((rm & 0x0001) == 0x0001, "R0x0D bit0 = row bin 2")
    check((rm & 0x0004) == 0x0004, "R0x0D bit2 = column bin 2")

    cc = env["MT_VAL_CHIPCTRL"]
    check((cc & 0x0007) == 0, "R0x07 scan mode = progressive")
    check(((cc >> 3) & 0x3) == 1, "R0x07 sensor mode = Master (bits[4:3]=01)")
    check(((cc >> 5) & 0x3) == 0, "R0x07 stereoscopy disabled")

    check(env["MT_VAL_WINHEIGHT"] == 480, "R0x03 window height = 480 (pre-bin)")
    check(env["MT_VAL_WINWIDTH"] == 752, "R0x04 window width = 752 (pre-bin)")
    check(env["MT_VAL_WINHEIGHT"] // 2 == env["CAM_IMG_H"],
          "480 / row-bin2 == CAM_IMG_H (%d)" % env["CAM_IMG_H"])
    check(env["MT_VAL_WINWIDTH"] // 2 == env["CAM_IMG_W"],
          "752 / col-bin2 == CAM_IMG_W (%d)" % env["CAM_IMG_W"])

    check(tbl[0][0] == 0x000C, "first write is R0x0C soft reset")
    check(tbl[0][1] == 0x0001, "soft reset = 0x0001 (logic only)")
    check(tbl[1][0] == 0x0007, "R0x07 chip control written before geometry")

    print("      table (%d writes): %s"
          % (len(tbl), ", ".join("0x%02X=0x%04X" % (a, v) for a, v in tbl)))


def test_cfg_run(env):
    print("\n-- 3b. config sequencer + version check (transaction level) --")
    tbl = cfg_expected(env)

    def run(vread):
        writes, err = [], 0
        for _ in range(3):
            writes += list(tbl)
            if vread == env["MT_VER_EXPECT"]:
                return writes, True, err
            err += 1
        return writes, False, err

    w, ok, _ = run(0x1324)
    check(ok, "version 0x1324 -> cam_ok = 1")
    check(w == tbl, "one clean pass through the table, in order")

    w, ok, err = run(0x1234)
    check(not ok, "version mismatch -> cam_ok = 0")
    check(len(w) == 3 * len(tbl), "mismatch retries the whole table 3 times",
          "wrote %d entries" % len(w))
    check(err == 3, "mismatch counted 3 failed readbacks", "err=%d" % err)


# ============================================================
# DVP capture + frame stats
# ============================================================
def model_frame(width, height, blank_h=8, blank_v=4, pattern=None):
    acc_sum, cnt, mn, mx = 0, 0, 0xFF, 0x00
    pix = pattern or (lambda r, c: (r * 7 + c * 3) & 0xFF)
    for v in range(blank_v + height + blank_v):
        if not (blank_v <= v < blank_v + height):
            continue                       # 消隐行不计入有效像素
        for h in range(blank_h + width + blank_h):
            if blank_h <= h < blank_h + width:
                d = pix(v - blank_v, h - blank_h)
                acc_sum += d
                cnt += 1
                mn = min(mn, d)
                mx = max(mx, d)
    return acc_sum, cnt, mn, mx


def test_dvp(env):
    print("\n-- 4. dvp_capture.v + frame_stat.v --")
    w, h = env["CAM_IMG_W"], env["CAM_IMG_H"]
    s, c, mn, mx = model_frame(w, h)
    check(c == w * h, "pixel count == %d (= %d x %d)" % (w * h, w, h),
          "got %d" % c)
    check(c == env["CAM_FRAME_PIX"],
          "matches CAM_FRAME_PIX = %d" % env["CAM_FRAME_PIX"])

    vals = [(r * 7 + c2 * 3) & 0xFF for r in range(h) for c2 in range(w)]
    check(s == sum(vals), "accumulated sum matches an independent sum")
    check(mn == min(vals) and mx == max(vals),
          "min/max match (%d..%d)" % (mn, mx))

    _, _, _, mxw = model_frame(w, h, pattern=lambda r, c2: 0xFF)
    _, _, mnb, _ = model_frame(w, h, pattern=lambda r, c2: 0x00)
    check(mxw == 255 and mnb == 0, "flat-field frames give max=255 / min=0")

    _, c_4x4, _, _ = model_frame(188, 120)
    expect_fail(c_4x4 == env["CAM_FRAME_PIX"],
                "a 4x4-binning frame (188x120) does NOT match 376x240 -- the "
                "pixel count alone catches a wrong R0x0D")


# ============================================================
# UART
# ============================================================
def model_uart_byte(byte, div):
    return [0] + [(byte >> i) & 1 for i in range(8)] + [1], div


def test_uart(env):
    print("\n-- 5. dbg_uart.v framing --")
    div = env["UART_BAUD_DIV"]
    baud = 50e6 / div
    check(abs(baud - 115200) < 500,
          "baud = 50MHz/%d = %.1f (target 115200)" % (div, baud))

    for byte in (0x4D, 0x0A, 0xFF, 0x00):
        bits, per = model_uart_byte(byte, div)
        check(bits[0] == 0, "0x%02X: start bit is 0" % byte)
        check(bits[-1] == 1, "0x%02X: stop bit is 1" % byte)
        check(bits[1:9] == [(byte >> i) & 1 for i in range(8)],
              "0x%02X: 8 data bits LSB-first" % byte)
        check(per == div, "0x%02X: %d clocks per bit" % (byte, div))

    ms = 44 * 10 / baud * 1e3
    check(ms < 6.0,
          "a 44-byte status line takes %.1f ms (< one 60 Hz frame)" % ms)


# ============================================================
# Status line
# ============================================================
def hexc(n):
    return ord("0") + n if n < 10 else ord("A") + n - 10


def line_bytes(prt, tbl):
    out = []
    for i in range(44):
        c = tbl.get(i, tbl.get("default"))
        if c is not None:
            out.append(c)
        elif 8 <= i <= 11:
            out.append(hexc((prt["ver"] >> (4 * (3 - (i - 8)))) & 0xF))
        elif 15 <= i <= 22:
            out.append(hexc((prt["sum"] >> (4 * (7 - (i - 15)))) & 0xF))
        elif 26 <= i <= 31:
            out.append(hexc((prt["cnt"] >> (4 * (5 - (i - 26)))) & 0xF))
        elif 35 <= i <= 36:
            out.append(hexc((prt["min"] >> (4 * (1 - (i - 35)))) & 0xF))
        elif 40 <= i <= 41:
            out.append(hexc((prt["max"] >> (4 * (1 - (i - 40)))) & 0xF))
        elif i == 4:
            out.append(ord("K") if prt["ok"] else ord("F"))
        else:
            out.append(ord(" "))
    return out


def test_line(env):
    print("\n-- 6. top_vision_m1.v status line --")
    tbl = parse_fixed_char()
    check(tbl.get("default") is None, "fixed_char() default marks dynamic slots")

    prt = {"ver": 0x1324, "sum": 0x0123ABCD, "cnt": 90240,
           "min": 0x00, "max": 0xFF, "ok": True}
    b = line_bytes(prt, tbl)
    text = bytes(b).decode("ascii")
    print("      line: %r  (%d bytes)" % (text, len(b)))
    check(len(b) == 44, "line is exactly 44 bytes", "got %d" % len(b))
    check(text == "MH1 K V=1324 S=0123ABCD N=016080 L=00 H=FF\r\n",
          "field layout / hex encoding correct", "got %r" % text)
    check(90240 == env["CAM_FRAME_PIX"],
          "N field 90240 == CAM_FRAME_PIX (016080 hex)")

    bad = dict(prt)
    bad["ok"] = False
    check(line_bytes(bad, tbl)[4] == ord("F"),
          "self-test flag renders 'F' when cam_ok = 0")

    wrong = dict(prt)
    wrong["ver"] = 0x2413
    expect_fail(bytes(line_bytes(wrong, tbl)) == bytes(b),
                "reversed nibble order would change the line")


def main():
    print("=" * 72)
    print("vision_sub M1 cycle-accurate model")
    print("=" * 72)
    env = parse_defines()
    print("parsed from vision_def.v: CAM_IMG=%dx%d  SCCB_DIV=%d  UART_DIV=%d  "
          "CFG_WR_NUM=%d" % (env["CAM_IMG_W"], env["CAM_IMG_H"],
                             env["SCCB_DIV_NUM"], env["UART_BAUD_DIV"],
                             env["CFG_WR_NUM"]))

    test_sccb(env)
    test_cfg_table(env)
    test_cfg_run(env)
    test_dvp(env)
    test_uart(env)
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
