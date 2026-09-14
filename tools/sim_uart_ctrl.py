#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_uart_ctrl.py -- cycle-accurate model of the Stage 1 串口屏 control path.

Why this exists
---------------
There is no Verilog simulator on this machine (see the project memory
"no-verilog-simulator-use-python-models"), so every RTL change is proven with a
cycle-accurate Python model before it goes anywhere near the board. This file
models the exact non-blocking behaviour of the three clock domains touched by
the serial-screen feature:

  * clk        (50 MHz)  -- uart_screen_ctrl RX FSM + TJC parser, the
                            mode/marquee override latch, the command->toggle
                            generator, and the brightness merge.
  * sd_card_clk(100 MHz) -- the toggle-CDC synchroniser chain that rebuilds
                            single-cycle cmd_*_pulse_sd pulses for sd_card_bmp.
  * video_clk  (25 MHz)  -- the 2FF synchroniser that carries the override
                            levels into the trans_mode / marquee_en muxes.

The three clocks are run on one merged time axis with deliberate phase offsets
(clk at t=0 mod 20ns, sd_card_clk at t=3 mod 10ns, video_clk at t=7 mod 40ns)
so no two posedges ever coincide. That makes the CDC sampling realistic: a
toggle flipped in the clk domain is captured by sd_card_clk on a genuinely
asynchronous edge, exactly like hardware.

What it verifies (passes A-F)
-----------------------------
  A. RX byte decode at the real divider (CLKS_PER_BIT = 50e6/9600 = 5208):
     a "NEXT" frame arrives as the exact 7 bytes on the wire.
  B. Parser command effects for all eleven commands, plus the exact-length /
     argument guards. MODE and FILT take ONE uppercase hex character and span
     the whole 0..F range; SPED takes ONE decimal digit and spans 1..8 seconds,
     the floor being set by the ~0.83 s band transition rather than by taste.
     Lowercase, 'G', a two-digit argument, and every out-of-range digit on the
     other commands must NOT fire. A rejected frame must not wedge the parser.
  C. Toggle-CDC: every clk-domain command produces exactly ONE sd_card_clk
     pulse (no loss, no double), and IMGX / SPED each carry the right value on
     their own data+toggle pair.
  D. Override mux + brightness merge + the FILT/FONT/MUSC crossing chains:
       - no screen command  -> trans_mode == ~sw and marquee_en == sw4,
                               bit-identical to the verified baseline;
       - MODE n / MARQ n    -> override wins, including the codes 8..15 that
                               the 3-bit physical branch can never produce;
       - physical sw change -> override cleared, physical path reclaims;
       - BRGT / BRUP / key3 -> brightness set, cycle-wrap, and OR-merge;
       - FILT n / FONT n    -> clk latch, bare 2FF into video_clk, then a
                               frame-atomic stage that only advances on
                               video_frame_start, so a mid-frame switch can
                               never tear the picture into a processed top
                               half and an unprocessed bottom half.
                               trans_mode / marquee_en stay bare 2FF.
       - MUSC n             -> clk latch into music_en, then a BARE 2FF into
                               both video_clk (the audio output mux and the
                               OSD's I_asrc) and sd_card_clk (sd_card_bmp's
                               music_req). Deliberately not frame-atomic: both
                               audio sources emit a continuous audio_valid
                               stream, so switching mid-frame costs at most one
                               sample of phase discontinuity, and gating it
                               would only delay the music_req withdrawal by a
                               frame and waste a sector read.
  E. Negative controls (the project mandates these):
       - only two 0xFF       -> no dispatch;
       - unknown keyword     -> no effect;
       - "NEXTX" (clen==5)   -> rejected by the exact-length guard;
       - half frame "MO"+FFF then a valid "NEXT" -> no false fire, later works;
       - full retreat: zero commands ever sent -> outputs track sw exactly,
         not one spurious pulse is emitted, filt_frame/font_frame stay 0
         (passthrough / flat glyphs), the audio source stays at its
         AUDIO_SRC_DEFAULT reset value in both domains, the carousel interval
         stays at AUTO_SEC_DEFAULT in the sd domain (asserted for two different
         parameter values, so it is the parameter being proved and not a
         coincidence of the reset value happening to be 1), and the physical
         trans_mode branch never exceeds 7 despite the 4-bit wire -- the
         property that keeps the new transitions serial-port-only.
  F. Link-debug registers behind LEDs A4/A3/C10 (uart_tx has no readback, so
     these are the only window onto the link). They must be observation-only:
     zero traffic leaves both at reset and dispatches nothing. Then the three
     bring-up signatures must be distinguishable -- bytes but no terminator
     (toggle lit, terminator flag dark), exactly two 0xFF (terminator flag lit,
     still no dispatch), and a clean frame (all three layers respond). Also
     locks two framing properties the screen project relies on: contiguous
     frames with no idle gap both dispatch, and a single inter-frame 0x20
     sacrifices exactly the next frame and then self-heals.

Honest limits
-------------
  * Passes B-F shrink CLKS_PER_BIT to 16 to run fast. The RX FSM and parser are
     parameter-independent (CPB only sets bit width; a constant 2-cycle
     synchroniser latency shifts every sample point uniformly and stays inside
     each bit window for any CPB > 4), so this is a legitimate speed-up. Pass A
     re-runs the real 5208 divider end-to-end to prove it.
  * The OR-merge inside sd_card_bmp (key_next_press || cmd_next_pulse) and the
     img_loaded_count gate on IMGX are NOT modelled end-to-end -- this harness
     proves the pulses and values that FEED that logic are correct, which is the
     CDC-sensitive part. The OR itself is a one-line `||`.
  * The physical key debouncers (key_press_debounce) are not modelled; key3 is
     injected directly as a clk-domain pulse to exercise the brightness merge.

Usage
-----
    python tools/sim_uart_ctrl.py            # run all passes, print PASS/FAIL
    python tools/sim_uart_ctrl.py --verbose  # extra per-command tracing
Exit code is 0 on success, 1 if any check fails.
"""

import argparse
import sys

# ---------------------------------------------------------------------------
# clock geometry (nanoseconds). Offsets guarantee no two posedges coincide.
# ---------------------------------------------------------------------------
CLK_PERIOD = 20      # 50 MHz
SD_PERIOD = 10       # 100 MHz
VID_PERIOD = 40      # 25 MHz
CLK_OFF = 0
SD_OFF = 3
VID_OFF = 7

RX_IDLE, RX_START, RX_DATA, RX_STOP = 0, 1, 2, 3
CPB_FAST = 16                 # shrunk divider for the fast passes
CPB_REAL = 50_000_000 // 9600  # 5208, what actually ships

# The single-character argument alphabet for MODE and FILT. Uppercase only:
# the RTL's c5_is_hex excludes 'a'-'f', and Pass B asserts that.
HEX_DIGITS = "0123456789ABCDEF"

# SPED's accepted argument alphabet: the carousel interval in whole seconds.
# The floor is 1 s, not 0, because the longest transition has to finish before
# the next picture change is asked for -- a band effect runs WIPE_HOLD 40 +
# FADE_IN 8 + WIPE_SETTLE 2 = 50 frames, about 0.83 s at 60 Hz, so the shipped
# 1 s interval already leaves only ~0.17 s of still picture. Anything faster
# would chain half-finished transitions into a continuous sweep. Pass B asserts
# that both '0' and '9' are rejected.
SPEED_DIGITS = "12345678"


# Two identical uart_screen_ctrl instances now feed one merge point, so the model
# instantiates a whole RX + parser register set per port. These are the names.
RX_KEYS = ('rx_sync', 'rx_state', 'rx_cnt', 'rx_bit', 'rx_shift', 'rx_byte',
           'rx_valid')
FRM_KEYS = ('c0', 'c1', 'c2', 'c3', 'c4', 'c5', 'clen', 'ffc',
            'dbg_rx_toggle', 'dbg_rx_ff')
PULSE_KEYS = ('cmd_next_pulse', 'cmd_auto_pulse', 'cmd_bright_cycle_pulse')
# value + one-clock strobe. The strobe name does not always match the value name
# (cmd_bright_set / cmd_bright_set_v), so the pairs are spelled out.
VAL_PAIRS = (('cmd_bright_set', 'cmd_bright_set_v', 3),
             ('cmd_mode', 'cmd_mode_set', 4),
             ('cmd_marquee', 'cmd_marquee_set', 1),
             ('cmd_img_sel', 'cmd_img_sel_set', 2),
             ('cmd_filt', 'cmd_filt_set', 4),
             ('cmd_font', 'cmd_font_set', 1),
             ('cmd_audio', 'cmd_audio_set', 1),
             ('cmd_speed', 'cmd_speed_set', 4))
PORTS = ('j1_', 'pc_')


def frame(cmd_str):
    """ASCII bytes of a command + the TJC three-byte terminator."""
    return [ord(c) for c in cmd_str] + [0xFF, 0xFF, 0xFF]


def build_rx_wave(byte_list, cpb, idle_before=8, idle_between=2, idle_after=48):
    """
    Expand a byte list into per-clk-cycle uart_rx pin levels: idle high, then
    for each byte a low start bit, 8 LSB-first data bits, a high stop bit (each
    cpb clk cycles wide), with idle gaps between bytes and at the end.
    """
    segs = [(1, idle_before)]
    for i, b in enumerate(byte_list):
        segs.append((0, cpb))                       # start bit
        for k in range(8):
            segs.append(((b >> k) & 1, cpb))        # data bits, LSB first
        segs.append((1, cpb))                       # stop bit
        if i != len(byte_list) - 1:
            segs.append((1, idle_between))
    segs.append((1, idle_after))
    levels = []
    for lvl, n in segs:
        levels.extend([lvl] * n)
    return levels


def _inst_regs():
    """
    Register set of ONE uart_screen_ctrl instance, unprefixed. Both command
    ports get a copy, so this is the single definition of what an instance holds
    and the parser only ever touches these names through a port prefix.
    """
    d = {k: 0 for k in RX_KEYS + FRM_KEYS + PULSE_KEYS}
    for val, strobe, _bits in VAL_PAIRS:
        d[val] = 0
        d[strobe] = 0
    d['rx_sync'] = 0b11
    d['rx_state'] = RX_IDLE
    return d


INST_KEYS = tuple(_inst_regs())


class System(object):
    """Mirrors every register of the three clock domains in the RTL."""

    def __init__(self, cpb=CPB_FAST, sw=0xF, audio_default=0, auto_sec_default=1,
                 pc_cmd_enable=1):
        self.cpb = cpb
        self.sw = sw                # physical DIP: sw[2:0]=SW1-3, sw[3]=SW4
        self.audio_default = audio_default   # top's AUDIO_SRC_DEFAULT parameter
        self.auto_sec_default = auto_sec_default   # top's AUTO_SEC_DEFAULT
        # top's PC_CMD_ENABLE: 0 must leave the Type-C bundle with zero effect AND
        # zero loads, which is what makes it a real one-line retreat.
        self.pc_cmd_enable = pc_cmd_enable
        # Negative control only: replace the RTL's priority mux with a bitwise
        # OR and watch a value appear that neither port ever sent. Nothing on the
        # happy path constructs a System with this set.
        self.merge_naive = 0
        self.key3_press = 0         # one-shot clk-domain pulse (injectable)
        self.vs_pin = 1             # video_clk vsync level (injectable, idles high)
        self.rx_levels = []         # J1 / D14
        self.rx_levels_pc = []      # Type-C / F12
        self.clk_n = 0
        self.sd_n = 0
        self.vid_n = 0
        self.st = self._reset_state()
        self.log = {'next': 0, 'auto': 0, 'bright_cycle': 0,
                    'bright_set': [], 'mode_set': [], 'marquee_set': [],
                    'img_set': [], 'filt_set': [], 'font_set': [],
                    'audio_set': [], 'speed_set': [], 'rx': [], 'rx_pc': []}
        self.sdlog = {'next': 0, 'auto': 0, 'img': 0, 'img_vals': [],
                      'speed': 0, 'speed_vals': []}

    def _reset_state(self):
        s = {}
        # --- clk domain: two uart_screen_ctrl instances, RX + parser, one per
        #     command port (J1 / D14 and Type-C / F12). ---
        for pfx in PORTS:
            for k, v in _inst_regs().items():
                s[pfx + k] = v
        # --- top's merged command wires. Not registers: pure functions of the
        #     two instances above, recomputed by _merge() at the start of every
        #     clk edge. Seeded here only so the keys exist before the first one.
        for k in PULSE_KEYS:
            s[k] = 0
        for val, strobe, _bits in VAL_PAIRS:
            s[val] = 0
            s[strobe] = 0
        # Same treatment for the debug taps: led[0]/led[1] show the J1
        # instance's raw byte/terminator observers, led[3] the PC one's.
        s['dbg_rx_toggle'] = 0
        s['dbg_rx_ff'] = 0
        s['dbg_pc_cmd_toggle'] = 0
        # --- clk domain: brightness + override latch + toggle gen ---
        s['brightness'] = 2
        s['sw_c0'] = s['sw_c1'] = s['sw_c2'] = 7
        s['sw4_c0'] = s['sw4_c1'] = s['sw4_c2'] = 1
        s['mode_ovr_val'] = 0
        s['mode_ovr_en'] = 0
        s['marq_ovr_val'] = 1
        s['marq_ovr_en'] = 0
        s['filt_val'] = 0           # FILT latch, 0 = passthrough
        s['font_val'] = 0           # FONT latch, 0 = flat glyphs
        s['music_en'] = self.audio_default   # MUSC latch, no DIP reclaim
        s['next_tgl'] = 0
        s['auto_tgl'] = 0
        s['img_tgl'] = 0
        s['img_sel_lat'] = 0
        s['spd_tgl'] = 0
        # Latched argument for the data+toggle crossing. It resets to the top's
        # AUTO_SEC_DEFAULT so that "no command was ever sent" is observable as a
        # real value on the sd side rather than as a don't-care.
        s['speed_lat'] = self.auto_sec_default
        # --- sd_card_clk domain ---
        s['next_tgl_s0'] = s['next_tgl_s1'] = s['next_tgl_s2'] = 0
        s['auto_tgl_s0'] = s['auto_tgl_s1'] = s['auto_tgl_s2'] = 0
        s['img_tgl_s0'] = s['img_tgl_s1'] = s['img_tgl_s2'] = 0
        s['img_sel_s0'] = s['img_sel_s1'] = 0
        s['spd_tgl_s0'] = s['spd_tgl_s1'] = s['spd_tgl_s2'] = 0
        s['speed_s0'] = s['speed_s1'] = self.auto_sec_default
        s['music_en_s0'] = s['music_en_s1'] = self.audio_default
        # --- video_clk domain ---
        s['sw_v0'] = s['sw_v1'] = 7
        s['sw4_v0'] = s['sw4_v1'] = 1
        s['mode_ovr_val_v0'] = s['mode_ovr_val_v1'] = 0
        s['mode_ovr_en_v0'] = s['mode_ovr_en_v1'] = 0
        s['marq_ovr_val_v0'] = s['marq_ovr_val_v1'] = 1
        s['marq_ovr_en_v0'] = s['marq_ovr_en_v1'] = 0
        s['filt_val_v0'] = s['filt_val_v1'] = 0
        s['font_val_v0'] = s['font_val_v1'] = 0
        s['music_en_v0'] = s['music_en_v1'] = self.audio_default
        s['filt_frame'] = 0         # frame-atomic: what the pixels actually see
        s['font_frame'] = 0
        s['vs_d'] = 0               # video_frame_start = vs_d & ~vs
        return s

    # -- combinational outputs -------------------------------------------
    def trans_mode(self):
        # 4 bits wide in the RTL now, but the physical branch is {1'b0, ~sw_v1}
        # so it still yields exactly 0..7: codes 8..15 can only arrive over the
        # serial port. That is deliberate -- the verified DIP path is unchanged.
        return self.st['mode_ovr_val_v1'] if self.st['mode_ovr_en_v1'] \
            else ((~self.st['sw_v1']) & 7)

    def marquee_en(self):
        return self.st['marq_ovr_val_v1'] if self.st['marq_ovr_en_v1'] \
            else self.st['sw4_v1']

    def filt_out(self):
        return self.st['filt_frame']

    def font_out(self):
        return self.st['font_frame']

    def audio_sel_v(self):
        """video_clk view: selects the audio output mux and drives OSD I_asrc."""
        return self.st['music_en_v1']

    def music_req_sd(self):
        """sd_card_clk view: sd_card_bmp's music_req, gates audio_phase arming."""
        return self.st['music_en_s1']

    def cmd_next_pulse_sd(self):
        return self.st['next_tgl_s1'] ^ self.st['next_tgl_s2']

    def cmd_auto_pulse_sd(self):
        return self.st['auto_tgl_s1'] ^ self.st['auto_tgl_s2']

    def cmd_img_sel_pulse_sd(self):
        return self.st['img_tgl_s1'] ^ self.st['img_tgl_s2']

    def cmd_speed_pulse_sd(self):
        return self.st['spd_tgl_s1'] ^ self.st['spd_tgl_s2']

    def speed_sd(self):
        """sd_card_clk view: the interval in seconds sd_card_bmp would latch."""
        return self.st['speed_s1']

    def j1(self, name):
        """
        A register inside the J1 instance, by its unprefixed RTL name.

        Needed for anything that asserts on a HELD command value: top's merged
        cmd_mode / cmd_filt / cmd_speed wires are muxed outputs, not latches, so
        they collapse to the other port's value the moment their strobe drops.
        Reading them after a frame settles would report 0, not the last command.
        """
        return self.st['j1_' + name]

    def pc(self, name):
        """Same, for the Type-C instance."""
        return self.st['pc_' + name]

    # -- the one place two command sources become one ----------------------
    def _merge(self, st):
        """
        top_tf_hdmi_audio's merge point, for the whole cmd_* bundle plus the
        debug taps. Reads only the per-port instance registers, so it is a pure
        function of Q and may be re-applied whenever a consumer needs the wire
        view -- which in the RTL is always, because they ARE wires.

        STROBES may be OR-ed: two sources firing the same one-clock pulse is
        harmless, each consumer treats a pulse as "advance once".
        VALUES may NOT. cmd_mode is a value paired with a one-clock cmd_mode_set
        strobe, so OR-ing the two buses yields a code neither source ever sent
        (MODE 1 | MODE E = 0xF). The RTL muxes them with J1 first; merge_naive
        flips exactly that mux to an OR so the failure is demonstrated rather
        than argued.

        pc_cmd_enable gates the values as well as the strobes. Gating only the
        strobes would leave `j1_set ? j1_val : pc_val` reading pc_val on every
        cycle the J1 strobe is low -- one un-removed load, and the synthesiser
        keeps the whole second instance alive (see the led[3] incident in
        README). This mirrors that requirement instead of hoping for pruning.
        """
        en = self.pc_cmd_enable
        out = {}
        for k in PULSE_KEYS:
            out[k] = (st['j1_' + k] | (st['pc_' + k] & en)) & 1
        for val, strobe, bits in VAL_PAIRS:
            mask = (1 << bits) - 1
            js = st['j1_' + strobe]
            ps = st['pc_' + strobe] & en
            jv = st['j1_' + val]
            pv = (st['pc_' + val] & mask) if en else 0
            out[strobe] = js | ps
            out[val] = ((jv | pv) if self.merge_naive
                        else (jv if js else pv)) & mask
        out['dbg_rx_toggle'] = st['j1_dbg_rx_toggle']
        out['dbg_rx_ff'] = st['j1_dbg_rx_ff']
        out['dbg_pc_cmd_toggle'] = st['pc_dbg_rx_toggle'] & en
        return out

    # -- clock-edge handlers (non-blocking: read old, write nxt) ---------
    def _step_rx_parser(self, s, nxt, pfx, rx_pin):
        """
        One uart_screen_ctrl instance: RX FSM + framed-ASCII parser.

        The port's registers are copied out unprefixed and written back through
        nxt with the prefix, so the body below stays a literal transcription of
        uart_screen_ctrl.v rather than a re-typing of it with 'j1_' / 'pc_'
        pasted into every name. The two instances are identical RTL, so the two
        models must be character-identical too -- that is the whole reason this
        is a function of pfx instead of a copy-paste.

        Registers that hold their value across an edge need no explicit line any
        more: the copy starts from the old state, so w[k] already IS the hold.
        Only the strobe-clearing default block and real transitions write.
        """
        r = {k: s[pfx + k] for k in INST_KEYS}
        w = dict(r)

        # ---- RX FSM ----
        rx_in = (r['rx_sync'] >> 1) & 1
        w['rx_sync'] = ((r['rx_sync'] & 1) << 1) | rx_pin
        rx_state_n, rx_cnt_n = r['rx_state'], r['rx_cnt']
        rx_bit_n, rx_shift_n = r['rx_bit'], r['rx_shift']
        rx_byte_n, rx_valid_n = r['rx_byte'], 0
        cpb = self.cpb
        if r['rx_state'] == RX_IDLE:
            rx_cnt_n, rx_bit_n = 0, 0
            if rx_in == 0:
                rx_state_n = RX_START
        elif r['rx_state'] == RX_START:
            if r['rx_cnt'] == (cpb - 1) // 2:
                if rx_in == 0:
                    rx_cnt_n, rx_state_n = 0, RX_DATA
                else:
                    rx_state_n = RX_IDLE
            else:
                rx_cnt_n = r['rx_cnt'] + 1
        elif r['rx_state'] == RX_DATA:
            if r['rx_cnt'] == cpb - 1:
                rx_cnt_n = 0
                rx_shift_n = ((rx_in << 7) | (r['rx_shift'] >> 1)) & 0xFF
                rx_bit_n = (r['rx_bit'] + 1) & 7
                if r['rx_bit'] == 7:
                    rx_state_n = RX_STOP
            else:
                rx_cnt_n = r['rx_cnt'] + 1
        elif r['rx_state'] == RX_STOP:
            if r['rx_cnt'] == cpb - 1:
                rx_cnt_n, rx_state_n = 0, RX_IDLE
                if rx_in == 1:
                    rx_byte_n, rx_valid_n = r['rx_shift'], 1
            else:
                rx_cnt_n = r['rx_cnt'] + 1
        w['rx_state'] = rx_state_n
        w['rx_cnt'] = rx_cnt_n & 0xFFFF
        w['rx_bit'] = rx_bit_n
        w['rx_shift'] = rx_shift_n
        w['rx_byte'] = rx_byte_n
        w['rx_valid'] = rx_valid_n

        # ---- parser (consumes the OLD rx_valid / rx_byte) ----
        # mirrors the RTL's "default: every strobe/pulse is high for one clock
        # only": the held value buses and the frame registers fall out of the
        # copy above, so clearing these is all that is left.
        for k in ('cmd_next_pulse', 'cmd_auto_pulse', 'cmd_bright_cycle_pulse',
                  'cmd_bright_set_v', 'cmd_mode_set', 'cmd_marquee_set',
                  'cmd_img_sel_set', 'cmd_filt_set', 'cmd_font_set',
                  'cmd_audio_set', 'cmd_speed_set'):
            w[k] = 0

        if r['rx_valid']:
            b = r['rx_byte']
            # observation-only, mirrors the RTL: driven before the 0xFF test so it
            # cannot perturb ffc / clen / any cmd_* effect.
            w['dbg_rx_toggle'] = r['dbg_rx_toggle'] ^ 1
            w['dbg_rx_ff'] = 1 if b == 0xFF else 0
            if b == 0xFF:
                if r['ffc'] == 2:
                    w['ffc'] = 0
                    w['clen'] = 0
                    key = bytes([r['c0'], r['c1'], r['c2'], r['c3']])
                    clen, c4, c5 = r['clen'], r['c4'], r['c5']
                    # c5_is_hex / c5_hex mirror the RTL wires of the same name.
                    # Lowercase is deliberately outside the accepted set.
                    c5_is_hex = (0x30 <= c5 <= 0x39) or (0x41 <= c5 <= 0x46)
                    c5_hex = (c5 - 0x30) if c5 <= 0x39 else (c5 - 0x41) + 10
                    if key == b"NEXT" and clen == 4:
                        w['cmd_next_pulse'] = 1
                    elif key == b"AUTO" and clen == 4:
                        w['cmd_auto_pulse'] = 1
                    elif key == b"BRUP" and clen == 4:
                        w['cmd_bright_cycle_pulse'] = 1
                    elif key == b"BRGT" and clen == 6 and c4 == 0x20 \
                            and 0x30 <= c5 <= 0x34:
                        w['cmd_bright_set'] = c5 - 0x30
                        w['cmd_bright_set_v'] = 1
                    elif key == b"MODE" and clen == 6 and c4 == 0x20 and c5_is_hex:
                        w['cmd_mode'] = c5_hex
                        w['cmd_mode_set'] = 1
                    elif key == b"MARQ" and clen == 6 and c4 == 0x20 \
                            and c5 in (0x30, 0x31):
                        w['cmd_marquee'] = 1 if c5 == 0x31 else 0
                        w['cmd_marquee_set'] = 1
                    elif key == b"IMGX" and clen == 6 and c4 == 0x20 \
                            and 0x31 <= c5 <= 0x34:
                        w['cmd_img_sel'] = (c5 - 0x30 - 1) & 3
                        w['cmd_img_sel_set'] = 1
                    elif key == b"FILT" and clen == 6 and c4 == 0x20 and c5_is_hex:
                        w['cmd_filt'] = c5_hex
                        w['cmd_filt_set'] = 1
                    elif key == b"FONT" and clen == 6 and c4 == 0x20 \
                            and c5 in (0x30, 0x31):
                        w['cmd_font'] = 1 if c5 == 0x31 else 0
                        w['cmd_font_set'] = 1
                    elif key == b"MUSC" and clen == 6 and c4 == 0x20 \
                            and c5 in (0x30, 0x31):
                        w['cmd_audio'] = 1 if c5 == 0x31 else 0
                        w['cmd_audio_set'] = 1
                    elif key == b"SPED" and clen == 6 and c4 == 0x20 \
                            and 0x31 <= c5 <= 0x38:
                        w['cmd_speed'] = (c5 - 0x30) & 15
                        w['cmd_speed_set'] = 1
                else:
                    w['ffc'] = (r['ffc'] + 1) & 3
            else:
                w['ffc'] = 0
                if r['clen'] < 6:
                    w['c%d' % r['clen']] = b
                if r['clen'] < 15:
                    w['clen'] = r['clen'] + 1

        for k, v in w.items():
            nxt[pfx + k] = v

    def _do_clk(self):
        s = self.st
        n = self.clk_n
        nxt = {}
        # The merged cmd_* bundle is combinational, so every consumer below must
        # see it as a function of THIS cycle's Q -- never of nxt. Refreshing it
        # once here is what keeps the ~40 consumer lines below byte-identical to
        # the single-port model.
        s.update(self._merge(s))
        self._step_rx_parser(s, nxt, 'j1_',
                             self.rx_levels[n] if n < len(self.rx_levels) else 1)
        self._step_rx_parser(s, nxt, 'pc_',
                             self.rx_levels_pc[n]
                             if n < len(self.rx_levels_pc) else 1)

        # ---- override latch (OLD cmd_*_set, OLD sw_c*, live sw) ----
        nxt['sw_c0'] = self.sw & 7
        nxt['sw_c1'] = s['sw_c0']
        nxt['sw_c2'] = s['sw_c1']
        nxt['sw4_c0'] = (self.sw >> 3) & 1
        nxt['sw4_c1'] = s['sw4_c0']
        nxt['sw4_c2'] = s['sw4_c1']
        nxt['mode_ovr_val'] = s['mode_ovr_val']
        nxt['mode_ovr_en'] = s['mode_ovr_en']
        nxt['marq_ovr_val'] = s['marq_ovr_val']
        nxt['marq_ovr_en'] = s['marq_ovr_en']
        if s['cmd_mode_set']:
            nxt['mode_ovr_val'] = s['cmd_mode']
            nxt['mode_ovr_en'] = 1
        elif s['sw_c1'] != s['sw_c2']:
            nxt['mode_ovr_en'] = 0
        if s['cmd_marquee_set']:
            nxt['marq_ovr_val'] = s['cmd_marquee']
            nxt['marq_ovr_en'] = 1
        elif s['sw4_c1'] != s['sw4_c2']:
            nxt['marq_ovr_en'] = 0

        # ---- FILT/FONT/MUSC value latch: own block in the RTL, no ovr_en, no
        #      DIP-reclaim (no physical control is free for these three) ----
        nxt['filt_val'] = s['cmd_filt'] if s['cmd_filt_set'] else s['filt_val']
        nxt['font_val'] = s['cmd_font'] if s['cmd_font_set'] else s['font_val']
        nxt['music_en'] = s['cmd_audio'] if s['cmd_audio_set'] else s['music_en']

        # ---- toggle generator (OLD cmd pulses) ----
        nxt['next_tgl'] = s['next_tgl']
        nxt['auto_tgl'] = s['auto_tgl']
        nxt['img_tgl'] = s['img_tgl']
        nxt['img_sel_lat'] = s['img_sel_lat']
        nxt['spd_tgl'] = s['spd_tgl']
        nxt['speed_lat'] = s['speed_lat']
        if s['cmd_next_pulse']:
            nxt['next_tgl'] = 1 - s['next_tgl']
        if s['cmd_auto_pulse']:
            nxt['auto_tgl'] = 1 - s['auto_tgl']
        if s['cmd_img_sel_set']:
            nxt['img_sel_lat'] = s['cmd_img_sel']
            nxt['img_tgl'] = 1 - s['img_tgl']
        if s['cmd_speed_set']:
            nxt['speed_lat'] = s['cmd_speed']
            nxt['spd_tgl'] = 1 - s['spd_tgl']

        # ---- brightness merge (OLD strobes, live key3) ----
        bright_n = s['brightness']
        if s['cmd_bright_set_v']:
            bright_n = s['cmd_bright_set']
        elif self.key3_press or s['cmd_bright_cycle_pulse']:
            bright_n = 0 if s['brightness'] == 4 else s['brightness'] + 1
        nxt['brightness'] = bright_n

        # ---- commit ----
        s.update(nxt)
        self.key3_press = 0     # consume the one-shot

        # ---- refresh the merged wires from the NEW Q, then log off them.
        #      The consumers above read merge(Q-before-edge), which is exactly
        #      what the RTL's always blocks see; this second pass is what an
        #      observer attached to the wires would measure right now. With
        #      traffic on J1 alone it reproduces the values the single-port
        #      model logged straight out of nxt -- that equivalence is the
        #      regression gate for splitting the instance in two.
        s.update(self._merge(s))
        if nxt['j1_rx_valid']:
            self.log['rx'].append(nxt['j1_rx_byte'])
        if nxt['pc_rx_valid']:
            self.log['rx_pc'].append(nxt['pc_rx_byte'])
        if s['cmd_next_pulse']:
            self.log['next'] += 1
        if s['cmd_auto_pulse']:
            self.log['auto'] += 1
        if s['cmd_bright_cycle_pulse']:
            self.log['bright_cycle'] += 1
        if s['cmd_bright_set_v']:
            self.log['bright_set'].append(s['cmd_bright_set'])
        if s['cmd_mode_set']:
            self.log['mode_set'].append(s['cmd_mode'])
        if s['cmd_marquee_set']:
            self.log['marquee_set'].append(s['cmd_marquee'])
        if s['cmd_img_sel_set']:
            self.log['img_set'].append(s['cmd_img_sel'])
        if s['cmd_filt_set']:
            self.log['filt_set'].append(s['cmd_filt'])
        if s['cmd_font_set']:
            self.log['font_set'].append(s['cmd_font'])
        if s['cmd_audio_set']:
            self.log['audio_set'].append(s['cmd_audio'])
        if s['cmd_speed_set']:
            self.log['speed_set'].append(s['cmd_speed'])

    def _do_sd(self):
        s = self.st
        # detect pulses from the PRE-edge s1^s2 (what sd_card_bmp would see)
        if s['next_tgl_s1'] ^ s['next_tgl_s2']:
            self.sdlog['next'] += 1
        if s['auto_tgl_s1'] ^ s['auto_tgl_s2']:
            self.sdlog['auto'] += 1
        if s['img_tgl_s1'] ^ s['img_tgl_s2']:
            self.sdlog['img'] += 1
            self.sdlog['img_vals'].append(s['img_sel_s1'])
        if s['spd_tgl_s1'] ^ s['spd_tgl_s2']:
            self.sdlog['speed'] += 1
            self.sdlog['speed_vals'].append(s['speed_s1'])
        nxt = {
            'next_tgl_s0': s['next_tgl'], 'next_tgl_s1': s['next_tgl_s0'],
            'next_tgl_s2': s['next_tgl_s1'],
            'auto_tgl_s0': s['auto_tgl'], 'auto_tgl_s1': s['auto_tgl_s0'],
            'auto_tgl_s2': s['auto_tgl_s1'],
            'img_tgl_s0': s['img_tgl'], 'img_tgl_s1': s['img_tgl_s0'],
            'img_tgl_s2': s['img_tgl_s1'],
            'img_sel_s0': s['img_sel_lat'], 'img_sel_s1': s['img_sel_s0'],
            'spd_tgl_s0': s['spd_tgl'], 'spd_tgl_s1': s['spd_tgl_s0'],
            'spd_tgl_s2': s['spd_tgl_s1'],
            'speed_s0': s['speed_lat'], 'speed_s1': s['speed_s0'],
            'music_en_s0': s['music_en'], 'music_en_s1': s['music_en_s0'],
        }
        s.update(nxt)

    def _do_vid(self):
        s = self.st
        nxt = {
            'sw_v0': self.sw & 7, 'sw_v1': s['sw_v0'],
            'sw4_v0': (self.sw >> 3) & 1, 'sw4_v1': s['sw4_v0'],
            'mode_ovr_val_v0': s['mode_ovr_val'],
            'mode_ovr_val_v1': s['mode_ovr_val_v0'],
            'mode_ovr_en_v0': s['mode_ovr_en'],
            'mode_ovr_en_v1': s['mode_ovr_en_v0'],
            'marq_ovr_val_v0': s['marq_ovr_val'],
            'marq_ovr_val_v1': s['marq_ovr_val_v0'],
            'marq_ovr_en_v0': s['marq_ovr_en'],
            'marq_ovr_en_v1': s['marq_ovr_en_v0'],
            'filt_val_v0': s['filt_val'],
            'filt_val_v1': s['filt_val_v0'],
            'font_val_v0': s['font_val'],
            'font_val_v1': s['font_val_v0'],
            # music_en is a bare 2FF, NOT frame-atomic: both audio sources emit
            # a continuous audio_valid stream, so a switch costs at most one
            # sample of phase discontinuity and the ACR reference never breaks.
            'music_en_v0': s['music_en'],
            'music_en_v1': s['music_en_v0'],
            'vs_d': self.vs_pin,
            # frame-atomic: video_frame_start = OLD vs_d & ~live vs
            'filt_frame': s['filt_val_v1'] if (s['vs_d'] and not self.vs_pin)
                          else s['filt_frame'],
            'font_frame': s['font_val_v1'] if (s['vs_d'] and not self.vs_pin)
                          else s['font_frame'],
        }
        s.update(nxt)

    # -- time advance ----------------------------------------------------
    def run_clk(self, target_n):
        """Process edges in strict time order until clk_n reaches target_n."""
        while self.clk_n < target_n:
            tc = CLK_OFF + CLK_PERIOD * self.clk_n
            ts = SD_OFF + SD_PERIOD * self.sd_n
            tv = VID_OFF + VID_PERIOD * self.vid_n
            m = min(tc, ts, tv)
            if m == tc:
                self._do_clk(); self.clk_n += 1
            elif m == ts:
                self._do_sd(); self.sd_n += 1
            else:
                self._do_vid(); self.vid_n += 1

    def send(self, byte_list, cpb=None, extra=120, port='j1'):
        """
        Push a frame onto one command wire and run enough cycles to fully
        drain it.

        port selects which uart_screen_ctrl instance sees the bytes: 'j1'
        (D14, the serial screen) or 'pc' (F12, the Type-C CH340). It defaults to
        'j1' so every pass written before the second source existed keeps
        driving exactly the port it drove then.
        """
        if cpb is None:
            cpb = self.cpb
        levels = self.rx_levels if port == 'j1' else self.rx_levels_pc
        # The level vectors are indexed by absolute clk cycle. If settle()/a
        # prior frame already advanced clk_n past the end of this wire, pad with
        # idle-high so the new frame's start bit lands exactly on the next clk
        # edge -- else the RX FSM would sample the middle of the wave and miss
        # it. Both ports must be padded: an index past the end of a short vector
        # reads idle-high, but only if it is not the one being extended.
        for wire in (self.rx_levels, self.rx_levels_pc):
            while len(wire) < self.clk_n:
                wire.append(1)
        levels.extend(build_rx_wave(byte_list, cpb))
        self.run_clk(max(len(self.rx_levels), len(self.rx_levels_pc)) + extra)

    def settle(self, cycles=40):
        """Run idle clk cycles (line stays high) to let CDC chains resolve."""
        self.run_clk(self.clk_n + cycles)

    def frame_boundary(self, low_cycles=4, high_cycles=4):
        """
        Drive one vsync low pulse so video_frame_start = vs_d & ~vs fires
        exactly once. FILT/FONT are frame-atomic in the RTL, so a pass that
        wants a new value to reach the pixels must cross a frame boundary.
        """
        self.vs_pin = 0
        self.run_clk(self.clk_n + low_cycles)
        self.vs_pin = 1
        self.run_clk(self.clk_n + high_cycles)


class Result(object):
    def __init__(self):
        self.rows = []
        self.failed = 0

    def add(self, ok, name, detail):
        self.rows.append((ok, name, detail))
        if not ok:
            self.failed += 1
        print("  [%s] %-46s %s" % ("PASS" if ok else "FAIL", name, detail))


# ---------------------------------------------------------------------------
# Pass A -- real divider byte decode
# ---------------------------------------------------------------------------
def pass_a(res, verbose):
    print("=" * 78)
    print("A. RX decode at the real divider CLKS_PER_BIT = %d (50 MHz / 9600)" % CPB_REAL)
    print("=" * 78)
    sysA = System(cpb=CPB_REAL)
    sysA.send(frame("NEXT"), extra=200)
    want = frame("NEXT")
    got = sysA.log['rx']
    res.add(got == want, "real-CPB byte stream",
            "decoded %d bytes %s" % (len(got), got))
    res.add(sysA.log['next'] == 1 and sysA.sdlog['next'] == 1,
            "real-CPB NEXT -> 1 clk + 1 sd pulse",
            "clk next=%d, sd next=%d" % (sysA.log['next'], sysA.sdlog['next']))
    if verbose:
        print("      rx bytes:", got)


# ---------------------------------------------------------------------------
# Pass B -- parser command effects and guards
# ---------------------------------------------------------------------------
def pass_b(res, verbose):
    print("=" * 78)
    print("B. Parser command effects (cpb=%d)" % CPB_FAST)
    print("=" * 78)

    # (frame, expected log key, expected value or None for pulse-count==1)
    pulse_cases = [
        ("NEXT", 'next'),
        ("AUTO", 'auto'),
        ("BRUP", 'bright_cycle'),
    ]
    for cmd, key in pulse_cases:
        s = System()
        s.send(frame(cmd))
        fired = s.log[key] == 1
        others = sum(v for k, v in s.log.items()
                     if k in ('next', 'auto', 'bright_cycle') and k != key)
        res.add(fired and others == 0, "%s fires once, nothing else" % cmd,
                "%s=%d, other pulses=%d" % (key, s.log[key], others))

    value_cases = [
        ("BRGT 3", 'bright_set', [3]),
        ("MARQ 0", 'marquee_set', [0]),
        ("MARQ 1", 'marquee_set', [1]),
        ("IMGX 2", 'img_set', [1]),      # picture 2 -> index 1
        ("IMGX 4", 'img_set', [3]),
    ]
    # MODE and FILT each span their whole accepted range now that the argument
    # is a hex character: 0 is the retreat (DIP path / passthrough) and F is the
    # top reserved code, so a mis-set bound or a wrong c5_hex decode shows up
    # here rather than on the board.
    for n in range(16):
        value_cases.append(("MODE %s" % HEX_DIGITS[n], 'mode_set', [n]))
    for n in range(16):
        value_cases.append(("FILT %s" % HEX_DIGITS[n], 'filt_set', [n]))
    # SPED spans 1..8 seconds. 1 is the retreat (bit-identical to the shipped
    # fixed 1 s carousel, because sec_target_m1 == 0 makes sec_last constantly
    # true) and 8 is the inclusive upper bound, so a mis-set guard bound shows
    # up at either end rather than only in the middle of the range.
    for d in SPEED_DIGITS:
        value_cases.append(("SPED %s" % d, 'speed_set', [ord(d) - 0x30]))
    value_cases += [
        ("FONT 0", 'font_set', [0]),
        ("FONT 1", 'font_set', [1]),
        ("MUSC 0", 'audio_set', [0]),
        ("MUSC 1", 'audio_set', [1]),
    ]
    for cmd, key, want in value_cases:
        s = System()
        s.send(frame(cmd))
        res.add(s.log[key] == want, "%s -> %s=%s" % (cmd, key, want),
                "got %s" % s.log[key])

    # guards: out-of-range digits, wrong lengths and wrong case must NOT fire
    guard_cases = [
        ("BRGT 5", 'bright_set', "digit >4 rejected"),
        ("IMGX 0", 'img_set', "digit <1 rejected"),
        ("IMGX 5", 'img_set', "digit >4 rejected"),
        ("MODE 33", 'mode_set', "two-digit arg -> clen==7 rejected"),
        ("FILT 33", 'filt_set', "two-digit arg -> clen==7 rejected"),
        ("FONT 11", 'font_set', "two-digit arg -> clen==7 rejected"),
        ("FONT 2", 'font_set', "digit >1 rejected"),
        ("MODE a", 'mode_set', "lowercase hex rejected"),
        ("MODE f", 'mode_set', "lowercase hex rejected"),
        ("FILT c", 'filt_set', "lowercase hex rejected"),
        ("MODE G", 'mode_set', "'G' is outside '0'-'9'/'A'-'F'"),
        ("FILT G", 'filt_set', "'G' is outside '0'-'9'/'A'-'F'"),
        ("MODE 10", 'mode_set', "decimal 10 as two chars -> clen==7 rejected"),
        ("FILT 10", 'filt_set', "decimal 10 as two chars -> clen==7 rejected"),
        ("MUSC 2", 'audio_set', "digit other than 0/1 rejected"),
        ("MUSC", 'audio_set', "no argument -> clen==4 rejected"),
        ("MUSCX 1", 'audio_set', "five-char keyword -> clen==7 rejected"),
        ("MUSC 11", 'audio_set', "two-digit arg -> clen==7 rejected"),
        ("SPED 0", 'speed_set', "0 s rejected: shorter than the 0.83 s band ramp"),
        ("SPED 9", 'speed_set', "digit >8 rejected"),
        ("SPED", 'speed_set', "no argument -> clen==4 rejected"),
        ("SPED 11", 'speed_set', "two-digit arg -> clen==7 rejected"),
        ("SPEDX 1", 'speed_set', "five-char keyword -> clen==7 rejected"),
        ("sped 1", 'speed_set', "lowercase keyword rejected"),
    ]
    for cmd, key, why in guard_cases:
        s = System()
        s.send(frame(cmd))
        res.add(len(s.log[key]) == 0, "%s silent (%s)" % (cmd, why),
                "%s=%s" % (key, s.log[key]))

    # brightness set actually lands in the register
    s = System()
    s.send(frame("BRGT 4"))
    res.add(s.st['brightness'] == 4, "BRGT 4 sets brightness_level",
            "brightness=%d" % s.st['brightness'])

    # A rejected frame must not wedge the parser: the next valid frame has to
    # dispatch normally, which is what makes a mistyped screen button harmless.
    s = System()
    s.send(frame("MODE a"))
    s.send(frame("MUSC 2"))
    s.settle(40)
    rejected = len(s.log['mode_set']) + len(s.log['audio_set'])
    s.send(frame("MODE C"))
    s.send(frame("MUSC 1"))
    s.settle(40)
    res.add(rejected == 0 and s.log['mode_set'] == [12]
            and s.log['audio_set'] == [1],
            "parser recovers after rejected frames",
            "rejected=%d then MODE C -> %s, MUSC 1 -> %s"
            % (rejected, s.log['mode_set'], s.log['audio_set']))
    if verbose:
        print("      guard cases all silent as expected")


# ---------------------------------------------------------------------------
# Pass C -- toggle-CDC: exactly one sd pulse per command, correct img value
# ---------------------------------------------------------------------------
def pass_c(res, verbose):
    print("=" * 78)
    print("C. Toggle-CDC clk -> sd_card_clk (exactly one pulse, no loss/double)")
    print("=" * 78)

    s = System()
    s.send(frame("NEXT"))
    s.send(frame("NEXT"))
    s.send(frame("NEXT"))
    res.add(s.log['next'] == 3 and s.sdlog['next'] == 3,
            "3x NEXT -> 3 clk, 3 sd pulses",
            "clk=%d sd=%d" % (s.log['next'], s.sdlog['next']))

    s = System()
    s.send(frame("AUTO"))
    s.send(frame("AUTO"))
    res.add(s.log['auto'] == 2 and s.sdlog['auto'] == 2,
            "2x AUTO -> 2 sd pulses",
            "clk=%d sd=%d" % (s.log['auto'], s.sdlog['auto']))

    s = System()
    for n, idx in (("IMGX 1", 0), ("IMGX 3", 2), ("IMGX 4", 3)):
        s.send(frame(n))
    res.add(s.sdlog['img'] == 3 and s.sdlog['img_vals'] == [0, 2, 3],
            "IMGX 1/3/4 -> sd img_vals [0,2,3]",
            "count=%d vals=%s" % (s.sdlog['img'], s.sdlog['img_vals']))

    # SPED crosses on its own data+toggle pair, the same primitive IMGX uses:
    # the value is latched in the clk domain BEFORE the toggle flips, so it is
    # stable for at least two sd_card_clk edges by the time the edge arrives.
    s = System()
    for cmd, sec in (("SPED 2", 2), ("SPED 5", 5), ("SPED 8", 8)):
        s.send(frame(cmd))
    res.add(s.sdlog['speed'] == 3 and s.sdlog['speed_vals'] == [2, 5, 8],
            "SPED 2/5/8 -> sd speed_vals [2,5,8]",
            "count=%d vals=%s" % (s.sdlog['speed'], s.sdlog['speed_vals']))

    # a command must never leak a pulse onto a different channel
    s = System()
    s.send(frame("MODE 2"))
    s.settle()
    res.add(s.sdlog['next'] == 0 and s.sdlog['auto'] == 0 and s.sdlog['img'] == 0
            and s.sdlog['speed'] == 0,
            "MODE leaks no sd pulse",
            "next=%d auto=%d img=%d speed=%d"
            % (s.sdlog['next'], s.sdlog['auto'], s.sdlog['img'], s.sdlog['speed']))

    s = System()
    s.send(frame("SPED 4"))
    s.settle()
    res.add(s.sdlog['next'] == 0 and s.sdlog['auto'] == 0 and s.sdlog['img'] == 0
            and s.sdlog['speed'] == 1,
            "SPED leaks no next/auto/img pulse",
            "next=%d auto=%d img=%d speed=%d"
            % (s.sdlog['next'], s.sdlog['auto'], s.sdlog['img'], s.sdlog['speed']))
    if verbose:
        print("      img_vals:", s.sdlog['img_vals'])


# ---------------------------------------------------------------------------
# Pass D -- override mux + brightness merge + physical reclaim
# ---------------------------------------------------------------------------
def pass_d(res, verbose):
    print("=" * 78)
    print("D. trans_mode / marquee_en override mux, brightness merge")
    print("=" * 78)

    # no command: trans_mode == ~sw across the whole switch range
    ok = True
    detail = []
    for v in range(8):
        s = System(sw=(v | 0b1000))    # sw[3]=1 (SW4 OFF), sw[2:0]=v
        s.settle(60)
        tm = s.trans_mode()
        want = (~v) & 7
        detail.append("%d->%d" % (v, tm))
        if tm != want:
            ok = False
    res.add(ok, "no cmd: trans_mode == ~sw (all 8)", " ".join(detail))

    # no command: marquee_en == sw4
    ok = True
    detail = []
    for sw4 in (0, 1):
        s = System(sw=(0b111 | (sw4 << 3)))
        s.settle(60)
        me = s.marquee_en()
        detail.append("sw4=%d->%d" % (sw4, me))
        if me != sw4:
            ok = False
    res.add(ok, "no cmd: marquee_en == sw4", " ".join(detail))

    # MODE n overrides, and survives while sw is held still
    s = System(sw=0xF)
    s.settle(40)
    s.send(frame("MODE 3"))
    s.settle(60)
    res.add(s.trans_mode() == 3, "MODE 3 overrides ~sw",
            "trans_mode=%d (sw held 111, ~sw would be 0)" % s.trans_mode())

    # MARQ 0 overrides
    s = System(sw=0xF)          # sw4=1 -> marquee shown by default
    s.settle(40)
    s.send(frame("MARQ 0"))
    s.settle(60)
    res.add(s.marquee_en() == 0, "MARQ 0 hides banner over sw4=1",
            "marquee_en=%d" % s.marquee_en())

    # physical sw change reclaims control from the override
    s = System(sw=0xF)
    s.settle(40)
    s.send(frame("MODE 3"))
    s.settle(60)
    pre = s.trans_mode()
    s.sw = 0b1001               # sw[2:0] 7 -> 1, a real DIP movement
    s.settle(60)
    post = s.trans_mode()
    res.add(pre == 3 and post == ((~1) & 7),
            "DIP move clears override, ~sw reclaims",
            "before=%d after=%d (want 6)" % (pre, post))

    # same for marquee, and prove the override is truly gone afterwards
    s = System(sw=0xF)
    s.settle(40)
    s.send(frame("MARQ 0"))
    s.settle(60)
    pre = s.marquee_en()                  # override hides banner while sw4=1
    en_before = s.st['marq_ovr_en_v1']
    s.sw = 0b0111                         # sw[3] 1 -> 0, physical SW4 flip
    s.settle(60)
    en_after = s.st['marq_ovr_en_v1']     # override must be cleared now
    s.sw = 0xF                            # flip SW4 back 0 -> 1
    s.settle(60)
    tracks = s.marquee_en()               # must follow physical (=1), not stale 0
    res.add(pre == 0 and en_before == 1 and en_after == 0 and tracks == 1,
            "SW4 flip clears marquee override, physical reclaims",
            "pre=%d ovr_en %d->%d, after flip-back marquee_en=%d (want 1)"
            % (pre, en_before, en_after, tracks))

    # ---- FILT/FONT: clk latch -> 2FF -> frame-atomic output ----
    s = System(sw=0xF)
    s.settle(40)
    s.send(frame("FILT 4"))
    s.settle(200)                      # plenty of video_clk edges, no vsync pulse
    lat, v1, frm = s.st['filt_val'], s.st['filt_val_v1'], s.filt_out()
    s.frame_boundary()
    after = s.filt_out()
    res.add(lat == 4 and v1 == 4 and frm == 0 and after == 4,
            "FILT 4 latches + 2FF settles, pixels wait for frame start",
            "clk=%d v1=%d filt_frame before=%d after boundary=%d"
            % (lat, v1, frm, after))

    # a mid-frame change must NOT reach the pixels: this is the tear the
    # frame-atomic stage exists to prevent
    s.send(frame("FILT 5"))
    s.settle(400)
    held = s.filt_out()
    s.frame_boundary()
    released = s.filt_out()
    res.add(held == 4 and released == 5,
            "mid-frame FILT 5 held until the next frame start",
            "held=%d (want 4) then released=%d (want 5)" % (held, released))

    # FONT 1/0 across frame boundaries
    s = System(sw=0xF)
    s.settle(40)
    s.send(frame("FONT 1"))
    s.settle(200)
    pre = s.font_out()
    s.frame_boundary()
    on = s.font_out()
    s.send(frame("FONT 0"))
    s.frame_boundary()
    off = s.font_out()
    res.add(pre == 0 and on == 1 and off == 0,
            "FONT 1 -> emboss on at frame start, FONT 0 -> flat again",
            "before=%d on=%d off=%d" % (pre, on, off))

    # ---- MODE codes above 7 survive the 3 -> 4 bit widening ----
    # The physical branch is {1'b0, ~sw_v1} and can never produce these, so if
    # anything between cmd_mode and trans_mode still truncates to 3 bits the
    # new transitions silently come out as the old ones.
    ok = True
    detail = []
    for n in (8, 11, 12, 14, 15):
        s = System(sw=0xF)
        s.settle(40)
        s.send(frame("MODE %s" % HEX_DIGITS[n]))
        s.settle(60)
        tm = s.trans_mode()
        detail.append("%X->%d" % (n, tm))
        if tm != n:
            ok = False
    res.add(ok, "MODE 8/B/C/E/F reach trans_mode untruncated",
            " ".join(detail))

    # ---- MUSC: clk latch -> bare 2FF into video_clk AND sd_card_clk ----
    # Deliberately not frame-atomic. Both audio sources emit a continuous
    # audio_valid stream, so the mux may switch at any video_clk edge; gating
    # it on video_frame_start would only delay sd_card_bmp's music_req
    # withdrawal by a whole frame and waste a sector read.
    s = System(sw=0xF)
    s.settle(60)
    tone_v, tone_sd = s.audio_sel_v(), s.music_req_sd()
    s.send(frame("MUSC 1"))
    s.settle(200)                  # many video_clk edges, but NO vsync pulse
    mus_v, mus_sd = s.audio_sel_v(), s.music_req_sd()
    s.frame_boundary()
    after_v = s.audio_sel_v()
    res.add(tone_v == 0 and tone_sd == 0 and mus_v == 1 and mus_sd == 1
            and after_v == 1,
            "MUSC 1 switches both domains with no frame boundary",
            "reset v/sd=%d/%d, after MUSC 1 (no vsync) v/sd=%d/%d, "
            "after boundary v=%d" % (tone_v, tone_sd, mus_v, mus_sd, after_v))

    s.send(frame("MUSC 0"))
    s.settle(200)
    res.add(s.audio_sel_v() == 0 and s.music_req_sd() == 0,
            "MUSC 0 returns to the test tone in both domains",
            "v=%d sd=%d" % (s.audio_sel_v(), s.music_req_sd()))

    # The one-line retreat: AUDIO_SRC_DEFAULT = 1 must power up in today's
    # already-board-verified "music from power-up" state with zero traffic.
    s = System(sw=0xF, audio_default=1)
    s.settle(120)
    res.add(s.st['music_en'] == 1 and s.audio_sel_v() == 1
            and s.music_req_sd() == 1 and len(s.log['audio_set']) == 0,
            "AUDIO_SRC_DEFAULT=1 retreat powers up on music, zero traffic",
            "music_en=%d v=%d sd=%d, MUSC frames seen=%d"
            % (s.st['music_en'], s.audio_sel_v(), s.music_req_sd(),
               len(s.log['audio_set'])))

    # A rejected MUSC must leave the audio source exactly where it was.
    s = System(sw=0xF, audio_default=1)
    s.settle(60)
    s.send(frame("MUSC 2"))
    s.send(frame("MUSCX 1"))
    s.settle(120)
    res.add(s.audio_sel_v() == 1 and len(s.log['audio_set']) == 0,
            "rejected MUSC leaves the audio source untouched",
            "still %d after MUSC 2 and MUSCX 1" % s.audio_sel_v())

    # contrast: trans_mode is deliberately NOT frame-gated (bare 2FF), and the
    # MODE 3 test above already settled it with no vsync pulse at all.
    res.add(s.trans_mode() == ((~7) & 7),
            "trans_mode stays bare-2FF (no frame gate) alongside",
            "trans_mode=%d with sw held 111" % s.trans_mode())

    # brightness: BRUP wraps 4->0, key3 OR-merges, BRGT priority
    s = System()
    s.send(frame("BRGT 4"))
    b4 = s.st['brightness']
    s.send(frame("BRUP"))
    bwrap = s.st['brightness']
    s.key3_press = 1
    s.settle(1)
    bkey = s.st['brightness']
    res.add(b4 == 4 and bwrap == 0 and bkey == 1,
            "brightness: set 4, BRUP wraps to 0, key3 -> 1",
            "after BRGT4=%d after BRUP=%d after key3=%d" % (b4, bwrap, bkey))
    if verbose:
        print("      override + reclaim verified")


# ---------------------------------------------------------------------------
# Pass E -- negative controls and full retreat
# ---------------------------------------------------------------------------
def pass_e(res, verbose):
    print("=" * 78)
    print("E. Negative controls + retreat (no command == baseline)")
    print("=" * 78)

    # only two 0xFF -> never dispatch
    s = System()
    s.send([ord(c) for c in "NEXT"] + [0xFF, 0xFF])
    s.settle(80)
    res.add(s.log['next'] == 0 and s.sdlog['next'] == 0,
            "two 0xFF only -> no dispatch",
            "clk next=%d sd next=%d" % (s.log['next'], s.sdlog['next']))

    # unknown keyword
    s = System()
    s.send(frame("ZZZZ"))
    s.settle(80)
    total = (s.log['next'] + s.log['auto'] + s.log['bright_cycle']
             + len(s.log['bright_set']) + len(s.log['mode_set'])
             + len(s.log['marquee_set']) + len(s.log['img_set'])
             + len(s.log['filt_set']) + len(s.log['font_set'])
             + len(s.log['audio_set']) + len(s.log['speed_set']))
    res.add(total == 0, "unknown keyword ZZZZ -> nothing",
            "total effects=%d" % total)

    # over-long keyword-only frame "NEXTX" -> clen==5, rejected
    s = System()
    s.send(frame("NEXTX"))
    s.settle(80)
    res.add(s.log['next'] == 0, "NEXTX (clen==5) rejected by length guard",
            "next=%d" % s.log['next'])

    # half frame "MO" + terminator, then a valid NEXT: no false MODE, NEXT works
    s = System()
    s.send([ord('M'), ord('O')] + [0xFF, 0xFF, 0xFF])
    s.settle(40)
    false_mode = len(s.log['mode_set'])
    s.send(frame("NEXT"))
    s.settle(40)
    res.add(false_mode == 0 and s.log['next'] == 1,
            "partial 'MO'+FFF then valid NEXT",
            "false mode_set=%d, later next=%d" % (false_mode, s.log['next']))

    # full retreat: zero commands ever, outputs track sw, no spurious pulse
    ok_track = True
    detail = []
    for v in range(8):
        s = System(sw=(v | 0b1000))
        s.settle(80)
        if s.trans_mode() != ((~v) & 7):
            ok_track = False
        detail.append("%d->%d" % (v, s.trans_mode()))
    s = System(sw=0xF)
    s.settle(400)
    # cross real frame boundaries with zero traffic: the retreat must hold for
    # filt_frame/font_frame too, not just for the combinational muxes
    for _ in range(4):
        s.frame_boundary()
    spur = (s.log['next'] + s.log['auto'] + s.log['bright_cycle']
            + s.sdlog['next'] + s.sdlog['auto'] + s.sdlog['img']
            + s.sdlog['speed']
            + len(s.log['filt_set']) + len(s.log['font_set'])
            + len(s.log['audio_set']) + len(s.log['speed_set']))
    res.add(ok_track, "retreat: trans_mode tracks ~sw with zero traffic",
            " ".join(detail))
    res.add(spur == 0 and s.st['brightness'] == 2,
            "retreat: no spurious pulse, brightness stays at reset 2",
            "spurious=%d brightness=%d" % (spur, s.st['brightness']))

    # The carousel interval retreat. With zero traffic the sd side must already
    # read AUTO_SEC_DEFAULT, and no speed pulse may ever fire -- so "the screen
    # is not connected" and "SPED 1" are the same behaviour, and that behaviour
    # is the shipped fixed 1 s carousel. Checking a second, non-default value of
    # the parameter is what makes this an assertion about the PARAMETER rather
    # than a coincidence of the reset value being 1.
    for dflt in (1, 4):
        p = System(auto_sec_default=dflt)
        p.settle(400)
        for _ in range(4):
            p.frame_boundary()
        res.add(p.speed_sd() == dflt and p.sdlog['speed'] == 0
                and len(p.log['speed_set']) == 0,
                "retreat: zero traffic -> interval = AUTO_SEC_DEFAULT(%d)" % dflt,
                "speed_sd=%d sd pulses=%d clk sets=%d"
                % (p.speed_sd(), p.sdlog['speed'], len(p.log['speed_set'])))
    res.add(s.filt_out() == 0 and s.font_out() == 0,
            "retreat: filt_frame=0 (passthrough), font_frame=0 (flat)",
            "filt_frame=%d font_frame=%d after 4 frame boundaries"
            % (s.filt_out(), s.font_out()))
    res.add(s.audio_sel_v() == 0 and s.music_req_sd() == 0,
            "retreat: audio source stays on the test tone",
            "video_clk=%d sd_card_clk=%d after 4 frame boundaries"
            % (s.audio_sel_v(), s.music_req_sd()))

    # The physical branch is {1'b0, ~sw_v1} on a now-4-bit wire. It must still
    # only ever produce 0..7 -- that is what keeps the seven new transitions
    # reachable from the serial port alone, leaving the verified DIP path
    # bit-for-bit as it was.
    phys = []
    for v in range(8):
        p = System(sw=(v | 0b1000))
        p.settle(80)
        phys.append(p.trans_mode())
    res.add(max(phys) <= 7 and sorted(phys) == list(range(8)),
            "retreat: physical trans_mode never exceeds 7 (4-bit wire)",
            "sw 0..7 -> %s" % phys)
    if verbose:
        print("      negative controls complete")


# Pass F -- link-debug LED registers, observation-only
def pass_f(res, verbose):
    print("=" * 78)
    print("F. Link-debug LEDs (dbg_rx_toggle / dbg_rx_ff are observation-only)")
    print("=" * 78)

    # negative control: idle line, zero traffic -> both indicators stay dark and
    # nothing is dispatched. Proves the debug registers cannot self-trigger.
    s = System()
    s.settle(200)
    res.add(s.st['dbg_rx_toggle'] == 0 and s.st['dbg_rx_ff'] == 0
            and len(s.log['rx']) == 0 and len(s.log['filt_set']) == 0,
            "no traffic -> both debug registers stay at reset",
            "tgl=%d ff=%d rx=%d" % (s.st['dbg_rx_toggle'],
                                    s.st['dbg_rx_ff'], len(s.log['rx'])))

    # a well-formed 9-byte frame: all three layers respond
    s = System()
    s.send(frame("FILT 1"))
    s.settle(80)
    res.add(len(s.log['rx']) == 9 and s.st['dbg_rx_toggle'] == 9 % 2
            and s.st['dbg_rx_ff'] == 1 and s.log['filt_set'] == [1],
            "clean frame -> byte parity odd, terminator seen, frame accepted",
            "rx=%d tgl=%d ff=%d filt_set=%s" % (len(s.log['rx']),
            s.st['dbg_rx_toggle'], s.st['dbg_rx_ff'], s.log['filt_set']))

    # signature 1: bytes arrive but the screen never sent `printh ff ff ff`.
    # Odd byte count so the toggle indicator ends lit, last byte is payload so
    # the terminator indicator is dark. On the board: LED0 blinks, LED1 never
    # lights, LED2 never blinks.
    s = System()
    s.send([ord(c) for c in "FILT 1"] + [ord("X")])
    s.settle(80)
    res.add(len(s.log['rx']) == 7 and s.st['dbg_rx_toggle'] == 1
            and s.st['dbg_rx_ff'] == 0 and len(s.log['filt_set']) == 0,
            "no terminator -> bytes seen, terminator flag dark, no dispatch",
            "rx=%d tgl=%d ff=%d filt_set=%d" % (len(s.log['rx']),
            s.st['dbg_rx_toggle'], s.st['dbg_rx_ff'], len(s.log['filt_set'])))

    # signature 2: exactly two 0xFF. The terminator flag is lit (the last byte
    # really was 0xFF) yet nothing dispatches, because ffc only reaches 2.
    # On the board: LED0 blinks, LED1 lights, LED2 stays dark.
    s = System()
    s.send([ord(c) for c in "FILT 1"] + [0xFF, 0xFF])
    s.settle(80)
    res.add(len(s.log['rx']) == 8 and s.st['dbg_rx_ff'] == 1
            and len(s.log['filt_set']) == 0,
            "two 0xFF -> terminator flag lit but still no dispatch",
            "rx=%d ff=%d filt_set=%d" % (len(s.log['rx']),
            s.st['dbg_rx_ff'], len(s.log['filt_set'])))

    # contiguous frames in ONE send() call, i.e. no idle gap at all between the
    # two terminators and the next start bit. Both must dispatch.
    s = System()
    s.send(frame("FILT 1") + frame("FILT 2"))
    s.settle(80)
    res.add(len(s.log['rx']) == 18 and s.log['filt_set'] == [1, 2]
            and s.j1('cmd_filt') == 2,
            "back-to-back frames with zero gap both dispatch",
            "rx=%d filt_set=%s cmd_filt=%d" % (len(s.log['rx']),
            s.log['filt_set'], s.j1('cmd_filt')))

    # a single inter-frame 0x20 shifts the next frame's keyword into c1..c4, so
    # {c0,c1,c2,c3} == " FIL" and it is silently dropped. Dispatch clears clen
    # unconditionally, so the frame after that one works: exactly one command is
    # lost and the link self-heals.
    s = System()
    s.send(frame("FILT 1") + [0x20] + frame("FILT 2") + frame("FILT 3"))
    s.settle(80)
    res.add(len(s.log['rx']) == 28 and s.log['filt_set'] == [1, 3]
            and s.j1('cmd_filt') == 3,
            "inter-frame 0x20 kills exactly the next frame, then self-heals",
            "rx=%d filt_set=%s cmd_filt=%d" % (len(s.log['rx']),
            s.log['filt_set'], s.j1('cmd_filt')))
    if verbose:
        print("      link-debug LED checks complete")


# ---------------------------------------------------------------------------
# Pass G -- the Type-C second command source and the single merge point
# ---------------------------------------------------------------------------
# top now instantiates uart_screen_ctrl twice and joins the two bundles at one
# place. Three claims have to be demonstrated rather than argued: the second
# source is not a second-class citizen (same commands, same downstream
# effects), the merge cannot invent a command when both ports speak in the same
# clk cycle, and PC_CMD_ENABLE=0 removes that port without touching the
# board-verified J1 path.
G_COMMANDS = [
    "NEXT", "AUTO", "BRUP", "BRGT 3", "MODE 5", "MARQ 1", "IMGX 2",
    "FILT 7", "FONT 1", "MUSC 1", "SPED 6",
]


def drive_both(s, j1_bytes, pc_bytes, extra=120):
    """
    Load one frame onto each command wire and run, with the two bit streams
    starting on the SAME clk cycle.

    build_rx_wave is deterministic and both frames here are the same length, so
    identical byte counts mean identical cycle counts: the two parsers reach the
    third 0xFF in the same clk edge, which is the collision case the priority
    mux exists for. Anything less aligned would test a sequencing accident.
    """
    s.rx_levels = build_rx_wave(j1_bytes, s.cpb)
    s.rx_levels_pc = build_rx_wave(pc_bytes, s.cpb)
    s.run_clk(max(len(s.rx_levels), len(s.rx_levels_pc)) + extra)


def snapshot(s):
    """
    Everything a command is allowed to change, as one comparable value. Two
    Systems fed the same frame on different ports must agree on every element,
    so each check below is an equality proof rather than a list of separately
    hand-written expectations.

    Deliberately excluded: the per-port received-byte logs and the per-port
    command registers. Those are the one thing the two ports CANNOT share --
    which port heard the frame is exactly what differs -- so the checks that
    care about them assert them explicitly.
    """
    return (
        s.log['next'], s.log['auto'], s.log['bright_cycle'],
        tuple(s.log['bright_set']), tuple(s.log['mode_set']),
        tuple(s.log['marquee_set']), tuple(s.log['img_set']),
        tuple(s.log['filt_set']), tuple(s.log['font_set']),
        tuple(s.log['audio_set']), tuple(s.log['speed_set']),
        s.sdlog['next'], s.sdlog['auto'], s.sdlog['img'],
        tuple(s.sdlog['img_vals']), s.sdlog['speed'],
        tuple(s.sdlog['speed_vals']),
        s.trans_mode(), s.marquee_en(), s.filt_out(), s.font_out(),
        s.audio_sel_v(), s.music_req_sd(), s.speed_sd(),
        s.st['brightness'],
    )


def driven(cmd, port):
    """Run one command down one port, across a frame boundary, and snapshot."""
    s = System()
    s.send(frame(cmd), port=port)
    s.settle(60)
    s.frame_boundary()
    s.settle(20)
    return s


def pass_g(res, verbose):
    print("=" * 78)
    print("G. Type-C second command source (merge point, priority, retreat)")
    print("=" * 78)

    # ---- G1: every command behaves identically on either port ----
    for cmd in G_COMMANDS:
        a = driven(cmd, 'j1')
        b = driven(cmd, 'pc')
        # a's PC port and b's J1 port must both be clean, or a shared bug in the
        # idle instance could make two broken ports look equal.
        quiet = (len(a.log['rx_pc']) == 0 and len(b.log['rx']) == 0
                 and a.pc('clen') == 0 and b.j1('clen') == 0
                 and b.j1('rx_valid') == 0 and a.pc('rx_valid') == 0)
        heard = (len(a.log['rx']) == len(frame(cmd))
                 and len(b.log['rx_pc']) == len(frame(cmd)))
        res.add(snapshot(a) == snapshot(b) and quiet and heard,
                "PC '%s' == J1 '%s'" % (cmd, cmd),
                "effects identical, other port silent, %d bytes each"
                % len(frame(cmd)))

    # ---- G2: same-cycle value collision, J1 wins ----
    # MODE 1 on J1 and MODE E on PC dispatch on the same clk edge. The strobe may
    # be OR-ed (one cmd_mode_set either way), the value must not.
    s = System()
    drive_both(s, frame("MODE 1"), frame("MODE E"))
    s.settle(60)
    s.frame_boundary()
    res.add(s.log['mode_set'] == [1] and s.st['mode_ovr_val'] == 1
            and s.trans_mode() == 1,
            "collision: J1 MODE 1 wins over PC MODE E",
            "mode_set=%s ovr=%d trans_mode=%d" % (s.log['mode_set'],
            s.st['mode_ovr_val'], s.trans_mode()))
    # Both frames really did parse -- otherwise G2 would also pass on a merge that
    # silently ignored PC. clen only returns to 0 through the dispatch branch.
    res.add(s.j1('clen') == 0 and s.pc('clen') == 0
            and s.pc('cmd_mode') == 0xE and s.j1('cmd_mode') == 1,
            "collision: both ports dispatched, PC's value survives in its own reg",
            "j1 cmd_mode=%d pc cmd_mode=%X" % (s.j1('cmd_mode'),
            s.pc('cmd_mode')))

    # ---- G3: same-cycle pulse collision fires once, not twice ----
    s = System()
    drive_both(s, frame("NEXT"), frame("NEXT"))
    s.settle(60)
    res.add(s.j1('clen') == 0 and s.pc('clen') == 0,
            "collision: both NEXT frames dispatched",
            "j1 clen=%d pc clen=%d" % (s.j1('clen'), s.pc('clen')))
    # Two merged pulses would flip next_tgl twice (back to 0) and the sd domain
    # would advance the picture twice. One flip is the proof the strobes were
    # coincident rather than merely both present.
    res.add(s.log['next'] == 1 and s.sdlog['next'] == 1
            and s.st['next_tgl'] == 1,
            "collision: NEXT on both ports advances exactly one picture",
            "clk next=%d sd next=%d tgl=%d" % (s.log['next'],
            s.sdlog['next'], s.st['next_tgl']))

    # ---- G4: NEGATIVE CONTROL -- the OR this design replaced ----
    s = System()
    s.merge_naive = 1
    drive_both(s, frame("MODE 1"), frame("MODE E"))
    s.settle(60)
    s.frame_boundary()
    res.add(s.log['mode_set'] == [0xF] and s.trans_mode() == 0xF,
            "NEGATIVE CONTROL: OR-ing the value bus invents MODE F",
            "mode_set=%s trans_mode=%d (neither port sent 0xF)"
            % ([hex(v) for v in s.log['mode_set']], s.trans_mode()))

    # ---- G5: PC_CMD_ENABLE=0, the one-line retreat ----
    off = System(pc_cmd_enable=0)
    for cmd in G_COMMANDS:
        off.send(frame(cmd), port='pc')
    off.settle(60)
    off.frame_boundary()
    off.settle(20)
    res.add(snapshot(off) == snapshot(System()),
            "retreat: PC_CMD_ENABLE=0 -> every F12 command is a no-op",
            "state identical to a board that heard nothing at all")
    # led[3] is the only physical evidence F12 has. Gating it with the same
    # parameter is what makes the retreat honest in both directions: the light
    # stays dark because the port is off, not because nothing arrived. The
    # parity equality below is what distinguishes the two.
    n_pc_bytes = sum(len(frame(c)) for c in G_COMMANDS)
    res.add(len(off.log['rx_pc']) == n_pc_bytes
            and off.pc('dbg_rx_toggle') == n_pc_bytes % 2
            and off.st['dbg_pc_cmd_toggle'] == 0,
            "retreat: bytes still reach the pin but led[3] stays dark",
            "%d bytes counted by the instance, merged tap gated to %d"
            % (len(off.log['rx_pc']), off.st['dbg_pc_cmd_toggle']))
    res.add(off.j1('dbg_rx_toggle') == 0,
            "retreat: J1's LEDs unaffected by the parameter",
            "j1 byte toggle still %d with zero J1 traffic" % off.j1('dbg_rx_toggle'))
    # the retreat must be one-sided: the verified port still works with PC off
    live = System(pc_cmd_enable=0)
    live.send(frame("MUSC 1"))
    live.send(frame("MODE 3"))
    live.settle(60)
    live.frame_boundary()
    res.add(live.audio_sel_v() == 1 and live.trans_mode() == 3
            and live.log['mode_set'] == [3],
            "retreat: J1 still commands the board with PC_CMD_ENABLE=0",
            "audio=%d trans_mode=%d" % (live.audio_sel_v(), live.trans_mode()))
    if verbose:
        print("      dual-source checks complete")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    res = Result()
    pass_a(res, args.verbose)
    pass_b(res, args.verbose)
    pass_c(res, args.verbose)
    pass_d(res, args.verbose)
    pass_e(res, args.verbose)
    pass_f(res, args.verbose)
    pass_g(res, args.verbose)
    print("=" * 78)
    total = len(res.rows)
    print("%d/%d checks passed, %d failed" % (total - res.failed, total, res.failed))
    print("=" * 78)
    sys.exit(1 if res.failed else 0)


if __name__ == "__main__":
    main()
