#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cycle-accurate model of sd_card_bmp.v's load scheduler, built to check the
picture-level retry added for the "only three of four pictures play" defect.

Why a model rather than a simulator
-----------------------------------
There is no iverilog or verilator on this machine, and the defect lives in a
pure scheduling decision -- which index gets loaded next, and what happens
when an attempt dies -- rather than in any datapath. The card-side facts that
decision depends on are already established by tools/sim_dir_scan.py against
the real card: the scanner finds all four BMPs, at clusters 3/116/229/342, so
img_found_count is 4 and the missing picture is a load that died, not a file
that was never seen.

Two rules this model has to respect to mean anything
----------------------------------------------------
1. Every right-hand side reads the PRE-cycle state. The RTL is one
   `always @(posedge clk)` block of non-blocking assignments, so `load_busy`
   on the right of the arming condition is the registered value even when the
   failure branch above has already scheduled it to clear. Reading an updated
   value here would let the model arm a new load in the same cycle a failure
   landed, which the hardware cannot do.
2. The two writers of next_load_idx -- the rollback on failure and the
   increment on arming -- must never both fire. They are exclusive because
   load_gave_up implies load_busy while arming requires !load_busy; the model
   asserts it instead of trusting it.

The 1s stall watchdog is modelled as STALL_LIMIT = 20 cycles rather than
100_000_000 - 1. Only the constant is scaled; the comparison, the progress
reset and the resulting load_abort are transcribed unchanged.

Scenarios
---------
  A  all four loads succeed
  B  picture 2 dies once then succeeds          -- the reported defect
  C  picture 3 dies on every attempt
  D  picture 0 dies on every attempt
  E  picture 1 dies on the stall watchdog instead of load_failed
  F  CONTROL  same single failure as B, pre-fix scheduler with no retry

Pass G covers the adjustable carousel interval (SPED n, n = 1..8 seconds).
tools/sim_uart_ctrl.py proves the command reaches sd_card_clk with the right
value; it cannot say anything about what sd_card_bmp then does with it, because
the interval counter lives here, in the tightest clock domain, alongside the
load watchdog. Two granularities are modelled:

  - Design.carousel() drives one auto_tick per step, so it checks the rotation
    semantics -- which picture comes up, and that every manual advance / SPED
    write / auto-off restarts the interval instead of inheriting a part-elapsed
    one.
  - IntervalCounter() runs the real cycle loop and checks the arithmetic: the
    interval has to be exactly n * CLK_FREQ_HZ cycles, with no off-by-one in
    either counter.

A-F stay untouched by this feature and are its retreat proof: with no SPED
command ever sent, sec_target is 1, sec_last is constantly true, and the
advance condition reduces to the bare auto_tick the RTL has always had.

Run:  python tools/sim_load_retry.py
"""

SCAN_TARGET_COUNT = 4
LOAD_MAX_RETRY = 3
STALL_LIMIT = 20
CYCLES_PER_LOAD = 50

# sector_lut(), from the four img_sectorN registers the scan fills in. These
# are the real absolute LBAs sim_dir_scan.py read off the card.
IMG_SECTORS = [34832, 36640, 38448, 40256]

REGS = ('img_found_count img_loaded_count next_load_idx img_idx disp_buf_idx '
        'load_buf_idx write_buf_idx load_idx load_sector load_busy '
        'load_retry_cnt load_stall_cnt source_done_seen write_done_seen '
        'scan_kicked first_image_committed display_valid scan_done '
        'scan_raw_only raw_fallback_started load_abort '
        'sec_cnt sec_target_m1').split()


def next_index_limited(cur, count):
    if count <= 1:
        return 0
    if count == 2:
        return 0 if cur == 1 else (cur + 1) & 3
    if count == 3:
        return 0 if cur == 2 else (cur + 1) & 3
    return 0 if cur == 3 else (cur + 1) & 3


class Design:
    """sd_card_bmp's scheduler plus just enough bmp_read to drive it."""

    def __init__(self, fail_plan=None, stall_plan=None, retry_enabled=True,
                 speed_bug=None):
        # fail_plan[pic] = how many leading attempts of that picture fail
        self.fail_plan = dict(fail_plan or {})
        # stall_plan[pic] = set of attempt numbers that go silent instead
        self.stall_plan = stall_plan or {}
        # False reproduces the pre-fix scheduler, where a dead attempt cleared
        # load_busy and nothing else: next_load_idx had already advanced, so
        # the picture was lost for good. Used as a negative control.
        self.retry_enabled = retry_enabled
        # Same idea for the carousel interval -- each value removes one line
        # the RTL has to have, so Pass G can show the matching check actually
        # bites:
        #   'no_gate'           sec_last forced true, i.e. today's RTL
        #   'no_reset_on_speed' SPED writes the target but not sec_cnt
        #   'no_reset_on_next'  a manual NEXT leaves sec_cnt part-elapsed
        #   'off_by_one'        sec_target_m1 gets the value, not value-1
        self.speed_bug = speed_bug

        for r in REGS:
            setattr(self, r, False if r.endswith(('busy', 'seen', 'kicked',
                                                  'committed', 'valid',
                                                  'done', 'only', 'started',
                                                  'abort'))
                    else 0)

        self.bmp_state = 'IDLE'
        self.bmp_age = 0
        self.cur_pic = None
        self.attempts = {}
        self.stalling = False
        self.pending_load_start = False
        self.pending_load_abort = False

        self.trace = []
        self.armed = []
        self.aborts = 0
        self.abort_run = 0
        self.max_abort_run = 0
        self.auto_play_en = False

    # ---- bmp_read -------------------------------------------------------
    def _fails(self, pic, attempt, stall):
        if stall:
            return attempt in self.stall_plan.get(pic, set())
        # attempt is 1-based, so "the first N attempts fail" is attempt <= N.
        return attempt <= self.fail_plan.get(pic, 0)

    def step_bmp_read(self):
        """Returns (bmp_ready, load_failed, write_finish, load_progress).

        Consumes the pulses sd_card_bmp registered last cycle, which is the
        one-cycle delay the RTL has between load_start_pulse and load_start.
        """
        ready = failed = wfinish = progress = False

        if self.pending_load_abort:
            self.aborts += 1
            self.bmp_state = 'IDLE'
            self.stalling = False
            ready = True
        elif self.bmp_state == 'IDLE':
            ready = True
            if self.pending_load_start:
                pic = IMG_SECTORS.index(self.load_sector)
                self.attempts[pic] = self.attempts.get(pic, 0) + 1
                self.cur_pic = pic
                self.bmp_state = 'LOADING'
                self.bmp_age = 0
                self.stalling = False
                ready = False
        elif self.bmp_state == 'LOADING':
            pic, att = self.cur_pic, self.attempts[self.cur_pic]
            if self.stalling or self._fails(pic, att, stall=True):
                # Silent: no bmp_data_wr_en, no write_finish, so the watchdog
                # in sd_card_bmp is the only thing that can end this.
                self.stalling = True
            else:
                progress = True
                self.bmp_age += 1
                if self.bmp_age >= CYCLES_PER_LOAD:
                    if self._fails(pic, att, stall=False):
                        failed = True
                        self.bmp_state = 'IDLE'
                    else:
                        self.bmp_state = 'DONE'
                        ready = True
                        wfinish = True
        elif self.bmp_state == 'DONE':
            self.bmp_state = 'IDLE'
            ready = True

        self.pending_load_start = False
        return ready, failed, wfinish, progress

    # ---- sd_card_bmp ----------------------------------------------------
    def step(self, scan_found_valid=False, auto_tick=False, next_press=False,
             speed_set=None):
        """One sd_card_clk cycle.

        auto_tick is the 1 s tick the RTL derives from auto_cnt; next_press is
        key_next_press/cmd_next_pulse; speed_set is the value a SPED command
        delivers, in whole seconds, or None for no command.
        """
        old = {r: getattr(self, r) for r in REGS}
        nxt = dict(old)

        # Defaults driven every cycle at the top of the RTL's else branch,
        # which later assignments in the same block override. load_abort is
        # defaulted here, not in the idle branch: that default is what makes
        # the stall path's load_abort <= load_stall_hit a one-cycle pulse.
        self.load_start_pulse = False
        nxt['load_abort'] = False

        bmp_ready, load_failed, write_finish, progress = self.step_bmp_read()

        if scan_found_valid and old['img_found_count'] < 4:
            nxt['img_found_count'] = old['img_found_count'] + 1

        if old['load_busy'] and bmp_ready:
            nxt['source_done_seen'] = True
        if old['load_busy'] and write_finish:
            nxt['write_done_seen'] = True

        source_done_now = old['source_done_seen'] or (old['load_busy'] and bmp_ready)
        write_done_now = old['write_done_seen'] or (old['load_busy'] and write_finish)
        load_complete_now = old['load_busy'] and source_done_now and write_done_now
        load_stall_hit = (old['load_busy'] and not progress and
                          old['load_stall_cnt'] >= STALL_LIMIT - 1)
        load_gave_up = (old['load_busy'] and load_failed) or load_stall_hit

        if load_gave_up:
            pic = old['next_load_idx'] - 1
            nxt['load_busy'] = False
            nxt['source_done_seen'] = False
            nxt['write_done_seen'] = False
            nxt['load_stall_cnt'] = 0
            nxt['load_abort'] = load_stall_hit
            if self.retry_enabled and old['load_retry_cnt'] < LOAD_MAX_RETRY:
                nxt['load_retry_cnt'] = old['load_retry_cnt'] + 1
                nxt['next_load_idx'] = old['next_load_idx'] - 1
                self.trace.append(
                    f'    picture {pic} attempt {self.attempts.get(pic)} died'
                    f'{" (stall)" if load_stall_hit else ""} -> retry '
                    f'{nxt["load_retry_cnt"]}/{LOAD_MAX_RETRY}, next_load_idx '
                    f'{old["next_load_idx"]}->{nxt["next_load_idx"]}')
            else:
                nxt['load_retry_cnt'] = 0
                if self.retry_enabled:
                    self.trace.append(
                        f'    picture {pic} exhausted its '
                        f'{LOAD_MAX_RETRY + 1} attempts -> DROPPED')
                else:
                    self.trace.append(
                        f'    picture {pic} died -> DROPPED immediately '
                        f'(pre-fix behaviour, no retry)')
        elif load_complete_now:
            nxt['load_busy'] = False
            nxt['source_done_seen'] = False
            nxt['write_done_seen'] = False
            nxt['load_stall_cnt'] = 0
            nxt['load_retry_cnt'] = 0
            if old['img_loaded_count'] < SCAN_TARGET_COUNT:
                nxt['img_loaded_count'] = old['img_loaded_count'] + 1
            if not old['first_image_committed'] and old['load_buf_idx'] == 0:
                nxt['disp_buf_idx'] = old['load_buf_idx']
                nxt['img_idx'] = old['load_buf_idx']
                nxt['display_valid'] = True
                nxt['first_image_committed'] = True
            self.trace.append(
                f'    picture {self.cur_pic} into buffer {old["load_buf_idx"]}'
                f', img_loaded_count={nxt["img_loaded_count"]}')
        elif old['load_busy']:
            nxt['load_stall_cnt'] = 0 if progress else old['load_stall_cnt'] + 1
        else:
            nxt['load_stall_cnt'] = 0

        if not old['scan_kicked'] and bmp_ready:
            nxt['scan_kicked'] = True
            nxt['first_image_committed'] = False
            nxt['img_found_count'] = 0
            nxt['img_loaded_count'] = 0
            nxt['next_load_idx'] = 0
            nxt['load_retry_cnt'] = 0
            nxt['sec_cnt'] = 0
        else:
            # The RTL drives auto_play_en from key_auto_press/cmd_auto_pulse;
            # the drivers here set it as a plain attribute instead, so the
            # auto-toggle reset site is not exercised at this level. It is
            # covered structurally by the RTL transcription gate, as are the
            # IMGX, load-arming, !sd_init_done and async-reset sites.
            auto_run = (self.auto_play_en and old['first_image_committed'] and
                        old['img_loaded_count'] > 1)
            if not auto_run:
                nxt['sec_cnt'] = 0
            elif auto_tick:
                sec_last = (True if self.speed_bug == 'no_gate'
                            else old['sec_cnt'] == old['sec_target_m1'])
                if sec_last:
                    n = next_index_limited(old['img_idx'],
                                           old['img_loaded_count'])
                    nxt['img_idx'] = n
                    nxt['disp_buf_idx'] = n
                    nxt['sec_cnt'] = 0
                else:
                    nxt['sec_cnt'] = (old['sec_cnt'] + 1) & 7

            if (next_press and old['first_image_committed'] and
                    old['img_loaded_count'] > 1):
                n = next_index_limited(old['img_idx'], old['img_loaded_count'])
                nxt['img_idx'] = n
                nxt['disp_buf_idx'] = n
                if self.speed_bug != 'no_reset_on_next':
                    nxt['sec_cnt'] = 0

            # Not gated on first_image_committed, matching the RTL: the
            # interval is configuration, so a SPED that lands during the
            # initial scan still has to take effect.
            if speed_set is not None:
                nxt['sec_target_m1'] = (speed_set if self.speed_bug == 'off_by_one'
                                        else speed_set - 1) & 7
                if self.speed_bug != 'no_reset_on_speed':
                    nxt['sec_cnt'] = 0

            arm = (old['scan_done'] and bmp_ready and not old['load_busy'] and
                   old['next_load_idx'] < old['img_found_count'] and
                   old['img_loaded_count'] < SCAN_TARGET_COUNT)
            if arm:
                assert not load_gave_up, \
                    'rollback and arming both wrote next_load_idx this cycle'
                idx = old['next_load_idx']
                nxt['load_idx'] = idx & 3
                nxt['load_buf_idx'] = old['img_loaded_count'] & 3
                nxt['load_sector'] = IMG_SECTORS[idx & 3]
                nxt['write_buf_idx'] = old['img_loaded_count'] & 3
                nxt['next_load_idx'] = idx + 1
                nxt['load_busy'] = True
                nxt['source_done_seen'] = False
                nxt['write_done_seen'] = False
                nxt['load_stall_cnt'] = 0
                self.load_start_pulse = True
                self.armed.append((idx, IMG_SECTORS[idx & 3],
                                   old['img_loaded_count'] & 3))

        for k, v in nxt.items():
            setattr(self, k, v)
        self.abort_run = self.abort_run + 1 if self.load_abort else 0
        self.max_abort_run = max(self.max_abort_run, self.abort_run)
        self.pending_load_start = self.load_start_pulse
        self.pending_load_abort = self.load_abort

    # ---- drivers --------------------------------------------------------
    def run(self, max_cycles=20000):
        self.step()                                   # kick the scan
        for sec in IMG_SECTORS:                       # four scan_found_valid
            self.step(scan_found_valid=True)
        self.scan_done = True
        for c in range(max_cycles):
            self.step()
            if (self.scan_done and not self.load_busy and
                    self.next_load_idx >= self.img_found_count):
                return c
        raise AssertionError('scheduler never settled -- runaway loop')

    def rotation(self, ticks=12):
        if self.img_loaded_count < 2 or not self.display_valid:
            return []
        self.auto_play_en = True
        out = []
        for _ in range(ticks):
            self.step(auto_tick=True)
            out.append(self.img_idx + 1)
        return out

    def carousel(self, ticks, speed=None):
        """Same shape as rotation(), plus an optional SPED write up front.

        speed is the interval in whole seconds; None leaves sec_target alone,
        which is how the "no command ever sent" retreat is driven.
        """
        if self.img_loaded_count < 2 or not self.display_valid:
            return []
        self.auto_play_en = True
        if speed is not None:
            self.step(speed_set=speed)
        out = []
        for _ in range(ticks):
            self.step(auto_tick=True)
            out.append(self.img_idx + 1)
        return out


def changes(seq, start):
    """1-based positions in seq whose value differs from its predecessor."""
    out = []
    prev = start
    for i, v in enumerate(seq, start=1):
        if v != prev:
            out.append(i)
        prev = v
    return out


def prepared():
    """A Design whose scheduler has settled with all four pictures loaded."""
    d = Design()
    d.run()
    return d


def prepared_bug(bug):
    """prepared(), but with one interval line deliberately missing."""
    d = Design(speed_bug=bug)
    d.run()
    return d


class IntervalCounter:
    """Cycle-level model of sd_card_bmp's auto_cnt / sec_cnt pair.

    Design.carousel() collapses a whole second into one auto_tick, which is the
    right granularity for rotation order but blind to the arithmetic. This runs
    the counters cycle by cycle instead.

    STALL_LIMIT stands in for CLK_FREQ_HZ exactly as it already does for the
    watchdog in Design.step. That is not a coincidence to be tidied away: the
    RTL spells both counters with the same constant expression, so scaling it
    once here preserves the ratio the hardware has.
    """

    TICK_LIMIT = STALL_LIMIT

    def __init__(self, sec=1, bug=None):
        self.bug = bug
        self.auto_cnt = 0
        self.sec_cnt = 0
        self.sec_target_m1 = sec if bug == 'off_by_one' else sec - 1

    def cycle(self):
        """One sd_card_clk cycle. True when the picture would change."""
        tick = (self.auto_cnt == self.TICK_LIMIT - 1)
        self.auto_cnt = 0 if tick else self.auto_cnt + 1
        if not tick:
            return False
        last = (True if self.bug == 'no_gate'
                else self.sec_cnt == self.sec_target_m1)
        if last:
            self.sec_cnt = 0
            return True
        self.sec_cnt = (self.sec_cnt + 1) & 7
        return False

    def first_two(self, limit=None):
        """1-based cycle numbers of the first two picture changes."""
        if limit is None:
            limit = self.TICK_LIMIT * (self.sec_target_m1 + 2) * 2 + 8
        hits = []
        for c in range(1, limit + 1):
            if self.cycle():
                hits.append(c)
                if len(hits) == 2:
                    break
        return hits


def report(name, design, expect_loaded, expect_rot, expect_arms=None):
    design.run()
    rot = design.rotation()
    ok = design.img_loaded_count == expect_loaded and rot == expect_rot
    if expect_arms is not None and len(design.armed) != expect_arms:
        ok = False
    # bmp_read holds itself in ST_IDLE while load_abort is high, so the abort
    # has to be a single pulse. Measured against the waveform the RTL's
    # top-of-block default produces, not assumed.
    abort_ok = design.max_abort_run <= 1
    if not abort_ok:
        ok = False
    print(f'{name}')
    print(f'  img_found_count   {design.img_found_count}')
    print(f'  img_loaded_count  {design.img_loaded_count}'
          f'   expected {expect_loaded}')
    print(f'  display_valid     {design.display_valid}')
    print(f'  OSD rotation      {rot}   expected {expect_rot}')
    print(f'  attempts          {len(design.armed)}'
          f'{f"   expected {expect_arms}" if expect_arms is not None else ""}'
          f'   {[(p, s, b) for p, s, b in design.armed]}')
    print(f'  stall aborts      {design.aborts}'
          f'   longest load_abort high run {design.max_abort_run} cycle(s)'
          f'{"" if abort_ok else "  <-- bmp_read frozen, not a pulse"}')
    for line in design.trace:
        print(line)
    print(f'  -> {"PASS" if ok else "FAIL"}')
    print()
    return ok


def pass_g():
    """Pass G -- adjustable carousel interval (SPED n, n = 1..8 seconds)."""
    print('=' * 64)
    print('G. Adjustable carousel interval')
    print('=' * 64)
    res = []

    def check(name, got, want):
        ok = got == want
        res.append(ok)
        print(f'  {"PASS" if ok else "FAIL"}  {name}')
        if not ok:
            print(f'        got  {got}')
            print(f'        want {want}')

    def ticks_of(design, ticks, speed=None):
        """1-based tick positions at which the picture changed."""
        before = design.img_idx + 1
        return changes(design.carousel(ticks, speed=speed), before)

    # G1 -- the retreat. No SPED ever sent, so sec_target is its reset value 1
    # and the rotation has to be bit-for-bit what it was before this feature.
    check('G1  zero traffic -> interval stays 1 s, rotation unchanged',
          prepared().carousel(8), [2, 3, 4, 1] * 2)

    # G2 -- the interval actually stretches.
    for sec, ticks in ((4, 12), (8, 16)):
        check(f'G2  SPED {sec} advances once every {sec} ticks over {ticks}',
              ticks_of(prepared(), ticks, speed=sec),
              list(range(sec, ticks + 1, sec)))

    # G3 -- stretching the interval must not reorder anything underneath it.
    check('G3  SPED 4 rotation order over 12 ticks',
          prepared().carousel(12, speed=4),
          [1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4])

    # G4/G5/G6 -- every site that clears auto_cnt has to clear sec_cnt too,
    # or the next interval inherits a part-elapsed one and comes up short.
    d = prepared()
    d.carousel(3, speed=8)                    # leaves sec_cnt at 3 of 8
    check('G4  SPED 2 after a part-elapsed SPED 8 restarts the interval',
          ticks_of(d, 4, speed=2), [2, 4])

    d = prepared()
    d.carousel(2, speed=4)                    # leaves sec_cnt at 2 of 4
    d.step(next_press=True)
    check('G5  a manual NEXT restarts the interval',
          ticks_of(d, 4), [4])

    d = prepared()
    d.carousel(2, speed=4)
    d.auto_play_en = False
    d.step()                                  # the auto-off else branch
    check('G6  turning auto play off and back on restarts the interval',
          ticks_of(d, 4), [4])

    # G7 -- cycle-level arithmetic: n seconds is exactly n * CLK_FREQ_HZ
    # cycles, for every value the parser accepts.
    for sec in range(1, 9):
        check(f'G7  SPED {sec} -> {STALL_LIMIT * sec} cycles between changes',
              IntervalCounter(sec=sec).first_two(),
              [STALL_LIMIT * sec, STALL_LIMIT * sec * 2])

    # G8 -- negative controls. Each removes exactly one line the RTL has to
    # have; if the matching check above still passed against these, it would
    # be testing nothing.
    check('G8a CONTROL  no sec gate -> SPED 4 still advances every tick',
          ticks_of(prepared_bug('no_gate'), 12, speed=4), list(range(1, 13)))
    check('G8b CONTROL  target off by one -> SPED 4 advances at 5 and 10',
          ticks_of(prepared_bug('off_by_one'), 10, speed=4), [5, 10])

    d = prepared_bug('no_reset_on_speed')
    d.carousel(3, speed=8)
    check('G8c CONTROL  SPED without a sec_cnt reset -> interval inherited',
          ticks_of(d, 4, speed=2), [])

    d = prepared_bug('no_reset_on_next')
    d.carousel(2, speed=4)
    d.step(next_press=True)
    check('G8d CONTROL  NEXT without a sec_cnt reset -> next interval short',
          ticks_of(d, 4), [2])

    check('G8e CONTROL  cycle counter with no gate -> 1 s even at SPED 4',
          IntervalCounter(sec=4, bug='no_gate').first_two(),
          [STALL_LIMIT, STALL_LIMIT * 2])
    check('G8f CONTROL  cycle counter off by one -> 5 s at SPED 4',
          IntervalCounter(sec=4, bug='off_by_one').first_two(),
          [STALL_LIMIT * 5, STALL_LIMIT * 10])

    print()
    return res


def main():
    four = [2, 3, 4, 1] * 3
    three = [2, 3, 1] * 4
    results = [
        report('A  all four loads succeed',
               Design(), 4, four, expect_arms=4),
        report('B  picture 2 dies once then succeeds  (the reported defect)',
               Design(fail_plan={2: 1}), 4, four, expect_arms=5),
        report('C  picture 3 dies on every attempt',
               Design(fail_plan={3: 99}), 3, three, expect_arms=7),
        report('D  picture 0 dies on every attempt',
               Design(fail_plan={0: 99}), 3, three, expect_arms=7),
        report('E  picture 1 dies on the 1s stall watchdog then succeeds',
               Design(stall_plan={1: {1}}), 4, four, expect_arms=5),
        # Negative control. Same single transient failure as B, but against
        # the pre-fix scheduler. It has to reproduce exactly what the user
        # reported -- three pictures playing, four never appearing -- or the
        # PASS on B above does not mean anything.
        report('F  CONTROL  picture 2 dies once, pre-fix scheduler (no retry)',
               Design(fail_plan={2: 1}, retry_enabled=False), 3, three,
               expect_arms=4),
    ]
    results += pass_g()
    print('=' * 64)
    verdict = 'ALL SCENARIOS PASS' if all(results) else 'FAILURES PRESENT'
    print(f'{verdict}   ({sum(results)}/{len(results)})')
    return 0 if all(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
