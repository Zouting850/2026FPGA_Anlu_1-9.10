#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Replay sd_audio_stream.v against the REAL bytes on the card, at the REAL LBA.

sim_audio_stream.py proves the streamer's logic is self-consistent on synthetic
sectors. That leaves a gap this closes: whether the sector addresses bmp_read's
scan actually latches point at canonical WAV headers on this particular card.
If one does not, sd_audio_stream's magic check fails on that arm and latches
S_FAULT, so the picture mapped to it plays with no music -- and since track N
belongs to picture N, a card where only the first song is canonical shows up as
"some pictures have sound, some do not", not as a dead design.

So this walks the same two steps the hardware does, once per track:

  1. replay the root directory scan (sim_dir_scan.replay_scan) to get the whole
     wav_sector / wav_size table sd_card_bmp would fill, in slot order;
  2. for each slot, read the sectors from that LBA onward and push them byte by
     byte through a register-level model of sd_audio_stream's S_READ datapath --
     hdr_skip, hdr_cnt, the RIFF/WAVE magic capture, byte_phase, pcm_cnt,
     fifo_we.

Reports per track whether magic_ok fires, whether the 4-byte frame alignment
survives the header skip and every sector boundary, and the amplitude of the
frames actually handed to the FIFO, so silence is distinguishable from a broken
address. A failing track does not stop the others: each is armed separately on
silicon, so each is reported separately here.

Run:  python tools/check_wav_on_card.py F: [--sectors N]
"""
import contextlib
import io
import os
import struct
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)

from sim_dir_scan import (Volume, read_dir_sectors, replay_scan,  # noqa: E402
                          SECTOR)

HDR_LEN = 44              # sd_audio_stream.v HDR_LEN
FAILURES = []
CHECKS = [0]


def check(cond, label, detail=""):
    # detail explains the failure mode, so it only belongs on a FAIL line
    CHECKS[0] += 1
    tag = "ok  " if cond else "FAIL"
    suffix = "" if cond or not detail else " -- " + detail
    print("    %s  %s%s" % (tag, label, suffix))
    if not cond:
        FAILURES.append(label)
    return cond


def s16(b):
    """HDMI audio path treats these as two's-complement 16-bit samples."""
    v = b & 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def stream_sectors(sectors, pcm_total, report_frames=8):
    """Register-level replay of sd_audio_stream.v's S_READ datapath.

    `sectors` is an iterable of 512-byte payloads in the order the streamer
    requests them (LBA +1 each time). Returns a dict of what the RTL would have
    done, mirroring the non-blocking assignment semantics: every register update
    computed from a byte is applied after that byte, and the values sampled by a
    comparison are the pre-update ones.
    """
    hdr_skip = HDR_LEN
    hdr_cnt = 0
    byte_phase = 0
    pcm_cnt = 0
    magic_done = False
    magic_ok = False
    state_fault = False
    riff = [0, 0, 0, 0]
    wave = [0, 0, 0, 0]
    l_lo = l_hi = r_lo = 0
    fifo_writes = []
    phase_at_sector_end = []
    sector_index = 0

    for payload in sectors:
        if state_fault:
            break
        for byte in payload:
            if hdr_skip != 0:
                if hdr_cnt < 4:
                    riff[hdr_cnt] = byte
                elif 8 <= hdr_cnt < 12:
                    wave[hdr_cnt - 8] = byte
                hdr_cnt += 1
                hdr_skip -= 1
                # RTL: if (hdr_skip == 6'd1 && !magic_done) -- evaluated with the
                # PRE-decrement value, i.e. on the last header byte.
                if hdr_skip == 0 and not magic_done:
                    magic_done = True
                    magic_ok = (bytes(riff) == b"RIFF" and bytes(wave) == b"WAVE")
                    if not magic_ok:
                        state_fault = True
                        break
            elif pcm_cnt < pcm_total:
                if byte_phase == 0:
                    l_lo = byte
                    byte_phase = 1
                elif byte_phase == 1:
                    l_hi = byte
                    byte_phase = 2
                elif byte_phase == 2:
                    r_lo = byte
                    byte_phase = 3
                else:
                    frame = (byte << 24) | (r_lo << 16) | (l_hi << 8) | l_lo
                    fifo_writes.append(frame)
                    byte_phase = 0
                    pcm_cnt += 4
        phase_at_sector_end.append(byte_phase)
        sector_index += 1

    return {
        "magic_done": magic_done,
        "magic_ok": magic_ok,
        "riff": bytes(riff),
        "wave": bytes(wave),
        "state_fault": state_fault,
        "fifo_writes": fifo_writes,
        "pcm_cnt": pcm_cnt,
        "phase_at_sector_end": phase_at_sector_end,
        "sectors": sector_index,
    }


def selftest():
    """Prove the per-track loop isolates a bad song instead of hiding the rest.

    This tool used to replay the single WAV the scan latched and `return` at the
    first failure -- correct when there was only ever one song. Now every track
    is armed separately on silicon, so an early `return` would announce "track 2
    is broken" and say nothing at all about track 3, which is precisely the
    failure shape this tool exists to catch. The synthetic card therefore puts
    the corrupt header in the MIDDLE and requires the last track to still run to
    completion after it.

    It also pins both edges of step 3's silence gate. That gate judges a window
    of `--sectors` (here 8 sectors = 0.02 s), which is shorter than the fade-in
    a real song opens with, so it used to fail perfectly good cards. It now
    reads forward, bounded, before declaring failure -- and a widened gate has
    to be shown to still bite, or it is decoration. Hence two more synthetic
    cards: one whose PCM is all zeros, which must still be reported as silence
    and must not stop the tracks after it, and one with a 0.5 s fade-in, which
    must now pass and must report where its audio actually starts. Both were
    mutation-checked: excusing zeros trips six of the assertions below, and
    disabling the probe trips five.
    """
    n_sec = 8
    pcm_bytes = n_sec * SECTOR * 40        # far more than --sectors will read
    good_hdr = (b"RIFF" + struct.pack("<I", pcm_bytes + 36) + b"WAVE"
                + b"fmt "
                + struct.pack("<IHHIIHH", 16, 1, 2, 48000, 192000, 4, 16)
                + b"data" + struct.pack("<I", pcm_bytes))
    assert len(good_hdr) == HDR_LEN, len(good_hdr)
    bad_hdr = b"XXXX" + good_hdr[4:]
    # L = -1000, R = +1000 per frame. Alternating, not DC: step 3's RMS is the
    # mean-subtracted deviation, so a constant sample value scores 0 and would
    # trip the "not digital silence" check on perfectly good data.
    body = b"\x18\xfc\xe8\x03" * (pcm_bytes // 4)

    class FakeVol(object):
        hid_sec = 2048

        def __init__(self, images):
            self.images = images          # [(base_vol_sector, image_bytes)]

        def read(self, vol_start, n):
            for base, img in self.images:
                span = (len(img) + SECTOR - 1) // SECTOR
                if base <= vol_start < base + span:
                    off = (vol_start - base) * SECTOR
                    return img[off:off + n * SECTOR]
            return b""

        def close(self):
            pass

    # The two bodies that probe step 3's widened silence gate from both sides:
    # all-zeros must still fail (an 8 s probe that excused this would turn the
    # check into a no-op), and a genuine 0.5 s fade-in must now pass.
    fade_bytes = int(0.5 * 192000)
    silent_body = b"\x00" * pcm_bytes
    fade_body = b"\x00" * fade_bytes + body[:pcm_bytes - fade_bytes]

    def build(bad_slot, n_wavs, bodies=None):
        bodies = bodies or {}
        images, found, wavs = [], [], []
        for i in range(4):
            base = 1000 + i * 400
            hdr = bad_hdr if i == bad_slot else good_hdr
            images.append((base, hdr + bodies.get(i, body)))
            found.append(("%02d_IMG.BMP" % i, 2 + i, 900000, base + 2048,
                          7, i))
            if i < n_wavs:
                wavs.append(("MUSIC%d.WAV" % i, 3 + i, HDR_LEN + pcm_bytes,
                             base + FakeVol.hid_sec, 7, i))
        return images, found, wavs

    saved = (Volume, read_dir_sectors, replay_scan)
    out = io.StringIO()
    fails = []
    try:
        for label, bad_slot, n_wavs, bodies, want_rc in (
                ("bad track 2 of 4", 2, 4, None, 1),
                ("all four canonical", -1, 4, None, 0),
                ("one track, four pictures", -1, 1, None, 0),
                ("track 1 is all zeros", -1, 4, {1: silent_body}, 1),
                ("track 0 fades in over 0.5 s", -1, 4, {0: fade_body}, 0)):
            images, found, wavs = build(bad_slot, n_wavs, bodies)
            del FAILURES[:]
            CHECKS[0] = 0
            globals()["Volume"] = lambda _drive: FakeVol(images)
            globals()["read_dir_sectors"] = lambda _vol: [b"\x00" * SECTOR]
            globals()["replay_scan"] = (
                lambda _ds, _v, _t, verbose=False: (found, "selftest", wavs))
            with contextlib.redirect_stdout(out):
                rc = main(["selftest", "SYNTH", "--sectors", str(n_sec)])
            text = out.getvalue()
            out.truncate(0)
            out.seek(0)

            def want(cond, why):
                print("    %s  [%s] %s" % ("ok  " if cond else "FAIL",
                                            label, why))
                if not cond:
                    fails.append("%s: %s" % (label, why))

            want(rc == want_rc, "rc %d (want %d)" % (rc, want_rc))
            for i in range(n_wavs):
                want("track %d: MUSIC%d.WAV" % (i, i) in text,
                     "track %d got its own section" % i)
            if bad_slot >= 0:
                # "    FAIL  " is check()'s line format; report() also prints a
                # line containing "FAILED", so counting bare "FAIL" overcounts.
                want(text.count("    FAIL  ") == 1,
                     "exactly one FAIL line, got %d"
                     % text.count("    FAIL  "))
                want("RIFF/WAVE magic verified" in text,
                     "the failure names the magic check")
                want("58 58 58 58" in text, "the hex dump shows XXXX")
                # The whole point: track 3 must run AFTER track 2 failed.
                # find(), not index() -- a mutant that returns early leaves no
                # track 3 section at all, and that must be reported as a failed
                # selfcheck rather than crashing this one.
                i3 = text.find("track 3:")
                ifail = text.find("    FAIL  ")
                want(i3 >= 0 and ifail >= 0 and i3 > ifail,
                     "track 3 was replayed after the failure, not skipped")
                want(text.count("PCM is not digital silence") == 3,
                     "the three good tracks each reached step 3, got %d"
                     % text.count("PCM is not digital silence"))
                want(len(FAILURES) == 1, "one recorded failure, got %d"
                     % len(FAILURES))
            else:
                silent_slot = next((k for k, v in (bodies or {}).items()
                                    if v is silent_body), -1)
                fade_slot = next((k for k, v in (bodies or {}).items()
                                  if v is fade_body), -1)
                if silent_slot >= 0:
                    # The 8 s probe widened this gate. Zeros for the whole
                    # probe must still be called what they are, or the check
                    # has become decoration.
                    want(text.count("    FAIL  ") == 1,
                         "exactly one FAIL line, got %d"
                         % text.count("    FAIL  "))
                    want("RMS 0.0 is effectively zero" in text,
                         "the failure names the silence check and prints RMS")
                    want(FAILURES == ["PCM is not digital silence"],
                         "recorded failure is the silence check, got %s"
                         % FAILURES[:2])
                    want("silent lead-in" not in text,
                         "the probe found nothing to excuse, so it said so")
                    want(text.count("RIFF/WAVE magic verified") == n_wavs,
                         "every header was canonical, so only step 3 can fail")
                    want(text.count("PCM is not digital silence") == n_wavs + 1,
                         "all four tracks still ran step 3, got %d (+1 is "
                         "report()'s summary line reprinting the failure)"
                         % text.count("PCM is not digital silence"))
                    i3 = text.find("track 3:")
                    ifail = text.find("    FAIL  ")
                    want(i3 >= 0 and ifail >= 0 and i3 > ifail,
                         "track 3 was replayed after the silent track, not "
                         "skipped")
                elif fade_slot >= 0:
                    # The other half: the probe exists to STOP failing good
                    # cards, so a real fade-in has to come through clean and
                    # be reported, not silently absorbed.
                    want(not FAILURES, "no failures: %s" % FAILURES[:2])
                    want("audio starts at 0.500 s" in text,
                         "the probe measured the fade-in as 0.500 s")
                    want("this 0.50 s of lead-in silence is audible" in text,
                         "the operator is told the lead-in is audible")
                    want(text.count("PCM is not digital silence") == n_wavs,
                         "every track reached step 3")
                else:
                    want(not FAILURES, "no failures: %s" % FAILURES[:2])
                    want(text.count("PCM is not digital silence") == n_wavs,
                         "every track reached step 3")
            note = "have no track of their own"
            want((note in text) == (n_wavs < 4),
                 "the no-track note appears exactly when WAVs < BMPs")
            want(("track %d:" % n_wavs) not in text,
                 "no section printed for a track the scan never reported")
            print("")
    finally:
        globals()["Volume"], globals()["read_dir_sectors"], \
            globals()["replay_scan"] = saved
        del FAILURES[:]
        CHECKS[0] = 0

    print("=" * 72)
    if fails:
        print("%d SELFCHECK(S) FAILED:" % len(fails))
        for f in fails:
            print("  %s" % f)
        return 1
    print("selftest passed: a corrupt track is isolated, the tracks after it "
          "are still replayed, the no-track note tracks the real counts, and "
          "the widened silence gate still fails 8 s of zeros while passing a "
          "0.5 s fade-in")
    return 0


def main(argv):
    if "--selftest" in argv:
        return selftest()
    if len(argv) < 2:
        print(__doc__)
        return 2
    drive = argv[1]
    n_sectors = 64
    if "--sectors" in argv:
        n_sectors = int(argv[argv.index("--sectors") + 1])

    vol = Volume(drive)
    try:
        print("=" * 72)
        print("sd_audio_stream.v replay against real card bytes: %s" % drive)
        print("=" * 72)

        # Step 1: what does the scan latch? replay_scan is chatty, and its own
        # report is not what this tool is about, so swallow it.
        with contextlib.redirect_stdout(io.StringIO()):
            dir_sectors = read_dir_sectors(vol)
            found, _stopped, wavs = replay_scan(dir_sectors, vol, 4)

        print("step 1: the track table the scan builds, in physical slot order")
        if not check(bool(wavs), "ST_SCAN_DIR reported at least one WAV entry",
                     "no WAV reported, wav_found_count stays 0, audio_phase can "
                     "never set"):
            return report()
        for n, (label, clus, size, abs_lba, dsec, dent) in enumerate(wavs):
            print("    track %d  %-12s dir sec %d ent %02d  cluster %5d  "
                  "%9d bytes" % (n, label, dsec, dent, clus, size))
            print("             wav_sector = %d absolute LBA "
                  "(= volume-relative %d)" % (abs_lba, abs_lba - vol.hid_sec))
        if len(wavs) < len(found):
            print("    NOTE %d of %d picture(s) have no track of their own; "
                  "track_req_lim clamps them onto slot 0, so they share the "
                  "first song rather than playing silence"
                  % (len(found) - len(wavs), len(found)))
        print("")

        # Steps 2 and 3, once per track. On silicon each track is armed
        # separately, so each one's magic check runs separately too: a card
        # where only the first song is canonical costs exactly the pictures
        # that map to the other three, and reporting just track 0 would call
        # that card good.
        for n, (label, clus, size, abs_lba, dsec, dent) in enumerate(wavs):
            print("-" * 72)
            print("track %d: %s" % (n, label))

            check(size > HDR_LEN + 4,
                  "wav_usable (wav_size %d > HDR_LEN + 4)" % size,
                  "sd_audio_stream would go straight to S_FAULT")

            # Step 2: read the sectors the streamer will request, in order. The
            # streamer advances LBA by +1 and never follows the FAT chain, so a
            # fragmented file reads as garbage after the first run -- worth
            # saying out loud because that failure mode is also silent.
            vol_start = abs_lba - vol.hid_sec
            need = min(n_sectors, (size + SECTOR - 1) // SECTOR)
            raw = vol.read(vol_start, need)
            check(len(raw) == need * SECTOR,
                  "read %d sector(s) from LBA %d" % (need, abs_lba),
                  "short read: got %d bytes" % len(raw))
            if len(raw) != need * SECTOR:
                continue

            print("    step 2: byte-level replay of S_READ over those sectors")
            r = stream_sectors(
                [raw[i * SECTOR:(i + 1) * SECTOR] for i in range(need)],
                size - HDR_LEN)
            print("      header bytes 0..3  = %r" % r["riff"])
            print("      header bytes 8..11 = %r" % r["wave"])
            check(r["magic_done"],
                  "magic capture ran (hdr_skip reached its last byte)")
            if not check(r["magic_ok"], "RIFF/WAVE magic verified",
                         "magic_ok is false -> S_FAULT, this picture is silent, "
                         "the next one still has its own track"):
                print("")
                print("      first 64 bytes at LBA %d:" % abs_lba)
                for off in range(0, 64, 16):
                    chunk = raw[off:off + 16]
                    print("        +%03d  %s  %s"
                          % (off, chunk.hex(" "),
                             "".join(chr(c) if 32 <= c < 127 else "."
                                     for c in chunk)))
                continue

            check(not r["state_fault"], "streamer never entered S_FAULT")

            # The design's stated invariant: 44 and 512 are both multiples of 4,
            # so byte_phase returns to 0 at every sector boundary and no
            # cross-sector frame bookkeeping is needed. If a card ever broke
            # this, samples would swap channels rather than go silent -- still
            # worth catching here.
            bad = [i for i, p in enumerate(r["phase_at_sector_end"]) if p != 0]
            check(not bad,
                  "byte_phase == 0 at all %d sector boundary(ies)" % need,
                  "misaligned after sector(s) %s" % bad[:8])

            nw = len(r["fifo_writes"])
            check(nw > 0,
                  "fifo_we pulsed %d time(s) over %d sector(s)"
                  % (nw, r["sectors"]),
                  "no PCM frames reached the FIFO write side")
            expected = (r["sectors"] * SECTOR - HDR_LEN) // 4
            check(nw == expected,
                  "frame count matches (sectors*512 - 44) // 4 = %d" % expected,
                  "model produced %d" % nw)

            print("    step 3: are those frames actual audio, or "
                  "silence/garbage?")
            show = 8
            print("      first %d frames as {R,L} (fifo_di = {R[15:0], "
                  "L[15:0]}):" % min(show, nw))
            for i, f in enumerate(r["fifo_writes"][:show]):
                print("        [%2d] fifo_di=0x%08x  L=%6d  R=%6d"
                      % (i, f, s16(f), s16(f >> 16)))

            samples = []
            for f in r["fifo_writes"]:
                samples.append(s16(f))
                samples.append(s16(f >> 16))

            def stats(samps):
                m = sum(samps) / float(len(samps))
                return ((sum((s - m) ** 2 for s in samps) / float(len(samps)))
                        ** 0.5, max(abs(min(samps)), abs(max(samps))))

            rms, peak = stats(samples)
            print("      over %d samples: RMS %.1f, peak %d"
                  % (len(samples), rms, peak))

            # The replayed window is only 64 sectors = 0.34 s, and real songs
            # routinely open with a fade-in longer than that (the four demo
            # tracks lead with 0.285 / 0.509 / 0.949 / 0.186 s of true digital
            # silence, in MUSIC0..MUSIC3 order). Judging silence on that window
            # alone therefore fails good cards. Read on, bounded, and only call
            # it a wrong address when 8 whole seconds are silent -- which is
            # what zeros or a gap actually looks like.
            lead = None
            if rms <= 50.0:
                probe_sec = (HDR_LEN + 8 * 192000 + SECTOR - 1) // SECTOR
                probe_n = min(probe_sec, (size + SECTOR - 1) // SECTOR)
                praw = vol.read(vol_start, probe_n)
                ps = []
                for i in range(HDR_LEN // 2, len(praw) // 2):
                    ps.append(s16(int.from_bytes(praw[i * 2:i * 2 + 2],
                                                 "little")))
                TH = 200
                hit = next((i for i, v in enumerate(ps) if abs(v) > TH), None)
                if hit is not None:
                    lead = hit / 2 / 48000.0
                    print("      first window is a silent lead-in; audio "
                          "starts at %.3f s (probe %d sectors, %d samples)"
                          % (lead, probe_n, len(ps)))
                    rms, peak = stats(ps[hit:hit + 2 * 48000] or ps[hit:])

            check(rms > 50.0, "PCM is not digital silence",
                  "RMS %.1f is effectively zero -- the address is right but "
                  "the payload is not the song" % rms)
            check(peak <= 32768, "samples are within 16-bit range",
                  "peak %d overflows, byte order is suspect" % peak)
            if lead is not None:
                print("      NOTE a track switch restarts the song from byte "
                      "0, so this %.2f s of lead-in silence is audible" % lead)
            print("")

        return report()
    finally:
        vol.close()


def report():
    print("")
    print("=" * 72)
    if FAILURES:
        print("%d of %d checks FAILED: %s"
              % (len(FAILURES), CHECKS[0], "; ".join(FAILURES)))
        return 1
    print("all %d checks passed" % CHECKS[0])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
