#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Replay bmp_read.v's root directory scan against the real card, byte for byte.

Why this tool exists
--------------------
Everything the FPGA reads off this card is decided by raw bytes that Explorer
will never show you: the physical order of the root directory slots, and whether
each file's clusters happen to be contiguous. Explorer lists the directory
through the file system driver, which follows the FAT chain and hides deleted
slots, so a card can look perfect in Explorer and still be unreadable to the
panel. This tool reads the volume handle at byte offsets and applies the RTL's
own predicates, so what it reports is what sd_card_bmp will actually see.

Two independent contracts, both checked here
--------------------------------------------
1) DIRECTORY ORDER. ST_SCAN_DIR samples a fixed set of offsets within each
   32 byte entry (rd_cnt[4:0] == 0, 8, 9, 10, 11, 20, 21, 26, 27, 28..31) and
   acts at rd_cnt[4:0] == 31:

       if (dir_first_byte == 8'h00)            -> scan_done, STOP
       else if (dir_entry_is_bmp_now)          -> record, and STOP once
                                                  scan_found_total reaches
                                                  scan_target_count
       else if (dir_entry_is_wav_now)          -> report, never counted, never
                                                  triggers the early stop

       dir_entry_is_file = first_byte != 0x00 && first_byte != 0xE5 &&
                           attr != 0x0F &&        // long file name slot
                           !attr[3] &&            // volume label
                           !attr[4] &&            // subdirectory
                           cluster >= 2 && size != 0
       dir_ext_is_bmp    = ext in {B,b}{M,m}{P,p}
       dir_ext_is_wav    = ext in {W,w}{A,a}{V,v}

   The stop-on-0x00 rule is correct FAT semantics -- a zero first byte means
   this entry and every entry after it are free -- but it makes the scan
   order-dependent. So does the early stop at scan_target_count: with four
   pictures, every WAV must sit BEFORE the fourth BMP in physical slot order or
   it is never reported, and the picture it was meant to accompany plays silent.

   Since bmp_read reports EVERY WAV now (the old wav_captured latch took only the
   first), sd_card_bmp fills wav_sector0..3 in slot order, and track N ends up
   belonging to picture N. That mapping is only as good as the order on the card.

2) PHYSICAL CONTIGUITY. Neither the WAV streamer nor the picture loader follows
   the FAT32 cluster chain: both take the first data sector from the directory
   entry and then advance LBA + 1. A file whose clusters are not consecutive is
   read straight past its end into whatever follows it -- silently, because the
   bytes still arrive.

Everything here is read through the volume handle at byte offsets, so no
administrator rights are needed.

The replay is checked against itself
------------------------------------
An assertion that cannot fail proves nothing, so after the real scan this tool
re-runs the identical replay over an in-memory copy of the directory with the
last WAV moved to AFTER the fourth BMP, and requires that copy to lose the WAV.
If that control ever stops biting, the order check above has gone vacuous and
the rest of the report should not be trusted.

Usage
-----
    python tools/sim_dir_scan.py F:
    python tools/sim_dir_scan.py F: --target 4
    python tools/sim_dir_scan.py F: --no-contig      # skip the FAT chain walk
"""

import ctypes
import os
import struct
import sys
from ctypes import wintypes

SECTOR = 512
ENTRY = 32
ROOT_SCAN_MAX_SECTORS = 128
FSCTL_GET_VOLUME_DISK_EXTENTS = 0x00090000
GENERIC_READ = 0x80000000
FILE_SHARE_READ = 0x1
FILE_SHARE_WRITE = 0x2
OPEN_EXISTING = 3
INVALID_HANDLE = wintypes.HANDLE(-1).value
FILE_BEGIN = 0

FAT_FREE = 0x00000000
FAT_BAD = 0x0FFFFFF7
FAT_EOC = 0x0FFFFFF8          # >= this means end of chain
FAT_MASK = 0x0FFFFFFF

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
k32.CreateFileW.restype = wintypes.HANDLE
k32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                            ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD,
                            wintypes.HANDLE]
k32.ReadFile.restype = wintypes.BOOL
k32.ReadFile.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD,
                         ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
k32.SetFilePointerEx.restype = wintypes.BOOL
k32.SetFilePointerEx.argtypes = [wintypes.HANDLE, ctypes.c_longlong,
                                 ctypes.POINTER(ctypes.c_longlong),
                                 wintypes.DWORD]
k32.DeviceIoControl.restype = wintypes.BOOL
k32.DeviceIoControl.argtypes = [wintypes.HANDLE, wintypes.DWORD, ctypes.c_void_p,
                                wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD,
                                ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
k32.CloseHandle.argtypes = [wintypes.HANDLE]


class Volume(object):
    """Raw sector reader plus the FAT32 geometry the RTL derives from the BPB."""

    def __init__(self, drive):
        self.dev = "\\\\.\\%s:" % drive.rstrip("\\").rstrip(":")
        self.h = k32.CreateFileW(self.dev, GENERIC_READ,
                                 FILE_SHARE_READ | FILE_SHARE_WRITE, None,
                                 OPEN_EXISTING, 0, None)
        if self.h == INVALID_HANDLE or self.h is None:
            raise ctypes.WinError(ctypes.get_last_error())
        boot = self.read(0, 1)
        self.parse_boot(boot)
        self._fat_cache = {}

    def close(self):
        k32.CloseHandle(wintypes.HANDLE(self.h))

    def read(self, vol_sector, count):
        """Read `count` sectors at a VOLUME-relative sector number."""
        pos = ctypes.c_longlong(0)
        if not k32.SetFilePointerEx(wintypes.HANDLE(self.h),
                                    ctypes.c_longlong(vol_sector * SECTOR),
                                    ctypes.byref(pos), FILE_BEGIN):
            raise ctypes.WinError(ctypes.get_last_error())
        buf = ctypes.create_string_buffer(count * SECTOR)
        got = wintypes.DWORD(0)
        if not k32.ReadFile(wintypes.HANDLE(self.h), buf, count * SECTOR,
                            ctypes.byref(got), None):
            raise ctypes.WinError(ctypes.get_last_error())
        return buf.raw[:got.value]

    def parse_boot(self, b):
        # Offsets exactly as sampled in bmp_read.v's ST_SCAN_BOOT case statement.
        self.sig = b[510:512]
        self.bytes_per_sec, = struct.unpack_from("<H", b, 11)
        self.sec_per_clus = b[13]
        self.rsvd_sec, = struct.unpack_from("<H", b, 14)
        self.num_fats = b[16]
        self.root_ent_cnt, = struct.unpack_from("<H", b, 17)
        self.fat_sz16, = struct.unpack_from("<H", b, 22)
        self.hid_sec, = struct.unpack_from("<I", b, 28)
        self.tot_sec32, = struct.unpack_from("<I", b, 32)
        self.fat_sz32, = struct.unpack_from("<I", b, 36)
        self.root_clus, = struct.unpack_from("<I", b, 44)
        self.fs_id = (b[82:90].rstrip(b"\x00 ").decode("latin1")
                      if len(b) > 90 else "?")

        # bmp_read.v: bpb_fat_size = fat_sz16 if non-zero else fat_sz32, then
        # bpb_fat_area_sectors = fat_size << 1 for num_fats of 2 (or anything
        # that is not 1).
        self.fat_size = self.fat_sz16 if self.fat_sz16 else self.fat_sz32
        self.fat_area = self.fat_size if self.num_fats == 1 else self.fat_size * 2
        # Volume-relative data area start. The RTL's data_start_sector is this
        # plus hid_sec, because it works in absolute LBA from the partition.
        self.data_start_vol = self.rsvd_sec + self.fat_area
        self.data_start_abs = self.hid_sec + self.data_start_vol
        self.root_dir_vol = self.data_start_vol + (self.root_clus - 2) * self.sec_per_clus
        self.fat_start_vol = self.rsvd_sec

    def disk_extent_start(self):
        """Cross-check hid_sec against the volume's real byte offset on disk."""
        out = ctypes.create_string_buffer(4096)
        got = wintypes.DWORD(0)
        ok = k32.DeviceIoControl(wintypes.HANDLE(self.h),
                                 FSCTL_GET_VOLUME_DISK_EXTENTS, None, 0,
                                 out, 4096, ctypes.byref(got), None)
        if not ok or got.value < 32:
            return None
        raw = out.raw
        n, = struct.unpack_from("<I", raw, 0)
        if n < 1:
            return None
        # DISK_EXTENT is 8-byte aligned: DiskNumber at 8, StartingOffset at 16.
        start, = struct.unpack_from("<q", raw, 16)
        return start // SECTOR

    # -- FAT32 cluster chain, which the RTL deliberately does NOT follow ------
    def fat_entry(self, clus):
        """One FAT32 entry, masked to 28 bits. FAT sectors are cached because a
        chain walk touches thousands of entries that mostly share a sector."""
        off = clus * 4
        sec = off // SECTOR
        if sec not in self._fat_cache:
            self._fat_cache[sec] = self.read(self.fat_start_vol + sec, 1)
        raw = self._fat_cache[sec]
        if len(raw) < SECTOR:
            return None
        val, = struct.unpack_from("<I", raw, off % SECTOR)
        return val & FAT_MASK

    def chain(self, first_clus, size_bytes, max_clusters=1 << 20):
        """Walk the chain and say whether it is contiguous.

        Returns (clusters, contiguous, terminated, walked). `clusters` is the
        chain in order; `contiguous` is True only if every step is exactly +1,
        which is the one layout the RTL's LBA+1 read law can survive. `terminated`
        is True if the chain ended on an end-of-chain marker rather than on a
        free/bad entry or the iteration cap. `walked` is how many clusters were
        visited, so a caller can tell a short chain from a truncated walk.
        """
        need = (size_bytes + self.sec_per_clus * SECTOR - 1) // (
            self.sec_per_clus * SECTOR)
        clusters = []
        terminated = False
        clus = first_clus
        while clus is not None and len(clusters) < max(need, 1) + 1 \
                and len(clusters) < max_clusters:
            clusters.append(clus)
            nxt = self.fat_entry(clus)
            if nxt is None:
                break
            if nxt >= FAT_EOC:
                terminated = True
                break
            if nxt == FAT_FREE or nxt == FAT_BAD:
                break
            # A non-consecutive step does NOT end the walk: reporting only the
            # first break hides how badly fragmented the file is, and the count
            # is what tells the user whether a reformat will fix it.
            clus = nxt
        contiguous = all(b == a + 1 for a, b in zip(clusters, clusters[1:]))
        return clusters, contiguous, terminated, len(clusters)


def cluster_offset(delta, spc):
    """bmp_read.v's cluster_sector_offset function, enumerated case by case."""
    table = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7}
    if spc in table:
        return delta << table[spc]
    return delta          # the RTL's default branch, and it is a silent one


def parse_entry(e):
    """One 32 byte directory slot, decomposed the way ST_SCAN_DIR decomposes it."""
    fb = e[0]
    attr = e[11]
    ext = e[8:11]
    chi, = struct.unpack_from("<H", e, 20)
    clo, = struct.unpack_from("<H", e, 26)
    size, = struct.unpack_from("<I", e, 28)
    clus = (chi << 16) | clo

    name = e[0:8].rstrip(b"\x20").decode("latin1", "replace")
    exts = ext.decode("latin1", "replace").rstrip("\x00 ")
    label = ("%s.%s" % (name, exts)) if exts else name
    if fb == 0x00:
        label = "<FREE / end of directory>"
    elif fb == 0xE5:
        label = "<DELETED %s>" % (e[1:8].rstrip(b"\x20").decode("latin1", "replace"))

    # The RTL's predicate, term by term.
    is_lfn = (attr == 0x0F)
    is_label = bool(attr & 0x08)
    is_dir = bool(attr & 0x10)
    used = (fb != 0x00) and (fb != 0xE5)
    is_file = used and not is_lfn and not is_label and not is_dir \
        and clus >= 2 and size != 0

    return {
        "fb": fb, "attr": attr, "ext": ext, "label": label, "clus": clus,
        "size": size, "is_lfn": is_lfn, "is_dir": is_dir, "is_label": is_label,
        "used": used, "is_file": is_file,
        "is_bmp": is_file and ext.lower() == b"bmp",
        "is_wav": is_file and ext.lower() == b"wav",
    }


def read_dir_sectors(vol, max_sectors=ROOT_SCAN_MAX_SECTORS):
    """Pre-read the root directory so the replay can also be run over a
    permuted in-memory copy of it -- which is how the control below proves the
    order assertion has teeth without touching the card."""
    out = []
    for i in range(max_sectors):
        raw = vol.read(vol.root_dir_vol + i, 1)
        if len(raw) < SECTOR:
            out.append(raw)
            break
        out.append(raw)
    return out


def walk_all(dir_sectors, vol):
    """Every slot in the directory, with no early stop. This is the ground truth
    of what is ON the card, as opposed to what the RTL would SEE."""
    entries = []
    for si, raw in enumerate(dir_sectors):
        for i in range(len(raw) // ENTRY):
            ent = parse_entry(raw[i * ENTRY:(i + 1) * ENTRY])
            ent["sec"] = si
            ent["idx"] = i
            if ent["fb"] == 0x00:
                return entries          # genuinely free from here on
            if ent["is_file"]:
                off = cluster_offset(ent["clus"] - 2, vol.sec_per_clus)
                ent["abs_sector"] = vol.data_start_abs + off
                entries.append(ent)
    return entries


def replay_scan(dir_sectors, vol, target, verbose=False):
    """Walk the root directory exactly as ST_SCAN_DIR does and report where it
    stops and what it recorded."""
    print("root directory replay")
    print("  root cluster           %d" % vol.root_clus)
    print("  root dir sector        %d volume-relative, %d absolute LBA"
          % (vol.root_dir_vol, vol.hid_sec + vol.root_dir_vol))
    print("  scan_target_count      %d" % target)
    print("")

    found = []
    wavs = []             # every WAV reported, in slot order -- no first-only latch
    stopped = None
    sec_count = 0
    while sec_count < len(dir_sectors):
        raw = dir_sectors[sec_count]
        if len(raw) < SECTOR:
            stopped = "short read on directory sector %d" % sec_count
            break
        for i in range(SECTOR // ENTRY):
            ent = parse_entry(raw[i * ENTRY:(i + 1) * ENTRY])
            fb = ent["fb"]
            clus = ent["clus"]
            size = ent["size"]
            label = ent["label"]
            slot = "sec%d ent%02d" % (sec_count, i)

            if ent["is_bmp"]:
                # dir_file_sector_now, the value the RTL would record.
                off = cluster_offset(clus - 2, vol.sec_per_clus)
                abs_sector = vol.data_start_abs + off
                found.append((label, clus, size, abs_sector, sec_count, i))
                note = "FOUND  cluster %d -> LBA %d" % (clus, abs_sector)
            elif ent["is_wav"]:
                # The RTL's third branch: else-if after the BMP test, so a WAV is
                # never counted toward the BMP target and never triggers the
                # early-stop. Reported for EVERY match now, in slot order, which
                # is what makes track N belong to picture N.
                off = cluster_offset(clus - 2, vol.sec_per_clus)
                abs_sector = vol.data_start_abs + off
                wavs.append((label, clus, size, abs_sector, sec_count, i))
                note = "WAV    reported, cluster %d -> LBA %d, size %d" % (
                    clus, abs_sector, size)
            elif fb == 0x00:
                note = "STOP   first_byte 0x00, scan_done asserted here"
            else:
                why = []
                if ent["is_lfn"]:
                    why.append("LFN slot, attr 0x0F")
                if ent["is_dir"]:
                    why.append("subdirectory, attr bit 4")
                if ent["is_label"]:
                    why.append("volume label, attr bit 3")
                if fb == 0xE5:
                    why.append("deleted, 0xE5")
                if ent["used"] and not ent["is_dir"] and not ent["is_lfn"] \
                        and not ent["is_label"]:
                    if clus < 2:
                        why.append("cluster %d < 2" % clus)
                    if size == 0:
                        why.append("size 0")
                    if not ent["is_bmp"] and not ent["is_wav"]:
                        why.append("extension %r is neither BMP nor WAV"
                                   % ent["ext"].decode("latin1", "replace"))
                note = "skip   %s" % ("; ".join(why) if why else "not a file entry")

            if verbose or ent["is_bmp"] or ent["is_wav"] or fb == 0x00:
                print("  %s  fb=0x%02x attr=0x%02x  %-30s %s"
                      % (slot, fb, ent["attr"], label, note))

            if fb == 0x00:
                stopped = ("scan_done at directory sector %d entry %d: "
                           "first_byte == 0x00" % (sec_count, i))
                break
            if ent["is_bmp"] and len(found) >= target:
                stopped = ("scan_done at directory sector %d entry %d: "
                           "scan_found_total reached scan_target_count %d"
                           % (sec_count, i, target))
                break
        if stopped:
            break
        sec_count += 1
    else:
        stopped = "reached ROOT_SCAN_MAX_SECTORS = %d" % len(dir_sectors)

    print("")
    print("  scanner recorded %d image(s) and %d track(s), stop reason:"
          % (len(found), len(wavs)))
    print("    %s" % stopped)
    return found, stopped, wavs


def slot_pos(sec, idx):
    """Absolute slot number, so WAV and BMP positions are directly comparable."""
    return sec * (SECTOR // ENTRY) + idx


def check_order(found, wavs, target):
    """Every WAV the RTL is supposed to play must be reported BEFORE the scan
    stops, and the scan stops at the target-th BMP."""
    print("directory order (the contract the early-stop imposes)")
    if not wavs:
        print("  no WAV was reported by the replay at all")
        return False, 0
    if not found:
        print("  %d WAV(s) reported and no BMP, so nothing stops the scan early:"
              % len(wavs))
        for w in wavs:
            print("    slot %-6d %s" % (slot_pos(w[4], w[5]), w[0]))
        return True, len(wavs)

    last_bmp_slot = slot_pos(found[-1][4], found[-1][5])
    stop_slot = last_bmp_slot if len(found) >= target else None
    if stop_slot is not None:
        where = "at the %d-th BMP, slot %d" % (target, stop_slot)
    else:
        where = ("on a 0x00 slot at %d, before reaching %d BMPs"
                 % (last_bmp_slot, target))
    print("  %d BMP(s) found, %d WAV(s) reported; the scan stops %s"
          % (len(found), len(wavs), where))
    for n, w in enumerate(wavs):
        s = slot_pos(w[4], w[5])
        rel = ""
        if stop_slot is not None:
            rel = "  <- BEFORE the stop" if s < stop_slot else "  <- AFTER the stop"
        print("    track %d  slot %-6d %-14s LBA %-10d size %d%s"
              % (n, s, w[0], w[3], w[2], rel))

    if stop_slot is None:
        ok = True
    else:
        late = [w for w in wavs if slot_pos(w[4], w[5]) > stop_slot]
        ok = not late
        if late:
            print("")
            print("  VERDICT: %d WAV(s) sit AFTER the scan's stop point and are"
                  % len(late))
            print("    therefore never reported: %s"
                  % ", ".join(w[0] for w in late))
            print("    Their pictures will play silent. Rewrite the card with")
            print("    tools/sync_to_sd.py, which writes every WAV before any")
            print("    BMP, or move the WAVs ahead of the BMPs by hand.")
    return ok, len(wavs)


def check_contiguity(vol, entries, what):
    """Follow each file's real FAT chain and compare it against the LBA+1 the
    RTL actually performs."""
    print("%s physical contiguity (the RTL reads LBA+1, never the chain)" % what)
    if vol.sec_per_clus * SECTOR == 0:
        print("  cannot check: sectors per cluster is 0")
        return True
    all_ok = True
    for ent in entries:
        clusters, contiguous, terminated, walked = vol.chain(
            ent["clus"], ent["size"])
        need = (ent["size"] + vol.sec_per_clus * SECTOR - 1) // (
            vol.sec_per_clus * SECTOR)
        if contiguous and terminated and walked >= need:
            print("    ok    %-14s %6d clusters, all consecutive, chain ends on "
                  "EOC after %d bytes" % (ent["label"], walked, ent["size"]))
        else:
            all_ok = False
            breaks = sum(1 for a, b in zip(clusters, clusters[1:]) if b != a + 1)
            print("    FAIL  %-14s %d clusters, %d of them NOT consecutive, "
                  "terminated=%s, file needs %d clusters"
                  % (ent["label"], walked, breaks, terminated, need))
            print("          the reader would run off the end of cluster %d and "
                  "start decoding whatever follows it" % clusters[0])
    return all_ok


def control_order_has_teeth(vol, dir_sectors, target, found, wavs):
    """Negative control: prove the order check can actually fail.

    Takes an in-memory copy of the real directory and moves the LAST reported WAV
    to a slot after the stop point, then re-runs the identical replay. If the
    replay still reports the same number of WAVs, the order assertion above is
    vacuous and the whole report is worthless -- so this must bite.

    Only the WAV's slot position may change. The vacated slot becomes 0xE5
    (deleted: the RTL skips it and keeps walking) rather than 0x00, because 0x00
    is the end-of-directory marker and would stop the replay on the spot, losing
    the BMPs as well and making the WAV loss over-determined. The moved entry
    keeps its original bytes verbatim. And the BMP count is required to come back
    unchanged, so what the control actually demonstrates is "same scan, same
    pictures, WAV invisible purely because of where it sits".
    """
    print("")
    print("control: does the directory-order check have teeth?")
    if len(found) < target or not wavs:
        print("  skipped -- the real card does not reach scan_target_count or "
              "reported no WAV, so there is no stop point to move a WAV past")
        return True

    entries = []
    for si, raw in enumerate(dir_sectors):
        for i in range(len(raw) // ENTRY):
            entries.append([si, i, raw[i * ENTRY:(i + 1) * ENTRY]])
    by_slot = {slot_pos(si, i): k for k, (si, i, _e) in enumerate(entries)}

    last_wav = max(wavs, key=lambda w: slot_pos(w[4], w[5]))
    stop = max(slot_pos(f[4], f[5]) for f in found)
    src = slot_pos(last_wav[4], last_wav[5])

    # Destination: the first slot strictly past the stop that holds no live file
    # entry, so the move displaces nothing the replay would otherwise have read.
    dst = None
    for s in range(stop + 1, len(entries)):
        ent = parse_entry(entries[by_slot[s]][2])
        if not ent["is_file"]:
            dst = s
            break
    if dst is None:
        print("  skipped -- every slot past the stop at %d holds a live file, so "
              "there is nowhere to move the WAV without displacing one" % stop)
        return True
    if src not in by_slot or dst not in by_slot:
        print("  skipped -- could not locate slots %d and %d" % (src, dst))
        return True

    original = entries[by_slot[src]][2]
    vacated = bytearray(original)
    vacated[0] = 0xE5                       # deleted: skipped, scan continues
    entries[by_slot[src]][2] = bytes(vacated)
    entries[by_slot[dst]][2] = original     # the WAV entry, byte for byte

    permuted = []
    for si in range(len(dir_sectors)):
        buf = bytearray(dir_sectors[si])
        for i in range(len(dir_sectors[si]) // ENTRY):
            for (esi, ei, data) in entries:
                if esi == si and ei == i:
                    buf[i * ENTRY:(i + 1) * ENTRY] = data
        permuted.append(bytes(buf))

    print("  moved %s from slot %d to slot %d, past the stop at %d; the vacated "
          "slot became 0xE5 so the walk continues" % (last_wav[0], src, dst, stop))
    f2, stop2, w2 = replay_scan_quiet(permuted, vol, target)
    print("  permuted replay: %d BMP(s), %d WAV(s), stop: %s"
          % (len(f2), len(w2), stop2))
    if len(f2) != len(found):
        print("  FAIL  the control changed the BMP count too (%d -> %d), so it is "
              "not isolating the WAV's position; the order verdict above is "
              "unproven" % (len(found), len(f2)))
        return False
    if len(w2) < len(wavs):
        print("  ok    control bites: the same replay, over the same four "
              "pictures, loses %d of %d WAV(s) once the slot order changes -- so "
              "the order check above is about position and nothing else"
              % (len(wavs) - len(w2), len(wavs)))
        return True
    print("  FAIL  control did NOT bite: the replay reports %d WAV(s) either way, "
          "so the order assertion proves nothing" % len(w2))
    return False


def replay_scan_quiet(dir_sectors, vol, target):
    """replay_scan without the printing, for the control."""
    import io
    import contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        return replay_scan(dir_sectors, vol, target, verbose=False)


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    drive = argv[1]
    target = 4
    do_contig = "--no-contig" not in argv
    verbose = "--verbose" in argv or "-v" in argv
    if "--target" in argv:
        target = int(argv[argv.index("--target") + 1])

    vol = Volume(drive)
    try:
        print("=" * 72)
        print("bmp_read.v root directory scan replay: %s" % drive)
        print("=" * 72)
        print("volume geometry, parsed at the exact offsets the RTL samples")
        print("  signature              %s  (boot_is_fat32 needs 55aa)" % vol.sig.hex())
        print("  file system id         %r" % vol.fs_id)
        print("  bytes per sector       %d" % vol.bytes_per_sec)
        print("  sectors per cluster    %d" % vol.sec_per_clus)
        print("  reserved sectors       %d" % vol.rsvd_sec)
        print("  number of FATs         %d" % vol.num_fats)
        print("  FAT size               %d (16 bit field %d, 32 bit field %d)"
              % (vol.fat_size, vol.fat_sz16, vol.fat_sz32))
        print("  FAT area               %d sectors" % vol.fat_area)
        print("  hidden sectors         %d  <- this is the partition LBA the RTL"
              " adds as boot_sector_lba" % vol.hid_sec)
        print("  data area start        %d volume-relative, %d absolute LBA"
              % (vol.data_start_vol, vol.data_start_abs))
        print("")

        extent = vol.disk_extent_start()
        if extent is None:
            print("  partition offset       not queryable without elevation; "
                  "using BPB hidden sectors %d" % vol.hid_sec)
        else:
            agree = (extent == vol.hid_sec)
            print("  partition offset       %d from the volume disk extent, %d "
                  "from BPB_HiddSec -> %s"
                  % (extent, vol.hid_sec, "AGREE" if agree else "DISAGREE"))
            if not agree:
                print("      the RTL derives boot_sector_lba from the MBR "
                      "partition table, not from BPB_HiddSec, so a disagreement "
                      "here is worth reading carefully rather than dismissing")
        print("")

        if vol.sec_per_clus not in (1, 2, 4, 8, 16, 32, 64, 128):
            print("  WARNING: %d sectors per cluster hits the DEFAULT branch of "
                  "cluster_sector_offset, which returns cluster_delta unshifted. "
                  "Every sector address the scanner records would then be wrong."
                  % vol.sec_per_clus)
            print("")

        dir_sectors = read_dir_sectors(vol)
        found, stopped, wavs = replay_scan(dir_sectors, vol, target,
                                           verbose=verbose)
        on_card = walk_all(dir_sectors, vol)
        order_ok, n_wavs = check_order(found, wavs, target)
        print("")

        contig_ok = True
        if do_contig:
            wav_entries = [e for e in on_card if e["is_wav"]]
            bmp_entries = [e for e in on_card if e["is_bmp"]]
            contig_ok = check_contiguity(vol, wav_entries, "WAV")
            print("")
            contig_ok = check_contiguity(vol, bmp_entries, "BMP") and contig_ok
            print("")

        wavs_on_disk = sorted(e for e in os.listdir(
            drive.rstrip("\\") + "\\") if e.lower().endswith(".wav"))
        print("Explorer sees %d WAV(s): %s"
              % (len(wavs_on_disk), ", ".join(wavs_on_disk) if wavs_on_disk else "none"))
        print("The scanner reports %d: %s"
              % (n_wavs, ", ".join(w[0] for w in wavs) if wavs else "none"))
        if wavs_on_disk and n_wavs < len(wavs_on_disk):
            print("")
            print("VERDICT: %d WAV(s) are on the card but never reported, so "
                  "wav_found_count stays below them and the pictures they belong "
                  "to play silent." % (len(wavs_on_disk) - n_wavs))
            print("  The scanner stops as soon as scan_found_total reaches %d "
                  "BMPs, so every WAV entry must sit BEFORE the last BMP in" % target)
            print("  physical slot order. sync_to_sd.py writes them in that "
                  "order; a hand-copied card often does not.")
            order_ok = False

        control_ok = control_order_has_teeth(vol, dir_sectors, target,
                                             found, wavs)

        print("")
        print("=" * 72)
        bmps_on_disk = sorted(e for e in os.listdir(
            drive.rstrip("\\") + "\\") if e.lower().endswith(".bmp"))
        print("Explorer sees %d BMP(s): %s" % (len(bmps_on_disk), ", ".join(bmps_on_disk)))
        print("The scanner sees %d: %s"
              % (len(found), ", ".join(f[0] for f in found) if found else "none"))
        if len(found) < len(bmps_on_disk):
            missing = [n for n in bmps_on_disk
                       if n.lower() not in [f[0].lower() for f in found]]
            print("")
            print("VERDICT: %d file(s) are invisible to the scanner: %s"
                  % (len(missing), ", ".join(missing)))
            print("  They are perfectly healthy files -- Explorer and the PC read")
            print("  them fine. They are hidden because a 0x00 directory slot sits")
            print("  before them, and ST_SCAN_DIR treats 0x00 as end of directory")
            print("  and asserts scan_done there. That is correct FAT semantics")
            print("  for a freshly written directory, but Windows does not always")
            print("  keep the slots compacted, so a hole can survive in the middle.")
            print("  This is a CARD LAYOUT problem, not an RTL misparse: every")
            print("  offset the scanner samples matches the FAT specification.")
            return 1
        if len(found) == len(bmps_on_disk):
            print("")
            print("VERDICT: the scanner sees every BMP on the card, so the")
            print("  two-of-four symptom is NOT a directory scan problem. Look at")
            print("  the load path instead: load_failed from header_match_r, or")
            print("  the one second stall watchdog in sd_card_bmp.v.")

        print("")
        if order_ok and contig_ok and control_ok and len(found) == len(bmps_on_disk):
            print("CARD OK: every BMP and every WAV is reported in slot order, "
                  "and each file's clusters are consecutive, so the LBA+1 read law "
                  "holds for all of them.")
            print("  track -> picture mapping the RTL will build:")
            for n, w in enumerate(wavs):
                print("    picture %d  <-  %s  (LBA %d, %d bytes, %d sectors)"
                      % (n, w[0], w[3], w[2],
                         (w[2] + SECTOR - 1) // SECTOR))
            if len(wavs) < len(found):
                print("  NOTE: %d picture(s) have no track of their own. "
                      "track_req_lim clamps them onto slot 0, so they share the "
                      "first song rather than playing silence."
                      % (len(found) - len(wavs)))
            return 0
        print("CARD NOT OK: see the FAIL/VERDICT lines above.")
        if not control_ok:
            print("  and the order control did not bite, so treat the order "
                  "verdict as unproven either way.")
        print("=" * 72)
        return 1
    finally:
        vol.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
