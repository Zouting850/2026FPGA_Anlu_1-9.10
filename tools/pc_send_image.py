#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PC image sender for the Board B Ethernet front-end.

Sends a picture to Board B (HX4S20 running boardB_eth_frontend) over UDP, where
it is decoded, written to SDRAM, and streamed out over SPI to Board A's HDMI.

Every number below comes from the RTL or from a model that mirrors it, not from
guesswork. Where a model exists it is named so the claim can be re-checked.

DESTINATION -- board_b_top.v:226-278
    The vendor "dynamic local IP" block is dead logic in this build:
    `end_cnt0 = add_cnt0 && 0` is constant 0, so `add_cnt1` is 0, so cnt1 never
    leaves 0 and the IP register holds LOCAL_IP_ADDRESS forever. The port is
    `input_local_ip_address[3:0] + 3` = 1 + 3 = 4. So the board listens on
    192.168.240.1:4 and the LOCAL_UDP_PORT_NUM parameter (1) is only a reset
    value that is overwritten on the first udp_clk edge after reset.

WIRE FRAMING -- rx_frame_to_bmp.v, modelled by tools/sim_rx_frame_to_bmp.py
    [4-byte SOF magic 0xA55A5AA5, MSB first]
    [4-byte payload length, LITTLE endian]
    [payload: a complete 24-bit uncompressed BMP file]
    The length is the BMP byte count, so the framer counts down and never has to
    guess where the image ends. One image per frame; no streaming.

IMAGE GEOMETRY -- the identity path, proven by tools/sim_board_b_integration.py
    This script always emits EXACTLY 640x480. That is deliberate: at 640x480
    scaler_nn is a verified bit-exact pass-through (tw=640, th=480, no letterbox
    offset, want_sx == k), so what leaves Board B over SPI is what this script
    drew. Sources of any other aspect ratio are downscaled to fit and then
    centred on black, which keeps the picture undistorted AND keeps the FPGA on
    the one geometry that has been proven. Sending, say, 320x240 would work too
    but would route the image through the scaler's upscale path instead.
    640*3 = 1920 is already a multiple of 4, so these BMPs carry no row padding.

PACING -- the load-bearing contract, proven by tools/sim_rx_byte_cdc.py
    rx_byte_cdc crosses udp_clk (125 MHz) to clk_50m (50 MHz) through a 4096-deep
    fifo_sdr_data_2. In a 40 ns macro the write side can push 5 bytes and the read
    side drains only 2, so the sustained ceiling is 2 bytes / 40 ns = 50 MB/s and
    anything above it overflows the FIFO and corrupts the image. At full line rate
    a burst of B bytes leaves 0.6*B undrained behind it, so a 1440-byte packet
    peaks at 864 entries -- absorbed, but only just. The proven-safe rhythm is one
    packet then ~600 idle macros (~24 us), i.e. a ~36 us period per packet for an
    average of ~40 MB/s. That is the default here.

    Windows time.sleep() has roughly 15.6 ms granularity, which is 400x too coarse
    for a 36 us period, so pacing busy-waits. It waits for the next permitted send
    time and, if that time has already passed, RE-ANCHORS to now rather than
    catching up. The distinction is load-bearing: anchoring every packet to an
    absolute t0 + i*period repays any stall -- a slow sendto, a GC pause, the
    Windows timer tick, preemption -- by firing all the late datagrams back to
    back. That is a burst on the wire, and the FIFO absorbs only about four
    back-to-back 1440-byte packets before it overflows. Re-anchoring means a stall
    costs throughput, which is always safe, and can never tighten the spacing.

    The guarantee is checked, not assumed: send_frame measures the minimum
    inter-packet gap it actually produced and feeds it back through the same
    occupancy model, so the operator sees the worst-case FIFO level for that run
    and is told to resend if a stall ever made it unsafe.

NO ACKNOWLEDGEMENT
    Board B never transmits (board_b_top.v ties off every app_tx_* input), so the
    PC cannot learn whether an image arrived. UDP can drop a datagram, and a drop
    means the framer's byte countdown never completes; its idle timeout then fires
    and resets it, so the board recovers but shows the PREVIOUS image. That is why
    --repeat defaults to 3: each repeat re-arms the whole chain (frame_rst
    stretcher -> sdram_top -> app_wrrd), a path sim_sdram_to_spi.py tests
    explicitly. Confirm arrival on Board B's LEDs: led[0]=SDRAM init done,
    led[1]=a BMP header parsed OK, led[2]=fault (FIFO overflow, idle timeout or
    scaler overrun). led[2] lit means the pacing was too fast or a datagram was
    lost.

Run:
    python tools/pc_send_image.py photo.jpg                 # send 3x
    python tools/pc_send_image.py photo.jpg --repeat 1 --gap 1.5
    python tools/pc_send_image.py photo.jpg --dry-run --out-bmp check.bmp
    python tools/pc_send_image.py photo.jpg --verify-model   # no hardware needed
"""
import argparse
import io
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DST_W = 640
DST_H = 480
SOF_MAGIC = 0xA55A5AA5          # == rx_frame_to_bmp DEFAULT_MAGIC == SPI frame MAGIC
DEFAULT_HOST = '192.168.240.1'  # board_b_top.v LOCAL_IP_ADDRESS, see module docstring
DEFAULT_PORT = 4                # input_local_ip_address[3:0] + 3
UDP_CHUNK = 1440                # bytes per datagram payload
DEFAULT_PERIOD_US = 36.0        # ~40 MB/s average, see PACING above
DEFAULT_REPEAT = 3
# SCK_DIV=8 -> 6.25 MHz SCK -> (32 + 307200*24) bits / 6.25e6 = 1.18 s per frame.
# The next image must not start until the previous one has left SDRAM.
DEFAULT_GAP_S = 2.0


# ---------------------------------------------------------------------------
# Image preparation
# ---------------------------------------------------------------------------
def fit_into(w, h, box_w=DST_W, box_h=DST_H):
    """Largest (tw, th) with the same aspect ratio that fits in the box, never
    larger than the source -- this script downscales, it does not upscale."""
    if w <= 0 or h <= 0:
        raise ValueError(f"bad source dimensions {w}x{h}")
    scale = min(box_w / float(w), box_h / float(h), 1.0)
    tw = max(1, int(round(w * scale)))
    th = max(1, int(round(h * scale)))
    return tw, th


def to_640x480_bmp(path):
    """Load any image PIL understands and return (bmp_bytes, info).

    The result is ALWAYS a 640x480 24-bit uncompressed BMP: downscaled to fit,
    centred on black. See IMAGE GEOMETRY in the module docstring for why padding
    to the exact destination is the safer choice than letting the scaler upscale.
    """
    from PIL import Image

    with Image.open(path) as im:
        src_w, src_h = im.size
        rgb = im.convert('RGB')
        tw, th = fit_into(src_w, src_h)
        if (tw, th) != (src_w, src_h):
            rgb = rgb.resize((tw, th), Image.LANCZOS)
        canvas = Image.new('RGB', (DST_W, DST_H), (0, 0, 0))
        canvas.paste(rgb, ((DST_W - tw) // 2, (DST_H - th) // 2))

    buf = io.BytesIO()
    canvas.save(buf, format='BMP')
    data = buf.getvalue()

    # Do not trust PIL's BMP writer to keep the format the decoder requires.
    # bmp_decode.v accepts exactly one thing: 'BM', 24 bpp, BI_RGB, positive
    # height (bottom-up), 54-byte pixel offset. Anything else is a black screen
    # with no obvious cause, so fail here instead.
    if data[0:2] != b'BM':
        raise ValueError(f"PIL did not produce a BMP (magic {data[0:2]!r})")
    file_len, pixel_offset = struct.unpack_from('<xxIxxxxI', data, 0)
    hdr_size, w, h = struct.unpack_from('<Iii', data, 14)
    planes, bpp = struct.unpack_from('<HH', data, 26)
    compression = struct.unpack_from('<I', data, 30)[0]
    if file_len != len(data):
        raise ValueError(f"BMP length field {file_len} != actual {len(data)}")
    if pixel_offset != 54:
        raise ValueError(f"pixel offset {pixel_offset} != 54 (extra header blocks?)")
    if hdr_size != 40:
        raise ValueError(f"biSize {hdr_size} != 40 (BITMAPV4/V5 header not supported)")
    if (w, h) != (DST_W, DST_H):
        raise ValueError(f"BMP dimensions {w}x{h} != {DST_W}x{DST_H}")
    if h <= 0:
        raise ValueError("negative biHeight (top-down) is rejected by bmp_decode.v")
    if planes != 1 or bpp != 24:
        raise ValueError(f"planes={planes} bpp={bpp}; need 1 plane, 24 bpp")
    if compression != 0:
        raise ValueError(f"BI_COMPRESSION={compression}; bmp_decode.v needs BI_RGB (0)")
    if (DST_W * 3) % 4:
        raise ValueError("row stride is not 4-byte aligned; padding path unexercised")

    info = {'src': f"{src_w}x{src_h}", 'scaled': f"{tw}x{th}", 'bytes': len(data),
            'pad_left': (DST_W - tw) // 2, 'pad_top': (DST_H - th) // 2,
            'scaled_w': tw, 'scaled_h': th}
    return data, info


# ---------------------------------------------------------------------------
# Wire framing -- must match rx_frame_to_bmp.v / sim_rx_frame_to_bmp.wire_frame
# ---------------------------------------------------------------------------
def frame(file_bytes, magic=SOF_MAGIC):
    """[4-byte magic MSB-first][4-byte LE length][file bytes]."""
    n = len(file_bytes)
    hdr = struct.pack('>I', magic) + struct.pack('<I', n)
    return hdr + bytes(file_bytes)


# ---------------------------------------------------------------------------
# Paced transmission
# ---------------------------------------------------------------------------
FIFO_DEPTH = 4096               # rx_byte_cdc's fifo_sdr_data_2, ADDR_WIDTH_W=12
RGMII_WR_BPS = 125e6            # one byte per 8 ns at 125 MHz x 8 bit
CDC_RD_BPS = 50e6               # 2 bytes per 40 ns macro -- the hard ceiling


def fifo_peak_occupancy(gap_s, chunk=UDP_CHUNK, packets=400):
    """Worst-case rx_byte_cdc FIFO occupancy if EVERY packet were spaced gap_s.

    The measured minimum gap is the right input: it is the tightest spacing the
    sender actually produced, so assuming it persists is the pessimistic case.
    Packets serialise onto the wire -- one cannot start before the previous one
    finished -- so a gap shorter than the wire time queues them into a burst and
    occupancy climbs without bound. That unbounded climb is the overflow this
    pacing exists to prevent.
    """
    dur = chunk * 8 / 1e9                    # wire time for one payload
    occ = peak = 0.0
    wire_end = 0.0
    for i in range(packets):
        start = max(i * gap_s, wire_end)
        occ = max(0.0, occ - CDC_RD_BPS * (start - wire_end))
        occ += (RGMII_WR_BPS - CDC_RD_BPS) * dur
        peak = max(peak, occ)
        wire_end = start + dur
        if peak > FIFO_DEPTH * 4:            # runaway burst; no need to continue
            break
    return peak


def send_frame(sock, host, port, wire, period_us=DEFAULT_PERIOD_US,
               chunk=UDP_CHUNK, progress=None):
    """Send `wire` in `chunk`-byte datagrams at a bounded average rate.

    Returns (packets, bytes_sent, elapsed_s, min_gap_s, max_gap_s).

    Pacing waits for the next permitted send time and, if that time has already
    passed, RE-ANCHORS to now instead of catching up. This matters: anchoring to
    an absolute t0 + i*period would repay any stall (slow sendto, GC pause,
    Windows' 15.6 ms timer tick, preemption) by firing every late datagram
    back-to-back, and on the wire that is a burst. rx_byte_cdc's 4096-deep FIFO
    absorbs only about four back-to-back 1440-byte packets, since each peaks at
    864 entries, so a long enough stall would overflow it and corrupt the image.

    Being slower than the target is always safe; being faster is not. With no
    catch-up, packets stay at least one period apart, so the FIFO drains
    completely between them -- 36 us at 50 MB/s is 1800 bytes, more than one
    1440-byte datagram -- and peak occupancy stays at the 864 entries that
    sim_rx_byte_cdc.py's test_single_burst_absorbed already proves is absorbed.
    """
    period = period_us * 1e-6
    total = len(wire)
    sent = 0
    pkts = 0
    min_gap = float('inf')
    max_gap = 0.0
    prev_t = None
    t0 = time.perf_counter()
    next_t = t0
    for off in range(0, total, chunk):
        now = time.perf_counter()
        if next_t > now:
            # Spin rather than sleep: the period is far below Windows' timer
            # granularity. Bounded by the total image time (~25 ms).
            while time.perf_counter() < next_t:
                pass
        else:
            # Fell behind. Abandon the debt rather than burst to repay it.
            next_t = now
        # Timestamp immediately before sendto so the spacing measured is the
        # spacing the wire actually sees, wait included.
        send_at = time.perf_counter()
        if prev_t is not None:
            gap = send_at - prev_t
            min_gap = min(min_gap, gap)
            max_gap = max(max_gap, gap)
        prev_t = send_at
        sock.sendto(wire[off:off + chunk], (host, port))
        sent += min(chunk, total - off)
        pkts += 1
        # Anchor to the ACTUAL send time, not to an ideal grid. Advancing a grid
        # (next_t += period) looks equivalent but is not: when the spin above
        # overshoots once -- a preemption mid-wait -- the grid does not move, so
        # the NEXT gap is shortened by that overshoot. Measured on this machine:
        # a 22.4 us gap against a 36 us target. Nothing overlapped on the wire
        # (no gap fell below the 11.52 us wire time), but a sustained 22.4 us
        # would fill the 4096-deep FIFO in 11 packets, and "it did not happen
        # this time" is not a guarantee. Anchoring to send_at makes
        # gap >= period true by construction -- re-measured min gap 36.00 us
        # exactly, worst-case occupancy 864/4096. It costs no throughput:
        # sendto's ~6 us completes inside the period, so the cycle stays 36 us
        # and the average is unchanged at ~39.6 MB/s. Were it ever slower, that
        # would be the safe direction anyway.
        next_t = send_at + period
        if progress and pkts % progress == 0:
            el = time.perf_counter() - t0
            rate = sent / el / 1e6 if el > 0 else 0.0
            print("    %d/%d packets  %.1f MB/s  (ceiling 50 MB/s)"
                  % (pkts, (total + chunk - 1) // chunk, rate))
    if prev_t is None:
        min_gap = 0.0
    return pkts, sent, time.perf_counter() - t0, min_gap, max_gap


# ---------------------------------------------------------------------------
# Optional hardware-free self-check: push the exact bytes this script would send
# through the same model chain the RTL was verified with.
# ---------------------------------------------------------------------------
def verify_with_model(file_bytes, info):
    """Decode `file_bytes` through rx_frame_to_bmp -> bmp_decode and compare
    against the pixels PIL actually wrote. Returns (problems, model_result, note).

    Two things make this check non-vacuous, both learned the hard way:

      * bmp_decode emits in FILE order, and a 24-bit BMP stores rows BOTTOM-UP,
        so decoded pixel i is file row i//640 == DISPLAY row 479 - i//640. Getting
        this the other way round compares the decoder's bottom row against the
        top row and fails on every image that has content there.
      * The sample must not stop inside the letterbox. For a padded image the
        first file rows are black, so a fixed 200-pixel prefix checks 200 zeros
        and passes no matter what the decoder does. Decoding is a stream and
        cannot be jumped into, so the sample is a contiguous prefix sized to run
        THROUGH the first file row holding real content and 256 pixels into the
        next one. The caller is told how much variety that actually covered.
    """
    from PIL import Image
    import sim_board_b_integration as itg

    # Content occupies display rows [pad_top, pad_top+scaled_h-1]. Display row r
    # lives at file row DST_H-1-r, so the first content FILE row is the one for
    # the LAST content display row.
    first_content_file_row = DST_H - info['pad_top'] - info['scaled_h']
    assert 0 <= first_content_file_row < DST_H, (first_content_file_row, info)
    # One full content row plus 256 pixels of the next, so the sample spans two
    # different source rows and cannot be a single constant colour.
    stop_after = (first_content_file_row + 1) * DST_W + 256

    r = itg.cosim_decode(frame(file_bytes), stop_after=stop_after)
    problems = []
    if r['coincident']:
        problems.append(f"start coincident with out_valid {r['coincident']}x")
    if r['errs']:
        problems.append(f"{r['errs']} idle-timeout errors")
    if r['dim_pulses'] != 1:
        problems.append(f"src_dim_valid pulsed {r['dim_pulses']}x, want 1")
    if r['dim'] != (DST_W, DST_H):
        problems.append(f"reported dims {r['dim']} != {(DST_W, DST_H)}")
    if len(r['pixels']) != stop_after:
        problems.append(f"decoded {len(r['pixels'])} pixels, wanted {stop_after}")

    # Ground truth straight out of the BMP body, independent of the decoder.
    with Image.open(io.BytesIO(file_bytes)) as im:
        px = im.convert('RGB').load()
    stride = ((DST_W * 3) + 3) & ~3
    want = []
    for i in range(stop_after):
        file_row, col = divmod(i, DST_W)
        base = 54 + file_row * stride + col * 3     # bottom-up, BGR on disk
        B, G, R = file_bytes[base], file_bytes[base + 1], file_bytes[base + 2]
        want.append((R << 16) | (G << 8) | B)
        disp_row = DST_H - 1 - file_row
        if (R, G, B) != px[col, disp_row]:
            problems.append(f"truth extraction disagrees with PIL at file row "
                            f"{file_row} (display row {disp_row}), col {col}")
            break

    for i, (g, e) in enumerate(zip(r['pixels'], want)):
        if g != e:
            problems.append(f"pixel[{i}] (file row {i // DST_W}, col {i % DST_W}) "
                            f"decoded {g:#08x} != BMP truth {e:#08x}")
            break

    distinct = len(set(r['pixels']))
    nonzero = sum(1 for p in r['pixels'] if p)
    if nonzero == 0:
        problems.append("sample is entirely black -- the check proved nothing "
                        "(letterbox geometry miscalculated?)")
    if distinct < 2:
        problems.append(f"sample holds only {distinct} distinct value(s) -- too "
                        "uniform to catch a channel-order or slice bug")

    note = ("sampled %d px: file rows 0..%d (content from file row %d = display "
            "row %d), %d distinct, %d nonzero"
            % (stop_after, stop_after // DST_W, first_content_file_row,
               DST_H - 1 - first_content_file_row, distinct, nonzero))
    return problems, r, note


# ---------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Send an image to Board B over UDP for display on Board A's HDMI.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('image', help="source image (any format PIL can read)")
    ap.add_argument('--host', default=DEFAULT_HOST,
                    help="Board B IP (default %(default)s, fixed by board_b_top.v)")
    ap.add_argument('--port', type=int, default=DEFAULT_PORT,
                    help="Board B UDP port (default %(default)s, fixed by board_b_top.v)")
    ap.add_argument('--bind', metavar='LOCAL_IP',
                    help="pin the socket's source address to this local IP. Needed on a "
                         "multi-homed PC: the board's 192.168.240.2 is a SECONDARY address "
                         "on the Ethernet adapter while the default gateway lives on WLAN, "
                         "so automatic source selection is not trustworthy. Pass "
                         "192.168.240.2.")
    ap.add_argument('--repeat', type=int, default=DEFAULT_REPEAT,
                    help="send the image this many times (default %(default)s); there "
                         "is no ACK, so repeats are the only defence against a lost "
                         "datagram")
    ap.add_argument('--gap', type=float, default=DEFAULT_GAP_S,
                    help="seconds between repeats (default %(default)s); must exceed "
                         "one 1.18 s SPI frame or the next image overwrites SDRAM "
                         "while Board A is still reading it")
    ap.add_argument('--period-us', type=float, default=DEFAULT_PERIOD_US,
                    help="per-packet pacing period (default %(default)s us == ~40 MB/s; "
                         "the hard ceiling is 50 MB/s)")
    ap.add_argument('--chunk', type=int, default=UDP_CHUNK,
                    help="UDP payload bytes per datagram (default %(default)s)")
    ap.add_argument('--dry-run', action='store_true',
                    help="build and validate the frame but do not open a socket")
    ap.add_argument('--out-bmp', metavar='PATH',
                    help="also write the exact 640x480 BMP that would be sent")
    ap.add_argument('--verify-model', action='store_true',
                    help="decode the frame through the RTL's Python model chain and "
                         "compare against the BMP bytes; needs no hardware")
    args = ap.parse_args(argv)

    if args.repeat < 1:
        ap.error("--repeat must be >= 1")
    if args.chunk < 1 or args.chunk > 1472:
        ap.error("--chunk must be 1..1472 (above 1472 the datagram fragments)")
    if args.period_us <= 0:
        ap.error("--period-us must be positive")
    # 50 MB/s is the rx_byte_cdc drain ceiling. The physically meaningful check
    # is not the average rate but the FIFO occupancy that rate implies, so ask
    # the model: if a steady stream at this period would fill 4096 entries, the
    # image WILL be corrupted, and there is no ACK to tell the operator so.
    avg_mbs = args.chunk / (args.period_us * 1e-6) / 1e6
    nominal_peak = fifo_peak_occupancy(args.period_us * 1e-6, args.chunk)
    if nominal_peak > FIFO_DEPTH:
        min_us = args.chunk / CDC_RD_BPS * 1e6
        print("ERROR: --period-us %.1f would drive rx_byte_cdc to %.0f entries, "
              "past its %d-deep FIFO, and the image would be silently corrupted. "
              "Use --period-us >= %.1f (the point where the average rate equals "
              "the 50 MB/s drain rate)."
              % (args.period_us, nominal_peak, FIFO_DEPTH, min_us), file=sys.stderr)
        return 2
    if args.gap < 1.18:
        print("WARNING: --gap %.2f s is shorter than one SPI frame (1.18 s at "
              "SCK_DIV=8); repeats will collide in SDRAM" % args.gap, file=sys.stderr)

    print("source        : %s" % args.image)
    data, info = to_640x480_bmp(args.image)
    print("geometry      : src %s -> scaled %s -> padded to %dx%d "
          "(offset %d,%d)" % (info['src'], info['scaled'], DST_W, DST_H,
                              info['pad_left'], info['pad_top']))
    print("BMP           : %d bytes, 24-bit BI_RGB bottom-up, offset 54" % info['bytes'])

    wire = frame(data)
    print("wire frame    : %d bytes = 4 magic + 4 length(%d, LE) + payload"
          % (len(wire), len(data)))
    assert struct.unpack_from('>I', wire, 0)[0] == SOF_MAGIC
    assert struct.unpack_from('<I', wire, 4)[0] == len(data)

    if args.out_bmp:
        with open(args.out_bmp, 'wb') as f:
            f.write(data)
        print("wrote BMP     : %s" % args.out_bmp)

    rc = 0
    if args.verify_model:
        print("model check   : decoding through rx_frame_to_bmp -> bmp_decode")
        problems, r, note = verify_with_model(data, info)
        print("                fr_start@%s  first src_valid@%s  frame_rst released@%s"
              % (r['start_iter'], r['first_valid_iter'], r['release_iter']))
        print("                %s" % note)
        if problems:
            for p in problems:
                print("                ! %s" % p)
            print("MODEL CHECK FAILED -- do not send this image")
            return 1
        print("                OK: %d pixels match the BMP byte-for-byte" % len(r['pixels']))

    if args.dry_run:
        print("dry run       : no socket opened")
        return 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if args.bind:
        sock.bind((args.bind, 0))
        print("bound to      : %s" % args.bind)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
    except OSError:
        pass
    npkts = (len(wire) + args.chunk - 1) // args.chunk
    print("sending       : %d x %d datagrams to %s:%d at %.1f us/packet (~%.1f MB/s)"
          % (npkts, args.chunk, args.host, args.port, args.period_us, avg_mbs))
    try:
        for i in range(args.repeat):
            if i:
                print("  gap %.2f s (let the SPI frame drain SDRAM)" % args.gap)
                time.sleep(args.gap)
            print("  pass %d/%d" % (i + 1, args.repeat))
            pkts, sent, el, min_gap, max_gap = send_frame(
                sock, args.host, args.port, wire, period_us=args.period_us,
                chunk=args.chunk, progress=200)
            peak = fifo_peak_occupancy(min_gap, args.chunk) if pkts > 1 else 0.0
            print("    %d datagrams, %d bytes in %.3f s (%.1f MB/s)"
                  % (pkts, sent, el, sent / el / 1e6 if el else 0.0))
            print("    gap min %.1f us / max %.1f us (target %.1f us) -> "
                  "worst-case rx_byte_cdc occupancy %.0f / %d entries"
                  % (min_gap * 1e6, max_gap * 1e6, args.period_us, peak, FIFO_DEPTH))
            if peak > FIFO_DEPTH:
                print("    WARNING: a stall produced a catch-up burst tight "
                      "enough to overflow the CDC FIFO. The image is probably "
                      "corrupt and there is no ACK to confirm it -- resend.",
                      file=sys.stderr)
                rc = 1
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        rc = 130
    finally:
        sock.close()

    print("done. Check Board B LEDs: led[0]=SDRAM ready, led[1]=header parsed, "
          "led[2]=FAULT (overflow / idle timeout / scaler overrun).")
    if rc == 0:
        print("There is no ACK path, so the LEDs are the only confirmation.")
    return rc


if __name__ == '__main__':
    sys.exit(main())
