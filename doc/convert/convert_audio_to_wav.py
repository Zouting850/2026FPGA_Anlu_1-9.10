#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把任意音频（MP3 等）离线转成本工程 FPGA 可直接流式播放的标准 WAV。

为什么是 WAV 而不是 MP3：
    EG4S20 片内做真正的 MP3 解码（Huffman + IMDCT + 32 子带多相合成）不现实，
    与图片管线一致——图片是离线把 PNG/JPG 转成无压缩 BMP、FPGA 只读原始字节，
    音频同理：离线解码成无压缩 PCM，FPGA 只负责扇区流读、CDC 与采样节拍。

输出契约（FPGA RTL 依赖，勿改）：
    - 采样率 48000 Hz（HDMI 发射核固定 AUDIO_SAMPLE_RATE="48K"、ACR_N=6144）
    - 立体声、16-bit 小端、帧交错 L_lo,L_hi,R_lo,R_hi
    - 标准 44 字节 canonical 头：RIFF/WAVE/fmt (PCM)/data，头恰好 44 字节
      （手工拼头，不用 ffmpeg 的容器写出，避免它插入 LIST/INFO 等额外块，
        否则 data 块不在偏移 44，FPGA 的固定偏移解析会错位）
    - 默认文件名 MUSIC.WAV（8.3 短名，避免 LFN，扩展名固定 WAV）

本机依赖：ffmpeg（已验证 8.1.1）。Python 3 标准库即可，无需 numpy。
"""

import argparse
import json
import os
import struct
import subprocess
import sys

SAMPLE_RATE = 48000
CHANNELS = 2
BITS_PER_SAMPLE = 16
WAV_HEADER_LEN = 44

# EBU R128 目标。多轨联动之后才需要：四首来源不同的曲子背靠背播放，音量落差
# 在展台上是听得出来的，而这恰恰是「切图切歌」这个功能自己带来的问题——单轨
# 时代不存在。单轨默认仍然不做任何响度处理，只有显式 --loudnorm 才走这条路。
LOUDNORM_I = -16.0     # 目标综合响度 LUFS
LOUDNORM_TP = -1.5     # 真峰值上限 dBTP，留出余量防止 16-bit 削顶
LOUDNORM_LRA = 11.0    # 目标响度范围 LU


def measure_loudness(input_path, target_i=LOUDNORM_I):
    """
    loudnorm 第一遍：只测量，不改音频。返回 ffmpeg 报告的 measured_* 字典。

    两遍法而不是单遍的原因：单遍 loudnorm 是动态的，它会跟着音乐起伏实时拉增益，
    安静段落被抬起来、强段落被压下去，听感是「喘气」。第二遍把第一遍测到的
    measured_I/TP/LRA/thresh 喂回去并开 linear=true，ffmpeg 就只做一次固定增益，
    动态范围原样保留——只有当固定增益会顶破真峰值上限时才退回动态。

    target_i 必须与第二遍 decode_to_raw_pcm 用的值一致：字典里的 target_offset 是
    相对这个目标算出来的，第二遍直接把它喂回去。
    """
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"找不到输入音频: {input_path}")

    af = "loudnorm=I=%s:TP=%s:LRA=%s:print_format=json" % (
        target_i, LOUDNORM_TP, LOUDNORM_LRA)
    cmd = ["ffmpeg", "-hide_banner", "-v", "info", "-i", input_path,
           "-af", af, "-f", "null", "-"]
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        raise RuntimeError("未找到 ffmpeg，请先安装并加入 PATH（本机应已装 ffmpeg 8.1.1）。")

    err = proc.stderr.decode("utf-8", "replace")
    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg 响度测量失败:\n{err}")

    # JSON 是打在 stderr 上的，且前面还有一堆进度行，所以取最后一个花括号块。
    start = err.rfind("{")
    end = err.rfind("}")
    if start < 0 or end <= start:
        raise RuntimeError(f"ffmpeg 没有输出响度测量 JSON:\n{err}")
    try:
        return json.loads(err[start:end + 1])
    except ValueError:
        raise RuntimeError(f"响度测量 JSON 解析失败:\n{err[start:end + 1]}")


def loudest_common_target(measurements, tp_limit=LOUDNORM_TP):
    """
    给定一组测量结果，算出「每首都能只靠一次固定增益达到、且真峰值不越限」的
    最响综合响度。返回 (target_i, 卡住的那一首的下标)，组里不足一首返回 (None, None)。

    为什么需要它：LOUDNORM_I=-16 是流媒体习惯值，但商业母带普遍在 -8~-11 LUFS，
    一律拉到 -16 等于白白丢掉 5~8 dB 音量，展台上的小喇叭会明显变轻。真正卡住
    上限的从来不是响度而是真峰值：某首的 TP 越高，它能被抬/必须被压的余地就越小。
    每首允许的最大增益是 tp_limit - TP_i，对应的最响可达响度是 I_i + 那个增益，
    整组取最小值就是大家都到得了的最响公共目标。
    """
    best = None
    who = None
    for i, m in enumerate(measurements):
        if not m:
            continue
        headroom = tp_limit - float(m["input_tp"])
        reach = float(m["input_i"]) + headroom
        if best is None or reach < best:
            best = reach
            who = i
    if best is None:
        return None, None
    return round(best, 2), who


def decode_to_raw_pcm(input_path, measured=None, target_i=LOUDNORM_I):
    """用 ffmpeg 把输入音频解码为 48k/立体声/s16le 原始 PCM 字节，返回 bytes。

    measured 为 None（默认）时不加任何滤镜，输出与历来一致；给了 measure_loudness
    的结果就串上第二遍 linear loudnorm。
    """
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"找不到输入音频: {input_path}")

    cmd = [
        "ffmpeg", "-v", "error",
        "-i", input_path,
    ]
    if measured is not None:
        # linear=true 需要全套 measured_* 才有意义，缺一个 ffmpeg 就退回动态模式。
        af = ("loudnorm=I=%s:TP=%s:LRA=%s"
              ":measured_I=%s:measured_TP=%s:measured_LRA=%s"
              ":measured_thresh=%s:offset=%s:linear=true:print_format=summary"
              % (target_i, LOUDNORM_TP, LOUDNORM_LRA,
                 measured["input_i"], measured["input_tp"], measured["input_lra"],
                 measured["input_thresh"], measured["target_offset"]))
        cmd += ["-af", af]
    cmd += [
        "-f", "s16le",          # 原始 PCM，无容器
        "-acodec", "pcm_s16le", # 16-bit 小端
        "-ar", str(SAMPLE_RATE),
        "-ac", str(CHANNELS),
        "-",                    # 输出到 stdout
    ]
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        raise RuntimeError("未找到 ffmpeg，请先安装并加入 PATH（本机应已装 ffmpeg 8.1.1）。")

    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg 解码失败:\n{proc.stderr.decode('utf-8', 'replace')}")
    if len(proc.stdout) == 0:
        raise RuntimeError("ffmpeg 返回空 PCM，输入文件可能损坏或无音轨。")
    return proc.stdout


def build_canonical_wav(pcm_bytes):
    """给原始 PCM 拼一个恰好 44 字节的 canonical WAV 头，返回完整 WAV bytes。"""
    data_size = len(pcm_bytes)
    byte_rate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE // 8)
    block_align = CHANNELS * (BITS_PER_SAMPLE // 8)

    header = b"RIFF"
    header += struct.pack("<I", 36 + data_size)  # ChunkSize = 36 + data
    header += b"WAVE"
    header += b"fmt "
    header += struct.pack("<I", 16)              # Subchunk1Size (PCM = 16)
    header += struct.pack("<H", 1)               # AudioFormat = 1 (PCM)
    header += struct.pack("<H", CHANNELS)
    header += struct.pack("<I", SAMPLE_RATE)
    header += struct.pack("<I", byte_rate)
    header += struct.pack("<H", block_align)
    header += struct.pack("<H", BITS_PER_SAMPLE)
    header += b"data"
    header += struct.pack("<I", data_size)       # Subchunk2Size

    assert len(header) == WAV_HEADER_LEN, f"WAV 头必须恰好 {WAV_HEADER_LEN} 字节，实际 {len(header)}"
    return header + pcm_bytes


def convert_mp3_to_wav(input_path, output_path, measured=None, target_i=LOUDNORM_I):
    """解码 + 拼头 + 写盘。返回 (pcm_len, total_len, seconds)。供 sync_to_sd.py 复用。

    measured 非 None 时在解码阶段串上第二遍 linear loudnorm。它只改样本幅度，
    不改采样率/声道/位深，也不改头，所以 RTL 的输出契约不受影响。
    target_i 必须与产生 measured 的那一遍一致。
    """
    pcm = decode_to_raw_pcm(input_path, measured, target_i)
    # 4 字节（一个立体声帧）对齐，避免末尾半帧让 FPGA 组帧错位
    frame_bytes = CHANNELS * (BITS_PER_SAMPLE // 8)
    usable = (len(pcm) // frame_bytes) * frame_bytes
    if usable != len(pcm):
        pcm = pcm[:usable]
    wav = build_canonical_wav(pcm)

    out_dir = os.path.dirname(os.path.abspath(output_path))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)
    with open(output_path, "wb") as f:
        f.write(wav)

    byte_rate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE // 8)
    seconds = usable / float(byte_rate)
    return len(pcm), len(wav), seconds


def main():
    ap = argparse.ArgumentParser(description="MP3/音频 -> 本工程 FPGA 可流式播放的标准 WAV(48k/立体声/16bit)")
    ap.add_argument("input", help="输入音频文件（如 MP3）")
    ap.add_argument("output", nargs="?", default="MUSIC.WAV",
                    help="输出 WAV 路径或盘符（默认当前目录 MUSIC.WAV；给盘符如 F: 则写 F:\\MUSIC.WAV）")
    ap.add_argument("--loudnorm", action="store_true",
                    help=f"两遍 EBU R128 响度归一到 TP<={LOUDNORM_TP} dBTP。"
                         f"多轨联动时把几首来源不同的曲子拉到同一响度用；单轨不需要")
    ap.add_argument("--target-i", type=float, default=LOUDNORM_I, metavar="LUFS",
                    help=f"--loudnorm 的目标综合响度，默认 {LOUDNORM_I}。"
                         f"商业母带常在 -8~-11，拉到 -16 会明显变轻；"
                         f"能拉到多响由真峰值上限决定，脚本会报出这首的天花板")
    args = ap.parse_args()

    out = args.output
    if len(out) == 2 and out[1] == ":":  # 形如 F: 的盘符
        out = os.path.join(out + os.sep, "MUSIC.WAV")

    print(f"输入音频 : {args.input}")
    print(f"输出 WAV : {out}")

    measured = None
    if args.loudnorm:
        print("响度测量 : 第一遍（只测量，不改音频）...")
        measured = measure_loudness(args.input, args.target_i)
        print(f"  输入   : I={measured['input_i']} LUFS  TP={measured['input_tp']} dBTP  "
              f"LRA={measured['input_lra']} LU  thresh={measured['input_thresh']} LUFS")
        print(f"  目标   : I={args.target_i} LUFS  TP={LOUDNORM_TP} dBTP  LRA={LOUDNORM_LRA} LU  "
              f"offset={measured['target_offset']} dB")
        ceiling, _ = loudest_common_target([measured])
        if ceiling is not None and args.target_i > ceiling:
            print(f"  [注意] 这首靠固定增益最响只能到 {ceiling} LUFS（真峰值卡在 "
                  f"{LOUDNORM_TP} dBTP）。目标 {args.target_i} 比它响，ffmpeg 会退回"
                  f"动态压缩，听感发平；要么把 --target-i 降到 {ceiling} 以下。")

    pcm_len, total_len, seconds = convert_mp3_to_wav(args.input, out, measured, args.target_i)
    mm, ss = divmod(int(seconds), 60)
    print("-" * 46)
    print(f"格式     : {SAMPLE_RATE} Hz / {CHANNELS} ch / {BITS_PER_SAMPLE}-bit PCM (canonical 44B 头)")
    print(f"时长     : {mm:02d}:{ss:02d}  ({seconds:.2f} s)")
    print(f"PCM 字节 : {pcm_len}")
    print(f"文件字节 : {total_len}  (= 44 + {pcm_len})")
    print(f"FPGA 校验: PCM 长度应为 文件字节-44 = {total_len - WAV_HEADER_LEN}")
    if measured is not None:
        # 复测写出去的 WAV，看落点。这里不报 linear/dynamic：measurement 那一遍
        # 报的是「它打算怎么处理这个文件」，不是「第二遍实际怎么处理的」，拿它当
        # 模式标签是错的。落点本身才是可核验的事实——够近就说明归一起作用了，
        # 差得多通常意味着这首歌真峰值太高，固定增益顶破 TP 上限后 ffmpeg 只能
        # 少加增益，此时另外几首会显得比它响。
        again = measure_loudness(out, args.target_i)
        got_i, got_tp = float(again["input_i"]), float(again["input_tp"])
        print(f"归一后   : I={got_i:.2f} LUFS  TP={got_tp:.2f} dBTP")
        if abs(got_i - args.target_i) > 1.0:
            print(f"           [注意] 与目标 {args.target_i} LUFS 差 {got_i - args.target_i:+.2f} LU，"
                  f"多轨之间仍可能听得出音量差。")
        if got_tp > LOUDNORM_TP + 0.1:
            print(f"           [注意] 真峰值 {got_tp:.2f} dBTP 超过上限 {LOUDNORM_TP}，"
                  f"16-bit 输出有削顶风险。")
    print("完成。可在 PC 播放器直接试听确认；再按 README 用 sync_to_sd.py 写入 TF 卡（WAV 须先写）。")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[错误] {e}", file=sys.stderr)
        sys.exit(1)
