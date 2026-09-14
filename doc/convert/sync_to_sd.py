import os
import sys
import glob
import shutil
import argparse

# 让本脚本无论从哪运行都能 import 同目录的 convert_audio_to_wav
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def corrupt_old_bmp_headers(target_drive):
    """
    遍历目标驱动器下的所有 .bmp 文件，将其开头的 'BM' 魔数修改为 'XX'。
    这样 FPGA 的物理扇区扫描就不会把这些“被标记为删除但物理数据还在”的文件识别为有效图片。
    """
    print(f"正在扫描 {target_drive} 驱动器，准备清除旧图片的 BMP 识别头...")
    
    # 查找根目录下的所有 bmp 文件（你也可以用 os.walk 查找所有子目录）
    search_pattern = os.path.join(target_drive, "*.bmp")
    bmp_files = glob.glob(search_pattern)
    
    corrupt_count = 0
    for bmp_file in bmp_files:
        try:
            # 以读写模式打开文件（不截断）
            with open(bmp_file, 'r+b') as f:
                header = f.read(2)
                if header == b'BM':
                    # 将指针移回开头
                    f.seek(0)
                    # 写入随意字符破坏魔数，例如 'XX'
                    f.write(b'XX')
                    corrupt_count += 1
                    print(f"  [清除有效] 已破坏头文件: {os.path.basename(bmp_file)}")
        except Exception as e:
            print(f"  [警告] 无法处理文件 {bmp_file}: {e}")
            
    print(f"头文件清除完毕，共处理了 {corrupt_count} 个旧图片文件。")

def clean_drive(target_drive):
    """
    删除目标驱动器下的所有文件，模拟格式化/清空操作（不删除隐藏系统文件夹）。
    """
    print(f"\n正在清空 {target_drive} 驱动器中的旧文件...")
    for item in os.listdir(target_drive):
        item_path = os.path.join(target_drive, item)
        # 跳过系统隐藏文件夹如 System Volume Information
        if item.startswith('.'):
            continue
            
        try:
            if os.path.isfile(item_path):
                os.remove(item_path)
            elif os.path.isdir(item_path):
                shutil.rmtree(item_path)
        except Exception as e:
            print(f"  [警告] 无法删除 {item_path}: {e}")
    print("驱动器清空完毕。")

def sync_new_images(source_dir, target_drive, max_count=4):
    """
    从源文件夹中挑选最多 max_count 张 BMP 图片，复制到目标驱动器。
    """
    if not os.path.exists(source_dir):
        print(f"\n[错误] 源文件夹不存在: {source_dir}")
        return

    bmp_files = glob.glob(os.path.join(source_dir, "*.bmp"))
    if not bmp_files:
        print(f"\n[错误] 源文件夹 {source_dir} 中没有找到任何 BMP 图片。")
        return

    # 按名称排序，保证顺序一致性
    bmp_files.sort()
    
    # 限制复制的数量
    files_to_copy = bmp_files[:max_count]
    
    print(f"\n准备将 {len(files_to_copy)} 张图片同步到 {target_drive}...")
    
    success_count = 0
    for i, bmp_file in enumerate(files_to_copy):
        try:
            filename = os.path.basename(bmp_file)
            # 为了让 FPGA 物理扇区尽量连续，我们在文件名前加上序号
            target_name = f"{i:02d}_{filename}"
            target_path = os.path.join(target_drive, target_name)
            
            shutil.copy2(bmp_file, target_path)
            print(f"  [同步成功] {filename} -> {target_name}")
            success_count += 1
        except Exception as e:
            print(f"  [同步失败] {filename}: {e}")
            
    print(f"\n同步完成！成功写入 {success_count} 张新图片。")
    print("你现在可以安全弹出 SD 卡，插入 FPGA 开发板了。")

MAX_TRACKS = 4

def track_name(idx):
    """
    第 idx 首音乐在卡上的文件名，严格 8.3。

    必须是 8.3：长文件名会在根目录里额外插入 attr=0x0F 的 LFN 槽。bmp_read 的
    dir_entry_is_file 已经把 0x0F 过滤掉了，所以 LFN 槽不会被误认成文件，但它
    照样占目录项——4 首音乐若都带 LFN 槽，就会把后面的 BMP 往后推，而扫描找满
    SCAN_TARGET_COUNT 张 BMP 就提前 scan_done，推得太远就等于扫不到。
    MUSIC0.WAV 是 6+3，MUSIC.WAV 是 5+3，都不产生 LFN 槽。
    """
    return "MUSIC%d.WAV" % idx

def sync_audio(audio_path, target_drive, out_name, measured=None, target_i=None):
    """
    把音频（如 MP3）离线转成标准 WAV，写到目标盘根目录 out_name。

    必须在写 BMP 之前调用，原因有两条，缺一不可：
      1) 目录顺序：FAT32 根目录项按创建先后排列。bmp_read 的扫描找满 4 张 BMP
         就提前 scan_done 停止，所以 WAV 的目录项必须排在那 4 张 BMP 之前才会被
         扫到。先写 MUSIC*.WAV -> 它们占用最前面的空闲目录槽。
      2) 物理连续：读卡不跟 FAT32 簇链（地址线性 +1），假设文件物理连续。空卡
         上先写这些大文件，FAT 基本为空 -> 一次性连续分配，规避碎片。
    为最大化第 2 点的可靠性，建议先对卡做一次 FAT32 格式化再运行本工具。

    measured 是 convert_audio_to_wav.measure_loudness 的结果，给了就做第二遍 linear
    loudnorm；不给就完全不动响度，输出与历来一致。
    """
    import convert_audio_to_wav as cw

    if not os.path.isfile(audio_path):
        print(f"\n[错误] 找不到音频文件: {audio_path}")
        return None

    out = os.path.join(target_drive, out_name)
    print(f"\n正在把音频转为标准 WAV 并【先于 BMP】写入...")
    print(f"  输入: {audio_path}")
    print(f"  目标: {out}")
    if measured is not None:
        ti = cw.LOUDNORM_I if target_i is None else target_i
        print(f"  响度: 输入 I={measured['input_i']} LUFS -> 目标 I={ti} LUFS")
    try:
        pcm_len, total_len, seconds = cw.convert_mp3_to_wav(
            audio_path, out, measured,
            cw.LOUDNORM_I if target_i is None else target_i)
    except Exception as e:
        print(f"  [音频失败] {e}")
        return None
    mm, ss = divmod(int(seconds), 60)
    print(f"  [音频成功] 时长 {mm:02d}:{ss:02d}  PCM {pcm_len} 字节  文件 {total_len} 字节")
    print(f"  FPGA 校验: PCM 长度 = 文件字节-44 = {total_len - cw.WAV_HEADER_LEN}")
    return out

def write_existing_wav(wav_path, target_drive, out_name):
    """
    把一个【已经做好的】标准 WAV 复制到目标盘根目录 out_name，同样保证【先于 BMP】写入。

    与 sync_audio 的区别：sync_audio 需要源 MP3 现场转码；本函数直接复用现成 WAV，
    用于卡上已有可用 MUSIC*.WAV、但手头没有源 MP3 的情况。clean_drive 会先删掉卡上的
    WAV，所以必须先把旧 WAV 备份到卡外，再用本函数写回。先写 WAV 的两条理由与
    sync_audio 完全相同：目录项要排在 4 张 BMP 之前才会被 bmp_read 扫到；空卡先写
    这个大文件才能物理连续。
    """
    import struct

    if not os.path.isfile(wav_path):
        print(f"\n[错误] 找不到 WAV 文件: {wav_path}")
        return None

    with open(wav_path, 'rb') as f:
        head = f.read(44)
    if head[0:4] != b'RIFF' or head[8:12] != b'WAVE':
        print(f"\n[错误] {wav_path} 不是合法的 RIFF/WAVE 文件")
        return None

    channels   = struct.unpack('<H', head[22:24])[0]
    samplerate = struct.unpack('<I', head[24:28])[0]
    bits       = struct.unpack('<H', head[34:36])[0]
    pcm_len    = os.path.getsize(wav_path) - 44
    if (channels, samplerate, bits) != (2, 48000, 16):
        print(f"  [警告] 非标准格式 {samplerate}Hz/{channels}ch/{bits}bit；"
              f"FPGA 音频链按 48k/2/16 设计，可能播放异常")

    out = os.path.join(target_drive, out_name)
    print(f"\n正在复用现成 WAV 并【先于 BMP】写入...")
    print(f"  输入: {wav_path}")
    print(f"  目标: {out}")
    shutil.copy2(wav_path, out)
    seconds = pcm_len / (samplerate * channels * (bits // 8))
    mm, ss = divmod(int(seconds), 60)
    print(f"  [音频成功] {samplerate}Hz/{channels}ch/{bits}bit  时长 {mm:02d}:{ss:02d}  PCM {pcm_len} 字节")
    return out

def sync_audio_set(paths, target_drive, transcode, loudnorm=False, target_i=None):
    """
    按给定顺序写入整组音乐，第 i 首 -> MUSICi.WAV，全部【先于任何 BMP】。

    写入顺序就是目录槽顺序，也就是 sd_card_bmp 里 wav_sector0..3 的填充顺序，
    所以这里的第 i 首必须就是想配给第 i 张图片的那一首：bmp_read 逐 WAV 上报
    scan_found_wav_valid，sd_card_bmp 按 wav_found_count 依次入槽，手动切图时
    track_req = img_idx，于是「第 i 张图 -> 第 i 首」。

    超过 4 首直接拒绝，而不是截断。RTL 的入槽 case 是 0/1/2/default，
    wav_found_count 在 4 处饱和，所以第 5 首会静默覆盖槽 3 —— 第 4 张图会播第 5
    首，而且没有任何报错。宁可在这里停下。

    loudnorm=True 时先把整组都测一遍响度并报出落差，再逐首归一到同一目标。只在
    转码路径（-a）有效：-w 是逐字节复制现成 WAV，动它就等于重新转码，与「复用已
    验证过的 WAV」这个用途矛盾，所以那条路径下直接说明不做。
    """
    writer = sync_audio if transcode else write_existing_wav
    if len(paths) > MAX_TRACKS:
        print(f"\n[错误] 给了 {len(paths)} 首音乐，但查找表只有 {MAX_TRACKS} 槽。")
        print(f"       RTL 的入槽 case 在 count>=3 时走 default 覆盖槽 3，而")
        print(f"       wav_found_count 在 4 处饱和，第 5 首会静默顶掉第 4 首。")
        print(f"       请只保留前 {MAX_TRACKS} 首后重试。")
        return []

    measured = [None] * len(paths)
    if loudnorm:
        import convert_audio_to_wav as cw
        ti = cw.LOUDNORM_I if target_i is None else target_i
        if not transcode:
            print("\n[提示] --loudnorm 在 -w（复用现成 WAV）下不生效：那条路径是逐字节")
            print("       复制，不改内容。要归一请用 -a 给源文件重新转码。")
        elif len(paths) < 2:
            print("\n[提示] 只有一首，响度落差无从谈起，--loudnorm 跳过。")
        else:
            print(f"\n第一遍：测量整组响度（目标 I={ti} LUFS, TP<={cw.LOUDNORM_TP} dBTP）...")
            for i, p in enumerate(paths):
                try:
                    measured[i] = cw.measure_loudness(p, ti)
                except Exception as e:
                    print(f"  [测量失败] 第 {i} 首 {os.path.basename(p)}: {e}")
                    print(f"           这一首不归一，按原响度写入。")
            got = [(i, float(m["input_i"]), float(m["input_tp"]))
                   for i, m in enumerate(measured) if m]
            if len(got) >= 2:
                louds = [v for _, v, _ in got]
                spread = max(louds) - min(louds)
                for i, v, tp in got:
                    print(f"  第 {i} 首 I={v:7.2f} LUFS  TP={tp:6.2f} dBTP   "
                          f"{os.path.basename(paths[i])}")
                # 3 LU 以内一般人听不出；再大就是展台上「切一首音量跳一下」。
                verdict = "听得出音量跳变" if spread > 3.0 else "落差可接受"
                print(f"  整组落差 {spread:.2f} LU -> {verdict}")

                ceiling, who = cw.loudest_common_target(measured)
                print(f"  真峰值天花板：整组能靠固定增益一起达到的最响目标是 "
                      f"{ceiling} LUFS（被第 {who} 首 "
                      f"{os.path.basename(paths[who])} 卡住）")
                if ti > ceiling:
                    print(f"  [注意] 目标 {ti} LUFS 比天花板响。超出的那几首 ffmpeg 会退回")
                    print(f"         动态压缩（实时拉增益），听感发平、有喘气声，而且它们会")
                    print(f"         比其余几首显得轻——正好破坏归一的目的。")
                    print(f"         建议 --target-i {ceiling}。")
                elif ti < ceiling - 3.0:
                    print(f"  [提示] 目标 {ti} 比天花板低 {ceiling - ti:.2f} dB，等于白扔音量。"
                          f"想更响可以用 --target-i {ceiling}。")

    print(f"\n准备写入 {len(paths)} 首音乐（全部先于 BMP，顺序即图号）...")
    written = []
    for i, p in enumerate(paths):
        if transcode:
            out = writer(p, target_drive, track_name(i), measured[i], target_i)
        else:
            out = writer(p, target_drive, track_name(i))
        if out is None:
            print(f"  [警告] 第 {i} 首写入失败，槽 {i} 会空缺。")
            print(f"         sd_card_bmp 仍按上报顺序入槽，所以后面的音乐会前移，")
            print(f"         图与曲的对应关系会从这一首开始错位。请修复后重跑。")
        else:
            written.append((i, os.path.basename(p), out))

    if written:
        print(f"\n音轨映射（手动切图时 第 i 张图 -> 第 i 首；自动轮播固定第 "
              f"AUTO_TRACK_IDX 首）:")
        for i, src, out in written:
            print(f"  图 {i}  <-  {os.path.basename(out)}   (源: {src})")
    return written

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FPGA SD卡 图片+多轨音乐同步工具 (防物理残留死锁版)")
    parser.add_argument('drive', help='SD 卡所在的盘符 (例如: E: 或 E:\\)')
    parser.add_argument('-s', '--source', help='存放转换好 BMP 图片的源文件夹 (默认: 当前目录的 output_bmp)', default=None)
    parser.add_argument('-n', '--num', type=int, default=4, help='最多同步的图片数量 (默认: 4)')
    parser.add_argument('-a', '--audio', nargs='+',
                        help='要播放的音频文件 (如 MP3)，可给多首，【顺序即图号】。离线转成 MUSIC0.WAV MUSIC1.WAV ... 并【先于所有 BMP】写入卡根目录',
                        default=None)
    parser.add_argument('-w', '--wav', nargs='+',
                        help='复用【现成的】标准 WAV (48k/2/16)，可给多首，【顺序即图号】，直接作为 MUSIC0.WAV MUSIC1.WAV ... 并【先于所有 BMP】写入。与 -a 互斥且优先；用于卡上已有可用 WAV 但无源 MP3 的情况',
                        default=None)
    parser.add_argument('--loudnorm', action='store_true',
                        help='两遍 EBU R128 响度归一：先测出整组落差再逐首拉到同一响度。'
                             '多首来源不同的曲子在展台上背靠背播放时音量会跳，用这个抹平。'
                             '仅对 -a 生效（-w 是逐字节复制现成 WAV，不改内容）',
                        default=False)
    parser.add_argument('--target-i', type=float, default=None, metavar='LUFS',
                        help='配合 --loudnorm：整组归一到的目标综合响度。不给则用 -16 LUFS。'
                             '开了 --loudnorm 后脚本会报出这组的真峰值天花板，'
                             '想更响就把这个值往天花板靠')

    args = parser.parse_args()

    # 处理盘符格式，确保以路径分隔符结尾
    target_drive = args.drive
    if not target_drive.endswith(':\\') and not target_drive.endswith(':/'):
        if target_drive.endswith(':'):
            target_drive += '\\'
        else:
            target_drive += ':\\'

    if not os.path.exists(target_drive):
        print(f"[致命错误] 找不到指定的驱动器: {target_drive}")
        print("请检查 SD 卡是否已正确插入电脑并分配了盘符。")
        sys.exit(1)

    # 确定源文件夹
    current_dir = os.path.dirname(os.path.abspath(__name__))
    source_dir = args.source if args.source else os.path.join(current_dir, "output_bmp")

    print("="*50)
    print(f"目标 SD 卡盘符: {target_drive}")
    print(f"图片源文件夹  : {source_dir}")
    if args.wav:
        print(f"音乐(现成WAV) : {len(args.wav)} 首 -> " + ", ".join(os.path.basename(p) for p in args.wav))
    elif args.audio:
        print(f"音乐(待转码)  : {len(args.audio)} 首 -> " + ", ".join(os.path.basename(p) for p in args.audio))
    print("="*50)

    # 为了防止误操作清空 C 盘，做个简单拦截
    if target_drive.upper().startswith('C:'):
        confirm = input("警告: 你指定了系统盘(C:)！这可能会导致系统崩溃。你确定要继续吗？(y/N): ")
        if confirm.lower() != 'y':
            print("操作已取消。")
            sys.exit(0)

    # 步骤 1: 破坏旧 BMP 的物理文件头
    corrupt_old_bmp_headers(target_drive)

    # 步骤 2: 清空 SD 卡（从文件系统层面）
    clean_drive(target_drive)

    # 步骤 3: 先写全部音乐（必须早于任何 BMP：目录项排序 + 物理连续）
    written = []
    if args.wav:
        written = sync_audio_set(args.wav, target_drive, transcode=False,
                                 loudnorm=args.loudnorm, target_i=args.target_i)
    elif args.audio:
        written = sync_audio_set(args.audio, target_drive, transcode=True,
                                 loudnorm=args.loudnorm, target_i=args.target_i)

    # 步骤 4: 复制新图片
    sync_new_images(source_dir, target_drive, max_count=args.num)

    # 步骤 5: 提示校验。目录槽顺序和物理连续性都看不见摸不着——资源管理器走的是
    # 文件系统驱动，永远不会暴露原始目录槽，也永远不会告诉你某个文件是不是被分成
    # 了几段簇链。而 RTL 的读法恰恰只依赖这两件事，所以写完必须用原始扇区回放
    # 校验一遍，别直接上板。
    if written:
        print("\n" + "="*50)
        print("下一步：先校验，再上板")
        print(f"  python tools/sim_dir_scan.py {args.drive}")
        print("它会按 bmp_read 的判据逐槽回放根目录扫描，报告：")
        print("  - 每首 WAV 的目录槽位置，是否全部排在第 %d 张 BMP 之前" % args.num)
        print("  - 每首 WAV 的簇链是否物理连续（RTL 只做 LBA+1，不跟簇链）")
        print("  - 扫描会在哪一槽 scan_done 停止，停止前记到了几首")
