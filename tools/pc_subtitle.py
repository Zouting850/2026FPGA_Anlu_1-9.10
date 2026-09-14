#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pc_subtitle.py -- PC 端「文字字幕」发送器（Type-C / 板载 CH340 -> FPGA F12）。

这是一个单文件 tkinter + pyserial GUI，把用户在电脑上输入的一句 ASCII 文字，
通过第二路独立 UART 发给 FPGA，让 marquee_overlay 把它当作滚动字幕显示。它与
J1 上的淘晶驰串口屏是两条完全独立的链路：串口屏走 D14/G11，本通道走 F12，互不
共享任何信号。

竞赛合规：PC 只发字符编码（等同一个键盘），不渲染任何字形、不做数据预处理。
字库、缩放、滚动全部在 FPGA 片内完成（见 marquee_ascii_font.vh / marquee_overlay.v）。

协议（与 RTL src/user_source/hdl_source/uart_pc_text.v 逐字节对应）：
  9600 8N1，ASCII 负载，连续三个 0xFF 终止一帧（沿用本项目 TJC 惯例）。
  负载永不为 0xFF，所以 0xFF 唯一地表示终止符。关键字大写、大小写敏感。

    帧                      字节                                    作用
    "PCTX 1" + FF FF FF     50 43 54 58 20 31 FF FF FF              进入电脑字幕模式
    "PCTX 0" + FF FF FF     50 43 54 58 20 30 FF FF FF              退出，恢复原中文标语
    "TEXT " + <1..24 字符>   54 45 58 54 20 <ascii...> FF FF FF       写入整句，不改 PCTX

  - "TEXT" 后第一个空格之后所有 0x20..0x7E 字节都收进 char_buf（内部空格保留）。
  - 文字上限 24 字符：marquee_overlay 的 11bit 借位寻址 s=x_pos+marq_pos 在 24 格
    以上会溢出回绕，所以 RTL 在源头 PC_CAP=24 截断，本 GUI 也在输入端就拦到 24。
  - 越界 / 小写关键字 / 非法字节 RTL 静默丢弃；本 GUI 在发送前就拒绝，避免白发。

三个按钮对应三帧：① 初始化·开启电脑字幕 (PCTX 1)、② 发送字幕 (TEXT)、
③ 恢复默认字幕 (PCTX 0)。

离线自检：
    python tools/pc_subtitle.py --selftest
  不需要串口、不需要 pyserial、不需要显示器，断言三帧的精确字节 + 负对照
  （空串 / 超长 / 非 ASCII / 控制字符必须被拒）。把协议编码做成单元测试。
"""

import sys

# ---------------------------------------------------------------------------
# 协议编码：纯 stdlib，不依赖 serial / tkinter，便于 --selftest 单元测试。
# ---------------------------------------------------------------------------

TERM = b"\xff\xff\xff"          # 三字节终止符（TJC 惯例）
MAX_CHARS = 24                  # 与 RTL PC_CAP 一致，勿改大（见模块 docstring）
LO, HI = 0x20, 0x7E             # 合法负载字节区间（可打印 ASCII，含空格）


def _validate_text(s):
    """校验一句字幕文字。合法返回 None，非法返回中文错误说明。"""
    if not isinstance(s, str):
        return "文字必须是字符串"
    if len(s) == 0:
        return "文字为空（至少 1 个字符）"
    if len(s) > MAX_CHARS:
        return "文字超过 %d 个字符（当前 %d）" % (MAX_CHARS, len(s))
    for ch in s:
        b = ord(ch)
        if b < LO or b > HI:
            return "含非 ASCII 可打印字符 %r（仅允许 0x20..0x7E，不支持中文/控制字符）" % ch
    return None


def frame_pctx(enable):
    """构造 PCTX 帧。enable=True -> "PCTX 1"（进入电脑字幕模式），False -> "PCTX 0"。"""
    arg = b"1" if enable else b"0"
    return b"PCTX " + arg + TERM


def frame_text(s):
    """构造 TEXT 帧。非法文字抛 ValueError（GUI 会先调 _validate_text 拦截）。"""
    err = _validate_text(s)
    if err is not None:
        raise ValueError(err)
    return b"TEXT " + s.encode("ascii") + TERM


def hexify(b):
    """把字节串渲染成状态栏用的大写十六进制串，如 '50 43 54 58'。"""
    return " ".join("%02X" % x for x in b)


# ---------------------------------------------------------------------------
# 离线自检：断言精确字节 + 负对照。无串口、无 pyserial、无显示器即可运行。
# ---------------------------------------------------------------------------

def _selftest():
    fails = []

    def check(cond, msg):
        if not cond:
            fails.append(msg)

    # ---- 正对照：三帧的精确字节 ----
    check(frame_pctx(True) == bytes([0x50, 0x43, 0x54, 0x58, 0x20, 0x31,
                                     0xFF, 0xFF, 0xFF]),
          "PCTX 1 字节不符: %s" % hexify(frame_pctx(True)))
    check(frame_pctx(False) == bytes([0x50, 0x43, 0x54, 0x58, 0x20, 0x30,
                                      0xFF, 0xFF, 0xFF]),
          "PCTX 0 字节不符: %s" % hexify(frame_pctx(False)))

    payload = "HELLO FPGA 2026"
    expect = bytes([0x54, 0x45, 0x58, 0x54, 0x20]) + payload.encode("ascii") + TERM
    check(frame_text(payload) == expect,
          "TEXT 字节不符: %s" % hexify(frame_text(payload)))

    # 内部空格必须保留（0x20 是合法负载）
    check(frame_text("A B") == b"TEXT A B" + TERM,
          "TEXT 内部空格未保留: %s" % hexify(frame_text("A B")))

    # 边界长度：1 字符与 24 字符都合法
    check(_validate_text("X") is None, "1 字符被误拒")
    check(_validate_text("X" * MAX_CHARS) is None, "24 字符被误拒")
    check(len(frame_text("X" * MAX_CHARS)) == 5 + MAX_CHARS + 3,
          "24 字符帧长度不符")

    # ---- 负对照：非法输入必须被拒 ----
    check(_validate_text("") is not None, "空串未被拒")
    check(_validate_text("X" * (MAX_CHARS + 1)) is not None, "25 字符未被拒")
    check(_validate_text("中文") is not None, "中文未被拒")
    check(_validate_text("tab\there") is not None, "制表符(0x09)未被拒")
    check(_validate_text("nl\nhere") is not None, "换行(0x0A)未被拒")
    check(_validate_text("del\x7f") is not None, "DEL(0x7F)未被拒")
    for bad in ("", "X" * (MAX_CHARS + 1), "中文", "a\x7f"):
        try:
            frame_text(bad)
            fails.append("frame_text 未对非法输入抛错: %r" % bad)
        except ValueError:
            pass

    # ---- 关键字大小写敏感（RTL 只认大写；小写会被静默丢弃，GUI 不发小写）----
    check(frame_pctx(True)[0:4] == b"PCTX", "PCTX 关键字不是大写")
    check(frame_text("hi")[0:4] == b"TEXT", "TEXT 关键字不是大写")

    if fails:
        print("SELFTEST FAIL (%d):" % len(fails))
        for f in fails:
            print("  -", f)
        return 1
    print("SELFTEST PASS: 3 frames byte-exact + %d negative controls" % 8)
    return 0


# ---------------------------------------------------------------------------
# GUI（仅在非 --selftest 时导入 serial / tkinter）。
# ---------------------------------------------------------------------------

def _run_gui():
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        sys.stderr.write(
            "缺少 pyserial。请先安装：\n"
            "    python -m pip install pyserial\n"
            "（--selftest 不需要 pyserial，可离线运行。）\n")
        return 2

    import tkinter as tk
    from tkinter import ttk, messagebox

    BAUDS = ["9600", "19200", "38400", "57600", "115200"]

    class App:
        def __init__(self, root):
            self.root = root
            self.ser = None
            root.title("FPGA 文字字幕发送器 (Type-C / F12, 9600 8N1)")
            root.resizable(False, False)

            # ---- 串口连接区 ----
            conn = ttk.LabelFrame(root, text="① 串口连接")
            conn.grid(row=0, column=0, padx=10, pady=8, sticky="we")

            ttk.Label(conn, text="端口").grid(row=0, column=0, padx=6, pady=6)
            self.port_var = tk.StringVar()
            self.port_cb = ttk.Combobox(conn, textvariable=self.port_var,
                                        width=28, state="readonly")
            self.port_cb.grid(row=0, column=1, padx=4, pady=6)
            ttk.Button(conn, text="刷新", width=6,
                       command=self.refresh_ports).grid(row=0, column=2, padx=4)

            ttk.Label(conn, text="波特率").grid(row=0, column=3, padx=6, pady=6)
            self.baud_var = tk.StringVar(value="9600")
            ttk.Combobox(conn, textvariable=self.baud_var, width=8,
                         values=BAUDS, state="readonly").grid(row=0, column=4, padx=4)

            self.conn_btn = ttk.Button(conn, text="连接", width=8,
                                       command=self.toggle_conn)
            self.conn_btn.grid(row=0, column=5, padx=8)

            # ---- 文字输入区 ----
            txt = ttk.LabelFrame(root, text="② 字幕文字（ASCII 英文/数字/标点，最多 %d 字符）" % MAX_CHARS)
            txt.grid(row=1, column=0, padx=10, pady=8, sticky="we")

            self.text_var = tk.StringVar()
            # 实时拦截：只保留合法可打印 ASCII，并截断到 MAX_CHARS。
            vcmd = (root.register(self._on_validate), "%P")
            self.entry = ttk.Entry(txt, textvariable=self.text_var, width=40,
                                   validate="key", validatecommand=vcmd)
            self.entry.grid(row=0, column=0, padx=6, pady=8, sticky="we")
            self.count_lbl = ttk.Label(txt, text="0/%d" % MAX_CHARS, width=8)
            self.count_lbl.grid(row=0, column=1, padx=6)
            self.text_var.set("HELLO FPGA 2026")

            # ---- 命令按钮区 ----
            btns = ttk.LabelFrame(root, text="③ 发送命令")
            btns.grid(row=2, column=0, padx=10, pady=8, sticky="we")

            ttk.Button(btns, text="初始化·开启电脑字幕\n(PCTX 1)",
                       command=lambda: self.send(frame_pctx(True),
                                                 "初始化·开启电脑字幕")).grid(
                           row=0, column=0, padx=6, pady=6)
            ttk.Button(btns, text="发送字幕\n(TEXT <文字>)",
                       command=self.send_text).grid(row=0, column=1, padx=6, pady=6)
            ttk.Button(btns, text="恢复默认字幕\n(PCTX 0)",
                       command=lambda: self.send(frame_pctx(False),
                                                 "恢复默认字幕")).grid(
                           row=0, column=2, padx=6, pady=6)

            # ---- 状态栏 ----
            self.status = tk.StringVar(value="未连接。先选端口并点「连接」。")
            ttk.Label(root, textvariable=self.status, anchor="w",
                      relief="sunken").grid(row=3, column=0, padx=10, pady=(0, 10),
                                            sticky="we")

            self.refresh_ports()

        # ---- 输入实时校验：滤除非 ASCII 可打印字符并截断 ----
        def _on_validate(self, proposed):
            cleaned = "".join(ch for ch in proposed if LO <= ord(ch) <= HI)
            if len(cleaned) > MAX_CHARS:
                cleaned = cleaned[:MAX_CHARS]
            if cleaned != proposed:
                # 在下一拍改写，避免在 validate 回调里直接 set 造成递归
                self.root.after(0, lambda: self.text_var.set(cleaned))
            self.count_lbl.config(text="%d/%d" % (len(cleaned), MAX_CHARS))
            # 永远返回 True：我们自己清洗，不阻止按键
            return True

        def refresh_ports(self):
            ports = list(list_ports.comports())
            items = ["%s - %s" % (p.device, p.description) for p in ports]
            self.port_cb["values"] = items
            self._port_devices = [p.device for p in ports]
            if items and not self.port_var.get():
                self.port_cb.current(0)
            if not items:
                self.status.set("未发现串口。确认 Type-C 已插、CH340 驱动已装，再点「刷新」。")

        def _selected_device(self):
            i = self.port_cb.current()
            if i is not None and 0 <= i < len(self._port_devices):
                return self._port_devices[i]
            return None

        def toggle_conn(self):
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.conn_btn.config(text="连接")
                self.status.set("已断开。")
                return
            dev = self._selected_device()
            if not dev:
                messagebox.showwarning("未选择端口", "请先在列表里选择一个串口。")
                return
            try:
                self.ser = serial.Serial(port=dev, baudrate=int(self.baud_var.get()),
                                         bytesize=serial.EIGHTBITS,
                                         parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_ONE,
                                         timeout=0.2, write_timeout=1.0)
            except Exception as e:
                messagebox.showerror("打开串口失败", "%s\n\n%s" % (dev, e))
                self.ser = None
                return
            self.conn_btn.config(text="断开")
            self.status.set("已连接 %s @ %s 8N1。可以发送命令。"
                            % (dev, self.baud_var.get()))

        def _require_conn(self):
            if self.ser is None:
                messagebox.showwarning("未连接", "请先选择端口并点「连接」。")
                return False
            return True

        def send(self, payload, label):
            if not self._require_conn():
                return
            try:
                self.ser.reset_output_buffer()
                self.ser.write(payload)
                self.ser.flush()
            except Exception as e:
                messagebox.showerror("发送失败", str(e))
                self.status.set("发送失败：%s" % e)
                return
            self.status.set("%s -> 已写出 %d 字节: %s"
                            % (label, len(payload), hexify(payload)))

        def send_text(self):
            s = self.text_var.get()
            err = _validate_text(s)
            if err is not None:
                messagebox.showwarning("文字非法", err)
                self.status.set("文字非法：%s" % err)
                return
            self.send(frame_text(s), "发送字幕 %r" % s)

        def on_close(self):
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.root.destroy()

    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
    return 0


def main(argv):
    if "--selftest" in argv:
        return _selftest()
    return _run_gui()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
