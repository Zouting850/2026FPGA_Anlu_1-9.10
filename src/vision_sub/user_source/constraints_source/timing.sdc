# ============================================================
# timing.sdc —— 视觉处理副板 M1 时序约束
# 顶层：src/vision_sub/user_source/hdl_source/top_vision_m1.v
#
# M1 只有两个时钟，且**不使用任何 PLL**：
#   sys_clk  = 50 MHz  板载晶振（R7），来自片外
#   cam_pclk ≈ 13.5 MHz 摄像头 PIXCLK（2x2 binning 后），来自片外
# 因此不需要 derive_clocks（没有 PLL 输出可推导）。
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
# ------------------------------------------------------------
create_clock -name sys_clk -period 20 -waveform {0 10} [get_ports {sys_clk}]

# ------------------------------------------------------------
# 2. 摄像头像素时钟
#
#    MT9V034 主时钟 27 MHz；开启列 bin2 后 PIXCLK 减半 = 13.5 MHz
#    → 周期 74.07 ns。这里取整到 74 ns 做约束。
#
#    ⚠️ 实测提示：M1 上板后请用示波器/逻辑分析仪量 D14 上的 PCLK。
#       若实测是 27 MHz（未生效 binning）或 6.75 MHz（误设成 bin4），
#       把 -period 改成对应值（37 / 148）再综合；否则约束与实物不符。
# ------------------------------------------------------------
create_clock -name cam_pclk -period 74 -waveform {0 37} [get_ports {cam_pclk}]

# ------------------------------------------------------------
# 3. 跨时钟域
#
#    sys_clk ↔ cam_pclk 之间只有三类路径，全部是**有意异步**的：
#      a) f_done_tog（pclk→sys）：frame_stat 的翻转信号，sys 侧两级同步
#      b) snapshot 多比特快照（pclk→sys）：准静态，整帧稳定，边沿后延时抓取
#      c) por_n（sys→pclk）：复位电平，pclk 侧两级同步
#
#    这些路径由同步器/准静态握手保证正确性，不参与常规时序分析，
#    因此按异步时钟组处理。
#
#    注意：主板工程 timing.sdc 第 6 节明令禁止对**厂商 FIFO IP** 的读写
#    时钟使用 set_clock_groups（会冲掉 IP 自带 .tcl 的格雷码约束）。
#    M1 工程没有任何厂商 FIFO/SDRAM IP，不存在该冲突，故可安全使用。
#    到 M1b 引入 SDRAM 控制器、M2 引入 async_fifo 时，必须改为
#    按 IP 文档用 set_max_delay -datapath_only 逐对放松。
# ------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks {sys_clk}] \
    -group [get_clocks {cam_pclk}]

# ------------------------------------------------------------
# 4. Input/Output delay —— 经评估后有意留空（非遗漏）
#
#    需要板级走线延迟与器件 tAC/tOH 才能填写，本工程三条接口都不具备前提：
#      a) 摄像头 DVP：cam_d/cam_href/cam_vsync 由 MT9V034 在 PIXCLK 下发出，
#         13.5 MHz 周期 74 ns，相对杜邦线走线延迟裕量极大（几十 ns 级）；
#         bin2 的意义正在于此。缺实测线长数据时凭空填数只会制造虚假可信度。
#      b) SCCB：SCL = 50MHz/(2×250) = 100 kHz，半周期 5 μs，完全无时序压力。
#      c) 调试串口：115200 bps，位宽 8.68 μs，同理。
#
#    因此这里保持空白并记录依据。若后续要上更高速率（如 27 MHz 全分辨率
#    PIXCLK、或 SPI >2 MHz），再按实测走线长度补 set_input_delay /
#    set_output_delay。
# ------------------------------------------------------------
