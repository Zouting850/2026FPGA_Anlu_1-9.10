// ============================================================
// vision_def.v
// 视觉处理副板（EG4S20 / HX4S20）—— 全局参数定义
//
// 本文件与引用它的全部 RTL（top_vision_m1/m2/m3/m4/m5.v、mt9v034_cfg.v、
// sccb_master.v、dvp_capture.v、frame_stat.v、dbg_uart.v、bg_store.v、
// bg_model.v、m2_engine.v、fg_stat.v、morph.v、projection.v、
// people_seg.v、exp_meter.v、auto_exp.v、behavior_fsm.v）同目录，
// `include "vision_def.v" 即可解析，无需在 TD 工程里额外配置包含路径。
//
// 参数按里程碑分组：M1（采集/配置/串口）、M2（背景建模）、M3（形态学 +
// 投影 + 人数）、M4（自动曝光闭环）、M5（行为状态机 + 置信度）。同一参数
// 只在一处定义，RTL 与 tools/ 下的 Python 验证模型都从本文件解析，
// 避免两边各写一份常数。
// ============================================================

`ifndef VISION_DEF_V
`define VISION_DEF_V

// ------------------------------------------------------------
// 摄像头输出几何（2x2 binning 后）
//   R0x03 = 480 行，行 bin2 → 输出 240 行
//   R0x04 = 752 列，列 bin2 → 输出 376 列
//   一帧有效像素 = 376 x 240 = 90240
// ------------------------------------------------------------
`define CAM_IMG_W        376
`define CAM_IMG_H        240
`define CAM_FRAME_PIX    (376*240)

// ------------------------------------------------------------
// SCCB（MT9V034 两线串行接口）
//   Table 6：{S_CTRL_ADR1, S_CTRL_ADR0} = 00 时
//            写地址 0x90、读地址 0x91（地址字节已含 R/W 位，不再左移）
//   分频：SCL = 50MHz / (2 x SCCB_DIV_NUM)
//         250 → 100 kHz（半周期 5us），手册要求的最保守档
// ------------------------------------------------------------
`define SCCB_DEV_ADDR    8'h90
`define SCCB_DIV_NUM     16'd250

// 软复位后等待时间（sys 时钟数）：1ms @50MHz
`define RESET_WAIT_CNT   32'd50000
// 两次寄存器写之间的间隔：100us @50MHz
`define WRITE_GAP_CNT    32'd5000
// 配置完成前的稳定等待：20ms @50MHz
`define SETTLE_WAIT_CNT  32'd1000000

// ------------------------------------------------------------
// 调试串口：50MHz / 115200 = 434
//   板载 CH340：FPGA 的 uart_tx → D12，uart_rx ← F12
// ------------------------------------------------------------
`define UART_BAUD_DIV    16'd434

// ------------------------------------------------------------
// 自动上报周期：1s @50MHz
// ------------------------------------------------------------
`define REPORT_PERIOD    26'd50000000

// ------------------------------------------------------------
// MT9V034 寄存器配置值
//   来源：MT9V034 Register Reference (RR-Rev.A) Table 1/2
//   R0x07 数据格式 0000 dddd dddd dddd，复位默认 0x0388
//        bits[2:0] Scan Mode        = 0 逐行(progressive)
//        bits[4:3] Sensor Op Mode   = 1 Master（传感器自生成时序）
//        bit[6:5]  立体/双目        = 0 关闭
//   R0x0D 数据格式 0000 0011 dddd dddd —— **bits[9:8] 恒为 1，不可清零**
//        bit0 Row Bin2 / bit2 Col Bin2
//   R0x0C bits[1:0]：bit0 复位数字逻辑、bit1 复位 AEC/AGC，均自清零
//   R0xAF bits[1:0]：bit0 AEC 使能、bit1 AGC 使能
// ------------------------------------------------------------
`define MT_VER_EXPECT     16'h1324   // R0x00 芯片版本（只读，恒定）
`define MT_REG_RESET      16'h000C
`define MT_VAL_RESET      16'h0001   // 复位数字逻辑
`define MT_REG_CHIPCTRL   16'h0007
`define MT_VAL_CHIPCTRL   16'h0388   // 逐行 + Master + 单目（= 复位默认，显式回写）
`define MT_REG_COLSTART   16'h0001
`define MT_VAL_COLSTART   16'h0001
`define MT_REG_ROWSTART   16'h0002
`define MT_VAL_ROWSTART   16'h0004
`define MT_REG_WINHEIGHT  16'h0003
`define MT_VAL_WINHEIGHT  16'h01E0   // 480（bin 前），bin2 后输出 240 行
`define MT_REG_WINWIDTH   16'h0004
`define MT_VAL_WINWIDTH   16'h02F0   // 752（bin 前），col bin2 后输出 376 列
`define MT_REG_READMODE   16'h000D
`define MT_VAL_READMODE   16'h0305   // bit9:8 固定 1 + RowBin2 + ColBin2
`define MT_REG_AECAGC     16'h00AF
`define MT_VAL_AECAGC     16'h0003   // AEC+AGC 使能（M1 阶段先交给传感器自动控制）
`define MT_REG_VERSION    16'h0000

`define CFG_WR_NUM        4'd8       // 配置表写条目数

// ------------------------------------------------------------
// M2 背景建模 + 帧差二值化参数
//
//   逐像素、全无符号运算，绝对差取幅度，从根上避开有符号回绕：
//     d    = |cur - bg|                                幅度 0..255
//     fg   = (d > FG_THRESH)                           前景判定位（1 = 前景）
//     want = fg ? 1 : max(1, d >> BG_SHIFT)            本帧想挪几级
//     step = min(want, d)                              ← 钳位，见下
//     bg'  = (cur >= bg) ? bg + step : bg - step
//
//   三个关键点（都有模型在 tools/sim_vision_sub_m2.py 里逐条验证）：
//   1) max(1, ·) 消掉移位积分器的死区：只用 d>>SHIFT 的话，d < 2^SHIFT
//      时 step 恒为 0，背景会永久停在离 cur 十几级的地方。
//   2) 但 max(1,·) 必须再钳到 d。否则 d=0 时 step=1，bg 冲到 cur±1；
//      cur=bg=255 时 bg' = 256 回绕成 0，下一帧 d=255，凭空满屏假前景。
//      钳位后 d=0 不动、d>=1 照常 1 级/帧收敛，且 bg' 恒落在 [bg, cur]。
//   3) 背景像素 d <= FG_THRESH。当前 FG_THRESH(24) <= 2*2^BG_SHIFT(32)，
//      于是 d>>SHIFT 只可能是 0/1，step 恒为 1 —— BG_SHIFT 的指数项暂时
//      不起作用，只有把 FG_THRESH 提到 2^BG_SHIFT 以上才会显现。
//
//   前景像素每 FG_DIV 帧才挪 1 级（≈7.5 级/秒 @60fps）：
//     - 站立的人不会被立刻吸收：对比度 C 的前景要 (C-FG_THRESH)*FG_DIV 帧
//       才融合，C=100 约 10s、C=150 约 17s（远大于 SINGLE 判据的稳定 2s）；
//     - 人离开后的残影也在同一量级上消退。
//   这两个时间常数是算法鲁棒性与响应速度的取舍，见 doc/vision_sub/README.md。
// ------------------------------------------------------------
`define FG_THRESH        8'd24      // 帧差阈值（灰度级）
`define BG_SHIFT         3'd4       // 背景收敛移位（当前 FG_THRESH<=2*2^SHIFT，实际步长恒为 1）
`define FG_DIV           8'd8       // 前景像素每 8 帧走 1 级 ≈ 7.5 级/秒 @60fps
`define FG_CNT_W         17         // 前景计数位宽（90240 < 2^17）
`define M2_LINE_LEN      6'd53      // M2 状态行长度

// 上电后先花这么些帧把背景直接灌成当前画面。
// 必要性：BRAM 上电内容未定义，若拿它当背景，开头会满屏假前景，
// 而前景像素每 FG_DIV 帧才走 1 级，靠算法收敛要好几十秒。
`define BG_LOAD_FRAMES   8'd16

// ------------------------------------------------------------
// M3 形态学 + 行列投影 + 人数分段参数
//
// 【形态学】3x3 开运算 = 先腐蚀后膨胀，流式实现。
//   每个 3x3 级用两条行缓冲（W bit 移位寄存器，右边进左边出）：
//   移位寄存器左端移出的那一 bit，恰好是"上一行同一列"——因为每行
//   正好 W 个像素、正好移 W 次，行列对齐天然保持。再加每行 3 bit 的
//   列抽头，就凑齐 3x3 窗口。
//   开运算吃掉的孤立噪点（1~2 像素）远小于人形宽度（数十像素），
//   所以既能压投影噪声，又不会削掉人。
//   两级级联后有效区域内缩：列 2..373、行 2..237（边缘 2 像素丢弃）。
//
// 【投影】对形态学输出做直方图。为省资源，直接按 4 列 / 4 行一箱：
//     列直方图 PROJ_COL_BINS = 94 箱（每箱 4 列），位宽 10
//       （上限 4 列 x 240 行 = 960 < 1024，不会溢出）
//     行直方图 PROJ_ROW_BINS = 60 箱（每箱 4 行），位宽 11
//       （上限 4 行 x 376 列 = 1504 < 2048）
//   用寄存器阵列而不是 BRAM：索引读 + 索引写在寄存器上可以同拍完成
//   （写译码器 + 读多路器彼此独立），完全绕开 BRAM 单口"读改写要分两拍、
//   读出与累加抢端口"的麻烦。开销约 1600 FF + 900 LUT，完全可接受。
//   分箱带来的唯一损失是分段分辨率降到 4 列——对人形（约 60 列宽）
//   完全够用，而且先合并相邻列再判阈值，抗单列噪点更好。
//
// 【帧切换】帧头（VSYNC 上升沿）读出上一帧直方图并顺手清零，
//   94 + 60 = 154 拍 ≈ 11 us，读出期间关掉累加。读出的同时把
//   "曲线字符"打包进移位寄存器，供串口逐字符吐出。
//
// 【人数分段】列直方图按 SEG_COL_THRESH 二值化 → 行程分段：
//   宽度 < SEG_MIN_WIDTH 的段丢弃（孤立噪点）；
//   间隔 < SEG_MIN_GAP 的两段合并（一人轮廓内的凹陷）。
//   段数即人数估计；±1 死区与滞回留给 M5 行为状态机。
//
// 【投影曲线】曲线值 = min(15, 箱值 >> CURVE_SHIFT)。
//   箱值上限 960，>> 6 后恰好落在 0..15，一个 hex 字符就能覆盖整条量程；
//   串口打出 CURVE_N = 94 个字符，就是一条可与目视直接对照的 ASCII
//   投影曲线——这是 M3 验收项"投影曲线正确"的板级证据。
// ------------------------------------------------------------
`define PROJ_BIN_SHIFT    2         // 每箱 2^2 = 4 列 / 4 行
`define PROJ_COL_BINS     7'd94      // 列直方图箱数（376 / 4）
`define PROJ_ROW_BINS     6'd60      // 行直方图箱数（240 / 4）
`define PROJ_CW           10         // 列直方图位宽
`define PROJ_RW           11         // 行直方图位宽

`define SEG_COL_THRESH    11'd32     // 列箱"占用"阈值（约 8 个前景像素 / 列）
`define SEG_ROW_THRESH    11'd32     // 行箱"占用"阈值（约 8 个前景像素 / 行）
`define SEG_MIN_WIDTH     4'd2       // 合法段最小宽度（箱）——比这窄的当噪点丢
`define SEG_MIN_GAP       4'd2       // 分段最小间隔（箱）——比这窄的凹陷合上

`define CURVE_SHIFT       4'd6       // 曲线值 = 箱值 >> 6，饱和到 15
`define CURVE_N           7'd94      // 曲线字符数 = 列箱数
`define M3_LINE_LEN       8'd141     // M3 状态行长度

// ------------------------------------------------------------
// M4 自动曝光闭环参数
//
// 【为什么需要】M1~M3 一直把曝光交给传感器片上 AEC/AGC（R0xAF=0x0003）。
//   片上 AEC 的目标是"看着好看"，会随场景来回调整，对背景建模是灾难：
//   曝光一变，整帧灰度集体平移，帧差满屏假前景，背景模型被反复冲垮。
//   M4 把 R0xAF 写 0 关掉片上 AEC，改由 FPGA 按固定目标亮度闭环——
//   亮度稳定了，M2 的背景才收敛得下来。
//
// 【寄存器语义】（MT9V034 数据手册 Table 7，已被 web 核准）
//   R0x0B = Coarse Shutter Width Total（粗快门总量，Context A；B 组为 0xD2）
//   R0x35 = Analog Gain Control      （模拟增益，  Context A；B 组为 0x36）
//   R0xAF = AEC/AGC Enable           （bit0 AEC / bit1 AGC）
//   手册原文：多数寄存器在**帧起始**整体生效；但快门（shutter width）与
//   V1~V4 是"下次曝光生效、n+2 帧图像才体现"。所以闭环必须：
//     - 只在帧起始（垂直消隐期）下发；
//     - 下发后至少等 2 帧再采样判断，否则拿旧亮度去算新误差 → 自激振荡。
//   本文件的 EXP_UPDATE_DIV 就是为此：两次回写之间至少隔这么多帧。
//
// 【控制律】比例步长（1 个乘法器，二次收敛，可证不过冲）
//     err  = sum - EXP_TARGET_SUM            （>0 偏亮，<0 偏暗）
//     Δshut = shut * |err| / EXP_TARGET_SUM  （按当前快门等比例缩放）
//   设线性传感器 sum = k*shut、目标 T，令 e = sum - T，可推得调整后
//     e' = -e² / T
//   即误差按平方收缩（|e|<T 时恒有 |e'|<|e|），所以**永不过冲反向放大**，
//   2~3 次回写就进死区。这正是用"乘一个比例"而不是"固定步长"的原因：
//   固定步长在接近目标时一定会在死区两侧来回跳（振荡）。
//   Δshut 再钳到 [EXP_STEP_MIN, EXP_STEP_MAX] 防止单次跳太狠。
//
// 【为什么用移位代替除法】除数 EXP_TARGET_SUM = 8663040 ≈ 2^23 = 8388608
//   （差 3.2%）。写成 `(shut*|err|) >> EXP_PSHIFT` 就是"乘一个常数再右移"，
//   只要 1 个乘法器、不要除法器——TD 对"除以常数"虽然也能综合，但会退化成
//   乘倒数再移位，不如直接写移位来得干净、可读。3.2% 的比例偏差只是把
//   收敛曲线稍微压扁一点，仍满足 |e'|<|e|，且最终被死区吸收。
//
// 【快门优先、增益兜底】偏亮先减快门、快门到底了再减增益；偏暗先加快门、
//   快门到顶了再加增益。快门（积分时间）决定信噪比且不引入额外噪声，
//   增益只是把模拟信号放大、同时放大噪声，所以永远最后动它。
//
// 【为什么不除 cnt 直接比 sum】mean = sum / cnt。cnt 正常情况下恒为
//   90240（M1 就在查这个），所以直接把 sum 和常量 EXP_TARGET_SUM =
//   TARGET_MEAN x 90240 比，就等价于比均值，却省掉一个除法器。
//   闭环还额外加一个"cnt 不对就不调"的门限：画面几何坏掉时不要乱动曝光。
//
// 【EXP_TARGET_SUM 为什么写成表达式】让它随 CAM_FRAME_PIX 自动联动，
//   改几何时不用记着同步改这里。
// ------------------------------------------------------------
`define MT_REG_SHUTTER    16'h000B   // Coarse Shutter Width Total
`define MT_REG_GAIN       16'h0035   // Analog Gain Control
`define MT_VAL_AECAGC_OFF 16'h0000   // R0xAF=0：关片上 AEC/AGC，交给 FPGA 闭环

`define EXP_TARGET_MEAN   8'd96                  // 目标帧平均灰度（0..255）
`define EXP_TARGET_SUM    (96 * (376*240))       // = 96 x 90240 = 8663040
`define EXP_DEADBAND_SUM  32'd262144             // 死区 ±262144 ≈ ±2.9 个灰度级
`define EXP_PSHIFT        5'd23                  // Δshut = (shut*|err|) >> 23（见下）
`define EXP_STEP_MIN      16'd1                  // 单次最小调整量（保证能爬到位）
`define EXP_STEP_MAX      16'd32                 // 单次最大调整量（限幅防跳变）
`define EXP_SHUT_MIN      16'd8                  // 快门下限
`define EXP_SHUT_MAX      16'd1023               // 快门上限
`define EXP_SHUT_INIT     16'd256                // 快门初值（上电第一次就用它）
`define EXP_GAIN_MIN      16'd8                  // 增益下限
`define EXP_GAIN_MAX      16'd64                 // 增益上限
`define EXP_GAIN_INIT     16'd16                 // 增益初值
`define EXP_GAIN_STEP     16'd1                  // 增益每次调整量（兜底，慢一点）
`define EXP_UPDATE_DIV    4'd2                   // 两次回写至少隔 2 帧（div_cnt 只在帧头计数）
                                                  // 写在第 N 帧头 → 影响第 N+2 帧图像 →
                                                  // 第 N+3 帧头才能量到，DIV=2 恰好只用量化后的数据
`define EXP_LOCK_FRAMES   4'd8                   // 连续 N 帧在死区内 → 判锁定
`define EXP_SAT_LEVEL     8'd250                 // 饱和像素判定阈值
`define EXP_SAT_MAX       24'd4096               // 饱和像素数超过此值 → 强制判"偏亮"
`define M4_LINE_LEN       8'd164                 // M4 状态行长度

// ------------------------------------------------------------
// M5 行为状态机 + 置信度参数
//
// 【职责】M3 只回答"这一帧看起来有几个人"（瞬时值，无时间维度）。
//   M5 补上时间维度：够不够稳、待了多久、是不是在互动、要不要报离开。
//   输出"行为状态 + 建议播放模式 + 置信度"，**最终裁决权仍在主板**
//   （副板不知道媒体清单/播放进度/内容分级，只能给建议）。
//
// 【状态机】（方案 v2.0 §2.2）
//   IDLE ──人数≥1 连续 EVT_CONFIRM 帧──→ PRESENCE
//   PRESENCE ──停止抖动且稳定 STABLE_FRAMES 帧──→ SINGLE
//   PRESENCE ──人数≥2 连续 EVT_CONFIRM 帧──→ MULTI
//   SINGLE/MULTI ──能量突增 且（横向位移 or 大幅突增）──→ INTERACT
//   INTERACT ──能量回落 或 冷却 COOL_FRAMES 到期──→ 回 SINGLE/MULTI
//   SINGLE/MULTI ──占用范围触及画面最外 GUARD_EDGE 列 连续确认──→ ALERT
//   任意(有人态) ──人数 0 持续 LEAVE_FRAMES──→ LEAVE（锁存停留时长）
//   LEAVE ──人数≥1──→ PRESENCE；LEAVE ──无人再持续 IDLE_HOLD_FRAMES──→ IDLE
//   ALERT ──离开警戒区 连续确认 且 人数≥1──→ 回 SINGLE/MULTI
//
// 【防误触发】方案 §2.2 "防误触发"四条，逐条落到常量：
//   1) 人数 ±1 死区：差 1 个不认（PPL_DEADBAND），必须同向连续
//      PPL_CONFIRM 帧才认；差 ≥2 立刻跟（真来了一群人不用等）。
//      实测边界（M5 模型逐条验证过，别把它的作用想大了）：
//        - 短于 PPL_CONFIRM 帧的 ±1 突发（如 2 帧）被完全滤掉，
//          连报告人数 U 都不会动；
//        - 长于等于 PPL_CONFIRM 帧的持续（如 4 帧）**会被当真**并
//          照常跃迁到 MULTI——那确实是"画面里真出现了第二个人"，
//          3 帧 = 50 ms @60fps 就是这条流水线的判定颗粒度；
//        - 因此 PPL_CONFIRM 与 EVT_CONFIRM 取同一数值（3）时，死区
//          在"防状态误跃迁"上与状态确认的作用是重叠的，它真正独立
//          贡献的是：报告人数（U 字段）稳定、以及把未确认的证据挡在
//          状态机计数器之前。
//   2) 所有状态跃迁都要连续 EVT_CONFIRM 帧确认（进入/退出用同一门限，
//      "进入门限 > 退出门限"的滞回体现在：进入要 3 帧，退出走各自的
//      长确认——LEAVE 要 2 s、SINGLE 要 2 s 稳定，都远长于 3 帧）。
//   3) 互动带 COOL_FRAMES 冷却，冷却期内不再触发新的互动事件。
//   4) 质心每帧位移 ≤ CTR_QUIET 箱才算"安静"（PRESENCE→SINGLE 的前提）。
//
// 【能量基线与突增】nrg 取形态学后的前景像素数（m_cnt），比形态学前
//   干净。基线用一阶 EMA：base += (nrg - base) >> NRG_BASE_SHIFT，
//   **只在非 INTERACT 状态更新**——否则挥手把自己的基线抬上去，
//   动作结束后基线迟迟降不回来，下一次挥手就检测不到了。
//   突增判据同时要绝对量和倍数：nrg > base + NRG_INT_DELTA 且
//   nrg > base + (base >> NRG_INT_RATIO)（后者即 1.5 倍）。
//   只要绝对量的话，基线高时小动静也误判；只要倍数的话，基线近 0 时
//   一点点噪声就乘以无穷大。两个都要。
//
//   ⚠️ 两处"基线还没收敛"的坑（建模时抓出，属于本里程碑的真实修正）：
//   1) 上电后 base 初值为 0，若直接 EMA，则观众一出现 nrg 就远大于
//      base+Δ → 被误判成"突增"。故**第一个有效帧直接把 base 灌成 nrg**
//      （base_init 标志），之后才走 EMA——与 M2 上电灌背景是同一思路。
//   2) 即便灌了初值，"无人→有人"或"1 人→2 人"这类人数阶跃本身仍会
//      让 nrg 跳一大截（真人确实带来了更多前景像素），而 EMA 需要约
//      NRG_BASE_SHIFT 倍的时间才跟上。若不设防，进入 MULTI 的瞬间就会
//      误报一次 INTERACT。故引入 warm_cnt：**原始人数一变就清零**，连续
//      NRG_BASE_READY 帧（≈0.5 s）内不判互动，等基线追上当前场景。
//      盯"原始人数"而不是滤波后的人数很关键：阶跃的能量跳变是这一帧
//      就发生的，而滤波后的 pplf 还要 PPL_CONFIRM 帧确认，用 pplf 会让
//      防护晚 3 帧生效，恰好漏掉阶跃那几帧（M5 模型实测抓到的第二个坑）。
//      挥手本身不改人数，所以不会把自己的检测窗口重置掉。
//
// 【方向判别】滑动窗口（DIR_WIN 帧）里记质心列的 min/max，
//   跨度 ≥ DIR_MIN 箱算"有横向位移"，方向取窗口首末差符号。
//   挥手时手臂来回扫，跨度足够；单纯站立抖动跨度很小。
//   方案原文把互动判据写成一个 AND（能量突增 且 方向成立），实测会
//   漏掉"前推"（手掌朝镜头推近：能量突增明显但质心几乎不动）。
//   这里放宽为：能量突增 且（横向位移成立 或 突增幅度 > 2x），
//   方向编码 3 = 前推。上板可按现场动作收紧 NRG_INT_DELTA。
//
// 【置信度】三个 0..255 的评分各自独立积分，合成后除以 3：
//     sc_dwell  : 有人 +SC_UP，无人 -SC_DN      （待得久 → 可信）
//     sc_motion : 能量 ≥ NRG_LO +SC_UP，否则 -SC_DN（真的有东西在）
//     sc_quiet  : 质心安静 +SC_UP，抖动 -SC_DN  （拿得准 → 可信）
//     conf = (sc_dwell + sc_motion + sc_quiet) * 85 >> 8
//   85/256 ≈ 1/3.01，用常数乘法代替除法器；三路和最大 765，
//   乘 85 后 65025 >> 8 = 254，天然不溢出 8 位。三个评分都做饱和，
//   所以 conf 对"稳定"单调、对"抖动/无人"单调下降，主板可以按
//   ≥128 采信、<64 忽略之类的阈值用。
// ------------------------------------------------------------
// 行为状态编码（3 bit）
`define ST_IDLE           3'd0
`define ST_PRESENCE       3'd1
`define ST_SINGLE         3'd2
`define ST_MULTI          3'd3
`define ST_INTERACT       3'd4
`define ST_LEAVE          3'd5
`define ST_ALERT          3'd6

// 建议播放模式编码（3 bit，对齐方案 §2.1；STANDBY 含"无人"与"未确认"两类）
`define MD_STANDBY        3'd0
`define MD_SINGLE         3'd1
`define MD_MULTI          3'd2
`define MD_INTERACT       3'd3
`define MD_ALERT          3'd4

// 防误触发
`define PPL_DEADBAND      4'd1        // 人数差 1 个不认（死区）
`define PPL_CONFIRM       4'd3        // 人数同向变化连续确认帧数
`define EVT_CONFIRM       3'd3        // 状态/警戒区跃迁连续确认帧数
`define STABLE_FRAMES     8'd120      // 2 s @60fps：PRESENCE → SINGLE
`define LEAVE_FRAMES      8'd120      // 2 s：有人态 → LEAVE（人数 0）
`define IDLE_HOLD_FRAMES  10'd600     // 10 s：LEAVE → IDLE
                                      // 注意宽度必须 ≥10：9'd600 装不下 600
                                      // （9 位上限 511），会被静默截成 88 帧≈1.5 s。
                                      // TD 只给 HDL-5007 WARNING，不报错。
`define COOL_FRAMES       8'd90       // 1.5 s：互动冷却

// 质心与方向
`define CTR_QUIET         7'd3        // 质心每帧位移 ≤ 此值（箱）算安静
`define DIR_WIN           5'd16       // 方向判别窗口（帧）
`define DIR_MIN           7'd2        // 窗口内质心跨度 ≥ 此值（箱）算有位移
`define DIR_WIN_MAX       5'd15       // DIR_WIN - 1（窗口计数比较用）

// 能量
`define NRG_BASE_SHIFT    5'd5        // 能量基线 EMA 移位（1/32）
`define NRG_BASE_READY    6'd32       // 人数变化后，基线需连续更新这么多帧才允许判互动
                                      // （否则"来人/变多人"这种能量阶跃会被当成挥手）
`define NRG_INT_DELTA     17'd1500    // 互动判据：相对基线的绝对增量
`define NRG_INT_RATIO     2'd1        // 互动判据：> base + (base>>1) 即 1.5 倍
`define NRG_LO            17'd300     // 有效能量的下限（低于此视为噪声/无人）
`define NRG_HI            17'd30000   // 有效能量的上限（超出视为满屏噪声）

// 警戒区：占用列范围触及画面最外侧 GUARD_EDGE 列 → 建议 ALERT
`define GUARD_EDGE        10'd47      // ≈ 376/8
`define GUARD_R_LEFT      10'd328     // CAM_IMG_W - 1 - GUARD_EDGE = 375 - 47

// 置信度评分步长与合成
`define SC_UP             8'd2
`define SC_DN_SOFT        8'd4
`define SC_DN_HARD        8'd8
`define CONF_MUL          8'd85       // /256 ≈ /3.01
`define CONF_SHIFT        4'd8        // 4 位才装得下 8（3'd8 会截成 0）

// 状态行
`define M5_LINE_LEN       8'd193      // 68(M4 前缀) + 29(M5 新字段) + 94 曲线 + CRLF
`define M5_CURVE_POS      8'd97       // 曲线起点
`define M5_CURVE_END      8'd190      // 曲线终点（含）
`define M5_CR_POS         8'd191
`define M5_LF_POS         8'd192

`endif
