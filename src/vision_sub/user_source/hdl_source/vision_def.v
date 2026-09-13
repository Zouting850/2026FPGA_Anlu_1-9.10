// ============================================================
// vision_def.v
// 视觉处理副板（EG4S20 / HX4S20）—— 全局参数定义
//
// 本文件与引用它的 top_vision_m1.v / mt9v034_cfg.v / dvp_capture.v /
// sccb_master.v / frame_stat.v / dbg_uart.v 同目录，``include "vision_def.v"``
// 即可解析，无需在 TD 工程里额外配置包含路径。
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

`endif
