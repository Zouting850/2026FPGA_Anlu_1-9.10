// ============================================================
// dvp_capture.v
// MT9V034 DVP 并行接口采集时序恢复
//
// MT9V034 的输出时序（Master Mode / simultaneous）：
//   FRAME_VALID（= 本文件 vsync）：一帧内保持高
//   LINE_VALID （= 本文件 href ）：一行有效期内保持高
//   PIXCLK      （= 本文件 pclk ）：DOUT[7:0] 在 PIXCLK 上升沿更新
//
// 本模块整体工作在 **PIXCLK 域**（约 13.5 MHz，2x2 binning 后），
// 不做跨时钟域——CDC 由顶层的 async_fifo / 快照握手负责。
//
// 输出：
//   pix_valid = 寄存器化后的 HREF，与 pix_data 同拍对齐
//   line_start= HREF 上升沿 1 拍
//   frame_start=FRAME_VALID 上升沿 1 拍
//   col_cnt   = 当前行内像素序号（HREF 上升沿清零）
//   row_cnt   = 当前帧内行序号（FRAME_VALID 上升沿清零）
// ============================================================
`include "vision_def.v"

module dvp_capture
(
	input                       pclk,       // CAM_PCLK，约 13.5 MHz
	input                       rst_n,      // 低有效（PCLK 域）
	input                       vsync,      // FRAME_VALID
	input                       href,       // LINE_VALID
	input[7:0]                  din,        // D0..D7
	output reg[7:0]             pix_data,
	output reg                  pix_valid,
	output reg                  line_start,
	output reg                  frame_start,
	output reg                  frame_valid,
	output reg[15:0]            col_cnt,
	output reg[15:0]            row_cnt,
	output reg[15:0]            frame_cnt
);

reg vsync_d;
reg href_d;

wire vsync_rise =  vsync & ~vsync_d;
wire href_rise  =  href  & ~href_d;
wire href_fall  = ~href  &  href_d;

always@(posedge pclk or negedge rst_n)
begin
	if(!rst_n)
	begin
		vsync_d     <= 1'b0;
		href_d      <= 1'b0;
		pix_data    <= 8'd0;
		pix_valid   <= 1'b0;
		line_start  <= 1'b0;
		frame_start <= 1'b0;
		frame_valid <= 1'b0;
		col_cnt     <= 16'd0;
		row_cnt     <= 16'd0;
		frame_cnt   <= 16'd0;
	end
	else
	begin
		vsync_d <= vsync;
		href_d  <= href;

		// 像素数据有效电平：HREF 为高时 D0..D7 有效
		pix_valid   <= href;
		pix_data    <= din;

		// 脉冲输出
		line_start  <= href_rise;
		frame_start <= vsync_rise;
		frame_valid <= vsync;

		// 行/帧计数器
		if(vsync_rise == 1'b1)
		begin
			row_cnt   <= 16'd0;
			frame_cnt <= frame_cnt + 16'd1;
		end
		else if(href_fall == 1'b1)
			row_cnt <= row_cnt + 16'd1;

		if(href_rise == 1'b1)
			col_cnt <= 16'd0;
		else if(href == 1'b1)
			col_cnt <= col_cnt + 16'd1;
	end
end

endmodule
