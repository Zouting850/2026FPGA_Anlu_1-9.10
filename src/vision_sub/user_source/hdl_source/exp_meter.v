// ============================================================
// exp_meter.v
// M4 曝光计量器（PCLK 域）——给自动曝光闭环提供"这一帧有多亮"
//
// 输出三个量，都在 frame_start（帧起始）锁存：
//   sum_o : 本帧全部有效像素的灰度和    —— 曝光闭环的主观测量
//   cnt_o : 本帧有效像素数              —— 应恒 = 90240，用于判断几何是否正常
//   sat_o : 本帧"接近饱和"的像素数      —— 削顶预警
//
// 【为什么再写一个累加器，而不复用 frame_stat.v】
//   frame_stat 已经算了 sum/cnt/min/max，看起来够用。但 M4 需要多一个
//   "饱和像素数"，而往 frame_stat 加端口就得动 M1 的模块与 M1 的模型。
//   本项目的原则是"里程碑一旦验完就冻结"，M4 只准新增、不准改动
//   M1~M3 的 RTL。多一个 25 位累加器的代价是约 60 个 FF，远低于
//   "改老模块再重新验证"的代价。所以宁可重复，也不回改。
//
// 【为什么 sat 只统计 > EXP_SAT_LEVEL，而不是整张直方图】
//   闭环只需要知道"有没有削顶"这一个 bit 级别的信息：如果大量像素贴近
//   255，说明画面已经顶死，再加大曝光只会把细节全糊掉。整张 256 箱直方图
//   要 256x17bit 的寄存器或 BRAM，对 M4 是浪费。
//
// 【为什么 sum 用 32 位】90240 x 255 = 23,011,200，25 位就够。留 32 位
//   是为了和 frame_stat 的 sum_o 位宽一致，方便顶层直连、也方便串口
//   状态行直接打印十六进制而不必再扩位。
//
// 【跨时钟域】锁存值在整帧（16.7ms）内稳定，远长于 done_tog 同步到
//   sys 域所需的几拍，因此顶层用"翻转边沿 + 读快照"是安全的
//   （准静态多比特跨时钟域），与 frame_stat / fg_stat 同一套做法。
// ============================================================
`include "vision_def.v"

module exp_meter
(
	input                       pclk,
	input                       rst_n,
	input                       pix_valid,
	input[7:0]                  pix_data,
	input                       frame_start,
	output reg[31:0]            sum_o,
	output reg[23:0]            cnt_o,
	output reg[23:0]            sat_o,
	output reg                  meter_done_tog
);

reg[31:0] acc_sum;
reg[23:0] acc_cnt;
reg[23:0] acc_sat;

always@(posedge pclk or negedge rst_n)
begin
	if(!rst_n)
	begin
		acc_sum        <= 32'd0;
		acc_cnt        <= 24'd0;
		acc_sat        <= 24'd0;
		sum_o          <= 32'd0;
		cnt_o          <= 24'd0;
		sat_o          <= 24'd0;
		meter_done_tog <= 1'b0;
	end
	else
	begin
		if(pix_valid == 1'b1)
		begin
			acc_sum <= acc_sum + {24'd0, pix_data};
			acc_cnt <= acc_cnt + 24'd1;
			// 饱和计数：只关心"贴近 255"，用于削顶预警
			if(pix_data >= `EXP_SAT_LEVEL)
				acc_sat <= acc_sat + 24'd1;
		end

		// FRAME_VALID 上升沿落在行消隐期，此时累加器已稳定
		if(frame_start == 1'b1)
		begin
			sum_o          <= acc_sum;
			cnt_o          <= acc_cnt;
			sat_o          <= acc_sat;
			meter_done_tog <= ~meter_done_tog;
			acc_sum        <= 32'd0;
			acc_cnt        <= 24'd0;
			acc_sat        <= 24'd0;
		end
	end
end

endmodule
