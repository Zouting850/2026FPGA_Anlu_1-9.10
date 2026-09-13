// ============================================================
// frame_stat.v
// M1 帧指纹累加器（PCLK 域）
//
// 目的：给 M1 一个"摄像头到底出没出图、出得像不像样"的可观测指标。
//   - sum/avg/min/max：判断是否全黑、是否过曝、是否有噪声
//   - cnt            ：判断有效像素数是否 = 376x240 = 90240
//                      （不符 → binning / 窗口 / 接线有问题）
//
// 累加在 HREF 有效期内进行；在 FRAME_VALID 上升沿（frame_start）
// 把结果锁存到 *_o 并翻转 frame_done_tog。
//
// 锁存值在整帧（16.7 ms）内保持稳定，远长于 frame_done_tog 传到
// 系统时钟域所需的几个周期，因此上层用"toggle 边沿 + 读快照"的
// 做法是安全的（准静态多比特跨时钟域）。
// ============================================================
`include "vision_def.v"

module frame_stat
(
	input                       pclk,
	input                       rst_n,
	input                       pix_valid,
	input[7:0]                  pix_data,
	input                       frame_start,
	output reg[31:0]            sum_o,
	output reg[23:0]            cnt_o,
	output reg[7:0]             min_o,
	output reg[7:0]             max_o,
	output reg                  frame_done_tog
);

reg[31:0] acc_sum;
reg[23:0] acc_cnt;
reg[7:0]  acc_min;
reg[7:0]  acc_max;

always@(posedge pclk or negedge rst_n)
begin
	if(!rst_n)
	begin
		acc_sum        <= 32'd0;
		acc_cnt        <= 24'd0;
		acc_min        <= 8'hFF;
		acc_max        <= 8'h00;
		sum_o          <= 32'd0;
		cnt_o          <= 24'd0;
		min_o          <= 8'hFF;
		max_o          <= 8'h00;
		frame_done_tog <= 1'b0;
	end
	else
	begin
		if(pix_valid == 1'b1)
		begin
			acc_sum <= acc_sum + {24'd0, pix_data};
			acc_cnt <= acc_cnt + 24'd1;
			if(pix_data < acc_min) acc_min <= pix_data;
			if(pix_data > acc_max) acc_max <= pix_data;
		end

		// FRAME_VALID 上升沿在行消隐期，累加器已稳定
		if(frame_start == 1'b1)
		begin
			sum_o          <= acc_sum;
			cnt_o          <= acc_cnt;
			min_o          <= acc_min;
			max_o          <= acc_max;
			frame_done_tog <= ~frame_done_tog;
			acc_sum        <= 32'd0;
			acc_cnt        <= 24'd0;
			acc_min        <= 8'hFF;
			acc_max        <= 8'h00;
		end
	end
end

endmodule
