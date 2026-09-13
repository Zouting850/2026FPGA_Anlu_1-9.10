// ============================================================
// fg_stat.v
// M2 逐帧前景统计：前景像素计数 + 前景包围盒
//
// 这两个量就是 M2 验收要看的证据：
//   - 前景像素计数 fg_cnt：静态场景应 → 0；有人 / 挥手时明显大于 0
//   - 包围盒 min/max(x,y)：证明前景是"一坨东西"而不是散布的噪点，
//     而且位置对得上（挥手 → 框出现在挥手的那片区域）
//     计数值为 0 时 box_nz=0，包围盒输出全 0。
//
// 全部工作在 PIXCLK 域。frame_start 时把上一帧的累加结果锁存到 *_o，
// 同时翻转 frame_done_tog，供顶层同步到 sys 域后取准静态快照
// （与 frame_stat.v 的握手方式一致）。
//
// 锁存块读的是清零前的旧值：两个 always 块都在同一个 frame_start 边沿
// 动作，而锁存块先采样、累加块后清零，因此拿到的是完整上一帧。
// ============================================================
`include "vision_def.v"

module fg_stat
(
	input                       clk,           // PIXCLK 域
	input                       rst_n,
	// ---- 来自 m2_engine 的前景像素流 ----
	input                       fg_valid,
	input                       fg,
	input[15:0]                 fg_x,
	input[15:0]                 fg_y,
	// ---- 帧起始（dvp_capture 的 frame_start）----
	input                       frame_start,
	// ---- 逐帧锁存结果 ----
	output reg[16:0]            fg_cnt_o,
	output reg[15:0]            box_min_x_o,
	output reg[15:0]            box_max_x_o,
	output reg[15:0]            box_min_y_o,
	output reg[15:0]            box_max_y_o,
	output reg                  box_nz_o,
	output reg                  frame_done_tog
);

// ------------------------------------------------------------
// 帧内累加器
// ------------------------------------------------------------
reg[16:0] acc_cnt;
reg[15:0] acc_mnx;
reg[15:0] acc_mxx;
reg[15:0] acc_mny;
reg[15:0] acc_mxy;
reg       acc_nz;

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		acc_cnt <= 17'd0;
		acc_mnx <= 16'd0;
		acc_mxx <= 16'd0;
		acc_mny <= 16'd0;
		acc_mxy <= 16'd0;
		acc_nz  <= 1'b0;
	end
	else if(frame_start == 1'b1)
	begin
		acc_cnt <= 17'd0;
		acc_mnx <= 16'd0;
		acc_mxx <= 16'd0;
		acc_mny <= 16'd0;
		acc_mxy <= 16'd0;
		acc_nz  <= 1'b0;
	end
	else if((fg_valid == 1'b1) && (fg == 1'b1))
	begin
		acc_cnt <= acc_cnt + 17'd1;
		acc_nz  <= 1'b1;
		if(acc_cnt == 17'd0)                    // 本帧第一个前景像素
		begin
			acc_mnx <= fg_x;
			acc_mxx <= fg_x;
			acc_mny <= fg_y;
			acc_mxy <= fg_y;
		end
		else
		begin
			if(fg_x < acc_mnx) acc_mnx <= fg_x;
			if(fg_x > acc_mxx) acc_mxx <= fg_x;
			if(fg_y < acc_mny) acc_mny <= fg_y;
			if(fg_y > acc_mxy) acc_mxy <= fg_y;
		end
	end
end

// ------------------------------------------------------------
// 帧起始锁存（读旧值）+ 翻转 done
// ------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		fg_cnt_o       <= 17'd0;
		box_min_x_o    <= 16'd0;
		box_max_x_o    <= 16'd0;
		box_min_y_o    <= 16'd0;
		box_max_y_o    <= 16'd0;
		box_nz_o       <= 1'b0;
		frame_done_tog <= 1'b0;
	end
	else if(frame_start == 1'b1)
	begin
		fg_cnt_o       <= acc_cnt;
		box_min_x_o    <= acc_nz ? acc_mnx : 16'd0;
		box_max_x_o    <= acc_nz ? acc_mxx : 16'd0;
		box_min_y_o    <= acc_nz ? acc_mny : 16'd0;
		box_max_y_o    <= acc_nz ? acc_mxy : 16'd0;
		box_nz_o       <= acc_nz;
		frame_done_tog <= ~frame_done_tog;
	end
end

endmodule
