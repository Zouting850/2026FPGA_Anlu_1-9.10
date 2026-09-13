// ============================================================
// m2_engine.v
// M2 像素流水线：光栅寻址 + 背景读写 + 前景流输出
//
// 工作在 PIXCLK 域，与 dvp_capture 同域，全程单时钟、无跨时钟域。
//
// 流水线（配合 bg_store 的 1 拍读延迟）：
//   拍 t   ：pix_valid=1，当前像素坐标 (x,y) 与线性地址 idx
//            rd_en=1, rd_addr=idx              → 下一拍拿到旧背景
//            cur / idx / x / y / valid 各打一拍
//   拍 t+1 ：bg_model 用 (cur_d, bg_rd) 算出 fg 与 bg_next
//            wr_en=1, wr_addr=idx_d, wr_data=bg_next
//   拍 t+2 ：输出 (fg_valid, fg, fg_x=x_d, fg_y=y_d) 全部对齐
//
// **为什么 fg 也要打一拍**：fg 是纯组合（由 cur_d / bg_rd 算出），有效拍是
// t+1；而 vld_d / x_d / y_d 是 t 拍的寄存器，有效拍是 t+2。若把组合的 fg
// 直接和 fg_valid 一起送 fg_stat，两者会差一拍——fg_stat 会把"上上拍那个
// 像素"的前景位，算到"上一拍那个像素"的坐标 (fg_x,fg_y) 上。效果是包围盒
// 整体偏移 1 个像素、帧边界计数偏差。把 fg 也寄存，四路输出才真正同拍。
//
// 任一拍内读地址与写地址必然不同（读 idx、写 idx-1），不存在同地址
// 读写冲突，因此不依赖 BRAM 的 write-first / read-first 模式。
//
// 装载阶段：复位后前 BG_LOAD_FRAMES 帧强制 bg ← cur 且 fg 恒 0。
// 原因见 vision_def.v：BRAM 上电内容未定义，若直接拿它当背景，开机
// 会满屏假前景，而前景像素每 FG_DIV 帧才走 1 级，靠算法收敛要好几十秒。
//
// 越界保护：一帧本应恰好 90240 个有效像素。多出来的 HREF 活动一律丢弃，
// 不让它绕回 17 bit 地址空间去踩别的像素。
// ============================================================
`include "vision_def.v"

module m2_engine
(
	input                       pclk,
	input                       rst_n,        // 低有效，PIXCLK 域
	// ---- 来自 dvp_capture ----
	input[7:0]                  pix_data,
	input                       pix_valid,
	input                       frame_start,
	// ---- 前景像素流（fg_valid 为高时 fg / fg_x / fg_y 有效）----
	output reg                  fg_valid,
	output reg                  fg,
	output reg[15:0]            fg_x,
	output reg[15:0]            fg_y,
	// ---- 状态输出 ----
	output reg                  bg_loading
);

localparam DEPTH = `CAM_FRAME_PIX;      // 90240

// ------------------------------------------------------------
// 光栅坐标 + 线性地址
//   x/y 完全由 pix_valid 递推（不依赖 line_start），这样每拍取到的
//   坐标就是"当拍那个像素"的坐标，不差一拍：
//     - 前一行的最后一个像素使 x 归零、y 加一
//     - frame_start 把 x/y/idx 一起清零
// ------------------------------------------------------------
reg[16:0] px_idx;
reg[15:0] x_pos;
reg[15:0] y_pos;

always@(posedge pclk or negedge rst_n)
begin
	if(!rst_n)
	begin
		px_idx <= 17'd0;
		x_pos  <= 16'd0;
		y_pos  <= 16'd0;
	end
	else if(frame_start == 1'b1)
	begin
		px_idx <= 17'd0;
		x_pos  <= 16'd0;
		y_pos  <= 16'd0;
	end
	else if(pix_valid == 1'b1)
	begin
		px_idx <= px_idx + 17'd1;
		if(x_pos == (`CAM_IMG_W - 1))
		begin
			x_pos <= 16'd0;
			y_pos <= y_pos + 16'd1;
		end
		else
			x_pos <= x_pos + 16'd1;
	end
end

wire idx_ok = (px_idx < DEPTH);

// ------------------------------------------------------------
// 打一拍，对齐 bg_store 的 1 拍读延迟
// ------------------------------------------------------------
reg[7:0]  cur_d;
reg[16:0] idx_d;
reg[15:0] x_d;
reg[15:0] y_d;
reg       vld_d;

always@(posedge pclk or negedge rst_n)
begin
	if(!rst_n)
	begin
		cur_d <= 8'd0;
		idx_d <= 17'd0;
		x_d   <= 16'd0;
		y_d   <= 16'd0;
		vld_d <= 1'b0;
	end
	else
	begin
		cur_d <= pix_data;
		idx_d <= px_idx;
		x_d   <= x_pos;
		y_d   <= y_pos;
		vld_d <= pix_valid & idx_ok;
	end
end

// ------------------------------------------------------------
// 装载阶段计数
//   load_cnt 记"已经装载完的帧数"。数到 BG_LOAD_FRAMES 时撤掉 bg_loading，
//   因此恰好有 BG_LOAD_FRAMES 帧（第 1..BG_LOAD_FRAMES 帧）走强制装载。
//   （若写成 load_cnt == BG_LOAD_FRAMES-1 就只剩 15 帧，是个经典 off-by-one。）
// ------------------------------------------------------------
reg[7:0] load_cnt;

always@(posedge pclk or negedge rst_n)
begin
	if(!rst_n)
	begin
		load_cnt   <= 8'd0;
		bg_loading <= 1'b1;
	end
	else if(frame_start == 1'b1)
	begin
		if(bg_loading == 1'b1)
		begin
			if(load_cnt == `BG_LOAD_FRAMES)
				bg_loading <= 1'b0;
			else
				load_cnt <= load_cnt + 8'd1;
		end
	end
end

// ------------------------------------------------------------
// 前景像素"每 FG_DIV 帧走 1 级"的全局使能
// ------------------------------------------------------------
reg[7:0] fg_fcnt;

always@(posedge pclk or negedge rst_n)
begin
	if(!rst_n)
		fg_fcnt <= 8'd0;
	else if(frame_start == 1'b1)
		fg_fcnt <= (fg_fcnt == (`FG_DIV - 8'd1)) ? 8'd0 : (fg_fcnt + 8'd1);
end

wire fg_tick = (fg_fcnt == 8'd0) && (bg_loading == 1'b0);

// ------------------------------------------------------------
// 组合逻辑网线（先声明后使用）
// ------------------------------------------------------------
wire[7:0] bg_rd;
wire[7:0] fg_diff;
wire      fg_raw;
wire[7:0] bg_upd;
wire[7:0] bg_wr;

wire rd_en = pix_valid & idx_ok;

// 装载阶段直接写当前像素，且不报前景
assign bg_wr   = bg_loading ? cur_d : bg_upd;
wire   fg_comb = bg_loading ? 1'b0  : fg_raw;

// ------------------------------------------------------------
// 背景存储（1 拍读延迟）
// ------------------------------------------------------------
bg_store u_store
(
	.clk      (pclk),
	.rd_en    (rd_en),
	.rd_addr  (px_idx),
	.rd_data  (bg_rd),
	.wr_en    (vld_d),
	.wr_addr  (idx_d),
	.wr_data  (bg_wr)
);

// ------------------------------------------------------------
// 背景模型
// ------------------------------------------------------------
bg_model u_model
(
	.cur      (cur_d),
	.bg       (bg_rd),
	.fg_tick  (fg_tick),
	.fg_diff  (fg_diff),
	.fg       (fg_raw),
	.bg_next  (bg_upd)
);

// ------------------------------------------------------------
// 输出对齐（四路同拍：fg_valid / fg / fg_x / fg_y）
// ------------------------------------------------------------
always@(posedge pclk or negedge rst_n)
begin
	if(!rst_n)
	begin
		fg_valid <= 1'b0;
		fg       <= 1'b0;
		fg_x     <= 16'd0;
		fg_y     <= 16'd0;
	end
	else
	begin
		fg_valid <= vld_d;
		fg       <= fg_comb;
		fg_x     <= x_d;
		fg_y     <= y_d;
	end
end

endmodule
