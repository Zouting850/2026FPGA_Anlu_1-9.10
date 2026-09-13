// ============================================================
// projection.v
// M3 行列投影：对形态学输出做列直方图与行直方图，并逐帧读出
//
// 直方图就是"投影曲线"：
//   列直方图 col_sum[k] = 第 4k..4k+3 列上的前景像素总数 → 水平投影曲线，
//                         人数分段就看它；
//   行直方图 row_sum[k] = 第 4k..4k+3 行上的前景像素总数 → 垂直投影曲线，
//                         用来给人所在的行范围（上下边界）。
//
// 【为什么按 4 列 / 4 行一箱，而不是逐列 376 箱】
//   1) 省资源：94 箱 x 10 bit + 60 箱 x 11 bit ≈ 1600 FF，读多路器 ≈ 500 LUT；
//      逐列 376 箱要 3000+ FF、1300+ LUT，贵一倍多。
//   2) 不损失可观测精度：串口打出来的曲线本来就只有 94 个字符（1 字符 = 1 箱），
//      全分辨率直方图在板子上根本看不见。
//   3) 抗噪更好：先把相邻 4 列加起来再判阈值，单列噪点不容易自己凑成一段。
//   分段分辨率降到 4 列，而人形约 60 列宽（15 箱），绰绰有余。
//
// 【为什么用寄存器阵列而不是 BRAM】
//   BRAM 单口同一拍只能干一件事，"累加读改写"和"读出清零"必须错开拍或错开
//   时间窗，代码和时序都会绕。寄存器阵列的索引读是组合多路器、索引写是译码器，
//   两者物理独立，于是：
//       - 累加可以单拍读改写（同一拍读旧值、写回 +1）；
//       - 读出可以同一拍读值、同一拍清零；
//   没有任何端口冲突，也不需要双缓冲。
//   代价是读多路器（94:1 x 10bit + 60:1 x 11bit 两次，累加一路、读出一路）
//   约 1000 LUT —— 相对 19.6K LUT 完全可接受。
//
// 【读出为什么用组合 ro_bin】
//   ro_bin 若也打一拍，读出第一拍读到的是上一帧留下残值（行相位末尾 = 59），
//   会去读/清 col_sum[59]，把第 59 箱的值提前清掉，之后真正轮到时读到 0。
//   所以 ro_bin 必须由 ro_cnt 组合产生，与 ro_cnt 同步。
//
// 【帧切换时序】
//   帧头（VSYNC 上升沿）起，用 94 + 60 = 154 拍把上一帧两条直方图读出，
//   边读边清零（顺手把曲线字符移入 curve_bits），最后一拍翻转 viz_done_tog
//   供顶层做跨时钟域快照。154 拍 ≈ 11 us @13.5MHz。
//
//   读出期间关掉累加（acc_en 里带 ~ro_active）。**这不会丢任何像素**：
//   154 拍 < 一行 376 拍，所以这 154 拍要么落在行/帧消隐里（本来就没有像素），
//   要么只覆盖第 0 行的一部分——而 morph 出口的有效区要求 y >= 4、x >= 6，
//   第 0 行的像素一律被丢弃。两种情况都不损失统计值。
// ============================================================
`include "vision_def.v"

module projection
(
	input                       clk,         // PCLK 域
	input                       rst_n,
	// ---- 形态学输出流（m_valid 为高时 m_fg / m_x / m_y 有效）----
	input                       m_valid,
	input                       m_fg,
	input[15:0]                 m_x,
	input[15:0]                 m_y,
	// ---- 帧边界（dvp_capture 的 frame_start）----
	input                       frame_start,
	// ---- 上一帧形态学后的前景像素数 ----
	output reg[16:0]            m_cnt_o,
	// ---- 直方图读出流（每拍一个箱，共 154 拍）----
	output                      ro_valid,
	output                      ro_phase,      // 0 = 列直方图，1 = 行直方图
	output[6:0]                 ro_bin,        // 箱号（组合，与 ro_cnt 同步）
	output[`PROJ_RW-1:0]        ro_val,        // 箱值（组合，与 ro_bin 同步）
	// ---- 读出结束（顶层据此做跨时钟域快照）----
	output reg                  viz_done_tog,
	// ---- 曲线字符：94 个 hex nibble 打包成 376 bit，bit[3:0] 是第 0 箱 ----
	// 打包顺序很讲究：新字符从**高位**进、已有内容整体右移 4 位。
	// 这样最后一个被打包的箱 93 停在最高位，而最先打包的箱 0 一路右移
	// 停在**最低** 4 位 —— 顶层从 bit[3:0] 开始每吐一个字符右移 4 位，
	// 正好按箱 0、1、2… 的顺序输出。
	// （若图省事写成 {curve_bits[CURVE_W-5:0], nib}，箱 0 会跑到最高位，
	//   串口打出来的曲线就是左右镜像的，峰位全反——模型里有这条对照。）
	output reg[`CURVE_N*4-1:0]  curve_bits
);

localparam[7:0] COL_N = `PROJ_COL_BINS;  // 94
localparam[7:0] ROW_N = `PROJ_ROW_BINS;  // 60
localparam[8:0] RO_N  = COL_N + ROW_N;   // 154

// ------------------------------------------------------------
// 直方图寄存器阵列
// ------------------------------------------------------------
reg[`PROJ_CW-1:0] col_sum[0:COL_N-1];    // 10 bit，上限 4*240 = 960
reg[`PROJ_RW-1:0] row_sum[0:ROW_N-1];    // 11 bit，上限 4*376 = 1504

// ------------------------------------------------------------
// 箱号
//   m_x <= 371、m_y <= 237（morph 出口内缩 2 像素），
//   所以 m_x>>2 <= 92 < 94、m_y>>2 <= 59 < 60，索引恒在范围内。
// ------------------------------------------------------------
wire[6:0] cb = {5'd0, m_x[15:2]};
wire[5:0] rb = m_y[15:2];

// ------------------------------------------------------------
// 读出计数器 / 进行中标志
// ------------------------------------------------------------
reg      ro_active;
reg[7:0] ro_cnt;

wire       ro_last = (ro_cnt == (RO_N - 9'd1));
wire       col_ph  = (ro_cnt < COL_N);
wire[7:0]  row_idx = ro_cnt - COL_N;

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		ro_active <= 1'b0;
		ro_cnt    <= 8'd0;
	end
	else if(frame_start == 1'b1)
	begin
		ro_active <= 1'b1;
		ro_cnt    <= 8'd0;
	end
	else if(ro_active == 1'b1)
	begin
		if(ro_last == 1'b1)
			ro_active <= 1'b0;
		else
			ro_cnt <= ro_cnt + 8'd1;
	end
end

// ------------------------------------------------------------
// 读出地址与相位（**都是组合**）
//   ro_phase 千万不能打拍：它若比 ro_bin 慢一拍，ro_cnt=94 那一拍的
//   ro_bin 已经是行箱 0（组合），ro_phase 却还是"列相位"，两者不自洽。
//   后果是 people_seg 的列相位收尾条件（ro_phase && ro_bin==0）永不成立，
//   画面最右边那一段永远不会被计数；同时行箱 0 会被当成列箱 0 混进列统计。
//   所以 ro_phase 必须和 ro_bin 一样由 ro_cnt 组合产生。
// ------------------------------------------------------------
assign ro_phase = ~col_ph;

assign ro_bin = col_ph ? {1'b0, ro_cnt[5:0]} : {1'b0, row_idx[5:0]};

assign ro_valid = ro_active;

assign ro_val = ro_phase ? row_sum[ro_bin[5:0]] :
                          {{(`PROJ_RW-`PROJ_CW){1'b0}}, col_sum[ro_bin[6:0]]};

// 曲线字符：列箱取高 4 位即 >>6。上限 960 >> 6 = 15，恰好落在 0..15，
// 不用饱和。
wire[3:0] nib = col_sum[ro_bin[6:0]][`PROJ_CW-1 -: 4];
// ------------------------------------------------------------
// 累加 / 清零
//   同一拍只会走其中一个分支；读出期间 acc_en 恒 0（见文件头说明）。
// ------------------------------------------------------------
wire acc_en = m_valid & m_fg & ~ro_active;

integer i;

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		for(i=0;i<COL_N;i=i+1)
			col_sum[i] <= {`PROJ_CW{1'b0}};
		for(i=0;i<ROW_N;i=i+1)
			row_sum[i] <= {`PROJ_RW{1'b0}};
	end
	else if(acc_en == 1'b1)
	begin
		col_sum[cb] <= col_sum[cb] + {{(`PROJ_CW-1){1'b0}}, 1'b1};
		row_sum[rb] <= row_sum[rb] + {{(`PROJ_RW-1){1'b0}}, 1'b1};
	end
	else if(ro_active == 1'b1)
	begin
		if(col_ph == 1'b1)
			col_sum[ro_bin[6:0]] <= {`PROJ_CW{1'b0}};   // 边读边清零
		else
			row_sum[ro_bin[5:0]] <= {`PROJ_RW{1'b0}};
	end
end

// ------------------------------------------------------------
// 帧内计数 → 帧头锁存
// ------------------------------------------------------------
reg[16:0] acc_cnt;

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		acc_cnt <= 17'd0;
		m_cnt_o <= 17'd0;
	end
	else if(frame_start == 1'b1)
	begin
		acc_cnt <= 17'd0;
		m_cnt_o <= acc_cnt;              // 读清零前的旧值 = 上一帧
	end
	else if((m_valid == 1'b1) && (m_fg == 1'b1))
		acc_cnt <= acc_cnt + 17'd1;
end

// ------------------------------------------------------------
// 曲线打包 + 读出结束
// ------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		curve_bits <= {(`CURVE_N*4){1'b0}};
	else if((ro_active == 1'b1) && (col_ph == 1'b1))
		curve_bits <= {nib, curve_bits[(`CURVE_N*4-1):4]};
end

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		viz_done_tog <= 1'b0;
	else if((ro_active == 1'b1) && (ro_last == 1'b1))
		viz_done_tog <= ~viz_done_tog;
end

endmodule
