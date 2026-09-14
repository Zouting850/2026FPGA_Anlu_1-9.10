// ============================================================
// auto_exp.v
// M4 自动曝光闭环控制器（sys 域，50MHz）
//
// 【它解决什么问题】M1~M3 把曝光交给 MT9V034 片上 AEC/AGC。片上 AEC 追求
//   "看着好看"，会随场景持续微调。对背景建模这是致命的：曝光一动，整帧
//   灰度集体平移，帧差立刻满屏假前景，M2 的背景模型被反复冲垮。所以 M4
//   先在 R0xAF 写 0，**关掉片上 AEC/AGC**，改由本模块按固定目标亮度闭环。
//
// 【寄存器】（MT9V034 数据手册 Table 7）
//   R0x0B = Coarse Shutter Width Total（粗快门总量）  ← 主调节量
//   R0x35 = Analog Gain Control      （模拟增益）      ← 兜底调节量
//   R0xAF = AEC/AGC Enable                             ← 先写 0
//
// 【为什么必须先关片上 AEC，再自己闭环】两个控制器（片上 AEC 与本模块）
//   同时改同一个寄存器会互相打架：片上 AEC 按它自己的目标调，本模块按
//   96 灰阶目标调，结果是谁也收敛不了、画面来回晃。必须独占。
//
// 【控制律】比例步长，见 vision_def.v 的长注释：
//     Δshut = (shut * |sum - TARGET|) >> EXP_PSHIFT     （1 个乘法器，无除法器）
//   线性传感器下可推得 e' = -e²/T，误差按平方收缩 → 二次收敛且**绝不过冲**。
//   再钳到 [EXP_STEP_MIN, EXP_STEP_MAX]，并限制不越过快门上下限。
//
// 【快门优先、增益兜底】
//   偏亮：先减快门；快门已到下限才减增益。
//   偏暗：先加快门；快门已到上限才加增益。
//   理由：快门 = 积分时间，不引入额外噪声；增益把模拟信号连同噪声一起放大。
//   所以永远最后动增益。
//
// 【时序对齐——本模块最关键的约束】
//   手册原文：多数寄存器在"帧起始"整体生效；但**快门宽度的改动要等到
//   n+2 帧才在图像里体现**。因此：
//     1) 只在 frame_tick（帧起始，落在垂直消隐期）做决策与下发——
//        这也满足方案 v2.0 §4.2"曝光回写必须对齐 VSYNC 消隐期"；
//     2) 两次回写之间至少隔 EXP_UPDATE_DIV 帧（div_cnt 计数），
//        否则会拿"还没生效的旧亮度"去算误差，闭环自激振荡。
//
// 【锁定判据】连续 EXP_LOCK_FRAMES 帧都落在死区内 → exp_locked=1。
//   这是 M4 验收项"亮度收敛且无振荡"的板级证据：锁定后 exp_shut 不再变。
//   若某帧又跑出死区，band_cnt 清零、锁定解除，重新调节。
//
// 【几何门限】只有 frm_cnt == 90240（画面几何正常）才动曝光。窗口/接线
//   坏掉时 cnt 会明显偏离，此时调曝光没有意义，还可能把画面推到极端。
//
// 【为什么要等 cfg_done】mt9v034_cfg 的配置表会把 R0xAF 写 0x0003
//   （打开片上 AEC）。本模块必须等它跑完（cfg_done=1）再去写 0 覆盖，
//   否则会被配置表重新打开，而且配置期间的 SCCB 事务也应由 cfg 独占。
// ============================================================
`include "vision_def.v"

module auto_exp
(
	input                       clk,        // sys 50 MHz
	input                       rst,        // 高有效（与 sccb_master/mt9v034_cfg 一致）
	// ---- 观测量（sys 域准静态快照，整帧稳定）----
	input[31:0]                 frm_sum,    // 本帧像素灰度和
	input[23:0]                 frm_cnt,    // 本帧有效像素数（应 = 90240）
	input[23:0]                 frm_sat,    // 本帧饱和像素数（削顶保护，见下）
	input                       frame_tick, // 帧起始一拍（= 垂直消隐期）
	input                       cfg_done,   // 上电配置完成，才允许接管曝光
	// ---- 接 sccb_master（写；经顶层与 mt9v034_cfg 仲裁）----
	output reg                  sccb_req,
	output reg[15:0]            sccb_addr,
	output reg[15:0]            sccb_wdata,
	input                       sccb_busy,
	input                       sccb_ack,
	// ---- 状态 ----
	output reg[15:0]            exp_shut,   // 当前快门值（写 R0x0B）
	output reg[15:0]            exp_gain,   // 当前增益值（写 R0x35）
	output reg[7:0]             exp_mean,   // 本帧平均灰度（状态行显示用）
	output reg                  exp_locked, // 收敛锁定标志
	output reg[15:0]            wr_cnt      // 累计曝光回写次数（诊断）
);

localparam E_INIT   = 3'd0;   // 等 cfg_done
localparam E_AECSET = 3'd1;   // 发起 R0xAF = 0（关片上 AEC/AGC）
localparam E_AECBSY = 3'd2;
localparam E_AECACK = 3'd3;
localparam E_RUN    = 3'd4;   // 闭环运行：等 frame_tick 决策
localparam E_WRSET  = 3'd5;   // 发起曝光/增益回写
localparam E_WRBSY  = 3'd6;
localparam E_WRACK  = 3'd7;

reg[2:0]  st;
reg[3:0]  band_cnt;      // 连续落在死区内的帧数
reg[3:0]  div_cnt;       // 距上次回写已过的帧数
reg[15:0] pend_addr;
reg[15:0] pend_data;

// ------------------------------------------------------------
// 组合：误差 / 比例步长 / 决策条件
// ------------------------------------------------------------
// |sum - TARGET|（无符号幅度，避开有符号减法回绕）
wire[31:0] diff    = (frm_sum >= `EXP_TARGET_SUM)
                     ? (frm_sum - `EXP_TARGET_SUM)
                     : (`EXP_TARGET_SUM - frm_sum);
// 幅值上限：max(90240*255, TARGET) = 23,011,200 < 2^25，25 位足够
wire[24:0] errmag  = diff[24:0];

// 比例步长：Δshut = (shut * errmag) >> EXP_PSHIFT（≈ /TARGET，见 vision_def.v）
// 16 位 x 25 位 = 41 位中间积
wire[40:0] psh     = exp_shut * errmag;
wire[17:0] raw     = psh >> `EXP_PSHIFT;

// 钳位到 [EXP_STEP_MIN, EXP_STEP_MAX]：太小爬不动，太大跳变
wire[15:0] step_c  = (raw > `EXP_STEP_MAX) ? `EXP_STEP_MAX
                   : ((raw < `EXP_STEP_MIN) ? `EXP_STEP_MIN : raw[15:0]);

// 单次调整量还要限制"不越过上下限"（否则会算出越界值写进传感器）
wire[15:0] dec_amt = (step_c > (exp_shut - `EXP_SHUT_MIN))
                     ? (exp_shut - `EXP_SHUT_MIN) : step_c;
wire[15:0] inc_amt = (step_c > (`EXP_SHUT_MAX - exp_shut))
                     ? (`EXP_SHUT_MAX - exp_shut) : step_c;

wire geom_ok    = (frm_cnt == `CAM_FRAME_PIX);
// 饱和削顶保护：大量像素贴到 255 时，sum 被削顶压缩、闭环会**低估**亮度
// （差值全被削平），光靠 sum 判会以为"还不够亮"而继续加曝光，把画面彻底
// 糊死。所以饱和像素数超限就强制判偏亮。高对比场景（小面积强反光 +
// 大面积暗部）会提前触发这个保护、整体压暗——这是有意的取舍：对背景
// 建模来说，"不过曝"比"平均亮度好看"更重要。
wire sat_alarm  = (frm_sat > `EXP_SAT_MAX);
wire too_bright = geom_ok && ((frm_sum >  (`EXP_TARGET_SUM + `EXP_DEADBAND_SUM))
                              || (sat_alarm == 1'b1));
wire too_dark   = geom_ok && (frm_sum <  (`EXP_TARGET_SUM - `EXP_DEADBAND_SUM));
wire in_band    = geom_ok && (~too_bright) && (~too_dark);
wire div_ok     = (div_cnt >= `EXP_UPDATE_DIV);

// 均值显示用：sum / 90240。
//
// 【为什么这里必须写成"乘倒数 + 右移"，不能写除法】
//   原写法 `frm_sum / CAM_FRAME_PIX` 看着无害（除数是常量），但 TD **不会**
//   自动把它变成"乘倒数 + 移位"——它用纯组合逻辑搭了一个 32 级恢复余数除法器。
//   真实综合实测（M5，2026-09-14）：
//     Data Path Delay 50.564ns, Logic Level 48 (ADDER=32, LUT2=16)
//     slack -30.680ns（sys_clk 50MHz/20ns 域，10 条违例里占 8 条）
//   即这一行显示用的除法，直接把整个 sys_clk 域的时序打穿了。
//
//   改成 186 / 2^24 = 1.108665e-5 ≈ 1/90240（偏差 +0.046%）：
//     * frm_sum 有硬上界：90240 x 255 = 23,011,200（25 位）。乘 186 后最大
//       4,280,083,200 < 2^32，**32 位中间积不可能溢出**（无需更宽的中间量）；
//     * 再右移 24 位，最大 255.13 -> 255，正好落在 8 位无符号范围内；
//     * 186 = 128+32+16+8+2，全用移位相加；综合后逻辑级数从 48 降到个位数级。
//   这是 display-only 字段（只喂状态行的 Y=），0.046% 偏差无任何影响。
wire[31:0] mean_m = frm_sum * 32'd186;
wire[7:0]  mean_c = mean_m[31:24];

always@(posedge clk or posedge rst)
begin
	if(rst)
	begin
		st         <= E_INIT;
		band_cnt   <= 4'd0;
		div_cnt    <= 4'd0;
		pend_addr  <= 16'd0;
		pend_data  <= 16'd0;
		sccb_req   <= 1'b0;
		sccb_addr  <= 16'd0;
		sccb_wdata <= 16'd0;
		exp_shut   <= `EXP_SHUT_INIT;
		exp_gain   <= `EXP_GAIN_INIT;
		exp_mean   <= 8'd0;
		exp_locked <= 1'b0;
		wr_cnt     <= 16'd0;
	end
	else
	begin
		case(st)
			// ------------------------------------------------
			// 等配置流程跑完（配置期间 SCCB 由 mt9v034_cfg 独占）
			// ------------------------------------------------
			E_INIT:
			begin
				exp_shut   <= `EXP_SHUT_INIT;
				exp_gain   <= `EXP_GAIN_INIT;
				exp_locked <= 1'b0;
				band_cnt   <= 4'd0;
				div_cnt    <= 4'd0;
				if(cfg_done == 1'b1)
				begin
					sccb_addr  <= `MT_REG_AECAGC;
					sccb_wdata <= `MT_VAL_AECAGC_OFF;
					sccb_req   <= 1'b1;
					st         <= E_AECBSY;
				end
			end

			// 等 sccb_master 接单
			E_AECBSY:
				if(sccb_busy == 1'b1)
				begin
					sccb_req <= 1'b0;
					st       <= E_AECACK;
				end

			// 关 AEC 完成 → 进入闭环
			E_AECACK:
				if(sccb_ack == 1'b1)
				begin
					div_cnt <= 4'd0;
					st      <= E_RUN;
				end

			// ------------------------------------------------
			// 闭环运行：每帧更新一次均值显示与锁定判定，
			// 满足"几何正常 + 间隔够"时做一次调整决策
			//
			// 【div_cnt 必须只在 frame_tick 计数】它语义是"距上次回写过了
			//   几帧"。若放在帧内每拍自增，EXP_UPDATE_DIV=2 会在写完成 2 个
			//   时钟拍后就放行下一次写——"隔 N 帧"守卫完全失效，闭环会拿
			//   还没生效的旧亮度连算两次，等于把步长翻倍，过冲风险大增。
			//   （这正是写模型时抓出来的第三类"差一拍/差一帧"bug。）
			// ------------------------------------------------
			E_RUN:
			begin
				if(frame_tick == 1'b1)
				begin
					// 距上次回写的帧数（饱和在 EXP_UPDATE_DIV）
					if(div_cnt != `EXP_UPDATE_DIV)
						div_cnt <= div_cnt + 4'd1;
					// 均值显示：sum / 90240（除数是常量，综合成乘+移位）
					exp_mean <= (frm_cnt == `CAM_FRAME_PIX)
					            ? mean_c[7:0]
					            : 8'd0;

					// ---- 锁定判定 ----
					if(geom_ok == 1'b1 && in_band == 1'b1)
					begin
						if(band_cnt >= `EXP_LOCK_FRAMES)
							exp_locked <= 1'b1;
						else
							band_cnt <= band_cnt + 4'd1;
					end
					else
					begin
						band_cnt   <= 4'd0;
						exp_locked <= 1'b0;
					end

					// ---- 调整决策 ----
					if(geom_ok == 1'b1 && div_ok == 1'b1)
					begin
						if(too_bright == 1'b1)
						begin
							if(exp_shut > `EXP_SHUT_MIN)
							begin
								pend_addr <= `MT_REG_SHUTTER;
								pend_data <= exp_shut - dec_amt;
								st        <= E_WRSET;
							end
							else if(exp_gain > `EXP_GAIN_MIN)
							begin
								pend_addr <= `MT_REG_GAIN;
								pend_data <= exp_gain - `EXP_GAIN_STEP;
								st        <= E_WRSET;
							end
							// 快门与增益都到底：已经很亮了，不再动
						end
						else if(too_dark == 1'b1)
						begin
							if(exp_shut < `EXP_SHUT_MAX)
							begin
								pend_addr <= `MT_REG_SHUTTER;
								pend_data <= exp_shut + inc_amt;
								st        <= E_WRSET;
							end
							else if(exp_gain < `EXP_GAIN_MAX)
							begin
								pend_addr <= `MT_REG_GAIN;
								pend_data <= exp_gain + `EXP_GAIN_STEP;
								st        <= E_WRSET;
							end
							// 快门与增益都到顶：已经很暗了，不再动
						end
					end
				end
			end

			// ------------------------------------------------
			// 发起一次寄存器写
			// ------------------------------------------------
			E_WRSET:
			begin
				sccb_addr  <= pend_addr;
				sccb_wdata <= pend_data;
				sccb_req   <= 1'b1;
				st         <= E_WRBSY;
			end

			E_WRBSY:
				if(sccb_busy == 1'b1)
				begin
					sccb_req <= 1'b0;
					st       <= E_WRACK;
				end

			// 事务结束：把新值落到本地状态，并重新开始间隔计数
			E_WRACK:
				if(sccb_ack == 1'b1)
				begin
					if(pend_addr == `MT_REG_SHUTTER)
						exp_shut <= pend_data;
					else
						exp_gain <= pend_data;
					wr_cnt  <= wr_cnt + 16'd1;
					div_cnt <= 4'd0;
					st      <= E_RUN;
				end

			default:
				st <= E_INIT;
		endcase
	end
end

endmodule
