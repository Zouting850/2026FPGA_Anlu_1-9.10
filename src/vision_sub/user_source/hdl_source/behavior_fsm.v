// ============================================================
// behavior_fsm.v
// M5 行为状态机 + 置信度（方案 v2.0 §2.1/§2.2）
//
// 上游 M3 只给"这一帧看起来有几个人"这个瞬时量，没有任何时间维度。
// 本模块把帧序列读成"行为"：
//   有人来了吗？站定了吗？是几个人？在互动吗？离开多久了？
// 输出 = 行为状态 + 建议播放模式 + 置信度 + 停留时长 + 互动次数。
//
// **副板只给建议，最终裁决权在主板**（方案 §2 结论）：副板不知道媒体
// 清单/播放进度/内容分级，只能把观众行为翻译成建议 + 置信度。所以这里
// 没有任何"动作指令"，只有 (state, mode, conf) 三元组和事件脉冲。
//
// 【运行节拍】全部逻辑在 sys 域、由 frame_tick 驱动：一个 frame_tick =
// 一帧。这样计数器单位就是帧，与 RTL 里"连续 N 帧"的表述一一对应，
// 模型（tools/sim_vision_sub_m5.py）也按帧推进，两边不会差拍。
//
// 【七态】
//   IDLE     无人（或已离场满 10 s）—— 建议 STANDBY
//   PRESENCE 有人但还没确认"站定"—— 建议 SINGLE（低置信度，主板可自行忽略）
//             之所以有人就先给 SINGLE 而不是 STANDBY：真实展项里观众一
//             走近就该开始播片头，"等人站定 2 s 再播"会显得迟钝。置信度
//             字段（C=）就是给主板做这个取舍用的。
//   SINGLE   1 人稳定 → SINGLE
//   MULTI    ≥2 人稳定 → MULTI
//   INTERACT 检测到互动动作 → INTERACT（驻留 COOL_FRAMES 即冷却期）
//   LEAVE    人刚走（人数 0 满 LEAVE_FRAMES）—— 锁存停留时长，建议 STANDBY
//   ALERT    目标贴到画面最外侧警戒带 —— 建议 ALERT
//
// 【人数 ±1 死区 + 同向确认】（方案 §2.2 防误触发第 1、2 条）
//   pplf 是滤波后的人数，与瞬时 ppl_i 的关系：
//     - 差值 ≥ 2（PPL_DEADBAND+1）：立刻跟随。真来了一群人不用等。
//     - 差值 = 1：必须**同方向**连续 PPL_CONFIRM 帧才跟。方向一变就
//       重新计数（1→2→1→2 这种反复横跳永远攒不满）。
//     - 相等：计数清零。
//   这一条是 M5 里最要紧的：M3 的段数在 1↔2 之间抖是常态（两人靠近/
//   分开、一人被自己的影子切成两段），不滤掉播放模式就会来回切。
//
// 【方向判别滑窗】DIR_WIN 帧为一窗，记窗内质心列的 min/max。
//   跨度 ≥ DIR_MIN 箱算"有横向位移"（挥手时手臂来回扫，跨度足够）。
//
// 【能量基线与突增】见 vision_def.v 的 M5 段注释：基线只在非 INTERACT
//   状态更新、首个有效帧直接灌初值、人数一变就要等 NRG_BASE_READY 帧
//   让基线追上（否则"来人/变多人"会被当成挥手），突增同时看绝对增量
//   和 1.5 倍，两个都要。
//
// 【置信度】三路 0..255 评分（dwell / motion / quiet）各自饱和积分，
//   合成 conf = (sc_sum * 85) >> 8 ≈ sc_sum / 3。
//   85 = 64+16+4+1，所以实现成移位相加，一个乘法器都不用。
//
// 【几何坏帧】geom_ok=0（有效像素数不对）时冻结状态机、只让置信度缓降。
//   理由：画面几何坏掉时人数/能量都不可信，此时做任何跃迁都是拿噪声
//   当证据；但也不能当"没人"——那会把观众误判成离场、白白切断播放。
// ============================================================
`include "vision_def.v"

module behavior_fsm
(
	input                       clk,
	input                       rst,
	// ---- 帧节拍与数据有效性 ----
	input                       frame_tick,   // 每帧一拍（亮度计量路径派生）
	input                       geom_ok,      // 本帧有效像素数正常
	// ---- M3 帧快照（帧粒度，准静态）----
	input[3:0]                  ppl_i,        // 瞬时人数（M3 段数）
	input[6:0]                  ctr_i,        // 质心列箱
	input[6:0]                  cf_i,         // 首个占用列箱
	input[6:0]                  cl_i,         // 末个占用列箱
	input[7:0]                  occ_i,        // 占用列箱数
	input[16:0]                 nrg_i,        // 形态学后前景像素数
	// ---- 结果 ----
	output[2:0]                 st_o,         // 行为状态
	output[2:0]                 mode_o,       // 建议播放模式
	output[7:0]                 conf_o,       // 置信度 0..254
	output reg[15:0]            dwell_o,      // 停留帧数（LEAVE 时停更）
	output reg[7:0]             icnt_o,       // 互动次数
	output[3:0]                 pplf_o,       // 死区滤波后人数
	output reg                  ev_enter_o,   // 检测到观众（1 拍脉冲）
	output reg                  ev_leave_o,   // 观众离开（1 拍脉冲）
	output reg                  ev_interact_o,// 互动命中（1 拍脉冲）
	output reg[1:0]             dir_o,        // 最近一次互动方向 1=左 2=右 3=前推
	output reg                  push_o        // 最近一次互动是前推
);

// ------------------------------------------------------------
// 状态寄存器
//
// 必须先于引用它们的组合逻辑声明。TD 编译会报
//   HDL-5373 WARNING: identifier 'xxx' is used before its declaration
// 对 pplf / ctr_p / nrg_base 这类多比特量，早期工具会退化成隐式
// 1-bit net —— 静默截断成 1 位，比报错更可怕。全部声明前移。
// ------------------------------------------------------------
reg[2:0]  st;
reg[3:0]  pplf;
reg[1:0]  pdir_cnt;
reg       pdir_up;

reg[2:0]  enter_cnt;      // IDLE/LEAVE -> PRESENCE
reg[2:0]  multi_cnt;      // -> MULTI
reg[2:0]  single_cnt;     // -> SINGLE（MULTI 回落）
reg[2:0]  guard_cnt;      // -> ALERT
reg[2:0]  unguard_cnt;    // ALERT 退出
reg[7:0]  stable_cnt;     // PRESENCE -> SINGLE
reg[7:0]  leave_cnt;      // -> LEAVE
reg[9:0]  idle_cnt;       // LEAVE -> IDLE
reg[7:0]  cool_cnt;       // INTERACT 驻留/冷却

reg[6:0]  ctr_p;
reg[6:0]  cmin, cmax, wfirst, wlast;
reg[4:0]  wcnt;
reg[16:0] nrg_base;
reg       base_init;      // 首个有效帧把基线直接灌成当前能量
reg[5:0]  warm_cnt;       // 人数变化后的基线收敛计数（饱和）
reg[3:0]  pplr_d;         // 原始人数的延后副本（检测"人数一变就清零 warm_cnt"）

reg[7:0]  sc_dwell, sc_motion, sc_quiet;

// ------------------------------------------------------------
// 组合：人数偏差 / 质心抖动 / 能量基线与突增 / 方向 / 警戒区
// ------------------------------------------------------------
wire[4:0] ppl_d   = ({1'b0, ppl_i} > {1'b0, pplf}) ?
                    ({1'b0, ppl_i} - {1'b0, pplf}) :
                    ({1'b0, pplf} - {1'b0, ppl_i});
wire      ppl_up  = (ppl_i > pplf);
wire      ppl_big = (ppl_d > {1'b0, `PPL_DEADBAND});

// 质心每帧位移（箱）
wire[6:0] ctr_d = (ctr_i > ctr_p) ? (ctr_i - ctr_p) : (ctr_p - ctr_i);
wire      quiet = geom_ok && (ctr_d <= `CTR_QUIET) && (nrg_i >= `NRG_LO);

// 能量基线一阶 EMA
wire[16:0] nbase_n = nrg_base - {3'd0, (nrg_base >> `NRG_BASE_SHIFT)}
                              + {3'd0, (nrg_i    >> `NRG_BASE_SHIFT)};
wire[17:0] nrg_x15 = {1'b0, nrg_base} + {2'd0, (nrg_base >> `NRG_INT_RATIO)};
wire       nrg_upth = ({1'b0, nrg_i} > nrg_x15);                       // 1.5 倍
wire       nrg_abth = (nrg_i > (nrg_base + `NRG_INT_DELTA));           // 绝对增量
wire       nrg_jump = nrg_abth && nrg_upth;
wire[17:0] push_thr = {1'b0, nrg_base} + {`NRG_INT_DELTA, 1'b0};       // base + 2Δ
wire       push_big = ({1'b0, nrg_i} > push_thr);

// 方向窗口
wire[6:0] wspan    = cmax - cmin;
wire      dir_ok   = geom_ok && (wspan >= `DIR_MIN) && (nrg_i >= `NRG_LO);
wire      dir_right= (wlast > wfirst);
wire      push_ok  = geom_ok && push_big && (wspan < `DIR_MIN);
// 基线收敛后才允许判互动（见 vision_def.v 的说明：人数变化会带来
// 能量阶跃，基线需要 NRG_BASE_READY 帧才追上）。
// 注意 pplr_chg 也进组合条件：warm_cnt 的清零是非阻塞赋值，若只用
// warm_cnt == READY 判断，人数阶跃的那一帧看到的还是旧值（32），
// 防护整整晚一拍 —— 而能量跳变恰好就发生在这一拍上。
wire      pplr_chg = (ppl_i != pplr_d);
wire      base_ok  = (warm_cnt == `NRG_BASE_READY) && (~pplr_chg);
wire      hit      = geom_ok && base_ok && ((nrg_jump && dir_ok) || push_ok);

// 警戒区：占用列范围触及画面最外侧 GUARD_EDGE 列
wire[9:0] cf_px = {1'b0, cf_i, 2'b00};          // 箱 -> 列像素（首列）
wire[9:0] cl_px = {1'b0, cl_i, 2'b11};          // 箱 -> 列像素（末列）
wire      guard_hit = (occ_i != 8'd0)
                   && ((cf_px <= `GUARD_EDGE) || (cl_px >= `GUARD_R_LEFT));

// 状态分类
wire busy = (st == `ST_PRESENCE) || (st == `ST_SINGLE) || (st == `ST_MULTI)
         || (st == `ST_INTERACT) || (st == `ST_ALERT);

wire leave_now = busy && (pplf == 4'd0)
              && (leave_cnt == (`LEAVE_FRAMES - 8'd1));

// 评分饱和加/减
wire[8:0] dw_n = {1'b0, sc_dwell}  + {1'b0, `SC_UP};
wire[8:0] mo_n = {1'b0, sc_motion} + {1'b0, `SC_UP};
wire[8:0] qu_n = {1'b0, sc_quiet}  + {1'b0, `SC_UP};
wire      nrg_valid = (nrg_i >= `NRG_LO) && (nrg_i <= `NRG_HI);

// 置信度合成：conf = (sc_sum * 85) >> 8 = 64+16+4+1 倍后右移 8
wire[9:0]  sc_sum = {2'b00, sc_dwell} + {2'b00, sc_motion} + {2'b00, sc_quiet};
wire[16:0] sc85   = ({7'd0, sc_sum} << 6) + ({7'd0, sc_sum} << 4)
                  + ({7'd0, sc_sum} << 2) + {7'd0, sc_sum};

assign st_o    = st;
assign pplf_o  = pplf;
assign conf_o  = sc85[15:8];

// 建议播放模式：状态的纯函数
function[2:0] mode_of;
	input[2:0] s;
	begin
		case(s)
			`ST_SINGLE:   mode_of = `MD_SINGLE;
			`ST_MULTI:    mode_of = `MD_MULTI;
			`ST_INTERACT: mode_of = `MD_INTERACT;
			`ST_ALERT:    mode_of = `MD_ALERT;
			`ST_PRESENCE: mode_of = `MD_SINGLE;
			default:      mode_of = `MD_STANDBY;
		endcase
	end
endfunction

assign mode_o = mode_of(st);

// ------------------------------------------------------------
// 主时序：全部在 frame_tick 上推进
// ------------------------------------------------------------
always@(posedge clk or posedge rst)
begin
	if(rst)
	begin
		st         <= `ST_IDLE;
		pplf       <= 4'd0;
		pdir_cnt   <= 2'd0;
		pdir_up    <= 1'b0;
		enter_cnt  <= 3'd0;
		multi_cnt  <= 3'd0;
		single_cnt <= 3'd0;
		guard_cnt  <= 3'd0;
		unguard_cnt<= 3'd0;
		stable_cnt <= 8'd0;
		leave_cnt  <= 8'd0;
		idle_cnt   <= 10'd0;
		cool_cnt   <= 8'd0;
		ctr_p      <= 7'd0;
		cmin       <= 7'd0;
		cmax       <= 7'd0;
		wfirst     <= 7'd0;
		wlast      <= 7'd0;
		wcnt       <= 5'd0;
		nrg_base   <= 17'd0;
		base_init  <= 1'b0;
		warm_cnt   <= 6'd0;
		pplr_d     <= 4'd0;
		sc_dwell   <= 8'd0;
		sc_motion  <= 8'd0;
		sc_quiet   <= 8'd0;
		dwell_o    <= 16'd0;
		icnt_o     <= 8'd0;
		ev_enter_o <= 1'b0;
		ev_leave_o <= 1'b0;
		ev_interact_o <= 1'b0;
		dir_o      <= 2'd0;
		push_o     <= 1'b0;
	end
	else if(frame_tick == 1'b1)
	begin
		ev_enter_o    <= 1'b0;
		ev_leave_o    <= 1'b0;
		ev_interact_o <= 1'b0;

		// ---------- 人数死区滤波（与状态无关，始终跟随）----------
		if(ppl_big == 1'b1)
		begin
			pplf     <= ppl_i;
			pdir_cnt <= 2'd0;
		end
		else if(ppl_up | (ppl_i < pplf))
		begin
			if(pdir_cnt == 2'd0)
			begin
				pdir_up  <= ppl_up;
				pdir_cnt <= 2'd1;
			end
			else if(pdir_up == ppl_up)
			begin
				if(pdir_cnt == (`PPL_CONFIRM - 4'd1))
				begin
					pplf     <= ppl_i;
					pdir_cnt <= 2'd0;
				end
				else
					pdir_cnt <= pdir_cnt + 2'd1;
			end
			else
			begin
				pdir_up  <= ppl_up;
				pdir_cnt <= 2'd1;
			end
		end
		else
			pdir_cnt <= 2'd0;

		// ---------- 几何坏帧：冻结状态机，只让置信度缓降 ----------
		if(geom_ok == 1'b0)
		begin
			if(sc_dwell  > `SC_DN_SOFT) sc_dwell  <= sc_dwell  - `SC_DN_SOFT; else sc_dwell  <= 8'd0;
			if(sc_motion > `SC_DN_SOFT) sc_motion <= sc_motion - `SC_DN_SOFT; else sc_motion <= 8'd0;
			if(sc_quiet  > `SC_DN_SOFT) sc_quiet  <= sc_quiet  - `SC_DN_SOFT; else sc_quiet  <= 8'd0;
		end
		else
		begin
			// ---------- 无状态依赖的历史量 ----------
			ctr_p <= ctr_i;

			// 能量基线：首个有效帧直接灌初值，之后走 EMA；
			// INTERACT 期间冻结（挥手不能把自己的基线抬上去）
			if(base_init == 1'b0)
			begin
				nrg_base  <= nrg_i;
				base_init <= 1'b1;
			end
			else if(st != `ST_INTERACT)
				nrg_base <= nbase_n;

			// 人数一变，基线收敛计数清零（等基线追上当前场景再判互动）。
			// 注意这里盯的是**原始人数** ppl_i 而不是滤波后的 pplf：
			// 人数阶跃带来的能量跳变是"这一帧就发生"的，而 pplf 还要走
			// PPL_CONFIRM 帧确认——用 pplf 会让防护晚 3 帧生效，正好漏掉
			// 阶跃的那几帧（模型实测：第二个人走进来会被误报成互动）。
			if(ppl_i != pplr_d)
			begin
				warm_cnt <= 6'd0;
				pplr_d   <= ppl_i;
			end
			else
			begin
				if(warm_cnt != `NRG_BASE_READY)
					warm_cnt <= warm_cnt + 6'd1;
			end

			if(wcnt == `DIR_WIN_MAX)
			begin
				cmin   <= ctr_i;
				cmax   <= ctr_i;
				wfirst <= ctr_i;
				wlast  <= ctr_i;
				wcnt   <= 5'd0;
			end
			else
			begin
				if(ctr_i < cmin) cmin <= ctr_i;
				if(ctr_i > cmax) cmax <= ctr_i;
				if(wcnt == 5'd0) wfirst <= ctr_i;
				wlast <= ctr_i;
				wcnt  <= wcnt + 5'd1;
			end

			// ---------- 停留计时 + 离开判定（占用态通用）----------
			if(busy == 1'b1)
				dwell_o <= dwell_o + 16'd1;

			if(leave_now == 1'b1)
			begin
				st          <= `ST_LEAVE;
				ev_leave_o  <= 1'b1;
				leave_cnt   <= 8'd0;
				idle_cnt    <= 10'd0;
				stable_cnt  <= 8'd0;
				guard_cnt   <= 3'd0;
				unguard_cnt <= 3'd0;
			end
			else if((busy == 1'b1) && (pplf == 4'd0))
				leave_cnt <= leave_cnt + 8'd1;
			else if(busy == 1'b1)
				leave_cnt <= 8'd0;

			// ---------- 状态跃迁 ----------
			if(leave_now == 1'b0)
			begin
				case(st)
					// ---------- 无人 ----------
					`ST_IDLE:
					begin
						dwell_o <= 16'd0;
						if(pplf >= 4'd1)
						begin
							if(enter_cnt == (`EVT_CONFIRM - 3'd1))
							begin
								st         <= `ST_PRESENCE;
								ev_enter_o <= 1'b1;
								enter_cnt  <= 3'd0;
								stable_cnt <= 8'd0;
							end
							else
								enter_cnt <= enter_cnt + 3'd1;
						end
						else
							enter_cnt <= 3'd0;
					end

					// ---------- 有人，等站定 ----------
					`ST_PRESENCE:
					begin
						if(pplf >= 4'd2)
						begin
							if(multi_cnt == (`EVT_CONFIRM - 3'd1))
							begin
								st         <= `ST_MULTI;
								multi_cnt  <= 3'd0;
								stable_cnt <= 8'd0;
							end
							else
								multi_cnt <= multi_cnt + 3'd1;
						end
						else
						begin
							multi_cnt <= 3'd0;
							if(quiet == 1'b1)
							begin
								if(stable_cnt == (`STABLE_FRAMES - 8'd1))
								begin
									st         <= `ST_SINGLE;
									stable_cnt <= 8'd0;
								end
								else
									stable_cnt <= stable_cnt + 8'd1;
							end
							else
								stable_cnt <= 8'd0;
						end
					end

					// ---------- 已确认有人（1 人或多人）----------
					`ST_SINGLE, `ST_MULTI:
					begin
						if(guard_hit == 1'b1)
						begin
							if(guard_cnt == (`EVT_CONFIRM - 3'd1))
							begin
								st          <= `ST_ALERT;
								guard_cnt   <= 3'd0;
								unguard_cnt <= 3'd0;
							end
							else
								guard_cnt <= guard_cnt + 3'd1;
						end
						else
							guard_cnt <= 3'd0;

						if(st == `ST_SINGLE)
						begin
							if(pplf >= 4'd2)
							begin
								if(multi_cnt == (`EVT_CONFIRM - 3'd1))
								begin
									st        <= `ST_MULTI;
									multi_cnt <= 3'd0;
								end
								else
									multi_cnt <= multi_cnt + 3'd1;
							end
							else
								multi_cnt <= 3'd0;
						end
						else
						begin
							if(pplf <= 4'd1)
							begin
								if(single_cnt == (`EVT_CONFIRM - 3'd1))
								begin
									st         <= `ST_SINGLE;
									single_cnt <= 3'd0;
								end
								else
									single_cnt <= single_cnt + 3'd1;
							end
							else
								single_cnt <= 3'd0;
						end

						// 互动优先级最低：命中就进 INTERACT
						if(hit == 1'b1)
						begin
							st            <= `ST_INTERACT;
							ev_interact_o <= 1'b1;
							icnt_o        <= (icnt_o == 8'hFF) ? 8'hFF : (icnt_o + 8'd1);
							cool_cnt      <= 8'd0;
							stable_cnt    <= 8'd0;
							// 方向负载（M6 的 EVT_INTERACT 要用）：
							// 有横向位移 → 左右；纯能量突增 → 前推
							dir_o  <= (dir_ok == 1'b1) ? (dir_right ? 2'd2 : 2'd1) : 2'd3;
							push_o <= (~dir_ok) & push_ok;
						end
					end

					// ---------- 互动中（驻留期 = 冷却期）----------
					`ST_INTERACT:
					begin
						if(cool_cnt == (`COOL_FRAMES - 8'd1))
						begin
							cool_cnt   <= 8'd0;
							stable_cnt <= 8'd0;
							st         <= (pplf >= 4'd2) ? `ST_MULTI : `ST_SINGLE;
						end
						else
							cool_cnt <= cool_cnt + 8'd1;
					end

					// ---------- 刚离开 ----------
					`ST_LEAVE:
					begin
						if(pplf >= 4'd1)
						begin
							st         <= `ST_PRESENCE;
							ev_enter_o <= 1'b1;
							dwell_o    <= 16'd0;
							stable_cnt <= 8'd0;
							idle_cnt   <= 10'd0;
						end
						else if(idle_cnt == (`IDLE_HOLD_FRAMES - 10'd1))
						begin
							st       <= `ST_IDLE;
							idle_cnt <= 10'd0;
						end
						else
							idle_cnt <= idle_cnt + 10'd1;
					end

					// ---------- 警戒 ----------
					`ST_ALERT:
					begin
						if(guard_hit == 1'b0)
						begin
							if(unguard_cnt == (`EVT_CONFIRM - 3'd1))
							begin
								unguard_cnt <= 3'd0;
								st          <= (pplf >= 4'd2) ? `ST_MULTI : `ST_SINGLE;
							end
							else
								unguard_cnt <= unguard_cnt + 3'd1;
						end
						else
							unguard_cnt <= 3'd0;
					end

					default:
						st <= `ST_IDLE;
				endcase
			end

			// ---------- 置信度评分积分 ----------
			if(busy == 1'b1)
			begin
				sc_dwell <= (dw_n[8] == 1'b1) ? 8'hFF : dw_n[7:0];
			end
			else
			begin
				if(sc_dwell > `SC_DN_HARD) sc_dwell <= sc_dwell - `SC_DN_HARD;
				else                       sc_dwell <= 8'd0;
			end

			if(nrg_valid == 1'b1)
			begin
				sc_motion <= (mo_n[8] == 1'b1) ? 8'hFF : mo_n[7:0];
			end
			else
			begin
				if(sc_motion > `SC_DN_HARD) sc_motion <= sc_motion - `SC_DN_HARD;
				else                        sc_motion <= 8'd0;
			end

			if(quiet == 1'b1)
			begin
				sc_quiet <= (qu_n[8] == 1'b1) ? 8'hFF : qu_n[7:0];
			end
			else
			begin
				if(sc_quiet > `SC_DN_SOFT) sc_quiet <= sc_quiet - `SC_DN_SOFT;
				else                       sc_quiet <= 8'd0;
			end
		end
	end
end

endmodule
