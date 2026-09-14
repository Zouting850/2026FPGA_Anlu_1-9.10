// ============================================================
// mt9v034_cfg.v
// MT9V034 上电配置序列发生器
//
// 交互：本模块发出 sccb_req，由 sccb_master 执行 16 位寄存器读写，
// 完成后 sccb_master 回一个 ack 脉冲。
//
// 时序：请求 → 等 sccb_busy 拉高（已被接受）→ 撤请求 → 等 ack
//       → 保持写间隔 → 下一条
//
// 流程：上电等待
//       → 【极性探测】用极性 0 单次读 R0x00；不是 0x1324 就换极性 1 再读一次
//       → 探测成功才继续：逐条写配置表 → 稳定等待 → 再读 R0x00 校验
//       → 版本 = 0x1324 → cam_ok=1；否则重写配置表，最多 3 次
//       → 探测两次都失败 → cam_ok=0（线路/上拉/供电问题，不是接反）
//       → 【自动重试】cam_ok=0 时每 PROBE_RETRY_CNT（2s）自动重走一遍探测，
//         现场改完接线/补完上拉只需等 2 秒，不必重新烧 bit
//
// 【诊断输出】ver_rd0 = 极性 0 那一次的读值，ver_rd = 最后一次的读值。
//   两个摆在一起就能分辨"哪根线读全 1、哪根读全 0"：
//     正常/接反 → 两者都能读到 0x1324（只是 swap 不同）
//     都不通    → 例如 ver_rd0=0xFFFF、ver_rd=0x0000，说明一根悬空、一根被拉低
//   这一对值是现场区分"断线 / 缺上拉 / 短路 / 改造没通"的关键证据。
//
// 【为什么要先探测极性】
//   改造后的总钻风把 CMOS 的 SCCB 两线引到 FFC 的 TXD/RXD，转接板 V3.1 又原样
//   透传到 P1-5/P1-7。逐飞的手册只说"CMOS 的 IIC 引脚将直接与 FFC 连通"，
//   **没有说明哪一根是 SCL**。实物上这是个 50/50 的赌注，接反的现象极有迷惑性：
//     总线毫无应答 → 读版本号得到 FFFF → 上电默认(752x480, AEC 开)原样保留
//     → 而 DVP 的 11 根线照常工作，串口上看起来"图像数据在动、只有两项不变"。
//   与其让用户去赌/去改线，不如让 FPGA 自己试出来：两种极性各读一次 R0x00，
//   谁回 0x1324 就用谁，并把结果输出到 sccb_swap 锁定、由状态行报告。
//
//   探测刻意做成"只发一次读事务"（约 0.6 ms），而不是跑整表：
//     * 接反时时钟会打到 CMOS 的 SDA 上，虽然 CMOS 的 SDA 是开漏、多半不会被
//       我们驱高而打架，但仍然越短越安全；
//     * 失败的那一次不写任何寄存器，不会把传感器改坏。
//
// 【为什么写完还要再读一次校验】
//   探测已经证明了总线通，但"总线通"不等于"每条写都落到了"。写完再读一次
//   R0x00 作为端到端校验；不匹配就整表重写（最多 3 次），仍失败则 cam_ok=0。
//
// 说明：M1 阶段保留传感器的 AEC/AGC 自动曝光，先拿到可用图像；
//       M4 再把 R0xAF 写 0x0000 关闭 AEC/AGC，改由 FPGA 闭环回写
//       R0x0B（粗曝光总量）/ R0x35（模拟增益）。
// ============================================================
`include "vision_def.v"

module mt9v034_cfg
(
	input                       clk,
	input                       rst,        // 高有效
	input                       restart,    // 拉高一拍重新配置（含重新探测极性）
	// ---- 接 sccb_master ----
	output reg                  sccb_req,
	output reg                  sccb_rw,
	output reg[15:0]            sccb_addr,
	output reg[15:0]            sccb_wdata,
	input                       sccb_busy,
	input                       sccb_ack,
	input[15:0]                 sccb_rdata,
	output reg                  sccb_swap,  // 探测出的两线极性，直接接 sccb_master.swap
	// ---- 状态 ----
	output reg                  cfg_busy,
	output reg                  cfg_done,
	output reg                  cam_ok,
	output reg[15:0]            ver_rd,     // 最后一次读回的 R0x00
	output reg[15:0]            ver_rd0,    // 【诊断】第一次（极性 0）探测读到的值
	                                        //   排障用：和 ver_rd 摆在一起就能看出
	                                        //   "哪根线读全 1、哪根读全 0"。
	                                        //   两根线都正常时应与 ver_rd 同为 0x1324。
	output reg[3:0]             wr_idx,
	output reg[3:0]             err_cnt     // 版本校验失败次数
);

localparam C_PWRWAIT = 5'd0;
localparam C_PR_GAP  = 5'd1;    // 两次极性探测之间的静默
localparam C_PR_SETUP= 5'd2;    // 探测：发起读 R0x00
localparam C_PR_BUSY = 5'd3;
localparam C_PR_ACK  = 5'd4;
localparam C_PR_CHK  = 5'd5;
localparam C_WRSETUP = 5'd6;
localparam C_WRBUSY  = 5'd7;
localparam C_WRACK   = 5'd8;
localparam C_WRGAP   = 5'd9;
localparam C_SETTLE  = 5'd10;
localparam C_RDSETUP = 5'd11;
localparam C_RDBUSY  = 5'd12;
localparam C_RDACK   = 5'd13;
localparam C_CHECK   = 5'd14;
localparam C_DONE    = 5'd15;

reg[4:0]  st;
reg[31:0] wait_cnt;
reg[3:0]  retry_cnt;
reg       trial;        // 正在试探的极性：0 = 先试"正常接法"，1 = 再试"接反"

// ------------------------------------------------------------
// 配置表：序号 → 寄存器地址 / 写入值
// ------------------------------------------------------------
function[15:0] cfg_addr_of;
	input[3:0] i;
	begin
		case(i)
			4'd0: cfg_addr_of = `MT_REG_RESET;      // 0x0C
			4'd1: cfg_addr_of = `MT_REG_CHIPCTRL;   // 0x07
			4'd2: cfg_addr_of = `MT_REG_COLSTART;   // 0x01
			4'd3: cfg_addr_of = `MT_REG_ROWSTART;   // 0x02
			4'd4: cfg_addr_of = `MT_REG_WINHEIGHT;  // 0x03
			4'd5: cfg_addr_of = `MT_REG_WINWIDTH;   // 0x04
			4'd6: cfg_addr_of = `MT_REG_READMODE;   // 0x0D
			4'd7: cfg_addr_of = `MT_REG_AECAGC;     // 0xAF
			default: cfg_addr_of = 16'h0000;
		endcase
	end
endfunction

function[15:0] cfg_data_of;
	input[3:0] i;
	begin
		case(i)
			4'd0: cfg_data_of = `MT_VAL_RESET;
			4'd1: cfg_data_of = `MT_VAL_CHIPCTRL;
			4'd2: cfg_data_of = `MT_VAL_COLSTART;
			4'd3: cfg_data_of = `MT_VAL_ROWSTART;
			4'd4: cfg_data_of = `MT_VAL_WINHEIGHT;
			4'd5: cfg_data_of = `MT_VAL_WINWIDTH;
			4'd6: cfg_data_of = `MT_VAL_READMODE;
			4'd7: cfg_data_of = `MT_VAL_AECAGC;
			default: cfg_data_of = 16'h0000;
		endcase
	end
endfunction

always@(posedge clk or posedge rst)
begin
	if(rst)
	begin
		st         <= C_PWRWAIT;
		wait_cnt   <= 32'd0;
		retry_cnt  <= 4'd0;
		trial      <= 1'b0;
		sccb_req   <= 1'b0;
		sccb_rw    <= 1'b0;
		sccb_addr  <= 16'd0;
		sccb_wdata <= 16'd0;
		sccb_swap  <= 1'b0;
		cfg_busy   <= 1'b1;
		cfg_done   <= 1'b0;
		cam_ok     <= 1'b0;
		ver_rd     <= 16'd0;
		ver_rd0    <= 16'd0;
		wr_idx     <= 4'd0;
		err_cnt    <= 4'd0;
	end
	else
	begin
		case(st)
			// ------------------------------------------------
			// 上电/复位后等传感器内部稳定
			// 探测从"极性 0 = 正常接法"开始
			// ------------------------------------------------
			C_PWRWAIT:
			begin
				cfg_busy <= 1'b1;
				cfg_done <= 1'b0;
				cam_ok   <= 1'b0;
				if(wait_cnt == (`RESET_WAIT_CNT - 32'd1))
				begin
					wait_cnt  <= 32'd0;
					wr_idx    <= 4'd0;
					trial     <= 1'b0;
					sccb_swap <= 1'b0;      // 先按"正常接法"探测
					st        <= C_PR_SETUP;
				end
				else
					wait_cnt <= wait_cnt + 32'd1;
			end

			// ------------------------------------------------
			// 两次极性探测之间的静默（让总线彻底回到空闲）
			// ------------------------------------------------
			C_PR_GAP:
				if(wait_cnt == (`PROBE_GAP_CNT - 32'd1))
				begin
					wait_cnt <= 32'd0;
					st       <= C_PR_SETUP;
				end
				else
					wait_cnt <= wait_cnt + 32'd1;

			// ------------------------------------------------
			// 极性探测：只发一条"读 R0x00"
			// ------------------------------------------------
			C_PR_SETUP:
			begin
				sccb_addr  <= `MT_REG_VERSION;
				sccb_wdata <= 16'd0;
				sccb_rw    <= 1'b1;
				sccb_req   <= 1'b1;
				st         <= C_PR_BUSY;
			end

			C_PR_BUSY:
			if(sccb_busy == 1'b1)
			begin
				sccb_req <= 1'b0;
				st       <= C_PR_ACK;
			end

			C_PR_ACK:
			if(sccb_ack == 1'b1)
			begin
				ver_rd <= sccb_rdata;
				// 只留住"极性 0"那一次：它是"按标注接线时这根线读到什么"的直接证据
				if(trial == 1'b0)
					ver_rd0 <= sccb_rdata;
				st     <= C_PR_CHK;
			end

			C_PR_CHK:
			begin
				if(ver_rd == `MT_VER_EXPECT)
				begin
					// 这一种极性通了 → 锁定（sccb_swap 保持 trial 的值），开始写配置表
					sccb_swap <= trial;
					cfg_busy  <= 1'b1;
					wr_idx    <= 4'd0;
					wait_cnt  <= 32'd0;
					st        <= C_WRSETUP;
				end
				else if(trial == 1'b0)
				begin
					// 正常接法不通 → 换极性 1（实物两线接反时走这条路）
					trial     <= 1'b1;
					sccb_swap <= 1'b1;
					wait_cnt  <= 32'd0;
					st        <= C_PR_GAP;
				end
				else
				begin
					// 两种极性都读不到 0x1324 → 不是接反，去查线路/上拉/供电。
					// 状态行会打出 F，并用 V（第二次读）与 P0（第一次读）把
					// 两根线各自的静态电平暴露出来。进入 C_DONE 时清零 wait_cnt，
					// 让自动重试从整段延时开始计。
					cam_ok   <= 1'b0;
					cfg_busy <= 1'b0;
					cfg_done <= 1'b1;
					wait_cnt <= 32'd0;
					st       <= C_DONE;
				end
			end

			// ------------------------------------------------
			// 发起一条写
			// ------------------------------------------------
			C_WRSETUP:
			begin
				sccb_addr  <= cfg_addr_of(wr_idx);
				sccb_wdata <= cfg_data_of(wr_idx);
				sccb_rw    <= 1'b0;
				sccb_req   <= 1'b1;
				st         <= C_WRBUSY;
			end

			// 等 sccb_master 接单（busy 拉高即为已接受）
			C_WRBUSY:
			if(sccb_busy == 1'b1)
			begin
				sccb_req <= 1'b0;
				st       <= C_WRACK;
			end

			// 等事务结束
			C_WRACK:
			if(sccb_ack == 1'b1)
			begin
				wait_cnt <= 32'd0;
				st       <= C_WRGAP;
			end

			// 写间隔：复位后的第一条用长延时，其余用短间隔
			C_WRGAP:
			begin
				if(wait_cnt == ((wr_idx == 4'd0)
						? (`RESET_WAIT_CNT - 32'd1)
						: (`WRITE_GAP_CNT  - 32'd1)))
				begin
					wait_cnt <= 32'd0;
					if(wr_idx == (`CFG_WR_NUM - 4'd1))
						st <= C_SETTLE;
					else
					begin
						wr_idx <= wr_idx + 4'd1;
						st     <= C_WRSETUP;
					end
				end
				else
					wait_cnt <= wait_cnt + 32'd1;
			end

			// ------------------------------------------------
			// 配置生效等待
			// ------------------------------------------------
			C_SETTLE:
				if(wait_cnt == (`SETTLE_WAIT_CNT - 32'd1))
				begin
					wait_cnt <= 32'd0;
					st       <= C_RDSETUP;
				end
				else
					wait_cnt <= wait_cnt + 32'd1;

			// ------------------------------------------------
			// 读完再校验一次（端到端）
			// ------------------------------------------------
			C_RDSETUP:
			begin
				sccb_addr  <= `MT_REG_VERSION;
				sccb_wdata <= 16'd0;
				sccb_rw    <= 1'b1;
				sccb_req   <= 1'b1;
				st         <= C_RDBUSY;
			end

			C_RDBUSY:
			if(sccb_busy == 1'b1)
			begin
				sccb_req <= 1'b0;
				st       <= C_RDACK;
			end

			C_RDACK:
			if(sccb_ack == 1'b1)
			begin
				ver_rd <= sccb_rdata;
				st     <= C_CHECK;
			end

			// ------------------------------------------------
			// 校验 / 重试
			// 极性已被探测证明，所以重试只重写配置表，不再重新探测
			// ------------------------------------------------
			C_CHECK:
			begin
				if(ver_rd == `MT_VER_EXPECT)
				begin
					cam_ok   <= 1'b1;
					cfg_busy <= 1'b0;
					cfg_done <= 1'b1;
					st       <= C_DONE;
				end
				else
				begin
					if(err_cnt != 4'hF)
						err_cnt <= err_cnt + 4'd1;

					if(retry_cnt == 4'd2)        // 已试满 3 次
					begin
						cam_ok   <= 1'b0;
						cfg_busy <= 1'b0;
						cfg_done <= 1'b1;
						wait_cnt <= 32'd0;       // 自动重试的延时从这里开始计
						st       <= C_DONE;
					end
					else
					begin
						retry_cnt <= retry_cnt + 4'd1;
						wr_idx    <= 4'd0;
						wait_cnt  <= 32'd0;
						st        <= C_WRSETUP;
					end
				end
			end

			// ------------------------------------------------
			C_DONE:
			begin
				cfg_done <= 1'b1;
				if(restart == 1'b1)
				begin
					cfg_done  <= 1'b0;
					cfg_busy  <= 1'b1;
					cam_ok    <= 1'b0;
					retry_cnt <= 4'd0;
					wr_idx    <= 4'd0;
					wait_cnt  <= 32'd0;
					trial     <= 1'b0;
					sccb_swap <= 1'b0;
					ver_rd0   <= 16'd0;
					// 重新走一遍极性探测：允许用户换过线之后不改 bit 直接重试
					st        <= C_PWRWAIT;
				end
				else if(cam_ok == 1'b0)
				begin
					// 【自动重试】两种极性都不通（cam_ok=0）时，每 PROBE_RETRY_CNT
					// 自动重走一遍完整探测（含极性重探）。现场排障时"改接线 → 复位
					// 重试"是最慢的一环，有了这条就变成"改完等 2 秒"。
					// 探测成功后不再重试；一帧图像采集照常，不受影响。
					if(wait_cnt == (`PROBE_RETRY_CNT - 32'd1))
					begin
						cfg_done  <= 1'b0;
						cfg_busy  <= 1'b1;
						cam_ok    <= 1'b0;
						retry_cnt <= 4'd0;
						wr_idx    <= 4'd0;
						wait_cnt  <= 32'd0;
						trial     <= 1'b0;
						sccb_swap <= 1'b0;
						ver_rd0   <= 16'd0;
						st        <= C_PWRWAIT;
					end
					else
						wait_cnt <= wait_cnt + 32'd1;
				end
				else
					wait_cnt <= 32'd0;
			end

			default:
				st <= C_PWRWAIT;
		endcase
	end
end

endmodule
