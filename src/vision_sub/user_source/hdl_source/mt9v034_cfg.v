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
// 流程：上电等待 → 逐条写配置表 → 稳定等待 → 读 R0x00 校验
//       → 版本 = 0x1324 → cam_ok=1；否则整表重试，最多 3 次
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
	input                       restart,    // 拉高一拍重新配置
	// ---- 接 sccb_master ----
	output reg                  sccb_req,
	output reg                  sccb_rw,
	output reg[15:0]            sccb_addr,
	output reg[15:0]            sccb_wdata,
	input                       sccb_busy,
	input                       sccb_ack,
	input[15:0]                 sccb_rdata,
	// ---- 状态 ----
	output reg                  cfg_busy,
	output reg                  cfg_done,
	output reg                  cam_ok,
	output reg[15:0]            ver_rd,     // 读回的 R0x00
	output reg[3:0]             wr_idx,
	output reg[3:0]             err_cnt     // 版本校验失败次数
);

localparam C_PWRWAIT = 4'd0;
localparam C_WRSETUP = 4'd1;
localparam C_WRBUSY  = 4'd2;
localparam C_WRACK   = 4'd3;
localparam C_WRGAP   = 4'd4;
localparam C_SETTLE  = 4'd5;
localparam C_RDSETUP = 4'd6;
localparam C_RDBUSY  = 4'd7;
localparam C_RDACK   = 4'd8;
localparam C_CHECK   = 4'd9;
localparam C_DONE    = 4'd10;

reg[3:0]  st;
reg[31:0] wait_cnt;
reg[3:0]  retry_cnt;

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
		sccb_req   <= 1'b0;
		sccb_rw    <= 1'b0;
		sccb_addr  <= 16'd0;
		sccb_wdata <= 16'd0;
		cfg_busy   <= 1'b1;
		cfg_done   <= 1'b0;
		cam_ok     <= 1'b0;
		ver_rd     <= 16'd0;
		wr_idx     <= 4'd0;
		err_cnt    <= 4'd0;
	end
	else
	begin
		case(st)
			// ------------------------------------------------
			// 上电/复位后等传感器内部稳定
			// ------------------------------------------------
			C_PWRWAIT:
			begin
				cfg_busy <= 1'b1;
				cfg_done <= 1'b0;
				cam_ok   <= 1'b0;
				if(wait_cnt == (`RESET_WAIT_CNT - 32'd1))
				begin
					wait_cnt <= 32'd0;
					wr_idx   <= 4'd0;
					st       <= C_WRSETUP;
				end
				else
					wait_cnt <= wait_cnt + 32'd1;
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
			// 读 R0x00 校验版本
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
						st       <= C_DONE;
					end
					else
					begin
						retry_cnt <= retry_cnt + 4'd1;
						wr_idx    <= 4'd0;
						wait_cnt  <= 32'd0;
						st        <= C_PWRWAIT;
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
					st        <= C_PWRWAIT;
				end
			end

			default:
				st <= C_PWRWAIT;
		endcase
	end
end

endmodule
