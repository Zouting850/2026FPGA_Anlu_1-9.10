// ============================================================
// sccb_master.v
// MT9V034 两线串行（SCCB / I2C 兼容）主机
//
// 协议要点（MT9V034 数据手册 Rev.A，Table 6 与 Two-Wire Serial Interface）：
//   - 寄存器地址 16 位、数据 16 位，先发地址高字节再发低字节
//   - 从机地址字节已含 R/W 位：写 0x90、读 0x91
//     （{S_CTRL_ADR1, S_CTRL_ADR0} = 00 时的取值）
//   - 写事务：START, dev_w, RA_H, RA_L, D_H, D_L, STOP
//   - 读事务：START, dev_w, RA_H, RA_L, RESTART, dev_r, D_H, D_L, STOP
//     读事务中主机是接收方：D_H 由**主机主动回 ACK**，D_L 由主机回 NACK
//     （若 D_H 也回 NACK，MT9V034 会释放 SDA，D_L 读成 0xFF → 版本号错误）
//
// 时序实现：
//   - 每个 SCL 周期 = 2 个 half，每个 half = SCCB_DIV_NUM 个 clk
//   - 发送字节：低半周期摆数据、高半周期为有效期
//   - 接收字节：低半周期释放 SDA（sda_oe = 0），高半周期起始采样
//   - STOP 用 4 个半周期节拍完成，确保 SDA 在 SCL 高电平期间上升
//
// ACK 策略：从机 NACK 只计数、不阻塞。MT9V034 在内部寄存器更新窗口内可能
// 不回 ACK，若把 NACK 当致命错误，配置流程会随机卡死。NACK 次数由
// nack_cnt 输出供上层观察。
// ============================================================
`include "vision_def.v"

module sccb_master
(
	input                       clk,
	input                       rst,
	// ---- 请求侧（sys 域）----
	input                       req,        // 拉高一拍即发起一次事务
	input                       rw,         // 1'b0 写 / 1'b1 读
	input[15:0]                 reg_addr,
	input[15:0]                 wr_data,
	output                      busy,
	output                      ack,        // 事务结束的一个周期脉冲
	output[15:0]                rd_data,
	output[4:0]                 nack_cnt,   // 累计 NACK 次数（诊断用）
	// ---- SCCB 物理接口 ----
	output                      scl,
	inout                       sda
);

localparam S_IDLE    = 3'd0;
localparam S_START   = 3'd1;
localparam S_BIT     = 3'd2;
localparam S_ACK     = 3'd3;
localparam S_RESTART = 3'd4;
localparam S_STOP    = 3'd5;
localparam S_END     = 3'd6;

reg[2:0]                     st;
reg[2:0]                     bitc;
reg[3:0]                     byte_idx;
reg[15:0]                    div;
reg                          half;       // 0 = 低半周期, 1 = 高半周期
reg[7:0]                     sh;         // 发送移位寄存器
reg[7:0]                     rx_sh;      // 接收移位寄存器
reg                          scl_r;
reg                          sda_oe;
reg                          sda_out;
reg[15:0]                    ra_r;
reg[15:0]                    wd_r;
reg[15:0]                    rd_r;
reg                          rw_r;
reg[4:0]                     nack_c;
reg                          busy_r;
reg                          ack_r;

wire                         sda_in;
wire                         tick;
wire                         rx_byte;    // 当前字节为"接收"方向
wire                         nack_byte;  // 当前字节需主机回 NACK

assign sda    = sda_oe ? sda_out : 1'bz;
assign sda_in = sda;
assign scl    = scl_r;

assign busy      = busy_r;
assign ack       = ack_r;
assign rd_data   = rd_r;
assign nack_cnt  = nack_c;
assign tick      = (div == (`SCCB_DIV_NUM - 1));
assign rx_byte   = (rw_r == 1'b1) && (byte_idx >= 4'd4);
// 读事务里主机是**接收方**，因此：
//   byte_idx == 4（D_H）：主机必须主动回 ACK（拉低 SDA），否则从机不会继续发 D_L；
//   byte_idx == 5（D_L）：主机回 NACK（释放/拉高），表示读结束。
// 写事务与地址字节则释放总线，等从机 ACK。
assign mack_byte = (rw_r == 1'b1) && (byte_idx == 4'd4);
assign nack_byte = (rw_r == 1'b1) && (byte_idx == 4'd5);

// ------------------------------------------------------------
// 字节序号 → 该字节内容（接收字节时 SDA 由从机驱动，取值无关）
// ------------------------------------------------------------
function[7:0] tx_byte_of;
	input[3:0] idx;
	begin
		case(idx)
			4'd0:    tx_byte_of = `SCCB_DEV_ADDR;
			4'd1:    tx_byte_of = ra_r[15:8];
			4'd2:    tx_byte_of = ra_r[7:0];
			4'd3:    tx_byte_of = (rw_r == 1'b1) ? (`SCCB_DEV_ADDR | 8'h01) : wd_r[15:8];
			4'd4:    tx_byte_of = (rw_r == 1'b1) ? 8'h00 : wd_r[7:0];
			default: tx_byte_of = 8'h00;
		endcase
	end
endfunction

always@(posedge clk or posedge rst)
begin
	if(rst)
	begin
		st       <= S_IDLE;
		bitc     <= 3'd0;
		byte_idx <= 4'd0;
		div      <= 16'd0;
		half     <= 1'b0;
		sh       <= 8'd0;
		rx_sh    <= 8'd0;
		scl_r    <= 1'b1;
		sda_oe   <= 1'b1;
		sda_out  <= 1'b1;
		ra_r     <= 16'd0;
		wd_r     <= 16'd0;
		rd_r     <= 16'd0;
		rw_r     <= 1'b0;
		nack_c   <= 5'd0;
		busy_r   <= 1'b0;
		ack_r    <= 1'b0;
	end
	else
	begin
		ack_r <= 1'b0;

		if(st != S_IDLE)
		begin
			if(tick)
				div <= 16'd0;
			else
				div <= div + 16'd1;
		end

		case(st)
			// ------------------------------------------------
			S_IDLE:
			begin
				scl_r   <= 1'b1;
				sda_oe  <= 1'b1;
				sda_out <= 1'b1;
				div     <= 16'd0;
				half    <= 1'b0;
				if(req == 1'b1)
				begin
					ra_r     <= reg_addr;
					wd_r     <= wr_data;
					rw_r     <= rw;
					byte_idx <= 4'd0;
					bitc     <= 3'd0;
					busy_r   <= 1'b1;
					st       <= S_START;
				end
			end

			// ---- START：SCL 保持高，SDA 由高变低 ----
			S_START:
			if(tick)
			begin
				half <= ~half;
				if(half == 1'b0)
				begin
					scl_r   <= 1'b1;
					sda_oe  <= 1'b1;
					sda_out <= 1'b1;
				end
				else
				begin
					sda_out <= 1'b0;
					sh      <= tx_byte_of(4'd0);
					bitc    <= 3'd0;
					st      <= S_BIT;
				end
			end

			// ---- 数据位 ----
			S_BIT:
			if(tick)
			begin
				half <= ~half;
				if(half == 1'b0)
				begin
					scl_r <= 1'b0;
					if(rx_byte == 1'b1)
						sda_oe <= 1'b0;
					else
					begin
						sda_oe  <= 1'b1;
						sda_out <= sh[7];
					end
				end
				else
				begin
					scl_r <= 1'b1;
					rx_sh <= {rx_sh[6:0], sda_in};
					if(bitc == 3'd7)
					begin
						bitc <= 3'd0;
						if(rx_byte == 1'b1)
						begin
							if(byte_idx == 4'd4)
								rd_r[15:8] <= {rx_sh[6:0], sda_in};
							else if(byte_idx == 4'd5)
								rd_r[7:0]  <= {rx_sh[6:0], sda_in};
						end
						st <= S_ACK;
					end
					else
					begin
						bitc <= bitc + 3'd1;
						if(rx_byte == 1'b0)
							sh <= {sh[6:0], 1'b0};
					end
				end
			end

			// ---- 应答位 ----
			S_ACK:
			if(tick)
			begin
				half <= ~half;
				if(half == 1'b0)
				begin
					scl_r <= 1'b0;
					if(nack_byte == 1'b1)
					begin
						sda_oe  <= 1'b1;
						sda_out <= 1'b1;      // 主机 NACK（读最后一字节）
					end
					else if(mack_byte == 1'b1)
					begin
						sda_oe  <= 1'b1;
						sda_out <= 1'b0;      // 主机主动 ACK（读第一字节）
					end
					else
						sda_oe <= 1'b0;       // 写/地址字节：释放总线，等从机拉低
				end
				else
				begin
					scl_r <= 1'b1;
					if((sda_in == 1'b1) && (nack_byte == 1'b0) && (mack_byte == 1'b0))
					begin
						if(nack_c != 5'd31)
							nack_c <= nack_c + 5'd1;
					end
					sda_oe  <= 1'b1;
					sda_out <= 1'b1;

					if(rw_r == 1'b0)
					begin
						// 写事务共 5 字节：0 dev, 1 RA_H, 2 RA_L, 3 D_H, 4 D_L
						if(byte_idx == 4'd4)
							st <= S_STOP;
						else
						begin
							byte_idx <= byte_idx + 4'd1;
							sh       <= tx_byte_of(byte_idx + 4'd1);
							bitc     <= 3'd0;
							st       <= S_BIT;
						end
					end
					else
					begin
						// 读事务：0/1/2 地址 → RESTART → 3 读地址, 4 D_H, 5 D_L
						if(byte_idx == 4'd2)
							st <= S_RESTART;
						else if(byte_idx == 4'd5)
							st <= S_STOP;
						else
						begin
							byte_idx <= byte_idx + 4'd1;
							sh       <= tx_byte_of(byte_idx + 4'd1);
							bitc     <= 3'd0;
							st       <= S_BIT;
						end
					end
				end
			end

			// ---- 重复起始：SCL 高，SDA 由高变低 ----
			S_RESTART:
			if(tick)
			begin
				half <= ~half;
				if(half == 1'b0)
				begin
					scl_r   <= 1'b1;
					sda_oe  <= 1'b1;
					sda_out <= 1'b1;
				end
				else
				begin
					sda_out  <= 1'b0;
					byte_idx <= 4'd3;
					sh       <= `SCCB_DEV_ADDR | 8'h01;
					bitc     <= 3'd0;
					st       <= S_BIT;
				end
			end

			// ---- STOP：必须在 SCL 为高时让 SDA 由低变高 ----
			// 分 4 个半周期节拍依次完成，避免 SCL/SDA 同时跳变被从机
			// 误判成 START，也保证 STOP 条件真正成立：
			//   0: SCL 拉低           1: SCL 低时 SDA 拉低
			//   2: SCL 拉高(SDA 仍低) 3: SCL 高时 SDA 拉高 = STOP
			S_STOP:
			if(tick)
			begin
				sda_oe <= 1'b1;
				case(bitc)
					3'd0: begin scl_r  <= 1'b0; sda_out <= 1'b1; bitc <= 3'd1; end
					3'd1: begin                  sda_out <= 1'b0; bitc <= 3'd2; end
					3'd2: begin scl_r  <= 1'b1; sda_out <= 1'b0; bitc <= 3'd3; end
					3'd3: begin                  sda_out <= 1'b1; bitc <= 3'd4; end
					default:
					begin
						bitc <= 3'd0;
						st   <= S_END;
					end
				endcase
			end

			// ---- 结束 ----
			S_END:
			begin
				ack_r  <= 1'b1;
				busy_r <= 1'b0;
				scl_r  <= 1'b1;
				st     <= S_IDLE;
			end

			default:
				st <= S_IDLE;
		endcase
	end
end

endmodule
