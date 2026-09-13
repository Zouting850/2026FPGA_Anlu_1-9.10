// ============================================================
// dbg_uart.v
// 115200-8N1 串口收发（调试用，复用板载 CH340：FPGA TX -> D12）
//
//   - 单纯 TX 用于打印帧指纹 / 自检结果
//   - RX 用于接收单字节命令（M1 预留：'r' 复位统计、's' 快照）
//   - 波特率分频由 vision_def.v 的 UART_BAUD_DIV 决定（50MHz/115200=434）
//
// 发送：tx_req 拉高一拍即开始；tx_busy 期间忽略新请求。
// 接收：uart_rx 经 2 级同步 + 起始位中点采样，抗抖。
// ============================================================
`include "vision_def.v"

module dbg_uart
(
	input                       clk,
	input                       rst,
	// ---- 发送侧 ----
	input                       tx_req,
	input[7:0]                  tx_data,
	output                      tx_busy,
	output                      uart_tx,
	// ---- 接收侧 ----
	input                       uart_rx,
	output reg[7:0]             rx_data,
	output reg                  rx_valid
);

localparam T_IDLE  = 2'd0;
localparam T_START = 2'd1;
localparam T_DATA  = 2'd2;
localparam T_STOP  = 2'd3;

localparam R_IDLE  = 2'd0;
localparam R_START = 2'd1;
localparam R_DATA  = 2'd2;
localparam R_STOP  = 2'd3;

// ------------------------------------------------------------
// 发送
// ------------------------------------------------------------
reg[1:0]  tst;
reg[15:0] tbcnt;
reg[3:0]  tbitc;
reg[7:0]  tsh;
reg       tx_r;

assign uart_tx = tx_r;
assign tx_busy = (tst != T_IDLE);

always@(posedge clk or posedge rst)
begin
	if(rst)
	begin
		tst   <= T_IDLE;
		tbcnt <= 16'd0;
		tbitc <= 4'd0;
		tsh   <= 8'd0;
		tx_r  <= 1'b1;
	end
	else
	begin
		case(tst)
			T_IDLE:
			begin
				tx_r <= 1'b1;
				if(tx_req == 1'b1)
				begin
					tsh   <= tx_data;
					tbcnt <= 16'd0;
					tbitc <= 4'd0;
					tst   <= T_START;
				end
			end

			T_START:                       // 起始位：1 个位时间的低电平
			begin
				tx_r <= 1'b0;
				if(tbcnt == (`UART_BAUD_DIV - 16'd1))
				begin
					tbcnt <= 16'd0;
					tst   <= T_DATA;
				end
				else
					tbcnt <= tbcnt + 16'd1;
			end

			T_DATA:                        // 8 位数据，LSB 先发
			begin
				tx_r <= tsh[0];
				if(tbcnt == (`UART_BAUD_DIV - 16'd1))
				begin
					tbcnt <= 16'd0;
					tsh   <= {1'b0, tsh[7:1]};
					if(tbitc == 4'd7)
						tst <= T_STOP;
					else
						tbitc <= tbitc + 4'd1;
				end
				else
					tbcnt <= tbcnt + 16'd1;
			end

			T_STOP:                        // 停止位：1 个位时间的高电平
			begin
				tx_r <= 1'b1;
				if(tbcnt == (`UART_BAUD_DIV - 16'd1))
				begin
					tbcnt <= 16'd0;
					tst   <= T_IDLE;
				end
				else
					tbcnt <= tbcnt + 16'd1;
			end

			default:
				tst <= T_IDLE;
		endcase
	end
end

// ------------------------------------------------------------
// 接收
// ------------------------------------------------------------
reg[1:0]  rxs;          // 2 级同步器
reg[1:0]  rstate;
reg[15:0] rbcnt;
reg[3:0]  rbitc;
reg[7:0]  rsh;

wire rx_s = rxs[1];

always@(posedge clk or posedge rst)
begin
	if(rst) begin
		rxs     <= 2'b11;
		rstate  <= R_IDLE;
		rbcnt   <= 16'd0;
		rbitc   <= 4'd0;
		rsh     <= 8'd0;
		rx_data <= 8'd0;
		rx_valid<= 1'b0;
	end
	else
	begin
		rxs      <= {rxs[0], uart_rx};
		rx_valid <= 1'b0;

		case(rstate)
			R_IDLE:
				if(rx_s == 1'b0)               // 起始位下降沿
				begin
					rbcnt  <= 16'd0;
					rstate <= R_START;
				end

			R_START:                            // 等半个位时间后确认起始位
				if(rbcnt == ((`UART_BAUD_DIV >> 1) - 16'd1))
				begin
					rbcnt <= 16'd0;
					if(rx_s == 1'b0)             // 仍为低 → 真起始位
					begin
						rbitc  <= 4'd0;
						rstate <= R_DATA;
					end
					else
						rstate <= R_IDLE;          // 毛刺，丢弃
				end
				else
					rbcnt <= rbcnt + 16'd1;

			R_DATA:                             // 每个位时间采样一次（近似中点）
				if(rbcnt == (`UART_BAUD_DIV - 16'd1))
				begin
					rbcnt <= 16'd0;
					rsh   <= {rx_s, rsh[7:1]};
					if(rbitc == 4'd7)
						rstate <= R_STOP;
					else
						rbitc <= rbitc + 4'd1;
				end
				else
					rbcnt <= rbcnt + 16'd1;

			R_STOP:
				if(rbcnt == (`UART_BAUD_DIV - 16'd1))
				begin
					rbcnt    <= 16'd0;
					rstate   <= R_IDLE;
					rx_data  <= rsh;
					rx_valid <= 1'b1;
				end
				else
					rbcnt <= rbcnt + 16'd1;

			default:
				rstate <= R_IDLE;
		endcase
	end
end

endmodule
