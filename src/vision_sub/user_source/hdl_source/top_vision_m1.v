// ============================================================
// top_vision_m1.v
// 视觉处理副板 M1 顶层（安路 EG4S20BG256 / 康芯 HX4S20）
//
// M1 目标（对应方案 v2.0 第 7 节）：
//   1) 用 FPGA 经 SCCB 配置 MT9V034，读回 R0x00 = 0x1324 自检通过
//   2) 正确采集 DVP 像素流（2x2 binning → 376x240）
//   3) 通过板载 CH340 串口(115200) 每秒打印一行帧指纹，供人工/脚本判读
//
// 本顶层刻意不引入 SDRAM/PLL：M1 只用 50 MHz 板载时钟即可闭环验证
// "配置 + 采集 + 统计"。行缓存/SDRAM 帧缓存属 M1b（需 TD 的 SDRAM
// 控制器 IP），M2 之后再接。
//
// 时钟域：
//   sys_clk = 50 MHz（R7）—— SCCB、配置序列、串口、统计上报
//   cam_pclk ≈ 13.5 MHz（bin2 后）—— DVP 采集、帧指纹累加
//   跨域只搬运"帧完成"翻转信号 + 准静态快照（详见 frame_stat.v 注释）
//
// 串口输出格式（52 字节定长，CRLF 结尾）：
//   MH1 K V=1324 S=xxxxxxxx N=xxxxxx L=xx H=ff P0=xxxx
//     K/W/F    = SCCB 自检通过 / 通过但两线接反已换向 / 两种极性都不通
//     V        = 最后一次读回的芯片版本（应为 1324）
//     S        = 本帧像素灰度累加和（32bit）
//     N        = 本帧有效像素数（应 = 90240 = 376*240）
//     L / H    = 本帧最小 / 最大灰度
//     P0       = 【诊断】第一次（极性 0）探测读到的 R0x00
//                排障用：都不通时把 V 与 P0 摆在一起看 —— 一个大一个小说明
//                其中一根线被拉低（短路/下拉/改造接错）；两个都是 FFFF 说明
//                两根线都没有任何东西在应答（断路、或改造没把 IIC 引出来）。
// ============================================================
`include "vision_def.v"

module top_vision_m1
(
	input                       sys_clk,     // R7, 50 MHz
	input                       rst_n,       // KEY1(A2)，低有效
	// ---- 摄像头（J1）----
	input                       cam_pclk,    // D14
	input                       cam_href,    // L14
	input                       cam_vsync,   // M14
	input[7:0]                  cam_d,       // [0..7] = G11 G12 F13 H13 H14 J14 J13 K12
	inout                       cam_scl,     // P11（可能是 SCL，也可能是 SDA，见 pin.adc）
	inout                       cam_sda,     // L10
	// ---- 调试串口（板载 CH340）----
	output                      uart_tx,     // D12
	input                       uart_rx,     // F12
	// ---- LED ----
	output[3:0]                 led          // A4 A3 C10 B12
);

localparam LINE_LEN = 6'd52;

// ------------------------------------------------------------
// 上电复位 + 双域复位同步
// ------------------------------------------------------------
reg[15:0] por_cnt;
reg       por_n;

always@(posedge sys_clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		por_cnt <= 16'd0;
		por_n   <= 1'b0;
	end
	else if(por_cnt != 16'hFFFF)
	begin
		por_cnt <= por_cnt + 16'd1;
		por_n   <= 1'b0;
	end
	else
		por_n <= 1'b1;
end

wire rst_sys = ~por_n;                  // sys 域，高有效

reg[1:0] por_sync_p = 2'b00;
always@(posedge cam_pclk)
	por_sync_p <= {por_sync_p[0], por_n};
wire rst_p_n = por_sync_p[1];           // pclk 域，低有效

// ------------------------------------------------------------
// SCCB 主机 + 配置序列
// ------------------------------------------------------------
wire        cfg_req;
wire        cfg_rw;
wire[15:0]  cfg_addr;
wire[15:0]  cfg_wdata;
wire        sccb_busy;
wire        sccb_ack;
wire[15:0]  sccb_rdata;
wire[4:0]   sccb_nack;
wire        sccb_swap;      // 由 u_cfg 探测出的两线极性，直接喂给 u_sccb.swap
wire        cfg_busy;
wire        cfg_done;
wire        cam_ok;
wire[15:0]  ver_rd;
wire[15:0]  ver_rd0;     // 极性 0 那一次探测的读值（排障用，见 mt9v034_cfg.v）
wire[3:0]   wr_idx;
wire[3:0]   err_cnt;

mt9v034_cfg u_cfg
(
	.clk        (sys_clk),
	.rst        (rst_sys),
	.restart    (1'b0),
	.sccb_req   (cfg_req),
	.sccb_rw    (cfg_rw),
	.sccb_addr  (cfg_addr),
	.sccb_wdata (cfg_wdata),
	.sccb_busy  (sccb_busy),
	.sccb_ack   (sccb_ack),
	.sccb_rdata (sccb_rdata),
	.sccb_swap  (sccb_swap),
	.cfg_busy   (cfg_busy),
	.cfg_done   (cfg_done),
	.cam_ok     (cam_ok),
	.ver_rd     (ver_rd),
	.ver_rd0    (ver_rd0),
	.wr_idx     (wr_idx),
	.err_cnt    (err_cnt)
);

sccb_master u_sccb
(
	.clk      (sys_clk),
	.rst      (rst_sys),
	.req      (cfg_req),
	.rw       (cfg_rw),
	.reg_addr (cfg_addr),
	.wr_data  (cfg_wdata),
	.busy     (sccb_busy),
	.ack      (sccb_ack),
	.rd_data  (sccb_rdata),
	.nack_cnt (sccb_nack),
	.swap     (sccb_swap),
	.scl      (cam_scl),
	.sda      (cam_sda)
);

// ------------------------------------------------------------
// DVP 采集 + 帧指纹（PCLK 域）
// ------------------------------------------------------------
wire[7:0]  pix_data;
wire       pix_valid;
wire       line_start;
wire       frame_start;
wire       frame_valid;
wire[15:0] col_cnt;
wire[15:0] row_cnt;
wire[15:0] frame_cnt;

dvp_capture u_cap
(
	.pclk        (cam_pclk),
	.rst_n       (rst_p_n),
	.vsync       (cam_vsync),
	.href        (cam_href),
	.din         (cam_d),
	.pix_data    (pix_data),
	.pix_valid   (pix_valid),
	.line_start  (line_start),
	.frame_start (frame_start),
	.frame_valid (frame_valid),
	.col_cnt     (col_cnt),
	.row_cnt     (row_cnt),
	.frame_cnt   (frame_cnt)
);

wire[31:0] f_sum;
wire[23:0] f_cnt;
wire[7:0]  f_min;
wire[7:0]  f_max;
wire       f_done_tog;

frame_stat u_stat
(
	.pclk           (cam_pclk),
	.rst_n          (rst_p_n),
	.pix_valid      (pix_valid),
	.pix_data       (pix_data),
	.frame_start    (frame_start),
	.sum_o          (f_sum),
	.cnt_o          (f_cnt),
	.min_o          (f_min),
	.max_o          (f_max),
	.frame_done_tog (f_done_tog)
);

// ------------------------------------------------------------
// 跨时钟域：帧完成翻转 → sys 域边沿 → 延时抓取准静态快照
// ------------------------------------------------------------
reg[2:0] tog_sync = 3'b000;
always@(posedge sys_clk)
	tog_sync <= {tog_sync[1:0], f_done_tog};

wire frm_edge = tog_sync[2] ^ tog_sync[1];

reg[3:0] latch_dly;
always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
		latch_dly <= 4'd0;
	else
		latch_dly <= {latch_dly[2:0], frm_edge};
end

wire snap_ld = latch_dly[3];           // 边沿后 3 拍再取数，留足建立时间

reg[31:0] snap_sum;
reg[23:0] snap_cnt;
reg[7:0]  snap_min;
reg[7:0]  snap_max;
reg[15:0] snap_fcnt;

always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
	begin
		snap_sum  <= 32'd0;
		snap_cnt  <= 24'd0;
		snap_min  <= 8'h00;
		snap_max  <= 8'h00;
		snap_fcnt <= 16'd0;
	end
	else if(snap_ld == 1'b1)
	begin
		snap_sum  <= f_sum;
		snap_cnt  <= f_cnt;
		snap_min  <= f_min;
		snap_max  <= f_max;
		snap_fcnt <= frame_cnt;
	end
end

// ------------------------------------------------------------
// 打印寄存器（发送开始时从快照锁存，保证整行一致）
// ------------------------------------------------------------
reg[15:0] prt_ver;
reg[15:0] prt_ver0;    // P0 = 第一次（极性 0）探测读值；与 V 对照可定位是哪根线不通
reg[31:0] prt_sum;
reg[23:0] prt_cnt;
reg[7:0]  prt_min;
reg[7:0]  prt_max;
reg       prt_ok;
reg       prt_sw;      // 上报用：自检通过时两线是否被判定为接反（'W'）

// ------------------------------------------------------------
// 行字节生成
// ------------------------------------------------------------
function[7:0] hexc;
	input[3:0] n;
	begin
		if(n < 4'd10)
			hexc = 8'h30 + {4'd0, n};
		else
			hexc = 8'h37 + {4'd0, n};
	end
endfunction

function[7:0] fixed_char;
	input[5:0] i;
	begin
		case(i)
			6'd0:  fixed_char = 8'h4D;   // 'M'
			6'd1:  fixed_char = 8'h48;   // 'H'
			6'd2:  fixed_char = 8'h31;   // '1'
			6'd3:  fixed_char = 8'h20;
			6'd5:  fixed_char = 8'h20;
			6'd6:  fixed_char = 8'h56;   // 'V'
			6'd7:  fixed_char = 8'h3D;   // '='
			6'd12: fixed_char = 8'h20;
			6'd13: fixed_char = 8'h53;   // 'S'
			6'd14: fixed_char = 8'h3D;
			6'd23: fixed_char = 8'h20;
			6'd24: fixed_char = 8'h4E;   // 'N'
			6'd25: fixed_char = 8'h3D;
			6'd32: fixed_char = 8'h20;
			6'd33: fixed_char = 8'h4C;   // 'L'
			6'd34: fixed_char = 8'h3D;
			6'd37: fixed_char = 8'h20;
			6'd38: fixed_char = 8'h48;   // 'H'
			6'd39: fixed_char = 8'h3D;
			6'd42: fixed_char = 8'h20;
			6'd43: fixed_char = 8'h50;   // 'P'
			6'd44: fixed_char = 8'h30;   // '0'
			6'd45: fixed_char = 8'h3D;   // '='
			6'd50: fixed_char = 8'h0D;   // CR
			6'd51: fixed_char = 8'h0A;   // LF
			default: fixed_char = 8'h00; // 0x00 表示该位为动态 hex
		endcase
	end
endfunction

function[7:0] line_byte;
	input[5:0] i;
	reg[7:0]  fc;
	reg[3:0]  nb;
	reg[31:0] vtmp;
	begin
		fc = fixed_char(i);
		if(fc != 8'h00)
			line_byte = fc;
		else if((i >= 6'd8) && (i <= 6'd11))                     // V = 4 hex
		begin
			nb = 4'd3 - (i - 6'd8);
			line_byte = hexc(prt_ver[nb*4 +: 4]);
		end
		else if((i >= 6'd15) && (i <= 6'd22))                    // S = 8 hex
		begin
			nb = 4'd7 - (i - 6'd15);
			line_byte = hexc(prt_sum[nb*4 +: 4]);
		end
		else if((i >= 6'd26) && (i <= 6'd31))                    // N = 6 hex
		begin
			nb = 4'd5 - (i - 6'd26);
			vtmp = {8'd0, prt_cnt};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if((i >= 6'd35) && (i <= 6'd36))                    // L = 2 hex
		begin
			nb = 4'd1 - (i - 6'd35);
			vtmp = {24'd0, prt_min};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if((i >= 6'd40) && (i <= 6'd41))                    // H = 2 hex
		begin
			nb = 4'd1 - (i - 6'd40);
			vtmp = {24'd0, prt_max};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if((i >= 6'd46) && (i <= 6'd49))                    // P0 = 4 hex
		begin
			nb = 4'd3 - (i - 6'd46);
			line_byte = hexc(prt_ver0[nb*4 +: 4]);
		end
		else if(i == 6'd4)                                       // 自检标志
			// 'K' = 通且两线是正常接法；'W' = 通但两线接反、已自动换向；
			// 'F' = 两种极性都读不到 0x1324（去查线路/上拉/供电）
			line_byte = prt_ok ? (prt_sw ? 8'h57 : 8'h4B) : 8'h46;
		else
			line_byte = 8'h20;
	end
endfunction

// ------------------------------------------------------------
// 上报触发：1 Hz 周期 + 配置完成瞬间各一次
// ------------------------------------------------------------
reg[25:0] rep_cnt;
reg       rep_pulse;

always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
	begin
		rep_cnt   <= 26'd0;
		rep_pulse <= 1'b0;
	end
	else if(rep_cnt == (`REPORT_PERIOD - 26'd1))
	begin
		rep_cnt   <= 26'd0;
		rep_pulse <= 1'b1;
	end
	else
	begin
		rep_cnt   <= rep_cnt + 26'd1;
		rep_pulse <= 1'b0;
	end
end

reg cfg_done_d;
always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
		cfg_done_d <= 1'b0;
	else
		cfg_done_d <= cfg_done;
end

wire cfg_done_rise = cfg_done & ~cfg_done_d;
wire send_trig     = rep_pulse | cfg_done_rise;

// ------------------------------------------------------------
// 调试串口
// ------------------------------------------------------------
reg        tx_req;
reg[7:0]   tx_data;
wire       tx_busy;
wire[7:0]  rx_data;
wire       rx_valid;

dbg_uart u_uart
(
	.clk      (sys_clk),
	.rst      (rst_sys),
	.tx_req   (tx_req),
	.tx_data  (tx_data),
	.tx_busy  (tx_busy),
	.uart_tx  (uart_tx),
	.uart_rx  (uart_rx),
	.rx_data  (rx_data),
	.rx_valid (rx_valid)
);

// ------------------------------------------------------------
// 发送状态机
// ------------------------------------------------------------
localparam X_IDLE = 2'd0;
localparam X_SEND = 2'd1;
localparam X_WAIT = 2'd2;
localparam X_NEXT = 2'd3;

reg[1:0] xst;
reg[5:0] xpos;
reg      send_pend;

always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
		send_pend <= 1'b0;
	else if(send_trig == 1'b1)
		send_pend <= 1'b1;
	else if(xst != X_IDLE)
		send_pend <= 1'b0;
end

always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
	begin
		xst     <= X_IDLE;
		xpos    <= 6'd0;
		tx_req  <= 1'b0;
		tx_data <= 8'd0;
		prt_ver <= 16'd0;
		prt_ver0<= 16'd0;
		prt_sum <= 32'd0;
		prt_cnt <= 24'd0;
		prt_min <= 8'h00;
		prt_max <= 8'h00;
		prt_ok  <= 1'b0;
		prt_sw  <= 1'b0;
	end
	else
	begin
		case(xst)
			X_IDLE:
			begin
				tx_req <= 1'b0;
				if(send_pend == 1'b1)
				begin
					prt_ver <= ver_rd;
					prt_ver0<= ver_rd0;
					prt_sum <= snap_sum;
					prt_cnt <= snap_cnt;
					prt_min <= snap_min;
					prt_max <= snap_max;
					prt_ok  <= cam_ok;
					prt_sw  <= sccb_swap;
					xpos    <= 6'd0;
					xst     <= X_SEND;
				end
			end

			X_SEND:
			begin
				if(tx_busy == 1'b0)
				begin
					tx_req  <= 1'b1;
					tx_data <= line_byte(xpos);
					xst     <= X_WAIT;
				end
			end

			X_WAIT:
			begin
				tx_req <= 1'b0;
				if(tx_busy == 1'b1)
					xst <= X_NEXT;
			end

			X_NEXT:
			begin
				if(tx_busy == 1'b0)
				begin
					if(xpos == (LINE_LEN - 6'd1))
						xst <= X_IDLE;
					else
					begin
						xpos <= xpos + 6'd1;
						xst  <= X_SEND;
					end
				end
			end

			default:
				xst <= X_IDLE;
		endcase
	end
end

// ------------------------------------------------------------
// LED / 心跳
// ------------------------------------------------------------
reg[23:0] hb_cnt;
reg       hb;

always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
	begin
		hb_cnt <= 24'd0;
		hb     <= 1'b0;
	end
	else if(hb_cnt == 24'd12500000)     // 4 Hz
	begin
		hb_cnt <= 24'd0;
		hb     <= ~hb;
	end
	else
		hb_cnt <= hb_cnt + 24'd1;
end

reg[24:0] frm_wd;
always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
		frm_wd <= 25'd0;
	else if(frm_edge == 1'b1)
		frm_wd <= 25'd0;
	else if(frm_wd != 25'h1FFFFFF)
		frm_wd <= frm_wd + 25'd1;
end

assign led[0] = cfg_done;
assign led[1] = cam_ok;
assign led[2] = (frm_wd < 25'd25000000);    // 0.5 s 内有帧 → 亮
assign led[3] = hb;

// ------------------------------------------------------------
// 显式标记未使用的信号，避免综合告警
// ------------------------------------------------------------
wire[72:0] unused_bus;
assign unused_bus = {sccb_nack, rx_data, rx_valid, wr_idx, err_cnt, cfg_busy,
                     snap_fcnt, col_cnt, row_cnt, frame_valid, line_start};

endmodule
