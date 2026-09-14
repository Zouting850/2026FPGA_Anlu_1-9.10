// ============================================================
// top_vision_m3.v
// 视觉处理副板 M3 顶层（安路 EG4S20BG256 / 康芯 HX4S20）
//
// M3 目标（对应方案 v2.0 第 7 节）：形态学 + 行列投影 + 人数分段。
//   在 M2（背景建模 + 帧差二值化 + 前景包围盒）之上叠加：
//     + morph.v       3x3 开运算，压掉孤立噪点
//     + projection.v  列/行投影直方图（4 列/4 行一箱），逐帧读出
//     + people_seg.v  列投影阈值 + 行程分段 → 人数估计
//
// 状态行 141 字节：
//   MH3 K N=xxxxxx F=xxxxxx M=xxxxxx P=x S=xxx V=<94 个 hex 字符>
//     K = SCCB 自检（cam_ok）
//     N = 本帧有效像素数，应 = 90240（几何回归，和 M1/M2 同义）
//     F = 形态学前（原始帧差）前景像素数，静态应 → 0
//     M = 形态学后前景像素数；与 F 对照可看出开运算压掉了多少噪点
//     P = 人数估计（列投影分段数，饱和 15）
//     S = 占用列数（列箱占用数 x 4）
//     V = 列投影曲线，94 个 hex 字符，每个字符是 4 列的前景像素和 >> 6
//         列箱上限 4x240 = 960，>> 6 恰好落在 0..15
//         **串口上打印出来就是一条 ASCII 曲线，可直接与目视对照**
//
// 时钟域与 M1/M2 相同：sys_clk 50MHz（SCCB/串口/上报）+
// cam_pclk≈13.5MHz（DVP 采集 + 全部像素流水线，单域内完成，无 CDC）。
// 唯一跨时钟域的是"帧结果快照"：PCLK 域的 viz_done_tog（投影读出结束）
// 同步到 sys 域取边沿，延时几拍后一次性抓走全部结果——这些量在整帧内
// 准静态，属于经典的"翻转握手 + 准静态多比特"做法。
// ============================================================
`include "vision_def.v"

module top_vision_m3
(
	input                       sys_clk,     // R7, 50 MHz
	input                       rst_n,       // KEY1(A2)，低有效
	// ---- 摄像头（J1）----
	input                       cam_pclk,    // D14
	input                       cam_href,    // L14
	input                       cam_vsync,   // M14
	input[7:0]                  cam_d,       // [0..7] = G11 G12 F13 H13 H14 J14 J13 K12
	inout                       cam_scl,     // P11（SCL 还是 SDA 由 u_cfg 探测决定，见 pin.adc）
	inout                       cam_sda,     // L10
	// ---- 调试串口（板载 CH340）----
	output                      uart_tx,     // D12
	input                       uart_rx,     // F12
	// ---- LED ----
	output[3:0]                 led          // A4 A3 C10 B12
);

localparam LINE_LEN = `M3_LINE_LEN;
localparam CURVE_W  = `CURVE_N * 4;      // 376

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
// DVP 采集（PCLK 域）
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

// ------------------------------------------------------------
// 帧指纹（几何/灰度回归用）
// ------------------------------------------------------------
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
// M2：背景建模 + 帧差（PCLK 域）
// ------------------------------------------------------------
wire       fg_valid;
wire       fg;
wire[15:0] fg_x;
wire[15:0] fg_y;
wire       bg_loading;

m2_engine u_m2
(
	.pclk        (cam_pclk),
	.rst_n       (rst_p_n),
	.pix_data    (pix_data),
	.pix_valid   (pix_valid),
	.frame_start (frame_start),
	.fg_valid    (fg_valid),
	.fg          (fg),
	.fg_x        (fg_x),
	.fg_y        (fg_y),
	.bg_loading  (bg_loading)
);

wire[16:0] g_cnt;
wire[15:0] g_minx;
wire[15:0] g_maxx;
wire[15:0] g_miny;
wire[15:0] g_maxy;
wire       g_nz;
wire       g_done_tog;

fg_stat u_fgstat
(
	.clk            (cam_pclk),
	.rst_n          (rst_p_n),
	.fg_valid       (fg_valid),
	.fg             (fg),
	.fg_x           (fg_x),
	.fg_y           (fg_y),
	.frame_start    (frame_start),
	.fg_cnt_o       (g_cnt),
	.box_min_x_o    (g_minx),
	.box_max_x_o    (g_maxx),
	.box_min_y_o    (g_miny),
	.box_max_y_o    (g_maxy),
	.box_nz_o       (g_nz),
	.frame_done_tog (g_done_tog)
);

// ------------------------------------------------------------
// M3：形态学（PCLK 域）
// ------------------------------------------------------------
wire       m_valid;
wire       m_fg;
wire[15:0] m_x;
wire[15:0] m_y;

morph u_morph
(
	.clk      (cam_pclk),
	.rst_n    (rst_p_n),
	.fg_valid (fg_valid),
	.fg       (fg),
	.fg_x     (fg_x),
	.fg_y     (fg_y),
	.m_valid  (m_valid),
	.m_fg     (m_fg),
	.m_x      (m_x),
	.m_y      (m_y)
);

// ------------------------------------------------------------
// M3：行列投影（PCLK 域）
// ------------------------------------------------------------
wire[16:0]                 m_cnt;
wire                       ro_valid;
wire                       ro_phase;
wire[6:0]                  ro_bin;
wire[`PROJ_RW-1:0]         ro_val;
wire                       viz_done_tog;
wire[CURVE_W-1:0]          curve_bits;

projection u_proj
(
	.clk          (cam_pclk),
	.rst_n        (rst_p_n),
	.m_valid      (m_valid),
	.m_fg         (m_fg),
	.m_x          (m_x),
	.m_y          (m_y),
	.frame_start  (frame_start),
	.m_cnt_o      (m_cnt),
	.ro_valid     (ro_valid),
	.ro_phase     (ro_phase),
	.ro_bin       (ro_bin),
	.ro_val       (ro_val),
	.viz_done_tog (viz_done_tog),
	.curve_bits   (curve_bits)
);

// ------------------------------------------------------------
// M3：人数分段（PCLK 域）
// ------------------------------------------------------------
wire[3:0]          s_ppl;
wire[7:0]          s_occ;
wire[6:0]          s_cf;
wire[6:0]          s_cl;
wire[`PROJ_RW-1:0] s_cp;
wire[6:0]          s_cpb;
wire[5:0]          s_rf;
wire[5:0]          s_rl;
wire               s_row_nz;

people_seg u_seg
(
	.clk         (cam_pclk),
	.rst_n       (rst_p_n),
	.ro_valid    (ro_valid),
	.ro_phase    (ro_phase),
	.ro_bin      (ro_bin),
	.ro_val      (ro_val),
	.frame_start (frame_start),
	.ppl_o       (s_ppl),
	.occ_o       (s_occ),
	.cf_o        (s_cf),
	.cl_o        (s_cl),
	.cp_o        (s_cp),
	.cpb_o       (s_cpb),
	.rf_o        (s_rf),
	.rl_o        (s_rl),
	.row_nz_o    (s_row_nz)
);

// ------------------------------------------------------------
// 跨时钟域：投影读出结束翻转 → sys 域边沿 → 延时抓取准静态快照
//   注意这里用 viz_done_tog（读出结束）而不是 frame_start：
//   人数/占用列/曲线都是读出过程里才逐箱攒出来的，必须在读出结束之后抓，
//   否则抓到的永远是上一帧的分段结果。读出 154 拍 ≈ 11us，仍在一帧内；
//   而 frame_stat / fg_stat / m_cnt 都在 frame_start 就锁存并在整帧内保持，
//   所以这一组数依然出自同一帧。
// ------------------------------------------------------------
reg[2:0] tog_sync = 3'b000;
always@(posedge sys_clk)
	tog_sync <= {tog_sync[1:0], viz_done_tog};

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

reg[23:0]        snap_cnt;
reg[16:0]        snap_fg;
reg[16:0]        snap_m;
reg[3:0]         snap_ppl;
reg[9:0]         snap_ocs;             // 占用列数 = 占用箱数 x 4
reg[CURVE_W-1:0] snap_curve;

always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
	begin
		snap_cnt   <= 24'd0;
		snap_fg    <= 17'd0;
		snap_m     <= 17'd0;
		snap_ppl   <= 4'd0;
		snap_ocs   <= 10'd0;
		snap_curve <= {CURVE_W{1'b0}};
	end
	else if(snap_ld == 1'b1)
	begin
		snap_cnt   <= f_cnt;
		snap_fg    <= g_cnt;
		snap_m     <= m_cnt;
		snap_ppl   <= s_ppl;
		snap_ocs   <= {s_occ, 2'b00};
		snap_curve <= curve_bits;
	end
end

// ------------------------------------------------------------
// 打印寄存器（发送开始时从快照锁存，保证整行一致）
// ------------------------------------------------------------
reg[23:0]        prt_cnt;
reg[16:0]        prt_fg;
reg[16:0]        prt_m;
reg[3:0]         prt_ppl;
reg[9:0]         prt_ocs;
reg              prt_ok;
reg       prt_sw;      // 上报用：自检通过时两线是否被判为接反（'W'）
reg[CURVE_W-1:0] curve_sh;

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
	input[7:0] i;
	begin
		case(i)
			8'd0:   fixed_char = 8'h4D;   // 'M'
			8'd1:   fixed_char = 8'h48;   // 'H'
			8'd2:   fixed_char = 8'h33;   // '3'
			8'd3:   fixed_char = 8'h20;
			8'd5:   fixed_char = 8'h20;
			8'd6:   fixed_char = 8'h4E;   // 'N'
			8'd7:   fixed_char = 8'h3D;   // '='
			8'd14:  fixed_char = 8'h20;
			8'd15:  fixed_char = 8'h46;   // 'F'
			8'd16:  fixed_char = 8'h3D;
			8'd23:  fixed_char = 8'h20;
			8'd24:  fixed_char = 8'h4D;   // 'M'
			8'd25:  fixed_char = 8'h3D;
			8'd32:  fixed_char = 8'h20;
			8'd33:  fixed_char = 8'h50;   // 'P'
			8'd34:  fixed_char = 8'h3D;
			8'd36:  fixed_char = 8'h20;
			8'd37:  fixed_char = 8'h53;   // 'S'
			8'd38:  fixed_char = 8'h3D;
			8'd42:  fixed_char = 8'h20;
			8'd43:  fixed_char = 8'h56;   // 'V'
			8'd44:  fixed_char = 8'h3D;
			8'd139: fixed_char = 8'h0D;   // CR
			8'd140: fixed_char = 8'h0A;   // LF
			default: fixed_char = 8'h00;  // 0x00 = 该位为动态字符
		endcase
	end
endfunction

function[7:0] line_byte;
	input[7:0] i;
	reg[7:0]  fc;
	reg[3:0]  nb;
	reg[31:0] vtmp;
	begin
		fc = fixed_char(i);
		if(fc != 8'h00)
			line_byte = fc;
		else if((i >= 8'd8) && (i <= 8'd13))          // N = 6 hex
		begin
			nb = 4'd5 - (i - 8'd8);
			vtmp = {8'd0, prt_cnt};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if((i >= 8'd17) && (i <= 8'd22))         // F = 6 hex
		begin
			nb = 4'd5 - (i - 8'd17);
			vtmp = {15'd0, prt_fg};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if((i >= 8'd26) && (i <= 8'd31))         // M = 6 hex
		begin
			nb = 4'd5 - (i - 8'd26);
			vtmp = {15'd0, prt_m};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if(i == 8'd35)                           // P = 1 hex
			line_byte = hexc(prt_ppl);
		else if((i >= 8'd39) && (i <= 8'd41))         // S = 3 hex
		begin
			nb = 4'd2 - (i - 8'd39);
			vtmp = {22'd0, prt_ocs};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if((i >= 8'd45) && (i <= 8'd138))        // 曲线：94 个 nibble
			line_byte = hexc(curve_sh[3:0]);
		else if(i == 8'd4)                            // 自检标志
			line_byte = prt_ok ? (prt_sw ? 8'h57 : 8'h4B) : 8'h46;   // 'K' 正常 / 'W' 接反已换向 / 'F' 失败
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
reg[7:0] xpos;
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
		xst      <= X_IDLE;
		xpos     <= 8'd0;
		tx_req   <= 1'b0;
		tx_data  <= 8'd0;
		prt_cnt  <= 24'd0;
		prt_fg   <= 17'd0;
		prt_m    <= 17'd0;
		prt_ppl  <= 4'd0;
		prt_ocs  <= 10'd0;
		prt_ok   <= 1'b0;
		prt_sw  <= 1'b0;
		curve_sh <= {CURVE_W{1'b0}};
	end
	else
	begin
		case(xst)
			X_IDLE:
			begin
				tx_req <= 1'b0;
				if(send_pend == 1'b1)
				begin
					prt_cnt  <= snap_cnt;
					prt_fg   <= snap_fg;
					prt_m    <= snap_m;
					prt_ppl  <= snap_ppl;
					prt_ocs  <= snap_ocs;
					prt_ok   <= cam_ok;
					prt_sw  <= sccb_swap;
					curve_sh <= snap_curve;   // 曲线的 94 个 nibble 一次性装填
					xpos     <= 8'd0;
					xst      <= X_SEND;
				end
			end

			X_SEND:
			begin
				if(tx_busy == 1'b0)
				begin
					tx_req  <= 1'b1;
					tx_data <= line_byte(xpos);
					// 曲线区每吐一个字符右移 4 位（高位补 0）。因为 projection
					// 是从高位开始打包的，箱 0 就停在 bit[3:0]，所以这里是
					// "先吐低位再右移"，输出顺序正好是箱 0、1、2…
					// 反过来的话曲线会左右镜像，峰位全反。
					if((xpos >= 8'd45) && (xpos <= 8'd138))
						curve_sh <= {4'd0, curve_sh[CURVE_W-1:4]};
					xst <= X_WAIT;
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
					if(xpos == (LINE_LEN - 8'd1))
						xst <= X_IDLE;
					else
					begin
						xpos <= xpos + 8'd1;
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
// LED
//   led[0] = 配置序列完成
//   led[1] = SCCB 自检通过
//   led[2] = 0.5s 内有新帧
//   led[3] = 背景已装载完成（bg_loading 结束）→ 可以开始判读了
// ------------------------------------------------------------
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

// bg_loading 在 PCLK 域，用两级同步到 sys 域只是做指示灯，不需要严格
reg[1:0] load_sync = 2'b11;
always@(posedge sys_clk)
	load_sync <= {load_sync[0], bg_loading};

assign led[0] = cfg_done;
assign led[1] = cam_ok;
assign led[2] = (frm_wd < 25'd25000000);    // 0.5 s 内有帧 → 亮
assign led[3] = ~load_sync[1];              // 背景装载完成 → 亮

// ------------------------------------------------------------
// 显式标记未使用的信号，避免综合告警（宽度 249 bit，逐项列全）
// ------------------------------------------------------------
wire[248:0] unused_bus;
assign unused_bus = {sccb_nack, rx_data, rx_valid, wr_idx, err_cnt, cfg_busy,
                     f_sum, f_min, f_max, ver_rd, col_cnt, row_cnt,
                     frame_valid, line_start, frame_cnt, g_nz,
                     g_minx, g_maxx, g_miny, g_maxy, f_done_tog, g_done_tog,
                     s_cf, s_cl, s_cp, s_cpb, s_rf, s_rl, s_row_nz};

endmodule
