// ============================================================
// top_vision_m4.v
// 视觉处理副板 M4 顶层（安路 EG4S20BG256 / 康芯 HX4S20）
//
// M4 目标（对应方案 v2.0 第 7 节）：自动曝光闭环。
//   在 M3（形态学 + 投影 + 人数）之上叠加：
//     + exp_meter.v   帧亮度计量（sum / cnt / 饱和像素数）
//     + auto_exp.v    关掉片上 AEC/AGC，改由 FPGA 按目标亮度闭环回写
//                     R0x0B（粗快门）/ R0x35（模拟增益）
//
// 【为什么 M4 是 M2/M3 能不能稳的前提】
//   片上 AEC 会随场景持续微调曝光；曝光一动整帧灰度集体平移，帧差立刻
//   满屏假前景，M2 的背景模型被反复冲垮、M3 的投影曲线跟着乱跳。
//   所以 M4 先把 R0xAF 写 0 独占曝光控制权，亮度稳定后前两级才有意义。
//
// 状态行 164 字节：
//   MH4 K N=xxxxxx F=xxxxxx M=xxxxxx P=x S=xxx E=xxxx G=xxx Y=xxx L=x V=<94 hex>
//     K = SCCB 自检（cam_ok）
//     N = 本帧有效像素数，应 = 90240
//     F = 形态学前前景像素数      M = 形态学后前景像素数
//     P = 人数估计                S = 占用列数
//     E = 当前曝光值（R0x0B 粗快门总量，4 位十六进制）
//     G = 当前模拟增益（R0x35，3 位十六进制）
//     Y = 本帧平均灰度（3 位十六进制）——闭环的直接观测量
//     L = 曝光锁定标志（1 = 已连续多帧落在死区内）
//     V = 列投影曲线，94 个 hex 字符（与 M3 同义）
//
// 时钟域与 M1/M2/M3 相同：sys_clk 50MHz（SCCB/串口/上报/曝光闭环）+
// cam_pclk≈13.5MHz（DVP 采集 + 全部像素流水线，单域内完成，无 CDC）。
// 跨时钟域用两处"翻转握手 + 准静态快照"：
//   1) viz_done_tog（投影读出结束）→ M3 结果快照
//   2) meter_done_tog（帧亮度锁存）→ M4 亮度快照，并派生 frame_tick
//      （帧起始一拍，落在垂直消隐期 —— 曝光回写必须对齐消隐期）
// ============================================================
`include "vision_def.v"

module top_vision_m4
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

localparam LINE_LEN = `M4_LINE_LEN;
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
// SCCB 主机 + 配置序列 + 曝光闭环回写 的请求仲裁
//
//   两个请求方共用一条 SCCB：
//     mt9v034_cfg（sys 域）—— 上电配置表 + 版本自检
//     auto_exp   （sys 域）—— 曝光/增益回写
//   仲裁规则极简：cfg_busy 期间一律归 cfg，否则归 auto_exp。
//   因为 cfg_busy 从复位起就是 1（mt9v034_cfg 的 C_PWRWAIT 就把它拉高），
//   所以 auto_exp 在配置完成前**天然发不出任何事务**；而 auto_exp 自身
//   还要等 cfg_done 才离开 E_INIT，两道锁叠加，不存在两个控制器同时
//   改 R0xAF 而互相打架的可能（那正是要避免的：片上 AEC 与 FPGA 闭环
//   抢同一个寄存器会谁也收敛不了）。
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

// 曝光闭环侧（只写）
wire        exp_req;
wire[15:0]  exp_addr;
wire[15:0]  exp_wdata;
wire        exp_busy;
wire        exp_ack;

wire        sccb_req_w   = cfg_busy ? cfg_req   : exp_req;
wire        sccb_rw_w    = cfg_busy ? cfg_rw    : 1'b0;      // auto_exp 只写
wire[15:0]  sccb_addr_w  = cfg_busy ? cfg_addr  : exp_addr;
wire[15:0]  sccb_wdata_w = cfg_busy ? cfg_wdata : exp_wdata;

// auto_exp 只在 cfg 空闲时才"看见"事务完成
assign exp_busy = sccb_busy & (~cfg_busy);
assign exp_ack  = sccb_ack  & (~cfg_busy);

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
	.req      (sccb_req_w),
	.rw       (sccb_rw_w),
	.reg_addr (sccb_addr_w),
	.wr_data  (sccb_wdata_w),
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
// 帧指纹（几何回归，N 字段的来源；与 M1/M2/M3 同源，保证 N 可比）
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
// M4：帧亮度计量（PCLK 域）
//   与 frame_stat 并行存在，故意不合并：多一个约 60 FF 的累加器，
//   换 M1 的模块与模型**零改动**。里程碑一经验证即冻结，是本项目的硬规矩。
// ------------------------------------------------------------
wire[31:0] e_sum;
wire[23:0] e_cnt;
wire[23:0] e_sat;
wire       e_tog;

exp_meter u_meter
(
	.pclk            (cam_pclk),
	.rst_n           (rst_p_n),
	.pix_valid       (pix_valid),
	.pix_data        (pix_data),
	.frame_start     (frame_start),
	.sum_o           (e_sum),
	.cnt_o           (e_cnt),
	.sat_o           (e_sat),
	.meter_done_tog  (e_tog)
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
// M4：曝光闭环控制器（sys 域）
//   输入用同步过来的亮度快照（见下），输出接到上面仲裁的 exp_* 一侧。
// ------------------------------------------------------------
wire[15:0] exp_shut_v;
wire[15:0] exp_gain_v;
wire[7:0]  exp_mean_v;
wire       exp_locked_v;
wire[15:0] exp_wr_cnt;

// 亮度快照 + frame_tick 的产生（见下节的 CDC 小节，这里先声明）
reg[31:0] snap_sum;
reg[23:0] snap_ecnt;
reg[23:0] snap_sat;
wire       frame_tick;

auto_exp u_aexp
(
	.clk       (sys_clk),
	.rst       (rst_sys),
	.frm_sum   (snap_sum),
	.frm_cnt   (snap_ecnt),
	.frm_sat   (snap_sat),
	.frame_tick(frame_tick),
	.cfg_done  (cfg_done),
	.sccb_req  (exp_req),
	.sccb_addr (exp_addr),
	.sccb_wdata(exp_wdata),
	.sccb_busy (exp_busy),
	.sccb_ack  (exp_ack),
	.exp_shut  (exp_shut_v),
	.exp_gain  (exp_gain_v),
	.exp_mean  (exp_mean_v),
	.exp_locked(exp_locked_v),
	.wr_cnt    (exp_wr_cnt)
);

// ------------------------------------------------------------
// 跨时钟域 1：投影读出结束翻转 → sys 域边沿 → 延时抓取 M3 准静态快照
//   用 viz_done_tog（读出结束）而不是 frame_start：人数/占用列/曲线都是
//   读出过程里逐箱攒出来的，必须等读出结束再抓，否则抓到的是上一帧结果。
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
// 跨时钟域 2：帧亮度锁存翻转 → sys 域边沿 → 亮度快照 + frame_tick
//   exp_meter 在 frame_start（落在垂直消隐期）锁存 sum/cnt/sat，
//   锁存值在整帧内稳定，所以"边沿 + 延时抓拍"两拍足够。
//   frame_tick 取在快照之后一拍，保证 auto_exp 决策时快照已稳。
//   这一拍也正好让"曝光回写只在帧起始/消隐期发生"成立。
// ------------------------------------------------------------
reg[2:0] mt_sync = 3'b000;
always@(posedge sys_clk)
	mt_sync <= {mt_sync[1:0], e_tog};

wire meter_edge = mt_sync[2] ^ mt_sync[1];

reg[5:0] mt_dly;
always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
		mt_dly <= 6'd0;
	else
		mt_dly <= {mt_dly[4:0], meter_edge};
end

wire exp_ld     = mt_dly[2];           // 抓亮度快照
// 决策脉冲必须等曝光决策流水线出结果（auto_exp.v 里 errmag_r / step_c_r 两级）：
//   快照在 mt_dly[2] 之后一拍可用，再经两段流水线，所以取 mt_dly[5] 而不是
//   原来的 mt_dly[3]（晚两拍 = 40ns）。一帧 16.7ms，这点延迟无影响；
//   但**必须**晚这两拍，否则状态机会取到流水线还没算完的值。
//   详细推导见 auto_exp.v 的"曝光决策流水线"注释。
assign frame_tick = mt_dly[5];

always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
	begin
		snap_sum  <= 32'd0;
		snap_ecnt <= 24'd0;
		snap_sat  <= 24'd0;
	end
	else if(exp_ld == 1'b1)
	begin
		snap_sum  <= e_sum;
		snap_ecnt <= e_cnt;
		snap_sat  <= e_sat;
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
reg[15:0]        prt_exp;       // E 字段：当前快门值
reg[15:0]        prt_gain;      // G 字段：当前增益值
reg[7:0]         prt_mean;      // Y 字段：本帧平均灰度
reg              prt_lock;      // L 字段：曝光锁定
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
			8'd2:   fixed_char = 8'h34;   // '4'
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
			8'd43:  fixed_char = 8'h45;   // 'E'
			8'd44:  fixed_char = 8'h3D;
			8'd49:  fixed_char = 8'h20;
			8'd50:  fixed_char = 8'h47;   // 'G'
			8'd51:  fixed_char = 8'h3D;
			8'd55:  fixed_char = 8'h20;
			8'd56:  fixed_char = 8'h59;   // 'Y'
			8'd57:  fixed_char = 8'h3D;
			8'd61:  fixed_char = 8'h20;
			8'd62:  fixed_char = 8'h4C;   // 'L'
			8'd63:  fixed_char = 8'h3D;
			8'd65:  fixed_char = 8'h20;
			8'd66:  fixed_char = 8'h56;   // 'V'
			8'd67:  fixed_char = 8'h3D;
			8'd162: fixed_char = 8'h0D;   // CR
			8'd163: fixed_char = 8'h0A;   // LF
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
		else if((i >= 8'd45) && (i <= 8'd48))         // E = 4 hex（曝光）
		begin
			nb = 4'd3 - (i - 8'd45);
			vtmp = {16'd0, prt_exp};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if((i >= 8'd52) && (i <= 8'd54))         // G = 3 hex（增益）
		begin
			nb = 4'd2 - (i - 8'd52);
			vtmp = {16'd0, prt_gain};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if((i >= 8'd58) && (i <= 8'd60))         // Y = 3 hex（帧均值）
		begin
			nb = 4'd2 - (i - 8'd58);
			vtmp = {24'd0, prt_mean};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if(i == 8'd64)                           // L = 1 hex（锁定）
			line_byte = hexc({3'd0, prt_lock});
		else if((i >= 8'd68) && (i <= 8'd161))        // 曲线：94 个 nibble
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

wire cfg_done_rise = cfg_done & (~cfg_done_d);
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
		prt_exp  <= 16'd0;
		prt_gain <= 16'd0;
		prt_mean <= 8'd0;
		prt_lock <= 1'b0;
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
					prt_exp  <= exp_shut_v;
					prt_gain <= exp_gain_v;
					prt_mean <= exp_mean_v;
					prt_lock <= exp_locked_v;
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
					// 反过来的话曲线会左右镜像，峰位全反（M3 的教训）。
					if((xpos >= 8'd68) && (xpos <= 8'd161))
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
//   led[3] = 曝光已锁定（连续 EXP_LOCK_FRAMES 帧落在死区内）
//            —— M4 的核心验收证据；背景装载指示已由 M2 验收步骤覆盖
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

assign led[0] = cfg_done;
assign led[1] = cam_ok;
assign led[2] = (frm_wd < 25'd25000000);    // 0.5 s 内有帧 → 亮
assign led[3] = exp_locked_v;               // 曝光收敛锁定 → 亮

// ------------------------------------------------------------
// 显式标记未使用的信号，避免综合告警（宽度 265 bit，逐项列全）
// ------------------------------------------------------------
wire[264:0] unused_bus;
assign unused_bus = {sccb_nack, rx_data, rx_valid, wr_idx, err_cnt, cfg_busy,
                     f_sum, f_min, f_max, ver_rd, col_cnt, row_cnt,
                     frame_valid, line_start, frame_cnt, g_nz,
                     g_minx, g_maxx, g_miny, g_maxy, f_done_tog, g_done_tog,
                     s_cf, s_cl, s_cp, s_cpb, s_rf, s_rl, s_row_nz,
                     exp_wr_cnt};

endmodule
