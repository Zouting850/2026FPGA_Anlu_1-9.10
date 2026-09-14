// ============================================================
// top_vision_m5.v
// 视觉处理副板 M5 顶层（安路 EG4S20BG256 / 康芯 HX4S20）
//
// M5 目标（对应方案 v2.0 §2.2/§7）：行为状态机 + 置信度。
//   在 M4（自动曝光闭环）之上叠加：
//     + behavior_fsm.v  把 M3 的逐帧瞬时量读成"行为"：
//                       有人来了 / 站定了 / 几个人 / 在互动 / 离开了多久
//     输出 = 行为状态 + 建议播放模式 + 置信度 + 停留时长 + 互动次数
//
// 【为什么 M5 是"副板真正有用"的那一步】
//   M1~M4 只是把画面变成数字；M5 才第一次给出主板能用的决策依据。
//   但注意方案 §2 的结论：**副板只给建议，最终裁决权在主板**——
//   副板不知道媒体清单/播放进度/内容分级，报错时主板要能降级到
//   定时循环而不是黑屏，所以这里只出 (state, mode, conf) 三元组。
//
// 状态行 193 字节：
//   MH5 K N=xxxxxx F=xxxxxx M=xxxxxx P=x S=xxx E=xxxx G=xxx Y=xxx L=x
//     T=x Q=x C=xx D=xxxx I=xx U=x V=<94 hex>
//     （前缀与 M4 **逐字段对齐**，唯一区别是版本标识 '4'→'5'，这样
//       同一套日志解析脚本不用改就能同时读 M4/M5；新增的 29 字节
//       M5 字段（66..94）紧接在 L=x 之后、'V=' 标签之前，曲线仍在
//       行尾 94 字符。字节账：66(M4 公共前缀，含尾随空格)
//       + 29(M5 字段) + 2("V=") + 94(曲线) + 2(CRLF) = 193）
//     K = SCCB 自检（cam_ok）      N = 本帧有效像素数（= 90240）
//     F = 形态学前前景像素数        M = 形态学后前景像素数
//     P = 人数估计（瞬时）          S = 占用列数
//     E = 当前曝光（R0x0B 粗快门）  G = 当前模拟增益（R0x35）
//     Y = 本帧平均灰度              L = 曝光锁定标志
//     T = 行为状态（0=IDLE 1=PRESENCE 2=SINGLE 3=MULTI 4=INTERACT
//                   5=LEAVE 6=ALERT）
//     Q = 建议播放模式（0=STANDBY 1=SINGLE 2=MULTI 3=INTERACT 4=ALERT）
//     C = 置信度 0..254（≥128 建议采信，<64 建议忽略）
//     D = 停留帧数（60 帧 = 1 s）   I = 互动次数
//     U = 死区滤波后人数（与 P 对照看，差值就是死区正在吸收的抖动）
//     V = 列投影曲线，94 个 hex 字符（与 M3/M4 同义）
//
// 时钟域与 M1~M4 相同：sys_clk 50MHz（SCCB/串口/上报/曝光闭环/行为状态机）
// + cam_pclk≈13.5MHz（DVP 采集 + 全部像素流水线，单域内完成）。
// 跨时钟域仍是"翻转握手 + 准静态快照"，M5 复用 M4 已经建立的
// frame_tick（每帧一拍，落在垂直消隐期）作为状态机节拍——不新增 CDC 通路。
// ============================================================
`include "vision_def.v"

module top_vision_m5
(
	input                       sys_clk,     // R7, 50 MHz
	input                       rst_n,       // KEY1(A2)，低有效
	// ---- 摄像头（J1）----
	input                       cam_pclk,    // D14
	input                       cam_href,    // L14
	input                       cam_vsync,   // M14
	input[7:0]                  cam_d,       // [0..7] = G11 G12 F13 H13 H14 J14 J13 K12
	output                      cam_scl,     // P11
	inout                       cam_sda,     // L10
	// ---- 调试串口（板载 CH340）----
	output                      uart_tx,     // D12
	input                       uart_rx,     // F12
	// ---- LED ----
	output[3:0]                 led          // A4 A3 C10 B12
);

localparam LINE_LEN = `M5_LINE_LEN;
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
//   （与 M4 完全一致：cfg_busy 期间归 cfg，否则归 auto_exp）
// ------------------------------------------------------------
wire        cfg_req;
wire        cfg_rw;
wire[15:0]  cfg_addr;
wire[15:0]  cfg_wdata;
wire        sccb_busy;
wire        sccb_ack;
wire[15:0]  sccb_rdata;
wire[4:0]   sccb_nack;
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
// 帧指纹（几何回归，N 字段的来源；与 M1~M4 同源，保证 N 可比）
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
// 帧亮度计量（PCLK 域，M4）
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
// ------------------------------------------------------------
wire[15:0] exp_shut_v;
wire[15:0] exp_gain_v;
wire[7:0]  exp_mean_v;
wire       exp_locked_v;
wire[15:0] exp_wr_cnt;

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
// ---- M5 新增快照（纯增量，不动上面任何字段，M1~M4 的 N/F/M/P/S 逐字节不变）----
reg[6:0]         snap_ctr;             // 质心列箱 = (cf + cl) >> 1
reg[6:0]         snap_cf;
reg[6:0]         snap_cl;
reg[7:0]         snap_occ;             // 占用列箱数（M5 判警戒区用）
reg[16:0]        snap_nrg;             // 形态学后前景像素数（M5 判能量突增用）

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
		snap_ctr   <= 7'd0;
		snap_cf    <= 7'd0;
		snap_cl    <= 7'd0;
		snap_occ   <= 8'd0;
		snap_nrg   <= 17'd0;
	end
	else if(snap_ld == 1'b1)
	begin
		snap_cnt   <= f_cnt;
		snap_fg    <= g_cnt;
		snap_m     <= m_cnt;
		snap_ppl   <= s_ppl;
		snap_ocs   <= {s_occ, 2'b00};
		snap_curve <= curve_bits;
		snap_ctr   <= (s_cf + s_cl) >> 1;
		snap_cf    <= s_cf;
		snap_cl    <= s_cl;
		snap_occ   <= s_occ;
		snap_nrg   <= m_cnt;
	end
end

// ------------------------------------------------------------
// 跨时钟域 2：帧亮度锁存翻转 → sys 域边沿 → 亮度快照 + frame_tick
// ------------------------------------------------------------
reg[2:0] mt_sync = 3'b000;
always@(posedge sys_clk)
	mt_sync <= {mt_sync[1:0], e_tog};

wire meter_edge = mt_sync[2] ^ mt_sync[1];

reg[3:0] mt_dly;
always@(posedge sys_clk or posedge rst_sys)
begin
	if(rst_sys)
		mt_dly <= 4'd0;
	else
		mt_dly <= {mt_dly[2:0], meter_edge};
end

wire exp_ld     = mt_dly[2];           // 抓亮度快照
assign frame_tick = mt_dly[3];           // 快照稳后再发决策脉冲（M5 也用它）

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
// M5：行为状态机（sys 域，帧节拍 = frame_tick）
//   输入全部取自上面两组快照，因此 M5 与 M4 共享同一个帧节拍，
//   不会出现"状态机看到的是第 n 帧、曝光看到的是第 n+1 帧"的错位。
//   geom_ok 用亮度计量的像素数判（每帧都为 90240；画面几何坏掉时
//   状态机冻结，见 behavior_fsm.v 的说明）。
// ------------------------------------------------------------
wire[2:0]  b_st;
wire[2:0]  b_mode;
wire[7:0]  b_conf;
wire[15:0] b_dwell;
wire[7:0]  b_icnt;
wire[3:0]  b_pplf;
wire       b_ev_enter;
wire       b_ev_leave;
wire       b_ev_interact;
wire[1:0]  b_dir;
wire       b_push;

behavior_fsm u_beh
(
	.clk          (sys_clk),
	.rst          (rst_sys),
	.frame_tick   (frame_tick),
	.geom_ok      (snap_ecnt == `CAM_FRAME_PIX),
	.ppl_i        (snap_ppl),
	.ctr_i        (snap_ctr),
	.cf_i         (snap_cf),
	.cl_i         (snap_cl),
	.occ_i        (snap_occ),
	.nrg_i        (snap_nrg),
	.st_o         (b_st),
	.mode_o       (b_mode),
	.conf_o       (b_conf),
	.dwell_o      (b_dwell),
	.icnt_o       (b_icnt),
	.pplf_o       (b_pplf),
	.ev_enter_o   (b_ev_enter),
	.ev_leave_o   (b_ev_leave),
	.ev_interact_o(b_ev_interact),
	.dir_o        (b_dir),
	.push_o       (b_push)
);

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
reg[CURVE_W-1:0] curve_sh;
// ---- M5 字段 ----
reg[2:0]         prt_st;        // T 字段
reg[2:0]         prt_mode;      // Q 字段
reg[7:0]         prt_conf;      // C 字段
reg[15:0]        prt_dwell;     // D 字段
reg[7:0]         prt_icnt;      // I 字段
reg[3:0]         prt_pplf;      // U 字段

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

// 固定字符表。M4 的 0..67 一个都没动，M5 新字段落在 68..96；
// 曲线整段后移到 97..190，CR/LF 落在 191/192。
function[7:0] fixed_char;
	input[7:0] i;
	begin
		case(i)
			8'd0:   fixed_char = 8'h4D;   // 'M'
			8'd1:   fixed_char = 8'h48;   // 'H'
			8'd2:   fixed_char = 8'h35;   // '5'（M5 版本标识；其余前缀与 M4 逐字节一致）
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
			// ---- M5 新字段（65..96，共 32 字节；M4 的 'V=' 从 66/67 挪到 95/96）----
			8'd66:  fixed_char = 8'h54;   // 'T'
			8'd67:  fixed_char = 8'h3D;   // '='
			8'd69:  fixed_char = 8'h20;
			8'd70:  fixed_char = 8'h51;   // 'Q'
			8'd71:  fixed_char = 8'h3D;
			8'd73:  fixed_char = 8'h20;
			8'd74:  fixed_char = 8'h43;   // 'C'
			8'd75:  fixed_char = 8'h3D;
			8'd78:  fixed_char = 8'h20;
			8'd79:  fixed_char = 8'h44;   // 'D'
			8'd80:  fixed_char = 8'h3D;
			8'd85:  fixed_char = 8'h20;
			8'd86:  fixed_char = 8'h49;   // 'I'
			8'd87:  fixed_char = 8'h3D;
			8'd90:  fixed_char = 8'h20;
			8'd91:  fixed_char = 8'h55;   // 'U'
			8'd92:  fixed_char = 8'h3D;
			8'd94:  fixed_char = 8'h20;
			8'd95:  fixed_char = 8'h56;   // 'V'
			8'd96:  fixed_char = 8'h3D;   // '='
			8'd191: fixed_char = 8'h0D;   // CR
			8'd192: fixed_char = 8'h0A;   // LF
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
		else if(i == 8'd68)                           // T = 1 hex（行为状态）
			line_byte = hexc({1'b0, prt_st});
		else if(i == 8'd72)                           // Q = 1 hex（建议模式）
			line_byte = hexc({1'b0, prt_mode});
		else if((i >= 8'd76) && (i <= 8'd77))         // C = 2 hex（置信度）
		begin
			nb = 4'd1 - (i - 8'd76);
			vtmp = {24'd0, prt_conf};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if((i >= 8'd81) && (i <= 8'd84))         // D = 4 hex（停留帧数）
		begin
			nb = 4'd3 - (i - 8'd81);
			vtmp = {16'd0, prt_dwell};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if((i >= 8'd88) && (i <= 8'd89))         // I = 2 hex（互动次数）
		begin
			nb = 4'd1 - (i - 8'd88);
			vtmp = {24'd0, prt_icnt};
			line_byte = hexc(vtmp[nb*4 +: 4]);
		end
		else if(i == 8'd93)                           // U = 1 hex（滤波后人数）
			line_byte = hexc(prt_pplf);
		else if((i >= `M5_CURVE_POS) && (i <= `M5_CURVE_END))   // 曲线：94 个 nibble
			line_byte = hexc(curve_sh[3:0]);
		else if(i == 8'd4)                            // 自检标志
			line_byte = prt_ok ? 8'h4B : 8'h46;       // 'K' / 'F'
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
		curve_sh <= {CURVE_W{1'b0}};
		prt_st   <= 3'd0;
		prt_mode <= 3'd0;
		prt_conf <= 8'd0;
		prt_dwell<= 16'd0;
		prt_icnt <= 8'd0;
		prt_pplf <= 4'd0;
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
					curve_sh <= snap_curve;   // 曲线的 94 个 nibble 一次性装填
					// M5：状态机输出与快照同时锁存 → 整行自洽
					prt_st   <= b_st;
					prt_mode <= b_mode;
					prt_conf <= b_conf;
					prt_dwell<= b_dwell;
					prt_icnt <= b_icnt;
					prt_pplf <= b_pplf;
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
					if((xpos >= `M5_CURVE_POS) && (xpos <= `M5_CURVE_END))
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
//   led[3] = 曝光已锁定（M4 证据）
//   M5 的验收证据是状态行里的 T=/Q=/C= 三个字段（"有人→T=2 C 上升"），
//   板载 LED 只有 4 颗，M4 已经占满，故不新增——这也是"里程碑冻结"
//   原则的体现：不为新里程碑去改旧里程碑的判读方式。
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
// 显式标记未使用的信号，避免综合告警（宽度 271 bit，逐项列全）
// ------------------------------------------------------------
wire[270:0] unused_bus;
assign unused_bus = {sccb_nack, rx_data, rx_valid, wr_idx, err_cnt, cfg_busy,
                     f_sum, f_min, f_max, ver_rd, col_cnt, row_cnt,
                     frame_valid, line_start, frame_cnt, g_nz,
                     g_minx, g_maxx, g_miny, g_maxy, f_done_tog, g_done_tog,
                     s_cf, s_cl, s_cp, s_cpb, s_rf, s_rl, s_row_nz,
                     exp_wr_cnt,
                     // ---- M5：事件脉冲与方向留给 M6 的事件包，暂未消费 ----
                     b_ev_enter, b_ev_leave, b_ev_interact, b_dir, b_push};

endmodule
