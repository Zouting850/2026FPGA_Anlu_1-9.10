// ============================================================
// people_seg.v
// M3 人数分段：吃 projection 的直方图读出流，吐人数与若干证据量
//
// 输入是 projection 每帧读出的 154 个箱值（94 列箱 + 60 行箱），
// 一拍一个。输出全部是寄存器，读出过程结束即稳定，顶层直接快照。
//
// 【列相位：阈值 → 行程分段】
//   1) 二值化：箱值 > SEG_COL_THRESH 才算"这一箱有人"；
//   2) 行程（run）分段：
//        - 连续被占用的箱连成一段；
//        - 段内允许有 < SEG_MIN_GAP 箱的空洞（人形轮廓里手臂/两腿之间的
//          凹陷会让某几箱变低），空洞不够宽就合上；
//        - 段宽 < SEG_MIN_WIDTH 箱的一律丢弃（残余噪点、反光点）；
//        - 存活段计数 = 人数估计 ppl。
//   3) 顺带记证据：占用箱数 occ、首/末占用箱 cf/cl、峰值箱值 cp 及所在箱 cpb。
//      板上"峰值数与人工目视一致"就靠 ppl + 曲线（curve_bits）一起判。
//
//   人数 ±1 死区、进入/离开的滞回与确认，属于 M5 行为状态机的职责，
//   这里只出"当前这一帧看起来有几个人"的瞬时值。
//
// 【行相位：上下边界】
//   行箱同理按 SEG_ROW_THRESH 二值化，只保留首/末占用行箱 rf/rl
//   （人站在画面里的上下范围），供后续里程碑画框/判越线用。
//
// 【相位切换的收尾】
//   列的最后一个箱处理完，可能还留着一个"开着"的段（画面最右边有人）。
//   它在下一拍（行相位第一个箱，ro_phase=1 且 ro_bin=0）统一收尾：
//   够宽就计数。这个收尾条件用读出流本身就能判，不需要额外端口。
// ============================================================
`include "vision_def.v"

module people_seg
(
	input                       clk,          // PCLK 域
	input                       rst_n,
	// ---- projection 直方图读出流 ----
	input                       ro_valid,
	input                       ro_phase,     // 0 = 列箱，1 = 行箱
	input[6:0]                  ro_bin,
	input[`PROJ_RW-1:0]         ro_val,
	// ---- 帧边界（用于清空本帧累加器）----
	input                       frame_start,
	// ---- 结果（读出结束即稳定）----
	output reg[3:0]             ppl_o,        // 人数估计（段数，饱和 15）
	output reg[7:0]             occ_o,        // 占用列箱数（0..94）
	output reg[6:0]             cf_o,         // 首个占用列箱
	output reg[6:0]             cl_o,         // 末个占用列箱
	output[`PROJ_RW-1:0]        cp_o,         // 列峰值（箱值）
	output reg[6:0]             cpb_o,        // 列峰值所在箱
	output reg[5:0]             rf_o,         // 首个占用行箱
	output reg[5:0]             rl_o,         // 末个占用行箱
	output reg                  row_nz_o      // 行相位是否有任何占用
);

localparam[7:0] COL_N = `PROJ_COL_BINS;

// ------------------------------------------------------------
// 相位标志
// ------------------------------------------------------------
wire flush = ro_valid & ro_phase & (ro_bin == 7'd0);   // 列相位收尾那一拍

// ------------------------------------------------------------
// 列相位累加器
// ------------------------------------------------------------
reg        run_open;      // 当前有一个段开着
reg        run_ok;        // 该段已够宽
reg[6:0]   run_start;
reg[3:0]   gap_cnt;

reg[3:0]   ppl;
reg[7:0]   occ;
reg        cf_nz;
reg        cl_nz;
reg[6:0]   cf;
reg[6:0]   cl;
reg[10:0]  cp;
reg[6:0]   cpb;

// 行相位累加器
reg        row_nz;
reg[5:0]   rf;
reg[5:0]   rl;

wire col_occ = (ro_val > `SEG_COL_THRESH);
wire row_occ = (ro_val > `SEG_ROW_THRESH);

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		run_open <= 1'b0;
		run_ok   <= 1'b0;
		run_start<= 7'd0;
		gap_cnt  <= 4'd0;
		ppl      <= 4'd0;
		occ      <= 8'd0;
		cf_nz    <= 1'b0;
		cl_nz    <= 1'b0;
		cf       <= 7'd0;
		cl       <= 7'd0;
		cp       <= 11'd0;
		cpb      <= 7'd0;
		row_nz   <= 1'b0;
		rf       <= 6'd0;
		rl       <= 6'd0;
	end
	else if(frame_start == 1'b1)
	begin
		run_open <= 1'b0;
		run_ok   <= 1'b0;
		run_start<= 7'd0;
		gap_cnt  <= 4'd0;
		ppl      <= 4'd0;
		occ      <= 8'd0;
		cf_nz    <= 1'b0;
		cl_nz    <= 1'b0;
		cf       <= 7'd0;
		cl       <= 7'd0;
		cp       <= 11'd0;
		cpb      <= 7'd0;
		row_nz   <= 1'b0;
		rf       <= 6'd0;
		rl       <= 6'd0;
	end
	else
	begin
		// ---------- 列相位收尾（行相位第一拍）----------
		if(flush == 1'b1)
		begin
			if((run_open == 1'b1) && (run_ok == 1'b1))
				ppl <= (ppl == 4'd15) ? 4'd15 : (ppl + 4'd1);
			run_open <= 1'b0;
			run_ok   <= 1'b0;
			gap_cnt  <= 4'd0;
		end

		// ---------- 列箱 ----------
		if((ro_valid == 1'b1) && (ro_phase == 1'b0))
		begin
			if(col_occ == 1'b1)
			begin
				occ   <= occ + 8'd1;
				cl_nz <= 1'b1;
				cl    <= ro_bin;
				if(cf_nz == 1'b0)
				begin
					cf_nz <= 1'b1;
					cf    <= ro_bin;
				end
				if(ro_val > cp)
				begin
					cp  <= ro_val;
					cpb <= ro_bin;
				end

				// 段
				if(run_open == 1'b0)
				begin
					run_open  <= 1'b1;
					run_start <= ro_bin;
					// 打开段的这一拍先按"最小宽度是否只有 1 箱"定初值；
					// 否则会拿上一段的 run_start 去算本段宽度，误置 run_ok
					run_ok    <= (`SEG_MIN_WIDTH <= 4'd1) ? 1'b1 : 1'b0;
				end
				else
				begin
					gap_cnt  <= 4'd0;
					if(((ro_bin - run_start) + 7'd1) >= {3'd0, `SEG_MIN_WIDTH})
						run_ok <= 1'b1;
				end
			end
			else
			begin
				if(run_open == 1'b1)
				begin
					if(gap_cnt == (`SEG_MIN_GAP - 4'd1))
					begin
						if(run_ok == 1'b1)
							ppl <= (ppl == 4'd15) ? 4'd15 : (ppl + 4'd1);
						run_open <= 1'b0;
						run_ok   <= 1'b0;
						gap_cnt  <= 4'd0;
					end
					else
						gap_cnt <= gap_cnt + 4'd1;
				end
			end
		end

		// ---------- 行箱 ----------
		if((ro_valid == 1'b1) && (ro_phase == 1'b1))
		begin
			if(row_occ == 1'b1)
			begin
				row_nz <= 1'b1;
				rl     <= ro_bin[5:0];
				if(row_nz == 1'b0)
					rf <= ro_bin[5:0];
			end
		end
	end
end

// ------------------------------------------------------------
// 输出
// ------------------------------------------------------------
assign cp_o = cp;

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		ppl_o    <= 4'd0;
		occ_o    <= 8'd0;
		cf_o     <= 7'd0;
		cl_o     <= 7'd0;
		cpb_o    <= 7'd0;
		rf_o     <= 6'd0;
		rl_o     <= 6'd0;
		row_nz_o <= 1'b0;
	end
	else
	begin
		ppl_o    <= ppl;
		occ_o    <= occ;
		cf_o     <= cf;
		cl_o     <= cl;
		cpb_o    <= cpb;
		rf_o     <= rf;
		rl_o     <= rl;
		row_nz_o <= row_nz;
	end
end

endmodule
