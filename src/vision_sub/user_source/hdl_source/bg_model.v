// ============================================================
// bg_model.v
// M2 逐像素：绝对差 + 前景判定 + 背景更新
//
// 输入同一像素的当前值 cur 与背景值 bg，输出：
//   fg_diff = |cur - bg|                 绝对差幅度（0..255）
//   fg      = (fg_diff > FG_THRESH)      1 = 前景
//   bg_next = 更新后的背景（在同一像素上）：
//             want = fg ? 1 : max(1, fg_diff >> BG_SHIFT)
//             step = min(want, fg_diff)          ← 钳位，见下
//             bg_next = upd_en ? bg ± step : bg
//
// **全无符号运算，且 bg_next 恒落在 [bg, cur]**：
//   cur >= bg 时 step <= cur-bg，故 bg+step <= cur <= 255；
//   cur <  bg 时 step <= bg-cur，故 bg-step >= cur >= 0。
//
// 两个关键点：
//   1) max(1, ·) 消掉移位积分器的死区：只有 d>>SHIFT 的话，d 很小时 step
//      恒为 0，背景会永久停在离 cur 几级的地方。
//   2) **但 max(1,·) 必须再钳到 d**（step = min(want, d)）。否则 d=0 时
//      step=1，bg 会冲到 cur±1；cur=bg=255 时 bg_next = 256，8bit 回绕成
//      0，下一帧 d=255，凭空造出满屏假前景。钳到 d 后：d=0 不动，d>=1 时
//      仍 1 级/帧地收敛（死区照样消除），且永不越界。
//
// 注：在当前常数下 FG_THRESH(24) <= 2*2^BG_SHIFT(32)，背景像素 d<=24 使得
// d>>SHIFT 只可能是 0 或 1，于是 step 恒为 1 —— BG_SHIFT 的指数项暂时不起
// 作用，只有把 FG_THRESH 提到 2^BG_SHIFT 以上才会显现。
//
// fg_tick：前景像素"每 FG_DIV 帧才允许走 1 级"的全局使能。
// ============================================================
`include "vision_def.v"

module bg_model
(
	input[7:0]                  cur,
	input[7:0]                  bg,
	input                       fg_tick,
	output[7:0]                 fg_diff,
	output                      fg,
	output[7:0]                 bg_next
);

wire        cur_ge = (cur >= bg);
wire[7:0]   d      = cur_ge ? (cur - bg) : (bg - cur);
wire        is_fg  = (d > `FG_THRESH);

// 目标步长：背景像素走 d>>SHIFT（至少 1），前景像素每 tick 走 1
wire[7:0]   want   = is_fg ? 8'd1 :
                     (((d >> `BG_SHIFT) == 8'd0) ? 8'd1 : (d >> `BG_SHIFT));
// 钳到 d：d=0 时不动，避免 bg 冲到 cur±1（d=0&bg=255 时会回绕成 256->0）
wire[7:0]   step   = (want > d) ? d : want;
wire        upd_en = is_fg ? fg_tick : 1'b1;

assign fg_diff = d;
assign fg      = is_fg;
assign bg_next = upd_en ? (cur_ge ? (bg + step) : (bg - step)) : bg;

endmodule
