// ============================================================
// bg_store.v
// M2 背景帧存储 —— 片内 BRAM 单缓冲
//
// 为什么这里不用 SDRAM：
//   背景建模只需要保存"一份背景"。因为像素是**按光栅顺序流式**处理的
//   —— 同一个位置每帧只访问一次（读出旧背景、就地写回新背景），所以
//   376x240x8bit = 88KB 的单缓冲就够，既不需要三缓冲，也不需要 SDRAM
//   控制器 IP。SDRAM 三缓冲留给需要整帧随机访问的场合（M1b）。
//
// 时序（简单双口 BRAM：一口读、一口写）：
//   rd_en 拉高后 rd_data 在**下一拍**有效（1 拍读延迟）；
//   读写可以同拍发生在不同地址，所以流水线能边读边写。
//
// 本模块的读写地址在同一拍内必然不同（读 idx、写 idx-1），不会出现
// 同地址读写冲突，因此不依赖 BRAM 的 write-first / read-first 模式。
//
// 注意：存储阵列**不加复位**，否则 TD 无法推断 ERAM9K。上电内容未定义，
// 由 m2_engine 的"背景装载阶段"负责在前若干帧把 bg 灌成当前画面。
// ============================================================
`include "vision_def.v"

module bg_store
(
	input                       clk,
	// 读口
	input                       rd_en,
	input[16:0]                 rd_addr,      // 17 bit：90240 < 2^17
	output reg[7:0]             rd_data,
	// 写口
	input                       wr_en,
	input[16:0]                 wr_addr,
	input[7:0]                  wr_data
);

localparam DEPTH = `CAM_FRAME_PIX;      // 90240

reg[7:0] mem[0:DEPTH-1];

always@(posedge clk)
begin
	if(wr_en == 1'b1)
		mem[wr_addr] <= wr_data;
	if(rd_en == 1'b1)
		rd_data <= mem[rd_addr];
end

endmodule
