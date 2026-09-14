
module top(
    input                       clk,
    input                       rst_n,
    input                       key1,           // 手动下一张
    input                       key2,           // 自动播放 开/关
    input                       key3,           // 亮度档位循环
    input       [3:0]           sw,             // 拨码开关：sw[2:0] (SW1-3) 选转场特效，sw[3] (SW4) 屏蔽滚动字幕（ON=隐藏，与 SW1-3 极性相反）
    input                       uart_rx,        // 串口屏 -> FPGA，D14 (J1 pin1)，4P TTL 飞线（PULLUP）
    output                      uart_tx,        // FPGA -> 串口屏，G11 (J1 pin2)，飞线；Stage 1 恒为空闲高
    input                       uart_pc_rx,     // PC(Type-C/CH340) -> FPGA，F12；第二路独立 UART，与 J1 串口屏无关
    input                       emg_in_manual,  // 应急硬线 T1：J2 pin5 = M3，手报按钮，PULLUP，动作拉低
    input                       emg_in_auto,    // 应急硬线 T2：J2 pin6 = M4，烟感/温感回路，PULLUP，动作拉低
    output                      spk,            // 无源蜂鸣器 H11：方波驱动，通断受板上 SW5
    output      [3:0]           led,            // 链路诊断，高电平点亮，见下面「串口链路诊断 LED」

    output [5:0]                seg_sel,
    output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;
parameter BUF2_ADDR     = 24'd614400;
parameter BUF3_ADDR     = 24'd921600;
// Power-up audio source. 0 = the built-in 48 kHz test tone owns the audio output
// and the screen command "MUSC 1" switches to the TF-card WAV; 1 = play the WAV
// straight from power-up, which is the behaviour that was verified on the board
// before the serial screen existed. This is the only retreat path for the audio
// source switch -- there is no DIP switch or key gesture for it, so flipping this
// parameter and re-synthesising is how you get the old build back.
parameter AUDIO_SRC_DEFAULT = 1'b0;

// PC text-subtitle channel master switch. 1 = the second UART (uart_pc_rx, F12)
// feeds the marquee overlay so a PC can type an ASCII banner; 0 = pc_en is tied
// low, the ASCII render arm is never selected and the marquee overlay collapses
// to bit-identical with the build that predates this channel. This is the
// one-line retreat for the whole PC-text feature: flip to 0 and re-synthesise.
//
// Currently 0, and not because the channel is broken. The four-track audio table
// added 262 registers, taking the device from 88.09% to 92.10% slices, and at
// that density place-and-route deadlocked: two auto-duplicated high-fanout
// registers were packed into the same tile and both claimed wire segment
// x35y55_e2beg4 (PHY-8023, then RUN-8102 after 125 rip-up iterations failed to
// escape). Timing was never the problem -- place setup was +1395 ps with zero
// violated endpoints. Extension requirement (2) audio-video linkage is a named
// contest requirement and this channel is not, so this one pays. It is worth
// roughly 2000 le, against the ~376 slices the deadlock needed.
parameter PC_TEXT_ENABLE = 1'b0;

// Second command source: a PC talking the same TJC framed protocol into the
// on-board CH340 (Type-C -> F12), so the board is controllable before the serial
// screen is wired up. It is a second uart_screen_ctrl instance, not a change to
// the J1 one -- D14/G11 keep every function they have today. Measured: at the gate
// stage this instance is 216 lut / 126 seq / 0 BRAM / 0 DSP and syn #reg
// 7562 -> 7688 is exactly those 126; after place&route the hierarchy row reads
// 245 le / 210 lut / 137 seq, against 276 le / 224 lut / 123 seq for the J1 twin.
// Setting this to 0 collapses the whole pc_cmd_* bundle to constants, which leaves
// the second instance with zero loads and gets it pruned -- probed on a real run:
// the row disappears and #reg returns to exactly 7562. See the merge point below
// for why the gating covers the command *values* and not just the strobes.
parameter PC_CMD_ENABLE = 1'b1;

// Emergency takeover master switch: the whole 应急发布 feature -- the fusion
// block, the full-screen alarm layer, the siren, the buzzer and the command
// freeze -- hangs off this one bit. It is the one-line retreat for a feature that
// touches four clock domains, and it defaults to 1 because extension requirement
// (3) names emergency information dissemination outright.
//
// The gate is on every load of all three outputs (emg_state / emg_tgl /
// vol_level), not just on the strobes. Gating only a strobe leaves the value bus
// with a live reader, synthesis keeps the register, and "one-line retreat" becomes
// a lie -- this project has already been caught doing exactly that on led[3].
//
// Input gating is NOT enough for a pure combinational layer. With this parameter at
// 0 and every input wire tied to a constant, u_emergency_ctrl, u_alarm_siren and
// u_buzzer_beep all left the area report -- but the alarm layer stayed, still
// holding 202 lut and 12 seq, because its frame counter and raster tracker are
// free-running off I_de and no amount of constant propagation on I_en gets them
// removed. That reading came off the gated-form report during this probe and has
// since been overwritten; the two reports that survive (below) prove the fixed
// shape. Re-producing the leak is one line: delete this guard and re-synthesise at
// 0. The deletion is structural, not a hope about what the tool folds.
//
// Proof stays the area hierarchy row, never the parameter value: at 0 all four
// rows must be absent from syn_1/*_gate.area -- u_emergency_ctrl, u_buzzer_beep,
// and the two generate-scoped ones, which the report names g_alarm_overlay$
// u_alarm_overlay and g_siren$u_alarm_siren (the whole scope disappears, not just
// the instance). tools/sim_emergency.py Pass H asserts the structural half of that
// against this file's text; the four rows close the other.
//
// The siren is behind a generate for a different reason: it deletes itself when
// gated, but the gate leaves S_phase_acc[31] without a driver and TD reports a
// fresh HDL-5314. A retreat build must warn exactly as loudly as the shipped one,
// or no later number is readable: measured 56 each, equal once "(NNNN)" is cut.
//
// Both halves are on disk, because the report is overwritten by the next build and
// a number nobody can re-read is not evidence: _build_logs/area_emg1_20260913_214421.txt
// (this value, four rows present, #reg 7869) and area_emg0_retreat_20260913_215258.txt
// (at 0, four rows absent, #reg 7692, still exactly 56 warnings).
parameter EMERGENCY_ENABLE = 1'b1;

// Carousel interval out of reset, in whole seconds, until a SPED command
// overrides it. It is passed down to sd_card_bmp so the clk-domain latch and the
// sd_card_clk-domain counter come out of reset already agreeing. 1 is not an
// arbitrary default: it is the interval this design has always had, and the only
// one that is self-consistent with video_transition.v's durations (see the
// comment there about 12/13/14 being excluded). Never sending SPED is the
// zero-traffic retreat and costs nothing; changing this parameter is how you
// move the power-on interval itself.
parameter AUTO_SEC_DEFAULT = 3'd1;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_eff;
wire [23:0] vout_data_raw;
wire [23:0] vout_data_base;
wire [23:0] vout_data_bright;
wire [23:0] vout_data_fade;
wire [23:0] vout_data_audio;
wire [23:0] vout_data_osd;
wire [23:0] vout_data;
wire        marquee_en;
wire        display_valid;
wire        auto_play_enabled;
wire        key3_bright_press;
wire        video_frame_start;

// Stage 4/5 transition controller outputs, all video_clk domain. bot/top are the
// two buffer selectors frame_fifo_read turns into read base addresses; they are
// equal except while a band effect is revealing the new picture. trans_effect is
// the band code 1..6 or 8..B that tells frame_fifo_read's select_top which sweep
// shape to draw, and is 0 when idle or fading.
wire [1:0]  trans_bot_idx;
wire [1:0]  trans_top_idx;
wire [3:0]  trans_effect;
wire [1:0]  trans_img_idx;
wire [3:0]  trans_fade_level;
// Transition mode select. The physical path inverts the active-low DIP switches
// (ON connects the pin to GND) to an intuitive ON=1 mode: 000 auto-cycle,
// 001..110 force one band effect, 111 force fade. The serial screen can override
// it through a mux at the assignment below; with no screen command the override
// is off and trans_mode is exactly ~sw_v1 zero-extended as before. sw[3] (SW4) is
// the banner mask, handled separately. Codes 8..F are reachable only from the
// serial screen -- the DIP path can never produce them, so the verified physical
// behaviour is bit-for-bit unchanged. Assigned below, next to sw_v1's declaration:
// initializing it here made TD warn HDL-5373 (used before declaration) and risked
// binding a 1-bit implicit net.
wire [3:0]  trans_mode;

wire [3:0]  state_code;
wire [6:0]  seg_data_0;
// Audio bring-up readout on the three 7-segment digits left of state_code.
// chain = {wav_found, audio_phase, streamer ever wrote, header magic rejected},
// wr = peak audio FIFO occupancy on the sd_card_clk side, rd = peak occupancy on
// the video_clk side. A non-zero wr with a zero rd pins the fault to the CDC.
// chain == 8 (WAV found, streamer not armed) is the EXPECTED power-up reading
// now that the test tone is the default audio source -- it means the screen has
// not sent "MUSC 1" yet, not that anything is broken. bit1 is sticky across a
// source switch, so once music has played it stays set even after "MUSC 0" puts
// the tone back: it reports history, never "music is playing right now".
wire [3:0]  dbg_audio_chain;
wire [3:0]  dbg_aud_wr_peak;
wire [3:0]  dbg_gate;
wire [3:0]  dbg_loaded_cnt;
wire [3:0]  dbg_fail;
wire [3:0]  dbg_found_cnt;
wire [3:0]  dbg_next_idx;
wire [6:0]  seg_data_aud_chain;
wire [6:0]  seg_data_gate;
wire [6:0]  seg_data_cnt;
wire [6:0]  seg_data_fail;
wire [6:0]  seg_data_found;
wire [6:0]  seg_data_next;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
// Write-side FIFO occupancy, routed from frame_read_write to the scaler inside
// sd_card_bmp so backpressure never has to leave the sd_card_clk domain.
wire [8:0]  sd_card_write_fifo_usedw;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;
reg  [1:0]  disp_buf_idx_v0;
reg  [1:0]  disp_buf_idx_v1;
reg  [3:0]  state_code_v0;
reg  [3:0]  state_code_v1;
reg         auto_play_v0;
reg         auto_play_v1;
reg         display_valid_v0;
reg         display_valid_v1;
reg  [2:0]  brightness_level;
reg  [2:0]  brightness_level_v0;
reg  [2:0]  brightness_level_v1;
reg  [2:0]  sw_v0;
reg  [2:0]  sw_v1;
reg         sw4_v0;
reg         sw4_v1;
reg         vs_d;

// ---- 应急接管 ----
// emg_state / emg_tgl / vol_level are the three outputs of the fusion block, and
// every one of them is read here only through an EMERGENCY_ENABLE gate, so a
// retreated build leaves the instance with zero loads (see the parameter above).
wire [2:0]  emg_state_raw;
wire        emg_tgl_raw;
wire [1:0]  vol_level_raw;
wire [2:0]  emg_state;
wire        emg_tgl;
wire [1:0]  vol_level;
wire        emg_hold;              // clk 域：告警期间冻结所有普通命令
wire [23:0] vout_data_alarm;

// Two crossings out of the clk domain, deliberately different, and the difference
// is the deliverable: sound must not wait for a frame boundary, pixels must not
// change before one. tools/sim_emergency.py Pass A measures both latencies.
//
// The buzzer is not fed from here at all. It has no reason to cross: buzzer_beep
// runs on clk, the same domain emergency_ctrl publishes in, so its path is one
// synchroniser shorter than this one. Only the siren needs the video_clk copy,
// because a PCM sample has to be muxed into a video_clk stream.
reg         emg_act_s0,  emg_act_v1;        // 裸 2FF -> 警笛 arm 选择（快路径）
reg  [1:0]  emg_type_s0, emg_type_v1;       // 同一条快路径上的类型码 -> 警笛节奏
reg         emg_tgl_s0,  emg_tgl_s1, emg_tgl_s2;
reg  [2:0]  emg_state_stg;                  // data+toggle：toggle 边沿到达时锁一次
reg  [2:0]  emg_state_frame;                // 帧原子生效 -> alarm_overlay
reg  [1:0]  vol_s0,    vol_v1;              // 裸 2FF -> 三级数字音量
reg         emg_hold_s0, emg_hold_sd;       // 裸 2FF -> sd_card_clk，掐实体键

// ---- 串口屏控制：命令效果。两个源（J1 串口屏 D14、Type-C 上位机 F12）各自引出
// 一组 j1_cmd_* / pc_cmd_*，在两个实例之后的唯一合并点汇成下面的 cmd_*。合并点
// 以下的所有消费者（亮度合并、锁存块、toggle-CDC、2FF、cmd_any_set）都不知道
// 有两个源，一行未改。
wire        cmd_next_pulse;
wire        cmd_auto_pulse;
wire        cmd_bright_cycle_pulse;
wire [2:0]  cmd_bright_set;
wire        cmd_bright_set_v;
wire [3:0]  cmd_mode;
wire        cmd_mode_set;
wire        cmd_marquee;
wire        cmd_marquee_set;
wire [1:0]  cmd_img_sel;
wire        cmd_img_sel_set;
wire [3:0]  cmd_filt;
wire        cmd_filt_set;
wire        cmd_font;
wire        cmd_font_set;
wire        cmd_audio;
wire        cmd_audio_set;
wire [3:0]  cmd_speed;
wire        cmd_speed_set;
// ALRM / VOLM：合并点下面唯一**不**被 emg_hold 冻结的两条。告警期间必须仍然收得到
// 解除指令与音量指令，否则冻结就成了锁死。
wire [1:0]  cmd_emg;
wire        cmd_emg_set;
wire [1:0]  cmd_vol;
wire        cmd_vol_set;
wire        dbg_rx_toggle;
wire        dbg_rx_ff;

// 源 1：J1 串口屏（D14 / G11），原功能一字未改。
wire        j1_cmd_next_pulse;
wire        j1_cmd_auto_pulse;
wire        j1_cmd_bright_cycle_pulse;
wire [2:0]  j1_cmd_bright_set;
wire        j1_cmd_bright_set_v;
wire [3:0]  j1_cmd_mode;
wire        j1_cmd_mode_set;
wire        j1_cmd_marquee;
wire        j1_cmd_marquee_set;
wire [1:0]  j1_cmd_img_sel;
wire        j1_cmd_img_sel_set;
wire [3:0]  j1_cmd_filt;
wire        j1_cmd_filt_set;
wire        j1_cmd_font;
wire        j1_cmd_font_set;
wire        j1_cmd_audio;
wire        j1_cmd_audio_set;
wire [3:0]  j1_cmd_speed;
wire        j1_cmd_speed_set;
wire [1:0]  j1_cmd_emg;
wire        j1_cmd_emg_set;
wire [1:0]  j1_cmd_vol;
wire        j1_cmd_vol_set;

// 源 2：Type-C 上位机（F12），协议与 J1 那路逐字节相同。
wire        pc_cmd_next_pulse;
wire        pc_cmd_auto_pulse;
wire        pc_cmd_bright_cycle_pulse;
wire [2:0]  pc_cmd_bright_set;
wire        pc_cmd_bright_set_v;
wire [3:0]  pc_cmd_mode;
wire        pc_cmd_mode_set;
wire        pc_cmd_marquee;
wire        pc_cmd_marquee_set;
wire [1:0]  pc_cmd_img_sel;
wire        pc_cmd_img_sel_set;
wire [3:0]  pc_cmd_filt;
wire        pc_cmd_filt_set;
wire        pc_cmd_font;
wire        pc_cmd_font_set;
wire        pc_cmd_audio;
wire        pc_cmd_audio_set;
wire [3:0]  pc_cmd_speed;
wire        pc_cmd_speed_set;
wire [1:0]  pc_cmd_emg;
wire        pc_cmd_emg_set;
wire [1:0]  pc_cmd_vol;
wire        pc_cmd_vol_set;
wire        dbg_pc_cmd_toggle;

// mode/marquee 覆盖：clk 域锁存屏幕设定值，物理拨码一旦变动即清除覆盖
// （last-writer-wins 兜底，两端互为退路）。ovr_en=0 时下面的 trans_mode /
// marquee_en 退回已验证的物理项 ~sw_v1 / sw4_v1，逐位不变——没接屏幕时就是
// 改前的行为。
reg  [3:0]  mode_ovr_val;
reg         mode_ovr_en;
reg         marq_ovr_val;
reg         marq_ovr_en;
// 物理拨码同步到 clk 域并做变动检测（与 video_clk 域的 sw_v0/sw_v1、sw4_v0/sw4_v1
// 是各自独立的同步器，互不影响）
reg  [2:0]  sw_c0, sw_c1, sw_c2;
reg         sw4_c0, sw4_c1, sw4_c2;
// 覆盖状态再 2FF 同步进 video_clk，供 trans_mode / marquee_en 的 mux 使用
reg  [3:0]  mode_ovr_val_v0, mode_ovr_val_v1;
reg         mode_ovr_en_v0, mode_ovr_en_v1;
reg         marq_ovr_val_v0, marq_ovr_val_v1;
reg         marq_ovr_en_v0, marq_ovr_en_v1;

// FILT/FONT 是纯 UART 电平命令：SW1-3 已是转场、SW4 已是字幕屏蔽、key1/2/3 已
// 占用，没有物理项可并联，所以这里刻意不写 ovr_en、不写拨码夺回逻辑。复位值
// filt=0（直通）、font=0（平面），出复位即与加这两条命令之前逐位一致——退路
// 就是"从不发命令"。
reg  [3:0]  filt_val;
reg         font_val;
reg  [3:0]  filt_val_v0, filt_val_v1;
reg         font_val_v0, font_val_v1;
// filt 若在帧中途翻转，整幅画会出现"上半已处理、下半未处理"的横缝，所以这两个
// 新信号在 video_frame_start 再打一拍、帧原子生效（与 video_transition 在转场
// 起点采样 I_mode 是同一惯用法）。trans_mode / marquee_en 保持裸 2FF 不动。
reg  [3:0]  filt_frame;
reg         font_frame;

// 音频源选择（MUSC 命令）。和 FILT/FONT 同一惯用法：clk 域锁存，复位到
// AUDIO_SRC_DEFAULT，没有 ovr_en、没有拨码夺回——没有对应的物理开关，屏幕是唯一
// 入口，退路就是那个参数。
//
// 为什么裸 2FF 就够、不需要 filt_frame 那种帧原子锁存：两个源都产出连续不断的
// ~48 kHz audio_valid 流（测试音是 mclk 域 128 BCLK/帧 @ 6.144 MHz，播放器是
// 25 MHz 上的小数累加器、FIFO 空时也发静音）。audio_arc_calculate 只数 48 个
// valid 来配 CTS 节拍，所以切换最坏情况是一个采样点的相位不连续，ACR 参考永不
// 断流。帧原子锁存反而会让 sd_card_clk 侧的 music_req 晚一整帧才撤，白白多读一个
// 扇区。
reg         music_en;
reg         music_en_v0, music_en_v1;
reg         music_en_s0, music_en_s1;

// next/auto/img/speed 命令脉冲 clk -> sd_card_clk 的 toggle-CDC：clk 域每来一条命令
// 翻转一个 toggle，sd_card_clk 域 2FF 同步后用 s1^s2 还原成单周期脉冲。img 的 2-bit
// 目标值用 data+toggle 同步（数据准静态、人类速率，toggle 边沿到达时 img_sel_s1
// 已稳定 ≥2 拍）。speed 的 4-bit 轮播间隔走完全相同的 data+toggle，因为它同样是
// 人类速率的准静态值，而且同样必须在 toggle 边沿之前稳定——sd_card_bmp 会把它
// 减一后存进 sec_target_m1。
reg         next_tgl, auto_tgl, img_tgl, spd_tgl;
reg  [1:0]  img_sel_lat;
reg  [3:0]  speed_lat;
reg         next_tgl_s0, next_tgl_s1, next_tgl_s2;
reg         auto_tgl_s0, auto_tgl_s1, auto_tgl_s2;
reg         img_tgl_s0, img_tgl_s1, img_tgl_s2;
reg         spd_tgl_s0, spd_tgl_s1, spd_tgl_s2;
reg  [1:0]  img_sel_s0, img_sel_s1;
reg  [3:0]  speed_s0, speed_s1;
wire        cmd_next_pulse_sd    = next_tgl_s1 ^ next_tgl_s2;
wire        cmd_auto_pulse_sd    = auto_tgl_s1 ^ auto_tgl_s2;
wire        cmd_img_sel_pulse_sd = img_tgl_s1  ^ img_tgl_s2;
wire        cmd_speed_pulse_sd   = spd_tgl_s1  ^ spd_tgl_s2;

// PC 文字字幕通道 clk -> video_clk 的跨域。复用本文件里两个已验证的惯用法，不
// 新造原语：
//   - pc_text_en（1bit 准静态电平，PCTX 命令置起/清零）走裸 2FF，与 marq_ovr /
//     music_en 同一处理；
//   - pc_char_buf(32×7) + pc_n_cells(6bit) 走 data+toggle（与 img_sel 同一处理）：
//     文字是人类速率、准静态，toggle 边沿（pc_text_toggle，每次 TEXT 提交翻转）
//     到达 video_clk 时数据已稳定 ≥2 拍，pc_tgl_edge 把 staging 锁一次；
//   - 最后 staging 在 video_frame_start 帧原子锁进 frame 寄存器（与 filt_frame /
//     font_frame 同一纪律），保证一帧之内横幅文字恒定、不撕。
// pc_en_gated 是 PC 通道的总退路：PC_TEXT_ENABLE=0 时它恒 0，marquee_overlay 的
// ASCII 臂永不选中，整条链路与加这条通道之前逐位一致。
wire        pc_text_en;
wire [223:0] pc_char_buf;
wire [5:0]  pc_n_cells;
wire        pc_text_toggle;
wire        dbg_pc_rx_toggle;
wire        dbg_pc_commit_toggle;

reg         pc_en_v0, pc_en_v1;          // 裸 2FF on the quasi-static level
reg         pc_tgl_s0, pc_tgl_s1, pc_tgl_s2;  // 3FF on the commit toggle
wire        pc_tgl_edge = pc_tgl_s1 ^ pc_tgl_s2;
reg  [223:0] pc_buf_stg;                  // staging, latched on toggle edge
reg  [5:0]  pc_cells_stg;
reg         pc_en_frame;                  // frame-atomic working regs
reg  [223:0] pc_buf_frame;
reg  [5:0]  pc_cells_frame;
wire        pc_en_gated = PC_TEXT_ENABLE & pc_en_frame;

// 转场模式：屏幕覆盖优先，否则退回已验证的物理项 ~sw_v1（零扩展成 4 位，值域
// 仍是 0..7，逐位不变；8..F 只能从串口到达）
assign trans_mode = mode_ovr_en_v1 ? mode_ovr_val_v1 : {1'b0, ~sw_v1};

// SW4 masks the scrolling slogan banner. Deliberately the inverse polarity of
// SW1-3: those are mode selectors where ON=1 picks something, this one is a
// kill switch, and the all-OFF power-up state must still show the banner so
// the board demos out of the box. Pin is active-low with a PULLUP, so the raw
// synced value is 1 when SW4 is OFF, hence no inversion here. The serial screen
// can override the mask (marq_ovr_en_v1); otherwise the verified physical term
// sw4_v1 wins, so with no screen command this is exactly the old behaviour.
assign marquee_en = marq_ovr_en_v1 ? marq_ovr_val_v1 : sw4_v1;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 音频发射相关。audio_valid / audio_left_data / audio_right_data 不再
// 直接由某一个源驱动，而是下面 emg_act_v1 优先、再按 music_en_v1 二选一的 mux 输出，
// 并且程序臂过一次三级数字音量。
wire        audio_pll_lock;
wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
wire        tone_valid;
wire [23:0] tone_left;
wire [23:0] tone_right;
wire        mus_valid;
wire [23:0] mus_left;
wire [23:0] mus_right;
wire        siren_valid;
wire [23:0] siren_left;
wire [23:0] siren_right;
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

// Music playback CDC. sd_audio_stream (inside sd_card_bmp, sd_card_clk domain)
// writes {R,L} stereo frames into the async FIFO; audio_pcm_player (video_clk
// domain) reads them at 48 kHz. wrusedw is the streamer's sector-boundary
// backpressure, rdusedw tells the pacer whether the show-ahead head is live.
wire        aud_fifo_we;
wire [31:0] aud_fifo_di;
wire [8:0]  aud_fifo_wrusedw;
wire        aud_fifo_re;
wire [31:0] aud_fifo_dout;
wire [8:0]  aud_fifo_rdusedw;

wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// video_frame_start fires on the vsync edge as seen AFTER video_delay's 20 tap
// shift register, while video_timing_data raises read_req on the same edge seen
// before it. So this pulse lands about 20 video clocks after the read request
// that fetches the current frame, which is why video_transition treats an index
// change made here as taking effect on the NEXT frame.
assign video_frame_start = vs_d & ~vs;

// 统一复位：TF 图像链路 + HDMI 音频链路
wire rst_all;
assign rst_all = ~rst_n | ~audio_pll_lock;

// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (1'b0)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .reset      (1'b0)
);

// 复用你 I2S->HDMI 工程里的 PLL，只取 12.288MHz 音频主时钟
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

key_press_debounce #(
    .CLK_FREQ_HZ (50_000_000),
    .DEBOUNCE_MS (20)
) u_key_brightness (
    .clk        (clk),
    .rst        (rst_all),
    .button_in  (key3),
    .press_pulse(key3_bright_press)
);

always @(posedge clk or posedge rst_all) begin
    if (rst_all)
        brightness_level <= 3'd2;
    else if (cmd_bright_set_v)                              // 屏幕 BRGT n：直接设档（优先）
        brightness_level <= cmd_bright_set;
    else if (key3_bright_press || cmd_bright_cycle_pulse)   // key3 或屏幕 BRUP：循环 +1
        brightness_level <= (brightness_level == 3'd4) ? 3'd0 : (brightness_level + 3'd1);
end

// 串口屏控制源：clk 域把 UART 字节翻译成命令效果。所有跨域注入都由下面的
// toggle-CDC / 2FF 同步完成，本实例只在 clk 域产生效果，不含合并策略。
uart_screen_ctrl #(
    .CLK_FREQ_HZ (50_000_000),
    .BAUD        (9600)
) u_uart_screen_ctrl (
    .clk                    (clk),
    .rst                    (rst_all),
    .uart_rx                (uart_rx),
    .uart_tx                (uart_tx),
    .cmd_next_pulse         (j1_cmd_next_pulse),
    .cmd_auto_pulse         (j1_cmd_auto_pulse),
    .cmd_bright_cycle_pulse (j1_cmd_bright_cycle_pulse),
    .cmd_bright_set         (j1_cmd_bright_set),
    .cmd_bright_set_v       (j1_cmd_bright_set_v),
    .cmd_mode               (j1_cmd_mode),
    .cmd_mode_set           (j1_cmd_mode_set),
    .cmd_marquee            (j1_cmd_marquee),
    .cmd_marquee_set        (j1_cmd_marquee_set),
    .cmd_img_sel            (j1_cmd_img_sel),
    .cmd_img_sel_set        (j1_cmd_img_sel_set),
    .cmd_filt               (j1_cmd_filt),
    .cmd_filt_set           (j1_cmd_filt_set),
    .cmd_font               (j1_cmd_font),
    .cmd_font_set           (j1_cmd_font_set),
    .cmd_audio              (j1_cmd_audio),
    .cmd_audio_set          (j1_cmd_audio_set),
    .cmd_speed              (j1_cmd_speed),
    .cmd_speed_set          (j1_cmd_speed_set),
    .cmd_emg                (j1_cmd_emg),
    .cmd_emg_set            (j1_cmd_emg_set),
    .cmd_vol                (j1_cmd_vol),
    .cmd_vol_set            (j1_cmd_vol_set),
    .dbg_rx_toggle          (dbg_rx_toggle),
    .dbg_rx_ff              (dbg_rx_ff)
);

// Type-C 上位机源：同一个解析器、同一套协议（关键字 + " n" + 三个 0xFF），
// 只是换了根线。串口屏还没接的时候，电脑就是控制面；屏幕插上后两路并存，
// 互不影响，也不共享任何状态（每路各自一份成帧计数器）。uart_tx 留空——
// Stage 1 仍然不发东西，回显是另一件事。
uart_screen_ctrl #(
    .CLK_FREQ_HZ (50_000_000),
    .BAUD        (9600)
) u_uart_pc_cmd (
    .clk                    (clk),
    .rst                    (rst_all),
    .uart_rx                (uart_pc_rx),
    .uart_tx                (),
    .cmd_next_pulse         (pc_cmd_next_pulse),
    .cmd_auto_pulse         (pc_cmd_auto_pulse),
    .cmd_bright_cycle_pulse (pc_cmd_bright_cycle_pulse),
    .cmd_bright_set         (pc_cmd_bright_set),
    .cmd_bright_set_v       (pc_cmd_bright_set_v),
    .cmd_mode               (pc_cmd_mode),
    .cmd_mode_set           (pc_cmd_mode_set),
    .cmd_marquee            (pc_cmd_marquee),
    .cmd_marquee_set        (pc_cmd_marquee_set),
    .cmd_img_sel            (pc_cmd_img_sel),
    .cmd_img_sel_set        (pc_cmd_img_sel_set),
    .cmd_filt               (pc_cmd_filt),
    .cmd_filt_set           (pc_cmd_filt_set),
    .cmd_font               (pc_cmd_font),
    .cmd_font_set           (pc_cmd_font_set),
    .cmd_audio              (pc_cmd_audio),
    .cmd_audio_set          (pc_cmd_audio_set),
    .cmd_speed              (pc_cmd_speed),
    .cmd_speed_set          (pc_cmd_speed_set),
    .cmd_emg                (pc_cmd_emg),
    .cmd_emg_set            (pc_cmd_emg_set),
    .cmd_vol                (pc_cmd_vol),
    .cmd_vol_set            (pc_cmd_vol_set),
    .dbg_rx_toggle          (dbg_pc_cmd_toggle),
    .dbg_rx_ff              ()
);

// ---- 唯一合并点 ----
// 为什么值不能跟着选通一起按位 OR：cmd_mode / cmd_filt / cmd_speed / cmd_img_sel /
// cmd_bright_set 是「数据 + 单拍选通」，两路同拍各发一条时按位 OR 会得到一个谁都没
// 发过的值（MODE 1 撞 MODE E 就成了 0xF，静默换成另一个转场）。所以选通可以 OR，
// 数据必须做优先级选择：J1 串口屏优先，Type-C 让路。
//
// PC_CMD_ENABLE=0 是这一路的一行退路。门控特意加在**数据也加**而不只加在选通上——
// 只掐选通的话 cmd_mode 仍无条件读 pc_cmd_mode，那个实例就还有一个负载，综合不会
// 把它删掉，「一行退路」就成了假话（本工程在 led[3] 上已经栽过一次）。
wire        pc_next       = PC_CMD_ENABLE & pc_cmd_next_pulse;
wire        pc_auto       = PC_CMD_ENABLE & pc_cmd_auto_pulse;
wire        pc_brup       = PC_CMD_ENABLE & pc_cmd_bright_cycle_pulse;
wire [2:0]  pc_brgt_v     = PC_CMD_ENABLE ? pc_cmd_bright_set : 3'd0;
wire        pc_brgt_set   = PC_CMD_ENABLE & pc_cmd_bright_set_v;
wire [3:0]  pc_mode_v     = PC_CMD_ENABLE ? pc_cmd_mode : 4'd0;
wire        pc_mode_set   = PC_CMD_ENABLE & pc_cmd_mode_set;
wire        pc_marquee_v  = PC_CMD_ENABLE & pc_cmd_marquee;
wire        pc_marquee_set= PC_CMD_ENABLE & pc_cmd_marquee_set;
wire [1:0]  pc_img_sel_v  = PC_CMD_ENABLE ? pc_cmd_img_sel : 2'd0;
wire        pc_img_sel_set= PC_CMD_ENABLE & pc_cmd_img_sel_set;
wire [3:0]  pc_filt_v     = PC_CMD_ENABLE ? pc_cmd_filt : 4'd0;
wire        pc_filt_set   = PC_CMD_ENABLE & pc_cmd_filt_set;
wire        pc_font_v     = PC_CMD_ENABLE & pc_cmd_font;
wire        pc_font_set   = PC_CMD_ENABLE & pc_cmd_font_set;
wire        pc_audio_v    = PC_CMD_ENABLE & pc_cmd_audio;
wire        pc_audio_set  = PC_CMD_ENABLE & pc_cmd_audio_set;
wire [3:0]  pc_speed_v    = PC_CMD_ENABLE ? pc_cmd_speed : 4'd0;
wire        pc_speed_set  = PC_CMD_ENABLE & pc_cmd_speed_set;
wire [1:0]  pc_emg_v      = PC_CMD_ENABLE ? pc_cmd_emg : 2'd0;
wire        pc_emg_set    = PC_CMD_ENABLE & pc_cmd_emg_set;
wire [1:0]  pc_vol_v      = PC_CMD_ENABLE ? pc_cmd_vol : 2'd2;   // 2 = 满音量
wire        pc_vol_set    = PC_CMD_ENABLE & pc_cmd_vol_set;

// ---- 告警冻结 ----
// emg_hold 拉高期间，除 ALRM/VOLM 之外的每一条命令选通都被丢掉。目的是「暂停」而不是
// 「遮挡」：告警图层盖在最上面，普通画面本来就看不见，但如果这期间 MODE/IMGX/NEXT
// 仍然生效，解除告警时落回的就是另一张图、另一种转场，操作员按下解除后看到的不是他
// 触发前的那一帧。冻结把接管瞬间的状态原样冻住，解除即逐位还原。
//
// 只掐选通、不掐数据：每个消费者都是「选通有效才写锁存」，选通为 0 时数据总线自然无人
// 读，省掉 11 个多路器。ALRM/VOLM 是唯一的例外，因为「解除」本身就要在告警期间送达——
// 把它们一起冻掉，接管就成了锁死。
assign cmd_next_pulse         = (j1_cmd_next_pulse         | pc_next)      & ~emg_hold;
assign cmd_auto_pulse         = (j1_cmd_auto_pulse         | pc_auto)      & ~emg_hold;
assign cmd_bright_cycle_pulse = (j1_cmd_bright_cycle_pulse | pc_brup)      & ~emg_hold;
assign cmd_bright_set_v       = (j1_cmd_bright_set_v       | pc_brgt_set)  & ~emg_hold;
assign cmd_bright_set         = j1_cmd_bright_set_v ? j1_cmd_bright_set : pc_brgt_v;
assign cmd_mode_set           = (j1_cmd_mode_set           | pc_mode_set)  & ~emg_hold;
assign cmd_mode               = j1_cmd_mode_set     ? j1_cmd_mode  : pc_mode_v;
assign cmd_marquee_set        = (j1_cmd_marquee_set        | pc_marquee_set) & ~emg_hold;
assign cmd_marquee            = j1_cmd_marquee_set  ? j1_cmd_marquee : pc_marquee_v;
assign cmd_img_sel_set        = (j1_cmd_img_sel_set        | pc_img_sel_set) & ~emg_hold;
assign cmd_img_sel            = j1_cmd_img_sel_set  ? j1_cmd_img_sel : pc_img_sel_v;
assign cmd_filt_set           = (j1_cmd_filt_set           | pc_filt_set)  & ~emg_hold;
assign cmd_filt               = j1_cmd_filt_set     ? j1_cmd_filt  : pc_filt_v;
assign cmd_font_set           = (j1_cmd_font_set           | pc_font_set)  & ~emg_hold;
assign cmd_font               = j1_cmd_font_set     ? j1_cmd_font  : pc_font_v;
assign cmd_audio_set          = (j1_cmd_audio_set          | pc_audio_set) & ~emg_hold;
assign cmd_audio              = j1_cmd_audio_set    ? j1_cmd_audio : pc_audio_v;
assign cmd_speed_set          = (j1_cmd_speed_set          | pc_speed_set) & ~emg_hold;
assign cmd_speed              = j1_cmd_speed_set    ? j1_cmd_speed : pc_speed_v;
// 这两条不带 ~emg_hold，见上面的注释。
assign cmd_emg_set            = j1_cmd_emg_set      | pc_emg_set;
assign cmd_emg                = j1_cmd_emg_set      ? j1_cmd_emg   : pc_emg_v;
assign cmd_vol_set            = j1_cmd_vol_set      | pc_vol_set;
assign cmd_vol                = j1_cmd_vol_set      ? j1_cmd_vol   : pc_vol_v;

// ---- 应急接管：三源融合，一个答案 ----
// 硬线 T1/T2（J2 M3/M4，PULLUP，动作拉低）与合并后的 ALRM 指令在这里比一次
// 「最低的非零类型码」，谁活着谁说了算，解除是所有请求者都清空之后的 AND。3FF 同步、
// 非对称消抖、优先级算术全在 emergency_ctrl.v 里，本块只负责例化、门控和跨域。
emergency_ctrl #(
    .EMERGENCY_ENABLE (EMERGENCY_ENABLE),
    .CLK_FREQ_HZ      (50_000_000),
    .RELEASE_MS       (20),
    .MANUAL_TYPE      (2'd2),      // T1 手报 -> 疏散
    .AUTO_TYPE        (2'd1),      // T2 烟感 -> 火警
    .BURN_SEC         (0)          // 0 = 烤机序列综合消失；7x24 老化时改这里
) u_emergency_ctrl (
    .I_clk          (clk),
    .I_rst          (rst_all),
    .I_t1           (emg_in_manual),
    .I_t2           (emg_in_auto),
    .I_cmd_emg_set  (cmd_emg_set),
    .I_cmd_emg      (cmd_emg),
    .I_cmd_vol_set  (cmd_vol_set),
    .I_cmd_vol      (cmd_vol),
    .O_emg_state    (emg_state_raw),
    .O_emg_tgl      (emg_tgl_raw),
    .O_vol_level    (vol_level_raw)
);

// 退路门控：三个输出**全部**过一道 EMERGENCY_ENABLE。只掐选通的话数据总线还有一个读者，
// 综合就不会删这个实例，「一行退路」就是假话（本工程在 led[3] 上栽过一次，合并点上面
// 那段注释讲的也是同一件事）。tools/sim_emergency.py Pass H 是对着本文件文本查的。
wire emg_gate = EMERGENCY_ENABLE;
assign emg_state  = emg_gate ? emg_state_raw : 3'd0;
assign emg_tgl    = emg_gate & emg_tgl_raw;
assign vol_level  = emg_gate ? vol_level_raw : 2'd2;   // 2 = 满音量 = 加这条链路之前
// 冻结电平。取的是 clk 域的 emg_state[2]，与所有命令锁存同域同拍：解除的那一拍起
// 普通命令就重新生效，不会再被一条同步链晚放行一两拍。
assign emg_hold = EMERGENCY_ENABLE & emg_state[2];

// 板载无源蜂鸣器：警笛的第二条独立发声通路，走 H11 直接出声，不碰 HDMI 音频链路。
// 它留在 clk 域，吃的是上面已经过退路门控的 emg_state / vol_level，所以 EMERGENCY_ENABLE
// 一条就同时掐掉了它；也因为它不出这个域，硬线触发到喇叭只隔着紧急融合自己的那三级同步，
// 比屏幕换画面还快（蜂鸣器不等帧边界，见 buzzer_beep.v 头注释）。
buzzer_beep #(
    .EMERGENCY_ENABLE (EMERGENCY_ENABLE),
    .CLK_FREQ_HZ      (50_000_000),
    .TONE_HZ          (1000),      // 板上谐振未知，上板调这一个参数
    .WINDOW_MS        (600)
) u_buzzer_beep (
    .I_clk    (clk),
    .I_rst    (rst_all),
    .I_alarm  (emg_state[2]),
    .I_type   (emg_state[1:0]),
    .I_level  (vol_level),
    .O_beep   (spk)
);

// PC 文字字幕源：第二路完全独立的 UART（F12），clk 域把整句 ASCII 翻译成
// pc_text_en / char_buf / n_cells / text_toggle。与上面的串口屏实例没有共享任何
// 信号，删掉本实例与 uart_pc_rx 约束即可让 PC 通道彻底消失而不影响 J1 串口屏。
//
// 但注意：本实例和上面新增的 u_uart_pc_cmd 监听的是**同一根 F12**（uart_pc_rx）。
// 两者现在互斥，靠的就是 PC_TEXT_ENABLE 默认 0 把本实例整体 prune 掉，而不是任何
// 硬件仲裁。同时打开会两头一起坏——字幕通道把 MODE/SPED 帧当文字铺到屏上，命令
// 解析器把整句字幕当成前缀污染的不完整长度而静默丢帧。所以这颗开关是「二选一」，
// 不是「可叠加」；真要在同一根线上共存，得先加一层帧头区分或改用第二只 UART。
uart_pc_text #(
    .CLK_FREQ_HZ (50_000_000),
    .BAUD        (9600)
) u_uart_pc_text (
    .clk               (clk),
    .rst               (rst_all),
    .uart_pc_rx        (uart_pc_rx),
    .pc_text_en        (pc_text_en),
    .char_buf          (pc_char_buf),
    .n_cells           (pc_n_cells),
    .text_toggle       (pc_text_toggle),
    .dbg_rx_toggle     (dbg_pc_rx_toggle),
    .dbg_commit_toggle (dbg_pc_commit_toggle)
);

// ---- 串口链路诊断 LED（高电平点亮，A4/A3/C10/B12）----
// uart_tx 在 Stage 1 恒为空闲高，没有任何回读，所以这几只是判断「字节到底有没有
// 送进来」的唯一手段，分三层，逐层收窄故障范围：
//   LED0 闪  = J1 那一路有字节到达（接线、共地、电平、波特率都对）
//   LED1 亮  = J1 那一路最近一个字节是 0xFF（帧终止符收到了，成帧没问题）
//   LED2 闪  = 有一帧被解析器派发。它吃的是两路**未冻结**选通的并集（见下面定义），
//              所以串口屏与 Type-C 任一路派发命令都会闪，告警接管期间也照闪——
//              这一只不区分来源，区分来源看下面。
// 三只全灭 = 问题在物理链路，不用再查协议；LED0 闪而 LED2 不闪 = 字节进来了
// 但帧被判非法，去查发送端的大小写、尾随空格和终止符。
// LED3 闪 = Type-C 命令通道（F12）每收到一个字节翻转一次，dbg_pc_cmd_toggle；
//   它是这一路唯一的物理观测窗口（同样无回读）。LED2 闪而 LED3 不闪 = 命令是从
//   串口屏那一路来的。PC_TEXT_ENABLE 置 1 时这只灯让回字幕通道的字节翻转（见下面）。
// 用两个源各自的**未冻结**选通拼这一句，不吃合并后的 cmd_*：合并选通现在被
// emg_hold 掐着，若 LED2 也读它，告警接管期间这只灯会恰好停闪——而那正是最需要知道
// 「字节还在不在进来」的时刻。LED2 的语义是「任一解析器派发了一帧」，与 top 有没有
// 采纳它是两件事。ALRM/VOLM 也计入，所以接管期间发解除指令仍然看得见闪。
wire cmd_any_set = j1_cmd_next_pulse | j1_cmd_auto_pulse | j1_cmd_bright_cycle_pulse
                 | j1_cmd_bright_set_v | j1_cmd_mode_set | j1_cmd_marquee_set
                 | j1_cmd_img_sel_set | j1_cmd_filt_set | j1_cmd_font_set
                 | j1_cmd_audio_set | j1_cmd_speed_set | j1_cmd_emg_set
                 | j1_cmd_vol_set
                 | pc_next | pc_auto | pc_brup | pc_brgt_set | pc_mode_set
                 | pc_marquee_set | pc_img_sel_set | pc_filt_set | pc_font_set
                 | pc_audio_set | pc_speed_set | pc_emg_set | pc_vol_set;

reg led2_toggle;
always @(posedge clk or posedge rst_all) begin
    if (rst_all)              led2_toggle <= 1'b0;
    else if (cmd_any_set)     led2_toggle <= ~led2_toggle;
end

// led[3] 归「当前活着的那一路 PC 侧通道」：字幕通道使能时仍是它的字节观测灯，
// 否则给 Type-C 命令通道——PC_TEXT_ENABLE=0 时那颗灯本来就恒灭，而 Type-C 这一路
// 在串口屏接上之前是唯一没有物理观测窗口的地方。写成条件式而不是并起来，是为了
// PC_TEXT_ENABLE=0 时 dbg_pc_rx_toggle 一个负载都不剩，u_uart_pc_text 继续被 prune
// （这个保证是 led[3] 这段注释一开始就在讲的那件事）。
assign led = {PC_TEXT_ENABLE ? dbg_pc_rx_toggle
                             : (PC_CMD_ENABLE & dbg_pc_cmd_toggle),
              led2_toggle, dbg_rx_ff, dbg_rx_toggle};

// mode/marquee 覆盖锁存 + 物理拨码变动检测（clk 域）。屏幕命令置 ovr_en 并锁值；
// 任一物理拨码变动清 ovr_en，物理路径立即重新接管。复位值匹配 PULLUP 空闲态
// （SW1-3 全 OFF = 111，SW4 OFF = 1），上电不会误判为"拨码变动"而清掉尚未置起的覆盖。
always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        sw_c0 <= 3'b111; sw_c1 <= 3'b111; sw_c2 <= 3'b111;
        sw4_c0 <= 1'b1;  sw4_c1 <= 1'b1;  sw4_c2 <= 1'b1;
        mode_ovr_val <= 4'd0; mode_ovr_en <= 1'b0;
        marq_ovr_val <= 1'b1; marq_ovr_en <= 1'b0;
    end else begin
        sw_c0 <= sw[2:0]; sw_c1 <= sw_c0; sw_c2 <= sw_c1;
        sw4_c0 <= sw[3];  sw4_c1 <= sw4_c0; sw4_c2 <= sw4_c1;

        if (cmd_mode_set) begin
            mode_ovr_val <= cmd_mode;
            mode_ovr_en  <= 1'b1;
        end else if (sw_c1 != sw_c2) begin
            mode_ovr_en  <= 1'b0;
        end

        if (cmd_marquee_set) begin
            marq_ovr_val <= cmd_marquee;
            marq_ovr_en  <= 1'b1;
        end else if (sw4_c1 != sw4_c2) begin
            marq_ovr_en  <= 1'b0;
        end
    end
end

// FILT/FONT/MUSC 值锁存（clk 域）。独立成一个 block，上面的覆盖锁存一行未动。
always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        filt_val <= 4'd0;
        font_val <= 1'b0;
        music_en <= AUDIO_SRC_DEFAULT;
    end else begin
        if (cmd_filt_set)  filt_val <= cmd_filt;
        if (cmd_font_set)  font_val <= cmd_font;
        if (cmd_audio_set) music_en <= cmd_audio;
    end
end

// 命令脉冲 -> toggle（clk 域）。img 的目标值先锁存再翻转 toggle，保证 data+toggle
// 同步时数据先于 toggle 边沿稳定。
always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        next_tgl <= 1'b0; auto_tgl <= 1'b0; img_tgl <= 1'b0;
        img_sel_lat <= 2'd0;
        spd_tgl     <= 1'b0;
        speed_lat   <= AUTO_SEC_DEFAULT;
    end else begin
        if (cmd_next_pulse) next_tgl <= ~next_tgl;
        if (cmd_auto_pulse) auto_tgl <= ~auto_tgl;
        if (cmd_img_sel_set) begin
            img_sel_lat <= cmd_img_sel;
            img_tgl     <= ~img_tgl;
        end
        if (cmd_speed_set) begin
            speed_lat   <= cmd_speed;
            spd_tgl     <= ~spd_tgl;
        end
    end
end

// sd_card_clk 域：2FF 同步 toggle，s1^s2 还原单周期脉冲；img 选图值与 speed 轮播
// 间隔同样 2FF 同步。
// 复位后 toggle 与同步链都为 0，不会冒出虚假脉冲；这些脉冲与 sd_card_bmp 内部
// 消抖出的 key_next_press / key_auto_press OR 合并，实体按键仍是兜底。
// music_en 是人类速率的准静态电平，走裸 2FF（不是 toggle），s1 直接接
// sd_card_bmp.music_req——那个模块按约定把所有 CDC 留在 top。
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        next_tgl_s0 <= 1'b0; next_tgl_s1 <= 1'b0; next_tgl_s2 <= 1'b0;
        auto_tgl_s0 <= 1'b0; auto_tgl_s1 <= 1'b0; auto_tgl_s2 <= 1'b0;
        img_tgl_s0  <= 1'b0; img_tgl_s1  <= 1'b0; img_tgl_s2  <= 1'b0;
        img_sel_s0  <= 2'd0; img_sel_s1  <= 2'd0;
        spd_tgl_s0  <= 1'b0; spd_tgl_s1  <= 1'b0; spd_tgl_s2  <= 1'b0;
        speed_s0    <= AUTO_SEC_DEFAULT; speed_s1    <= AUTO_SEC_DEFAULT;
        music_en_s0 <= AUDIO_SRC_DEFAULT; music_en_s1 <= AUDIO_SRC_DEFAULT;
        emg_hold_s0 <= 1'b0; emg_hold_sd <= 1'b0;
    end else begin
        next_tgl_s0 <= next_tgl; next_tgl_s1 <= next_tgl_s0; next_tgl_s2 <= next_tgl_s1;
        auto_tgl_s0 <= auto_tgl; auto_tgl_s1 <= auto_tgl_s0; auto_tgl_s2 <= auto_tgl_s1;
        img_tgl_s0  <= img_tgl;  img_tgl_s1  <= img_tgl_s0;  img_tgl_s2  <= img_tgl_s1;
        img_sel_s0  <= img_sel_lat; img_sel_s1 <= img_sel_s0;
        spd_tgl_s0  <= spd_tgl;  spd_tgl_s1  <= spd_tgl_s0;  spd_tgl_s2  <= spd_tgl_s1;
        speed_s0    <= speed_lat;   speed_s1    <= speed_s0;
        music_en_s0 <= music_en; music_en_s1 <= music_en_s0;
        // 冻结电平走裸 2FF，不是 data+toggle：它许可的是「实体键还准不准动轮播」这种
        // 电平语义，丢一个采样只是把门控推迟一拍，而漏掉一个边沿会把门控卡死在半路。
        emg_hold_s0 <= emg_hold; emg_hold_sd <= emg_hold_s0;
    end
end

// 将 SD 控制域的慢速状态同步到 video_clk 域，供 OSD 和黑屏门控使用
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        disp_buf_idx_v0  <= 2'd0;
        disp_buf_idx_v1  <= 2'd0;
        state_code_v0    <= 4'd0;
        state_code_v1    <= 4'd0;
        auto_play_v0     <= 1'b0;
        auto_play_v1     <= 1'b0;
        display_valid_v0 <= 1'b0;
        display_valid_v1 <= 1'b0;
        brightness_level_v0 <= 3'd2;
        brightness_level_v1 <= 3'd2;
        sw_v0 <= 3'b111;                    // ~3'b111 = 3'b000 = auto-cycle out of reset
        sw_v1 <= 3'b111;
        sw4_v0 <= 1'b1;                     // PULLUP: SW4 OFF = 1 = banner shown out of reset
        sw4_v1 <= 1'b1;
        mode_ovr_val_v0 <= 4'd0; mode_ovr_val_v1 <= 4'd0;
        mode_ovr_en_v0  <= 1'b0; mode_ovr_en_v1  <= 1'b0;   // 覆盖默认关：trans_mode 退回 ~sw_v1
        marq_ovr_val_v0 <= 1'b1; marq_ovr_val_v1 <= 1'b1;
        marq_ovr_en_v0  <= 1'b0; marq_ovr_en_v1  <= 1'b0;   // 覆盖默认关：marquee_en 退回 sw4_v1
        filt_val_v0 <= 4'd0; filt_val_v1 <= 4'd0;           // 0 = 直通
        font_val_v0 <= 1'b0; font_val_v1 <= 1'b0;           // 0 = 平面字
        filt_frame  <= 4'd0; font_frame  <= 1'b0;
        music_en_v0 <= AUDIO_SRC_DEFAULT; music_en_v1 <= AUDIO_SRC_DEFAULT;
        // PC 文字通道全部复位为 0：pc_en 出复位即低，横幅在收到 PCTX 1 之前一直
        // 由原中文标语独占，因此"从不发 PC 命令"就是与加这条通道之前逐位一致的退路。
        pc_en_v0 <= 1'b0; pc_en_v1 <= 1'b0;
        pc_tgl_s0 <= 1'b0; pc_tgl_s1 <= 1'b0; pc_tgl_s2 <= 1'b0;
        pc_buf_stg <= 224'd0; pc_cells_stg <= 6'd0;
        pc_en_frame <= 1'b0; pc_buf_frame <= 224'd0; pc_cells_frame <= 6'd0;
        // 应急接管：复位即「无告警、满音量」，所以从不触发就是与加这条链路之前逐位
        // 一致的退路，和上面 PC 文字通道那一段是同一个道理。
        emg_act_s0 <= 1'b0; emg_act_v1 <= 1'b0;
        emg_type_s0 <= 2'd0; emg_type_v1 <= 2'd0;
        emg_tgl_s0 <= 1'b0; emg_tgl_s1 <= 1'b0; emg_tgl_s2 <= 1'b0;
        emg_state_stg <= 3'd0; emg_state_frame <= 3'd0;
        vol_s0 <= 2'd2; vol_v1 <= 2'd2;
        vs_d <= 1'b0;
    end else begin
        disp_buf_idx_v0  <= disp_buf_idx;
        disp_buf_idx_v1  <= disp_buf_idx_v0;
        state_code_v0    <= state_code;
        state_code_v1    <= state_code_v0;
        auto_play_v0     <= auto_play_enabled;
        auto_play_v1     <= auto_play_v0;
        display_valid_v0 <= display_valid;
        display_valid_v1 <= display_valid_v0;
        brightness_level_v0 <= brightness_level;
        brightness_level_v1 <= brightness_level_v0;
        sw_v0 <= sw[2:0];                   // synchronize raw active-low pins SW1-3; sw[3] is synced separately below
        sw_v1 <= sw_v0;
        sw4_v0 <= sw[3];                    // SW4 banner mask, own 2-FF chain, see marquee_en
        sw4_v1 <= sw4_v0;
        mode_ovr_val_v0 <= mode_ovr_val; mode_ovr_val_v1 <= mode_ovr_val_v0;  // clk -> video_clk 2FF
        mode_ovr_en_v0  <= mode_ovr_en;  mode_ovr_en_v1  <= mode_ovr_en_v0;
        marq_ovr_val_v0 <= marq_ovr_val; marq_ovr_val_v1 <= marq_ovr_val_v0;
        marq_ovr_en_v0  <= marq_ovr_en;  marq_ovr_en_v1  <= marq_ovr_en_v0;
        filt_val_v0 <= filt_val; filt_val_v1 <= filt_val_v0;   // clk -> video_clk 2FF
        font_val_v0 <= font_val; font_val_v1 <= font_val_v0;
        music_en_v0 <= music_en; music_en_v1 <= music_en_v0;   // 裸 2FF，刻意不进下面的帧原子锁存
        // PC 文字通道：pc_text_en 走裸 2FF；char_buf/n_cells 走 data+toggle，toggle
        // 边沿到达时数据已稳定 ≥2 拍，故 pc_tgl_edge 这一拍把 staging 锁一次。
        pc_en_v0 <= pc_text_en; pc_en_v1 <= pc_en_v0;
        pc_tgl_s0 <= pc_text_toggle; pc_tgl_s1 <= pc_tgl_s0; pc_tgl_s2 <= pc_tgl_s1;
        if (pc_tgl_edge) begin
            pc_buf_stg   <= pc_char_buf;
            pc_cells_stg <= pc_n_cells;
        end
        // 应急接管的两条路，刻意不同：
        //   电平+类型 emg_act_v1 / emg_type_v1 —— 裸 2FF，不等帧边界。警笛走这一条：
        //     告警开始的那半个静音帧是真正要紧的缺陷，所以这里宁可放弃帧原子。类型码
        //     必须跟电平成对走同一条路，否则接管的头一帧里电平已经是 1、类型码还留在
        //     帧原子那份 0 上，警笛会先用错节奏响最多 16.80 ms。
        //   像素 emg_state_stg —— data+toggle，交给下面的帧起点锁存。图层中途换会让
        //     屏幕撕出一道红/正常之间的横缝。
        // 蜂鸣器两条都不走，它在 clk 域里直接吃 emg_state，见 buzzer_beep.v。
        // 两条延迟各测各的，见 tools/sim_emergency.py Pass A。
        emg_act_s0 <= emg_state[2]; emg_act_v1 <= emg_act_s0;
        emg_type_s0 <= emg_state[1:0]; emg_type_v1 <= emg_type_s0;
        vol_s0 <= vol_level;        vol_v1 <= vol_s0;
        if (emg_tgl_s1 ^ emg_tgl_s2) emg_state_stg <= emg_state;
        emg_tgl_s0 <= emg_tgl; emg_tgl_s1 <= emg_tgl_s0; emg_tgl_s2 <= emg_tgl_s1;
        // 帧原子生效：只在帧起点放行，避免算法/字体/PC 文字在帧中途切换撕出横缝
        if (video_frame_start) begin
            filt_frame <= filt_val_v1;
            font_frame <= font_val_v1;
            pc_en_frame    <= pc_en_v1;
            pc_buf_frame   <= pc_buf_stg;
            pc_cells_frame <= pc_cells_stg;
            emg_state_frame <= emg_state_stg;
        end
        vs_d <= vs;
    end
end

// ===================== TF 多图扫描与缓存（双缓冲） =====================
// 接管期间实体键不再翻页：key1/key2 会动轮播、会起 SD 读，而告警承诺的是「解除后落回
// 触发前那一帧」。emg_hold_sd 自带 EMERGENCY_ENABLE 门控，退路构建里它恒为 0，这两只
// 键的行为与加这条链路之前逐位相同。
// key3（亮度）刻意不冻：亮度是 u_video_brightness 上的模拟增益，在告警图层**下面**，
// 操作员在火场里把它调满是正当需求，且它不碰轮播、不碰 SD。
wire emg_key_gate = ~emg_hold_sd;

sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (3'd4),
    .AUTO_SEC_DEFAULT  (AUTO_SEC_DEFAULT)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_all),
    .key_next          (key1 & emg_key_gate),
    .key_auto          (key2 & emg_key_gate),
    .cmd_next_pulse    (cmd_next_pulse_sd),
    .cmd_auto_pulse    (cmd_auto_pulse_sd),
    .cmd_img_sel       (img_sel_s1),
    .cmd_img_sel_pulse (cmd_img_sel_pulse_sd),
    .cmd_speed         (speed_s1),
    .cmd_speed_pulse   (cmd_speed_pulse_sd),
    .music_req         (music_en_s1),
    .state_code        (state_code),
    .display_valid     (display_valid),
    .auto_play_enabled (auto_play_enabled),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .write_fifo_usedw  (sd_card_write_fifo_usedw),
    .aud_fifo_we       (aud_fifo_we),
    .aud_fifo_di       (aud_fifo_di),
    .aud_fifo_wrusedw  (aud_fifo_wrusedw),
    .dbg_audio_chain   (dbg_audio_chain),
    .dbg_aud_wr_peak   (dbg_aud_wr_peak),
    .dbg_gate          (dbg_gate),
    .dbg_loaded_cnt    (dbg_loaded_cnt),
    .dbg_fail          (dbg_fail),
    .dbg_found_cnt     (dbg_found_cnt),
    .dbg_next_idx      (dbg_next_idx),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso)
);

seg_decoder seg_decoder_m0(
    .bin_data          (state_code),
    .seg_data          (seg_data_0)
);

// Audio bring-up readout; all six digits are used. Left to right the panel
// reads [loaded count][fail cause][scan found count][next load index][chain]
// [state]. With chain == 8 and count < 4, found vs next separates "the scan
// never saw a fourth BMP" (found < 4) from "the fourth load was abandoned"
// (found == 4, next == 4), and fail says which watchdog did it: bit3 the 1 s
// no-progress stall, bit2 a rejected header, bits1:0 the retry counter.
seg_decoder seg_decoder_aud_cnt(
    .bin_data          (dbg_loaded_cnt),
    .seg_data          (seg_data_cnt)
);

seg_decoder seg_decoder_aud_fail(
    .bin_data          (dbg_fail),
    .seg_data          (seg_data_fail)
);

seg_decoder seg_decoder_aud_found(
    .bin_data          (dbg_found_cnt),
    .seg_data          (seg_data_found)
);

seg_decoder seg_decoder_aud_next(
    .bin_data          (dbg_next_idx),
    .seg_data          (seg_data_next)
);

seg_decoder seg_decoder_aud_chain(
    .bin_data          (dbg_audio_chain),
    .seg_data          (seg_data_aud_chain)
);

seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        ({1'b1,seg_data_cnt}),
    .seg_data_1        ({1'b1,seg_data_fail}),
    .seg_data_2        ({1'b1,seg_data_found}),
    .seg_data_3        ({1'b1,seg_data_next}),
    .seg_data_4        ({1'b1,seg_data_aud_chain}),
    .seg_data_5        ({1'b1,seg_data_0})
);

// ===================== 原图像时序与帧缓存 =====================
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

// 图像点运算（FILT n）刻意插在 video_delay 的输入端，而不是显示链里：
// video_brightness -> video_fade -> audio_visualizer -> osd_overlay ->
// rgb_to_axis 那条组合链已经是 video_clk 的关键路径（实测 27.3 ns / 20 级 /
// 余量 12.3 ns），而 read_buf BRAM 输出 -> video_effect -> vout_data_r 这条
// 路径余量接近满周期——video_delay 里现成的捕获寄存器（de_d[19] 门控的那一级）
// 免费吸收算法的组合延迟。附带两个好处：不新增流水级，hs/vs/de 与像素的相对
// 对齐一位不动；消隐期寄存器直接载 0，所以反色模式在消隐期不会刷白。
video_effect u_video_effect (
    .I_rgb (video_read_data[31:8]),
    .I_sel (filt_frame),
    .O_rgb (vout_data_eff)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_en           (video_read_en),
    .read_data         (vout_data_eff),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)
);

// 首图提交前黑屏；提交后一直显示当前显示缓冲区内容
assign vout_data_base = display_valid_v1 ? vout_data_raw : 24'd0;

video_brightness u_video_brightness (
    .I_rgb   (vout_data_base),
    .I_level (display_valid_v1 ? brightness_level_v1 : 3'd2),
    .O_rgb   (vout_data_bright)
);

// Stage 4 transition controller. Decides when the panel is allowed to see a
// buffer switch and how. sd_card_bmp still owns which picture is current; this
// only gates the handover, so the SD side and the write side are untouched.
video_transition #(
    .FADE_MAX    (4'd8),
    // Frames the two selectors are held apart. Must exceed the ramp length
    // WIPE_GRP_MAX / WIPE_GRP_STEP = 240 / 8 = 30 frames in frame_read_write,
    // plus margin for the one frame offset between I_frame_start and the read
    // request that precedes it. 36 leaves 6 frames of saturated full new
    // picture before the selectors are equalised again, and 38 frames end to
    // end is 0.63s at 60Hz, inside the 1s auto play interval in sd_card_bmp,
    // so a wipe always finishes before the picture is allowed to advance again.
    .WIPE_HOLD   (6'd36),
    .WIPE_SETTLE (6'd2)
) u_video_transition (
    .I_clk           (video_clk),
    .I_rst           (rst_all),
    .I_frame_start   (video_frame_start),
    .I_display_valid (display_valid_v1),
    .I_disp_idx      (disp_buf_idx_v1),
    .I_mode          (trans_mode),
    .O_bot_idx       (trans_bot_idx),
    .O_top_idx       (trans_top_idx),
    .O_effect        (trans_effect),
    .O_img_idx       (trans_img_idx),
    .O_fade_level    (trans_fade_level)
);

video_fade u_video_fade (
    .I_display_valid (display_valid_v1),
    .I_level         (trans_fade_level),
    .I_rgb           (vout_data_bright),
    .O_rgb           (vout_data_fade)
);

audio_visualizer #(
    .H_ACTIVE (640),
    .V_ACTIVE (480)
) u_audio_visualizer (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_de          (de),
    .I_frame_start (video_frame_start),
    .I_rgb         (vout_data_fade),
    .I_audio_valid (audio_valid),
    .I_audio_left  (audio_left_data),
    .I_audio_right (audio_right_data),
    .O_rgb         (vout_data_audio)
);

osd_overlay #(
    .H_ACTIVE (640),
    .V_ACTIVE (480)
) u_osd_overlay (
    .I_clk           (video_clk),
    .I_rst           (rst_all),
    .I_de            (de),
    .I_rgb           (vout_data_audio),
    .I_display_valid (display_valid_v1),
    .I_image_index   (trans_img_idx),
    .I_auto_play     (auto_play_v1),
    .I_brightness    (brightness_level_v1),
    .I_state_code    (state_code_v1),
    .I_filt          (filt_frame),
    .I_font          (font_frame),
    .I_asrc          (music_en_v1),
    .O_rgb           (vout_data_osd)
);

marquee_overlay #(
    .H_ACTIVE (640),
    .V_ACTIVE (480)
) u_marquee_overlay (
    .I_clk (video_clk),
    .I_rst (rst_all),
    .I_de  (de),
    .I_rgb (vout_data_osd),
    .I_en  (marquee_en),
    .I_3d  (font_frame),
    .I_pc_en       (pc_en_gated),
    .I_pc_cells    (pc_cells_frame),
    .I_pc_char_buf (pc_buf_frame),
    .O_rgb (vout_data)
);

// 视频链最后一个多路器：盖在标语、OSD 面板、频谱、TF 图之上，它们谁都不用知道告警存在。
// I_en 吃的是帧原子锁存后的 emg_state_frame[2]，所以图层只能在帧起点出现。
//
// 为什么是 generate 而不是「把 I_en 门控成 0 就完事」：这一层是纯组合的直通 mux，
// 把 I_en 绑成常数 0 之后综合仍然保留了整个实例（实测 202 lut / 12 seq，就是上面
// EMERGENCY_ENABLE 那段注释里记的数）。它自己的 x_pos/y_pos 光栅跟踪与 fc 帧计数是
// 跟着 I_de 自由跑的，不受 I_en 门控，常数传播删不掉。generate 才是结构性的删除：
// 参数为 0 时这个模块根本不参与例化，vout_data_alarm 变成 vout_data 的一根别名，
// 「退路」这句话才不需要赌工具会不会折叠。
generate
if (EMERGENCY_ENABLE) begin : g_alarm_overlay
    alarm_overlay #(
        .H_ACTIVE (640),
        .V_ACTIVE (480)
    ) u_alarm_overlay (
        .I_clk  (video_clk),
        .I_rst  (rst_all),
        .I_de   (de),
        .I_rgb  (vout_data),
        .I_en   (emg_state_frame[2]),
        .I_type (emg_state_frame[1:0]),
        .O_rgb  (vout_data_alarm)
    );
end else begin : g_no_alarm_overlay
    assign vout_data_alarm = vout_data;
end
endgenerate

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (BUF0_ADDR),
    .read_addr_1       (BUF1_ADDR),
    .read_addr_2       (BUF2_ADDR),
    .read_addr_3       (BUF3_ADDR),
    .read_addr_index   (trans_bot_idx),
    .read_addr_index_top (trans_top_idx),
    .read_effect       (trans_effect),
    .read_len          (FRAME_PIXELS),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (BUF2_ADDR),
    .write_addr_3      (BUF3_ADDR),
    .write_addr_index  (write_buf_idx),
    .write_len         (FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .write_fifo_usedw  (sd_card_write_fifo_usedw)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== 音频：警笛 / 测试音 / TF 卡 WAV 三选一 =====================
// Three independent sources feed the HDMI audio core through one mux.
//
// Source 0 (music_en_v1 == 0, the AUDIO_SRC_DEFAULT power-up state) is the built-in
// test tone: hdmi_audio_tone_i2s_64fs runs a DDS + ADSR in the audio_mclk domain
// and drives a real I2S bus, which I2S_receiver samples back in video_clk. That
// round trip through actual BCLK/LRCK is deliberate -- it exercises the same
// interface a real codec would, and it is what audio_mclk and PLL_HDMI_AUDIO's
// clk2_out exist for.
//
// Source 1 is the TF-card WAV: sd_audio_stream (inside sd_card_bmp, sd_card_clk
// domain) streams PCM frames off the card into this async FIFO, which crosses
// them into video_clk. audio_pcm_player emits the continuous 48 kHz valid/sample
// stream -- including silence on underrun, so the ACR reference never gaps.
//
// All three sources therefore produce an unbroken valid stream at all times, which
// is what makes a bare 2FF select safe here -- including the alarm's, which is why
// emg_act_v1 is allowed to appear in the audio_valid expression at all:
// audio_arc_calculate only counts 48 valids to pace CTS, so a switch costs at most
// one sample of phase discontinuity and never gaps the ACR reference. No
// frame-atomic latch here. The rule each arm has to obey, and the reason alarm_siren
// zeroes SAMPLES instead of withholding valid, is in that file's header.
//
// The tone generator is instantiated with its MODULE DEFAULT parameters. Do not
// re-apply an AMP override: the default 24'sd8000000 is the value the envelope
// was calibrated against (sustain sits ENV_SUSTAIN steps below the peak), and
// tools/sim_tone_gen.py parses the module defaults.
wfifo_32_32_512 u_audio_fifo (
    .rst        (rst_all),
    .clkw       (sd_card_clk),
    .clkr       (video_clk),
    .we         (aud_fifo_we),
    .di         (aud_fifo_di),
    .re         (aud_fifo_re),
    .dout       (aud_fifo_dout),
    .valid      (),
    .full_flag  (),
    .empty_flag (),
    .afull      (),
    .aempty     (),
    .wrusedw    (aud_fifo_wrusedw),
    .rdusedw    (aud_fifo_rdusedw)
);

hdmi_audio_tone_i2s_64fs u_tone (
    .I_mclk     (audio_mclk),
    .I_rst      (rst_all),
    .O_i2s_BCLK (audio_i2s_bclk),
    .O_i2s_LRCK (audio_i2s_lrck),
    .O_i2s_DOUT (audio_i2s_dout)
);

I2S_receiver u_i2s_rx (
    .I_clk              (video_clk),
    .I_rst              (rst_all),
    .I_i2s_BCLK         (audio_i2s_bclk),
    .I_i2s_LRCK         (audio_i2s_lrck),
    .I_i2s_DOUT         (audio_i2s_dout),
    .O_audio_valid      (tone_valid),
    .O_audio_left_data  (tone_left),
    .O_audio_right_data (tone_right)
);

audio_pcm_player #(
    .CLK_FREQ_HZ    (25_000_000),
    .SAMPLE_RATE_HZ (48_000)
) u_audio_pcm_player (
    .I_clk              (video_clk),
    .I_rst              (rst_all),
    .fifo_re            (aud_fifo_re),
    .fifo_dout          (aud_fifo_dout),
    .fifo_rdusedw       (aud_fifo_rdusedw),
    .O_audio_valid      (mus_valid),
    .O_audio_left_data  (mus_left),
    .O_audio_right_data (mus_right)
);

// Source 2, and the one that outranks the other two: the emergency siren. Pure
// counters in video_clk -- no card read, no FIFO, no waveform ROM -- because the
// sound of an alarm must survive a missing or busy TF card. Its enable and its
// type code come off the FAST 2FF pair, not the frame-atomic copy, for the reason
// spelled out where those registers are declared.
//
// Same generate as the alarm layer below, for the same reason plus one more: with
// only the input gate, the retreat build pruned this instance from the area report
// but still elaborated it and reported HDL-5314 on S_phase_acc[31] -- its whole
// body is inside `else if (EMERGENCY_ENABLE)`, so at 0 nothing drives it. A
// retreat switch that leaves a fresh warning behind is a retreat switch whose next
// measurement nobody can read. The else branch drives all three nets to constants,
// which is also what makes the 4:1 audio mux below fold.
generate
if (EMERGENCY_ENABLE) begin : g_siren
    alarm_siren #(
        .EMERGENCY_ENABLE (EMERGENCY_ENABLE),
        .CLK_FREQ_HZ      (25_000_000),
        .SAMPLE_RATE_HZ   (48_000)
    ) u_alarm_siren (
        .I_clk              (video_clk),
        .I_rst              (rst_all),
        .I_en               (emg_act_v1),
        .I_type             (emg_type_v1),
        .O_audio_valid      (siren_valid),
        .O_audio_left_data  (siren_left),
        .O_audio_right_data (siren_right)
    );
end else begin : g_no_siren
    assign siren_valid   = 1'b0;
    assign siren_left    = 24'd0;
    assign siren_right   = 24'd0;
end
endgenerate

// 三级数字音量。满 / -12 dB / 静音，用算术右移而不是乘法（本工程房规，见
// video_brightness.v 与 hdmi_audio_tone_i2s_64fs.v:137 那段）。$signed 是关键的一半：
// 无符号 >> 会把负半周的高位补 0，-1 变成 0x7FFFFF，那是满幅爆裂，是这条通路上最贵
// 的一种错。-12 dB 而不是 -6 dB：这一档要给的是「整个房间明显轻下来」，扩音设备上
// 6 dB 只会被听成「稍微轻了一点点」，操作员发了 VOLM 1 却听不出区别就是没做到。
function [23:0] vol_gain;
    input [23:0] s;
    input [1:0]  lvl;
    begin
        case (lvl)
            2'd0:    vol_gain = 24'd0;
            2'd1:    vol_gain = $signed(s) >>> 2;
            default: vol_gain = s;
        endcase
    end
endfunction

// 四路优先：警笛 > (WAV | 测试音) x 音量。组合逻辑，不打拍——audio_valid 一旦
// 比 sample 多延一拍，audio_arc_calculate 数到的 48 个 valid 就和数据错位了。
wire        S_pgm_valid = music_en_v1 ? mus_valid : tone_valid;
wire [23:0] S_pgm_left  = vol_gain(music_en_v1 ? mus_left  : tone_left,  vol_v1);
wire [23:0] S_pgm_right = vol_gain(music_en_v1 ? mus_right : tone_right, vol_v1);

// 警笛不吃增益，这是有意的：VOLM 0 关掉的是标牌的背景音，不是火灾时的告警。
// tools/sim_emergency.py Pass G 把这条量成了断言。
assign audio_valid      = emg_act_v1 ? siren_valid   : S_pgm_valid;
assign audio_left_data  = emg_act_v1 ? siren_left    : S_pgm_left;
assign audio_right_data = emg_act_v1 ? siren_right   : S_pgm_right;

audio_arc_calculate #(
    .ACR_N         (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (vs),
    .I_de          (de),
    .I_rgb         (vout_data_alarm),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

// ===================== 带音频的 HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule
