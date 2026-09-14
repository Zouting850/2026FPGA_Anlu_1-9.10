module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,
    // Extra attempts per picture after the first one. sd_card_sec_read_write
    // already retries a missed start token three times, but when all three
    // miss it skips the sector and keeps going. During ST_LOAD_DATA that
    // shifts every later pixel by 512 bytes; during ST_LOAD_HDR it produces a
    // headerless sector, header_match_r fails, load_failed fires, and the
    // picture would be lost for good -- next_load_idx has already advanced
    // past it and nothing ever walks it back. Retrying the whole picture from
    // its header is what turns a transient bit error into a slightly longer
    // startup instead of a permanently short carousel.
    parameter [2:0]   LOAD_MAX_RETRY    = 3'd3,
    // When music is allowed to start. 1: together with the first picture, so
    // audio and video reach the panel on the same event and the remaining
    // pictures load in the background while the track plays. 0: the old
    // behaviour, music waits until every picture is committed -- which puts
    // roughly five seconds of silent slideshow on screen first, and makes a
    // single picture that never commits cost the music as well. Retreat switch
    // only; the sector arbiter below works either way, because with 0 the two
    // consumers go back to being strictly time-disjoint.
    parameter         AUDIO_START_ON_FIRST_IMAGE = 1'b1,
    // Per-image audio (contest extension 2). 1: the scan's WAV directory entries
    // fill a 4-slot table and track N plays under picture N. 0: only slot 0 is
    // ever captured or selected, so the design collapses back to today's
    // single-track loop and the 192 registers holding slots 1..3 constant-fold
    // away. Retreat switch only -- nothing below depends on which is set.
    parameter         AUDIO_MULTI_TRACK   = 1'b1,
    // Which track the auto carousel holds. The carousel deliberately does NOT
    // follow the picture: one unchanging track over a rotating slideshow is the
    // requested behaviour, and a track that restarted every interval would never
    // get past its first seconds.
    parameter [1:0]   AUTO_TRACK_IDX      = 2'd0,
    // 1 makes the auto carousel follow img_idx like a manual switch does, i.e.
    // per-image music in both modes. Retreat/alternative-behaviour switch; the
    // shipped value is 0.
    parameter         AUDIO_FOLLOW_IN_AUTO = 1'b0,
    // Carousel interval out of reset, in whole seconds. The top level hands down
    // its own AUTO_SEC_DEFAULT so the clk-domain latch and this counter agree
    // before any SPED command arrives. It lands in sec_target_m1 as target-1 so
    // the per-tick comparison stays a plain register equality -- see sec_last.
    parameter [2:0]   AUTO_SEC_DEFAULT  = 3'd1
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    // Serial-screen commands, already crossed into this sd_card_clk domain by
    // the top level (toggle-CDC for the pulses, data+toggle for cmd_img_sel and
    // cmd_speed), so this module stays single-clock. The two pulses OR into the
    // debounced key conditions below; cmd_img_sel directly selects an
    // already-loaded picture; cmd_speed is the carousel interval in whole
    // seconds, held between pulses and latched on cmd_speed_pulse.
    // uart_screen_ctrl already rejects everything outside 1..8, so the range is
    // not re-checked here -- one place owns the protocol.
    input                       cmd_next_pulse,
    input                       cmd_auto_pulse,
    input       [1:0]           cmd_img_sel,
    input                       cmd_img_sel_pulse,
    input       [3:0]           cmd_speed,
    input                       cmd_speed_pulse,
    // Audio source select (screen command MUSC), already 2FF synchronised into
    // this domain by the top level like everything else above. 1 = play the
    // TF-card WAV, 0 = the built-in test tone owns the audio output and this
    // module must not arm the streamer at all, so it never requests a sector and
    // bmp_read gets the whole port.
    input                       music_req,
    output [3:0]                state_code,
    output reg                  display_valid,
    output                      auto_play_enabled,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,

    // Occupancy of the write-side async FIFO, routed out of frame_read_write so
    // the scaler can apply backpressure from inside this clock domain.
    input  [8:0]                write_fifo_usedw,

    // Audio stream FIFO write side. The async FIFO body lives in the top level
    // (it crosses sd_card_clk -> video_clk); sd_card_bmp stays single-clock and
    // only drives the write port and reads the write-side occupancy for the
    // streamer's sector-boundary backpressure.
    output                      aud_fifo_we,
    output [31:0]               aud_fifo_di,
    input  [8:0]                aud_fifo_wrusedw,

    // Bring-up visibility for a silent audio chain, read by the 7-segment in the
    // top level. dbg_audio_chain is {wav_found, audio_phase, ever_we, fault}:
    // the scan saw a WAV, the streamer is armed, the streamer has written at
    // least one frame since power-up, and it rejected the header.
    // dbg_aud_wr_peak is a sticky high-water mark of aud_fifo_wrusedw[8:5], so a
    // FIFO that filled and drained again still reads back how full it got.
    output [3:0]                dbg_audio_chain,
    output reg [3:0]            dbg_aud_wr_peak,

    // chain == 8 says the streamer is not armed, but not which input holds it
    // off. There are now three, and the third is not a fault: the screen has not
    // selected the music source (music_req low), which is the power-up default
    // and means the built-in test tone owns the audio output. Tell them apart by
    // ear and by the OSD line 3 echo, not by this nibble.
    //
    // The other two: under AUDIO_START_ON_FIRST_IMAGE, wav_found and
    // first_image_committed. display_valid rises on the very same
    // load_complete_now that sets the latter, so dbg_gate bit0 already names the
    // stuck term -- either no WAV was found or no picture ever committed. The
    // other three bits of dbg_gate ({bmp_ready, ~load_busy, scan_done}) are the
    // terms of the AUDIO_START_ON_FIRST_IMAGE == 0 retreat gate, and are what to
    // read with that parameter cleared. dbg_loaded_cnt is img_loaded_count.
    //
    // ever_we is sticky and survives a source switch, so chain bit1 reads 1
    // forever after the music has played once. It answers "did PCM ever reach
    // the FIFO write side", never "is music playing now".
    output [3:0]                dbg_gate,
    output [3:0]                dbg_loaded_cnt,

    // count < 4 on hardware means the fourth picture never committed, which is
    // a scan problem (found_cnt < 4) or a load problem (next_idx ran past the
    // last found entry after the retries gave up). dbg_fail says which load
    // failure mode fired: bit3 the 1 s no-progress watchdog, bit2 a rejected
    // header sector, bits1:0 the retry counter at the moment of reading.
    output [3:0]                dbg_fail,
    output [3:0]                dbg_found_cnt,
    output [3:0]                dbg_next_idx,

    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

wire key_next_press;
wire key_auto_press;

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
// Two consumers of the single SD sector-read port: bmp_read (pictures) and
// sd_audio_stream (music). They share it sector by sector through the one-hot
// arbiter below rather than being kept strictly time-disjoint, which is what
// lets the track start on the same event as the first picture instead of after
// the last one. sd_card_sec_read_write latches the address in S_WAIT_READ_WRITE
// on any cycle sd_sec_read is high and only returns there through the
// single-cycle S_READ_END, so a sector is atomic and its boundary is the only
// safe point to change hands.
wire             bmp_sd_sec_read;
wire [31:0]      bmp_sd_sec_read_addr;
wire             aud_sd_sec_read;
wire [31:0]      aud_sd_sec_read_addr;
// Per-owner views of the shared response. Gating at the instantiation boundary
// is what keeps bmp_read.v and sd_audio_stream.v untouched: each only ever sees
// the bytes and the end pulse belonging to a sector it asked for. Without it an
// audio sector landing while bmp_read sits in ST_LOAD_HDR would walk rd_cnt and
// corrupt the header parse, and one landing during ST_LOAD_DATA would be counted
// into bmp_len_cnt, shifting every later pixel.
wire             bmp_sec_data_valid;
wire             bmp_sec_read_end;
wire             aud_sec_data_valid;
wire             aud_sec_read_end;
wire             aud_dbg_ever_we;
wire             aud_dbg_fault;
// Retire acknowledgement from sd_audio_stream: high only in its S_IDLE, which is
// the one state that holds no granted sector. The track table rewrite below waits
// on it, so a switch can never retarget a sector that is already in flight.
wire             aud_stream_idle;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire [15:0]      bmp_src_width;
wire [15:0]      bmp_src_height;
wire             bmp_src_dim_valid;
wire             scaler_dst_valid;
wire [23:0]      scaler_dst_pixel;
wire             sd_init_done;
wire             bmp_ready;
wire [3:0]       bmp_state_code;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;
wire             scan_found_wav_valid;
wire [31:0]      scan_found_wav_sector;
wire [31:0]      scan_found_wav_size;
wire             load_failed;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_raw_only;
reg              raw_fallback_started;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
// Carousel interval, in whole seconds, as a count of auto_tick pulses. Three
// bits is the whole 1..8 range and nothing more: sec_target_m1 holds target-1
// (0..7) and sec_cnt counts up to it.
reg [2:0]        sec_cnt;
reg [2:0]        sec_target_m1;
reg [2:0]        img_found_count;
reg [2:0]        img_loaded_count;
reg [2:0]        next_load_idx;
reg [1:0]        img_idx;
reg [1:0]        load_idx;
reg [1:0]        load_buf_idx;
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              load_busy;
reg              source_done_seen;
reg              write_done_seen;
reg [31:0]       load_stall_cnt;
reg              load_abort;
reg [2:0]        load_retry_cnt;

// WAV directory entries captured during the scan, in physical directory order,
// plus the live pair sd_audio_stream actually reads. Slot N is track N and pairs
// with picture N, which is the whole of contest extension 2: bmp_read emits one
// scan_found_wav_valid pulse per WAV and this counts them exactly the way the
// block above counts scan_found_valid into img_sector0..3.
//
// The live wav_sector/wav_size pair is a registered copy of one slot, never a
// combinational mux into the streamer. It may only be rewritten while the
// streamer reports stream_idle, because that is the one state that is not holding
// a granted sector and will not read wav_start_sector until start rises again.
reg [31:0]       wav_sector0;
reg [31:0]       wav_sector1;
reg [31:0]       wav_sector2;
reg [31:0]       wav_sector3;
reg [31:0]       wav_size0;
reg [31:0]       wav_size1;
reg [31:0]       wav_size2;
reg [31:0]       wav_size3;
reg [2:0]        wav_found_count;
reg [31:0]       wav_sector;
reg [31:0]       wav_size;
// Clamped track selection, the slot the live pair currently holds, and the
// in-flight switch flag. track_pending is what drops the streamer's start; see
// the handshake block below for why the switch is a retire-and-rearm rather than
// an in-place retarget.
reg [1:0]        track_req_r;
reg [1:0]        track_cur;
reg              track_pending;
wire             wav_found;
reg              audio_phase;
// One-hot ownership of the SD sector-read port for the sector in flight. See the
// arbiter block below for why these are one-hot rather than a grant flag plus a
// select bit: the gated response wires are a single AND with a register each.
reg              arb_bmp_own;
reg              arb_aud_own;
reg              dbg_stall_seen;
reg              dbg_hdr_seen;

reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire auto_tick;
wire sec_last;
wire [1:0] next_from_loaded;
wire       source_done_now;
wire       write_done_now;
wire       load_complete_now;
wire       load_progress;
wire [2:0] loaded_count_plus_one;
wire       load_stall_hit;
wire       load_gave_up;

// The scaler owns the write stream now. It emits exactly FRAME_WIDTH *
// FRAME_HEIGHT pixels per image whatever the source geometry, which is what
// keeps write_len and the WRITE_V_FLIP row addressing in frame_fifo_write
// valid without any change there. bmp_data_wr_en only feeds the scaler.
assign write_en   = scaler_dst_valid;
assign write_data = {scaler_dst_pixel, 8'b0};
// Music start condition. AUDIO_START_ON_FIRST_IMAGE picks between syncing to the
// first picture and the old wait-for-everything behaviour; see the parameter.
// music_req gates the whole thing: with the test tone selected the streamer is
// never armed, so it never enters the sector arbiter and the picture loads run
// unopposed.
wire audio_start_now = music_req &&
                       (AUDIO_START_ON_FIRST_IMAGE
                       ? first_image_committed
                       : (bmp_ready && !load_busy &&
                          (img_loaded_count >= SCAN_TARGET_COUNT)));

assign wav_found = (wav_found_count != 3'd0);

// Track selection. A manual switch follows the picture; the auto carousel holds
// one track instead, because a track that restarted on every interval would
// never get past its opening seconds. AUDIO_FOLLOW_IN_AUTO makes both modes
// per-image; AUDIO_MULTI_TRACK=0 collapses everything onto slot 0 and restores
// the single-track loop.
//
// The clamp matters. A card with two tracks and four pictures would otherwise
// read an empty slot for pictures 3 and 4 -- sector 0, size 0 -- which is a
// silent S_FAULT rather than a visible error. Falling back to track 0 keeps
// music playing on every picture whatever the card holds.
wire [1:0] track_req_raw = AUDIO_MULTI_TRACK
                         ? ((AUDIO_FOLLOW_IN_AUTO || !auto_play_en)
                            ? img_idx : AUTO_TRACK_IDX)
                         : 2'd0;
wire [1:0] track_req_lim = ({1'b0, track_req_raw} < wav_found_count)
                         ? track_req_raw : 2'd0;
// The streamer's arm level. track_pending holds it low for the whole switch,
// which is what retires sd_audio_stream to S_IDLE at its next sector boundary so
// the table can be rewritten.
wire       aud_start     = audio_phase && !track_pending;

// ---------------------------------------------------------------------------
// SD sector-read port arbiter.
//
// A grant lasts exactly one sector. sd_card_sec_read_write latches the address
// in S_WAIT_READ_WRITE on any cycle sd_sec_read is high and returns to that
// state only via the single-cycle S_READ_END, so releasing on sd_sec_read_end
// hands the port over at the only point where changing hands is safe.
//
// Two properties here are load-bearing, not stylistic:
//
// 1. sd_sec_read is driven by the owner flags alone, never by the consumer's
//    request. Both consumers drop their request for exactly one cycle at each
//    sector boundary and re-assert it with addr+1 (bmp_read ST_LOAD_DATA,
//    sd_audio_stream S_READ), and the arbiter is idle on that very cycle.
//    Passing the request through would let the reader latch a sector the
//    arbiter had not granted while the grant went to the other consumer, so the
//    grantee would ingest 512 bytes belonging to someone else.
//
// 2. Because sd_sec_read does not depend on the request, a granted sector
//    always runs to its end pulse whatever the consumer does next. load_abort
//    puts bmp_read back in ST_IDLE and a failed RIFF/WAVE check puts the
//    streamer in S_FAULT, both with their request low; the arbiter still gets
//    its release, so "hand over on sd_sec_read_end" is total rather than
//    conditional on the consumer cooperating. The cost is one wasted sector,
//    whose bytes the withdrawn owner ignores: bmp_read sits in ST_IDLE where
//    reading_sector is false, so rd_cnt stays 0, and bmp_len_cnt only counts
//    in ST_LOAD_DATA.
//
// Audio wins ties. Its 512-frame FIFO is a hard ~10.7 ms deadline -- an underrun
// is audible silence -- while a picture loses nothing by waiting one sector for
// its next load slot. At 48 kHz stereo 16-bit it asks for one sector per 2.67 ms
// against a port that delivers one in roughly 184 us, so it takes about 7% of
// the bandwidth and the picture loads barely notice.
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst || !sd_init_done) begin
        arb_bmp_own <= 1'b0;
        arb_aud_own <= 1'b0;
    end else if (sd_sec_read_end) begin
        arb_bmp_own <= 1'b0;
        arb_aud_own <= 1'b0;
    end else if (!arb_bmp_own && !arb_aud_own) begin
        if (aud_sd_sec_read)
            arb_aud_own <= 1'b1;
        else if (bmp_sd_sec_read)
            arb_bmp_own <= 1'b1;
    end
end

assign sd_sec_read      = arb_bmp_own | arb_aud_own;
assign sd_sec_read_addr = arb_aud_own ? aud_sd_sec_read_addr : bmp_sd_sec_read_addr;

assign bmp_sec_data_valid = sd_sec_read_data_valid & arb_bmp_own;
assign bmp_sec_read_end   = sd_sec_read_end        & arb_bmp_own;
assign aud_sec_data_valid = sd_sec_read_data_valid & arb_aud_own;
assign aud_sec_read_end   = sd_sec_read_end        & arb_aud_own;

assign dbg_audio_chain  = {wav_found, audio_phase, aud_dbg_ever_we, aud_dbg_fault};
assign dbg_gate         = {bmp_ready, ~load_busy, scan_done, display_valid};
assign dbg_loaded_cnt   = {1'b0, img_loaded_count};
assign dbg_fail         = {dbg_stall_seen, dbg_hdr_seen, load_retry_cnt[1:0]};
assign dbg_found_cnt    = {1'b0, img_found_count};
assign dbg_next_idx     = {1'b0, next_load_idx};
assign auto_tick  = (auto_cnt == (CLK_FREQ_HZ - 1));
// auto_tick still fires once per second exactly as it always has; sec_last says
// whether the interval has elapsed. The picture changes on the AND of the two.
// This is deliberately a second parallel counter and NOT a variable compare
// folded into auto_tick: sd_card_clk is the tightest of the four domains, and
// putting a 32-bit magnitude compare against a value that comes from another
// clock domain on that path would spend slack that does not exist. Two 3-bit
// registers meet at a single AND instead, and storing target-1 when SPED
// arrives keeps the subtractor off this path as well.
assign sec_last   = (sec_cnt == sec_target_m1);
assign next_from_loaded = next_index_limited(img_idx, img_loaded_count);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign source_done_now = source_done_seen | (load_busy && bmp_ready);
assign write_done_now  = write_done_seen  | (load_busy && write_finish_pulse);
assign load_complete_now = load_busy && source_done_now && write_done_now;
// The stall watchdog is fed by anything that shows the load is still moving.
// scaler_dst_valid is included because the source stream ends before the
// destination stream does whenever the scaler has black border rows left to
// emit.
assign load_progress = bmp_data_wr_en || scaler_dst_valid || write_finish_pulse || write_req_ack;
assign loaded_count_plus_one = img_loaded_count + 3'd1;
// The 1s silence watchdog. Named here rather than spelled out inline because
// the retry decision below has to cover it as well as load_failed: from this
// module's point of view a picture that stopped moving and a picture whose
// header came back unreadable are the same event, an attempt that did not
// produce a frame buffer worth keeping.
assign load_stall_hit = load_busy && !load_progress &&
                        (load_stall_cnt >= CLK_FREQ_HZ - 1);
assign load_gave_up   = (load_busy && load_failed) || load_stall_hit;
assign auto_play_enabled = auto_play_en;
assign state_code = (!sd_init_done)                 ? 4'd0 :
                    (display_valid && auto_play_en) ? 4'd6 :
                    (display_valid)                 ? 4'd5 :
                    (scan_raw_only && !scan_done)   ? 4'd7 :
                                                       bmp_state_code;

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press)
);

function [1:0] next_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;
            3'd1: next_index_limited = 2'd0;
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
        endcase
    end
endfunction

function [31:0] sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

// Same shape as sector_lut above, which is the point: a registered 4-way select
// reading four registers, already proven to meet timing in this domain on the
// picture side. Registering the result into wav_sector/wav_size keeps the mux off
// the streamer's arm path entirely.
function [31:0] wav_sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: wav_sector_lut = wav_sector0;
            2'd1: wav_sector_lut = wav_sector1;
            2'd2: wav_sector_lut = wav_sector2;
            2'd3: wav_sector_lut = wav_sector3;
            default: wav_sector_lut = wav_sector0;
        endcase
    end
endfunction

function [31:0] wav_size_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: wav_size_lut = wav_size0;
            2'd1: wav_size_lut = wav_size1;
            2'd2: wav_size_lut = wav_size2;
            2'd3: wav_size_lut = wav_size3;
            default: wav_size_lut = wav_size0;
        endcase
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_raw_only         <= 1'b0;
        raw_fallback_started  <= 1'b0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        sec_cnt               <= 3'd0;
        sec_target_m1         <= AUTO_SEC_DEFAULT - 3'd1;
        img_found_count       <= 3'd0;
        img_loaded_count      <= 3'd0;
        next_load_idx         <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        load_buf_idx          <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        load_busy             <= 1'b0;
        source_done_seen      <= 1'b0;
        write_done_seen       <= 1'b0;
        load_stall_cnt        <= 32'd0;
        load_abort            <= 1'b0;
        load_retry_cnt        <= 3'd0;
        display_valid         <= 1'b0;
        wav_sector0           <= 32'd0;
        wav_sector1           <= 32'd0;
        wav_sector2           <= 32'd0;
        wav_sector3           <= 32'd0;
        wav_size0             <= 32'd0;
        wav_size1             <= 32'd0;
        wav_size2             <= 32'd0;
        wav_size3             <= 32'd0;
        wav_found_count       <= 3'd0;
        wav_sector            <= 32'd0;
        wav_size              <= 32'd0;
        track_req_r           <= 2'd0;
        track_cur             <= 2'd0;
        track_pending         <= 1'b0;
        audio_phase           <= 1'b0;
        dbg_stall_seen        <= 1'b0;
        dbg_hdr_seen          <= 1'b0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        load_abort       <= 1'b0;

        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            // sec_cnt is elapsed time, so it restarts with auto_cnt.
            // sec_target_m1 is configuration and is deliberately left alone: a
            // card re-init should not forget the interval the operator chose.
            sec_cnt               <= 3'd0;
            img_found_count       <= 3'd0;
            img_loaded_count      <= 3'd0;
            next_load_idx         <= 3'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            load_buf_idx          <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            load_busy             <= 1'b0;
            source_done_seen      <= 1'b0;
            write_done_seen       <= 1'b0;
            load_stall_cnt        <= 32'd0;
            load_abort            <= 1'b0;
            load_retry_cnt        <= 3'd0;
            display_valid         <= 1'b0;
            scan_raw_only         <= 1'b0;
            raw_fallback_started  <= 1'b0;
            wav_sector0           <= 32'd0;
            wav_sector1           <= 32'd0;
            wav_sector2           <= 32'd0;
            wav_sector3           <= 32'd0;
            wav_size0             <= 32'd0;
            wav_size1             <= 32'd0;
            wav_size2             <= 32'd0;
            wav_size3             <= 32'd0;
            wav_found_count       <= 3'd0;
            wav_sector            <= 32'd0;
            wav_size              <= 32'd0;
            track_req_r           <= 2'd0;
            track_cur             <= 2'd0;
            track_pending         <= 1'b0;
            audio_phase           <= 1'b0;
            dbg_stall_seen        <= 1'b0;
            dbg_hdr_seen          <= 1'b0;
        end else begin
            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    default: ;
                endcase

                if (img_found_count < 3'd4)
                    img_found_count <= img_found_count + 3'd1;
            end

            // Fill the track table in physical directory order, exactly the way
            // the scan_found_valid block above fills img_sector0..3 -- bmp_read
            // now emits one pulse per WAV instead of latching only the first.
            // The AUDIO_MULTI_TRACK term stops the capture at slot 0 when
            // cleared, which is what constant-folds slots 1..3 away for the
            // retreat rather than leaving them allocated and unused.
            if (scan_found_wav_valid &&
                (AUDIO_MULTI_TRACK || (wav_found_count == 3'd0))) begin
                case (wav_found_count)
                    3'd0: begin
                        wav_sector0 <= scan_found_wav_sector;
                        wav_size0   <= scan_found_wav_size;
                    end
                    3'd1: begin
                        wav_sector1 <= scan_found_wav_sector;
                        wav_size1   <= scan_found_wav_size;
                    end
                    3'd2: begin
                        wav_sector2 <= scan_found_wav_sector;
                        wav_size2   <= scan_found_wav_size;
                    end
                    default: begin
                        wav_sector3 <= scan_found_wav_sector;
                        wav_size3   <= scan_found_wav_size;
                    end
                endcase

                if (wav_found_count < 3'd4)
                    wav_found_count <= wav_found_count + 3'd1;
            end

            // Arm the music streamer. Under AUDIO_START_ON_FIRST_IMAGE this is
            // the same event that raises display_valid in the load_complete_now
            // branch below, so the picture and the track reach the panel
            // together and the remaining pictures load in the background while
            // it plays -- the arbiter above is what makes sharing the port
            // mid-slideshow safe.
            //
            // No longer sticky until reset: it follows music_req, so MUSC 0
            // disarms and sd_audio_stream retires to S_IDLE at its next sector
            // boundary. Re-arming needs no picture event, because
            // first_image_committed is itself sticky, so audio_start_now is
            // already true the cycle music_req comes back.
            if (!audio_phase && wav_found && audio_start_now)
                audio_phase <= 1'b1;
            else if (audio_phase && !music_req)
                audio_phase <= 1'b0;

            // ---- per-image track switch ------------------------------------
            //
            // A switch is a retire-and-rearm, never an in-place retarget. start
            // falls, the streamer retires to S_IDLE at the end of the sector in
            // flight, stream_idle comes back, and only then does the live pair
            // take the new slot's values. That is the exact path MUSC 0 already
            // uses and tools/sim_audio_stream.py already proves: a granted
            // sector always runs to its end pulse, the arbiter always gets its
            // release, and the abandoned tail (at most 128 words, 2.67 ms) drains
            // unheard. Retargeting wav_start_sector mid-sector instead would
            // have no such proof, and the failure mode is silent frame
            // misalignment rather than anything observable.
            //
            // track_req_r is a register purely for timing. It gives the 3-bit
            // clamp compare its own pipeline stage so wav_sector_lut reads a
            // register, making that path register -> 4:1 mux -> register -- the
            // same shape as sector_lut feeding load_sector on the picture side,
            // in the domain with the least slack in the design.
            //
            // Rewriting the live pair while !audio_phase is safe even though the
            // streamer may still be retiring through S_READ: its rewind branch
            // is an else-if after `if (!start)`, so with start low it goes to
            // S_IDLE and never reads wav_start_sector, and pcm_total/wav_usable
            // are only sampled in S_IDLE.
            track_req_r <= track_req_lim;

            if (!audio_phase) begin
                // Follow the selection freely. This is also what loads slot 0
                // ahead of the very first arm, so no separate initialisation
                // exists to get out of step with the scan. track_pending is
                // cleared here on purpose: left set, it would survive into the
                // next arm and hold start low forever, which is a deadlock with
                // nothing to indicate it.
                wav_sector    <= wav_sector_lut(track_req_r);
                wav_size      <= wav_size_lut(track_req_r);
                track_cur     <= track_req_r;
                track_pending <= 1'b0;
            end else if (!track_pending && (track_req_r != track_cur)) begin
                track_pending <= 1'b1;
            end else if (track_pending && aud_stream_idle) begin
                // Taking track_req_r here rather than a snapshot from when
                // pending was raised means a second switch arriving mid-retire
                // lands on the newest selection instead of the stale one.
                wav_sector    <= wav_sector_lut(track_req_r);
                wav_size      <= wav_size_lut(track_req_r);
                track_cur     <= track_req_r;
                track_pending <= 1'b0;
            end

            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            if (load_busy && write_finish_pulse)
                write_done_seen <= 1'b1;

            if (load_gave_up) begin
                // bmp_read rejected the header sector, or the picture stopped
                // moving for a whole second. Either way this attempt left
                // nothing worth keeping in the frame buffer.
                if (load_stall_hit) dbg_stall_seen <= 1'b1;
                if (load_failed)    dbg_hdr_seen   <= 1'b1;
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                load_stall_cnt   <= 32'd0;
                // Only the stall path needs to be told to let go: on
                // load_failed bmp_read has already put itself back in
                // ST_IDLE.
                load_abort       <= load_stall_hit;

                // Walking next_load_idx back re-arms the same picture: the
                // re-arm branch below reads sector_lut(next_load_idx) and
                // load_buf_idx from img_loaded_count, which did not advance,
                // so the retry re-reads the same file into the same buffer.
                if (load_retry_cnt < LOAD_MAX_RETRY) begin
                    load_retry_cnt <= load_retry_cnt + 3'd1;
                    next_load_idx  <= next_load_idx - 3'd1;
                end else begin
                    load_retry_cnt <= 3'd0;
                end
            end else if (load_complete_now) begin
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                load_stall_cnt   <= 32'd0;
                load_retry_cnt   <= 3'd0;

                if (img_loaded_count < SCAN_TARGET_COUNT)
                    img_loaded_count <= loaded_count_plus_one;

                if (!first_image_committed && (load_buf_idx == 2'd0)) begin
                    disp_buf_idx          <= load_buf_idx;
                    img_idx               <= load_buf_idx;
                    display_valid         <= 1'b1;
                    first_image_committed <= 1'b1;
                end
            end else if (load_busy) begin
                if (load_progress)
                    load_stall_cnt <= 32'd0;
                else
                    load_stall_cnt <= load_stall_cnt + 32'd1;
            end else begin
                load_stall_cnt <= 32'd0;
            end

            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                sec_cnt               <= 3'd0;
                img_found_count       <= 3'd0;
                img_loaded_count      <= 3'd0;
                next_load_idx         <= 3'd0;
                img_idx               <= 2'd0;
                load_idx              <= 2'd0;
                load_buf_idx          <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                display_valid         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                load_stall_cnt        <= 32'd0;
                load_abort            <= 1'b0;
                load_retry_cnt        <= 3'd0;
                scan_raw_only         <= 1'b0;
                raw_fallback_started  <= 1'b0;
            end else begin
                // Every site below that clears auto_cnt also clears sec_cnt.
                // They are one event -- "the carousel timing restarts here" --
                // and clearing only one of the two hands the next interval a
                // part-elapsed second, so that picture comes up early once and
                // nothing in a steady-state rotation would ever show it.
                if ((key_auto_press || cmd_auto_pulse) && first_image_committed && (img_found_count > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;
                    sec_cnt      <= 3'd0;
                end

                // auto_cnt stays the one-second tick generator and restarts on
                // every tick regardless of the interval; sec_cnt counts those
                // ticks. Only the AND of the two moves the picture.
                if (auto_play_en && first_image_committed && (img_loaded_count > 3'd1)) begin
                    if (auto_tick) begin
                        auto_cnt <= 32'd0;
                        if (sec_last) begin
                            sec_cnt      <= 3'd0;
                            img_idx      <= next_from_loaded;
                            disp_buf_idx <= next_from_loaded;
                        end else begin
                            sec_cnt <= sec_cnt + 3'd1;
                        end
                    end else begin
                        auto_cnt <= auto_cnt + 32'd1;
                    end
                end else begin
                    auto_cnt <= 32'd0;
                    sec_cnt  <= 3'd0;
                end

                if ((key_next_press || cmd_next_pulse) && first_image_committed && (img_loaded_count > 3'd1)) begin
                    img_idx      <= next_from_loaded;
                    disp_buf_idx <= next_from_loaded;
                    auto_cnt     <= 32'd0;
                    sec_cnt      <= 3'd0;
                end

                if (cmd_img_sel_pulse && first_image_committed &&
                    ({1'b0, cmd_img_sel} < img_loaded_count)) begin
                    img_idx      <= cmd_img_sel;
                    disp_buf_idx <= cmd_img_sel;
                    auto_cnt     <= 32'd0;
                    sec_cnt      <= 3'd0;
                end

                // SPED: store the interval as target-1 so sec_last stays a
                // register equality. Not gated on first_image_committed -- this
                // is configuration, not a picture action, and dropping one that
                // arrives during the initial scan would lose it silently on a
                // screen that never reads back.
                if (cmd_speed_pulse) begin
                    sec_target_m1 <= cmd_speed[2:0] - 3'd1;
                    sec_cnt       <= 3'd0;
                end

                if (scan_done && bmp_ready && !load_busy &&
                    !scan_raw_only && !raw_fallback_started && !first_image_committed &&
                    (next_load_idx >= img_found_count)) begin
                    // The picture table is cleared and rebuilt because the raw
                    // fallback rescan repopulates it. The WAV track table is
                    // deliberately NOT cleared here, and the asymmetry is not an
                    // oversight: ST_SCAN_RAW is a raw sector sweep looking for
                    // BMP headers and never walks the directory again, so it
                    // emits no scan_found_wav_valid pulses at all. Clearing
                    // wav_found_count on this path would drop wav_found for good
                    // and silence the audio on any card that ever needed the
                    // fallback, with nothing on screen to say why.
                    scan_start_pulse     <= 1'b1;
                    scan_raw_only        <= 1'b1;
                    raw_fallback_started <= 1'b1;
                    img_found_count      <= 3'd0;
                    img_loaded_count     <= 3'd0;
                    next_load_idx        <= 3'd0;
                    img_sector0          <= 32'd0;
                    img_sector1          <= 32'd0;
                    img_sector2          <= 32'd0;
                    img_sector3          <= 32'd0;
                    source_done_seen     <= 1'b0;
                    write_done_seen      <= 1'b0;
                    load_stall_cnt       <= 32'd0;
                    load_retry_cnt       <= 3'd0;
                end else if (scan_done && bmp_ready && !load_busy &&
                             (next_load_idx < img_found_count) &&
                             (img_loaded_count < SCAN_TARGET_COUNT)) begin
                    load_idx         <= next_load_idx[1:0];
                    load_buf_idx     <= img_loaded_count[1:0];
                    load_sector      <= sector_lut(next_load_idx[1:0]);
                    write_buf_idx    <= img_loaded_count[1:0];
                    next_load_idx    <= next_load_idx + 3'd1;
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    load_stall_cnt   <= 32'd0;
                    auto_cnt         <= 32'd0;
                    sec_cnt          <= 3'd0;
                end
            end
        end
    end
end

bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),

    .scan_start             (scan_start_pulse),
    .scan_raw_only          (scan_raw_only),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),
    .scan_found_wav_valid   (scan_found_wav_valid),
    .scan_found_wav_sector  (scan_found_wav_sector),
    .scan_found_wav_size    (scan_found_wav_size),

    .load_start             (load_start_pulse),
    .load_abort             (load_abort),
    .load_sector            (load_sector),
    .load_failed            (load_failed),

    .sd_init_done           (sd_init_done),
    .state_code             (bmp_state_code),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (bmp_sd_sec_read),
    .sd_sec_read_addr       (bmp_sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (bmp_sec_data_valid),
    .sd_sec_read_end        (bmp_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data),
    .src_width              (bmp_src_width),
    .src_height             (bmp_src_height),
    .src_dim_valid          (bmp_src_dim_valid)
);

// Nearest-neighbour scaler, stage 3. Destination geometry is fixed at 640x480
// to match FRAME_WIDTH/FRAME_HEIGHT in frame_read_write, because the
// WRITE_V_FLIP row addressing there assumes exactly 640 pixels per stream row.
scaler_nn #(
    .DST_W                  (640),
    .DST_H                  (480),
    .MAX_UPSCALE            (4),
    // 4096 parked source pixels. Sized in scaler_nn.v for the black border
    // rows, which emit without consuming: off_y rows of 2 * DST_W cycles park
    // off_y * 1280 / 96 pixels of a source stream that cannot be paused.
    .SK_AW                  (12),
    .STALL_THRESH           (384)
) scaler_nn_m0 (
    .clk                    (clk),
    .rst                    (rst),
    .i_src_valid            (bmp_data_wr_en),
    .i_src_pixel            (bmp_data),
    .i_dim_valid            (bmp_src_dim_valid),
    .i_src_w                (bmp_src_width),
    .i_src_h                (bmp_src_height),
    .i_fifo_usedw           (write_fifo_usedw),
    .o_dst_valid            (scaler_dst_valid),
    .o_dst_pixel            (scaler_dst_pixel),
    .o_busy                 (),
    .o_done                 (),
    // Sticky "the elastic buffer overflowed, this picture is wrong" flag, left
    // open on purpose: it is a simulation and bring-up hook, and 4096 entries
    // cover the border row bound for every geometry bmp_read accepts. Hook it
    // to the OSD if a source ever needs more slack than that.
    .o_overflow             ()
);

// Sticky high-water mark of the audio FIFO write side. The streamer refills in
// 128-word sectors and the read side drains at 48 kHz, so the live occupancy is
// useless on a 200 Hz multiplexed display; the peak is not. Zero here means not
// one frame ever reached the FIFO.
always @(posedge clk or posedge rst) begin
    if (rst)
        dbg_aud_wr_peak <= 4'd0;
    else if (aud_fifo_wrusedw[8:5] > dbg_aud_wr_peak)
        dbg_aud_wr_peak <= aud_fifo_wrusedw[8:5];
end

// Music streamer. Shares the SD sector-read port with bmp_read through the
// arbiter above, one sector at a time. start is audio_phase gated by the track
// switch handshake, so it rises with the first picture under
// AUDIO_START_ON_FIRST_IMAGE, loops the selected track, and drops for at most one
// sector whenever the picture changes and a different track belongs to it. Its
// FIFO write side is routed straight out to the top level, where the async FIFO
// crosses into video_clk.
sd_audio_stream #(
    .HDR_LEN                (44),
    .PAUSE_THRESH           (9'd256)
) sd_audio_stream_m0 (
    .clk                    (clk),
    .rst                    (rst),
    .start                  (aud_start),
    .wav_start_sector       (wav_sector),
    .wav_size               (wav_size),
    .sd_sec_read            (aud_sd_sec_read),
    .sd_sec_read_addr       (aud_sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (aud_sec_data_valid),
    .sd_sec_read_end        (aud_sec_read_end),
    .fifo_we                (aud_fifo_we),
    .fifo_di                (aud_fifo_di),
    .fifo_wrusedw           (aud_fifo_wrusedw),
    .dbg_ever_we            (aud_dbg_ever_we),
    .dbg_fault              (aud_dbg_fault),
    .stream_idle            (aud_stream_idle)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,
    output reg  press_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
