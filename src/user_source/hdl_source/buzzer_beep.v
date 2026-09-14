// On-board passive buzzer (H11, net BUZZER/spk) drive. The board's buzzer is
// passive, so it needs a square wave, not a level: holding a pin high across it
// produces silence and a warm coil. Everything this module does follows from that
// one fact.
//
// It is the second, independent announcement of the same alarm, and the reason to
// have it is a failure the HDMI path can suffer and this cannot: the siren in
// alarm_siren.v is PCM inside an AXI-Stream audio link, so a TV with its volume
// down, a monitor with no audio input selected, or a bad cable all leave the room
// silent while the screen screams. This is a piezo on the FPGA board. It also
// never touches audio_mclk, I2S or the ACR reference, so nothing here can
// interfere with the link it is backing up.
//
// Clock domain: this runs on clk (50 MHz), not video_clk, and that is why its
// inputs are the RAW emergency_ctrl outputs rather than top's crossed copies.
// emg_state / vol_level are already clk-domain registers, so the hardline trigger
// reaches the horn with one synchroniser stage of its own (the 3FF inside
// emergency_ctrl) and zero CDC on the way out. The audio siren has to cross into
// video_clk to be muxed into a PCM stream; this does not have to cross anywhere.
// Measured against the headline claim, that makes the buzzer the fastest
// end-to-end indication on the board: 3FF + debounce make (instant) + 2 divider
// clocks, versus 3FF + 2FF + up to one 16.80 ms frame boundary for the panel.
//
// Pattern. The type code selects how long the horn sounds inside a fixed 600 ms
// window, which is the difference between three alarms you can tell apart with
// your eyes shut:
//
//   T1 fire     600 ms on / 0 off   continuous              -> long blast
//   T2 evac     150 ms on / 450 off 1.67 Hz, sparse        -> the stutter
//   T3 general  400 ms on / 200 off 1.67 Hz, mostly on     -> general warning
//
// T2 and T3 repeat at the same rate on purpose and differ only in duty, which is
// exactly how real buzzer codes are told apart. One counter, one window, three
// constants, one comparator.
//
// The tone frequency is deliberately NOT per type: a piezo has a resonance and
// this board's is not documented, so the honest options are "one frequency that is
// audible" (1 kHz, the range the vendor's own 13_music buzzer demo plays in) and a
// build that gets the pattern right but is quiet on two of its three types. TUNE
// IT ON THE BOARD: TONE_HZ is a parameter for exactly that.
//
// Volume: mute (vol_level 0) is honoured, and it is the ONLY level this module
// can express. A passive piezo has no amplitude control -- the drive is a square
// wave into a reactive load, so "half volume" means pulse-density modulating the
// envelope, which an operator hears as a rough, spitting tone rather than a
// quieter one. The three-level volume ladder therefore lives in the digital audio
// gain in top (full / -12 dB / mute) and the horn is a binary device: it sounds,
// or it has been told not to.
//
// Hardware prerequisite: SW5 must be ON. It is a series switch in the buzzer's
// supply path, not an FPGA signal, so a build that is otherwise perfect is silent
// with SW5 open, and that has the exact symptom of a broken feature.
//
// Retreat: EMERGENCY_ENABLE gates every load in the block below, so at 0 the
// registers have no loads, the instance is pruned, and O_beep comes out of reset
// at 1'b0 and stays there -- the pin is quiescent, which is also the safe state
// for a device that cannot be left half-driven.
module buzzer_beep #(
    parameter  EMERGENCY_ENABLE = 1'b1,
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer TONE_HZ     = 1000,
    parameter integer WINDOW_MS   = 600
)(
    input  wire        I_clk,             // clk, 50 MHz
    input  wire        I_rst,             // rst_all, active high

    input  wire        I_alarm,           // emg_state[2], same domain, no CDC
    input  wire [1:0]  I_type,            // emg_state[1:0]
    input  wire [1:0]  I_level,           // vol_level: 0 = mute, anything else sounds

    output reg         O_beep             // -> spk (H11), 50% duty square wave
);

localparam [1:0] TYPE_FIRE    = 2'd1;
localparam [1:0] TYPE_EVAC    = 2'd2;

localparam integer HALF_TONE  = CLK_FREQ_HZ / TONE_HZ / 2;
localparam integer MS_TICKS   = CLK_FREQ_HZ / 1000;

// Sounding window per type, in ms of the WINDOW_MS period. TYPE_FIRE is the window
// itself, so the comparator is never satisfied and the tone runs continuously.
localparam [9:0] ON_FIRE = WINDOW_MS;
localparam [9:0] ON_EVAC = 150;
localparam [9:0] ON_GEN  = 400;

reg [14:0] S_tone_cnt;      // HALF_TONE - 1 = 24999 at 1 kHz from 50 MHz
reg [15:0] S_ms_cnt;        // MS_TICKS - 1 = 49999
reg [9:0]  S_win_ms;        // position inside the repetition window

wire [9:0] W_on_ms = (I_type == TYPE_FIRE) ? ON_FIRE :
                     (I_type == TYPE_EVAC) ? ON_EVAC : ON_GEN;

// Every comparison is against a value strictly below its counter's range, so the
// TYPE_FIRE case above (ON_FIRE == WINDOW_MS) can never be reached and continuous
// sounding falls out of the arithmetic instead of needing a special case.
wire        W_envelope = I_alarm && (I_level != 2'd0) && (S_win_ms < W_on_ms);
wire [14:0] W_tone_next = S_tone_cnt + 15'd1;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        S_tone_cnt <= 15'd0;
        S_ms_cnt   <= 16'd0;
        S_win_ms   <= 10'd0;
        O_beep     <= 1'b0;
    end else if (EMERGENCY_ENABLE) begin
        // The window ladder runs whenever the alarm is ARMED, never whenever the
        // horn happens to be sounding. Gating it by W_envelope looked equivalent
        // and was not: the instant a burst's on-time expired the envelope went
        // low, the reset branch zeroed the position that had just expired, and the
        // off-period became unreachable -- every type sounded continuously.
        if (I_alarm && (I_level != 2'd0)) begin
            if (S_ms_cnt >= MS_TICKS - 1) begin
                S_ms_cnt <= 16'd0;
                if (S_win_ms >= WINDOW_MS - 1) S_win_ms <= 10'd0;
                else                           S_win_ms <= S_win_ms + 10'd1;
            end else begin
                S_ms_cnt <= S_ms_cnt + 16'd1;
            end
        end else begin
            S_ms_cnt <= 16'd0;
            S_win_ms <= 10'd0;
        end

        if (W_envelope) begin
            // The 50 % duty square wave itself. Toggled, not looked up, so the
            // waveform stays exactly half-and-half at any TONE_HZ that divides.
            if (S_tone_cnt >= HALF_TONE - 1) begin
                S_tone_cnt <= 15'd0;
                O_beep     <= ~O_beep;
            end else begin
                S_tone_cnt <= W_tone_next;
            end
        end else begin
            // Silence is a driven 0, not a floating half-state: a piezo left at
            // whatever the last edge happened to be is a DC step across the coil.
            S_tone_cnt <= 15'd0;
            O_beep     <= 1'b0;
        end
    end
end

endmodule
