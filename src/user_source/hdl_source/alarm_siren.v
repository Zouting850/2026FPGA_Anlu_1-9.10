// Alarm siren synthesiser. Produces the 24-bit PCM pair that owns the HDMI audio
// output for as long as the emergency takeover is held, in the video_clk domain,
// out of nothing but counters -- no TF card, no WAV file, no BRAM waveform table,
// no DSP. That is the whole point of this file: the sound of an alarm must not
// depend on a storage medium that can be missing, unseated, corrupt or busy
// loading the next image, which is exactly the failure mode a CPU-based signage
// player has and a counter does not.
//
// Three types, three cadences. The type code is the one emergency_ctrl already
// resolved (1 fire < 2 evac < 3 general, lowest non-zero wins), so nothing here
// re-derives priority:
//
//   T1 fire     800 Hz <-> 1000 Hz two-tone, 341 ms each   -> the wailing siren
//   T2 evac     1000 Hz pulsed, 170 ms on / 170 ms off     -> the guidance preamble
//   T3 general  700 Hz continuous                          -> a single long tone
//
// The cadence is two bits of one free-running 48 kHz tick counter, which is why
// the numbers above are 341/170 ms and not 300/200: mod_cnt[14] and mod_cnt[13]
// cost one wire each, while "exactly 400 ms" would cost a 32-bit comparator and a
// reload per pattern. Nothing about an alarm's recognisability depends on the
// difference, and the counter is held at zero while I_en is low so a takeover
// always starts on the LOUD edge instead of the silent half of its own window.
//
// The waveform is a square, from bit 31 of the phase accumulator, exactly as
// hdmi_audio_tone_i2s_64fs.v:132 does for the built-in test tone. A triangle was
// tried on paper first and rejected on cost: folding the phase into a bipolar ramp
// needs an inverter, a 16-bit mux and a subtractor (~50 slices this build cannot
// spare for cosmetics) and a siren's identity is its frequency pattern, not its
// harmonic content -- the HDMI sink and the speaker low-pass it anyway. The extra
// harmonics also give the spectrum bars on screen something to draw, which is the
// only place anyone will see this waveform.
//
// Peak amplitude is the test tone's own AMP (8_000_000 of a 8_388_607 full scale),
// so taking over changes the loudness of the line, not its level scale, and the
// visualizer's calibration (tuned against a steady 2_000_000) stays meaningful.
//
// Two contracts this module must keep, both inherited:
//
//   * O_audio_valid is a 48 kHz pulse train, asserted once per tick, ALWAYS. It is
//     never gated by I_en, by the pattern gate, or by mute. audio_arc_calculate
//     counts 48 valids per CTS and the HDMI audio clock regeneration rides on that;
//     a gap de-locks the audio link, and the symptom would be "the siren stopped
//     working" when what actually broke is the reference. Silence is expressed by
//     zeroing the SAMPLES. Same rule audio_pcm_player.v:54 states outright.
//   * I_en / I_type arrive on the FAST 2FF path (top's emg_act_v1 / emg_type_v1),
//     not the frame-atomic one. Sound must not wait for a frame boundary; pixels
//     must. See top's merge-point comment for why the split is deliberate.
//
// Retreat: EMERGENCY_ENABLE gates the entire body of the always block, so at 0
// every register here has zero loads and synthesis prunes the instance -- proven
// the only way it can be proven, by the post-route area hierarchy row, not by the
// parameter value. phase_acc is the one register with no reset on purpose: it is
// only ever advanced (reloading it per cycle is what would need the multiplier
// this file forbids), it feeds nothing until the first tick, and 32 FFs of reset
// fanout bought with that would be a bad trade.
module alarm_siren #(
    parameter  EMERGENCY_ENABLE = 1'b1,
    parameter integer CLK_FREQ_HZ    = 25_000_000,
    parameter integer SAMPLE_RATE_HZ = 48_000
)(
    input  wire        I_clk,             // video_clk, 25 MHz
    input  wire        I_rst,             // rst_all, active high

    input  wire        I_en,              // 1 = the alarm owns the audio output
    input  wire [1:0]  I_type,            // 1 fire / 2 evac / 3 general

    output reg         O_audio_valid,
    output reg  [23:0] O_audio_left_data,
    output reg  [23:0] O_audio_right_data
);

localparam [1:0] TYPE_FIRE   = 2'd1;
localparam [1:0] TYPE_EVAC   = 2'd2;
localparam [1:0] TYPE_GENERAL = 2'd3;

// Direct digital synthesis steps, step = round(f * 2^32 / 48000). Rounding leaves
// the worst case at 4 uHz (about 5e-9 relative), which is why there is no
// multiplier anywhere in this file: the tuning is a constant, not a divide.
localparam [31:0] STEP_700   = 32'd62634940;   // -> 700.000003 Hz
localparam [31:0] STEP_800   = 32'd71582788;   // -> 799.999997 Hz
localparam [31:0] STEP_1000  = 32'd89478485;   // -> 999.999996 Hz

localparam signed [23:0] AMP = 24'sd8000000;   // = hdmi_audio_tone_i2s_64fs's AMP

reg  [31:0] S_tick_acc;     // 25 MHz -> 48 kHz, fractional, reloaded at every tick
reg  [31:0] S_phase_acc;    // DDS phase, advanced only, wraps on its own
reg  [15:0] S_mod_cnt;      // SAMPLE ticks since the takeover began

wire [31:0] W_tick_next = S_tick_acc + SAMPLE_RATE_HZ;
wire        W_tick      = (W_tick_next >= CLK_FREQ_HZ);

// The cadence, as two bits of that counter. Declared once and read by both the
// frequency select and the gate, so T1's alternation and T2's pulsing can never
// drift apart from each other.
wire        W_alt  = S_mod_cnt[14];           // 16384 samples = 341.3 ms
wire        W_pulse = S_mod_cnt[13];          //  8192 samples = 170.7 ms

wire [31:0] W_step = (I_type == TYPE_FIRE) ? (W_alt ? STEP_1000 : STEP_800) :
                     (I_type == TYPE_EVAC) ? STEP_1000 : STEP_700;

// Only TYPE_EVAC is gated. For every other code the tone runs as long as the
// takeover does, which is what a continuous long blast is.
wire        W_audible = I_en && ((I_type == TYPE_EVAC) ? ~W_pulse : 1'b1);

wire signed [23:0] W_square = S_phase_acc[31] ? AMP : -AMP;
wire signed [23:0] W_sample = W_audible ? W_square : 24'sd0;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        S_tick_acc         <= 32'd0;
        S_mod_cnt          <= 16'd0;
        O_audio_valid      <= 1'b0;
        O_audio_left_data  <= 24'd0;
        O_audio_right_data <= 24'd0;
    end else if (EMERGENCY_ENABLE) begin
        // Single-cycle by default; the tick below re-arms it every ~520 clocks.
        O_audio_valid <= 1'b0;

        if (W_tick) begin
            S_tick_acc <= W_tick_next - CLK_FREQ_HZ;
            O_audio_valid <= 1'b1;

            // Once per SAMPLE, not once per clock. Accumulating on every 25 MHz
            // edge would put the tone 520x above the frequency the step constant
            // was derived for.
            S_phase_acc <= S_phase_acc + W_step;

            // Same rule one level up: the cadence counter must advance with the
            // samples, not with the clocks. Left at the module's clock enable
            // this counted 25 MHz, so mod_cnt[13] pulsed every 328 us instead of
            // every 170.7 ms -- an audible 1.5 kHz amplitude modulation that
            // looked like a broken synthesiser rather than an evacuation pattern.
            if (I_en) S_mod_cnt <= S_mod_cnt + 16'd1;
            else      S_mod_cnt <= 16'd0;

            // Mono: an alarm is not a stereo event, and duplicating one register
            // pair onto both channels is what the test tone does too.
            O_audio_left_data  <= W_sample;
            O_audio_right_data <= W_sample;
        end else begin
            S_tick_acc <= W_tick_next;
        end
    end
end

endmodule
