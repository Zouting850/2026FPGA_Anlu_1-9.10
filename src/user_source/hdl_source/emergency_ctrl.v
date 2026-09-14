// Emergency command / hardline fusion. The whole feature lives behind
// EMERGENCY_ENABLE, which gates every load in this module -- not just the output
// strobes -- so a build with the parameter at 0 leaves no register here for
// synthesis to keep (the led[3] lesson in top's merge-point comment).
//
// Three requestors, one answer. A UART command (ALRM n) is what an operator or a
// serial screen sends; T1 and T2 are dry contacts on J2 that a manual call point
// and a smoke loop close. Whichever is live decides the alarm, and the alarm only
// ends when ALL of them are clear -- release is an AND, never last-writer-wins,
// because the command that clears must not be able to silence a fire contact that
// is still closed.
//
//                     +- UART  ALRM n -- req_cmd -+
//   J2 M3 (T1) -3FF-debounce- t1_lvl - MANUAL_TYPE +- min_live - emg_state -+- 2FF --> audio, buzzer
//   J2 M4 (T2) -3FF-debounce- t2_lvl - AUTO_TYPE  -/  lowest non-zero  type -/
//                                                        code wins           3FF+edge -> overlay
//
// The type codes are chosen so that severity is arithmetic: 1 fire < 2 evac <
// 3 general, and 0 means "this requestor is not asking". Taking the lowest non-zero
// code is then the whole priority table, with no case statement to keep in sync.
//
// One crossing, one same-domain tap -- deliberately different (tools/sim_emergency.py
// measures all three latencies):
//   * O_emg_state[2] as a bare 2FF level -> the siren. Sound must not wait for a
//     frame boundary; a silent half-frame at the start of an alarm is the part
//     that actually matters.
//   * O_emg_state + O_emg_tgl as data+toggle -> video_clk staging, committed at
//     video_frame_start -> the overlay. Pixels must not change mid-frame or the
//     screen tears a red/normal seam across itself.
//   * The buzzer crosses nothing. It lives on I_clk, the same domain this module
//     is in, so the horn is the fastest end-to-end indication the board has:
//     three synchroniser FFs and a debounce gate, no CDC chain and no frame
//     boundary. top's port map is what keeps that true, and Pass G pins it there.
//
// That split is the same one the PC subtitle channel already uses in top
// (pc_en_v0/v1 bare, pc_char_buf + pc_text_toggle staged and frame-latched), so
// the timing argument has one precedent in this file rather than two.
module emergency_ctrl #(
    parameter  EMERGENCY_ENABLE = 1'b1,
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer RELEASE_MS  = 20,       // break-side debounce window
    parameter [1:0] MANUAL_TYPE   = 2'd2,     // T1 = manual call point -> evac
    parameter [1:0] AUTO_TYPE     = 2'd1,     // T2 = smoke/heat contact -> fire
    parameter integer BURN_SEC    = 0         // 0 = test sequencer disabled
)(
    input  wire        I_clk,             // clk, 50 MHz
    input  wire        I_rst,             // rst_all, active high

    // Dry contacts. PULLUPed, and a firing device pulls the line to GND, so the
    // alarm state is a synced 0 -- see the .adc entries for M3/M4.
    input  wire        I_t1,
    input  wire        I_t2,

    // Merged command source from top (J1 serial screen or Type-C, already
    // priority-selected there). Not frozen by the alarm: ALRM 0 is how it ends.
    input  wire        I_cmd_emg_set,
    input  wire [1:0]  I_cmd_emg,         // 0 = release, 1..3 = type
    input  wire        I_cmd_vol_set,
    input  wire [1:0]  I_cmd_vol,         // 2 full / 1 mid / 0 mute

    output reg  [2:0]  O_emg_state,       // {active, type[1:0]}
    output reg         O_emg_tgl,         // companion toggle, flips with any change
    output reg  [1:0]  O_vol_level
);

localparam integer DEBOUNCE_TICKS = (CLK_FREQ_HZ / 1000) * RELEASE_MS;
localparam integer BURN_TICKS     = CLK_FREQ_HZ * BURN_SEC;

localparam [1:0] VOL_FULL = 2'd2;

// ---------------------------------------------------------------------------
// Inputs: 3FF, the same depth key_press_debounce uses, so the level the debouncer
// acts on is never itself metastable.
// ---------------------------------------------------------------------------
reg        t1_s0, t1_s1, t1_s2;
reg        t2_s0, t2_s1, t2_s2;
reg        t1_lvl, t2_lvl;                // debounced: 1 = asking for an alarm
reg [31:0] t1_cnt, t2_cnt;                // break-side countdown, in clk ticks
reg [1:0]  req_cmd;                       // what the last ALRM n said

// Burn-in sequencer registers. It publishes a value plus a valid flag rather than
// writing req_cmd itself: two always blocks driving one reg is illegal Verilog,
// and even if it compiled, the "operator wins this cycle" rule below would be a
// race instead of a mux.
reg [31:0] burn_cnt;
reg        burn_on;
reg [1:0]  burn_type;
reg [1:0]  burn_req;
reg        burn_req_v;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        t1_s0 <= 1'b1; t1_s1 <= 1'b1; t1_s2 <= 1'b1;
        t2_s0 <= 1'b1; t2_s1 <= 1'b1; t2_s2 <= 1'b1;
        t1_lvl <= 1'b0; t1_cnt <= 32'd0;
        t2_lvl <= 1'b0; t2_cnt <= 32'd0;
        req_cmd <= 2'd0;
    end else begin
        t1_s0 <= I_t1; t1_s1 <= t1_s0; t1_s2 <= t1_s1;
        t2_s0 <= I_t2; t2_s1 <= t2_s0; t2_s2 <= t2_s1;

        // Asymmetric, and the asymmetry is the point.
        //
        // MAKE follows the synced level with no wait. Pushing an alarm through a
        // symmetric 20 ms debouncer costs one frame on the wire that is supposed
        // to be the fast one -- 20 ms of debounce against a 16.80 ms frame always
        // crosses a second frame boundary and the "<= 1 frame" claim dies.
        // tools/sim_emergency.py Pass E runs exactly that as its negative control.
        //
        // BREAK needs the whole window, because the line opens when the device
        // stops firing and that is the noisy edge: a hand brushing a Dupont line,
        // a relay chattering as a smoke loop clears. Every active sample re-arms
        // the countdown, so a bounce restarts it from the far end.
        if (!EMERGENCY_ENABLE) begin
            t1_lvl <= 1'b0;
            t2_lvl <= 1'b0;
        end else begin
            if (t1_s2 == 1'b0) begin
                t1_lvl <= 1'b1;
                t1_cnt <= DEBOUNCE_TICKS;
            end else if (t1_cnt != 32'd0) begin
                t1_cnt <= t1_cnt - 32'd1;
            end else begin
                t1_lvl <= 1'b0;
            end

            if (t2_s2 == 1'b0) begin
                t2_lvl <= 1'b1;
                t2_cnt <= DEBOUNCE_TICKS;
            end else if (t2_cnt != 32'd0) begin
                t2_cnt <= t2_cnt - 32'd1;
            end else begin
                t2_lvl <= 1'b0;
            end
        end

        // ALRM n, or the burn-in stand-in for it when that sequencer is enabled.
        if (EMERGENCY_ENABLE && (I_cmd_emg_set || burn_req_v))
            req_cmd <= I_cmd_emg_set ? I_cmd_emg : burn_req;
    end
end

// ---------------------------------------------------------------------------
// Burn-in sequencer, gone at build time when BURN_SEC = 0. It walks
// fire -> evac -> general -> release forever, which is the only way a 7x24 soak can
// exercise takeover, siren and layer without someone typing at a terminal.
// ---------------------------------------------------------------------------
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        burn_cnt   <= 32'd0;
        burn_on    <= 1'b0;
        burn_type  <= 2'd1;
        burn_req   <= 2'd0;
        burn_req_v <= 1'b0;
    end else if (EMERGENCY_ENABLE && BURN_SEC != 0) begin
        burn_req_v <= 1'b0;
        if (burn_cnt + 32'd1 >= BURN_TICKS) begin
            burn_cnt <= 32'd0;
            burn_on  <= ~burn_on;
            if (!burn_on) begin
                burn_type <= (burn_type == 2'd3) ? 2'd1 : (burn_type + 2'd1);
                burn_req  <= (burn_type == 2'd3) ? 2'd1 : (burn_type + 2'd1);
                burn_req_v <= 1'b1;
            end else begin
                burn_req   <= 2'd0;
                burn_req_v <= 1'b1;
            end
        end else begin
            burn_cnt <= burn_cnt + 32'd1;
        end
    end
end

// ---------------------------------------------------------------------------
// Resolve. Lowest non-zero code wins, and 0 means "not asking", so a clear
// requestor drops out of the min and the release is an AND over the ones left.
// ---------------------------------------------------------------------------
function [1:0] min_live;
    input [1:0] a;
    input [1:0] b;
    begin
        if (a == 2'd0)      min_live = b;
        else if (b == 2'd0) min_live = a;
        else                min_live = (a < b) ? a : b;
    end
endfunction

wire [1:0] W_t2   = t2_lvl ? AUTO_TYPE   : 2'd0;
wire [1:0] W_t1   = t1_lvl ? MANUAL_TYPE : 2'd0;
wire [1:0] W_type = min_live(min_live(W_t2, W_t1), req_cmd);

// A retreated build must not leave this pair ticking, so the gate is on the state
// itself and not only on the loads above.
wire [2:0] W_state = EMERGENCY_ENABLE ? {W_type != 2'd0, W_type} : 3'b000;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        O_emg_state <= 3'd0;
        O_emg_tgl   <= 1'b0;
        O_vol_level <= VOL_FULL;
    end else begin
        if (W_state != O_emg_state) begin
            O_emg_state <= W_state;
            O_emg_tgl   <= ~O_emg_tgl;
        end
        if (EMERGENCY_ENABLE && I_cmd_vol_set) O_vol_level <= I_cmd_vol;
    end
end

endmodule
