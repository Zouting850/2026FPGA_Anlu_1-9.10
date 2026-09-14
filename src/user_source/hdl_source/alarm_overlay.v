// Full-screen emergency alarm layer. The last mux in the video chain: it sits
// after marquee_overlay and replaces every pixel, so the banner, the OSD panel,
// the spectrum visualiser and the TF picture are all gone without any of them
// having to know the alarm exists.
//
// Why it is procedural. The only art source this board has is the TF card, and a
// 640x480 24-bit image takes ~1.8 s to load -- three orders of magnitude off the
// <= 1 frame the alarm path promises. So everything on this screen is generated
// from x_pos/y_pos at draw time: no frame buffer, no SDRAM traffic, no dependency
// on a card being present, correctly formatted or still spinning. Pulling the TF
// card out mid-alarm changes nothing here, which is the sentence the whole design
// exists to make true.
//
// Why it does not endanger timing. Every overlay in this chain is a combinational
// output mux, and top's own comment records that video_brightness -> video_fade ->
// audio_visualizer -> osd_overlay -> rgb_to_axis is already the critical path
// (27.3 ns / 20 levels / 12.3 ns of margin). This layer adds ONE mux to that
// chain. Its own decode cone hangs off x_pos/y_pos registers -- which are at the
// START of the cycle, not the end -- so it runs in parallel with the pixel path
// rather than in series with it, and the glyph table is a 4608-bit case LUT
// (smaller than the marquee's own table already in the shipped build), not a
// memory.
//
// The screen, top to bottom:
//   inset border      6 px white rule, 12 px in from the panel edge. It stays
//                     lit in both flash phases on purpose: the field behind it
//                     is what pulses, so the frame is always legible.
//   headline          4 glyphs, 48 px tall, from the dedicated 24x24 table scaled
//                     x2 (the marquee's 15-cell slogan table is 15x wider and the
//                     alarm must not depend on it)
//   chevron row       ">" arrows marching right, drawn as a wrapped sum so the
//                     shape costs bit slices instead of a multiplier
//   hazard stripes    bottom band, the standard caution-tape diagonal
//
// Type codes drive content, and only content: 1 fire, 2 evac are red, 3 is amber,
// the headline words differ, and the flash rate differs. Nothing about the type
// changes latency, priority or the release rule.
//
// Geometry is mirrored in tools/gen_alarm_font.py, which refuses to emit unless
// these numbers stay mutually consistent.
module alarm_overlay #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire        I_clk,             // video_clk
    input  wire        I_rst,
    input  wire        I_de,
    input  wire [23:0] I_rgb,             // passed through when the alarm is down
    input  wire        I_en,              // frame-atomic alarm enable
    input  wire [1:0]  I_type,            // 1 fire / 2 evac / 3 general
    output wire [23:0] O_rgb
);

// ---- geometry, shared with tools/gen_alarm_font.py --------------------------
localparam BORDER_IN      = 12;             // inset of the outer rectangle
localparam BORDER_W       = 6;              // its thickness
localparam BORDER_X0      = BORDER_IN;                      // 12
localparam BORDER_X0_LAST = BORDER_IN + BORDER_W - 1;       // 17
localparam BORDER_X1      = H_ACTIVE  - BORDER_IN - BORDER_W;  // 622
localparam BORDER_X1_LAST = H_ACTIVE  - BORDER_IN - 1;      // 627
localparam BORDER_Y1      = V_ACTIVE  - BORDER_IN - BORDER_W;  // 462
localparam BORDER_Y1_LAST = V_ACTIVE  - BORDER_IN - 1;      // 467

localparam HEAD_N      = 4;               // glyphs on the headline
localparam GLYPH       = 24;              // table cell, and it is scaled x2
localparam HEAD_PITCH  = 64;              // 2 * GLYPH + 16: power of two, so cell
                                          // and column stay bit slices
localparam HEAD_INK    = GLYPH * 2;       // 48
localparam HEAD_X      = (H_ACTIVE - HEAD_N * HEAD_PITCH) / 2;   // 192
localparam HEAD_X_LAST = HEAD_X + HEAD_N * HEAD_PITCH - 1;       // 447
localparam HEAD_GUT    = (HEAD_PITCH - HEAD_INK) / 2;            // 8
localparam HEAD_GUT_LAST = HEAD_GUT + HEAD_INK - 1;              // 55
localparam HEAD_Y      = 76;
localparam HEAD_Y_LAST = HEAD_Y + HEAD_INK - 1;                  // 123

// A chevron is drawn in a 32 x 16 logical cell and scaled x2 on both axes, so
// the real period is 64 px and the real row block 32 px tall.
localparam ARROW_LOG_ROWS  = 16;
localparam [3:0] ARROW_MID_W = 4'd15;      // logical row whose column offset is 0
localparam ARROW_PITCH     = 64;           // 2 * 32 logical px
localparam ARROW_Y         = 196;
localparam ARROW_H         = ARROW_LOG_ROWS * 2;               // 32
localparam ARROW_Y_LAST    = ARROW_Y + ARROW_H - 1;            // 227

localparam TAPE_PITCH  = 64;              // yellow/black period along x+y
localparam TAPE_H      = 60;
localparam TAPE_Y      = V_ACTIVE - 92;                        // 388
localparam TAPE_Y_LAST = TAPE_Y + TAPE_H - 1;                  // 447

// ---- palette ---------------------------------------------------------------
localparam [23:0] FIRE_HI   = 24'hE01818;
localparam [23:0] FIRE_LO   = 24'h880000;
localparam [23:0] GEN_HI    = 24'hD08800;
localparam [23:0] GEN_LO    = 24'h784A00;
localparam [23:0] INK_WHITE = 24'hFFFFFF;
localparam [23:0] INK_BLACK = 24'h101010;
localparam [23:0] TAPE_YEL  = 24'hF2C400;

// Width-tagged copies, the way osd_overlay and marquee_overlay do it, so every
// compare below is against a 10-bit constant and never extends x_pos/y_pos.
localparam [9:0] H_ACTIVE_W     = H_ACTIVE;
localparam [9:0] V_ACTIVE_W     = V_ACTIVE;
localparam [9:0] BORDER_IN_W     = BORDER_X0;
localparam [9:0] BORDER_IN_L_W   = BORDER_X0_LAST;
localparam [9:0] BORDER_X1_W     = BORDER_X1;
localparam [9:0] BORDER_X1_L_W   = BORDER_X1_LAST;
localparam [9:0] BORDER_Y1_W     = BORDER_Y1;
localparam [9:0] BORDER_Y1_L_W   = BORDER_Y1_LAST;
localparam [9:0] HEAD_X_W        = HEAD_X;
localparam [9:0] HEAD_X_LAST_W   = HEAD_X_LAST;
localparam [9:0] HEAD_Y_W        = HEAD_Y;
localparam [9:0] HEAD_Y_LAST_W   = HEAD_Y_LAST;
localparam [9:0] ARROW_Y_W       = ARROW_Y;
localparam [9:0] ARROW_Y_LAST_W  = ARROW_Y_LAST;
localparam [9:0] TAPE_Y_W        = TAPE_Y;
localparam [9:0] TAPE_Y_LAST_W   = TAPE_Y_LAST;
localparam [5:0] HEAD_GUT_W      = HEAD_GUT;
localparam [5:0] HEAD_GUT_L_W    = HEAD_GUT_LAST;

// ---- raster tracker, identical to osd_overlay and marquee_overlay -----------
reg [9:0] x_pos;
reg [9:0] y_pos;
reg       de_d;
reg [5:0] fc;            // free-running frame counter; a wire IS the flash period
reg [4:0] arrow_phase;   // logical units, +1 per frame, wraps mod 32 for free
reg [5:0] tape_phase;    // real units,   +2 per frame, wraps mod 64 for free

wire        frame_wrap = !I_de && de_d && (y_pos == V_ACTIVE_W - 10'd1);

wire [1:0] W_type = (I_type == 2'd0) ? 2'd3 : I_type;   // never an undefined code
wire       is_gen = (W_type == 2'd3);

// Flash period by type: 8 / 16 / 32 frames (134 / 269 / 538 ms at 59.52 Hz). The
// period is a power of two so the "modulo" is one wire of the free-running
// counter, and picking which wire is a 2-deep mux on the type -- no divider, no
// comparator chain, and the flash can never drift relative to vsync because the
// counter only ever advances at frame_wrap.
wire blink = ~(W_type == 2'd1 ? fc[3] : (W_type == 2'd2 ? fc[4] : fc[5]));

wire [23:0] bg_rgb = is_gen ? (blink ? GEN_HI  : GEN_LO)
                            : (blink ? FIRE_HI : FIRE_LO);

// ---- headline --------------------------------------------------------------
// HEAD_PITCH is 64 for exactly the reason the marquee makes about its own 32 px
// pitch: the cell index and the column inside it fall out as bit slices, never a
// divide.
wire in_head_y = (y_pos >= HEAD_Y_W) && (y_pos <= HEAD_Y_LAST_W);
wire in_head_x = (x_pos >= HEAD_X_W) && (x_pos <= HEAD_X_LAST_W);
wire [9:0] head_yrel = y_pos - HEAD_Y_W;        // 0..47, only read inside in_head_y
wire [9:0] head_xrel = x_pos - HEAD_X_W;        // 0..255, likewise
wire [4:0] head_row  = head_yrel[5:1];          // /2 -> 0..23
wire [3:0] head_cell = head_xrel[7:6];          // /64 -> 0..3
wire [5:0] head_col  = head_xrel[5:0];          // inside the pitch, 0..63
wire       head_ink_col = (head_col >= HEAD_GUT_W) && (head_col <= HEAD_GUT_L_W);
wire [5:0] head_gut = head_col - HEAD_GUT_W;    // 0..47
wire [4:0] head_gcol = head_gut[5:1];           // /2 -> 0..23

// Which four of the eight glyphs, by type. A 12-bit constant -- four 3-bit
// indexes -- so this stays one LUT word, and the words live here rather than in
// the font file because the font is glyphs, not messages.
//   glyph table: 0 JIN  1 JI  2 SHU  3 SAN  4 HUO  5 JING  6 CHU  7 KOU (the
//   eight glyphs, romanised; alarm_font.vh carries the shapes and names them).
// The hex is NOT the index sequence: the fields are 3 bits, so 1 fire = HUO JING
// SHU SAN = 100_101_010_011 = 0x953, 2 evac = JIN JI SHU SAN = 0x053 and 3 general
// = JIN JI CHU KOU = 0x077 -- not 0x4523 and not 0x123. Writing the digits straight
// across silently renders SHU HUO HUO SAN. gen_alarm_font.py derives these from its
// own WORDS table and refuses to emit if they stop matching, so change words there.
wire [11:0] W_msg = (W_type == 2'd1) ? 12'h953 :
                    (W_type == 2'd2) ? 12'h053 : 12'h077;
wire [2:0] msg_idx = (head_cell == 2'd0) ? W_msg[11:9] :
                     (head_cell == 2'd1) ? W_msg[8:6]  :
                     (head_cell == 2'd2) ? W_msg[5:3]  : W_msg[2:0];
// `cell_idx`, not `cell`: cell is a Verilog-2001 reserved word and TD rejects it
// with HDL-8007.
wire [23:0] head_bits = alarm_glyph(msg_idx, head_row);
wire head_on = in_head_y && in_head_x && head_ink_col &&
               head_bits[5'd23 - head_gcol];

// ---- chevrons --------------------------------------------------------------
// One logical ">" is a stroke whose column window is 16..23 of the 32-logical-px
// period at the apex rows and 9..16 at the top and bottom rows, so ink is a
// window on (col + phase + set(row)). Everything is taken modulo the period,
// which a 5-bit sum does for free, and the 8-logical-px window is just the top
// two bits both set -- no comparator, no subtractor chain.
wire in_arrow_y = (y_pos >= ARROW_Y_W) && (y_pos <= ARROW_Y_LAST_W);
// Slicing a subtraction is not legal in Verilog-2001, so the row offset is a wire.
wire [9:0] arrow_yrel = y_pos - ARROW_Y_W;                    // 0..31 when gated
wire [3:0] arrow_row = arrow_yrel[4:1];                       // /2 -> 0..15
wire [3:0] arrow_rise = (arrow_row <= 4'd7) ? arrow_row : (ARROW_MID_W - arrow_row);
wire [3:0] arrow_set  = ARROW_MID_W - arrow_rise;             // 8..15, logical px
wire [4:0] arrow_q = x_pos[5:1] - arrow_phase + arrow_set;
wire arrow_on = in_arrow_y && (arrow_q[4] & arrow_q[3]);

// ---- hazard tape -----------------------------------------------------------
// (x + y) constant is a 45 degree line, so a window on the low 6 bits of that sum
// is a set of parallel diagonal bands with one XOR-free test.
wire in_tape_y = (y_pos >= TAPE_Y_W) && (y_pos <= TAPE_Y_LAST_W);
wire [5:0] tape_s = x_pos[5:0] + y_pos[5:0] - tape_phase;
wire       tape_yellow = tape_s[5];

// ---- border ---------------------------------------------------------------
// One rectangle containment test ANDed with "within BORDER_W px of any of its
// four sides" -- 8 compares on x_pos/y_pos, all in parallel with the pixel path.
wire in_rect = (x_pos >= BORDER_IN_W) && (x_pos <= BORDER_X1_L_W) &&
               (y_pos >= BORDER_IN_W) && (y_pos <= BORDER_Y1_L_W);
wire near_side = (x_pos <= BORDER_IN_L_W) || (x_pos >= BORDER_X1_W) ||
                 (y_pos <= BORDER_IN_L_W) || (y_pos >= BORDER_Y1_W);
wire on_border = in_rect && near_side;

// All three ink arms drive INK_WHITE, so ORing them is exactly equivalent to a
// priority chain no matter how they overlap -- the chevrons do cut straight
// through the border's vertical arms -- and it costs this chain one mux level
// instead of three. Do not "optimise" by dropping a term on the theory that its
// band is covered: only the colour is redundant here, not the geometry.
wire ink_on = head_on || on_border || arrow_on;

assign O_rgb = !(I_en && I_de) ? I_rgb :
               ink_on    ? INK_WHITE :
               in_tape_y ? (tape_yellow ? TAPE_YEL : INK_BLACK) :
                           bg_rgb;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        x_pos       <= 10'd0;
        y_pos       <= 10'd0;
        de_d        <= 1'b0;
        fc          <= 6'd0;
        arrow_phase <= 5'd0;
        tape_phase  <= 6'd0;
    end else begin
        de_d <= I_de;

        if (I_de) begin
            if (!de_d) begin
                x_pos <= 10'd1;
            end else if (x_pos == H_ACTIVE_W - 10'd1) begin
                x_pos <= 10'd0;
            end else begin
                x_pos <= x_pos + 10'd1;
            end
        end else begin
            x_pos <= 10'd0;

            // frame_wrap is de_d && y_pos == V_ACTIVE-1 here, since I_de is 0 on
            // this branch -- the same edge osd_overlay and marquee_overlay count on.
            if (frame_wrap) begin
                y_pos       <= 10'd0;
                fc          <= fc + 6'd1;
                arrow_phase <= arrow_phase + 5'd1;
                tape_phase  <= tape_phase + 6'd2;
            end else if (de_d) begin
                y_pos <= y_pos + 10'd1;
            end
        end
    end
end

`include "alarm_font.vh"

endmodule
