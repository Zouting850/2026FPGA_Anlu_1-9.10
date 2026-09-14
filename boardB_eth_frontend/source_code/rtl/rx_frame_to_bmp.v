// ---------------------------------------------------------------------------
// rx_frame_to_bmp.v  (Board B -- UDP byte stream -> bmp_decode file stream)
//
// What it does
//   The PC sends each image as one logical byte stream spread over many UDP
//   packets:
//       [4-byte SOF magic][4-byte little-endian file length][file length bytes]
//   The bytes arrive from the async RX FIFO (udp_rx_clk -> clk_50m) one per
//   cycle on in_valid/in_byte. This module re-frames that stream for
//   bmp_decode.v: it hunts the SOF magic, captures the transport length, pulses
//   `start` ONCE (alone, a cycle ahead of the first file byte -- bmp_decode's
//   verified arm contract), then forwards exactly `file length` bytes as
//   out_valid/out_byte and re-hunts for the next image.
//
// Why the length rides in the transport header (not derived from bmp_decode)
//   Framing is deliberately INDEPENDENT of decode success. bmp_decode.busy is
//   hdr_ok ? (byte_cnt<file_len) : 1'b1, so a corrupt header (hdr_ok never
//   latches) leaves busy stuck high forever; ending a frame on busy would then
//   wedge the framer and swallow every later image. Counting transport bytes
//   instead means a bad BMP still completes its frame and re-arms cleanly, and
//   bmp_decode rejects it internally. The only failure counting cannot cover is
//   a TRUNCATED stream (UDP loss => fewer bytes than the header promised), so a
//   single idle watchdog abandons a frame that stops receiving bytes and
//   re-hunts. That is the whole error model: one counter, one sticky-ish pulse.
//
// Why a magic at all, and why it is safe
//   The SOF magic delimits one image from the next on a stream that is
//   otherwise just bytes. It is hunted ONLY in S_HUNT (idle). It is never
//   re-detected mid-frame, because the 4 magic bytes can occur inside BMP pixel
//   data and a mid-stream match would truncate the image. In S_STREAM the frame
//   ends on the byte count, never on a pattern. The match detector restarts
//   correctly on a partial match (the magic A5,5A,5A,A5 repeats A5 at both ends,
//   so a mismatch falls back to "is this byte M0", not to zero).
//
// Reassembly / ordering assumption
//   The async FIFO concatenates packet payloads in arrival order, so packet
//   boundaries are invisible here and the magic may span them. This assumes
//   in-order UDP delivery, which holds on a direct PC->Board B link (the same
//   assumption the national-first-prize reference's headerless chunker makes).
//   Out-of-order/loss recovery (a per-packet sequence number) is out of scope
//   for this static-image path; loss shows up as a truncated frame the watchdog
//   drops, and the PC re-sends.
//
// Interface (all clk_50m, the Board B pixel-pipeline clock)
//   in_valid/in_byte : a stream byte present THIS cycle, consumed this cycle
//                      (no backpressure -- bmp_decode and the scaler's elastic
//                      buffer always accept; pacing is the PC's job per the
//                      Option-A contract, source <= 640x480).
//   start            : one-cycle arm pulse to bmp_decode, alone, before byte 0.
//   out_valid/out_byte : forwarded file bytes to bmp_decode.in_valid/in_byte.
//   idle_timeout_err : bring-up hook, pulses when the watchdog abandons a frame.
//
// Reset: rst ACTIVE HIGH (posedge rst), matching bmp_decode.v / scaler_nn.v.
//   The Board B top owns the inversion from the vendor rst_n.
// ---------------------------------------------------------------------------
module rx_frame_to_bmp #(
    parameter [31:0] SOF_MAGIC = 32'hA55A_5AA5,  // wire order MSB-first: A5,5A,5A,A5
    parameter integer IDLE_AW  = 25              // watchdog = 2^(IDLE_AW-1) cycles
                                                 // = 16.7M = 0.335s @ 50MHz; must
                                                 // exceed the PC's max chunk gap
)(
    input                 clk,
    input                 rst,                  // active high

    input                 in_valid,
    input  [7:0]          in_byte,

    output reg            start,                // -> bmp_decode.start
    output reg            out_valid,            // -> bmp_decode.in_valid
    output reg [7:0]      out_byte,             // -> bmp_decode.in_byte
    output reg            idle_timeout_err      // bring-up hook
);
    localparam [1:0] S_HUNT   = 2'd0;
    localparam [1:0] S_LEN    = 2'd1;
    localparam [1:0] S_STREAM = 2'd2;

    // Magic bytes, MSB-first, derived from the parameter so they cannot drift.
    localparam [7:0] M0 = SOF_MAGIC[31:24];
    localparam [7:0] M1 = SOF_MAGIC[23:16];
    localparam [7:0] M2 = SOF_MAGIC[15:8];
    localparam [7:0] M3 = SOF_MAGIC[7:0];

    reg [1:0]  state;
    reg [2:0]  magic_idx;     // 0..4 matched magic bytes so far
    reg [1:0]  len_idx;       // 0..3 length bytes captured
    reg [31:0] file_len_t;    // transport file length (bytes to forward)
    reg [31:0] byte_remain;   // file bytes still to forward in S_STREAM

    reg [IDLE_AW-1:0] idle_cnt;
    wire idle_sat = idle_cnt[IDLE_AW-1];

    // ------------------------------------------------------------------
    // Idle watchdog: counts consecutive cycles with no in_valid while a frame
    // is open (S_LEN or S_STREAM). Held at zero in S_HUNT (idling there is
    // normal) and cleared on any received byte. Saturates and stops.
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst)                 idle_cnt <= {IDLE_AW{1'b0}};
        else if (state == S_HUNT) idle_cnt <= {IDLE_AW{1'b0}};
        else if (in_valid)        idle_cnt <= {IDLE_AW{1'b0}};
        else if (!idle_sat)       idle_cnt <= idle_cnt + 1'b1;
    end

    // ------------------------------------------------------------------
    // Framing FSM.
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state            <= S_HUNT;
            magic_idx        <= 3'd0;
            len_idx          <= 2'd0;
            file_len_t       <= 32'd0;
            byte_remain      <= 32'd0;
            start            <= 1'b0;
            out_valid        <= 1'b0;
            out_byte         <= 8'd0;
            idle_timeout_err <= 1'b0;
        end else begin
            // Defaults: start, out_valid and the error pulse are one-cycle.
            start            <= 1'b0;
            out_valid        <= 1'b0;
            idle_timeout_err <= 1'b0;

            case (state)
                // ------------------------------------------------------
                S_HUNT: begin
                    if (in_valid) begin
                        case (magic_idx)
                            3'd0: magic_idx <= (in_byte == M0) ? 3'd1 : 3'd0;
                            3'd1: magic_idx <= (in_byte == M1) ? 3'd2 :
                                               ((in_byte == M0) ? 3'd1 : 3'd0);
                            3'd2: magic_idx <= (in_byte == M2) ? 3'd3 :
                                               ((in_byte == M0) ? 3'd1 : 3'd0);
                            3'd3: begin
                                if (in_byte == M3) begin
                                    // Full magic seen: open the length phase.
                                    state     <= S_LEN;
                                    magic_idx <= 3'd0;
                                    len_idx   <= 2'd0;
                                end else begin
                                    magic_idx <= (in_byte == M0) ? 3'd1 : 3'd0;
                                end
                            end
                            default: magic_idx <= 3'd0;
                        endcase
                    end
                end

                // ------------------------------------------------------
                S_LEN: begin
                    if (idle_sat) begin
                        // Truncated header: abandon, re-hunt.
                        state            <= S_HUNT;
                        len_idx          <= 2'd0;
                        idle_timeout_err <= 1'b1;
                    end else if (in_valid) begin
                        case (len_idx)
                            2'd0: begin file_len_t[7:0]   <= in_byte; len_idx <= 2'd1; end
                            2'd1: begin file_len_t[15:8]  <= in_byte; len_idx <= 2'd2; end
                            2'd2: begin file_len_t[23:16] <= in_byte; len_idx <= 2'd3; end
                            2'd3: begin
                                // b3 is on in_byte now; file_len_t[31:24] not yet
                                // registered, so assemble byte_remain from both.
                                file_len_t[31:24] <= in_byte;
                                byte_remain       <= {in_byte, file_len_t[23:0]};
                                len_idx           <= 2'd0;
                                state             <= S_STREAM;
                                start             <= 1'b1;   // arm bmp_decode, alone
                            end
                            default: len_idx <= 2'd0;
                        endcase
                    end
                end

                // ------------------------------------------------------
                S_STREAM: begin
                    if (idle_sat) begin
                        // Truncated payload (UDP loss): drop the partial frame.
                        state            <= S_HUNT;
                        magic_idx        <= 3'd0;
                        byte_remain      <= 32'd0;
                        idle_timeout_err <= 1'b1;
                    end else if (in_valid) begin
                        out_byte  <= in_byte;
                        out_valid <= 1'b1;
                        if (byte_remain == 32'd1) begin
                            byte_remain <= 32'd0;
                            state       <= S_HUNT;   // frame complete, re-hunt
                            magic_idx   <= 3'd0;
                        end else begin
                            byte_remain <= byte_remain - 32'd1;
                        end
                    end
                end

                default: state <= S_HUNT;
            endcase
        end
    end

endmodule
