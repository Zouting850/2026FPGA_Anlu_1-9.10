// ---------------------------------------------------------------------------
// rx_byte_cdc.v  (Board B -- UDP RX byte stream, udp_clk 125M -> clk_50m 50M)
//
// Why this exists
//   The vendor UDP/IP stack pushes each received payload byte on
//   app_rx_data_valid / app_rx_data in the udp_clk (125 MHz) domain with NO
//   backpressure -- it is a pure source. The whole pixel pipeline downstream
//   (rx_frame_to_bmp -> bmp_decode -> scaler_nn) runs on clk_50m (50 MHz) for
//   timing safety, because scaler_nn's fill/drain sequencer is calibrated for
//   Board A's ~25 MHz sd_card_clk budget and 125 MHz would be marginal. This
//   module is the elastic crossing between the two: a SHOW_AHEAD async soft FIFO
//   (the vendor fifo_sdr_data_2 already generated for app_wrrd) one byte wide.
//
// Read protocol (SHOW_AHEAD)
//   In SHOW_AHEAD mode dout always holds the head byte and valid == !empty_flag,
//   so out_valid = !empty and the head is advanced by asserting re for one
//   clk_50m cycle. rx_frame_to_bmp consumes exactly one byte on every cycle it
//   is offered one (it has no ready/backpressure output by design), so re =
//   !empty IS the entire read controller: pop whenever a byte is present, the
//   consumer always takes it. No byte is popped that is not simultaneously
//   presented on out_byte, so there is neither loss nor duplication.
//
// Sizing and the PC pacing contract (Option A)
//   The 50 MHz consumer drains at most 50 MB/s; a gigabit RX burst can momentarily
//   push toward 125 MB/s. A DEPTH-deep FIFO absorbs the per-packet burst: a 1440
//   byte UDP payload arrives in ~11.5 us at line rate, during which the consumer
//   drains ~576 bytes, so a single packet needs < 900 bytes of headroom and 4096
//   is comfortable. Sustained lossless operation therefore only requires the PC
//   sender to keep the AVERAGE rate under ~40 MB/s, which a Python socket sender
//   does naturally through per-packet syscall overhead; #22 records it as an
//   explicit contract. If the average is exceeded the FIFO overflows and silently
//   drops a byte (inside fifo_sdr_data_2 wr_en_s = !full_flag & we); that shows up
//   downstream as a truncated frame which rx_frame_to_bmp's idle watchdog drops,
//   and the PC re-sends. `overflow` latches that event for bring-up visibility.
//
// Reset: rst ACTIVE HIGH and asynchronous, matching the vendor `reset` net that
//   already drives the UDP stack. fifo_sdr_data_2 synchronises the release into
//   each clock domain internally (asy_w_rst / asy_r_rst chains), so a single
//   active-high reset is correct for both sides.
// ---------------------------------------------------------------------------
module rx_byte_cdc #(
    parameter integer ADDR_WIDTH = 12        // 4096 bytes deep, see sizing note
)(
    input  wire        wr_clk,               // udp_clk, 125 MHz (write side)
    input  wire        rst,                  // active high, async
    input  wire        in_valid,             // app_rx_data_valid
    input  wire [7:0]  in_byte,              // app_rx_data

    input  wire        rd_clk,               // clk_50m, 50 MHz (read side)
    output wire        out_valid,            // -> rx_frame_to_bmp.in_valid
    output wire [7:0]  out_byte,             // -> rx_frame_to_bmp.in_byte

    output reg         overflow              // sticky: a byte was dropped (rst clears)
);
    wire        empty;
    wire        full;
    wire [7:0]  dout;

    fifo_sdr_data_2 #(
        .DATA_WIDTH_W (8),
        .ADDR_WIDTH_W (ADDR_WIDTH),
        .DATA_WIDTH_R (8),
        .ADDR_WIDTH_R (ADDR_WIDTH),
        .SHOW_AHEAD_EN(1'b1)
    ) u_rx_fifo (
        .rst       (rst),
        .clkw      (wr_clk),
        .we        (in_valid),
        .di        (in_byte),
        .clkr      (rd_clk),
        .re        (!empty),                 // pop whenever a byte is present
        .dout      (dout),
        .valid     (),
        .full_flag (full),
        .empty_flag(empty),
        .afull     (),
        .aempty    (),
        .wrusedw   (),
        .rdusedw   ()
    );

    assign out_valid = !empty;
    assign out_byte  = dout;

    // Sticky overflow indicator, write-domain detection (full is a wr-domain flag).
    always @(posedge wr_clk or posedge rst) begin
        if (rst)                    overflow <= 1'b0;
        else if (in_valid && full)  overflow <= 1'b1;
    end
endmodule
