`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// sdram_to_spi.v  (Board B -- SDRAM readback -> SPI master push to Board A)
//
// Replaces the example's udp_loopback on the SDRAM READ side. app_wrrd reads the
// scaled 640x480 frame back out of SDRAM (sdr_clk = 125MHz domain) exactly the
// way it used to feed udp_loopback; this module catches those pixels in an async
// FIFO, hands them to spi_master_tx (clk_50m domain), and feeds the FIFO write
// occupancy back to app_wrrd's udp_wrusedw port so the readback is throttled to
// the SPI drain rate -- the identical backpressure contract udp_loopback had
// (app_wrrd pauses reads while udp_wrusedw >= 2048, resumes below it).
//
// Why a FIFO and not a direct wire: the SDRAM readback bursts at 125MHz while SPI
// drains one pixel per 24 SCK periods (~192 clk_50m cycles at SCK=6.25MHz), a
// ~50x rate difference across two clock domains. The 4096-deep fifo_sdr_data_2
// (the same async SHOW-AHEAD IP app_wrrd already uses) absorbs it; app_wrrd's
// <2048 gating keeps it at most half full, so it can neither overflow nor, given
// the readback is far faster than the drain, underrun mid-frame.
//
// Frame re-arm: frame_rst (a short clk_50m pulse derived from the new image's
// rx_frame_to_bmp.start, stretched in board_b_top) flushes the FIFO and aborts
// any half-sent SPI frame, then leaves awaiting_frame set. The next frame's first
// readback pixel (FIFO goes non-empty) pulses spi frame_start exactly once.
// app_wrrd is reset by the same frame_rst (synchronised into sdr_clk inside
// sdram_top) so its one-shot readback re-arms too; because that reset clears
// wr_done, no stale pixel can be pushed into the just-flushed FIFO -- the new
// readback only begins after the whole new frame has been written to SDRAM.
//
// Reset polarity: rst_n ACTIVE LOW async (Board B vendor-tree convention, matches
// spi_master_tx). frame_rst ACTIVE HIGH. The FIFO's own rst is ACTIVE HIGH async
// and synchronises its own release into each clock domain internally.
// ---------------------------------------------------------------------------
module sdram_to_spi #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer SCK_DIV     = 8,            // 8 -> 6.25MHz SCK
    parameter integer PIXELS      = 307200,       // 640*480, must match app_wrrd
    parameter [31:0]  MAGIC       = 32'hA55A_5AA5 // must match spi_master_tx / Board A
)(
    // ---- SDRAM readback side (sdr_clk = 125MHz, from sdram_top) ----
    input  wire        sdr_clk,
    input  wire        Sdr_rd_en,      // readback strobe: Sdr_rd_dout valid this cycle
    input  wire [23:0] Sdr_rd_dout,    // readback pixel
    output wire [11:0] udp_wrusedw,    // -> app_wrrd read gating (pauses at >=2048)

    // ---- SPI / control side (clk = clk_50m = 50MHz) ----
    input  wire        clk,
    input  wire        rst_n,          // power-on reset, active low async
    input  wire        frame_rst,      // re-arm pulse, active high (clk domain)

    output wire        spi_sck,
    output wire        spi_mosi,
    output wire        spi_cs_n,
    output reg         frame_done      // 1-clk pulse when a full frame finished
);

    wire        fifo_empty;
    wire        fifo_full;
    wire [23:0] fifo_dout;
    wire        spi_pixel_ready;
    wire        spi_busy;

    reg         awaiting_frame;
    reg         frame_start;
    reg         busy_d;

    // ------------------------------------------------------------------
    // Readback pixel FIFO: sdr_clk write -> clk read. SHOW-AHEAD, so fifo_dout is
    // always the head pixel and ~fifo_empty its valid -- which is precisely the
    // pixel_valid/pixel pair spi_master_tx wants. we=Sdr_rd_en is safe ungated
    // because app_wrrd already throttles the readback at udp_wrusedw<2048 and the
    // IP ignores we when full internally.
    // ------------------------------------------------------------------
    fifo_sdr_data_2 #(
        .DATA_WIDTH_W (24),
        .ADDR_WIDTH_W (12),
        .DATA_WIDTH_R (24),
        .ADDR_WIDTH_R (12),
        .SHOW_AHEAD_EN(1'b1)
    ) u_rd_fifo (
        .rst        (frame_rst | ~rst_n),          // async active-high: flush on re-arm, held during power-on
        .clkw       (sdr_clk),
        .clkr       (clk),
        .we         (Sdr_rd_en),
        .di         (Sdr_rd_dout),
        .re         (spi_pixel_ready & ~fifo_empty),
        .dout       (fifo_dout),
        .valid      (),
        .full_flag  (fifo_full),
        .empty_flag (fifo_empty),
        .afull      (),
        .aempty     (),
        .wrusedw    (udp_wrusedw),                // write-domain occupancy -> app_wrrd
        .rdusedw    ()
    );

    // ------------------------------------------------------------------
    // One frame_start pulse per re-arm, on the first pixel of the new readback.
    // awaiting_frame is set by frame_rst (and at power-on) and cleared the cycle
    // frame_start fires, so a frame can never be double-triggered. Gated on
    // ~spi_busy so a re-arm that lands mid-transmission waits for SPI to idle
    // (SPI is also held in reset by frame_rst, so in practice it is already idle).
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awaiting_frame <= 1'b1;
            frame_start    <= 1'b0;
        end else begin
            frame_start <= 1'b0;
            if (frame_rst)
                awaiting_frame <= 1'b1;
            else if (awaiting_frame && !fifo_empty && !spi_busy) begin
                frame_start    <= 1'b1;
                awaiting_frame <= 1'b0;
            end
        end
    end

    // frame_done: falling edge of spi busy = a whole frame went out the wire.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy_d     <= 1'b0;
            frame_done <= 1'b0;
        end else begin
            busy_d     <= spi_busy;
            frame_done <= busy_d && !spi_busy;
        end
    end

    // ------------------------------------------------------------------
    // SPI master push engine. rst_n is gated with ~frame_rst so a re-arm cleanly
    // aborts a half-sent frame (CS_n returns high, engine returns to S_IDLE)
    // instead of leaving the new frame's pixels appended to the old one.
    // ------------------------------------------------------------------
    spi_master_tx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .SCK_DIV    (SCK_DIV),
        .PIXELS     (PIXELS),
        .MAGIC      (MAGIC)
    ) u_spi (
        .clk          (clk),
        .rst_n        (rst_n & ~frame_rst),
        .frame_start  (frame_start),
        .pixel_valid  (~fifo_empty),
        .pixel        (fifo_dout),
        .pixel_ready  (spi_pixel_ready),
        .busy         (spi_busy),
        .spi_sck      (spi_sck),
        .spi_mosi     (spi_mosi),
        .spi_cs_n     (spi_cs_n)
    );

endmodule
