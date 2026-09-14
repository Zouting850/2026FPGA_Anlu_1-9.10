`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// spi_master_tx  (Board B -> Board A image push link, SPI MASTER, mode 0)
//
// Role in the two-board network-image system:
//   Board B receives a BMP from the PC over Ethernet, decodes + scales it to
//   640x480 RGB888, and pushes the resulting pixel stream to Board A over a
//   3-wire SPI link (SCK / MOSI / CS_n + GND) wired on the J1/J2 headers.
//   Board A is the SPI SLAVE; its RX + CDC FIFO + frame-buffer write-port mux
//   are a SEPARATE design and must mirror the contract below exactly.
//
// Why SPI and not a parallel/LVDS bus: the link is Dupont flying leads (not
// impedance-controlled) and the payload is a STATIC image (latency irrelevant),
// so a master-paced synchronous serial link is the robust choice and needs no
// PLL on either side -- critical because Board B is already at 4/4 PLL with the
// Ethernet stack alone. SPI also gives flow control for free: the master paces,
// so no backpressure wire is needed as long as Board A's CDC FIFO drains faster
// than SCK delivers (true: SCK <=12.5MHz vs Board A's sd_card_clk write side).
//
// Wire protocol (THE CONTRACT -- Board A slave must match this bit for bit):
//   - SPI mode 0: CPOL=0 (SCK idle low), CPHA=0 (slave samples on RISING edge).
//     MOSI is set up at the start of each SCK low phase and is stable well
//     before the rising edge. 50% duty SCK.
//   - CS_n falling edge = start of frame. CS_n stays low for the whole frame.
//   - First word: 32-bit MAGIC (default 0xA55A5AA5), sent MSB-first. Board A
//     validates this before trusting the rest; a mismatch means the frame is
//     noise / out of sync and is dropped.
//   - Then PIXELS pixels, each 24-bit RGB888, sent MSB-first:
//     pixel[23], pixel[22], ... pixel[0].  (pixel[23:16]/[15:8]/[7:0] = the
//     three colour bytes; the exact R/G/B vs B/G/R assignment is decided by the
//     scaler that feeds `pixel`, and Board A reproduces it verbatim.)
//   - CS_n rising edge = end of frame.
//   Total bits per frame = 32 + PIXELS*24. At SCK=CLK/SCK_DIV the frame time is
//   (32 + PIXELS*24) * SCK_DIV / CLK_FREQ_HZ seconds (640x480 @ 6.25MHz ~1.18s).
//
// Upstream handshake (all in `clk` domain):
//   - frame_start: pulse to begin pushing a fresh frame. IGNORED while busy.
//   - pixel_valid / pixel / pixel_ready: standard valid-ready. The engine stalls
//     in S_PIX_LOAD with SCK held low if pixel_valid is deasserted, so the
//     source (a FIFO draining the scaler / SDRAM read) sets the pace; nothing is
//     dropped. pixel_ready pulses for one clk per consumed pixel.
//   - busy: high from the frame_start accept until CS_n returns high.
//
// Reset: rst_n ACTIVE LOW, asynchronous -- matches the Board B vendor tree
// (sys_rst_n / rst_n). NOTE this is the OPPOSITE polarity of Board A's user
// modules (rst active high); the new Board B top owns the inversion. Do not flip.
//
// Parameters:
//   SCK_DIV : clks per SCK period. Must be EVEN and >=4 (so the low phase, the
//             rising edge, and the high phase are each at least one clk).
//             SCK = CLK_FREQ_HZ / SCK_DIV. 8 -> 6.25MHz, 4 -> 12.5MHz.
//   PIXELS  : pixels per frame, 640*480 = 307200 by default.
// ---------------------------------------------------------------------------
module spi_master_tx #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer SCK_DIV     = 8,
    parameter integer PIXELS      = 307200,
    parameter [31:0]  MAGIC       = 32'hA55A_5AA5
)(
    input  wire        clk,            // clk_50m (Board B)
    input  wire        rst_n,          // active low, async

    // ---- upstream pixel stream (clk domain), valid/ready ----
    input  wire        frame_start,    // pulse: a fresh scaled frame is ready
    input  wire        pixel_valid,
    input  wire [23:0] pixel,
    output reg         pixel_ready,    // pulse: pixel consumed
    output wire        busy,

    // ---- SPI master physical outputs (to Board A slave) ----
    output reg         spi_sck,
    output reg         spi_mosi,
    output reg         spi_cs_n
);
    // div must hold 0..SCK_DIV-1; pix_cnt must hold 0..PIXELS; bit_cnt 0..32.
    localparam integer DIVW = (SCK_DIV <= 2) ? 1 : $clog2(SCK_DIV);
    localparam integer PIXW = $clog2(PIXELS + 1);
    localparam integer HALF = SCK_DIV / 2;

    localparam [2:0] S_IDLE     = 3'd0,
                     S_MAGIC    = 3'd1,
                     S_PIX_LOAD = 3'd2,
                     S_PIX_SHIFT= 3'd3,
                     S_END      = 3'd4;

    reg [2:0]      state;
    reg [DIVW-1:0] div;
    reg [5:0]      bit_cnt;     // bits clocked out in the current word (0..31)
    reg [5:0]      bit_total;   // 32 for MAGIC, 24 for a pixel
    reg [31:0]     shift;       // MSB-first shift register; bit out = shift[31]
    reg [PIXW-1:0] pix_cnt;     // pixels sent so far this frame

    assign busy = (state != S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            div         <= {DIVW{1'b0}};
            bit_cnt     <= 6'd0;
            bit_total   <= 6'd0;
            shift       <= 32'd0;
            pix_cnt     <= {PIXW{1'b0}};
            pixel_ready <= 1'b0;
            spi_sck     <= 1'b0;
            spi_mosi    <= 1'b0;
            spi_cs_n    <= 1'b1;
        end else begin
            pixel_ready <= 1'b0;          // default: single-clk pulse

            case (state)
            // ---------------------------------------------------------- idle
            S_IDLE: begin
                spi_cs_n <= 1'b1;
                spi_sck  <= 1'b0;
                div      <= {DIVW{1'b0}};
                pix_cnt  <= {PIXW{1'b0}};
                if (frame_start) begin
                    spi_cs_n  <= 1'b0;            // frame begins (falling edge)
                    spi_mosi  <= MAGIC[31];       // first bit, set up early
                    shift     <= MAGIC;
                    bit_total <= 6'd32;
                    bit_cnt   <= 6'd0;
                    div       <= {DIVW{1'b0}};
                    state     <= S_MAGIC;
                end
            end

            // ---------------------------------- clock out MAGIC or one pixel
            S_MAGIC, S_PIX_SHIFT: begin
                if (div == 0)
                    spi_mosi <= shift[31];                  // set up this bit
                if (div == HALF-1)
                    spi_sck  <= 1'b1;                       // rising edge: sample
                if (div == SCK_DIV-1) begin
                    spi_sck <= 1'b0;                        // falling edge
                    shift   <= {shift[30:0], 1'b0};
                    bit_cnt <= bit_cnt + 6'd1;
                    div     <= {DIVW{1'b0}};
                    if (bit_cnt == bit_total - 6'd1) begin  // word complete
                        if (state == S_PIX_SHIFT)
                            pix_cnt <= pix_cnt + 1'b1;
                        state <= S_PIX_LOAD;
                    end
                end else begin
                    div <= div + 1'b1;
                end
            end

            // ------------------------------------- fetch next pixel or finish
            S_PIX_LOAD: begin
                spi_sck <= 1'b0;                            // SCK idle low
                div     <= {DIVW{1'b0}};
                bit_cnt <= 6'd0;
                if (pix_cnt == PIXELS[PIXW-1:0]) begin
                    state <= S_END;                         // whole frame sent
                end else if (pixel_valid) begin
                    pixel_ready <= 1'b1;                    // consume pixel
                    shift       <= {pixel, 8'b0};           // pixel[23] -> shift[31]
                    bit_total   <= 6'd24;
                    spi_mosi    <= pixel[23];               // first bit of pixel
                    state       <= S_PIX_SHIFT;
                end
                // else: stall (SCK low) until the source presents a pixel
            end

            // ------------------------------------------------------- end frame
            S_END: begin
                spi_cs_n <= 1'b1;                           // rising edge = EOF
                spi_sck  <= 1'b0;
                spi_mosi <= 1'b0;
                state    <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
