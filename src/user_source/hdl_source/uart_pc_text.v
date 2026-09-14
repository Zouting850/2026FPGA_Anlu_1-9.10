// ---------------------------------------------------------------------------
// uart_pc_text
//
// clk(50 MHz)-domain second UART that turns a PC (Type-C -> on-board CH340 ->
// F12) into a subtitle keyboard. It is COMPLETELY INDEPENDENT of the serial
// screen on J1 (uart_screen_ctrl): separate RX pin, separate parser, separate
// effects. Nothing here touches the screen path.
//
// Protocol (see the README's PC subtitle channel section):
//   - RX is 8N1 at BAUD (default 9600), mid-bit sampling, LSB first -- the exact
//     RX engine cloned from uart_screen_ctrl, which is board-proven.
//   - A frame is ASCII payload terminated by three consecutive 0xFF bytes (the
//     TJC convention this project already uses). Payload is 0x20..0x7E, never
//     0xFF, so 0xFF unambiguously means terminator.
//   - Frames (keywords uppercase, case-sensitive, like the screen protocol):
//       "PCTX 0" / "PCTX 1"  -> leave / enter PC-subtitle mode (pc_text_en).
//       "TEXT <1..24 chars>"  -> load the scrolling string. Does NOT change
//                                pc_text_en; the init button owns that.
//     Byte 4 is always the ' ' separator; PCTX's argument is byte 5; TEXT's
//     payload starts at byte 5. Internal spaces inside TEXT are kept (0x20 is a
//     legal payload char).
//   - Over-long TEXT is truncated at PC_CAP (24): the 11-bit borrow math in
//     marquee_overlay (s = x_pos + marq_pos) overflows above 24 cells, so the
//     cap is enforced HERE at the source, and marquee_overlay saturates again as
//     a boundary guard.
//
// Effects, all clk domain:
//   - pc_text_en   : level, 2FF-crossed into video_clk by the top.
//   - char_buf     : 32 x 7-bit ASCII (cell_idx*7 +: 7). Written live during a
//                    TEXT frame; only meaningful after the commit strobe.
//   - n_cells      : committed character count (1..24).
//   - text_toggle  : flips once per committed TEXT. The top uses data+toggle
//                    CDC (the IMGX idiom) and latches char_buf/n_cells
//                    frame-atomically at video_frame_start (the filt_frame
//                    idiom), so a string can never tear mid-frame.
//   - dbg_rx_toggle / dbg_commit_toggle : observation-only, feed led[3]; not
//     part of the protocol.
//
// Reset polarity follows the project convention: rst is rst_all, ACTIVE HIGH,
// `posedge rst` + `if (rst)`. Do not flip it.
//
// Stage 1 is RX-only: there is no TX engine and no readback to the PC, so the
// PC software is fire-and-forget and the dbg toggles are the only link window.
// ---------------------------------------------------------------------------

module uart_pc_text #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD        = 9600
)(
    input  wire        clk,
    input  wire        rst,                    // active high (rst_all)
    input  wire        uart_pc_rx,             // F12

    // ---- committed state, clk domain ----
    output reg         pc_text_en,             // 1 = show PC string, 0 = slogan
    output reg  [223:0] char_buf,              // 32 x 7-bit ASCII
    output reg  [5:0]  n_cells,                // committed length, 1..PC_CAP
    output reg         text_toggle,            // flips per committed TEXT (CDC)

    // ---- link debug (led[3]); observation-only ----
    output reg         dbg_rx_toggle,          // flips once per received byte
    output reg         dbg_commit_toggle       // flips once per committed frame
);

    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD;   // 5208 @ 50M/9600
    localparam [5:0]   PC_CAP       = 6'd24;                // display cap (see above)
    localparam [223:0] CHAR_BUF_0   = 224'd0;

    localparam [1:0] MODE_IGNORE = 2'd0,
                     MODE_PCTX   = 2'd1,
                     MODE_TEXT   = 2'd2;

    // ------------------------------------------------------------------
    // UART RX: 8N1, mid-bit sampling, LSB first. Cloned verbatim from the
    // board-proven uart_screen_ctrl RX so the two links behave identically.
    // ------------------------------------------------------------------
    localparam [1:0] RX_IDLE  = 2'd0,
                     RX_START = 2'd1,
                     RX_DATA  = 2'd2,
                     RX_STOP  = 2'd3;

    reg [1:0]  rx_sync;
    reg [1:0]  rx_state;
    reg [15:0] rx_cnt;
    reg [2:0]  rx_bit;
    reg [7:0]  rx_shift;
    reg [7:0]  rx_byte;
    reg        rx_valid;

    wire       rx_in = rx_sync[1];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_sync  <= 2'b11;
            rx_state <= RX_IDLE;
            rx_cnt   <= 16'd0;
            rx_bit   <= 3'd0;
            rx_shift <= 8'd0;
            rx_byte  <= 8'd0;
            rx_valid <= 1'b0;
        end else begin
            rx_sync  <= {rx_sync[0], uart_pc_rx};
            rx_valid <= 1'b0;
            case (rx_state)
            RX_IDLE: begin
                rx_cnt <= 16'd0;
                rx_bit <= 3'd0;
                if (rx_in == 1'b0)
                    rx_state <= RX_START;
            end
            RX_START: begin
                if (rx_cnt == (CLKS_PER_BIT-1)/2) begin
                    if (rx_in == 1'b0) begin
                        rx_cnt   <= 16'd0;
                        rx_state <= RX_DATA;
                    end else begin
                        rx_state <= RX_IDLE;
                    end
                end else begin
                    rx_cnt <= rx_cnt + 16'd1;
                end
            end
            RX_DATA: begin
                if (rx_cnt == CLKS_PER_BIT-1) begin
                    rx_cnt   <= 16'd0;
                    rx_shift <= {rx_in, rx_shift[7:1]};
                    rx_bit   <= rx_bit + 3'd1;
                    if (rx_bit == 3'd7)
                        rx_state <= RX_STOP;
                end else begin
                    rx_cnt <= rx_cnt + 16'd1;
                end
            end
            RX_STOP: begin
                if (rx_cnt == CLKS_PER_BIT-1) begin
                    rx_cnt   <= 16'd0;
                    rx_state <= RX_IDLE;
                    if (rx_in == 1'b1) begin
                        rx_byte  <= rx_shift;
                        rx_valid <= 1'b1;
                    end
                end else begin
                    rx_cnt <= rx_cnt + 16'd1;
                end
            end
            default: rx_state <= RX_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Parser: accumulate ASCII, dispatch on the third 0xFF.
    // ------------------------------------------------------------------
    reg [31:0] kw;             // bytes 0..3, the keyword
    reg [5:0]  bc;             // payload byte index
    reg        sep_ok;         // byte 4 was ' '
    reg [1:0]  mode;
    reg        pctx_valid;     // a legal '0'/'1' argument was seen
    reg        pending_en;
    reg [5:0]  char_count;     // chars stored so far this TEXT frame
    reg [1:0]  ffc;

    wire is_ff      = (rx_byte == 8'hFF);
    wire sep_byte   = (rx_byte == 8'h20);
    wire payload_ok = (rx_byte >= 8'h20) && (rx_byte <= 8'h7E);
    // char_count is capped below PC_CAP, so the widest byte offset is 23*7=161;
    // the base must be 8 bits or the part-select index silently truncates.
    wire [7:0] char_base_w = char_count * 8'd7;
    wire do_store   = (mode == MODE_TEXT) && sep_ok && (bc >= 6'd5) &&
                      payload_ok && (char_count < PC_CAP);
    wire pctx_arg   = (bc == 6'd5) && (mode == MODE_PCTX) && sep_ok &&
                      ((rx_byte == 8'h30) || (rx_byte == 8'h31));
    wire pctx_bad   = (bc >= 6'd6) && (mode == MODE_PCTX);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            kw                <= 32'd0;
            bc                <= 6'd0;
            sep_ok            <= 1'b0;
            mode              <= MODE_IGNORE;
            pctx_valid        <= 1'b0;
            pending_en        <= 1'b0;
            char_count        <= 6'd0;
            ffc               <= 2'd0;
            pc_text_en        <= 1'b0;
            char_buf          <= CHAR_BUF_0;
            n_cells           <= 6'd0;
            text_toggle       <= 1'b0;
            dbg_rx_toggle     <= 1'b0;
            dbg_commit_toggle <= 1'b0;
        end else begin
            if (rx_valid) begin
                dbg_rx_toggle <= ~dbg_rx_toggle;

                if (is_ff) begin
                    if (ffc == 2'd2) begin
                        // ---- third 0xFF: frame complete, commit ----
                        if ((mode == MODE_PCTX) && pctx_valid)
                            pc_text_en <= pending_en;
                        if ((mode == MODE_TEXT) && sep_ok && (char_count >= 6'd1)) begin
                            n_cells           <= char_count;
                            text_toggle       <= ~text_toggle;
                            dbg_commit_toggle <= ~dbg_commit_toggle;
                        end
                        bc         <= 6'd0;
                        kw         <= 32'd0;
                        sep_ok     <= 1'b0;
                        mode       <= MODE_IGNORE;
                        pctx_valid <= 1'b0;
                        char_count <= 6'd0;
                        ffc        <= 2'd0;
                    end else begin
                        ffc <= ffc + 2'd1;
                    end
                end else begin
                    // ---- payload byte ----
                    ffc <= 2'd0;

                    if (bc < 6'd4)
                        kw <= {kw[23:0], rx_byte};
                    if (bc == 6'd4) begin
                        sep_ok <= sep_byte;
                        mode   <= (kw == "PCTX") ? MODE_PCTX :
                                  (kw == "TEXT") ? MODE_TEXT : MODE_IGNORE;
                    end
                    if (pctx_arg) begin
                        pending_en <= (rx_byte == 8'h31);
                        pctx_valid <= 1'b1;
                    end
                    if (pctx_bad)
                        pctx_valid <= 1'b0;
                    if (do_store) begin
                        char_buf[char_base_w +: 7] <= rx_byte;
                        char_count <= char_count + 6'd1;
                    end
                    if (bc < 6'd63)
                        bc <= bc + 6'd1;
                end
            end
        end
    end

endmodule
