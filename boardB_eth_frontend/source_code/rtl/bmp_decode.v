// ---------------------------------------------------------------------------
// bmp_decode.v  (Board B -- streaming BMP byte-stream -> RGB888 source pixels)
//
// Why this module exists
//   Board A's bmp_read.v is a full FAT32 directory scanner + SD-sector BMP
//   loader: it is welded to the sd_sec_read/sd_sec_read_addr/sd_sec_read_data
//   sector handshake and to FAT32 boot/MBR/cluster geometry, and its pixel
//   path is hand-tuned against the sd_card_clk budget. Dragging all of that to
//   Board B and re-backing it with SDRAM would be a large, risky surgery.
//
//   Board B does not have an SD card or a filesystem. It receives a BMP FILE as
//   an in-order UDP byte stream from the PC. Decoding that is just the
//   ST_LOAD_HDR + ST_LOAD_DATA essence of bmp_read -- header parse, geometry
//   validation, 4-byte row-padding drop, and 3-byte->RGB888 packing -- with
//   NONE of the FAT32/SD machinery. This module is that essence, lifted
//   expression-for-expression from the board-proven bmp_read.v so the decode
//   behaviour is bit-identical to the path the user already verified on Board A.
//
// Output interface deliberately MATCHES bmp_read's pixel side
//   src_valid / src_pixel[23:0] / src_width / src_height / src_dim_valid use the
//   same names, widths and semantics as bmp_read's bmp_data_wr_en / bmp_data /
//   src_width / src_height / src_dim_valid, so scaler_nn.v plugs in UNCHANGED.
//   Byte order is identical to bmp_read: a BMP stores B,G,R; byte0->[7:0],
//   byte1->[15:8], byte2->[23:16], i.e. src_pixel = {R,G,B} with R in the high
//   byte. Everything downstream (scaler, SPI link, Board A) inherits that.
//
// Provenance (bmp_read.v line refs, kept identical on purpose):
//   header field offsets ....... lines 377-401
//   geometry validation ........ lines 521-526 (registered; top-down BMP has a
//                               negative biHeight whose upper half is non-zero,
//                               so it is rejected here exactly as on Board A)
//   header_match ............... lines 170-175
//   src_w3 / src_stride ........ lines 187-188, 527-528
//   in_pixel_region ............ lines 181-182
//   row padding drop ........... lines 494-507, 183-185
//   3-byte RGB888 packing ...... lines 542-579
//
// Stream contract (all in `clk` domain):
//   start    : pulse BEFORE a file's first byte. Resets the byte counter, the
//              row-padding counter, the header latches and the done flags, and
//              arms the decoder. The UDP-RX reassembly asserts it on the first
//              packet of a new image.
//   in_valid : high on every cycle a file byte is present on in_byte.
//   in_byte  : the file byte, in file order, starting at 'B'.
//   busy     : high from `start` until byte_cnt reaches file_len (file done).
//   hdr_ok   : latched 1 once the header parses AND validates; 0 if the first
//              34 bytes are not a 24-bit uncompressed BMP within the supported
//              geometry. When hdr_ok is 0 no pixels are emitted.
//   src_dim_valid : one-cycle pulse when the header has validated, carrying
//              src_width/src_height -- a full pixel-region ahead of the first
//              src_valid, so a downstream scaler can be armed in time (this is
//              the same lead bmp_read gives scaler_nn).
//
// Supported geometry (mirrors bmp_read SRC_*_MIN/MAX): W 64..1920, H 64..1080,
//   24-bit, uncompressed (biCompression==0). Outside that, hdr_ok stays 0.
//
// Reset: rst ACTIVE HIGH (posedge rst), matching bmp_read.v / scaler_nn.v --
//   the user pixel-pipeline convention. NOTE this is the OPPOSITE polarity of
//   the Board B vendor tree (rst_n) and of spi_master_tx.v; the new Board B top
//   owns the inversion so all three pixel-pipeline modules stay consistent.
// ---------------------------------------------------------------------------
module bmp_decode(
    input                       clk,
    input                       rst,            // active high

    input                       start,          // pulse: arm for a new file
    input                       in_valid,
    input  [7:0]                in_byte,

    output reg                  src_valid,      // == bmp_read.bmp_data_wr_en
    output reg [23:0]           src_pixel,      // == bmp_read.bmp_data
    output reg [15:0]           src_width,
    output reg [15:0]           src_height,
    output reg                  src_dim_valid,
    output reg                  hdr_ok,
    output                      busy
);
    // Accepted source geometry -- identical bounds to bmp_read.v lines 71-74.
    localparam [15:0] SRC_W_MIN = 16'd64;
    localparam [15:0] SRC_W_MAX = 16'd1920;
    localparam [15:0] SRC_H_MIN = 16'd64;
    localparam [15:0] SRC_H_MAX = 16'd1080;

    // File byte index from the first 'B'. 32 bits like bmp_read.bmp_len_cnt so a
    // multi-MB source file (e.g. 1920x1080x3 ~ 6.2 MB) cannot wrap.
    reg  [31:0] byte_cnt;

    // Header latches (bmp_read.v lines 79-86).
    reg  [7:0]  header_0;
    reg  [7:0]  header_1;
    reg  [31:0] file_len;
    reg  [31:0] pixel_offset;
    reg  [31:0] width;
    reg  [31:0] height;
    reg  [15:0] bit_count;
    reg  [31:0] compression;

    // Registered geometry validation + derived row pitch (bmp_read.v 512-537).
    reg         width_ok_r;
    reg         height_ok_r;
    reg  [15:0] src_w3;       // width*3, the real (unpadded) row byte count
    reg  [15:0] src_stride;   // row byte count padded up to a multiple of 4
    reg         header_match_r;
    reg         dim_pulsed;   // guards src_dim_valid to a single pulse per file

    // Row padding + pixel packing state (bmp_read.v 540-548, 491-507).
    reg  [15:0] row_byte_cnt; // byte index within the current source row
    reg  [1:0]  bmp_byte_idx; // 0,1,2 within one RGB888 triple

    wire [31:0] w3_calc     = width + (width << 1);
    wire [31:0] stride_calc = (w3_calc + 32'd3) & 32'hffff_fffc;

    wire header_match_c = (header_0 == "B") &&
                          (header_1 == "M") &&
                          width_ok_r &&
                          height_ok_r &&
                          (bit_count   == 16'd24) &&
                          (compression == 32'd0);

    // Pixel region + padding gate (bmp_read.v 181-185). row_byte_cnt counts every
    // pixel-region byte (padding included) and wraps at src_stride, so the
    // row_byte_cnt < src_w3 test drops exactly the alignment padding.
    wire in_pixel_region = (byte_cnt >= pixel_offset) &&
                           (byte_cnt <  file_len);
    wire bmp_data_valid  = in_valid && hdr_ok && in_pixel_region &&
                           (row_byte_cnt < src_w3);

    assign busy = hdr_ok ? (byte_cnt < file_len) : 1'b1;

    // ------------------------------------------------------------------
    // Wait for the WHOLE header before deciding.
    //
    // The last field header_match_c reads is `compression`, parsed at byte
    // index 33 and therefore final from byte_cnt == 34 onward; the REGISTERED
    // header_match_r reflects that one cycle later, at byte_cnt == 35. Latching
    // hdr_ok the instant header_match_r first rises (~byte 31) would decide on a
    // compression field that has not arrived yet -- and compression RESETS TO 0,
    // which is the accept value, so an RLE-compressed BMP (compression != 0)
    // would be wrongly accepted during the cycles before its byte 33 lands.
    //
    // bmp_read.v sidesteps this by only sampling header_match_r at end of sector
    // (rd_cnt ~ 511), long after byte 33. byte_cnt >= 35 is the streaming
    // equivalent of that "the whole header is in" guarantee, and is the tightest
    // correct bound: byte_cnt >= 34 would still trust a header_match_r captured
    // before byte 33, so a compression whose only set bit is in byte 33
    // (e.g. 32'h0100_0000) would slip through.
    // ------------------------------------------------------------------
    localparam [31:0] HDR_SETTLE_CNT = 32'd35;
    wire header_settled = (byte_cnt >= HDR_SETTLE_CNT);

    // ------------------------------------------------------------------
    // File byte counter.
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst)
            byte_cnt <= 32'd0;
        else if (start)
            byte_cnt <= 32'd0;
        else if (in_valid)
            byte_cnt <= byte_cnt + 32'd1;
    end

    // ------------------------------------------------------------------
    // Header field parse. byte_cnt is the index of the byte present THIS
    // cycle (it increments non-blocking), so the case matches bmp_read's
    // rd_cnt-indexed parse exactly (bmp_read.v 377-401).
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            header_0     <= 8'd0;
            header_1     <= 8'd0;
            file_len     <= 32'd0;
            pixel_offset <= 32'd54;
            width        <= 32'd0;
            height       <= 32'd0;
            bit_count    <= 16'd0;
            compression  <= 32'd0;
        end else if (start) begin
            header_0     <= 8'd0;
            header_1     <= 8'd0;
            file_len     <= 32'd0;
            pixel_offset <= 32'd54;   // BMP default; overwritten at bytes 10..13
            width        <= 32'd0;
            height       <= 32'd0;
            bit_count    <= 16'd0;
            compression  <= 32'd0;
        end else if (in_valid && (byte_cnt <= 32'd33)) begin
            case (byte_cnt)
                32'd0 : header_0 <= in_byte;
                32'd1 : header_1 <= in_byte;
                32'd2 : file_len[7:0]     <= in_byte;
                32'd3 : file_len[15:8]    <= in_byte;
                32'd4 : file_len[23:16]   <= in_byte;
                32'd5 : file_len[31:24]   <= in_byte;
                32'd10: pixel_offset[7:0]   <= in_byte;
                32'd11: pixel_offset[15:8]  <= in_byte;
                32'd12: pixel_offset[23:16] <= in_byte;
                32'd13: pixel_offset[31:24] <= in_byte;
                32'd18: width[7:0]   <= in_byte;
                32'd19: width[15:8]  <= in_byte;
                32'd20: width[23:16] <= in_byte;
                32'd21: width[31:24] <= in_byte;
                32'd22: height[7:0]   <= in_byte;
                32'd23: height[15:8]  <= in_byte;
                32'd24: height[23:16] <= in_byte;
                32'd25: height[31:24] <= in_byte;
                32'd28: bit_count[7:0]  <= in_byte;
                32'd29: bit_count[15:8] <= in_byte;
                32'd30: compression[7:0]   <= in_byte;
                32'd31: compression[15:8]  <= in_byte;
                32'd32: compression[23:16] <= in_byte;
                32'd33: compression[31:24] <= in_byte;
                default: ;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Registered geometry validation + row pitch (bmp_read.v 512-532). These
    // are quasi-static for the whole file; they settle by byte ~22-35, long
    // before the first pixel byte at pixel_offset (>=54).
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            width_ok_r <= 1'b0;
            height_ok_r<= 1'b0;
            src_w3     <= 16'd0;
            src_stride <= 16'd0;
            src_width  <= 16'd0;
            src_height <= 16'd0;
        end else if (start) begin
            width_ok_r <= 1'b0;
            height_ok_r<= 1'b0;
            src_w3     <= 16'd0;
            src_stride <= 16'd0;
            src_width  <= 16'd0;
            src_height <= 16'd0;
        end else begin
            width_ok_r  <= (width[31:16]  == 16'd0) &&
                           (width[15:0]   >= SRC_W_MIN) &&
                           (width[15:0]   <= SRC_W_MAX);
            height_ok_r <= (height[31:16] == 16'd0) &&
                           (height[15:0]  >= SRC_H_MIN) &&
                           (height[15:0]  <= SRC_H_MAX);
            src_w3      <= w3_calc[15:0];
            src_stride  <= stride_calc[15:0];
            src_width   <= width[15:0];
            src_height  <= height[15:0];
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst)        header_match_r <= 1'b0;
        else if (start) header_match_r <= 1'b0;
        else            header_match_r <= header_match_c;
    end

    // ------------------------------------------------------------------
    // hdr_ok latch + single src_dim_valid pulse, raised as soon as the header
    // validates (header_match_r rises), well before the first pixel byte.
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            hdr_ok        <= 1'b0;
            src_dim_valid <= 1'b0;
            dim_pulsed    <= 1'b0;
        end else if (start) begin
            hdr_ok        <= 1'b0;
            src_dim_valid <= 1'b0;
            dim_pulsed    <= 1'b0;
        end else begin
            src_dim_valid <= 1'b0;
            if (header_match_r && header_settled && !dim_pulsed) begin
                hdr_ok        <= 1'b1;
                src_dim_valid <= 1'b1;
                dim_pulsed    <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Row byte counter for padding drop (bmp_read.v 494-507). Advances on every
    // pixel-region byte (padding included) and wraps at src_stride.
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst)
            row_byte_cnt <= 16'd0;
        else if (start)
            row_byte_cnt <= 16'd0;
        else if (in_valid && hdr_ok && in_pixel_region) begin
            if ((row_byte_cnt + 16'd1) >= src_stride)
                row_byte_cnt <= 16'd0;
            else
                row_byte_cnt <= row_byte_cnt + 16'd1;
        end
    end

    // ------------------------------------------------------------------
    // 3-byte -> RGB888 packing (bmp_read.v 542-579). bmp_byte_idx walks 0,1,2
    // over each real pixel triple; the third byte emits src_valid.
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst)
            bmp_byte_idx <= 2'd0;
        else if (start)
            bmp_byte_idx <= 2'd0;
        else if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            src_valid <= 1'b0;
            src_pixel <= 24'd0;
        end else begin
            src_valid <= 1'b0;
            if (bmp_data_valid) begin
                case (bmp_byte_idx)
                    2'd0: begin
                        src_valid   <= 1'b0;
                        src_pixel[7:0] <= in_byte;     // B
                    end
                    2'd1: begin
                        src_valid   <= 1'b0;
                        src_pixel[15:8] <= in_byte;    // G
                    end
                    2'd2: begin
                        src_valid    <= 1'b1;
                        src_pixel[23:16] <= in_byte;   // R
                    end
                    default: src_valid <= 1'b0;
                endcase
            end
        end
    end

endmodule
