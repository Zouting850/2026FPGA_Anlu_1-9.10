`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// board_b_top.v  (Board B -- Ethernet image front-end, SPI master to Board A)
//
// WHAT THIS IS
//   The new top level for the spare HX4S20 board acting as the network
//   front-end of the two-board image system. It receives a BMP file from the PC
//   as an in-order UDP byte stream, decodes it, nearest-neighbour scales it to
//   640x480 RGB888, buffers the frame in the on-chip SDRAM, reads it back and
//   pushes it to Board A over a 3-wire SPI master link (SCK/MOSI/CS_n + GND on
//   the J1/J2 headers). Board A is the SPI slave and owns the HDMI display.
//
// PROVENANCE / INTEGRITY
//   The Ethernet + clock infrastructure below (pll_50, clk_gen_rst_gen, rx_pll,
//   temac_block, tx/rx_client_fifo, udp_ip_protocol_stack, udp_clk_gen, the
//   reset/phy_reset sequencing and the dynamic-IP logic that settles to
//   192.168.240.1 port 4) is ported VERBATIM from the vendor UDP_Example_Top.v
//   so the board-proven network stack is bit-identical. The vendor SD-card image
//   path (sd_ctrl_top / sd_read_photo), udp_loopback and udp_data_tpg are
//   PRUNED -- Board B has no SD card and does not echo packets. The pixel
//   datapath that replaces them is the user's own verified glue:
//       app_rx (udp_clk) -> rx_byte_cdc -> rx_frame_to_bmp -> bmp_decode
//       -> scaler_nn (all clk_50m) -> sdram_top write side
//       sdram_top readback (sdr_clk) -> sdram_to_spi -> SPI pins
//
// CLOCK / PLL BUDGET (the hard wall)
//   Board B is at 4/4 PLL with the bare Ethernet stack: pll_50 (clk_50m), the
//   clk_pll inside sdram_top (sdr_clk=125M), rx_pll, and clk_gen_rst_gen's
//   internal PLL. There is ZERO PLL headroom, which is exactly why the SPI link
//   was chosen -- spi_master_tx derives SCK from a clk_50m divider and needs no
//   PLL of its own. Every new module here reuses an existing clock:
//     clk_50m (50M)  -- the whole pixel pipeline (decode/scale/frame/bridge)
//     udp_clk (125M) -- the UDP RX byte source
//     sdr_clk (125M) -- SDRAM readback into the bridge FIFO write side
//
// DATA_WIDTH NOTE
//   sdram_top.Sdr_rd_dout is `DATA_WIDTH = 32 bits (global_def.v); the pixel
//   lives in the low 24 bits, zero-extended. The old top silently truncated it
//   onto a 24-bit wire. Here the slice Sdr_rd_dout[23:0] is EXPLICIT.
//
// RESET POLARITY
//   The vendor Ethernet tree is active-low (sys_rst_n -> rst_n). The user pixel
//   pipeline (bmp_decode / rx_frame_to_bmp / scaler_nn / rx_byte_cdc) is
//   active-high. This top owns the single inversion: pix_rst = ~rst_n.
//
// RX-ONLY
//   Board B never transmits application data (no UDP ACK back-channel); the PC
//   paces images with fixed waits per the approved Option-A contract. The
//   udp_ip_protocol_stack TX app interface is tied off idle.
// ---------------------------------------------------------------------------
module board_b_top(
    input               clk_50,             // R7, 50 MHz board oscillator
    input               sys_rst_n,          // KEY, active low

    // ---- gigabit Ethernet PHY (RGMII) ----
    input               phy1_rgmii_rx_clk,
    input               phy1_rgmii_rx_ctl,
    input  [3:0]        phy1_rgmii_rx_data,
    output wire         phy1_rgmii_tx_clk,
    output wire         phy1_rgmii_tx_ctl,
    output wire [3:0]   phy1_rgmii_tx_data,

    // ---- SPI master to Board A (J1/J2 flying leads) ----
    output wire         spi_sck,
    output wire         spi_mosi,
    output wire         spi_cs_n,

    output [2:0]        led
);

parameter  DEVICE             = "EG4";      //"PH1","EG4"
parameter  LOCAL_UDP_PORT_NUM = 16'h0001;
parameter  LOCAL_IP_ADDRESS   = 32'hc0a8f001;   // 192.168.240.1
parameter  LOCAL_MAC_ADDRESS  = 48'h0123456789ab;
parameter  DST_UDP_PORT_NUM   = 16'h0002;
parameter  DST_IP_ADDRESS     = 32'hc0a8f002;

// pixel-pipeline geometry / SPI contract
parameter  integer PIXELS         = 307200;     // 640*480
parameter  integer SCK_DIV         = 8;          // 8 -> 6.25 MHz SCK, ~1.18 s/frame
parameter  [31:0]  SPI_MAGIC       = 32'hA55A_5AA5;
parameter  integer FRAME_RST_WIDTH = 16;         // clk_50m cycles app_wrrd is held reset

//==========================================================================
// Ethernet / clock infrastructure -- ported VERBATIM from UDP_Example_Top.v
//==========================================================================
wire         key1 = sys_rst_n;

wire         app_rx_data_valid;//synthesis keep
wire [7:0]   app_rx_data;//synthesis keep
wire [15:0]  app_rx_data_length;//synthesis keep
wire [15:0]  app_rx_port_num;

wire         udp_tx_ready;//synthesis keep
wire         app_tx_ack;//synthesis keep
wire         app_tx_data_request;//synthesis keep
wire         app_tx_data_valid;//synthesis keep
wire [7:0]   app_tx_data;//synthesis keep
wire [15:0]  udp_data_length;

//temac signals
wire         tx_stop;
wire [7:0]   tx_ifg_val;
wire         pause_req;
wire [15:0]  pause_val;
wire [47:0]  pause_source_addr;
wire [47:0]  unicast_address;
wire [19:0]  mac_cfg_vector;

wire         temac_tx_ready;//synthesis keep
wire         temac_tx_valid;//synthesis keep
wire [7:0]   temac_tx_data;//synthesis keep
wire         temac_tx_sof;
wire         temac_tx_eof;

wire         temac_rx_ready;
wire         temac_rx_valid;//synthesis keep
wire [7:0]   temac_rx_data;//synthesis keep
wire         temac_rx_sof;
wire         temac_rx_eof;

wire         rx_correct_frame;
wire         rx_error_frame;
wire [1:0]   TRI_speed;
assign TRI_speed = 2'b10;   //千兆2'b10 百兆2'b01 十兆2'b00

wire         rx_clk_int;
wire         rx_clk_en_int;
wire         tx_clk_int;
wire         tx_clk_en_int;

wire         temac_clk;//synthesis keep
wire         udp_clk;  //synthesis keep
wire         temac_clk90;//synthesis keep
wire         clk_125_out;
wire         clk_12_5_out;
wire         clk_1_25_out;
wire         rx_valid;//synthesis keep
wire [7:0]   rx_data;//synthesis keep
wire [7:0]   tx_data; //synthesis keep
wire         tx_valid; //synthesis keep
wire         tx_rdy;
wire         tx_collision;
wire         tx_retransmit;

wire         reset, reset_reg;
wire         clk_50_out;
wire         phy_reset;
reg  [7:0]   phy_reset_cnt = 'd0;
reg  [7:0]   soft_reset_cnt = 8'hff;
reg          sys_rst_n_1, sys_rst_n_2;
wire         key2;
assign key2 = sys_rst_n_2;
wire         locked;
wire         clk_50m, clk_50m_180deg /* synthesis syn_keep=1 */;
wire         clk_sample;//synthesis keep = 1

assign reset = ~key1 || reset_reg || (soft_reset_cnt != 'd0);

pll_50 u_pll_50(
  .refclk   (clk_50),
  .reset    (!sys_rst_n_2),
  .extlock  (locked),
  .clk0_out (clk_50m),
  .clk1_out (clk_50m_180deg),
  .clk2_out (clk_sample)
);

always @(posedge clk_50 or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        sys_rst_n_1 <= 1'b0;
        sys_rst_n_2 <= 1'b0;
    end else begin
        sys_rst_n_1 <= sys_rst_n;
        sys_rst_n_2 <= sys_rst_n_1;
    end
end

wire rst_n /* synthesis syn_keep=1 */;
assign rst_n = sys_rst_n_2;

// active-high reset for the user pixel pipeline (bmp_decode / rx_frame_to_bmp /
// scaler_nn / rx_byte_cdc all use posedge rst). Single inversion owned here.
wire pix_rst = ~rst_n;

always @(posedge clk_50_out or negedge key1) begin
    if(~key1)
        phy_reset_cnt <= 'd0;
    else if(phy_reset_cnt < 255)
        phy_reset_cnt <= phy_reset_cnt + 1;
    else
        phy_reset_cnt <= phy_reset_cnt;
end
assign phy_reset = phy_reset_cnt[7];

always @(posedge udp_clk or negedge key1) begin
    if(~key1)
        soft_reset_cnt <= 8'hff;
    else if(soft_reset_cnt > 0)
        soft_reset_cnt <= soft_reset_cnt - 1;
    else
        soft_reset_cnt <= soft_reset_cnt;
end

//-----------------------------------------------------
// MAC client configuration (verbatim)
//-----------------------------------------------------
assign tx_stop            = 1'b0;
assign tx_ifg_val         = 8'h00;
assign pause_req          = 1'b0;
assign pause_val          = 16'h0;
assign pause_source_addr  = 48'h5af1f2f3f4f5;
assign unicast_address    = {  LOCAL_MAC_ADDRESS[7:0],
                               LOCAL_MAC_ADDRESS[15:8],
                               LOCAL_MAC_ADDRESS[23:16],
                               LOCAL_MAC_ADDRESS[31:24],
                               LOCAL_MAC_ADDRESS[39:32],
                               LOCAL_MAC_ADDRESS[47:40] };
assign mac_cfg_vector     = {1'b0,2'b00,TRI_speed,8'b00000010,7'b0000010};

//-----------------------------------------------------
// dynamic_local_ip_address (verbatim): cnt1 free-runs to 0 so the local IP
// settles to LOCAL_IP_ADDRESS (192.168.240.1) and the local UDP port settles
// to input_local_ip_address[3:0]+3 = 4. The PC sends images to .1:4.
//-----------------------------------------------------
reg  [32:0] cnt0;
wire        end_cnt0;
wire        add_cnt0;
reg  [7:0]  cnt1;
wire        end_cnt1;
wire        add_cnt1;

always @(posedge udp_clk or negedge sys_rst_n_2) begin
    if(!sys_rst_n_2)     cnt0 <= 0;
    else if(add_cnt0) begin
        if(end_cnt0)     cnt0 <= 0;
        else             cnt0 <= cnt0 + 1;
    end
end
assign add_cnt0 = 1;
assign end_cnt0 = add_cnt0 && 0;

always @(posedge udp_clk or negedge sys_rst_n_2) begin
    if(!sys_rst_n_2)     cnt1 <= 0;
    else if(add_cnt1) begin
        if(end_cnt1)     cnt1 <= 0;
        else             cnt1 <= cnt1 + 1;
    end
end
assign add_cnt1 = end_cnt0;
assign end_cnt1 = add_cnt1 && cnt1 == 15;

reg [31:0] input_local_ip_address;
reg        input_local_ip_address_valid;
always @(posedge udp_clk or posedge reset) begin
    if(reset) begin
        input_local_ip_address       <= LOCAL_IP_ADDRESS;
        input_local_ip_address_valid <= 1'b0;
    end else if(end_cnt0 == 1'b1) begin
        input_local_ip_address       <= {LOCAL_IP_ADDRESS[31:8],cnt1};
        input_local_ip_address_valid <= 1'b1;
    end else begin
        input_local_ip_address       <= input_local_ip_address;
        input_local_ip_address_valid <= 1'b1;
    end
end

reg [15:0] input_local_udp_port_num;
reg        input_local_udp_port_num_valid;
always @(posedge udp_clk or posedge reset) begin
    if(reset) begin
        input_local_udp_port_num       <= LOCAL_UDP_PORT_NUM;
        input_local_udp_port_num_valid <= 1'b0;
    end else begin
        input_local_udp_port_num       <= input_local_ip_address[3:0] + 3;
        input_local_udp_port_num_valid <= 1'b1;
    end
end

//==========================================================================
// Clock / reset generators (verbatim)
//==========================================================================
clk_gen_rst_gen#(
    .DEVICE (DEVICE)
)u_clk_gen(
    .reset        (~key1),
    .clk_in       (clk_50),
    .rst_out      (reset_reg),
    .clk_125_out0 (temac_clk),
    .clk_125_out1 (clk_125_out),
    .clk_125_out2 (temac_clk90),
    .clk_12_5_out (clk_12_5_out),
    .clk_1_25_out (clk_1_25_out),
    .clk_25_out   (clk_50_out)
);

//==========================================================================
// NEW PIXEL DATAPATH
//==========================================================================

// ---- 1) UDP RX byte stream CDC: udp_clk(125M) -> clk_50m(50M) ----
wire        rxb_valid;
wire [7:0]  rxb_byte;
wire        rxb_overflow;
rx_byte_cdc #(
    .ADDR_WIDTH (12)
) u_rx_byte_cdc (
    .wr_clk    (udp_clk),
    .rst       (reset),                 // active-high async; matches vendor reset net
    .in_valid  (app_rx_data_valid),
    .in_byte   (app_rx_data),
    .rd_clk    (clk_50m),
    .out_valid (rxb_valid),
    .out_byte  (rxb_byte),
    .overflow  (rxb_overflow)
);

// ---- 2) transport reframing: hunt SOF magic, capture length, pulse start ----
wire        fr_start;
wire        fr_out_valid;
wire [7:0]  fr_out_byte;
wire        fr_idle_err;
rx_frame_to_bmp #(
    .SOF_MAGIC (SPI_MAGIC),             // same 0xA55A5AA5 delimiter as the SPI frame
    .IDLE_AW   (25)
) u_rx_frame (
    .clk              (clk_50m),
    .rst              (pix_rst),
    .in_valid         (rxb_valid),
    .in_byte          (rxb_byte),
    .start            (fr_start),
    .out_valid        (fr_out_valid),
    .out_byte         (fr_out_byte),
    .idle_timeout_err (fr_idle_err)
);

// ---- 3) BMP decode: file byte stream -> RGB888 source pixels ----
wire        bmp_src_valid;
wire [23:0] bmp_src_pixel;
wire [15:0] bmp_src_width;
wire [15:0] bmp_src_height;
wire        bmp_dim_valid;
wire        bmp_hdr_ok;
wire        bmp_busy;
bmp_decode u_bmp_decode (
    .clk           (clk_50m),
    .rst           (pix_rst),
    .start         (fr_start),
    .in_valid      (fr_out_valid),
    .in_byte       (fr_out_byte),
    .src_valid     (bmp_src_valid),
    .src_pixel     (bmp_src_pixel),
    .src_width     (bmp_src_width),
    .src_height    (bmp_src_height),
    .src_dim_valid (bmp_dim_valid),
    .hdr_ok        (bmp_hdr_ok),
    .busy          (bmp_busy)
);

// ---- 4) SDRAM frame buffer + readback (vendor tree, additive frame_rst) ----
wire         sdr_clk;
wire         Sdr_init_done;
wire         Sdr_rd_en;
wire [31:0]  Sdr_rd_dout_full;     // DATA_WIDTH=32; pixel is the low 24 bits
wire [11:0]  udp_wrusedw;          // bridge FIFO write occupancy -> app_wrrd read gate
wire [11:0]  wr_fifo_usedw;        // app_wrrd write FIFO occupancy -> scaler backpressure

// frame_rst stretcher: rx_frame_to_bmp.start (1 clk_50m pulse) widened to
// FRAME_RST_WIDTH cycles so the sdr_clk-domain 2-FF synchroniser inside
// sdram_top reliably catches it and app_wrrd is held reset long enough to
// fully re-arm. Timing is measured, not estimated, by
// tools/sim_board_b_integration.py: fr_start at cycle 8, frame_rst deasserting
// at cycle 24 (== start + FRAME_RST_WIDTH), bmp_decode's first src_valid at
// cycle 65 -> 41 cycles of margin. The latency is the 54-byte BMP pixel offset
// plus two more bytes to complete the first BGR triple (byte_cnt 56), NOT a
// 35-byte header. 41 is also the conservative figure: it counts the scaler's
// first INPUT pixel, whereas app_wrrd only has to be ready for the scaler's
// first SDRAM WRITE, which is later still by the scaler's park-and-fill delay.
// Raising FRAME_RST_WIDTH past ~50 would eat that margin and drop the first
// pixels of every frame; the same model rejects WIDTH=100 as a negative control.
reg [7:0]  frst_cnt;
wire       frame_rst = (frst_cnt != 8'd0);
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n)
        frst_cnt <= 8'd0;
    else if(fr_start)
        frst_cnt <= FRAME_RST_WIDTH[7:0];
    else if(frst_cnt != 8'd0)
        frst_cnt <= frst_cnt - 8'd1;
end

// scaler output -> SDRAM write side. Backpressure: saturate the 12-bit write
// FIFO occupancy to the scaler's 9-bit i_fifo_usedw port so a full FIFO reads as
// 511 (> STALL_THRESH=384) instead of wrapping to 0 and defeating the stall.
wire        scaler_dst_valid;
wire [23:0] scaler_dst_pixel;
wire        scaler_busy;
wire        scaler_done;
wire        scaler_overflow;
wire [8:0]  scaler_fifo_usedw = (wr_fifo_usedw >= 12'd511) ? 9'd511 : wr_fifo_usedw[8:0];

scaler_nn #(
    .DST_W        (640),
    .DST_H        (480),
    .MAX_UPSCALE  (4),
    .STALL_THRESH (384)
) u_scaler (
    .clk           (clk_50m),
    .rst           (pix_rst),
    .i_src_valid   (bmp_src_valid),
    .i_src_pixel   (bmp_src_pixel),
    .i_dim_valid   (bmp_dim_valid),
    .i_src_w       (bmp_src_width),
    .i_src_h       (bmp_src_height),
    .i_fifo_usedw  (scaler_fifo_usedw),
    .o_dst_valid   (scaler_dst_valid),
    .o_dst_pixel   (scaler_dst_pixel),
    .o_busy        (scaler_busy),
    .o_done        (scaler_done),
    .o_overflow    (scaler_overflow)
);

sdram_top u_sdram (
    .SYS_CLK        (clk_50m),
    .rst_n          (rst_n),
    .sd_clk         (1'b0),                 // unused: app_wrrd.sd_clk is tied to SYS_CLK inside
    .sdr_clk        (sdr_clk),
    .LED            (),
    .Sdr_init_done  (Sdr_init_done),
    .wr_done        (),
    .sdr_data_valid (scaler_dst_valid),
    .sdr_data       (scaler_dst_pixel),
    .Sdr_rd_en      (Sdr_rd_en),
    .Sdr_rd_dout    (Sdr_rd_dout_full),
    .full_flag      (1'b0),                 // unused by app_wrrd (no loopback TX path)
    .full_flag_sdr  (),
    .udp_wrusedw    (udp_wrusedw),
    .frame_rst      (frame_rst),
    .wr_fifo_usedw  (wr_fifo_usedw)
);

// ---- 5) SDRAM readback -> SPI master push to Board A ----
wire spi_frame_done;
sdram_to_spi #(
    .CLK_FREQ_HZ (50_000_000),
    .SCK_DIV     (SCK_DIV),
    .PIXELS      (PIXELS),
    .MAGIC       (SPI_MAGIC)
) u_bridge (
    .sdr_clk      (sdr_clk),
    .Sdr_rd_en    (Sdr_rd_en),
    .Sdr_rd_dout  (Sdr_rd_dout_full[23:0]),  // EXPLICIT slice; Sdr_rd_dout is 32-bit
    .udp_wrusedw  (udp_wrusedw),
    .clk          (clk_50m),
    .rst_n        (rst_n),
    .frame_rst    (frame_rst),
    .spi_sck      (spi_sck),
    .spi_mosi     (spi_mosi),
    .spi_cs_n     (spi_cs_n),
    .frame_done   (spi_frame_done)
);

//==========================================================================
// UDP / IP protocol stack + TEMAC + FIFOs -- ported VERBATIM
//==========================================================================
udp_ip_protocol_stack #(
    .DEVICE             (DEVICE),
    .LOCAL_UDP_PORT_NUM (LOCAL_UDP_PORT_NUM),
    .LOCAL_IP_ADDRESS   (LOCAL_IP_ADDRESS),
    .LOCAL_MAC_ADDRESS  (LOCAL_MAC_ADDRESS)
) u3_udp_ip_protocol_stack (
    .udp_rx_clk                     (udp_clk),
    .udp_tx_clk                     (udp_clk),
    .reset                          (reset),
    .udp2app_tx_ready               (udp_tx_ready),
    .udp2app_tx_ack                 (app_tx_ack),
    .app_tx_request                 (app_tx_data_request),
    .app_tx_data_valid              (app_tx_data_valid),
    .app_tx_data                    (app_tx_data),
    .app_tx_data_length             (udp_data_length),
    .app_tx_dst_port                (DST_UDP_PORT_NUM),
    .ip_tx_dst_address              (DST_IP_ADDRESS),

    .input_local_udp_port_num       (input_local_udp_port_num),
    .input_local_udp_port_num_valid (input_local_udp_port_num_valid),
    .input_local_ip_address         (input_local_ip_address),
    .input_local_ip_address_valid   (input_local_ip_address_valid),

    .app_rx_data_valid              (app_rx_data_valid),
    .app_rx_data                    (app_rx_data),
    .app_rx_data_length             (app_rx_data_length),
    .app_rx_port_num                (app_rx_port_num),
    .temac_rx_ready                 (temac_rx_ready),
    .temac_rx_valid                 (!temac_rx_valid),
    .temac_rx_data                  (temac_rx_data),
    .temac_rx_sof                   (temac_rx_sof),
    .temac_rx_eof                   (temac_rx_eof),
    .temac_tx_ready                 (temac_tx_ready),
    .temac_tx_valid                 (temac_tx_valid),
    .temac_tx_data                  (temac_tx_data),
    .temac_tx_sof                   (temac_tx_sof),
    .temac_tx_eof                   (temac_tx_eof),
    .ip_rx_error                    (),
    .arp_request_no_reply_error     ()
);

// RX-only: tie the application TX interface off idle. The stack still needs a
// well-formed, never-requesting TX client so its internal handshake stays put.
assign app_tx_data_request = 1'b0;
assign app_tx_data_valid   = 1'b0;
assign app_tx_data         = 8'h00;
assign udp_data_length     = 16'h0000;

wire phy1_rgmii_rx_clk_0;
wire phy1_rgmii_rx_clk_90;
rx_pll u_rx_pll(
    .refclk   (phy1_rgmii_rx_clk),
    .reset    (1'b0),
    .clk0_out (phy1_rgmii_rx_clk_0),
    .clk1_out (phy1_rgmii_rx_clk_90)
);

temac_block#(
    .DEVICE (DEVICE)
) u4_trimac_block (
    .reset              (reset),
    .gtx_clk            (clk_125_out),
    .gtx_clk_90         (temac_clk90),
    .rx_clk             (rx_clk_int),
    .rx_clk_en          (rx_clk_en_int),
    .rx_data            (rx_data),
    .rx_data_valid      (rx_valid),
    .rx_correct_frame   (rx_correct_frame),
    .rx_error_frame     (rx_error_frame),
    .rx_status_vector   (),
    .rx_status_vld      (),
    .tx_clk             (tx_clk_int),
    .tx_clk_en          (tx_clk_en_int),
    .tx_data            (tx_data),
    .tx_data_en         (tx_valid),
    .tx_rdy             (tx_rdy),
    .tx_stop            (tx_stop),
    .tx_collision       (tx_collision),
    .tx_retransmit      (tx_retransmit),
    .tx_ifg_val         (tx_ifg_val),
    .tx_status_vector   (),
    .tx_status_vld      (),
    .pause_req          (pause_req),
    .pause_val          (pause_val),
    .pause_source_addr  (pause_source_addr),
    .unicast_address    (unicast_address),
    .mac_cfg_vector     (mac_cfg_vector),
    .rgmii_txd          (phy1_rgmii_tx_data),
    .rgmii_tx_ctl       (phy1_rgmii_tx_ctl),
    .rgmii_txc          (phy1_rgmii_tx_clk),
    .rgmii_rxd          (phy1_rgmii_rx_data),
    .rgmii_rx_ctl       (phy1_rgmii_rx_ctl),
    .rgmii_rxc          (phy1_rgmii_rx_clk_90),
    .inband_link_status (),
    .inband_clock_speed (),
    .inband_duplex_status ()
);

udp_clk_gen#(
    .DEVICE (DEVICE)
) u5_temac_clk_gen (
    .reset       (~key1),
    .tri_speed   (TRI_speed),
    .clk_125_in  (clk_125_out),
    .clk_12_5_in (clk_12_5_out),
    .clk_1_25_in (clk_1_25_out),
    .udp_clk_out (udp_clk)
);

tx_client_fifo #(
    .DEVICE (DEVICE)
) u6_tx_fifo (
    .rd_clk         (tx_clk_int),
    .rd_sreset      (reset),
    .rd_enable      (tx_clk_en_int),
    .tx_data        (tx_data),
    .tx_data_valid  (tx_valid),
    .tx_ack         (tx_rdy),
    .tx_collision   (tx_collision),
    .tx_retransmit  (tx_retransmit),
    .overflow       (),
    .wr_clk         (udp_clk),
    .wr_sreset      (reset),
    .wr_data        (temac_tx_data),
    .wr_sof_n       (temac_tx_sof),
    .wr_eof_n       (temac_tx_eof),
    .wr_src_rdy_n   (temac_tx_valid),
    .wr_dst_rdy_n   (temac_tx_ready),
    .wr_fifo_status ()
);

rx_client_fifo#(
    .DEVICE (DEVICE)
) u7_rx_fifo (
    .wr_clk         (rx_clk_int),
    .wr_enable      (rx_clk_en_int),
    .wr_sreset      (reset),
    .rx_data        (rx_data),
    .rx_data_valid  (rx_valid),
    .rx_good_frame  (rx_correct_frame),
    .rx_bad_frame   (rx_error_frame),
    .overflow       (),
    .rd_clk         (udp_clk),
    .rd_sreset      (reset),
    .rd_data_out    (temac_rx_data),
    .rd_sof_n       (temac_rx_sof),
    .rd_eof_n       (temac_rx_eof),
    .rd_src_rdy_n   (temac_rx_valid),
    .rd_dst_rdy_n   (temac_rx_ready),
    .rx_fifo_status ()
);

//==========================================================================
// Status LEDs (bring-up visibility)
//   led[0] : SDRAM initialised and ready
//   led[1] : a BMP header has validated at least once this power cycle (sticky)
//   led[2] : fault -- RX FIFO overflowed, OR the framer watchdog dropped a
//            truncated frame, OR the scaler elastic buffer overflowed (sticky)
//==========================================================================
reg hdr_ok_sticky;
reg fault_sticky;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        hdr_ok_sticky <= 1'b0;
        fault_sticky  <= 1'b0;
    end else begin
        if(bmp_hdr_ok)                                        hdr_ok_sticky <= 1'b1;
        if(rxb_overflow || fr_idle_err || scaler_overflow)   fault_sticky  <= 1'b1;
    end
end

assign led[0] = Sdr_init_done;
assign led[1] = hdr_ok_sticky;
assign led[2] = fault_sticky;

endmodule
