// top_matrix_uart.v
// Top-level module for the 4×4 matrix multiplication accelerator.
// Target: Arty A7-100T  |  Clock: 100 MHz  |  UART: 115200 8N1
//
// Pin mapping (Arty A7-100T defaults):
//   clk    → E3  (100 MHz oscillator)
//   rst_n  → C2  (active-low push-button BTN0, or tie high)
//   uart_rx→ A9  (USB-UART RX)
//   uart_tx→ D10 (USB-UART TX)
//   led[7:0]→ H5,J5,T9,T10,G6... (Arty LD0–LD3 are RGB; LD4–LD7 are plain)
//
// LED assignment:
//   LED[0]   = busy
//   LED[1]   = done
//   LED[7:2] = C[0][0][5:0]   (6-bit preview of top-left result element)

module top_matrix_uart (
    input  wire       clk,        // 100 MHz
    input  wire       rst_n,      // active-low reset (BTN0)
    input  wire       uart_rx,
    output wire       uart_tx,
    output wire [7:0] led
);

    wire rst = ~rst_n;

    // ─── Baud generator (16× oversampled tick) ────────────────
    wire baud_tick;

    baud_gen #(
        .CLK_FREQ  (100_000_000),
        .BAUD_RATE (115_200),
        .OVERSAMPLE(16)
    ) u_baud (
        .clk  (clk),
        .rst  (rst),
        .tick (baud_tick)
    );

    // ─── UART RX ──────────────────────────────────────────────
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx u_rx (
        .clk       (clk),
        .rst       (rst),
        .baud_tick (baud_tick),
        .rx_serial (uart_rx),
        .rx_data   (rx_data),
        .rx_valid  (rx_valid)
    );

    // ─── UART TX ──────────────────────────────────────────────
    wire [7:0] tx_data;
    wire       tx_start;
    wire       tx_busy;

    uart_tx u_tx (
        .clk       (clk),
        .rst       (rst),
        .baud_tick (baud_tick),
        .tx_data   (tx_data),
        .tx_start  (tx_start),
        .tx_serial (uart_tx),
        .tx_busy   (tx_busy)
    );

    // ─── Matrix BRAM ──────────────────────────────────────────
    wire [3:0]       a_wr_addr, a_rd_addr;
    wire [7:0]       a_wr_data;
    wire             a_wr_en;
    wire signed [7:0] a_rd_data;

    wire [3:0]       b_wr_addr, b_rd_addr;
    wire [7:0]       b_wr_data;
    wire             b_wr_en;
    wire signed [7:0] b_rd_data;

    wire [3:0]       c_wr_addr, c_rd_addr;
    wire signed [31:0] c_wr_data;
    wire             c_wr_en;
    wire signed [31:0] c_rd_data;

    matrix_bram u_bram (
        .clk        (clk),
        // A
        .a_wr_addr  (a_wr_addr),
        .a_wr_data  (a_wr_data),
        .a_wr_en    (a_wr_en),
        .a_rd_addr  (a_rd_addr),
        .a_rd_data  (a_rd_data),
        // B
        .b_wr_addr  (b_wr_addr),
        .b_wr_data  (b_wr_data),
        .b_wr_en    (b_wr_en),
        .b_rd_addr  (b_rd_addr),
        .b_rd_data  (b_rd_data),
        // C
        .c_wr_addr  (c_wr_addr),
        .c_wr_data  (c_wr_data),
        .c_wr_en    (c_wr_en),
        .c_rd_addr  (c_rd_addr),
        .c_rd_data  (c_rd_data)
    );

    // ─── Systolic array ───────────────────────────────────────
    wire        sa_rst_acc, sa_en;
    wire signed [7:0] sa_a0, sa_a1, sa_a2, sa_a3;
    wire signed [7:0] sa_b0, sa_b1, sa_b2, sa_b3;

    wire signed [31:0] sa_c00, sa_c01, sa_c02, sa_c03;
    wire signed [31:0] sa_c10, sa_c11, sa_c12, sa_c13;
    wire signed [31:0] sa_c20, sa_c21, sa_c22, sa_c23;
    wire signed [31:0] sa_c30, sa_c31, sa_c32, sa_c33;

    systolic_array_4x4 u_sa (
        .clk      (clk),
        .rst      (rst),
        .rst_acc  (sa_rst_acc),
        .en       (sa_en),
        .a_in_0   (sa_a0), .a_in_1 (sa_a1), .a_in_2 (sa_a2), .a_in_3 (sa_a3),
        .b_in_0   (sa_b0), .b_in_1 (sa_b1), .b_in_2 (sa_b2), .b_in_3 (sa_b3),
        .c_out_00 (sa_c00), .c_out_01 (sa_c01), .c_out_02 (sa_c02), .c_out_03 (sa_c03),
        .c_out_10 (sa_c10), .c_out_11 (sa_c11), .c_out_12 (sa_c12), .c_out_13 (sa_c13),
        .c_out_20 (sa_c20), .c_out_21 (sa_c21), .c_out_22 (sa_c22), .c_out_23 (sa_c23),
        .c_out_30 (sa_c30), .c_out_31 (sa_c31), .c_out_32 (sa_c32), .c_out_33 (sa_c33)
    );

    // ─── Controller ───────────────────────────────────────────
    wire busy, done;

    controller u_ctrl (
        .clk        (clk),
        .rst        (rst),
        // UART
        .rx_data    (rx_data),
        .rx_valid   (rx_valid),
        .tx_data    (tx_data),
        .tx_start   (tx_start),
        .tx_busy    (tx_busy),
        // BRAM A
        .a_wr_addr  (a_wr_addr), .a_wr_data (a_wr_data), .a_wr_en (a_wr_en),
        .a_rd_addr  (a_rd_addr), .a_rd_data (a_rd_data),
        // BRAM B
        .b_wr_addr  (b_wr_addr), .b_wr_data (b_wr_data), .b_wr_en (b_wr_en),
        .b_rd_addr  (b_rd_addr), .b_rd_data (b_rd_data),
        // BRAM C
        .c_wr_addr  (c_wr_addr), .c_wr_data (c_wr_data), .c_wr_en (c_wr_en),
        .c_rd_addr  (c_rd_addr), .c_rd_data (c_rd_data),
        // Systolic
        .sa_rst_acc (sa_rst_acc), .sa_en (sa_en),
        .sa_a0 (sa_a0), .sa_a1 (sa_a1), .sa_a2 (sa_a2), .sa_a3 (sa_a3),
        .sa_b0 (sa_b0), .sa_b1 (sa_b1), .sa_b2 (sa_b2), .sa_b3 (sa_b3),
        .sa_c00 (sa_c00), .sa_c01 (sa_c01), .sa_c02 (sa_c02), .sa_c03 (sa_c03),
        .sa_c10 (sa_c10), .sa_c11 (sa_c11), .sa_c12 (sa_c12), .sa_c13 (sa_c13),
        .sa_c20 (sa_c20), .sa_c21 (sa_c21), .sa_c22 (sa_c22), .sa_c23 (sa_c23),
        .sa_c30 (sa_c30), .sa_c31 (sa_c31), .sa_c32 (sa_c32), .sa_c33 (sa_c33),
        // Status
        .busy       (busy),
        .done       (done)
    );

    // ─── LED output ───────────────────────────────────────────
    // LED[0] = busy, LED[1] = done, LED[7:2] = C[0][0][5:0]
    assign led[0]   = busy;
    assign led[1]   = done;
    assign led[7:2] = sa_c00[5:0];   // live 6-bit preview of top-left result

endmodule
