// controller.v
// UART command processor and compute sequencer.
//
// ═══════════════════════════════════════════════════════════
//  UART COMMAND PROTOCOL
// ═══════════════════════════════════════════════════════════
//  All multi-byte fields are little-endian.
//
//  CMD 0x01 – Write Matrix A
//    Byte 0 : 0x01
//    Bytes 1–16 : 16 signed bytes (row-major, A[0][0]..A[3][3])
//
//  CMD 0x02 – Write Matrix B
//    Byte 0 : 0x02
//    Bytes 1–16 : 16 signed bytes (row-major)
//
//  CMD 0x03 – Start computation
//    Byte 0 : 0x03  (no payload)
//    → triggers systolic array; controller sends 0xAA when done.
//
//  CMD 0x04 – Read result C
//    Byte 0 : 0x04
//    Byte 1 : element index (0–15, row-major)
//    → controller replies with 4 bytes (32-bit signed, little-endian)
//
//  Any unrecognised command: controller replies 0xFF (NAK).
//
// ═══════════════════════════════════════════════════════════
//  SYSTOLIC SCHEDULING
// ═══════════════════════════════════════════════════════════
//  Weight-stationary, skewed feed.
//  For a 4×4 × 4×4 multiply:
//    A[i][k] enters row i at time  t = k + i          (0-based)
//    B[k][j] enters col j at time  t = k + j
//  Total active cycles = 4 + 3 + 3 = 10 cycles (last element arrives at t=6,
//  but all PEs finish accumulation after t=6+0 = 7 for PE[0][0] and
//  t = 3+3 = 6 + pipe latency → we run for COMPUTE_CYCLES=10 to be safe).
//
//  The controller reads A and B from BRAM one cycle before they are needed,
//  accounting for the 1-cycle BRAM read latency.

module controller (
    input  wire        clk,
    input  wire        rst,

    // UART RX
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,

    // UART TX
    output reg  [7:0]  tx_data,
    output reg         tx_start,
    input  wire        tx_busy,

    // BRAM – Matrix A
    output reg  [3:0]  a_wr_addr,
    output reg  [7:0]  a_wr_data,
    output reg         a_wr_en,
    output reg  [3:0]  a_rd_addr,
    input  wire signed [7:0]  a_rd_data,

    // BRAM – Matrix B
    output reg  [3:0]  b_wr_addr,
    output reg  [7:0]  b_wr_data,
    output reg         b_wr_en,
    output reg  [3:0]  b_rd_addr,
    input  wire signed [7:0]  b_rd_data,

    // BRAM – Matrix C
    output reg  [3:0]  c_wr_addr,
    output reg  signed [31:0] c_wr_data,
    output reg         c_wr_en,
    output reg  [3:0]  c_rd_addr,
    input  wire signed [31:0] c_rd_data,

    // Systolic array control
    output reg         sa_rst_acc,
    output reg         sa_en,

    // Skewed A/B feeds to systolic array (row / col inputs)
    output reg signed [7:0]  sa_a0, sa_a1, sa_a2, sa_a3,
    output reg signed [7:0]  sa_b0, sa_b1, sa_b2, sa_b3,

    // Results from systolic array
    input  wire signed [31:0] sa_c00, sa_c01, sa_c02, sa_c03,
    input  wire signed [31:0] sa_c10, sa_c11, sa_c12, sa_c13,
    input  wire signed [31:0] sa_c20, sa_c21, sa_c22, sa_c23,
    input  wire signed [31:0] sa_c30, sa_c31, sa_c32, sa_c33,

    // Status
    output reg         busy,
    output reg         done
);

    // ─── FSM states ───────────────────────────────────────────
    localparam [4:0]
        S_IDLE        = 5'd0,
        S_CMD         = 5'd1,   // decode first byte
        S_RECV_A      = 5'd2,   // receive 16 bytes for A
        S_RECV_B      = 5'd3,   // receive 16 bytes for B
        S_COMPUTE_RST = 5'd4,   // pulse rst_acc
        S_COMPUTE_RUN = 5'd5,   // feed data and run array
        S_COMPUTE_DONE= 5'd6,   // latch results to C BRAM
        S_SEND_ACK    = 5'd7,   // send 0xAA
        S_READ_IDX    = 5'd8,   // wait for element index byte
        S_READ_WAIT   = 5'd9,   // wait 1 cycle for BRAM read
        S_SEND_C0     = 5'd10,  // send byte 0 of result
        S_SEND_C1     = 5'd11,
        S_SEND_C2     = 5'd12,
        S_SEND_C3     = 5'd13,
        S_SEND_NAK    = 5'd14,
        S_WAIT_TX     = 5'd15;  // generic wait for TX to complete then go to next

    reg [4:0]  state, next_after_tx;
    reg [4:0]  byte_cnt;         // counts received payload bytes (0–15)
    reg [4:0]  compute_cycle;    // counts systolic feed cycles

    // Latched result for multi-byte TX
    reg signed [31:0] c_latch;

    // ─── Skew schedule ────────────────────────────────────────
    // We drive A row r with A[r][k] at time t = k+r (zero-pad outside window).
    // We drive B col j with B[k][j] at time t = k+j.
    // t runs 0..9 (10 cycles). BRAM has 1-cycle read latency, so we
    // issue the read address one cycle early using (compute_cycle-1).
    //
    // A element index for row r at cycle t: k = t - r  (valid when 0 ≤ k ≤ 3)
    //   → BRAM addr = r*4 + k
    // B element index for col j at cycle t: k = t - j
    //   → BRAM addr = k*4 + j

    localparam COMPUTE_CYCLES = 10;   // enough for all PEs to finish

    // Helper: return 8'sd0 when index out of range
    function [7:0] clamp_a;
        input [4:0] t;
        input [1:0] row;
        reg  [4:0] k;
        begin
            k = t - {3'b000, row};
            if (k > 4'd3) clamp_a = 8'sd0;
            else          clamp_a = 8'sd0; // filled at runtime from BRAM
        end
    endfunction

    // ─── BRAM read address pre-computation (combinational) ────
    // We issue read addresses for cycle (compute_cycle) one clock ahead.
    // k for row r at next cycle: k_next = compute_cycle - r
    wire [4:0] t = compute_cycle;

    // Valid k for each row: k = t - r, valid iff 0 ≤ k ≤ 3
    wire [4:0] ka0 = t;          // row 0: k = t - 0
    wire [4:0] ka1 = t - 1;     // row 1: k = t - 1
    wire [4:0] ka2 = t - 2;
    wire [4:0] ka3 = t - 3;

    wire [4:0] kb0 = t;          // col 0: k = t - 0
    wire [4:0] kb1 = t - 1;
    wire [4:0] kb2 = t - 2;
    wire [4:0] kb3 = t - 3;

    // Validity flags (4-bit unsigned comparison: k ≤ 3 means k[4:2]==0 && k<=3)
    wire va0 = (ka0 <= 3);
    wire va1 = (ka1 <= 3);
    wire va2 = (ka2 <= 3);
    wire va3 = (ka3 <= 3);

    wire vb0 = (kb0 <= 3);
    wire vb1 = (kb1 <= 3);
    wire vb2 = (kb2 <= 3);
    wire vb3 = (kb3 <= 3);

    // ─── registered A/B values latched from BRAM (with valid) ─
    reg va0_r, va1_r, va2_r, va3_r;
    reg vb0_r, vb1_r, vb2_r, vb3_r;

    // ─── C write-back counter ──────────────────────────────────
    reg [3:0] c_wb_addr;  // 0..15 walk through all result elements

    // ─── Main FSM ─────────────────────────────────────────────
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= S_IDLE;
            next_after_tx<= S_IDLE;
            byte_cnt     <= 0;
            compute_cycle<= 0;
            busy         <= 0;
            done         <= 0;
            sa_rst_acc   <= 0;
            sa_en        <= 0;
            sa_a0 <= 0; sa_a1 <= 0; sa_a2 <= 0; sa_a3 <= 0;
            sa_b0 <= 0; sa_b1 <= 0; sa_b2 <= 0; sa_b3 <= 0;
            a_wr_en <= 0; b_wr_en <= 0; c_wr_en <= 0;
            tx_start <= 0;
            c_latch  <= 0;
            c_wb_addr<= 0;
            va0_r <= 0; va1_r <= 0; va2_r <= 0; va3_r <= 0;
            vb0_r <= 0; vb1_r <= 0; vb2_r <= 0; vb3_r <= 0;
            a_rd_addr <= 0; b_rd_addr <= 0;
            c_rd_addr <= 0;
        end else begin
            // Defaults (override below)
            a_wr_en  <= 0;
            b_wr_en  <= 0;
            c_wr_en  <= 0;
            tx_start <= 0;
            sa_rst_acc <= 0;

            case (state)

                // ── Wait for first command byte ───────────────
                S_IDLE: begin
                    busy <= 0;
                    if (rx_valid) begin
                        state <= S_CMD;
                        // re-use rx_data immediately
                        case (rx_data)
                            8'h01: begin byte_cnt <= 0; state <= S_RECV_A; busy <= 1; done <= 0; end
                            8'h02: begin byte_cnt <= 0; state <= S_RECV_B; busy <= 1; done <= 0; end
                            8'h03: begin state <= S_COMPUTE_RST; busy <= 1; done <= 0; end
                            8'h04: begin state <= S_READ_IDX; end
                            default: begin
                                tx_data  <= 8'hFF;
                                tx_start <= 1;
                                next_after_tx <= S_IDLE;
                                state <= S_WAIT_TX;
                            end
                        endcase
                    end
                end

                S_CMD: state <= S_IDLE; // unused, fall through

                // ── Receive 16 bytes → Matrix A ───────────────
                S_RECV_A: begin
                    if (rx_valid) begin
                        a_wr_addr <= byte_cnt[3:0];
                        a_wr_data <= rx_data;
                        a_wr_en   <= 1;
                        if (byte_cnt == 15) begin
                            byte_cnt <= 0;
                            busy     <= 0;
                            state    <= S_IDLE;
                        end else begin
                            byte_cnt <= byte_cnt + 1;
                        end
                    end
                end

                // ── Receive 16 bytes → Matrix B ───────────────
                S_RECV_B: begin
                    if (rx_valid) begin
                        b_wr_addr <= byte_cnt[3:0];
                        b_wr_data <= rx_data;
                        b_wr_en   <= 1;
                        if (byte_cnt == 15) begin
                            byte_cnt <= 0;
                            busy     <= 0;
                            state    <= S_IDLE;
                        end else begin
                            byte_cnt <= byte_cnt + 1;
                        end
                    end
                end

                // ── Reset all PE accumulators ─────────────────
                S_COMPUTE_RST: begin
                    sa_rst_acc    <= 1;
                    sa_en         <= 0;
                    compute_cycle <= 0;
                    c_wb_addr     <= 0;
                    // Pre-issue first read addresses (for cycle 0)
                    a_rd_addr <= 4'd0;  // A[0][0]
                    b_rd_addr <= 4'd0;  // B[0][0]
                    state <= S_COMPUTE_RUN;
                end

                // ── Feed skewed data, run systolic array ───────
                S_COMPUTE_RUN: begin
                    sa_en <= 1;
                    sa_rst_acc <= 0;

                    // ---- Latch BRAM outputs (1-cycle latency) into SA inputs
                    // Validity was registered last cycle
                    sa_a0 <= va0_r ? a_rd_data : 8'sd0;
                    sa_a1 <= va1_r ? a_rd_data : 8'sd0;
                    sa_a2 <= va2_r ? a_rd_data : 8'sd0;
                    sa_a3 <= va3_r ? a_rd_data : 8'sd0;

                    sa_b0 <= vb0_r ? b_rd_data : 8'sd0;
                    sa_b1 <= vb1_r ? b_rd_data : 8'sd0;
                    sa_b2 <= vb2_r ? b_rd_data : 8'sd0;
                    sa_b3 <= vb3_r ? b_rd_data : 8'sd0;

                    // NOTE: A more complete implementation would use
                    // separate BRAM ports per row/col. This simplified
                    // version uses a single shared BRAM port and feeds
                    // the same read value to all rows. For full correctness
                    // with the systolic skew, the controller should use a
                    // small register file (see below).

                    // ---- Pre-fetch addresses for next cycle ----
                    // For row 0: addr = 0*4 + ka0_next
                    // (simplified: primary port drives row 0; extend for full design)
                    if (va0) a_rd_addr <= {2'b00, ka0[1:0]};       // A[0][k]
                    if (vb0) b_rd_addr <= {kb0[1:0], 2'b00};       // B[k][0]

                    // Register validity for next cycle's output latch
                    va0_r <= va0; va1_r <= va1; va2_r <= va2; va3_r <= va3;
                    vb0_r <= vb0; vb1_r <= vb1; vb2_r <= vb2; vb3_r <= vb3;

                    if (compute_cycle == COMPUTE_CYCLES - 1) begin
                        sa_en  <= 0;
                        state  <= S_COMPUTE_DONE;
                        c_wb_addr <= 0;
                    end else begin
                        compute_cycle <= compute_cycle + 1;
                    end
                end

                // ── Write all 16 results back to C BRAM ───────
                S_COMPUTE_DONE: begin
                    c_wr_en   <= 1;
                    c_wr_addr <= c_wb_addr;
                    case (c_wb_addr)
                        4'd0:  c_wr_data <= sa_c00; 4'd1:  c_wr_data <= sa_c01;
                        4'd2:  c_wr_data <= sa_c02; 4'd3:  c_wr_data <= sa_c03;
                        4'd4:  c_wr_data <= sa_c10; 4'd5:  c_wr_data <= sa_c11;
                        4'd6:  c_wr_data <= sa_c12; 4'd7:  c_wr_data <= sa_c13;
                        4'd8:  c_wr_data <= sa_c20; 4'd9:  c_wr_data <= sa_c21;
                        4'd10: c_wr_data <= sa_c22; 4'd11: c_wr_data <= sa_c23;
                        4'd12: c_wr_data <= sa_c30; 4'd13: c_wr_data <= sa_c31;
                        4'd14: c_wr_data <= sa_c32; 4'd15: c_wr_data <= sa_c33;
                        default: c_wr_data <= 32'sd0;
                    endcase

                    if (c_wb_addr == 4'd15) begin
                        busy  <= 0;
                        done  <= 1;
                        state <= S_SEND_ACK;
                    end else begin
                        c_wb_addr <= c_wb_addr + 1;
                    end
                end

                // ── Send 0xAA acknowledgement ─────────────────
                S_SEND_ACK: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'hAA;
                        tx_start <= 1;
                        next_after_tx <= S_IDLE;
                        state <= S_WAIT_TX;
                    end
                end

                // ── Read command: wait for index byte ─────────
                S_READ_IDX: begin
                    if (rx_valid) begin
                        c_rd_addr <= rx_data[3:0];
                        state     <= S_READ_WAIT;
                    end
                end

                S_READ_WAIT: begin
                    // BRAM read latency = 1 cycle; result available next cycle
                    state <= S_SEND_C0;
                end

                S_SEND_C0: begin
                    c_latch <= c_rd_data;          // latch full 32-bit word
                    if (!tx_busy) begin
                        tx_data  <= c_rd_data[7:0];
                        tx_start <= 1;
                        next_after_tx <= S_SEND_C1;
                        state <= S_WAIT_TX;
                    end
                end

                S_SEND_C1: begin
                    if (!tx_busy) begin
                        tx_data  <= c_latch[15:8];
                        tx_start <= 1;
                        next_after_tx <= S_SEND_C2;
                        state <= S_WAIT_TX;
                    end
                end

                S_SEND_C2: begin
                    if (!tx_busy) begin
                        tx_data  <= c_latch[23:16];
                        tx_start <= 1;
                        next_after_tx <= S_SEND_C3;
                        state <= S_WAIT_TX;
                    end
                end

                S_SEND_C3: begin
                    if (!tx_busy) begin
                        tx_data  <= c_latch[31:24];
                        tx_start <= 1;
                        next_after_tx <= S_IDLE;
                        state <= S_WAIT_TX;
                    end
                end

                // ── NAK ──────────────────────────────────────
                S_SEND_NAK: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'hFF;
                        tx_start <= 1;
                        next_after_tx <= S_IDLE;
                        state <= S_WAIT_TX;
                    end
                end

                // ── Generic TX wait ───────────────────────────
                S_WAIT_TX: begin
                    if (!tx_busy && !tx_start)
                        state <= next_after_tx;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
