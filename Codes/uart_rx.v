// uart_rx.v
// 8N1 UART receiver, 16x oversampled.
// Inputs:  clk, rst, baud_tick (16x baud pulse from baud_gen), rx_serial
// Outputs: rx_data [7:0], rx_valid (1-cycle pulse when a byte is ready)

module uart_rx (
    input  wire       clk,
    input  wire       rst,
    input  wire       baud_tick,   // 16x oversampled tick from baud_gen
    input  wire       rx_serial,   // raw UART RX line
    output reg  [7:0] rx_data,
    output reg        rx_valid     // 1-cycle pulse: rx_data is valid
);

    // ---------- double-flop synchroniser (metastability) ----------
    reg rx_s1, rx_s2;
    always @(posedge clk) begin
        rx_s1 <= rx_serial;
        rx_s2 <= rx_s1;
    end

    // ---------- FSM states ----------
    localparam IDLE  = 2'd0,
               START = 2'd1,
               DATA  = 2'd2,
               STOP  = 2'd3;

    reg [1:0] state;
    reg [3:0] tick_cnt;   // counts 0..15 within each bit period
    reg [2:0] bit_idx;    // which data bit we're receiving (0..7)
    reg [7:0] shift_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= IDLE;
            tick_cnt  <= 0;
            bit_idx   <= 0;
            shift_reg <= 0;
            rx_data   <= 0;
            rx_valid  <= 0;
        end else begin
            rx_valid <= 1'b0; // default: not valid

            case (state)

                // ---- wait for falling edge (start bit) ----------
                IDLE: begin
                    if (!rx_s2) begin           // line pulled low → start bit
                        state    <= START;
                        tick_cnt <= 0;
                    end
                end

                // ---- verify start bit at mid-point (tick 7) -----
                START: begin
                    if (baud_tick) begin
                        if (tick_cnt == 4'd7) begin
                            if (!rx_s2) begin   // still low: valid start bit
                                state    <= DATA;
                                tick_cnt <= 0;
                                bit_idx  <= 0;
                            end else begin      // glitch: abort
                                state <= IDLE;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end
                    end
                end

                // ---- sample each data bit at mid-point ----------
                DATA: begin
                    if (baud_tick) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt            <= 0;
                            shift_reg[bit_idx]  <= rx_s2;  // LSB first
                            if (bit_idx == 3'd7) begin
                                state   <= STOP;
                            end else begin
                                bit_idx <= bit_idx + 1;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end
                    end
                end

                // ---- verify stop bit ----------------------------
                STOP: begin
                    if (baud_tick) begin
                        if (tick_cnt == 4'd15) begin
                            if (rx_s2) begin    // stop bit must be high
                                rx_data  <= shift_reg;
                                rx_valid <= 1'b1;
                            end
                            // whether good or framing error, return to IDLE
                            state    <= IDLE;
                            tick_cnt <= 0;
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
