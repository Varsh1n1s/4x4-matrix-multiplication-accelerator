// uart_tx.v
// 8N1 UART transmitter, 16x oversampled timing.
// Accepts a byte + send strobe; asserts tx_busy while transmitting.
//
// Protocol: assert tx_start for 1 cycle with tx_data valid.
// Do not assert tx_start again while tx_busy is high.

module uart_tx (
    input  wire       clk,
    input  wire       rst,
    input  wire       baud_tick,   // 16x oversampled tick from baud_gen
    input  wire [7:0] tx_data,
    input  wire       tx_start,    // 1-cycle strobe: load tx_data and begin
    output reg        tx_serial,   // UART TX line
    output reg        tx_busy
);

    localparam IDLE  = 2'd0,
               START = 2'd1,
               DATA  = 2'd2,
               STOP  = 2'd3;

    reg [1:0] state;
    reg [3:0] tick_cnt;
    reg [2:0] bit_idx;
    reg [7:0] shift_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state      <= IDLE;
            tx_serial  <= 1'b1;   // idle high
            tx_busy    <= 1'b0;
            tick_cnt   <= 0;
            bit_idx    <= 0;
            shift_reg  <= 0;
        end else begin
            case (state)

                IDLE: begin
                    tx_serial <= 1'b1;
                    tx_busy   <= 1'b0;
                    if (tx_start) begin
                        shift_reg <= tx_data;
                        tx_busy   <= 1'b1;
                        tick_cnt  <= 0;
                        state     <= START;
                    end
                end

                // ---- send start bit (low) for 16 ticks ----------
                START: begin
                    tx_serial <= 1'b0;
                    if (baud_tick) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 0;
                            bit_idx  <= 0;
                            state    <= DATA;
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end
                    end
                end

                // ---- send 8 data bits, LSB first ----------------
                DATA: begin
                    tx_serial <= shift_reg[bit_idx];
                    if (baud_tick) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 0;
                            if (bit_idx == 3'd7) begin
                                state <= STOP;
                            end else begin
                                bit_idx <= bit_idx + 1;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end
                    end
                end

                // ---- send stop bit (high) for 16 ticks ----------
                STOP: begin
                    tx_serial <= 1'b1;
                    if (baud_tick) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 0;
                            state    <= IDLE;
                            tx_busy  <= 1'b0;
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
