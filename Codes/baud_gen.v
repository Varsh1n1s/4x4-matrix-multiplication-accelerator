// baud_gen.v
// Baud rate tick generator for 115200 baud at 100 MHz clock.
// Generates a single-cycle pulse (tick) at the desired baud rate.
// Oversamples at 16x for UART RX sampling accuracy.
//
// CLK_FREQ  = 100_000_000
// BAUD_RATE = 115_200
// Divider   = 100_000_000 / (16 * 115_200) = ~54  (exact: 54.25 → use 54)

module baud_gen #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200,
    parameter OVERSAMPLE = 16
)(
    input  wire clk,
    input  wire rst,
    output reg  tick        // 1-cycle pulse at 16x baud rate
);

    // Divider value: how many clk cycles per oversample tick
    localparam integer DIVIDER = CLK_FREQ / (BAUD_RATE * OVERSAMPLE); // 54

    // Counter width: clog2(54) = 6 bits
    localparam CNT_W = $clog2(DIVIDER);

    reg [CNT_W-1:0] count;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            count <= 0;
            tick  <= 1'b0;
        end else begin
            if (count == DIVIDER - 1) begin
                count <= 0;
                tick  <= 1'b1;
            end else begin
                count <= count + 1;
                tick  <= 1'b0;
            end
        end
    end

endmodule
