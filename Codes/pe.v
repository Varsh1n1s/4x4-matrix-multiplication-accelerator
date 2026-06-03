// pe.v — Processing Element for the 4×4 systolic array.
//
// Each PE performs:
//   acc += a_in * b_in   (8-bit signed × 8-bit signed → 32-bit signed accumulator)
//
// Data flow (weight-stationary variant):
//   • a_in  flows horizontally (left → right).  PE registers and passes a_in east.
//   • b_in  flows vertically   (top  → bottom). PE registers and passes b_in south.
//   • acc   is local; cleared by rst_acc; read out via acc_out.
//
// Timing: all registers update on posedge clk.

module pe (
    input  wire        clk,
    input  wire        rst,        // global synchronous reset
    input  wire        rst_acc,    // clear accumulator (begin new computation)
    input  wire        en,         // compute enable
    input  wire signed [7:0]  a_in,
    input  wire signed [7:0]  b_in,
    output reg  signed [7:0]  a_out,  // passes a_in to the right neighbour
    output reg  signed [7:0]  b_out,  // passes b_in to the bottom neighbour
    output reg  signed [31:0] acc_out // running accumulator value
);

    wire signed [15:0] product = a_in * b_in;  // 8×8 → 16-bit, sign-extended

    always @(posedge clk) begin
        if (rst) begin
            a_out   <= 8'sd0;
            b_out   <= 8'sd0;
            acc_out <= 32'sd0;
        end else begin
            // Always pipeline the pass-through registers
            a_out <= a_in;
            b_out <= b_in;

            if (rst_acc) begin
                acc_out <= 32'sd0;
            end else if (en) begin
                acc_out <= acc_out + {{16{product[15]}}, product}; // sign-extend to 32
            end
        end
    end

endmodule
