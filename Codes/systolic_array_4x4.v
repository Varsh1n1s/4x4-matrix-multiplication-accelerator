// systolic_array_4x4.v
// 4×4 weight-stationary systolic array built from 16 PEs.
//
// Operation: C = A × B  where A, B, C are 4×4 matrices of 8-bit signed integers.
// Each output element C[i][j] = sum_k( A[i][k] * B[k][j] )
//
// Data supply (orchestrated by controller):
//   • A rows are fed from the left, one element per row per clock (skewed).
//   • B columns are fed from the top, one element per column per clock (skewed).
//   • After 4 compute cycles each PE accumulates one dot-product element.
//
// Skew model:
//   PE[i][j] receives A[i][k] and B[k][j] at time k + i + j.
//   The controller must apply the skew (delay) before driving a_in / b_in.
//   Here the array itself is purely structural — all 4 rows of A and 4 columns
//   of B are presented simultaneously; skewing is the controller's responsibility
//   (feed zeros for the padding cycles).
//
// Interface:
//   a_in[r]  — 8-bit signed input to row r, column 0
//   b_in[c]  — 8-bit signed input to column c, row 0
//   rst_acc  — clear all accumulators
//   en       — enable compute for all PEs
//   c_out[r][c] — 32-bit result for element C[r][c], always readable

module systolic_array_4x4 (
    input  wire        clk,
    input  wire        rst,
    input  wire        rst_acc,
    input  wire        en,

    // Row inputs (left edge of each row)
    input  wire signed [7:0] a_in_0,
    input  wire signed [7:0] a_in_1,
    input  wire signed [7:0] a_in_2,
    input  wire signed [7:0] a_in_3,

    // Column inputs (top edge of each column)
    input  wire signed [7:0] b_in_0,
    input  wire signed [7:0] b_in_1,
    input  wire signed [7:0] b_in_2,
    input  wire signed [7:0] b_in_3,

    // 4×4 = 16 result words
    output wire signed [31:0] c_out_00, c_out_01, c_out_02, c_out_03,
    output wire signed [31:0] c_out_10, c_out_11, c_out_12, c_out_13,
    output wire signed [31:0] c_out_20, c_out_21, c_out_22, c_out_23,
    output wire signed [31:0] c_out_30, c_out_31, c_out_32, c_out_33
);

    // ---------- internal wires: horizontal (a) and vertical (b) ----------
    // a_h[r][c] : a value flowing from column c to column c+1 in row r
    // b_v[r][c] : b value flowing from row r to row r+1 in column c
    wire signed [7:0] a_h [0:3][0:4]; // [row][col 0..4], col 0 = input
    wire signed [7:0] b_v [0:4][0:3]; // [row 0..4][col], row 0 = input

    // Connect boundary inputs
    assign a_h[0][0] = a_in_0;
    assign a_h[1][0] = a_in_1;
    assign a_h[2][0] = a_in_2;
    assign a_h[3][0] = a_in_3;

    assign b_v[0][0] = b_in_0;
    assign b_v[0][1] = b_in_1;
    assign b_v[0][2] = b_in_2;
    assign b_v[0][3] = b_in_3;

    // ---------- PE accumulator outputs ----------
    wire signed [31:0] pe_acc [0:3][0:3];

    // ---------- Instantiate 16 PEs ----------
    genvar r, c;
    generate
        for (r = 0; r < 4; r = r + 1) begin : row
            for (c = 0; c < 4; c = c + 1) begin : col
                pe u_pe (
                    .clk     (clk),
                    .rst     (rst),
                    .rst_acc (rst_acc),
                    .en      (en),
                    .a_in    (a_h[r][c]),
                    .b_in    (b_v[r][c]),
                    .a_out   (a_h[r][c+1]),
                    .b_out   (b_v[r+1][c]),
                    .acc_out (pe_acc[r][c])
                );
            end
        end
    endgenerate

    // ---------- Map accumulator array to flat output ports ----------
    assign c_out_00 = pe_acc[0][0]; assign c_out_01 = pe_acc[0][1];
    assign c_out_02 = pe_acc[0][2]; assign c_out_03 = pe_acc[0][3];
    assign c_out_10 = pe_acc[1][0]; assign c_out_11 = pe_acc[1][1];
    assign c_out_12 = pe_acc[1][2]; assign c_out_13 = pe_acc[1][3];
    assign c_out_20 = pe_acc[2][0]; assign c_out_21 = pe_acc[2][1];
    assign c_out_22 = pe_acc[2][2]; assign c_out_23 = pe_acc[2][3];
    assign c_out_30 = pe_acc[3][0]; assign c_out_31 = pe_acc[3][1];
    assign c_out_32 = pe_acc[3][2]; assign c_out_33 = pe_acc[3][3];

endmodule
