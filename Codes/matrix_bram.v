// matrix_bram.v
// Synchronous single-port BRAM-backed storage for matrices A, B, and C.
//
// Memory layout (flat address space):
//   Addresses 0x00–0x0F : Matrix A  (16 × 8-bit signed elements)
//   Addresses 0x10–0x1F : Matrix B  (16 × 8-bit signed elements)
//   Addresses 0x20–0x2F : Matrix C  (16 × 32-bit signed results, byte 0)
//   Addresses 0x30–0x3F : Matrix C byte 1
//   Addresses 0x40–0x4F : Matrix C byte 2
//   Addresses 0x50–0x5F : Matrix C byte 3
//
// Simpler view for the controller:
//   A/B reads/writes use 8-bit data path (addr 0x00–0x1F).
//   C reads use a 32-bit wide port via c_rd_addr (0–15).
//   C writes use a 32-bit wide port via c_wr_* signals.
//
// This keeps synthesis simple (inferred Block RAM) and avoids
// complex byte-enable logic.

module matrix_bram (
    input  wire        clk,

    // ---------- Matrix A port (8-bit) ----------
    input  wire [3:0]  a_wr_addr,   // element index 0–15 (row*4 + col)
    input  wire [7:0]  a_wr_data,
    input  wire        a_wr_en,
    input  wire [3:0]  a_rd_addr,
    output reg  signed [7:0]  a_rd_data,

    // ---------- Matrix B port (8-bit) ----------
    input  wire [3:0]  b_wr_addr,
    input  wire [7:0]  b_wr_data,
    input  wire        b_wr_en,
    input  wire [3:0]  b_rd_addr,
    output reg  signed [7:0]  b_rd_data,

    // ---------- Matrix C port (32-bit) ----------
    input  wire [3:0]  c_wr_addr,
    input  wire signed [31:0] c_wr_data,
    input  wire        c_wr_en,
    input  wire [3:0]  c_rd_addr,
    output reg  signed [31:0] c_rd_data
);

    // ---------- Storage arrays ----------
    // Xilinx tools will infer Block RAM for these patterns.
    reg signed [7:0]  mem_a [0:15];
    reg signed [7:0]  mem_b [0:15];
    reg signed [31:0] mem_c [0:15];

    integer i;
    initial begin
        for (i = 0; i < 16; i = i + 1) begin
            mem_a[i] = 8'sd0;
            mem_b[i] = 8'sd0;
            mem_c[i] = 32'sd0;
        end
    end

    // ---------- Matrix A ----------
    always @(posedge clk) begin
        if (a_wr_en)
            mem_a[a_wr_addr] <= a_wr_data;
        a_rd_data <= mem_a[a_rd_addr];
    end

    // ---------- Matrix B ----------
    always @(posedge clk) begin
        if (b_wr_en)
            mem_b[b_wr_addr] <= b_wr_data;
        b_rd_data <= mem_b[b_rd_addr];
    end

    // ---------- Matrix C ----------
    always @(posedge clk) begin
        if (c_wr_en)
            mem_c[c_wr_addr] <= c_wr_data;
        c_rd_data <= mem_c[c_rd_addr];
    end

endmodule
