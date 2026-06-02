`timescale 1ns/1ps
// Module: falconsign_ntt_bfly
// Purpose: pipelined modular butterfly for q=12289 NTT arithmetic.
//
// NTT butterfly with Barrett modular reduction (q = 12289).
// 2-stage pipeline: stage0 = b*w Barrett reduce, stage1 = a +/- t mod q.
// y0 = (a + b*w) mod q,  y1 = (a - b*w) mod q
module falconsign_ntt_bfly (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [13:0] a_i,
    input  wire [13:0] b_i,
    input  wire [13:0] w_i,
    output reg         out_valid,
    input  wire        out_ready,
    output reg  [13:0] y0_o,
    output reg  [13:0] y1_o
);

    localparam [13:0] Q          = 14'd12289;
    localparam [27:0] BARRETT_MU = 28'd21843;  // floor(2^28 / 12289)

    // ── Stage 0: multiply and Barrett reduce (combinational) ──
    wire [27:0] prod = b_i * w_i;

    // Barrett: q_hat = (prod * MU) >> 28
    wire [42:0] qhat_full = prod * BARRETT_MU;
    wire [13:0] qhat       = qhat_full[41:28];
    wire [27:0] qhat_mul_q = qhat * Q;
    wire [27:0] r1 = prod - qhat_mul_q;
    wire [27:0] r2 = (r1 >= Q) ? (r1 - Q) : r1;
    wire [13:0] t  = (r2 >= Q) ? (r2[13:0] - Q) : r2[13:0];

    // ── Pipeline registers ──
    reg        s0_valid;
    reg [13:0] s0_a;
    reg [13:0] s0_t;

    // ── Stage 1: butterfly add/sub (combinational from registered values) ──
    wire [14:0] y0_raw = {1'b0, s0_a} + {1'b0, s0_t};
    wire [13:0] y0_mod = (y0_raw >= Q) ? (y0_raw[13:0] - Q) : y0_raw[13:0];

    wire signed [14:0] diff = {1'b0, s0_a} - {1'b0, s0_t};
    wire [13:0] y1_mod = diff[14] ? (diff[13:0] + Q) : diff[13:0];

    assign in_ready = 1'b1;

    // Two-stage butterfly pipeline: stage0 registers a and reduced b*w;
    // stage1 emits modular sum/difference.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_valid  <= 1'b0;
            s0_a      <= 14'd0;
            s0_t      <= 14'd0;
            out_valid <= 1'b0;
            y0_o      <= 14'd0;
            y1_o      <= 14'd0;
        end else begin
            s0_valid <= in_valid;
            s0_a     <= a_i;
            s0_t     <= t;

            out_valid <= s0_valid;
            y0_o      <= y0_mod;
            y1_o      <= y1_mod;
        end
    end

endmodule
