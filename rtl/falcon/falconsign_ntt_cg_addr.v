`timescale 1ns/1ps
// Module: falconsign_ntt_cg_addr
// Purpose: address generator for the constant-geometry NTT schedule.
//
// Stockham Constant-Geometry NTT address generator.
// Produces natural-order coefficient pairs — no bit-reversal needed.
module falconsign_ntt_cg_addr #(
    parameter LOGN  = 5'd9,
    parameter ADDR_W = 11
) (
    input  wire [4:0]  stage_idx,   // 0..8
    input  wire [8:0]  pair_idx,    // 0..255
    output wire [8:0]  coeff_a,     // 0..511
    output wire [8:0]  coeff_b,
    output wire [7:0]  twiddle_idx,
    output wire [ADDR_W-1:0] word_a,
    output wire [3:0]  lane_a,
    output wire [ADDR_W-1:0] word_b,
    output wire [3:0]  lane_b
);

    // Stockham CG: stride = 2^stage, group_size = 2 * stride = 2^(stage+1)
    // For pair p in 0..255:
    //   group   = p >> stage        // p / stride
    //   k       = p & (stride - 1)  // p % stride
    //   a_idx   = group * group_size + k
    //   b_idx   = a_idx + stride
    //   t_idx   = k << (LOGN - 1 - stage)

    wire [8:0] stride     = 9'd1 << stage_idx;
    wire [8:0] stride_m1  = stride - 9'd1;
    wire [8:0] group_size = stride << 1;

    wire [8:0] group      = pair_idx >> stage_idx;
    wire [8:0] k_idx      = pair_idx & stride_m1;

    assign coeff_a = group * group_size + k_idx;
    assign coeff_b = coeff_a + stride;

    wire [4:0] tw_shift = LOGN - 5'd1 - stage_idx;
    assign twiddle_idx = k_idx[7:0] << tw_shift;

    assign word_a = {{(ADDR_W-5){1'b0}}, coeff_a[8:4]};
    assign lane_a = coeff_a[3:0];
    assign word_b = {{(ADDR_W-5){1'b0}}, coeff_b[8:4]};
    assign lane_b = coeff_b[3:0];

endmodule
