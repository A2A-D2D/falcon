`timescale 1ns/1ps
// Module: falcon_f64_mul
// Purpose: combinational IEEE-754 double-precision multiplier for Falcon f64
// datapaths.

module falcon_f64_mul (
    input      [63:0] a,
    input      [63:0] b,
    output reg [63:0] y,
    output reg        invalid,
    output reg        overflow,
    output reg        underflow,
    output reg        inexact
);

    localparam [63:0] QNAN = 64'h7ff8_0000_0000_0000;

    reg        sign_a;
    reg        sign_b;
    reg [10:0] exp_a;
    reg [10:0] exp_b;
    reg [51:0] frac_a;
    reg [51:0] frac_b;
    reg        a_is_zero;
    reg        b_is_zero;
    reg        a_is_denorm;
    reg        b_is_denorm;
    reg        a_is_special;
    reg        b_is_special;
    reg [52:0] mant_a;
    reg [52:0] mant_b;
    reg [105:0] product;
    reg [52:0] mant_res;
    reg [53:0] mant_round;
    reg        sign_res;
    reg        guard_bit;
    reg        round_bit;
    reg        sticky_bit;
    reg        round_up;
    integer    exp_res;

    // Combinational f64 multiply datapath: unpack, multiply mantissas,
    // normalize, round-to-nearest-even, and repack.
    always @(*) begin
        sign_a = a[63];
        sign_b = b[63];
        exp_a  = a[62:52];
        exp_b  = b[62:52];
        frac_a = a[51:0];
        frac_b = b[51:0];

        a_is_zero    = (exp_a == 11'd0) && (frac_a == 52'd0);
        b_is_zero    = (exp_b == 11'd0) && (frac_b == 52'd0);
        a_is_denorm  = (exp_a == 11'd0) && (frac_a != 52'd0);
        b_is_denorm  = (exp_b == 11'd0) && (frac_b != 52'd0);
        a_is_special = (exp_a == 11'h7ff);
        b_is_special = (exp_b == 11'h7ff);

        mant_a = a_is_zero || a_is_denorm ? 53'd0 : {1'b1, frac_a};
        mant_b = b_is_zero || b_is_denorm ? 53'd0 : {1'b1, frac_b};
        product = 106'd0;
        mant_res = 53'd0;
        mant_round = 54'd0;
        sign_res = sign_a ^ sign_b;
        guard_bit = 1'b0;
        round_bit = 1'b0;
        sticky_bit = 1'b0;
        round_up = 1'b0;
        exp_res = 0;

        y         = 64'd0;
        invalid   = 1'b0;
        overflow  = 1'b0;
        underflow = 1'b0;
        inexact   = 1'b0;

        if (a_is_special || b_is_special) begin
            invalid = 1'b1;
            y       = QNAN;
        end else if ((mant_a == 53'd0) || (mant_b == 53'd0)) begin
            y = {sign_res, 11'd0, 52'd0};
            if (a_is_denorm || b_is_denorm) begin
                underflow = 1'b1;
            end
        end else begin
            product = mant_a * mant_b;
            exp_res = exp_a + exp_b - 1023;

            if (product[105]) begin
                mant_res   = product[105:53];
                guard_bit  = product[52];
                round_bit  = product[51];
                sticky_bit = |product[50:0];
                exp_res    = exp_res + 1;
            end else begin
                mant_res   = product[104:52];
                guard_bit  = product[51];
                round_bit  = product[50];
                sticky_bit = |product[49:0];
            end

            inexact  = guard_bit | round_bit | sticky_bit;
            round_up = guard_bit & (round_bit | sticky_bit | mant_res[0]);
            mant_round = {1'b0, mant_res} + round_up;

            if (mant_round[53]) begin
                mant_res = mant_round[53:1];
                exp_res  = exp_res + 1;
                inexact  = 1'b1;
            end else begin
                mant_res = mant_round[52:0];
            end

            if (exp_res >= 2047) begin
                overflow = 1'b1;
                y = {sign_res, 11'h7ff, 52'd0};
            end else if (exp_res <= 0) begin
                underflow = 1'b1;
                y = {sign_res, 11'd0, 52'd0};
            end else begin
                y = {sign_res, exp_res[10:0], mant_res[51:0]};
            end
        end
    end

endmodule
