`timescale 1ns/1ps
// Module: falcon_f64_add
// Purpose: combinational IEEE-754 double-precision add/subtract helper for
// Falcon f64 datapaths.

module falcon_f64_add (
    input      [63:0] a,
    input      [63:0] b,
    input             sub,
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
    reg [52:0] mant_a;
    reg [52:0] mant_b;
    reg        a_is_zero;
    reg        b_is_zero;
    reg        a_is_denorm;
    reg        b_is_denorm;
    reg        a_is_special;
    reg        b_is_special;

    reg        sign_big;
    reg        sign_small;
    reg [10:0] exp_big;
    reg [10:0] exp_small;
    reg [52:0] mant_big;
    reg [52:0] mant_small;

    reg [55:0] ext_big;
    reg [55:0] ext_small;
    reg [55:0] norm_ext;
    reg [56:0] sum_ext;
    reg [55:0] diff_ext;

    reg        sign_res;
    reg [52:0] mant_res;
    reg [53:0] mant_round;
    reg        guard_bit;
    reg        round_bit;
    reg        sticky_bit;
    reg        round_up;
    reg [5:0]  norm_shift;
    reg signed [11:0] exp_res;

    reg [10:0] align_shift;
    reg [5:0]  scan_idx;
    reg        shift_sticky;
    reg [55:0] shift_value;
    reg [55:0] shift_tmp;

    // Combinational f64 add/sub datapath: unpack, align, add/subtract,
    // normalize, round-to-nearest-even, and repack.
    always @(*) begin
        sign_a = a[63];
        sign_b = b[63] ^ sub;
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

        sign_big   = 1'b0;
        sign_small = 1'b0;
        exp_big    = 11'd0;
        exp_small  = 11'd0;
        mant_big   = 53'd0;
        mant_small = 53'd0;
        ext_big    = 56'd0;
        ext_small  = 56'd0;
        norm_ext   = 56'd0;
        sum_ext    = 57'd0;
        diff_ext   = 56'd0;
        sign_res   = 1'b0;
        mant_res   = 53'd0;
        mant_round = 54'd0;
        guard_bit  = 1'b0;
        round_bit  = 1'b0;
        sticky_bit = 1'b0;
        round_up   = 1'b0;
        norm_shift = 6'd0;
        exp_res    = 0;
        align_shift = 11'd0;
        scan_idx    = 6'd0;
        shift_sticky = 1'b0;
        shift_value  = 56'd0;
        shift_tmp    = 56'd0;

        y         = 64'd0;
        invalid   = 1'b0;
        overflow  = 1'b0;
        underflow = 1'b0;
        inexact   = 1'b0;

        if (a_is_special || b_is_special) begin
            invalid = 1'b1;
            y       = QNAN;
        end else if ((mant_a == 53'd0) && (mant_b == 53'd0)) begin
            y = 64'd0;
        end else if (mant_a == 53'd0) begin
            y = {sign_b, exp_b, frac_b};
            if (b_is_denorm) begin
                y = {sign_b, 11'd0, 52'd0};
            end
        end else if (mant_b == 53'd0) begin
            y = {sign_a, exp_a, frac_a};
            if (a_is_denorm) begin
                y = {sign_a, 11'd0, 52'd0};
            end
        end else begin
            if ((exp_b > exp_a) || ((exp_b == exp_a) && (mant_b > mant_a))) begin
                sign_big   = sign_b;
                sign_small = sign_a;
                exp_big    = exp_b;
                exp_small  = exp_a;
                mant_big   = mant_b;
                mant_small = mant_a;
            end else begin
                sign_big   = sign_a;
                sign_small = sign_b;
                exp_big    = exp_a;
                exp_small  = exp_b;
                mant_big   = mant_a;
                mant_small = mant_b;
            end

            ext_big   = {mant_big, 3'b000};
            shift_value = {mant_small, 3'b000};
            align_shift = exp_big - exp_small;
            shift_sticky = 1'b0;
            if (align_shift == 11'd0) begin
                ext_small = shift_value;
            end else if (align_shift >= 11'd56) begin
                ext_small = 56'd0;
                ext_small[0] = |shift_value;
            end else begin
                shift_tmp = shift_value >> align_shift;
                for (scan_idx = 0; scan_idx < 56; scan_idx = scan_idx + 1) begin
                    if ((scan_idx < align_shift) && shift_value[scan_idx]) begin
                        shift_sticky = 1'b1;
                    end
                end
                shift_tmp[0] = shift_tmp[0] | shift_sticky;
                ext_small = shift_tmp;
            end

            if (sign_big == sign_small) begin
                sum_ext = {1'b0, ext_big} + {1'b0, ext_small};
                exp_res = exp_big;
                sign_res = sign_big;

                if (sum_ext[56]) begin
                    norm_ext = sum_ext[56:1];
                    norm_ext[0] = norm_ext[0] | sum_ext[0];
                    exp_res = exp_res + 1;
                end else begin
                    norm_ext = sum_ext[55:0];
                end
            end else begin
                diff_ext = ext_big - ext_small;
                sign_res = sign_big;
                exp_res  = exp_big;

                if (diff_ext == 56'd0) begin
                    y = 64'd0;
                    exp_res = -1;
                end else begin
                    norm_shift = 6'd56;
                    for (scan_idx = 0; scan_idx < 56; scan_idx = scan_idx + 1) begin
                        if ((norm_shift == 6'd56) && diff_ext[55-scan_idx]) begin
                            norm_shift = scan_idx;
                        end
                    end
                    if ((norm_shift == 6'd56) || (exp_res <= norm_shift)) begin
                        underflow = 1'b1;
                        y = {sign_res, 11'd0, 52'd0};
                        exp_res = -1;
                    end else begin
                        norm_ext = diff_ext << norm_shift;
                        exp_res  = exp_res - norm_shift;
                    end
                end
            end

            if (exp_res > 0) begin
                mant_res   = norm_ext[55:3];
                guard_bit  = norm_ext[2];
                round_bit  = norm_ext[1];
                sticky_bit = norm_ext[0];
                inexact    = guard_bit | round_bit | sticky_bit;
                round_up   = guard_bit & (round_bit | sticky_bit | mant_res[0]);
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
    end

endmodule
