`timescale 1ns/1ps
// Module: falcon_fp_fpu
// Purpose: small request/response floating-point execution unit. It wraps the
// local f64 add/mul helpers and sequences FMADD/FMSUB-style operations.


module falcon_fp_fpu (
    input         clk,
    input         rst_n,
    input         req_valid,
    output        req_ready,
    input  [3:0]  req_op,
    input  [63:0] req_a,
    input  [63:0] req_b,
    input  [63:0] req_c,
    input  [1:0]  req_fmt,
    input  [2:0]  req_rm,
    input  [1:0]  req_fcvt_op,
    output        rsp_valid,
    input         rsp_ready,
    output [63:0] rsp_result,
    output [4:0]  rsp_flags,
    output        busy
);

    localparam [3:0] OP_FADD     = 4'd0;
    localparam [3:0] OP_FSUB     = 4'd1;
    localparam [3:0] OP_FMUL     = 4'd2;
    localparam [3:0] OP_FMADD    = 4'd3;
    localparam [3:0] OP_FMSUB    = 4'd4;
    localparam [3:0] OP_FNMSUB   = 4'd5;
    localparam [3:0] OP_FNMADD   = 4'd6;
    localparam [3:0] OP_FDIV     = 4'd7;
    localparam [3:0] OP_FSQRT    = 4'd8;
    localparam [3:0] OP_FCMP     = 4'd9;
    localparam [3:0] OP_FSGNJ    = 4'd10;
    localparam [3:0] OP_FMAX     = 4'd11;
    localparam [3:0] OP_FCVT_F2I = 4'd12;
    localparam [3:0] OP_FCVT_I2F = 4'd13;
    localparam [3:0] OP_FCVT_F2F = 4'd14;

    localparam [1:0] ST_IDLE  = 2'd0;
    localparam [1:0] ST_EXEC0 = 2'd1;
    localparam [1:0] ST_EXEC1 = 2'd2;
    localparam [1:0] ST_HOLD  = 2'd3;

    reg [1:0]  state;
    reg [3:0]  op_q;
    reg [63:0] a_q;
    reg [63:0] b_q;
    reg [63:0] c_q;
    reg [1:0]  fmt_q;
    reg [2:0]  rm_q;
    reg [1:0]  fcvt_op_q;
    reg [63:0] tmp_q;
    reg [4:0]  tmp_flags_q;
    reg [63:0] result_q;
    reg [4:0]  flags_q;

    reg  [63:0] add_a;
    reg  [63:0] add_b;
    reg         add_sub;
    wire [63:0] add_y;
    wire        add_invalid;
    wire        add_overflow;
    wire        add_underflow;
    wire        add_inexact;

    reg  [63:0] mul_a;
    reg  [63:0] mul_b;
    wire [63:0] mul_y;
    wire        mul_invalid;
    wire        mul_overflow;
    wire        mul_underflow;
    wire        mul_inexact;

    wire [63:0] neg_tmp;
    integer    fcvt_exp_unb;
    integer    fcvt_rsh;
    reg [63:0] fcvt_int_part;
    reg        fcvt_guard;
    reg        fcvt_rnd_stk;
    reg        fcvt_neg;
    reg [63:0] fcvt_abs_val;
    reg [10:0] fcvt_exp;
    reg [51:0] fcvt_frac;
    integer    fcvt_ii;
    integer    fcvt_pos;
    reg        fcvt_found;

    assign req_ready  = (state == ST_IDLE);
    assign rsp_valid  = (state == ST_HOLD);
    assign rsp_result = result_q;
    assign rsp_flags  = flags_q;
    assign busy       = (state != ST_IDLE);

    assign neg_tmp = {~tmp_q[63], tmp_q[62:0]};

    falcon_f64_add u_add (
        .a         (add_a),
        .b         (add_b),
        .sub       (add_sub),
        .y         (add_y),
        .invalid   (add_invalid),
        .overflow  (add_overflow),
        .underflow (add_underflow),
        .inexact   (add_inexact)
    );

    falcon_f64_mul u_mul (
        .a         (mul_a),
        .b         (mul_b),
        .y         (mul_y),
        .invalid   (mul_invalid),
        .overflow  (mul_overflow),
        .underflow (mul_underflow),
        .inexact   (mul_inexact)
    );

    // Operand mux for the internal add and multiply helpers. Multi-cycle
    // operations reuse the helpers in EXEC0/EXEC1 rather than instantiating
    // separate hardware for each FPU opcode.
    always @(*) begin
        add_a   = 64'd0;
        add_b   = 64'd0;
        add_sub = 1'b0;
        mul_a   = 64'd0;
        mul_b   = 64'd0;

        case (state)
            ST_EXEC0: begin
                case (op_q)
                    OP_FADD: begin
                        add_a = a_q;
                        add_b = b_q;
                    end
                    OP_FSUB: begin
                        add_a   = a_q;
                        add_b   = b_q;
                        add_sub = 1'b1;
                    end
                    OP_FMUL,
                    OP_FMADD,
                    OP_FMSUB,
                    OP_FNMSUB,
                    OP_FNMADD: begin
                        mul_a = a_q;
                        mul_b = b_q;
                    end
                    default: begin
                    end
                endcase
            end

            ST_EXEC1: begin
                case (op_q)
                    OP_FMADD: begin
                        add_a = tmp_q;
                        add_b = c_q;
                    end
                    OP_FMSUB: begin
                        add_a   = tmp_q;
                        add_b   = c_q;
                        add_sub = 1'b1;
                    end
                    OP_FNMSUB: begin
                        add_a   = neg_tmp;
                        add_b   = c_q;
                        add_sub = 1'b1;
                    end
                    OP_FNMADD: begin
                        add_a = neg_tmp;
                        add_b = c_q;
                    end
                    default: begin
                    end
                endcase
            end

            default: begin
            end
        endcase
    end

    // Request/response FSM. A request is latched in IDLE, one or two execute
    // cycles produce the result, and HOLD preserves it until rsp_ready.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            op_q       <= OP_FADD;
            a_q        <= 64'd0;
            b_q        <= 64'd0;
            c_q        <= 64'd0;
            fmt_q      <= 2'b01;
            rm_q       <= 3'b000;
            fcvt_op_q  <= 2'b00;
            tmp_q      <= 64'd0;
            tmp_flags_q <= 5'd0;
            result_q   <= 64'd0;
            flags_q    <= 5'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (req_valid) begin
                        op_q      <= req_op;
                        a_q       <= req_a;
                        b_q       <= req_b;
                        c_q       <= req_c;
                        fmt_q     <= req_fmt;
                        rm_q      <= req_rm;
                        fcvt_op_q <= req_fcvt_op;
                        tmp_q     <= 64'd0;
                        tmp_flags_q <= 5'd0;
                        state     <= ST_EXEC0;
                    end
                end

                ST_EXEC0: begin
                    case (op_q)
                        OP_FADD,
                        OP_FSUB: begin
                            result_q <= add_y;
                            flags_q  <= {add_invalid, 1'b0, add_overflow, add_underflow, add_inexact};
                            state    <= ST_HOLD;
                        end

                        OP_FMUL: begin
                            result_q <= mul_y;
                            flags_q  <= {mul_invalid, 1'b0, mul_overflow, mul_underflow, mul_inexact};
                            state    <= ST_HOLD;
                        end

                        OP_FCMP: begin
                            // FP64 compare a >= b → 1.0 else 0.0
                            // IEEE754 comparison: flip all bits for negative values
                            // to get lexicographic ordering as signed integer.
                            if (a_q == b_q) begin
                                result_q <= 64'h3FF0000000000000;  // 1.0
                            end else if (a_q[63] && !b_q[63]) begin
                                result_q <= 64'd0;  // negative < positive
                            end else if (!a_q[63] && b_q[63]) begin
                                result_q <= 64'h3FF0000000000000;  // positive > negative
                            end else begin
                                // same sign: compare as if sign-bit flipped
                                if ({~a_q[63], a_q[62:0]} >= {~b_q[63], b_q[62:0]})
                                    result_q <= 64'h3FF0000000000000;
                                else
                                    result_q <= 64'd0;
                            end
                            flags_q <= 5'd0;
                            state   <= ST_HOLD;
                        end

                        OP_FCVT_F2I: begin
                            // FP64 -> signed 64-bit integer, round-to-nearest
                            // Integer = 2^exp + (frac >> (52-exp))   (for 0 <= exp <= 52)
                            if (a_q[62:52] < 11'd1023) begin
                                result_q <= 64'd0;
                            end else if (a_q[62:52] >= 11'd1085) begin
                                result_q <= a_q[63] ? 64'h8000000000000000
                                                     : 64'h7FFFFFFFFFFFFFFF;
                            end else begin
                                fcvt_exp_unb = a_q[62:52] - 11'd1023;
                                if (fcvt_exp_unb <= 52) begin
                                    fcvt_rsh = 52 - fcvt_exp_unb;
                                    fcvt_int_part = (64'd1 << fcvt_exp_unb) + (a_q[51:0] >> fcvt_rsh);
                                    fcvt_guard    = (fcvt_rsh > 0) ? ((a_q[51:0] >> (fcvt_rsh - 1)) & 1'b1) : 1'b0;
                                    fcvt_rnd_stk  = (fcvt_rsh > 0) ? |(a_q[51:0] & ((64'd1 << fcvt_rsh) - 1)) : 1'b0;
                                end else begin
                                    fcvt_int_part = (a_q[51:0] << (fcvt_exp_unb - 52));
                                    fcvt_int_part = fcvt_int_part | (64'd1 << fcvt_exp_unb);
                                    fcvt_guard    = 1'b0;
                                    fcvt_rnd_stk  = 1'b0;
                                end
                                // round-to-nearest-even
                                if (fcvt_guard && (fcvt_int_part[0] || fcvt_rnd_stk))
                                    fcvt_int_part = fcvt_int_part + 1'b1;
                                result_q <= a_q[63] ? (~fcvt_int_part + 1'b1) : fcvt_int_part;
                            end
                            flags_q <= 5'd0;
                            state   <= ST_HOLD;
                        end

                        OP_FCVT_I2F: begin
                            // signed 64-bit integer -> FP64
                            if (a_q == 64'd0) begin
                                result_q <= 64'd0;
                            end else begin
                                fcvt_neg = a_q[63];
                                fcvt_abs_val = fcvt_neg ? (~a_q + 1'b1) : a_q;
                                // find position of leading 1
                                fcvt_pos = 63;
                                fcvt_found = 1'b0;
                                for (fcvt_ii = 0; fcvt_ii < 64; fcvt_ii = fcvt_ii + 1) begin
                                    if (fcvt_abs_val[63 - fcvt_ii] && !fcvt_found) begin
                                        fcvt_pos = 63 - fcvt_ii;
                                        fcvt_found = 1'b1;
                                    end
                                end
                                fcvt_exp = 11'd1023 + fcvt_pos;
                                // extract the 52 bits below the leading 1
                                // Equivalent to: (abs_val << (63 - pos)) >> 11
                                fcvt_frac = (fcvt_abs_val << (63 - fcvt_pos)) >> 11;
                                result_q <= {fcvt_neg, fcvt_exp, fcvt_frac};
                            end
                            flags_q <= 5'd0;
                            state   <= ST_HOLD;
                        end

                        OP_FMADD,
                        OP_FMSUB,
                        OP_FNMSUB,
                        OP_FNMADD: begin
                            tmp_q       <= mul_y;
                            tmp_flags_q <= {mul_invalid, 1'b0, mul_overflow, mul_underflow, mul_inexact};
                            state       <= ST_EXEC1;
                        end

                        default: begin
                            result_q <= 64'd0;
                            flags_q  <= 5'b10000;
                            state    <= ST_HOLD;
                        end
                    endcase
                end

                ST_EXEC1: begin
                    result_q <= add_y;
                    flags_q  <= tmp_flags_q | {add_invalid, 1'b0, add_overflow, add_underflow, add_inexact};
                    state    <= ST_HOLD;
                end

                ST_HOLD: begin
                    if (rsp_ready) begin
                        state <= ST_IDLE;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
