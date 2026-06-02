`timescale 1ns/1ps
// Module: falcon_f64_complex_bfly
// Purpose: one complex FFT butterfly over f64 values. It computes
// y0 = a + b*w and y1 = a - b*w through a sequenced FMA datapath.

module falcon_f64_complex_bfly (
    input         clk,
    input         rst_n,
    input         in_valid,
    output        in_ready,
    input  [63:0] a_re,
    input  [63:0] a_im,
    input  [63:0] b_re,
    input  [63:0] b_im,
    input  [63:0] w_re,
    input  [63:0] w_im,
    output reg    out_valid,
    input         out_ready,
    output reg [63:0] y0_re,
    output reg [63:0] y0_im,
    output reg [63:0] y1_re,
    output reg [63:0] y1_im,
    output reg    status_invalid,
    output reg    status_overflow,
    output reg    status_underflow,
    output reg    status_inexact,
    output        busy
);

    localparam [3:0] ST_IDLE  = 4'd0;
    localparam [3:0] ST_Y0R_A = 4'd1;
    localparam [3:0] ST_Y0R_B = 4'd2;
    localparam [3:0] ST_Y1R_A = 4'd3;
    localparam [3:0] ST_Y1R_B = 4'd4;
    localparam [3:0] ST_Y0I_A = 4'd5;
    localparam [3:0] ST_Y0I_B = 4'd6;
    localparam [3:0] ST_Y1I_A = 4'd7;
    localparam [3:0] ST_Y1I_B = 4'd8;
    localparam [3:0] ST_DONE  = 4'd9;

    localparam [3:0] OP_FMADD  = 4'd3;
    localparam [3:0] OP_FNMSUB = 4'd5;
    localparam [3:0] OP_FNMADD = 4'd6;

    reg [3:0] state;
    reg       fpu_pending;

    reg [63:0] a_re_q;
    reg [63:0] a_im_q;
    reg [63:0] b_re_q;
    reg [63:0] b_im_q;
    reg [63:0] w_re_q;
    reg [63:0] w_im_q;

    reg [63:0] tmp_re_q;
    reg [63:0] tmp_im_q;

    reg         fpu_req_valid;
    wire        fpu_req_ready;
    reg  [3:0]  fpu_req_op;
    reg  [63:0] fpu_req_a;
    reg  [63:0] fpu_req_b;
    reg  [63:0] fpu_req_c;
    wire        fpu_rsp_valid;
    wire [63:0] fpu_rsp_result;
    wire [4:0]  fpu_rsp_flags;

    assign in_ready = (state == ST_IDLE);
    assign busy     = (state != ST_IDLE);

    // Combinational FPU command decoder. The sequential FSM below owns state
    // advancement; this block only selects the operation for the current phase.
    always @(*) begin
        fpu_req_valid = 1'b0;
        fpu_req_op    = OP_FMADD;
        fpu_req_a     = 64'd0;
        fpu_req_b     = 64'd0;
        fpu_req_c     = 64'd0;
        out_valid     = 1'b0;

        case (state)
            ST_Y0R_A: begin
                if (!fpu_pending) begin
                    fpu_req_valid = 1'b1;
                    fpu_req_op    = OP_FMADD;
                    fpu_req_a     = b_re_q;
                    fpu_req_b     = w_re_q;
                    fpu_req_c     = a_re_q;
                end
            end

            ST_Y0R_B: begin
                if (!fpu_pending) begin
                    fpu_req_valid = 1'b1;
                    fpu_req_op    = OP_FNMADD;
                    fpu_req_a     = b_im_q;
                    fpu_req_b     = w_im_q;
                    fpu_req_c     = tmp_re_q;
                end
            end

            ST_Y1R_A: begin
                if (!fpu_pending) begin
                    fpu_req_valid = 1'b1;
                    fpu_req_op    = OP_FNMADD;
                    fpu_req_a     = b_re_q;
                    fpu_req_b     = w_re_q;
                    fpu_req_c     = a_re_q;
                end
            end

            ST_Y1R_B: begin
                if (!fpu_pending) begin
                    fpu_req_valid = 1'b1;
                    fpu_req_op    = OP_FMADD;
                    fpu_req_a     = b_im_q;
                    fpu_req_b     = w_im_q;
                    fpu_req_c     = tmp_re_q;
                end
            end

            ST_Y0I_A: begin
                if (!fpu_pending) begin
                    fpu_req_valid = 1'b1;
                    fpu_req_op    = OP_FMADD;
                    fpu_req_a     = b_re_q;
                    fpu_req_b     = w_im_q;
                    fpu_req_c     = a_im_q;
                end
            end

            ST_Y0I_B: begin
                if (!fpu_pending) begin
                    fpu_req_valid = 1'b1;
                    fpu_req_op    = OP_FMADD;
                    fpu_req_a     = b_im_q;
                    fpu_req_b     = w_re_q;
                    fpu_req_c     = tmp_im_q;
                end
            end

            ST_Y1I_A: begin
                if (!fpu_pending) begin
                    fpu_req_valid = 1'b1;
                    fpu_req_op    = OP_FNMADD;
                    fpu_req_a     = b_re_q;
                    fpu_req_b     = w_im_q;
                    fpu_req_c     = a_im_q;
                end
            end

            ST_Y1I_B: begin
                if (!fpu_pending) begin
                    fpu_req_valid = 1'b1;
                    fpu_req_op    = OP_FNMADD;
                    fpu_req_a     = b_im_q;
                    fpu_req_b     = w_re_q;
                    fpu_req_c     = tmp_im_q;
                end
            end

            ST_DONE: begin
                out_valid = 1'b1;
            end

            default: begin
            end
        endcase
    end

    falcon_fp_fpu u_fpu (
        .clk         (clk),
        .rst_n       (rst_n),
        .req_valid   (fpu_req_valid),
        .req_ready   (fpu_req_ready),
        .req_op      (fpu_req_op),
        .req_a       (fpu_req_a),
        .req_b       (fpu_req_b),
        .req_c       (fpu_req_c),
        .req_fmt     (2'b01),
        .req_rm      (3'b000),
        .req_fcvt_op (2'b00),
        .rsp_valid   (fpu_rsp_valid),
        .rsp_ready   (1'b1),
        .rsp_result  (fpu_rsp_result),
        .rsp_flags   (fpu_rsp_flags),
        .busy        ()
    );

    // Butterfly FSM. It latches one input tuple, runs the eight FMA phases, and
    // holds the four output components valid until the caller accepts them.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= ST_IDLE;
            fpu_pending      <= 1'b0;
            a_re_q           <= 64'd0;
            a_im_q           <= 64'd0;
            b_re_q           <= 64'd0;
            b_im_q           <= 64'd0;
            w_re_q           <= 64'd0;
            w_im_q           <= 64'd0;
            tmp_re_q         <= 64'd0;
            tmp_im_q         <= 64'd0;
            y0_re            <= 64'd0;
            y0_im            <= 64'd0;
            y1_re            <= 64'd0;
            y1_im            <= 64'd0;
            status_invalid   <= 1'b0;
            status_overflow  <= 1'b0;
            status_underflow <= 1'b0;
            status_inexact   <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (in_valid) begin
                        a_re_q           <= a_re;
                        a_im_q           <= a_im;
                        b_re_q           <= b_re;
                        b_im_q           <= b_im;
                        w_re_q           <= w_re;
                        w_im_q           <= w_im;
                        tmp_re_q         <= 64'd0;
                        tmp_im_q         <= 64'd0;
                        status_invalid   <= 1'b0;
                        status_overflow  <= 1'b0;
                        status_underflow <= 1'b0;
                        status_inexact   <= 1'b0;
                        fpu_pending      <= 1'b0;
                        state            <= ST_Y0R_A;
                    end
                end

                ST_Y0R_A: begin
                    if (!fpu_pending) begin
                        if (fpu_req_ready) begin
                            fpu_pending <= 1'b1;
                        end
                    end else if (fpu_rsp_valid) begin
                        tmp_re_q         <= fpu_rsp_result;
                        status_invalid   <= status_invalid | fpu_rsp_flags[4];
                        status_overflow  <= status_overflow | fpu_rsp_flags[2];
                        status_underflow <= status_underflow | fpu_rsp_flags[1];
                        status_inexact   <= status_inexact | fpu_rsp_flags[0];
                        fpu_pending      <= 1'b0;
                        state            <= ST_Y0R_B;
                    end
                end

                ST_Y0R_B: begin
                    if (!fpu_pending) begin
                        if (fpu_req_ready) begin
                            fpu_pending <= 1'b1;
                        end
                    end else if (fpu_rsp_valid) begin
                        y0_re            <= fpu_rsp_result;
                        status_invalid   <= status_invalid | fpu_rsp_flags[4];
                        status_overflow  <= status_overflow | fpu_rsp_flags[2];
                        status_underflow <= status_underflow | fpu_rsp_flags[1];
                        status_inexact   <= status_inexact | fpu_rsp_flags[0];
                        fpu_pending      <= 1'b0;
                        state            <= ST_Y1R_A;
                    end
                end

                ST_Y1R_A: begin
                    if (!fpu_pending) begin
                        if (fpu_req_ready) begin
                            fpu_pending <= 1'b1;
                        end
                    end else if (fpu_rsp_valid) begin
                        tmp_re_q         <= fpu_rsp_result;
                        status_invalid   <= status_invalid | fpu_rsp_flags[4];
                        status_overflow  <= status_overflow | fpu_rsp_flags[2];
                        status_underflow <= status_underflow | fpu_rsp_flags[1];
                        status_inexact   <= status_inexact | fpu_rsp_flags[0];
                        fpu_pending      <= 1'b0;
                        state            <= ST_Y1R_B;
                    end
                end

                ST_Y1R_B: begin
                    if (!fpu_pending) begin
                        if (fpu_req_ready) begin
                            fpu_pending <= 1'b1;
                        end
                    end else if (fpu_rsp_valid) begin
                        y1_re            <= fpu_rsp_result;
                        status_invalid   <= status_invalid | fpu_rsp_flags[4];
                        status_overflow  <= status_overflow | fpu_rsp_flags[2];
                        status_underflow <= status_underflow | fpu_rsp_flags[1];
                        status_inexact   <= status_inexact | fpu_rsp_flags[0];
                        fpu_pending      <= 1'b0;
                        state            <= ST_Y0I_A;
                    end
                end

                ST_Y0I_A: begin
                    if (!fpu_pending) begin
                        if (fpu_req_ready) begin
                            fpu_pending <= 1'b1;
                        end
                    end else if (fpu_rsp_valid) begin
                        tmp_im_q         <= fpu_rsp_result;
                        status_invalid   <= status_invalid | fpu_rsp_flags[4];
                        status_overflow  <= status_overflow | fpu_rsp_flags[2];
                        status_underflow <= status_underflow | fpu_rsp_flags[1];
                        status_inexact   <= status_inexact | fpu_rsp_flags[0];
                        fpu_pending      <= 1'b0;
                        state            <= ST_Y0I_B;
                    end
                end

                ST_Y0I_B: begin
                    if (!fpu_pending) begin
                        if (fpu_req_ready) begin
                            fpu_pending <= 1'b1;
                        end
                    end else if (fpu_rsp_valid) begin
                        y0_im            <= fpu_rsp_result;
                        status_invalid   <= status_invalid | fpu_rsp_flags[4];
                        status_overflow  <= status_overflow | fpu_rsp_flags[2];
                        status_underflow <= status_underflow | fpu_rsp_flags[1];
                        status_inexact   <= status_inexact | fpu_rsp_flags[0];
                        fpu_pending      <= 1'b0;
                        state            <= ST_Y1I_A;
                    end
                end

                ST_Y1I_A: begin
                    if (!fpu_pending) begin
                        if (fpu_req_ready) begin
                            fpu_pending <= 1'b1;
                        end
                    end else if (fpu_rsp_valid) begin
                        tmp_im_q         <= fpu_rsp_result;
                        status_invalid   <= status_invalid | fpu_rsp_flags[4];
                        status_overflow  <= status_overflow | fpu_rsp_flags[2];
                        status_underflow <= status_underflow | fpu_rsp_flags[1];
                        status_inexact   <= status_inexact | fpu_rsp_flags[0];
                        fpu_pending      <= 1'b0;
                        state            <= ST_Y1I_B;
                    end
                end

                ST_Y1I_B: begin
                    if (!fpu_pending) begin
                        if (fpu_req_ready) begin
                            fpu_pending <= 1'b1;
                        end
                    end else if (fpu_rsp_valid) begin
                        y1_im            <= fpu_rsp_result;
                        status_invalid   <= status_invalid | fpu_rsp_flags[4];
                        status_overflow  <= status_overflow | fpu_rsp_flags[2];
                        status_underflow <= status_underflow | fpu_rsp_flags[1];
                        status_inexact   <= status_inexact | fpu_rsp_flags[0];
                        fpu_pending      <= 1'b0;
                        state            <= ST_DONE;
                    end
                end

                ST_DONE: begin
                    if (out_ready) begin
                        state <= ST_IDLE;
                    end
                end

                default: begin
                    state       <= ST_IDLE;
                    fpu_pending <= 1'b0;
                end
            endcase
        end
    end

endmodule
