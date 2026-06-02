`timescale 1ns/1ps
// Module: falcon_f64_complex_bfly_extfpu
// Purpose: Same as falcon_f64_complex_bfly but FPU is external (shared).
// Exposes FPU request/response ports instead of internal FPU instance.
// Used by multi-BFU EXUs to share the top-level FPU.

module falcon_f64_complex_bfly_extfpu (
    input         clk, rst_n,
    input         in_valid,        output in_ready,
    input  [63:0] a_re, a_im, b_re, b_im, w_re, w_im,
    output reg    out_valid,
    input         out_ready,
    output reg [63:0] y0_re, y0_im, y1_re, y1_im,
    output reg    status_invalid, status_overflow, status_underflow, status_inexact,
    output        busy,

    // External FPU interface
    output reg    fpu_req_valid,
    input         fpu_req_ready,
    output reg [3:0]  fpu_req_op,
    output reg [63:0] fpu_req_a, fpu_req_b, fpu_req_c,
    input         fpu_rsp_valid,
    input  [63:0] fpu_rsp_result,
    input  [4:0]  fpu_rsp_flags
);

    localparam [3:0] ST_IDLE=0, ST_Y0R_A=1, ST_Y0R_B=2, ST_Y1R_A=3, ST_Y1R_B=4,
                     ST_Y0I_A=5, ST_Y0I_B=6, ST_Y1I_A=7, ST_Y1I_B=8, ST_DONE=9;
    localparam [3:0] OP_FMADD=3, OP_FNMADD=6;

    reg [3:0] state; reg fpu_pending;
    reg [63:0] a_re_q, a_im_q, b_re_q, b_im_q, w_re_q, w_im_q;
    reg [63:0] tmp_re_q, tmp_im_q;

    assign in_ready=(state==ST_IDLE); assign busy=(state!=ST_IDLE);

    always @(*) begin
        fpu_req_valid=0; fpu_req_op=OP_FMADD; fpu_req_a=0; fpu_req_b=0; fpu_req_c=0; out_valid=0;
        case(state)
            ST_Y0R_A: if(!fpu_pending) begin fpu_req_valid=1; fpu_req_op=OP_FMADD;  fpu_req_a=b_re_q; fpu_req_b=w_re_q; fpu_req_c=a_re_q; end
            ST_Y0R_B: if(!fpu_pending) begin fpu_req_valid=1; fpu_req_op=OP_FNMADD; fpu_req_a=b_im_q; fpu_req_b=w_im_q; fpu_req_c=tmp_re_q; end
            ST_Y1R_A: if(!fpu_pending) begin fpu_req_valid=1; fpu_req_op=OP_FNMADD; fpu_req_a=b_re_q; fpu_req_b=w_re_q; fpu_req_c=a_re_q; end
            ST_Y1R_B: if(!fpu_pending) begin fpu_req_valid=1; fpu_req_op=OP_FMADD;  fpu_req_a=b_im_q; fpu_req_b=w_im_q; fpu_req_c=tmp_re_q; end
            ST_Y0I_A: if(!fpu_pending) begin fpu_req_valid=1; fpu_req_op=OP_FMADD;  fpu_req_a=b_re_q; fpu_req_b=w_im_q; fpu_req_c=a_im_q; end
            ST_Y0I_B: if(!fpu_pending) begin fpu_req_valid=1; fpu_req_op=OP_FMADD;  fpu_req_a=b_im_q; fpu_req_b=w_re_q; fpu_req_c=tmp_im_q; end
            ST_Y1I_A: if(!fpu_pending) begin fpu_req_valid=1; fpu_req_op=OP_FNMADD; fpu_req_a=b_re_q; fpu_req_b=w_im_q; fpu_req_c=a_im_q; end
            ST_Y1I_B: if(!fpu_pending) begin fpu_req_valid=1; fpu_req_op=OP_FNMADD; fpu_req_a=b_im_q; fpu_req_b=w_re_q; fpu_req_c=tmp_im_q; end
            ST_DONE: out_valid=1;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=ST_IDLE; fpu_pending<=0; a_re_q<=0; a_im_q<=0; b_re_q<=0; b_im_q<=0; w_re_q<=0; w_im_q<=0;
            tmp_re_q<=0; tmp_im_q<=0; y0_re<=0; y0_im<=0; y1_re<=0; y1_im<=0;
            status_invalid<=0; status_overflow<=0; status_underflow<=0; status_inexact<=0;
        end else case(state)
            ST_IDLE: if(in_valid) begin
                a_re_q<=a_re; a_im_q<=a_im; b_re_q<=b_re; b_im_q<=b_im; w_re_q<=w_re; w_im_q<=w_im;
                tmp_re_q<=0; tmp_im_q<=0; fpu_pending<=0; state<=ST_Y0R_A;
            end
            ST_Y0R_A: if(!fpu_pending) begin if(fpu_req_ready) fpu_pending<=1; end
            else if(fpu_rsp_valid) begin tmp_re_q<=fpu_rsp_result; fpu_pending<=0; state<=ST_Y0R_B; end
            ST_Y0R_B: if(!fpu_pending) begin if(fpu_req_ready) fpu_pending<=1; end
            else if(fpu_rsp_valid) begin y0_re<=fpu_rsp_result; fpu_pending<=0; state<=ST_Y1R_A; end
            ST_Y1R_A: if(!fpu_pending) begin if(fpu_req_ready) fpu_pending<=1; end
            else if(fpu_rsp_valid) begin tmp_re_q<=fpu_rsp_result; fpu_pending<=0; state<=ST_Y1R_B; end
            ST_Y1R_B: if(!fpu_pending) begin if(fpu_req_ready) fpu_pending<=1; end
            else if(fpu_rsp_valid) begin y1_re<=fpu_rsp_result; fpu_pending<=0; state<=ST_Y0I_A; end
            ST_Y0I_A: if(!fpu_pending) begin if(fpu_req_ready) fpu_pending<=1; end
            else if(fpu_rsp_valid) begin tmp_im_q<=fpu_rsp_result; fpu_pending<=0; state<=ST_Y0I_B; end
            ST_Y0I_B: if(!fpu_pending) begin if(fpu_req_ready) fpu_pending<=1; end
            else if(fpu_rsp_valid) begin y0_im<=fpu_rsp_result; fpu_pending<=0; state<=ST_Y1I_A; end
            ST_Y1I_A: if(!fpu_pending) begin if(fpu_req_ready) fpu_pending<=1; end
            else if(fpu_rsp_valid) begin tmp_im_q<=fpu_rsp_result; fpu_pending<=0; state<=ST_Y1I_B; end
            ST_Y1I_B: if(!fpu_pending) begin if(fpu_req_ready) fpu_pending<=1; end
            else if(fpu_rsp_valid) begin y1_im<=fpu_rsp_result; fpu_pending<=0; state<=ST_DONE; end
            ST_DONE: if(out_ready) state<=ST_IDLE;
            default: state<=ST_IDLE;
        endcase
    end
endmodule
