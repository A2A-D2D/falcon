`timescale 1ns/1ps
// Module: falcon_f64_complex_bfly2_extfpu
// Purpose: 2-lane BFU sharing a single external FPU. Lanes are processed
// sequentially (lane 0 first, then lane 1) to ensure correct FPU response
// routing. Throughput = same as 1-BFU, but uses the shared FPU for bit-exact
// results with the rest of the pipeline.

module falcon_f64_complex_bfly2_extfpu (
    input         clk, rst_n,
    input         in_valid,        output in_ready,
    input  [63:0] a0_re, a0_im, b0_re, b0_im,
    input  [63:0] a1_re, a1_im, b1_re, b1_im,
    input  [63:0] w_re, w_im,
    output reg    out_valid,
    input         out_ready,
    output wire [63:0] y00_re, y00_im, y10_re, y10_im,
    output wire [63:0] y01_re, y01_im, y11_re, y11_im,
    output        busy,

    // External FPU (shared, single-issue)
    output        fpu_req_valid,
    input         fpu_req_ready,
    output [3:0]  fpu_req_op,
    output [63:0] fpu_req_a, fpu_req_b, fpu_req_c,
    input         fpu_rsp_valid,
    input  [63:0] fpu_rsp_result,
    input  [4:0]  fpu_rsp_flags
);

    // Lane 0 BFU (active first)
    wire       ln0_in_valid, ln0_out_valid;
    wire [63:0] ln0_fpu_req_a, ln0_fpu_req_b, ln0_fpu_req_c;
    wire [3:0] ln0_fpu_req_op;
    wire       ln0_fpu_req_valid, ln0_fpu_req_ready;
    wire       ln0_fpu_rsp_valid;
    wire [63:0] ln0_fpu_rsp_result;
    wire [4:0] ln0_fpu_rsp_flags;

    falcon_f64_complex_bfly_extfpu u0 (
        .clk,.rst_n,.in_valid(ln0_in_valid),.in_ready(),
        .a_re(a0_re),.a_im(a0_im),.b_re(b0_re),.b_im(b0_im),.w_re,.w_im,
        .out_valid(ln0_out_valid),.out_ready(1'b1),
        .y0_re(y00_re),.y0_im(y00_im),.y1_re(y10_re),.y1_im(y10_im),
        .status_invalid(),.status_overflow(),.status_underflow(),.status_inexact(),
        .busy(),
        .fpu_req_valid(ln0_fpu_req_valid),.fpu_req_ready(ln0_fpu_req_ready),
        .fpu_req_op(ln0_fpu_req_op),.fpu_req_a(ln0_fpu_req_a),.fpu_req_b(ln0_fpu_req_b),.fpu_req_c(ln0_fpu_req_c),
        .fpu_rsp_valid(ln0_fpu_rsp_valid),.fpu_rsp_result(ln0_fpu_rsp_result),.fpu_rsp_flags(ln0_fpu_rsp_flags)
    );

    // Lane 1 BFU (active after lane 0 finishes)
    wire       ln1_in_valid, ln1_out_valid;
    wire [63:0] ln1_fpu_req_a, ln1_fpu_req_b, ln1_fpu_req_c;
    wire [3:0] ln1_fpu_req_op;
    wire       ln1_fpu_req_valid, ln1_fpu_req_ready;
    wire       ln1_fpu_rsp_valid;
    wire [63:0] ln1_fpu_rsp_result;
    wire [4:0] ln1_fpu_rsp_flags;

    falcon_f64_complex_bfly_extfpu u1 (
        .clk,.rst_n,.in_valid(ln1_in_valid),.in_ready(),
        .a_re(a1_re),.a_im(a1_im),.b_re(b1_re),.b_im(b1_im),.w_re,.w_im,
        .out_valid(ln1_out_valid),.out_ready(1'b1),
        .y0_re(y01_re),.y0_im(y01_im),.y1_re(y11_re),.y1_im(y11_im),
        .status_invalid(),.status_overflow(),.status_underflow(),.status_inexact(),
        .busy(),
        .fpu_req_valid(ln1_fpu_req_valid),.fpu_req_ready(ln1_fpu_req_ready),
        .fpu_req_op(ln1_fpu_req_op),.fpu_req_a(ln1_fpu_req_a),.fpu_req_b(ln1_fpu_req_b),.fpu_req_c(ln1_fpu_req_c),
        .fpu_rsp_valid(ln1_fpu_rsp_valid),.fpu_rsp_result(ln1_fpu_rsp_result),.fpu_rsp_flags(ln1_fpu_rsp_flags)
    );

    // Sequential control: lane 0 first, then lane 1
    reg        lane1_pending;
    assign ln0_in_valid = in_valid;          // lane 0 starts immediately
    assign ln1_in_valid = lane1_pending;     // lane 1 starts after lane 0

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin lane1_pending<=0; out_valid<=0; end
        else begin
            if(ln0_out_valid && !lane1_pending) lane1_pending<=1;
            if(ln1_out_valid) begin lane1_pending<=0; out_valid<=1; end
            if(out_valid && out_ready) out_valid<=0;
        end
    end

    // FPU mux: lane 0 when active, lane 1 after
    assign fpu_req_valid   = lane1_pending ? ln1_fpu_req_valid   : ln0_fpu_req_valid;
    assign fpu_req_op      = lane1_pending ? ln1_fpu_req_op      : ln0_fpu_req_op;
    assign fpu_req_a       = lane1_pending ? ln1_fpu_req_a       : ln0_fpu_req_a;
    assign fpu_req_b       = lane1_pending ? ln1_fpu_req_b       : ln0_fpu_req_b;
    assign fpu_req_c       = lane1_pending ? ln1_fpu_req_c       : ln0_fpu_req_c;
    assign ln0_fpu_req_ready = (!lane1_pending) ? fpu_req_ready : 1'b0;
    assign ln1_fpu_req_ready = lane1_pending     ? fpu_req_ready : 1'b0;

    // FPU response: route to active lane
    assign ln0_fpu_rsp_valid  = (!lane1_pending) ? fpu_rsp_valid : 1'b0;
    assign ln1_fpu_rsp_valid  = lane1_pending     ? fpu_rsp_valid : 1'b0;
    assign ln0_fpu_rsp_result = fpu_rsp_result;
    assign ln1_fpu_rsp_result = fpu_rsp_result;
    assign ln0_fpu_rsp_flags  = fpu_rsp_flags;
    assign ln1_fpu_rsp_flags  = fpu_rsp_flags;

    assign in_ready = 1'b1;
    assign busy = lane1_pending || ln0_out_valid || ln1_out_valid;

endmodule
