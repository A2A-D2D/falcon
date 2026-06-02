`timescale 1ns/1ps
// Module: falcon_f64_complex_bfly4
// Purpose: 4-lane vector complex FFT butterfly. Each lane computes
//   y0 = a + b*w,  y1 = a - b*w
// independently with its own FPU. All 4 lanes share a single twiddle
// factor (vector reuse, ~74.6% twiddle access reduction per FalconSign).
//
// Ports (lane k, k = 0..3):
//   ak_re/im, bk_re/im  — complex inputs for lane k
//   w_re/im              — shared twiddle factor (same across all lanes)
//   y0k_re/im, y1k_re/im — complex outputs for lane k

module falcon_f64_complex_bfly4 (
    input         clk,
    input         rst_n,
    input         in_valid,
    output        in_ready,

    // Lane 0
    input  [63:0] a0_re, a0_im, b0_re, b0_im,
    // Lane 1
    input  [63:0] a1_re, a1_im, b1_re, b1_im,
    // Lane 2
    input  [63:0] a2_re, a2_im, b2_re, b2_im,
    // Lane 3
    input  [63:0] a3_re, a3_im, b3_re, b3_im,

    // Shared twiddle factor
    input  [63:0] w_re, w_im,

    output reg    out_valid,
    input         out_ready,

    // Lane 0 results
    output wire [63:0] y00_re, y00_im, y10_re, y10_im,
    // Lane 1 results
    output wire [63:0] y01_re, y01_im, y11_re, y11_im,
    // Lane 2 results
    output wire [63:0] y02_re, y02_im, y12_re, y12_im,
    // Lane 3 results
    output wire [63:0] y03_re, y03_im, y13_re, y13_im,

    output wire   busy
);

    // ─── 4 lanes of complex BFU ───
    wire [3:0] bfy_in_ready;
    wire [3:0] bfy_out_valid;
    wire [3:0] bfy_busy;
    reg  [3:0] bfy_out_ready_latched;
    reg        all_lanes_done_r;

    // Lane 0
    falcon_f64_complex_bfly u_bfy0 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(bfy_in_ready[0]),
        .a_re(a0_re), .a_im(a0_im), .b_re(b0_re), .b_im(b0_im),
        .w_re(w_re),  .w_im(w_im),
        .out_valid(bfy_out_valid[0]),
        .out_ready(bfy_out_ready_latched[0]),
        .y0_re(y00_re), .y0_im(y00_im),
        .y1_re(y10_re), .y1_im(y10_im),
        .status_invalid(), .status_overflow(),
        .status_underflow(), .status_inexact(),
        .busy(bfy_busy[0])
    );

    // Lane 1
    falcon_f64_complex_bfly u_bfy1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(bfy_in_ready[1]),
        .a_re(a1_re), .a_im(a1_im), .b_re(b1_re), .b_im(b1_im),
        .w_re(w_re),  .w_im(w_im),
        .out_valid(bfy_out_valid[1]),
        .out_ready(bfy_out_ready_latched[1]),
        .y0_re(y01_re), .y0_im(y01_im),
        .y1_re(y11_re), .y1_im(y11_im),
        .status_invalid(), .status_overflow(),
        .status_underflow(), .status_inexact(),
        .busy(bfy_busy[1])
    );

    // Lane 2
    falcon_f64_complex_bfly u_bfy2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(bfy_in_ready[2]),
        .a_re(a2_re), .a_im(a2_im), .b_re(b2_re), .b_im(b2_im),
        .w_re(w_re),  .w_im(w_im),
        .out_valid(bfy_out_valid[2]),
        .out_ready(bfy_out_ready_latched[2]),
        .y0_re(y02_re), .y0_im(y02_im),
        .y1_re(y12_re), .y1_im(y12_im),
        .status_invalid(), .status_overflow(),
        .status_underflow(), .status_inexact(),
        .busy(bfy_busy[2])
    );

    // Lane 3
    falcon_f64_complex_bfly u_bfy3 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(bfy_in_ready[3]),
        .a_re(a3_re), .a_im(a3_im), .b_re(b3_re), .b_im(b3_im),
        .w_re(w_re),  .w_im(w_im),
        .out_valid(bfy_out_valid[3]),
        .out_ready(bfy_out_ready_latched[3]),
        .y0_re(y03_re), .y0_im(y03_im),
        .y1_re(y13_re), .y1_im(y13_im),
        .status_invalid(), .status_overflow(),
        .status_underflow(), .status_inexact(),
        .busy(bfy_busy[3])
    );

    // ─── Handshake logic ───
    assign in_ready = &bfy_in_ready;  // all 4 lanes ready
    assign busy     = |bfy_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bfy_out_ready_latched <= 4'b0000;
            out_valid             <= 1'b0;
            all_lanes_done_r      <= 1'b0;
        end else begin
            // Accept output from each lane as it completes
            bfy_out_ready_latched <= 4'b0000;
            if (bfy_out_valid[0]) bfy_out_ready_latched[0] <= 1'b1;
            if (bfy_out_valid[1]) bfy_out_ready_latched[1] <= 1'b1;
            if (bfy_out_valid[2]) bfy_out_ready_latched[2] <= 1'b1;
            if (bfy_out_valid[3]) bfy_out_ready_latched[3] <= 1'b1;

            // out_valid when all 4 lanes have produced results
            if (&bfy_out_valid) begin
                out_valid <= 1'b1;
            end else if (out_valid && out_ready) begin
                out_valid <= 1'b0;
            end
        end
    end

endmodule
