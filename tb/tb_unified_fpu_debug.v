`timescale 1ns/1ps

module tb_unified_fpu_debug;

    reg clk, rst_n;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg         req_valid;
    wire        req_ready;
    reg  [63:0] req_a_re, req_a_im, req_b_re, req_b_im;
    reg  [63:0] req_w_re, req_w_im;

    wire        rsp_valid;
    reg         rsp_ready;
    wire [63:0] rsp_y0_re, rsp_y0_im, rsp_y1_re, rsp_y1_im;
    wire        busy;

    falconsign_unified_fpu_array dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_ready(req_ready),
        .req_a_re(req_a_re), .req_a_im(req_a_im),
        .req_b_re(req_b_re), .req_b_im(req_b_im),
        .req_w_re(req_w_re), .req_w_im(req_w_im),
        .rsp_valid(rsp_valid), .rsp_ready(rsp_ready),
        .rsp_y0_re(rsp_y0_re), .rsp_y0_im(rsp_y0_im),
        .rsp_y1_re(rsp_y1_re), .rsp_y1_im(rsp_y1_im),
        .busy(busy)
    );

    // Debug output
    always @(posedge clk) begin
        $display("T=%0t state=%0d fpu_req=%b fpu_rsp=%b rsp_valid=%b", 
                 $time, dut.state, dut.fpu_req_valid, dut.fpu_rsp_valid, rsp_valid);
    end

    initial begin
        rst_n = 0;
        req_valid = 0;
        rsp_ready = 1;
        {req_a_re, req_a_im, req_b_re, req_b_im} = 256'd0;
        {req_w_re, req_w_im} = 128'd0;

        #30 rst_n = 1;
        #20;

        $display("=== Starting Test ===");
        req_valid = 1;
        req_a_re = 64'h3FF0000000000000;  // 1.0
        req_a_im = 64'h4000000000000000;  // 2.0
        req_b_re = 64'h4008000000000000;  // 3.0
        req_b_im = 64'h4010000000000000;  // 4.0
        req_w_re = 64'h3FE0000000000000;  // 0.5
        req_w_im = 64'h3FE0000000000000;  // 0.5

        @(posedge clk);
        req_valid = 0;

        #200;
        $display("=== Test End ===");
        $finish;
    end

endmodule
