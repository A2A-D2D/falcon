`timescale 1ns/1ps

module tb_unified_fpu_final;

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

    // Debug output - only print state changes
    reg [3:0] prev_state;
    always @(posedge clk) begin
        if (dut.state != prev_state) begin
            $display("T=%0t state=%0d", $time, dut.state);
            prev_state <= dut.state;
        end
        if (rsp_valid) begin
            $display("T=%0t RSP VALID: y0=%h+%hi, y1=%h+%hi", 
                     $time, rsp_y0_re, rsp_y0_im, rsp_y1_re, rsp_y1_im);
        end
    end

    initial begin
        rst_n = 0;
        req_valid = 0;
        rsp_ready = 1;
        prev_state = 0;
        {req_a_re, req_a_im, req_b_re, req_b_im} = 256'd0;
        {req_w_re, req_w_im} = 128'd0;

        #30 rst_n = 1;
        #20;

        $display("=== Starting Butterfly Test ===");
        $display("a=1+2i, b=3+4i, w=0.5+0.5i");
        
        req_valid = 1;
        req_a_re = 64'h3FF0000000000000;  // 1.0
        req_a_im = 64'h4000000000000000;  // 2.0
        req_b_re = 64'h4008000000000000;  // 3.0
        req_b_im = 64'h4010000000000000;  // 4.0
        req_w_re = 64'h3FE0000000000000;  // 0.5
        req_w_im = 64'h3FE0000000000000;  // 0.5

        @(posedge clk);
        req_valid = 0;

        wait(rsp_valid);
        @(posedge clk);

        $display("=== Results ===");
        $display("y0 = %h + %hi", rsp_y0_re, rsp_y0_im);
        $display("y1 = %h + %hi", rsp_y1_re, rsp_y1_im);
        
        // Expected: 
        // b*w = (3+4i)*(0.5+0.5i) = 1.5+1.5i+2i-2 = -0.5+3.5i
        // y0 = a + b*w = (1+2i) + (-0.5+3.5i) = 0.5+5.5i
        // y1 = a - b*w = (1+2i) - (-0.5+3.5i) = 1.5-1.5i
        
        $display("=== Test Complete ===");
        #100;
        $finish;
    end

    // Timeout
    initial begin
        #5000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
