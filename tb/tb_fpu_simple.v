`timescale 1ns/1ps

module tb_fpu_simple;

    reg clk, rst_n;
    initial begin clk = 0; forever #5 clk = ~clk; end

    // FPU signals
    reg         req_valid;
    wire        req_ready;
    reg  [3:0]  req_op;
    reg  [63:0] req_a, req_b, req_c;
    wire        rsp_valid;
    wire [63:0] rsp_result;

    // Instantiate FPU
    falcon_fp_fpu u_fpu (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_ready(req_ready),
        .req_op(req_op), .req_a(req_a), .req_b(req_b), .req_c(req_c),
        .req_fmt(2'd0), .req_rm(3'd0), .req_fcvt_op(2'd0),
        .rsp_valid(rsp_valid), .rsp_ready(1'b1), .rsp_result(rsp_result),
        .rsp_flags(), .busy()
    );

    // Test sequence
    initial begin
        rst_n = 0;
        req_valid = 0;
        req_op = 0;
        {req_a, req_b, req_c} = 192'd0;

        #30 rst_n = 1;
        #20;

        // Test: 1.0 + 2.0 = 3.0
        $display("=== Test: 1.0 + 2.0 ===");
        $display("req_ready = %b", req_ready);
        
        req_valid = 1;
        req_op = 4'd0;  // FADD
        req_a = 64'h3FF0000000000000;  // 1.0
        req_b = 64'h4000000000000000;  // 2.0

        $display("Waiting for response...");
        
        wait(rsp_valid);
        @(posedge clk);
        
        $display("Result: %h", rsp_result);
        $display("Expected: 4008000000000000 (3.0)");
        
        #100;
        $display("=== Test Complete ===");
        $finish;
    end

    // Debug output
    always @(posedge clk) begin
        $display("T=%0t req_v=%b req_r=%b rsp_v=%b", 
                 $time, req_valid, req_ready, rsp_valid);
    end

    // Timeout
    initial begin
        #1000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
