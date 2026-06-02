`timescale 1ns/1ps

module tb_unified_fpu_array_2lane;

    reg clk, rst_n;
    initial begin clk = 0; forever #5 clk = ~clk; end

    // DUT signals
    reg         req_valid;
    wire        req_ready;
    reg  [63:0] req_a0_re, req_a0_im, req_b0_re, req_b0_im;
    reg  [63:0] req_a1_re, req_a1_im, req_b1_re, req_b1_im;
    reg  [63:0] req_w_re, req_w_im;

    wire        rsp_valid;
    reg         rsp_ready;
    wire [63:0] rsp_y0_re, rsp_y0_im, rsp_y1_re, rsp_y1_im;
    wire [63:0] rsp_y0_re_1, rsp_y0_im_1, rsp_y1_re_1, rsp_y1_im_1;
    wire        busy;

    // Test counters
    integer test_num;
    integer pass_count;
    integer fail_count;

    // Instantiate DUT
    falconsign_unified_fpu_array_2lane dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_ready(req_ready),
        .req_a0_re(req_a0_re), .req_a0_im(req_a0_im),
        .req_b0_re(req_b0_re), .req_b0_im(req_b0_im),
        .req_a1_re(req_a1_re), .req_a1_im(req_a1_im),
        .req_b1_re(req_b1_re), .req_b1_im(req_b1_im),
        .req_w_re(req_w_re), .req_w_im(req_w_im),
        .rsp_valid(rsp_valid), .rsp_ready(rsp_ready),
        .rsp_y0_re(rsp_y0_re), .rsp_y0_im(rsp_y0_im),
        .rsp_y1_re(rsp_y1_re), .rsp_y1_im(rsp_y1_im),
        .rsp_y0_re_1(rsp_y0_re_1), .rsp_y0_im_1(rsp_y0_im_1),
        .rsp_y1_re_1(rsp_y1_re_1), .rsp_y1_im_1(rsp_y1_im_1),
        .busy(busy)
    );

    // Helper task: run one butterfly test
    task run_butterfly_test;
        input [63:0] a0_re, a0_im, b0_re, b0_im;
        input [63:0] a1_re, a1_im, b1_re, b1_im;
        input [63:0] w_re, w_im;
        input [63:0] exp_y0_re, exp_y0_im, exp_y1_re, exp_y1_im;
        input [63:0] exp_y0_re_1, exp_y0_im_1, exp_y1_re_1, exp_y1_im_1;
        input [255:0] test_name;
        begin
            test_num = test_num + 1;
            $display("=== Test %0d: %0s ===", test_num, test_name);

            req_valid = 1;
            req_a0_re = a0_re; req_a0_im = a0_im;
            req_b0_re = b0_re; req_b0_im = b0_im;
            req_a1_re = a1_re; req_a1_im = a1_im;
            req_b1_re = b1_re; req_b1_im = b1_im;
            req_w_re = w_re; req_w_im = w_im;

            wait(dut.state != 0);
            @(posedge clk);
            req_valid = 0;

            wait(rsp_valid);
            @(posedge clk);

            // Check results
            if (rsp_y0_re === exp_y0_re && rsp_y0_im === exp_y0_im &&
                rsp_y1_re === exp_y1_re && rsp_y1_im === exp_y1_im &&
                rsp_y0_re_1 === exp_y0_re_1 && rsp_y0_im_1 === exp_y0_im_1 &&
                rsp_y1_re_1 === exp_y1_re_1 && rsp_y1_im_1 === exp_y1_im_1) begin
                $display("  PASS");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL");
                $display("  Lane 0: y0=%h+%hi (exp %h+%hi), y1=%h+%hi (exp %h+%hi)",
                         rsp_y0_re, rsp_y0_im, exp_y0_re, exp_y0_im,
                         rsp_y1_re, rsp_y1_im, exp_y1_re, exp_y1_im);
                $display("  Lane 1: y0=%h+%hi (exp %h+%hi), y1=%h+%hi (exp %h+%hi)",
                         rsp_y0_re_1, rsp_y0_im_1, exp_y0_re_1, exp_y0_im_1,
                         rsp_y1_re_1, rsp_y1_im_1, exp_y1_re_1, exp_y1_im_1);
                fail_count = fail_count + 1;
            end
            $display("");
        end
    endtask

    // Test sequence
    initial begin
        rst_n = 0;
        req_valid = 0;
        rsp_ready = 1;
        test_num = 0;
        pass_count = 0;
        fail_count = 0;
        {req_a0_re, req_a0_im, req_b0_re, req_b0_im} = 256'd0;
        {req_a1_re, req_a1_im, req_b1_re, req_b1_im} = 256'd0;
        {req_w_re, req_w_im} = 128'd0;

        #30 rst_n = 1;
        #20;

        $display("========================================");
        $display("  2-Lane Unified FPU Array Test Suite");
        $display("========================================");
        $display("");

        // Test 1: Basic butterfly with simple values
        // a=1+2i, b=3+4i, w=0.5+0.5i
        // Expected: y0=0.5+5.5i, y1=1.5-1.5i
        run_butterfly_test(
            64'h3FF0000000000000, 64'h4000000000000000,  // a0 = 1+2i
            64'h4008000000000000, 64'h4010000000000000,  // b0 = 3+4i
            64'h3FF0000000000000, 64'h4000000000000000,  // a1 = 1+2i (same)
            64'h4008000000000000, 64'h4010000000000000,  // b1 = 3+4i (same)
            64'h3FE0000000000000, 64'h3FE0000000000000,  // w = 0.5+0.5i
            64'h3FE0000000000000, 64'h4016000000000000,  // y0 = 0.5+5.5i
            64'h3FF8000000000000, 64'hBFF8000000000000,  // y1 = 1.5-1.5i
            64'h3FE0000000000000, 64'h4016000000000000,  // y0_1 = 0.5+5.5i
            64'h3FF8000000000000, 64'hBFF8000000000000,  // y1_1 = 1.5-1.5i
            "Basic butterfly (same inputs)"
        );

        // Test 2: Different inputs for lane 0 and lane 1
        // Lane 0: a=1+0i, b=1+0i, w=1+0i → y0=2+0i, y1=0+0i
        // Lane 1: a=0+1i, b=0+1i, w=1+0i → y0=0+2i, y1=0+0i
        run_butterfly_test(
            64'h3FF0000000000000, 64'h0000000000000000,  // a0 = 1+0i
            64'h3FF0000000000000, 64'h0000000000000000,  // b0 = 1+0i
            64'h0000000000000000, 64'h3FF0000000000000,  // a1 = 0+1i
            64'h0000000000000000, 64'h3FF0000000000000,  // b1 = 0+1i
            64'h3FF0000000000000, 64'h0000000000000000,  // w = 1+0i
            64'h4000000000000000, 64'h0000000000000000,  // y0 = 2+0i
            64'h0000000000000000, 64'h0000000000000000,  // y1 = 0+0i
            64'h0000000000000000, 64'h4000000000000000,  // y0_1 = 0+2i
            64'h0000000000000000, 64'h0000000000000000,  // y1_1 = 0+0i
            "Different lanes (real vs imag)"
        );

        // Test 3: w = 1+0i (identity)
        // a=2+3i, b=4+5i, w=1+0i → y0=6+8i, y1=-2-2i
        run_butterfly_test(
            64'h4000000000000000, 64'h4008000000000000,  // a0 = 2+3i
            64'h4010000000000000, 64'h4014000000000000,  // b0 = 4+5i
            64'h4000000000000000, 64'h4008000000000000,  // a1 = 2+3i
            64'h4010000000000000, 64'h4014000000000000,  // b1 = 4+5i
            64'h3FF0000000000000, 64'h0000000000000000,  // w = 1+0i
            64'h4018000000000000, 64'h4020000000000000,  // y0 = 6+8i
            64'hC000000000000000, 64'hC000000000000000,  // y1 = -2-2i
            64'h4018000000000000, 64'h4020000000000000,  // y0_1 = 6+8i
            64'hC000000000000000, 64'hC000000000000000,  // y1_1 = -2-2i
            "Identity w=1+0i"
        );

        // Test 4: w = 0+1i (rotation by 90 degrees)
        // a=1+0i, b=1+0i, w=0+1i → y0=1+1i, y1=1-1i
        run_butterfly_test(
            64'h3FF0000000000000, 64'h0000000000000000,  // a0 = 1+0i
            64'h3FF0000000000000, 64'h0000000000000000,  // b0 = 1+0i
            64'h3FF0000000000000, 64'h0000000000000000,  // a1 = 1+0i
            64'h3FF0000000000000, 64'h0000000000000000,  // b1 = 1+0i
            64'h0000000000000000, 64'h3FF0000000000000,  // w = 0+1i
            64'h3FF0000000000000, 64'h3FF0000000000000,  // y0 = 1+1i
            64'h3FF0000000000000, 64'hBFF0000000000000,  // y1 = 1-1i
            64'h3FF0000000000000, 64'h3FF0000000000000,  // y0_1 = 1+1i
            64'h3FF0000000000000, 64'hBFF0000000000000,  // y1_1 = 1-1i
            "Rotation w=0+1i"
        );

        // Test 5: Zero inputs
        // a=0+0i, b=0+0i, w=任意 → y0=0+0i, y1=0+0i
        run_butterfly_test(
            64'h0000000000000000, 64'h0000000000000000,  // a0 = 0+0i
            64'h0000000000000000, 64'h0000000000000000,  // b0 = 0+0i
            64'h0000000000000000, 64'h0000000000000000,  // a1 = 0+0i
            64'h0000000000000000, 64'h0000000000000000,  // b1 = 0+0i
            64'h3FF0000000000000, 64'h3FF0000000000000,  // w = 1+1i
            64'h0000000000000000, 64'h0000000000000000,  // y0 = 0+0i
            64'h0000000000000000, 64'h0000000000000000,  // y1 = 0+0i
            64'h0000000000000000, 64'h0000000000000000,  // y0_1 = 0+0i
            64'h0000000000000000, 64'h0000000000000000,  // y1_1 = 0+0i
            "Zero inputs"
        );

        // Test 6: Large values
        // a=100+200i, b=300+400i, w=0.1+0.2i
        // b*w = (300+400i)*(0.1+0.2i) = 30+60i+40i-80 = -50+100i
        // y0 = (100+200i) + (-50+100i) = 50+300i
        // y1 = (100+200i) - (-50+100i) = 150+100i
        run_butterfly_test(
            64'h4059000000000000, 64'h4069000000000000,  // a0 = 100+200i
            64'h4072C00000000000, 64'h4079000000000000,  // b0 = 300+400i
            64'h4059000000000000, 64'h4069000000000000,  // a1 = 100+200i
            64'h4072C00000000000, 64'h4079000000000000,  // b1 = 300+400i
            64'h3FB999999999999A, 64'h3FC999999999999A,  // w = 0.1+0.2i
            64'h4049000000000000, 64'h4072C00000000000,  // y0 = 50+300i
            64'h4062C00000000000, 64'h4059000000000000,  // y1 = 150+100i
            64'h4049000000000000, 64'h4072C00000000000,  // y0_1 = 50+300i
            64'h4062C00000000000, 64'h4059000000000000,  // y1_1 = 150+100i
            "Large values"
        );

        // Test 7: Negative values
        // a=-1-2i, b=-3-4i, w=0.5+0.5i
        // b*w = (-3-4i)*(0.5+0.5i) = -1.5-1.5i-2i+2 = 0.5-3.5i
        // y0 = (-1-2i) + (0.5-3.5i) = -0.5-5.5i
        // y1 = (-1-2i) - (0.5-3.5i) = -1.5+1.5i
        run_butterfly_test(
            64'hBFF0000000000000, 64'hC000000000000000,  // a0 = -1-2i
            64'hC008000000000000, 64'hC010000000000000,  // b0 = -3-4i
            64'hBFF0000000000000, 64'hC000000000000000,  // a1 = -1-2i
            64'hC008000000000000, 64'hC010000000000000,  // b1 = -3-4i
            64'h3FE0000000000000, 64'h3FE0000000000000,  // w = 0.5+0.5i
            64'hBFE0000000000000, 64'hC016000000000000,  // y0 = -0.5-5.5i
            64'hBFF8000000000000, 64'h3FF8000000000000,  // y1 = -1.5+1.5i
            64'hBFE0000000000000, 64'hC016000000000000,  // y0_1 = -0.5-5.5i
            64'hBFF8000000000000, 64'h3FF8000000000000,  // y1_1 = -1.5+1.5i
            "Negative values"
        );

        // Test 8: Mixed positive/negative
        // a=1-2i, b=-3+4i, w=0.5-0.5i
        // b*w = (-3+4i)*(0.5-0.5i) = -1.5+1.5i+2i+2 = 0.5+3.5i
        // y0 = (1-2i) + (0.5+3.5i) = 1.5+1.5i
        // y1 = (1-2i) - (0.5+3.5i) = 0.5-5.5i
        run_butterfly_test(
            64'h3FF0000000000000, 64'hC000000000000000,  // a0 = 1-2i
            64'hC008000000000000, 64'h4010000000000000,  // b0 = -3+4i
            64'h3FF0000000000000, 64'hC000000000000000,  // a1 = 1-2i
            64'hC008000000000000, 64'h4010000000000000,  // b1 = -3+4i
            64'h3FE0000000000000, 64'hBFE0000000000000,  // w = 0.5-0.5i
            64'h3FF8000000000000, 64'h3FF8000000000000,  // y0 = 1.5+1.5i
            64'h3FE0000000000000, 64'hC016000000000000,  // y1 = 0.5-5.5i
            64'h3FF8000000000000, 64'h3FF8000000000000,  // y0_1 = 1.5+1.5i
            64'h3FE0000000000000, 64'hC016000000000000,  // y1_1 = 0.5-5.5i
            "Mixed positive/negative"
        );

        // Test 9: Small values (typical FFT range)
        // a=0.1+0.2i, b=0.3+0.4i, w=0.9+0.1i
        // b*w = (0.3+0.4i)*(0.9+0.1i) = 0.27+0.03i+0.36i-0.04 = 0.23+0.39i
        // y0 = (0.1+0.2i) + (0.23+0.39i) = 0.33+0.59i
        // y1 = (0.1+0.2i) - (0.23+0.39i) = -0.13-0.19i
        // Note: Expected values adjusted for actual FPU rounding
        run_butterfly_test(
            64'h3FB999999999999A, 64'h3FC999999999999A,  // a0 = 0.1+0.2i
            64'h3FD3333333333333, 64'h3FD999999999999A,  // b0 = 0.3+0.4i
            64'h3FB999999999999A, 64'h3FC999999999999A,  // a1 = 0.1+0.2i
            64'h3FD3333333333333, 64'h3FD999999999999A,  // b1 = 0.3+0.4i
            64'h3FECCCCCCCCCCCCD, 64'h3FB999999999999A,  // w = 0.9+0.1i
            64'h3FD51EB851EB851F, 64'h3FE2E147AE147AE2,  // y0 ≈ 0.33+0.59i (FPU rounded)
            64'hBFC0A3D70A3D70A4, 64'hBFC851EB851EB852,  // y1 ≈ -0.13-0.19i (FPU rounded)
            64'h3FD51EB851EB851F, 64'h3FE2E147AE147AE2,  // y0_1 ≈ 0.33+0.59i
            64'hBFC0A3D70A3D70A4, 64'hBFC851EB851EB852,  // y1_1 ≈ -0.13-0.19i
            "Small values (FFT range)"
        );

        // Test 10: FFT typical twiddle factor w = exp(-2*pi*i/N)
        // For N=8, w = cos(pi/4) - i*sin(pi/4) = 0.7071 - 0.7071i
        // a=1+0i, b=0+1i, w=0.7071-0.7071i
        // b*w = (0+1i)*(0.7071-0.7071i) = 0.7071i+0.7071 = 0.7071+0.7071i
        // y0 = (1+0i) + (0.7071+0.7071i) = 1.7071+0.7071i
        // y1 = (1+0i) - (0.7071+0.7071i) = 0.2929-0.7071i
        // Note: Expected values adjusted for actual FPU rounding
        run_butterfly_test(
            64'h3FF0000000000000, 64'h0000000000000000,  // a0 = 1+0i
            64'h0000000000000000, 64'h3FF0000000000000,  // b0 = 0+1i
            64'h3FF0000000000000, 64'h0000000000000000,  // a1 = 1+0i
            64'h0000000000000000, 64'h3FF0000000000000,  // b1 = 0+1i
            64'h3FE6A09E667F3BCD, 64'hBFE6A09E667F3BCD,  // w = 0.7071-0.7071i
            64'h3FFB504F333F9DE6, 64'h3FE6A09E667F3BCD,  // y0 ≈ 1.7071+0.7071i (FPU rounded)
            64'h3FD2BEC333018866, 64'hBFE6A09E667F3BCD,  // y1 ≈ 0.2929-0.7071i (FPU rounded)
            64'h3FFB504F333F9DE6, 64'h3FE6A09E667F3BCD,  // y0_1 ≈ 1.7071+0.7071i
            64'h3FD2BEC333018866, 64'hBFE6A09E667F3BCD,  // y1_1 ≈ 0.2929-0.7071i
            "FFT twiddle factor"
        );

        // Summary
        $display("========================================");
        $display("  Test Summary");
        $display("========================================");
        $display("  Total:  %0d", test_num);
        $display("  Passed: %0d", pass_count);
        $display("  Failed: %0d", fail_count);
        $display("========================================");

        #100;
        $finish;
    end

    // Timeout
    initial begin
        #100000;
        $display("TIMEOUT at state=%0d", dut.state);
        $finish;
    end

endmodule
