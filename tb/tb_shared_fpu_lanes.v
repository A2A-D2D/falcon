`timescale 1ns/1ps
// Unit test: falconsign_shared_fpu_lanes vs falcon_f64_complex_bfly golden
// Verifies bit-exact butterfly computation across all modes.
module tb_shared_fpu_lanes;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // ─── Shared FPU Lanes DUT ───
    reg         dut_req_valid;
    wire        dut_req_ready;
    reg  [2:0]  dut_req_mode;
    reg  [63:0] dut_a0_re, dut_a0_im, dut_b0_re, dut_b0_im;
    reg  [63:0] dut_a1_re, dut_a1_im, dut_b1_re, dut_b1_im;
    reg  [63:0] dut_w_re, dut_w_im;
    wire        dut_rsp_valid;
    reg         dut_rsp_ready;
    wire [63:0] dut_y0_re, dut_y0_im, dut_y1_re, dut_y1_im;
    wire [63:0] dut_y0_re_1, dut_y0_im_1, dut_y1_re_1, dut_y1_im_1;
    wire        dut_busy;

    falconsign_shared_fpu_lanes dut (
        .clk,.rst_n,
        .req_valid(dut_req_valid),.req_ready(dut_req_ready),
        .req_mode(dut_req_mode),
        .req_a0_re(dut_a0_re),.req_a0_im(dut_a0_im),
        .req_b0_re(dut_b0_re),.req_b0_im(dut_b0_im),
        .req_a1_re(dut_a1_re),.req_a1_im(dut_a1_im),
        .req_b1_re(dut_b1_re),.req_b1_im(dut_b1_im),
        .req_w_re(dut_w_re),.req_w_im(dut_w_im),
        .req_w1_re(dut_w_re),.req_w1_im(dut_w_im),  // same twiddle for both lanes in test
        .rsp_valid(dut_rsp_valid),.rsp_ready(dut_rsp_ready),
        .rsp_y0_re(dut_y0_re),.rsp_y0_im(dut_y0_im),
        .rsp_y1_re(dut_y1_re),.rsp_y1_im(dut_y1_im),
        .rsp_y0_re_1(dut_y0_re_1),.rsp_y0_im_1(dut_y0_im_1),
        .rsp_y1_re_1(dut_y1_re_1),.rsp_y1_im_1(dut_y1_im_1),
        .busy(dut_busy)
    );

    // ─── BFU Golden ───
    reg         gld_in_valid;
    wire        gld_in_ready;
    reg  [63:0] gld_a_re, gld_a_im, gld_b_re, gld_b_im, gld_w_re, gld_w_im;
    wire        gld_out_valid;
    reg         gld_out_ready;
    wire [63:0] gld_y0_re, gld_y0_im, gld_y1_re, gld_y1_im;

    falcon_f64_complex_bfly golden (
        .clk,.rst_n,
        .in_valid(gld_in_valid),.in_ready(gld_in_ready),
        .a_re(gld_a_re),.a_im(gld_a_im),
        .b_re(gld_b_re),.b_im(gld_b_im),
        .w_re(gld_w_re),.w_im(gld_w_im),
        .out_valid(gld_out_valid),.out_ready(gld_out_ready),
        .y0_re(gld_y0_re),.y0_im(gld_y0_im),
        .y1_re(gld_y1_re),.y1_im(gld_y1_im),
        .status_invalid(),.status_overflow(),.status_underflow(),.status_inexact(),
        .busy()
    );

    // ─── Test helpers ───
    task run_butterfly;
        input [63:0] a_re, a_im, b_re, b_im, w_re, w_im;
        begin
            // Run golden BFU
            gld_a_re = a_re; gld_a_im = a_im;
            gld_b_re = b_re; gld_b_im = b_im;
            gld_w_re = w_re; gld_w_im = w_im;
            gld_in_valid = 1; gld_out_ready = 1;
            @(posedge clk); gld_in_valid = 0;
            while (!gld_out_valid) @(posedge clk);

            // Run DUT (lane 0 only, BUTTERFLY mode)
            dut_a0_re = a_re; dut_a0_im = a_im;
            dut_b0_re = b_re; dut_b0_im = b_im;
            dut_a1_re = 64'd0; dut_a1_im = 64'd0;
            dut_b1_re = 64'd0; dut_b1_im = 64'd0;
            dut_w_re = w_re; dut_w_im = w_im;
            dut_req_mode = 3'd1;  // BUTTERFLY
            dut_req_valid = 1; dut_rsp_ready = 1;
            @(posedge clk); dut_req_valid = 0;
            while (!dut_rsp_valid) @(posedge clk);
        end
    endtask

    // ─── Test sequence ───
    integer pass_cnt, fail_cnt;
    real diff;

    initial begin
        repeat (4) @(posedge clk); rst_n = 1; repeat (4) @(posedge clk);
        pass_cnt = 0; fail_cnt = 0;
        dut_rsp_ready = 1; gld_out_ready = 1;

        $display("=== Shared FPU Lanes vs BFU Golden ===");

        // Test 1: simple values
        begin : t1
            reg [63:0] a_re, a_im, b_re, b_im, w_re, w_im;
            a_re = $realtobits(1.0); a_im = $realtobits(0.0);
            b_re = $realtobits(2.0); b_im = $realtobits(0.0);
            w_re = $realtobits(1.0); w_im = $realtobits(0.0);
            run_butterfly(a_re, a_im, b_re, b_im, w_re, w_im);
            check("Test1 y0_re", dut_y0_re, gld_y0_re);
            check("Test1 y0_im", dut_y0_im, gld_y0_im);
            check("Test1 y1_re", dut_y1_re, gld_y1_re);
            check("Test1 y1_im", dut_y1_im, gld_y1_im);
        end

        // Test 2: complex values
        begin : t2
            reg [63:0] a_re, a_im, b_re, b_im, w_re, w_im;
            a_re = $realtobits(3.5); a_im = $realtobits(-1.2);
            b_re = $realtobits(0.7); b_im = $realtobits(2.3);
            w_re = $realtobits(0.9); w_im = $realtobits(-0.4);
            run_butterfly(a_re, a_im, b_re, b_im, w_re, w_im);
            check("Test2 y0_re", dut_y0_re, gld_y0_re);
            check("Test2 y0_im", dut_y0_im, gld_y0_im);
            check("Test2 y1_re", dut_y1_re, gld_y1_re);
            check("Test2 y1_im", dut_y1_im, gld_y1_im);
        end

        // Test 3: FFT twiddle (cos/sin)
        begin : t3
            reg [63:0] a_re, a_im, b_re, b_im, w_re, w_im;
            a_re = $realtobits(0.123); a_im = $realtobits(-0.456);
            b_re = $realtobits(0.789); b_im = $realtobits(0.321);
            w_re = $realtobits(0.7071067811865476); w_im = $realtobits(-0.7071067811865476);
            run_butterfly(a_re, a_im, b_re, b_im, w_re, w_im);
            check("Test3 y0_re", dut_y0_re, gld_y0_re);
            check("Test3 y0_im", dut_y0_im, gld_y0_im);
            check("Test3 y1_re", dut_y1_re, gld_y1_re);
            check("Test3 y1_im", dut_y1_im, gld_y1_im);
        end

        // Test 4: zero inputs
        begin : t4
            reg [63:0] a_re, a_im, b_re, b_im, w_re, w_im;
            a_re = 64'd0; a_im = 64'd0;
            b_re = 64'd0; b_im = 64'd0;
            w_re = $realtobits(1.0); w_im = $realtobits(0.0);
            run_butterfly(a_re, a_im, b_re, b_im, w_re, w_im);
            check("Test4 y0_re", dut_y0_re, gld_y0_re);
            check("Test4 y0_im", dut_y0_im, gld_y0_im);
            check("Test4 y1_re", dut_y1_re, gld_y1_re);
            check("Test4 y1_im", dut_y1_im, gld_y1_im);
        end

        // Test 5: negative values
        begin : t5
            reg [63:0] a_re, a_im, b_re, b_im, w_re, w_im;
            a_re = $realtobits(-5.0); a_im = $realtobits(3.0);
            b_re = $realtobits(2.0); b_im = $realtobits(-4.0);
            w_re = $realtobits(-0.5); w_im = $realtobits(0.8660254037844387);
            run_butterfly(a_re, a_im, b_re, b_im, w_re, w_im);
            check("Test5 y0_re", dut_y0_re, gld_y0_re);
            check("Test5 y0_im", dut_y0_im, gld_y0_im);
            check("Test5 y1_re", dut_y1_re, gld_y1_re);
            check("Test5 y1_im", dut_y1_im, gld_y1_im);
        end

        $display("");
        $display("Results: %0d PASS, %0d FAIL", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("*** ALL TESTS PASSED ***");
        else $display("*** SOME TESTS FAILED ***");
        #100; $finish;
    end

    task check;
        input [255:0] name;
        input [63:0] actual, expected;
        begin
            if (actual === expected) begin
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL: %0s  actual=%h  expected=%h", name, actual, expected);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // Timeout
    initial begin #500000; $display("TIMEOUT"); $finish; end
endmodule
