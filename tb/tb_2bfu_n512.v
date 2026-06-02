`timescale 1ns/1ps
// N=512 Falcon FWD FFT testbench: compares 2-BFU vs 1-BFU golden
// FIXED: properly instantiates shared BFU instances and connects them
module tb_2bfu_n512;
    localparam ADDR_W = 13;
    localparam N = 512;
    localparam HN = 256;

    reg clk=0, rst_n=0;
    always #5 clk=~clk;

    // ─── Shared memory for 2-BFU DUT ───
    reg [63:0] m2_re[0:N-1], m2_im[0:N-1];
    // ─── Shared memory for 1-BFU golden ───
    reg [63:0] m1_re[0:N-1], m1_im[0:N-1];
    // ─── Twiddle ROM (shared) ───
    reg [63:0] tw_re[0:1023], tw_im[0:1023];

    // ─── GM table for 2-BFU EXU (loaded via $readmemh) ───
    // The 2-BFU EXU reads twiddles from its internal fal_gm_re/fal_gm_im arrays.
    // We also maintain local tw_re/tw_im for the 1-BFU golden.

    // ═══════════════════════════════════════════════════
    // 2-BFU DUT: EXU + 2 shared BFU instances
    // ═══════════════════════════════════════════════════

    // EXU command interface
    reg         d2_cmd_valid;
    wire        d2_cmd_ready;
    reg  [2:0]  d2_cmd_opcode;
    reg  [4:0]  d2_cmd_logn;

    // EXU memory read ports (combinational)
    wire [ADDR_W-1:0] d2_rd0, d2_rd1;
    wire [63:0] d2_rd0_re = m2_re[d2_rd0];
    wire [63:0] d2_rd0_im = m2_im[d2_rd0];
    wire [63:0] d2_rd1_re = m2_re[d2_rd1];
    wire [63:0] d2_rd1_im = m2_im[d2_rd1];

    // EXU twiddle (unused - 2-BFU reads from internal GM table)
    wire [ADDR_W-1:0] d2_tw;

    // EXU memory write ports
    wire        d2_wr_en;
    wire [ADDR_W-1:0] d2_wr0, d2_wr1;
    wire [63:0] d2_wr0_re, d2_wr0_im, d2_wr1_re, d2_wr1_im;

    // Write to shared memory
    always @(posedge clk) begin
        if (d2_wr_en) begin
            m2_re[d2_wr0] <= d2_wr0_re;
            m2_im[d2_wr0] <= d2_wr0_im;
            m2_re[d2_wr1] <= d2_wr1_re;
            m2_im[d2_wr1] <= d2_wr1_im;
        end
    end

    // EXU status
    wire d2_rsp_valid, d2_rsp_done, d2_rsp_fail;
    wire [7:0] d2_rsp_status;
    wire d2_busy;

    // ─── Shared FPU lane wires between EXU and FPU lanes ───
    wire        fc_fpu_req_v, fc_fpu_req_r;
    wire [2:0]  fc_fpu_mode;
    wire [63:0] fc_fpu_a0_re, fc_fpu_a0_im, fc_fpu_b0_re, fc_fpu_b0_im;
    wire [63:0] fc_fpu_a1_re, fc_fpu_a1_im, fc_fpu_b1_re, fc_fpu_b1_im;
    wire [63:0] fc_fpu_w_re, fc_fpu_w_im;
    wire        fc_fpu_rsp_v;
    wire [63:0] fc_fpu_y0_re, fc_fpu_y0_im, fc_fpu_y1_re, fc_fpu_y1_im;
    wire [63:0] fc_fpu_y0_re_1, fc_fpu_y0_im_1, fc_fpu_y1_re_1, fc_fpu_y1_im_1;

    // ─── 2-BFU EXU ───
    falcon_f64_fft_exu_2bfu #(.ADDR_W(ADDR_W)) dut2 (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(d2_cmd_valid), .cmd_ready(d2_cmd_ready),
        .cmd_opcode(d2_cmd_opcode), .cmd_logn(d2_cmd_logn),
        .mem_rd_addr0(d2_rd0), .mem_rd_addr1(d2_rd1),
        .mem_rd_data0_re(d2_rd0_re), .mem_rd_data0_im(d2_rd0_im),
        .mem_rd_data1_re(d2_rd1_re), .mem_rd_data1_im(d2_rd1_im),
        .twiddle_addr(d2_tw), .twiddle_re(64'd0), .twiddle_im(64'd0),
        .mem_wr_en(d2_wr_en), .mem_wr_addr0(d2_wr0), .mem_wr_addr1(d2_wr1),
        .mem_wr_data0_re(d2_wr0_re), .mem_wr_data0_im(d2_wr0_im),
        .mem_wr_data1_re(d2_wr1_re), .mem_wr_data1_im(d2_wr1_im),
        .rsp_valid(d2_rsp_valid), .rsp_done(d2_rsp_done), .rsp_fail(d2_rsp_fail),
        .rsp_status(d2_rsp_status),
        .status_invalid(), .status_overflow(), .status_underflow(), .status_inexact(),
        .busy(d2_busy),
        // External shared FPU lane ports
        .fc_fpu_req_v(fc_fpu_req_v), .fc_fpu_req_r(fc_fpu_req_r),
        .fc_fpu_mode(fc_fpu_mode),
        .fc_fpu_a0_re(fc_fpu_a0_re), .fc_fpu_a0_im(fc_fpu_a0_im),
        .fc_fpu_b0_re(fc_fpu_b0_re), .fc_fpu_b0_im(fc_fpu_b0_im),
        .fc_fpu_a1_re(fc_fpu_a1_re), .fc_fpu_a1_im(fc_fpu_a1_im),
        .fc_fpu_b1_re(fc_fpu_b1_re), .fc_fpu_b1_im(fc_fpu_b1_im),
        .fc_fpu_w_re(fc_fpu_w_re), .fc_fpu_w_im(fc_fpu_w_im),
        .fc_fpu_rsp_v(fc_fpu_rsp_v),
        .fc_fpu_y0_re(fc_fpu_y0_re), .fc_fpu_y0_im(fc_fpu_y0_im),
        .fc_fpu_y1_re(fc_fpu_y1_re), .fc_fpu_y1_im(fc_fpu_y1_im),
        .fc_fpu_y0_re_1(fc_fpu_y0_re_1), .fc_fpu_y0_im_1(fc_fpu_y0_im_1),
        .fc_fpu_y1_re_1(fc_fpu_y1_re_1), .fc_fpu_y1_im_1(fc_fpu_y1_im_1)
    );

    // ─── Shared FPU Lanes ───
    falconsign_shared_fpu_lanes u_sh_fpu (
        .clk(clk), .rst_n(rst_n),
        .req_valid(fc_fpu_req_v), .req_ready(fc_fpu_req_r),
        .req_mode(fc_fpu_mode),
        .req_a0_re(fc_fpu_a0_re), .req_a0_im(fc_fpu_a0_im),
        .req_b0_re(fc_fpu_b0_re), .req_b0_im(fc_fpu_b0_im),
        .req_a1_re(fc_fpu_a1_re), .req_a1_im(fc_fpu_a1_im),
        .req_b1_re(fc_fpu_b1_re), .req_b1_im(fc_fpu_b1_im),
        .req_w_re(fc_fpu_w_re), .req_w_im(fc_fpu_w_im),
        .req_w1_re(fc_fpu_w_re), .req_w1_im(fc_fpu_w_im),  // same twiddle for both lanes in FFT
        .rsp_valid(fc_fpu_rsp_v), .rsp_ready(1'b1),
        .rsp_y0_re(fc_fpu_y0_re), .rsp_y0_im(fc_fpu_y0_im),
        .rsp_y1_re(fc_fpu_y1_re), .rsp_y1_im(fc_fpu_y1_im),
        .rsp_y0_re_1(fc_fpu_y0_re_1), .rsp_y0_im_1(fc_fpu_y0_im_1),
        .rsp_y1_re_1(fc_fpu_y1_re_1), .rsp_y1_im_1(fc_fpu_y1_im_1),
        .busy()
    );

    // ═══════════════════════════════════════════════════
    // 1-BFU golden (embedded BFU, no external ports)
    // ═══════════════════════════════════════════════════

    reg         g1_cmd_valid;
    wire        g1_cmd_ready;
    reg  [2:0]  g1_cmd_opcode;
    reg  [4:0]  g1_cmd_logn;

    wire [ADDR_W-1:0] g1_rd0, g1_rd1;
    wire [63:0] g1_rd0_re = m1_re[g1_rd0];
    wire [63:0] g1_rd0_im = m1_im[g1_rd0];
    wire [63:0] g1_rd1_re = m1_re[g1_rd1];
    wire [63:0] g1_rd1_im = m1_im[g1_rd1];

    wire [ADDR_W-1:0] g1_tw;

    wire        g1_wr_en;
    wire [ADDR_W-1:0] g1_wr0, g1_wr1;
    wire [63:0] g1_wr0_re, g1_wr0_im, g1_wr1_re, g1_wr1_im;

    always @(posedge clk) begin
        if (g1_wr_en) begin
            m1_re[g1_wr0] <= g1_wr0_re;
            m1_im[g1_wr0] <= g1_wr0_im;
            m1_re[g1_wr1] <= g1_wr1_re;
            m1_im[g1_wr1] <= g1_wr1_im;
        end
    end

    wire g1_rsp_valid, g1_rsp_done, g1_rsp_fail;
    wire [7:0] g1_rsp_status;
    wire g1_busy;

    falcon_f64_fft_exu #(.ADDR_W(ADDR_W)) dut1 (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(g1_cmd_valid), .cmd_ready(g1_cmd_ready),
        .cmd_opcode(g1_cmd_opcode), .cmd_logn(g1_cmd_logn),
        .mem_rd_addr0(g1_rd0), .mem_rd_addr1(g1_rd1),
        .mem_rd_data0_re(g1_rd0_re), .mem_rd_data0_im(g1_rd0_im),
        .mem_rd_data1_re(g1_rd1_re), .mem_rd_data1_im(g1_rd1_im),
        .twiddle_addr(g1_tw), .twiddle_re(64'd0), .twiddle_im(64'd0),
        .mem_wr_en(g1_wr_en), .mem_wr_addr0(g1_wr0), .mem_wr_addr1(g1_wr1),
        .mem_wr_data0_re(g1_wr0_re), .mem_wr_data0_im(g1_wr0_im),
        .mem_wr_data1_re(g1_wr1_re), .mem_wr_data1_im(g1_wr1_im),
        .rsp_valid(g1_rsp_valid), .rsp_done(g1_rsp_done), .rsp_fail(g1_rsp_fail),
        .rsp_status(g1_rsp_status),
        .status_invalid(), .status_overflow(), .status_underflow(), .status_inexact(),
        .busy(g1_busy)
    );

    // ═══════════════════════════════════════════════════
    // Debug: track FPU lane signals
    // ═══════════════════════════════════════════════════
    reg [31:0] d2_fpu_fire_cnt;
    reg [31:0] d2_fpu_done_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d2_fpu_fire_cnt <= 0;
            d2_fpu_done_cnt <= 0;
        end else begin
            if (fc_fpu_req_v && fc_fpu_req_r) d2_fpu_fire_cnt <= d2_fpu_fire_cnt + 1;
            if (fc_fpu_rsp_v) d2_fpu_done_cnt <= d2_fpu_done_cnt + 1;
        end
    end

    // ═══════════════════════════════════════════════════
    // Test sequence
    // ═══════════════════════════════════════════════════
    integer i, err_cnt;
    real diff_re, diff_im, max_err;
    real re2, im2, re1, im1;
    integer cyc2, cyc1;

    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (4) @(posedge clk);

        $display("=== N=512 Falcon FWD FFT: 2-BFU vs 1-BFU Golden ===");

        // Load test data: real coefficients (simulating falcon polynomial)
        $display("Loading test data...");
        for (i = 0; i < N; i = i + 1) begin
            m2_re[i] = $realtobits($sin(i * 0.123 + 0.5) * 2.0);
            m2_im[i] = 64'd0;
            m1_re[i] = m2_re[i];
            m1_im[i] = 64'd0;
        end

        // ─── Run 2-BFU FFT ───
        $display("Starting 2-BFU Falcon FWD FFT (logn=9)...");
        d2_cmd_opcode = 3'd3;  // OP_FFT_FALCON_FWD
        d2_cmd_logn = 5'd9;
        d2_cmd_valid = 1;
        @(posedge clk);
        d2_cmd_valid = 0;

        cyc2 = 0;
        while (!d2_rsp_valid && cyc2 < 500000) begin
            @(posedge clk);
            cyc2 = cyc2 + 1;
        end
        if (d2_rsp_fail)
            $display("FAIL: 2-BFU reported failure status=%h", d2_rsp_status);
        $display("2-BFU completed in %0d cycles, done=%b fail=%b", cyc2, d2_rsp_done, d2_rsp_fail);
        $display("2-BFU FPU fires=%0d, FPU dones=%0d", d2_fpu_fire_cnt, d2_fpu_done_cnt);

        // ─── Run 1-BFU FFT (golden) ───
        $display("Starting 1-BFU Falcon FWD FFT (golden)...");
        g1_cmd_opcode = 3'd3;
        g1_cmd_logn = 5'd9;
        g1_cmd_valid = 1;
        @(posedge clk);
        g1_cmd_valid = 0;

        cyc1 = 0;
        while (!g1_rsp_valid && cyc1 < 500000) begin
            @(posedge clk);
            cyc1 = cyc1 + 1;
        end
        if (g1_rsp_fail)
            $display("FAIL: 1-BFU reported failure status=%h", g1_rsp_status);
        $display("1-BFU completed in %0d cycles, done=%b fail=%b", cyc1, g1_rsp_done, g1_rsp_fail);

        $display("Speedup: %.1fx (1-BFU:%0d / 2-BFU:%0d)", $itor(cyc1)*1.0/$itor(cyc2), cyc1, cyc2);

        // ─── Compare results ───
        err_cnt = 0;
        max_err = 0.0;
        for (i = 0; i < N; i = i + 1) begin
            re2 = $bitstoreal(m2_re[i]);
            im2 = $bitstoreal(m2_im[i]);
            re1 = $bitstoreal(m1_re[i]);
            im1 = $bitstoreal(m1_im[i]);
            diff_re = re2 - re1; if (diff_re < 0) diff_re = -diff_re;
            diff_im = im2 - im1; if (diff_im < 0) diff_im = -diff_im;
            if (diff_re > max_err) max_err = diff_re;
            if (diff_im > max_err) max_err = diff_im;
            if (diff_re > 0.001 || diff_im > 0.001) begin
                if (err_cnt < 20)
                    $display("  Mismatch[%0d]: 2bfu=(%.6f,%.6f) 1bfu=(%.6f,%.6f) diff=(%.2e,%.2e)",
                        i, re2, im2, re1, im1, diff_re, diff_im);
                err_cnt = err_cnt + 1;
            end
        end

        if (err_cnt == 0)
            $display("PASS: All %0d outputs match 1-BFU golden (max_err=%.2e)", N, max_err);
        else
            $display("FAIL: %0d mismatches out of %0d (max_err=%.2e)", err_cnt, N, max_err);

        #100;
        $finish;
    end

    // Timeout
    initial begin
        #10000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
