`timescale 1ns/1ps
// Testbench for falcon_f64_fft_exu — double-buffered with stage-boundary swap

module tb_falcon_f64_fft_exu;

    localparam ADDR_W = 4;
    localparam MAX_N  = (1 << ADDR_W);
    localparam MAX_NH = MAX_N / 2;

    localparam [2:0] OP_FFT_FWD = 3'd0;
    localparam [2:0] OP_FFT_INV = 3'd1;

    reg clk, rst_n, cmd_valid;
    wire cmd_ready, rsp_valid, rsp_done, rsp_fail, busy;
    reg [2:0] cmd_opcode;
    reg [4:0] cmd_logn;

    wire [ADDR_W-1:0] mem_rd_addr, mem_wr_addr, twiddle_addr;
    wire [255:0] mem_rd_data, mem_wr_data;
    wire [63:0] twiddle_re, twiddle_im;
    wire mem_wr_en;

    // ─── Double-buffered memory ───
    reg [255:0] buf_a [0:MAX_NH-1];
    reg [255:0] buf_b [0:MAX_NH-1];
    reg         buf_sel;  // 0: read A write B, 1: read B write A
    reg         swap_pending;

    // Read from active read buffer
    assign mem_rd_data = buf_sel ? buf_b[mem_rd_addr] : buf_a[mem_rd_addr];
    assign twiddle_re = tw_re_rom[twiddle_addr];
    assign twiddle_im = tw_im_rom[twiddle_addr];

    // Write to active write buffer (opposite of read)
    always @(posedge clk) begin
        if (mem_wr_en) begin
            if (buf_sel)
                buf_a[mem_wr_addr] <= mem_wr_data;
            else
                buf_b[mem_wr_addr] <= mem_wr_data;
        end
    end

    // ─── Stage boundary detection and buffer swap ───
    wire stage_boundary = (dut.state == 5'd3) &&
                          (dut.pair_idx == (dut.pair_count_limit - 1'b1));

    always @(posedge clk) begin
        if (!rst_n) begin
            swap_pending <= 1'b0;
            buf_sel <= 1'b0;
        end else begin
            if (stage_boundary)
                swap_pending <= 1'b1;
            else if (swap_pending) begin
                buf_sel <= ~buf_sel;
                swap_pending <= 1'b0;
            end
        end
    end

    // Unpacked arrays for testbench
    reg [63:0] mem_re [0:MAX_N-1];
    reg [63:0] mem_im [0:MAX_N-1];
    reg [63:0] ref_re [0:MAX_N-1];
    reg [63:0] ref_im [0:MAX_N-1];
    reg [63:0] saved_re [0:MAX_N-1];
    reg [63:0] saved_im [0:MAX_N-1];
    reg [63:0] tw_re_rom [0:MAX_N-1];
    reg [63:0] tw_im_rom [0:MAX_N-1];

    integer error_count, wait_count;

    falcon_f64_fft_exu #(.ADDR_W(ADDR_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_opcode(cmd_opcode), .cmd_logn(cmd_logn),
        .mem_rd_addr(mem_rd_addr), .mem_rd_data(mem_rd_data),
        .twiddle_addr(twiddle_addr), .twiddle_re(twiddle_re), .twiddle_im(twiddle_im),
        .mem_wr_en(mem_wr_en), .mem_wr_addr(mem_wr_addr), .mem_wr_data(mem_wr_data),
        .rsp_valid(rsp_valid), .rsp_done(rsp_done), .rsp_fail(rsp_fail),
        .rsp_status(),
        .status_invalid(), .status_overflow(), .status_underflow(), .status_inexact(),
        .busy(busy));

    always #5 clk = ~clk;

    // ─── Sync helpers ───
    task sync_to_buffers;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 2) begin
                buf_a[i/2] = {mem_im[i+1], mem_re[i+1], mem_im[i], mem_re[i]};
                buf_b[i/2] = {mem_im[i+1], mem_re[i+1], mem_im[i], mem_re[i]};
            end
            buf_sel = 0;  // read from A, write to B
        end
    endtask

    task sync_from_read_buffer;
        input integer n;
        integer i;
        reg [255:0] word;
        begin
            for (i = 0; i < n; i = i + 2) begin
                word = buf_sel ? buf_b[i/2] : buf_a[i/2];
                mem_re[i]   = word[63:0];
                mem_im[i]   = word[127:64];
                mem_re[i+1] = word[191:128];
                mem_im[i+1] = word[255:192];
            end
        end
    endtask

    // ─── Tasks ───
    task generate_twiddles;
        input integer n; integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                tw_re_rom[i] = $realtobits($cos(-2.0 * 3.14159265358979 * i / n));
                tw_im_rom[i] = $realtobits($sin(-2.0 * 3.14159265358979 * i / n));
            end
            for (i = n; i < MAX_N; i = i + 1) begin
                tw_re_rom[i] = 64'd0; tw_im_rom[i] = 64'd0;
            end
        end
    endtask

    task load_simple_vectors;
        input integer n; integer i; real r;
        begin for (i = 0; i < n; i = i + 1) begin
            r = 0.5 + i * 0.75; mem_re[i] = $realtobits(r); mem_im[i] = $realtobits(-r * 0.3);
        end end
    endtask
    task load_random_vectors;
        input integer n; input integer seed; integer i;
        begin for (i = 0; i < n; i = i + 1) begin
            mem_re[i] = $realtobits($sin(seed + i * 31337) * 1.5);
            mem_im[i] = $realtobits($cos(seed + i * 31337 + 1000) * 1.5);
        end end
    endtask
    task load_zero_vectors;
        input integer n; integer i;
        begin for (i = 0; i < n; i = i + 1) begin mem_re[i] = 64'd0; mem_im[i] = 64'd0; end end
    endtask
    task load_impulse_vectors;
        input integer n; integer i;
        begin for (i = 0; i < n; i = i + 1) begin mem_re[i] = 64'd0; mem_im[i] = 64'd0; end
            mem_re[0] = 64'h3ff0_0000_0000_0000; end
    endtask
    task load_all_ones;
        input integer n; integer i;
        begin for (i = 0; i < n; i = i + 1) begin
            mem_re[i] = 64'h3ff0_0000_0000_0000; mem_im[i] = 64'd0; end end
    endtask
    task save_originals;
        input integer n; integer i;
        begin for (i = 0; i < n; i = i + 1) begin saved_re[i] = mem_re[i]; saved_im[i] = mem_im[i]; end end
    endtask
    task copy_to_ref;
        input integer n; integer i;
        begin for (i = 0; i < n; i = i + 1) begin ref_re[i] = mem_re[i]; ref_im[i] = mem_im[i]; end end
    endtask

    // Correct DFT reference (no data hazard)
    task reference_fft;
        input integer n; input integer logn; input inverse;
        integer k, stage, m, half_m, tw_idx;
        real a_re_r, a_im_r, b_re_r, b_im_r, w_re_r, w_im_r, t_re_r, t_im_r;
        // Use temporary arrays to avoid read-after-write hazard
        reg [63:0] tmp_re [0:MAX_N-1];
        reg [63:0] tmp_im [0:MAX_N-1];
        begin
            for (k = 0; k < logn; k = k + 1) begin
                stage = inverse ? (logn - 1 - k) : k;
                half_m = (1 << stage);
                // Read all inputs first (no hazard)
                for (m = 0; m < (n >> 1); m = m + 1) begin
                    tw_idx = (m & (half_m - 1)) << (logn - stage - 1);
                    a_re_r = $bitstoreal(ref_re[2*m]); a_im_r = $bitstoreal(ref_im[2*m]);
                    b_re_r = $bitstoreal(ref_re[2*m+1]); b_im_r = $bitstoreal(ref_im[2*m+1]);
                    w_re_r = $bitstoreal(tw_re_rom[tw_idx]);
                    w_im_r = inverse ? -$bitstoreal(tw_im_rom[tw_idx]) : $bitstoreal(tw_im_rom[tw_idx]);
                    if (inverse) begin
                        t_re_r = (a_re_r - b_re_r) * w_re_r - (a_im_r - b_im_r) * w_im_r;
                        t_im_r = (a_re_r - b_re_r) * w_im_r + (a_im_r - b_im_r) * w_re_r;
                        tmp_re[2*m]   = $realtobits((a_re_r + b_re_r) * 0.5);
                        tmp_im[2*m]   = $realtobits((a_im_r + b_im_r) * 0.5);
                        tmp_re[2*m+1] = $realtobits(t_re_r * 0.5);
                        tmp_im[2*m+1] = $realtobits(t_im_r * 0.5);
                    end else begin
                        t_re_r = b_re_r * w_re_r - b_im_r * w_im_r;
                        t_im_r = b_re_r * w_im_r + b_im_r * w_re_r;
                        tmp_re[2*m]   = $realtobits(a_re_r + t_re_r);
                        tmp_im[2*m]   = $realtobits(a_im_r + t_im_r);
                        tmp_re[2*m+1] = $realtobits(a_re_r - t_re_r);
                        tmp_im[2*m+1] = $realtobits(a_im_r - t_im_r);
                    end
                end
                // Write all outputs
                for (m = 0; m < n; m = m + 1) begin
                    ref_re[m] = tmp_re[m]; ref_im[m] = tmp_im[m];
                end
            end
        end
    endtask

    task run_fft_cmd;
        input integer logn; input [2:0] opcode;
        begin
            swap_pending = 1'b0;
            @(posedge clk); while (!cmd_ready) @(posedge clk);
            cmd_opcode <= opcode; cmd_logn <= logn; cmd_valid <= 1;
            @(posedge clk); cmd_valid <= 0;
            wait_count = 0;
            while ((rsp_valid !== 1'b1) && (wait_count <= 20000)) begin #1; wait_count = wait_count + 1; end
            if (wait_count > 20000) begin
                $display("TB_FAIL - fft timeout logn=%0d opcode=%0d", logn, opcode);
                error_count = error_count + 1;
            end
            // Wait for final swap
            repeat (5) @(posedge clk);
            #1;
        end
    endtask

    task compare_with_reference;
        input integer n; input [128*8-1:0] phase_name;
        integer i; real exp_re, exp_im, got_re, got_im, err_re, err_im, mag;
        begin
            for (i = 0; i < n; i = i + 1) begin
                exp_re = $bitstoreal(ref_re[i]); exp_im = $bitstoreal(ref_im[i]);
                got_re = $bitstoreal(mem_re[i]); got_im = $bitstoreal(mem_im[i]);
                err_re = got_re - exp_re; if (err_re < 0) err_re = -err_re;
                err_im = got_im - exp_im; if (err_im < 0) err_im = -err_im;
                mag = (exp_re > 0 ? exp_re : -exp_re) + (exp_im > 0 ? exp_im : -exp_im);
                if (mag < 1e-150) begin
                    if ((err_re > 1e-12) || (err_im > 1e-12)) begin
                        $display("TB_FAIL - %0s [n=%0d] idx=%0d exp=(%e,%e) got=(%e,%e)", phase_name, n, i, exp_re, exp_im, got_re, got_im);
                        error_count = error_count + 1;
                    end
                end else begin
                    if ((err_re > mag * 1e-12) || (err_im > mag * 1e-12)) begin
                        $display("TB_FAIL - %0s [n=%0d] idx=%0d exp=(%e,%e) got=(%e,%e)", phase_name, n, i, exp_re, exp_im, got_re, got_im);
                        error_count = error_count + 1;
                    end
                end
            end
        end
    endtask

    task check_roundtrip;
        input integer n; input [128*8-1:0] phase_name;
        integer i; real err_re, err_im, max_err;
        begin
            max_err = 0.0;
            for (i = 0; i < n; i = i + 1) begin
                err_re = $bitstoreal(mem_re[i]) - $bitstoreal(saved_re[i]); if (err_re < 0) err_re = -err_re;
                err_im = $bitstoreal(mem_im[i]) - $bitstoreal(saved_im[i]); if (err_im < 0) err_im = -err_im;
                if (err_re > max_err) max_err = err_re;
                if (err_im > max_err) max_err = err_im;
                if ((err_re > 1e-6) || (err_im > 1e-6)) begin
                    $display("TB_FAIL - %0s [n=%0d] idx=%0d roundtrip mismatch", phase_name, n, i);
                    error_count = error_count + 1;
                end
            end
            $display("TB_INFO - %0s [n=%0d] roundtrip max_err=%e", phase_name, n, max_err);
        end
    endtask

    task run_testcase;
        input integer n; input integer logn; input [128*8-1:0] name;
        reg [128*8-1:0] full_name;
        begin
            $display("=== Test: %0s n=%0d ===", name, n);
            generate_twiddles(n); save_originals(n);

            // Forward FFT
            $sformat(full_name, "%s_fft_fwd", name);
            copy_to_ref(n); reference_fft(n, logn, 1'b0);
            sync_to_buffers(n); run_fft_cmd(logn, OP_FFT_FWD); sync_from_read_buffer(n);
            compare_with_reference(n, full_name);

            // Inverse FFT
            $sformat(full_name, "%s_fft_inv", name);
            copy_to_ref(n); reference_fft(n, logn, 1'b1);
            sync_to_buffers(n); run_fft_cmd(logn, OP_FFT_INV); sync_from_read_buffer(n);
            compare_with_reference(n, full_name);

            // Round-trip
            $sformat(full_name, "%s_roundtrip", name);
            sync_to_buffers(n);
            run_fft_cmd(logn, OP_FFT_FWD);
            run_fft_cmd(logn, OP_FFT_INV);
            sync_from_read_buffer(n);
            check_roundtrip(n, full_name);
        end
    endtask

    initial begin
        integer i;
        clk = 0; rst_n = 0; cmd_valid = 0; cmd_opcode = 0; cmd_logn = 0; error_count = 0;
        buf_sel = 0; swap_pending = 0;

        repeat (3) @(posedge clk); rst_n = 1; repeat (2) @(posedge clk);

        load_simple_vectors(4);  run_testcase(4, 2, "simple_n4");
        load_random_vectors(4, 42); run_testcase(4, 2, "random_n4");
        load_zero_vectors(4);    run_testcase(4, 2, "zeros_n4");
        load_impulse_vectors(4); run_testcase(4, 2, "impulse_n4");
        load_all_ones(4);        run_testcase(4, 2, "allones_n4");

        load_simple_vectors(8);  run_testcase(8, 3, "simple_n8");
        load_random_vectors(8, 137); run_testcase(8, 3, "random_n8");
        load_zero_vectors(8);    run_testcase(8, 3, "zeros_n8");
        load_impulse_vectors(8); run_testcase(8, 3, "impulse_n8");
        load_all_ones(8);        run_testcase(8, 3, "allones_n8");

        load_simple_vectors(16);  run_testcase(16, 4, "simple_n16");
        load_random_vectors(16, 999); run_testcase(16, 4, "random_n16");
        load_zero_vectors(16);    run_testcase(16, 4, "zeros_n16");
        load_impulse_vectors(16); run_testcase(16, 4, "impulse_n16");
        load_all_ones(16);        run_testcase(16, 4, "allones_n16");

        if (error_count == 0) begin
            $display(""); $display("##########################################");
            $display("  TB_PASS falcon_f64_fft_exu");
            $display("##########################################");
        end else begin
            $display(""); $display("##########################################");
            $display("  TB_FAIL falcon_f64_fft_exu error_count=%0d", error_count);
            $display("##########################################");
        end
        $finish;
    end

endmodule
