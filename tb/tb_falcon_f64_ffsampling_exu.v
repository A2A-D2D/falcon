`timescale 1ns/1ps

module tb_falcon_f64_ffsampling_exu;
    localparam ADDR_W = 6;
    localparam OP_READ_L10 = 4'd0;
    localparam OP_SPLIT = 4'd1;
    localparam OP_ADJUST = 4'd2;
    localparam OP_SAMPLE = 4'd3;
    localparam OP_MERGE = 4'd4;

    reg clk;
    reg rst_n;
    reg task_valid;
    wire task_ready;
    reg [67:0] task_word;
    wire task_done;
    wire task_fail;
    wire [7:0] task_status;

    wire mem_rd_en;
    wire [ADDR_W-1:0] mem_rd_addr;
    reg [255:0] mem_rd_data;
    wire mem_wr_en;
    wire [ADDR_W-1:0] mem_wr_addr;
    wire [255:0] mem_wr_data;

    wire [ADDR_W-1:0] twiddle_addr;
    wire [63:0] twiddle_re;
    wire [63:0] twiddle_im;

    wire fpu_req_valid;
    wire fpu_req_ready;
    wire [3:0] fpu_req_op;
    wire [63:0] fpu_req_a;
    wire [63:0] fpu_req_b;
    wire [63:0] fpu_req_c;
    wire fpu_rsp_valid;
    wire [63:0] fpu_rsp_result;
    wire [4:0] fpu_rsp_flags;

    wire sz_cmd_valid;
    reg sz_cmd_ready;
    wire [63:0] sz_cmd_mu;
    wire [63:0] sz_cmd_sigma_inv;
    wire sz_cmd_pair;
    reg sz_rsp_valid;
    reg [63:0] sz_rsp_z0;
    reg [63:0] sz_rsp_z1;

    reg [255:0] mem [0:(1 << ADDR_W)-1];
    integer errors;
    integer sample_seen;

    assign twiddle_re = 64'h3FF0000000000000; // 1.0
    assign twiddle_im = 64'h0000000000000000; // 0.0

    falcon_f64_ffsampling_exu #(.ADDR_W(ADDR_W)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .task_valid(task_valid),
        .task_ready(task_ready),
        .task_word(task_word),
        .task_done(task_done),
        .task_fail(task_fail),
        .task_status(task_status),
        .mem_rd_en(mem_rd_en),
        .mem_rd_addr(mem_rd_addr),
        .mem_rd_data(mem_rd_data),
        .mem_wr_en(mem_wr_en),
        .mem_wr_addr(mem_wr_addr),
        .mem_wr_data(mem_wr_data),
        .twiddle_addr(twiddle_addr),
        .twiddle_re(twiddle_re),
        .twiddle_im(twiddle_im),
        .fpu_req_valid(fpu_req_valid),
        .fpu_req_ready(fpu_req_ready),
        .fpu_req_op(fpu_req_op),
        .fpu_req_a(fpu_req_a),
        .fpu_req_b(fpu_req_b),
        .fpu_req_c(fpu_req_c),
        .fpu_rsp_valid(fpu_rsp_valid),
        .fpu_rsp_result(fpu_rsp_result),
        .sz_cmd_valid(sz_cmd_valid),
        .sz_cmd_ready(sz_cmd_ready),
        .sz_cmd_mu(sz_cmd_mu),
        .sz_cmd_sigma_inv(sz_cmd_sigma_inv),
        .sz_cmd_pair(sz_cmd_pair),
        .sz_rsp_valid(sz_rsp_valid),
        .sz_rsp_z0(sz_rsp_z0),
        .sz_rsp_z1(sz_rsp_z1)
    );

    falcon_fp_fpu u_fpu (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(fpu_req_valid),
        .req_ready(fpu_req_ready),
        .req_op(fpu_req_op),
        .req_a(fpu_req_a),
        .req_b(fpu_req_b),
        .req_c(fpu_req_c),
        .req_fmt(2'b01),
        .req_rm(3'b000),
        .req_fcvt_op(2'b00),
        .rsp_valid(fpu_rsp_valid),
        .rsp_ready(1'b1),
        .rsp_result(fpu_rsp_result),
        .rsp_flags(fpu_rsp_flags),
        .busy()
    );

    always #5 clk = ~clk;

    always @(*) begin
        mem_rd_data = mem[mem_rd_addr];
    end

    always @(posedge clk) begin
        if (mem_wr_en) begin
            mem[mem_wr_addr] <= mem_wr_data;
        end
    end

    function [67:0] pack_task;
        input [3:0] opcode;
        input [3:0] level;
        input [13:0] src0;
        input [13:0] src1;
        input [13:0] dst;
        begin
            pack_task = 68'd0;
            pack_task[67:64] = opcode;
            pack_task[63:60] = level;
            pack_task[49:36] = src0;
            pack_task[35:22] = src1;
            pack_task[21:8]  = dst;
        end
    endfunction

    function [67:0] pack_adjust_task;
        input [3:0] level;
        input [13:0] t0_dst;
        input [13:0] t1_src;
        input [13:0] l10_src;
        input [13:0] z1_src;
        begin
            pack_adjust_task = pack_task(OP_ADJUST, level, t1_src, l10_src, z1_src);
            pack_adjust_task[59:50] = t0_dst[9:0];
            pack_adjust_task[7:0]   = {t0_dst[13:10], 4'h0};
        end
    endfunction

    task issue_task;
        input [67:0] word;
        integer timeout;
        begin
            @(posedge clk);
            while (!task_ready) @(posedge clk);
            task_word  <= word;
            task_valid <= 1'b1;
            @(posedge clk);
            task_valid <= 1'b0;

            timeout = 0;
            while (!task_done && !task_fail && timeout < 1000) begin
                timeout = timeout + 1;
                @(posedge clk);
            end

            if (timeout >= 1000) begin
                $display("TB_ERROR task timeout opcode=%0d", word[67:64]);
                errors = errors + 1;
            end
            if (task_fail) begin
                $display("TB_ERROR task_fail opcode=%0d status=%02x", word[67:64], task_status);
                errors = errors + 1;
            end
        end
    endtask

    task expect_word;
        input [ADDR_W-1:0] addr;
        input [255:0] exp;
        begin
            if (mem[addr] !== exp) begin
                $display("TB_ERROR mem[%0d] got=%064x exp=%064x", addr, mem[addr], exp);
                errors = errors + 1;
            end
        end
    endtask

    task issue_sample_task;
        input [67:0] word;
        integer timeout;
        begin
            sample_seen = 0;
            @(posedge clk);
            while (!task_ready) @(posedge clk);
            task_word  <= word;
            task_valid <= 1'b1;
            @(posedge clk);
            task_valid <= 1'b0;

            timeout = 0;
            while (!task_done && !task_fail && timeout < 1000) begin
                timeout = timeout + 1;
                @(posedge clk);
                if (sz_cmd_valid) begin
                    if (sample_seen == 0) begin
                        if (sz_cmd_mu !== mem[30][63:0] || sz_cmd_sigma_inv !== mem[31][63:0]) begin
                            $display("TB_ERROR sample0 params mu=%h sig=%h", sz_cmd_mu, sz_cmd_sigma_inv);
                            errors = errors + 1;
                        end
                        sz_rsp_z0 <= 64'h3FF0000000000000; // 1.0
                    end else if (sample_seen == 1) begin
                        if (sz_cmd_mu !== mem[30][127:64] || sz_cmd_sigma_inv !== mem[31][127:64]) begin
                            $display("TB_ERROR sample1 params mu=%h sig=%h", sz_cmd_mu, sz_cmd_sigma_inv);
                            errors = errors + 1;
                        end
                        sz_rsp_z0 <= 64'h4000000000000000; // 2.0
                    end
                    sample_seen = sample_seen + 1;
                    @(posedge clk);
                    sz_rsp_valid <= 1'b1;
                    @(posedge clk);
                    sz_rsp_valid <= 1'b0;
                end
            end

            if (timeout >= 1000) begin
                $display("TB_ERROR sample task timeout");
                errors = errors + 1;
            end
            if (sample_seen != 2) begin
                $display("TB_ERROR sample_seen=%0d", sample_seen);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        task_valid = 1'b0;
        task_word = 68'd0;
        sz_cmd_ready = 1'b1;
        sz_rsp_valid = 1'b0;
        sz_rsp_z0 = 64'd0;
        sz_rsp_z1 = 64'd0;
        errors = 0;

        mem[0] = {128'd0, 64'h4000000000000000, 64'h3FF0000000000000}; // 1.0 + j2.0
        mem[1] = {128'd0, 64'h4010000000000000, 64'h4008000000000000}; // 3.0 + j4.0

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        issue_task(pack_task(OP_SPLIT, 4'd8, 14'd0, 14'd0, 14'd10));
        issue_task(pack_task(OP_MERGE, 4'd8, 14'd10, 14'd0, 14'd20));

        // Scalar MERGE at level 8 produces one output (reconstructed complex)
        expect_word(6'd20, mem[0]);

        mem[5] = {128'd0, 64'd0, 64'h3FF0000000000000}; // l10 = 1.0 + j0
        mem[8] = {128'd0, 64'h4034000000000000, 64'h4024000000000000}; // t0 = 10 + j20
        mem[9] = {128'd0, 64'h401C000000000000, 64'h4014000000000000}; // t1 = 5 + j7
        mem[10] = {128'd0, 64'h4008000000000000, 64'h4000000000000000}; // z1 = 2 + j3
        issue_task(pack_task(OP_READ_L10, 4'd8, 14'd0, 14'd5, 14'd0));
        issue_task(pack_adjust_task(4'd8, 14'd8, 14'd9, 14'd5, 14'd10));
        expect_word(6'd8, {128'd0, 64'h4038000000000000, 64'h402A000000000000}); // 13 + j24

        mem[30] = {128'd0, 64'h4008000000000000, 64'h4004000000000000}; // mu0=2.5, mu1=3.0
        mem[31] = {128'd0, 64'h3FE0000000000000, 64'h3FF0000000000000}; // sig0=1.0, sig1=0.5
        issue_sample_task(pack_task(OP_SAMPLE, 4'd8, 14'd30, 14'd31, 14'd32));
        expect_word(6'd32, {128'd0, 64'h4000000000000000, 64'h3FF0000000000000});

        if (errors == 0) begin
            $display("TB_PASS falcon_f64_ffsampling_exu");
        end else begin
            $display("TB_FAIL falcon_f64_ffsampling_exu errors=%0d", errors);
        end
        $finish;
    end
endmodule
