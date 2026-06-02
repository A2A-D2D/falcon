`timescale 1ns/1ps

module tb_tmp_fpu_tg;
    reg clk;
    reg rst_n;
    reg req_valid;
    wire req_ready;
    reg [3:0] req_op;
    reg [63:0] req_a;
    reg [63:0] req_b;
    reg [63:0] req_c;
    wire rsp_valid;
    wire [63:0] rsp_result;
    wire [4:0] rsp_flags;

    falcon_fp_fpu dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_ready(req_ready),
        .req_op(req_op), .req_a(req_a), .req_b(req_b), .req_c(req_c),
        .req_fmt(2'b01), .req_rm(3'b000), .req_fcvt_op(2'b00),
        .rsp_valid(rsp_valid), .rsp_ready(1'b1),
        .rsp_result(rsp_result), .rsp_flags(rsp_flags), .busy()
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task run_op;
        input [3:0] op;
        input [63:0] a;
        input [63:0] b;
        input [63:0] c;
        begin
            @(posedge clk);
            req_op <= op;
            req_a <= a;
            req_b <= b;
            req_c <= c;
            req_valid <= 1'b1;
            @(posedge clk);
            req_valid <= 1'b0;
            while (!rsp_valid) @(posedge clk);
            $display("op=%0d result=%h real=%f flags=%b", op, rsp_result, $bitstoreal(rsp_result), rsp_flags);
        end
    endtask

    initial begin
        rst_n = 1'b0;
        req_valid = 1'b0;
        req_op = 4'd0;
        req_a = 64'd0;
        req_b = 64'd0;
        req_c = 64'd0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        run_op(4'd2, 64'h413ebf9ba6a4d05b, 64'h4073797cafc86fa0, 64'd0);
        run_op(4'd6, 64'hc0cbe6439090dc68, 64'h40425e8209e3f608, 64'h41c3f7e530d35706);
        run_op(4'd2, 64'h41c2b04b67356028, 64'h3f1554e39097a782, 64'd0);
        $finish;
    end
endmodule
