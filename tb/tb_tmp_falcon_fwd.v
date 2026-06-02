`timescale 1ns/1ps

module tb_tmp_falcon_fwd;
    localparam ADDR_W = 4;
    reg clk, rst_n, cmd_valid;
    wire cmd_ready;
    reg [2:0] cmd_opcode;
    reg [4:0] cmd_logn;
    wire [ADDR_W-1:0] rd0, rd1, tw_addr, wr0, wr1;
    reg [63:0] mem_re [0:15];
    reg [63:0] mem_im [0:15];
    wire [63:0] rd0_re = mem_re[rd0];
    wire [63:0] rd0_im = mem_im[rd0];
    wire [63:0] rd1_re = mem_re[rd1];
    wire [63:0] rd1_im = mem_im[rd1];
    wire wr_en;
    wire [63:0] wr0_re, wr0_im, wr1_re, wr1_im;
    wire rsp_valid, rsp_done, rsp_fail;
    wire [7:0] rsp_status;

    falcon_f64_fft_exu #(.ADDR_W(ADDR_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_opcode(cmd_opcode), .cmd_logn(cmd_logn),
        .mem_rd_addr0(rd0), .mem_rd_addr1(rd1),
        .mem_rd_data0_re(rd0_re), .mem_rd_data0_im(rd0_im),
        .mem_rd_data1_re(rd1_re), .mem_rd_data1_im(rd1_im),
        .twiddle_addr(tw_addr), .twiddle_re(64'd0), .twiddle_im(64'd0),
        .mem_wr_en(wr_en), .mem_wr_addr0(wr0), .mem_wr_addr1(wr1),
        .mem_wr_data0_re(wr0_re), .mem_wr_data0_im(wr0_im),
        .mem_wr_data1_re(wr1_re), .mem_wr_data1_im(wr1_im),
        .rsp_valid(rsp_valid), .rsp_done(rsp_done), .rsp_fail(rsp_fail),
        .rsp_status(rsp_status), .status_invalid(), .status_overflow(),
        .status_underflow(), .status_inexact(), .busy()
    );

    integer i;
    initial clk = 1'b0;
    always #5 clk = ~clk;
    always @(posedge clk) begin
        if (wr_en) begin
            mem_re[wr0] <= wr0_re; mem_im[wr0] <= wr0_im;
            mem_re[wr1] <= wr1_re; mem_im[wr1] <= wr1_im;
        end
    end

    initial begin
        for (i = 0; i < 16; i = i + 1) begin mem_re[i] = 64'd0; mem_im[i] = 64'd0; end
        mem_re[0] = 64'h3ff0000000000000;
        mem_re[1] = 64'h4000000000000000;
        mem_re[2] = 64'h4008000000000000;
        mem_re[3] = 64'h4010000000000000;
        rst_n = 1'b0; cmd_valid = 1'b0; cmd_opcode = 3'd3; cmd_logn = 5'd2;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        cmd_valid = 1'b1;
        @(posedge clk);
        cmd_valid = 1'b0;
        while (!rsp_valid) @(posedge clk);
        for (i = 0; i < 4; i = i + 1)
            $display("%0d re=%h %f im=%h %f", i, mem_re[i], $bitstoreal(mem_re[i]), mem_im[i], $bitstoreal(mem_im[i]));
        $finish;
    end
endmodule
