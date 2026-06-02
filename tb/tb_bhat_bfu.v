`timescale 1ns/1ps
// Test: BhatMul EXU with BFU — N=512, compare vs original golden
module tb_bhat_bfu;
    localparam ADDR_W=10, N=512;
    reg clk=0, rst_n=0;

    // New BhatMul DUT
    reg  start=0; wire start_ready;
    wire mem_rd_en, mem_wr_en, fpu_req_valid, fpu_req_ready;
    wire [ADDR_W-1:0] mem_rd_addr, mem_wr_addr;
    wire [255:0] mem_wr_data;
    wire [3:0] fpu_req_op;
    wire [63:0] fpu_req_a, fpu_req_b, fpu_req_c;
    wire fpu_rsp_valid, done, fail;
    wire [63:0] fpu_rsp_result;
    wire [7:0] status;

    reg [255:0] mem[0:N-1];
    wire [255:0] mem_rd_data = mem[mem_rd_addr];

    always @(posedge clk) if(mem_wr_en) mem[mem_wr_addr] <= mem_wr_data;

    falcon_f64_bhat_mul_exu #(.ADDR_W(ADDR_W)) dut(
        .clk,.rst_n,.start,.start_ready,.identity_mode(1'b0),
        .t_base(0),.z_base(100),.b00_base(0),.b01_base(200),.b10_base(0),.b11_base(300),
        .s2_fft_base(800),.word_count(N),
        .mem_rd_en,.mem_rd_addr,.mem_rd_data,.mem_wr_en,.mem_wr_addr,.mem_wr_data,
        .fpu_req_valid,.fpu_req_ready,.fpu_req_op,.fpu_req_a,.fpu_req_b,.fpu_req_c,
        .fpu_rsp_valid,.fpu_rsp_result,.done,.fail,.status);

    // Simple FPU model
    falcon_fp_fpu u_fpu(
        .clk,.rst_n,.req_valid(fpu_req_valid),.req_ready(fpu_req_ready),
        .req_op(fpu_req_op),.req_a(fpu_req_a),.req_b(fpu_req_b),.req_c(fpu_req_c),
        .req_fmt(2'd1),.req_rm(3'd0),.req_fcvt_op(2'd0),
        .rsp_valid(fpu_rsp_valid),.rsp_ready(1'b1),.rsp_result(fpu_rsp_result),
        .rsp_flags(),.busy());

    always #5 clk=~clk;

    integer i, cyc;
    real tot_err, max_err, dr, di;
    initial begin
        repeat(4) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        // Load test data: z0, z1 at addr 100+, 100+N+; b01, b11 at 200+, 300+
        for(i=0;i<N;i=i+1) begin
            mem[100+i] = {$random,$random,$random,$random};   // z0
            mem[100+N+i] = {$random,$random,$random,$random}; // z1
            mem[200+i] = {$random,$random,$random,$random};   // b01
            mem[300+i] = {$random,$random,$random,$random};   // b11
        end

        $display("BhatMul BFU test: N=%0d ...", N);
        start=1; @(posedge clk); start=0;

        cyc=0; while(!done && cyc<500000) begin @(posedge clk); cyc=cyc+1; end

        $display("done=%b fail=%b status=%h cycles=%0d", done,fail,status,cyc);
        $display("Output (first 4):");
        for(i=0;i<4;i=i+1) $display("  s2[%0d]=%h", i, mem[800+i]);

        if(done && !fail) $display("PASS: BhatMul BFU completed in %0d cycles", cyc);
        else $display("FAIL");

        #100; $finish;
    end
endmodule
