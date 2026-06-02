`timescale 1ns/1ps
module tb_fe_debug;
    localparam ADDR_W=14;
    reg clk=0, rst_n=0;
    reg task_valid=0; wire task_ready, task_done, task_fail; wire [7:0] task_status;
    wire mem_rd_en, mem_wr_en, fpu_req_valid, sz_cmd_valid;
    wire [ADDR_W-1:0] mem_rd_addr, mem_wr_addr;

    reg [255:0] mem[0:1023];
    wire [255:0] mem_rd_data = mem[mem_rd_addr];
    wire [63:0] twiddle_re=64'd0, twiddle_im=64'd0;
    wire fpu_req_ready=1, fpu_rsp_valid=0;
    wire [63:0] fpu_rsp_result=0;
    wire sz_cmd_ready=0, sz_rsp_valid=0;
    wire [63:0] sz_rsp_z0=0, sz_rsp_z1=0;

    always @(posedge clk) if(mem_wr_en) mem[mem_wr_addr]<=256'd0;

    falcon_f64_ffsampling_exu #(.ADDR_W(ADDR_W)) dut(
        .clk,.rst_n,.task_valid,.task_ready,
        .task_word({4'd4, 4'd1, 10'd0, 14'd100, 14'd200, 14'd300, 8'd0}),
        .task_done,.task_fail,.task_status,
        .mem_rd_en,.mem_rd_addr,.mem_rd_data,
        .mem_wr_en,.mem_wr_addr,.mem_wr_data(),
        .twiddle_addr(),.twiddle_re,.twiddle_im,
        .fpu_req_valid,.fpu_req_ready,.fpu_req_op(),.fpu_req_a(),.fpu_req_b(),.fpu_req_c(),
        .fpu_rsp_valid,.fpu_rsp_result,
        .sz_cmd_valid,.sz_cmd_ready,.sz_cmd_mu(),.sz_cmd_sigma_inv(),.sz_cmd_pair(),
        .sz_rsp_valid,.sz_rsp_z0,.sz_rsp_z1);

    always #5 clk=~clk;

    reg [31:0] cyc;
    always @(posedge clk) if(rst_n) cyc<=cyc+1; else cyc<=0;

    always @(posedge clk) begin
        if(rst_n && cyc>0 && (cyc % 500 == 0))
            $display("cy=%0d st=%0d op=%0d lvl=%0d idx=%0d plt=%0d", cyc, dut.state, dut.op_q, dut.level_q, dut.idx_q, dut.pair_limit_q);
    end

    initial begin
        repeat(5) @(posedge clk); rst_n=1;
        repeat(5) @(posedge clk);
        task_valid=1; @(posedge clk); task_valid=0;
        while(!task_done && !task_fail && cyc<5000) @(posedge clk);
        $display("Done: cy=%0d done=%b fail=%b st=%0d", cyc, task_done, task_fail, dut.state);
        $finish;
    end
endmodule
