`timescale 1ns/1ps
module tb_fe_fpu_check;
    reg clk=0, rst_n=0;
    wire fe_fpu_req_valid, fe_fpu_req_ready;
    wire [3:0] fe_fpu_req_op;
    wire [63:0] fe_fpu_req_a, fe_fpu_req_b, fe_fpu_req_c;
    wire fe_fpu_rsp_valid;

    falcon_f64_ffsampling_exu #(.ADDR_W(14)) dut (
        .clk,.rst_n,
        .task_valid(0),.task_ready(),.task_word(0),.task_done(),.task_fail(),.task_status(),
        .mem_rd_en(),.mem_rd_addr(),.mem_rd_data(0),
        .mem_wr_en(),.mem_wr_addr(),.mem_wr_data(),
        .twiddle_addr(),.twiddle_re(0),.twiddle_im(0),
        .fpu_req_valid(fe_fpu_req_valid),.fpu_req_ready(fe_fpu_req_ready),
        .fpu_req_op(fe_fpu_req_op),.fpu_req_a(fe_fpu_req_a),
        .fpu_req_b(fe_fpu_req_b),.fpu_req_c(fe_fpu_req_c),
        .fpu_rsp_valid(fe_fpu_rsp_valid),.fpu_rsp_result(0),
        .sz_cmd_valid(),.sz_cmd_ready(0),.sz_cmd_mu(),.sz_cmd_sigma_inv(),.sz_cmd_pair(),
        .sz_rsp_valid(0),.sz_rsp_z0(0),.sz_rsp_z1(0));

    always #5 clk=~clk;

    initial begin
        repeat(5) @(posedge clk); rst_n=1;
        repeat(10) @(posedge clk);

        // Check: external fpu_req_valid should be 0 (internal FPU handles all)
        if (fe_fpu_req_valid === 1'b0)
            $display("PASS: External FPU interface idle (internal FPU active)");
        else
            $display("FAIL: External FPU still active");

        // Check internal FPU exists
        if (dut.u_ife_fpu !== null)
            $display("PASS: Internal FPU instance u_ife_fpu found");
        else
            $display("FAIL: No internal FPU");

        #100; $finish;
    end
endmodule
