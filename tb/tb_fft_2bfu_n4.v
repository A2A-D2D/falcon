`timescale 1ns/1ps
// N=4 standalone test for 2-BFU FFT EXU (parallel, internal FPUs)
module tb_fft_2bfu_n4;
    localparam ADDR_W = 4;
    localparam N = 4;

    reg clk=0, rst_n=0, cmd_valid=0;
    wire cmd_ready; reg [2:0] cmd_opcode; reg [4:0] cmd_logn;

    wire [ADDR_W-1:0] rd0, rd1, tw_addr, wr0, wr1;
    wire wr_en; wire [63:0] wr0_re, wr0_im, wr1_re, wr1_im;
    wire rsp_valid, rsp_done, rsp_fail;
    wire [7:0] rsp_status;
    wire busy;

    reg [63:0] mre[0:15], mim[0:15];
    always @(posedge clk) begin
        if(wr_en) begin mre[wr0]<=wr0_re; mim[wr0]<=wr0_im; mre[wr1]<=wr1_re; mim[wr1]<=wr1_im; end
    end

    // Parallel 2-BFU EXU (internal FPUs, no external FPU needed)
    falcon_f64_fft_exu_2bfu #(.ADDR_W(ADDR_W)) dut(
        .clk,.rst_n,.cmd_valid,.cmd_ready,.cmd_opcode,.cmd_logn,
        .mem_rd_addr0(rd0),.mem_rd_addr1(rd1),
        .mem_rd_data0_re(mre[rd0]),.mem_rd_data0_im(mim[rd0]),
        .mem_rd_data1_re(mre[rd1]),.mem_rd_data1_im(mim[rd1]),
        .twiddle_addr(tw_addr),.twiddle_re(0),.twiddle_im(0),
        .mem_wr_en(wr_en),.mem_wr_addr0(wr0),.mem_wr_addr1(wr1),
        .mem_wr_data0_re(wr0_re),.mem_wr_data0_im(wr0_im),
        .mem_wr_data1_re(wr1_re),.mem_wr_data1_im(wr1_im),
        .rsp_valid,.rsp_done,.rsp_fail,.rsp_status,
        .status_invalid(),.status_overflow(),.status_underflow(),.status_inexact(),
        .busy());

    always #5 clk=~clk;

    // Golden values (N=4, input [1,2,3,4] -> packed [1+3i,2+4i] -> GM[2]=e^(ipi/4)=0.707+0.707i)
    // f[0]=a+b*s=(1+3i)+(2+4i)*(0.707+0.707i)=(-0.414,7.243)
    // f[1]=a-b*s=(1+3i)-(2+4i)*(0.707+0.707i)=(2.414,-1.243)
    // mirror: f[2]=conj(f[0])=(-0.414,-7.243), f[3]=conj(f[1])=(2.414,1.243)
    real gold_re[0:3], gold_im[0:3];
    integer g;
    initial begin
        gold_re[0]=-0.414213562373095; gold_im[0]= 7.24264068711929;
        gold_re[1]= 2.414213562373095; gold_im[1]=-1.24264068711929;
        gold_re[2]= 2.414213562373095; gold_im[2]= 1.24264068711929; // conj(f[1])
        gold_re[3]=-0.414213562373095; gold_im[3]=-7.24264068711929; // conj(f[0])
    end

    integer cyc, err;
    initial begin
        // Load test data: c = [1,2,3,4] as real FP64 values
        mre[0]=64'h3FF0000000000000; mim[0]=0; // c[0]=1.0
        mre[1]=64'h4000000000000000; mim[1]=0; // c[1]=2.0
        mre[2]=64'h4008000000000000; mim[2]=0; // c[2]=3.0
        mre[3]=64'h4010000000000000; mim[3]=0; // c[3]=4.0

        repeat(4) @(posedge clk); rst_n=1; repeat(2) @(posedge clk);

        // Start Falcon FWD FFT (opcode=3'd3)
        cmd_opcode=3'd3; cmd_logn=5'd2; cmd_valid=1;
        @(posedge clk); cmd_valid=0;

        // Wait for done
        cyc=0; while(!rsp_valid && cyc<10000) begin @(posedge clk); cyc=cyc+1; end

        $display("2-BFU (parallel) N=4 FFT done in %0d cycles: done=%b fail=%b", cyc, rsp_done, rsp_fail);
        $display("Results:");
        err=0;
        for(g=0;g<4;g=g+1) begin
            $display("  f[%0d]: re=%h (%.6f) im=%h (%.6f) | gold: (%.6f, %.6f)",
                g, mre[g], $bitstoreal(mre[g]), mim[g], $bitstoreal(mim[g]),
                gold_re[g], gold_im[g]);
        end
        for(g=0;g<4;g=g+1) begin
            if($bitstoreal(mre[g])<gold_re[g]-0.001 || $bitstoreal(mre[g])>gold_re[g]+0.001 ||
               $bitstoreal(mim[g])<gold_im[g]-0.001 || $bitstoreal(mim[g])>gold_im[g]+0.001) err=err+1;
        end
        if(err==0) $display("PASS: all outputs match golden within 0.001 tolerance");
        else $display("FAIL: %0d mismatches", err);
        @(posedge clk); $finish;
    end

    // Dump FSM state transitions + bf2_out_valid edge
    reg [5:0] prev_state; reg prev_bf2_out;
    initial begin prev_state = 99; prev_bf2_out = 0; end
    integer trace_cyc;
    initial trace_cyc = 0;
    always @(posedge clk) if(rst_n) trace_cyc <= trace_cyc + 1;
    always @(posedge clk) begin
        if(rst_n && dut.u_bfly2.out_valid != prev_bf2_out)
            $display("  T=%0t cy=%0d BFU2 out_valid: %b→%b",
                $time, trace_cyc, prev_bf2_out, dut.u_bfly2.out_valid);
        prev_bf2_out <= dut.u_bfly2.out_valid;
        if(rst_n && dut.state != prev_state) begin
            $display("  T=%0t cy=%0d st→%0d(%s) bsub=%0d pbase=%0d stage=%0d idx=%0d",
                $time, trace_cyc, dut.state,
                dut.state==0?"IDLE":dut.state==1?"PACK_RD":dut.state==2?"PACK_WR":
                dut.state==3?"BF_INIT":dut.state==4?"BATCH_RD":dut.state==5?"BFLY_RUN":dut.state==6?"BATCH_WR":
                dut.state==7?"MR_INIT":dut.state==8?"MIR_RD":dut.state==9?"MIR_WR":"OTHER",
                dut.batch_sub, dut.pair_base, dut.stage_idx, dut.fal_idx);
            prev_state <= dut.state;
        end
        // Show memory operations
        if(rst_n && wr_en) $display("    WR en=%b a0=%0d a1=%0d d0=(%h,%h) d1=(%h,%h)", wr_en, wr0, wr1, wr0_re, wr0_im, wr1_re, wr1_im);
        if(rst_n && dut.state==4) $display("    BATCH_RD: rd0=%0d(data=%h,%h) rd1=%0d(data=%h,%h)", rd0, mre[rd0], mim[rd0], rd1, mre[rd1], mim[rd1]);
    end
endmodule
