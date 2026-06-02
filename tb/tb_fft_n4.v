`timescale 1ns/1ps
// Minimal N=4 Falcon forward FFT test — compare with C golden
module tb_fft_n4;
    localparam ADDR_W = 4;
    localparam N = 4;

    reg clk, rst_n, cmd_valid;
    wire cmd_ready;
    reg [2:0] cmd_opcode;
    reg [4:0] cmd_logn;

    wire [ADDR_W-1:0] rd0, rd1, tw_addr, wr0, wr1;
    wire wr_en;
    wire [63:0] wr0_re, wr0_im, wr1_re, wr1_im;
    wire rsp_valid, rsp_done, rsp_fail;
    wire [7:0] rsp_status;
    wire busy;

    // Simple memory
    reg [63:0] mem_re [0:15];
    reg [63:0] mem_im [0:15];
    wire [63:0] rd0_re = mem_re[rd0];
    wire [63:0] rd0_im = mem_im[rd0];
    wire [63:0] rd1_re = mem_re[rd1];
    wire [63:0] rd1_im = mem_im[rd1];

    // twiddle ROM mock — needed for module instantiation
    wire [63:0] tw_re, tw_im;
    assign tw_re = 64'd0;
    assign tw_im = 64'd0;

    falcon_f64_fft_exu #(.ADDR_W(ADDR_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_opcode(cmd_opcode), .cmd_logn(cmd_logn),
        .mem_rd_addr0(rd0), .mem_rd_addr1(rd1),
        .mem_rd_data0_re(rd0_re), .mem_rd_data0_im(rd0_im),
        .mem_rd_data1_re(rd1_re), .mem_rd_data1_im(rd1_im),
        .twiddle_addr(tw_addr), .twiddle_re(tw_re), .twiddle_im(tw_im),
        .mem_wr_en(wr_en), .mem_wr_addr0(wr0), .mem_wr_addr1(wr1),
        .mem_wr_data0_re(wr0_re), .mem_wr_data0_im(wr0_im),
        .mem_wr_data1_re(wr1_re), .mem_wr_data1_im(wr1_im),
        .rsp_valid(rsp_valid), .rsp_done(rsp_done), .rsp_fail(rsp_fail),
        .rsp_status(rsp_status),
        .status_invalid(), .status_overflow(),
        .status_underflow(), .status_inexact(), .busy(busy)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (wr_en) begin
            mem_re[wr0] <= wr0_re; mem_im[wr0] <= wr0_im;
            mem_re[wr1] <= wr1_re; mem_im[wr1] <= wr1_im;
        end
    end

    // Test: N=4 real input [1, 2, 3, 4]
    // Packed complex: [1+3i, 2+4i]
    // FFT of size 2:
    //   f[0] = (1+3i) + (2+4i)*1 = 3+7i
    //   f[1] = (1+3i) - (2+4i)*1 = -1-1i
    // Mirror: f[2] = conj(f[0]) = 3-7i, f[3] = conj(f[1]) = -1+i
    initial begin
        // Load test data in Falcon FFT memory format
        // c at addr 0..3: each word = {0, 0, 0, c[idx]}
        mem_re[0] = 64'h3FF0000000000000; mem_im[0] = 64'd0; // c[0]=1.0
        mem_re[1] = 64'h4000000000000000; mem_im[1] = 64'd0; // c[1]=2.0
        mem_re[2] = 64'h4008000000000000; mem_im[2] = 64'd0; // c[2]=3.0
        mem_re[3] = 64'h4010000000000000; mem_im[3] = 64'd0; // c[3]=4.0

        rst_n = 1'b0; cmd_valid = 1'b0;
        cmd_opcode = 3'd3; cmd_logn = 5'd2;  // Falcon FWD, logn=2 (N=4)
        repeat(4) @(posedge clk);
        rst_n = 1'b1;
        repeat(2) @(posedge clk);

        // Start FFT
        cmd_valid = 1'b1;
        @(posedge clk);
        cmd_valid = 1'b0;

        // Wait for done
        while (!rsp_valid) @(posedge clk);

        $display("FFT N=4 results:");
        for (int i = 0; i < 4; i++) begin
            $display("  f[%0d]: re=%h (%.6f)  im=%h (%.6f)",
                i, mem_re[i], $bitstoreal(mem_re[i]),
                mem_im[i], $bitstoreal(mem_im[i]));
        end

        // Expected (C golden):
        // After pack: addr0={c[2],c[0]}={3,1}, addr1={c[3],c[1]}={4,2}
        // GM[2] for N=4: w = exp(ipi/2) = i = (0, 1)
        // Butterfly: f[0]=a+b*i=(1+3i)+(2+4i)*i=(1+3i)+(2i-4)=(-3+5i)
        //            f[1]=a-b*i=(1+3i)-(2+4i)*i=(1+3i)-(2i-4)=(5+i)
        // Mirror: f[2]=conj(f[0])=(-3-5i), f[3]=conj(f[1])=(5-i)
        $display("");
        $display("Expected (C golden):");
        $display("  f[0]: re=-3.0 im=5.0");
        $display("  f[1]: re=5.0 im=1.0");
        $display("  f[2]: re=-3.0 im=-5.0");
        $display("  f[3]: re=5.0 im=-1.0");

        #100;
        if (mem_re[0] == 64'hC008000000000000 && mem_im[0] == 64'h4014000000000000)
            $display("PASS: f[0] matches expected!");
        else
            $display("FAIL: f[0] mismatch");

        $finish;
    end
endmodule
