`timescale 1ns/1ps

module tb_fft_compare;

    reg clk, rst_n;
    initial begin clk = 0; forever #5 clk = ~clk; end

    // Test FFT with known input
    reg         cmd_valid;
    wire        cmd_ready;
    reg  [2:0]  cmd_opcode;
    reg  [4:0]  cmd_logn;

    // Memory interface
    wire        mem_rd_en;
    wire [12:0] mem_rd_addr0, mem_rd_addr1;
    reg  [63:0] mem_rd_data0_re, mem_rd_data0_im;
    reg  [63:0] mem_rd_data1_re, mem_rd_data1_im;

    wire        mem_wr_en;
    wire [12:0] mem_wr_addr0, mem_wr_addr1;
    wire [63:0] mem_wr_data0_re, mem_wr_data0_im;
    wire [63:0] mem_wr_data1_re, mem_wr_data1_im;

    // Twiddle interface
    wire [12:0] twiddle_addr;
    wire [63:0] twiddle_re, twiddle_im;

    // Response
    wire        rsp_valid, rsp_done, rsp_fail;
    wire [7:0]  rsp_status;
    wire        busy;

    // Memory array
    reg [63:0] mem_re [0:511];
    reg [63:0] mem_im [0:511];

    // Instantiate 1-BFU FFT
    falcon_f64_fft_exu #(.ADDR_W(13)) u_fft_1bfu (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_opcode(cmd_opcode), .cmd_logn(cmd_logn),
        .mem_rd_addr0(mem_rd_addr0), .mem_rd_addr1(mem_rd_addr1),
        .mem_rd_data0_re(mem_rd_data0_re), .mem_rd_data0_im(mem_rd_data0_im),
        .mem_rd_data1_re(mem_rd_data1_re), .mem_rd_data1_im(mem_rd_data1_im),
        .twiddle_addr(twiddle_addr), .twiddle_re(twiddle_re), .twiddle_im(twiddle_im),
        .mem_wr_en(mem_wr_en), .mem_wr_addr0(mem_wr_addr0), .mem_wr_addr1(mem_wr_addr1),
        .mem_wr_data0_re(mem_wr_data0_re), .mem_wr_data0_im(mem_wr_data0_im),
        .mem_wr_data1_re(mem_wr_data1_re), .mem_wr_data1_im(mem_wr_data1_im),
        .rsp_valid(rsp_valid), .rsp_done(rsp_done), .rsp_fail(rsp_fail),
        .rsp_status(rsp_status), .busy(busy),
        .status_invalid(), .status_overflow(), .status_underflow(), .status_inexact()
    );

    // Twiddle ROM
    falconsign_twiddle_rom #(.ADDR_W(8), .DEPTH(256)) u_tw (
        .clk(clk), .addr(twiddle_addr[7:0]),
        .twiddle_re(twiddle_re), .twiddle_im(twiddle_im));

    // Memory read logic
    always @(*) begin
        mem_rd_data0_re = mem_re[mem_rd_addr0];
        mem_rd_data0_im = mem_im[mem_rd_addr0];
        mem_rd_data1_re = mem_re[mem_rd_addr1];
        mem_rd_data1_im = mem_im[mem_rd_addr1];
    end

    // Memory write logic
    always @(posedge clk) begin
        if (mem_wr_en) begin
            mem_re[mem_wr_addr0] <= mem_wr_data0_re;
            mem_im[mem_wr_addr0] <= mem_wr_data0_im;
            mem_re[mem_wr_addr1] <= mem_wr_data1_re;
            mem_im[mem_wr_addr1] <= mem_wr_data1_im;
        end
    end

    // Test sequence
    integer i;
    initial begin
        rst_n = 0;
        cmd_valid = 0;
        cmd_opcode = 0;
        cmd_logn = 0;

        #30 rst_n = 1;
        #20;

        // Initialize memory with simple pattern
        $display("=== Initializing memory ===");
        for (i = 0; i < 512; i = i + 1) begin
            mem_re[i] = $realtobits($itor(i) * 0.001);
            mem_im[i] = 64'd0;
        end

        // Run forward FFT
        $display("=== Running forward FFT ===");
        cmd_valid = 1;
        cmd_opcode = 3'd3;  // OP_FFT_FALCON_FWD
        cmd_logn = 5'd9;    // N=512
        @(posedge clk);
        cmd_valid = 0;

        // Wait for completion
        wait(rsp_done || rsp_fail);
        @(posedge clk);

        $display("FFT done: rsp_done=%0d, rsp_fail=%0d, status=%0d", 
                 rsp_done, rsp_fail, rsp_status);

        // Check first few output values
        $display("");
        $display("=== FFT output (first 8 values) ===");
        for (i = 0; i < 8; i = i + 1) begin
            $display("  [%0d] = %h + %hi", i, mem_re[i], mem_im[i]);
        end

        #100;
        $display("=== Test Complete ===");
        $finish;
    end

    // Timeout
    initial begin
        #100000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
