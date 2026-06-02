`timescale 1ns/1ps
module tb_bhatmul_debug;
    reg clk, rst_n;
    initial begin clk=0; forever #5 clk=~clk; end
    
    // Instantiate just the BhatMul with memory
    reg start;
    wire start_ready, done, fail;
    wire [7:0] status;
    wire mem_rd_en, mem_wr_en;
    wire [12:0] mem_rd_addr, mem_wr_addr;
    reg [255:0] mem_rd_data;
    wire [255:0] mem_wr_data;
    
    // FPU signals
    wire fpu_req_valid, fpu_req_ready;
    wire [3:0] fpu_req_op;
    wire [63:0] fpu_req_a, fpu_req_b, fpu_req_c;
    wire fpu_rsp_valid;
    wire [63:0] fpu_rsp_result;
    
    // BFU signals
    wire bf0_in_v, bf0_in_r, bf0_out_v;
    wire [63:0] bf0_b_re, bf0_b_im, bf0_w_re, bf0_w_im;
    wire [63:0] bf0_y0r, bf0_y0i;
    wire bf1_in_v, bf1_in_r, bf1_out_v;
    wire [63:0] bf1_b_re, bf1_b_im, bf1_w_re, bf1_w_im;
    wire [63:0] bf1_y0r, bf1_y0i;
    
    falcon_f64_bhat_mul_exu #(.ADDR_W(13)) u_vd (
        .clk(clk), .rst_n(rst_n),
        .start(start), .start_ready(start_ready),
        .identity_mode(1'b0),
        .t_base(0), .z_base(0), .b00_base(1000), .b01_base(1500),
        .b10_base(2000), .b11_base(2500), .s2_fft_base(3000),
        .word_count(16),  // Small test: 16 coefficients
        .mem_rd_en(mem_rd_en), .mem_rd_addr(mem_rd_addr),
        .mem_rd_data(mem_rd_data),
        .mem_wr_en(mem_wr_en), .mem_wr_addr(mem_wr_addr),
        .mem_wr_data(mem_wr_data),
        .fpu_req_valid(fpu_req_valid), .fpu_req_ready(fpu_req_ready),
        .fpu_req_op(fpu_req_op), .fpu_req_a(fpu_req_a),
        .fpu_req_b(fpu_req_b), .fpu_req_c(fpu_req_c),
        .fpu_rsp_valid(fpu_rsp_valid), .fpu_rsp_result(fpu_rsp_result),
        .done(done), .fail(fail), .status(status),
        .vd_bfu0_in_v(bf0_in_v), .vd_bfu0_in_r(bf0_in_r),
        .vd_bfu0_b_re(bf0_b_re), .vd_bfu0_b_im(bf0_b_im),
        .vd_bfu0_w_re(bf0_w_re), .vd_bfu0_w_im(bf0_w_im),
        .vd_bfu0_out_v(bf0_out_v), .vd_bfu0_y0r(bf0_y0r), .vd_bfu0_y0i(bf0_y0i),
        .vd_bfu1_in_v(bf1_in_v), .vd_bfu1_in_r(bf1_in_r),
        .vd_bfu1_b_re(bf1_b_re), .vd_bfu1_b_im(bf1_b_im),
        .vd_bfu1_w_re(bf1_w_re), .vd_bfu1_w_im(bf1_w_im),
        .vd_bfu1_out_v(bf1_out_v), .vd_bfu1_y0r(bf1_y0r), .vd_bfu1_y0i(bf1_y0i)
    );
    
    // Simple FPU model
    assign fpu_req_ready = 1'b1;
    reg [3:0] fpu_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin fpu_cnt<=0; end
        else begin
            if (fpu_req_valid && fpu_req_ready) fpu_cnt <= 1;
            else if (fpu_cnt > 0 && fpu_cnt < 4) fpu_cnt <= fpu_cnt + 1;
            else fpu_cnt <= 0;
        end
    end
    assign fpu_rsp_valid = (fpu_cnt == 3);
    assign fpu_rsp_result = 64'h3FF0000000000000; // 1.0
    
    // Simple BFU models (10 cycle latency)
    reg [3:0] bf0_cnt, bf1_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin bf0_cnt<=0; bf1_cnt<=0; end
        else begin
            if (bf0_in_v && bf0_in_r) bf0_cnt <= 1;
            else if (bf0_cnt > 0 && bf0_cnt < 10) bf0_cnt <= bf0_cnt + 1;
            else bf0_cnt <= 0;
            
            if (bf1_in_v && bf1_in_r) bf1_cnt <= 1;
            else if (bf1_cnt > 0 && bf1_cnt < 10) bf1_cnt <= bf1_cnt + 1;
            else bf1_cnt <= 0;
        end
    end
    assign bf0_in_r = (bf0_cnt == 0);
    assign bf0_out_v = (bf0_cnt == 9);
    assign bf0_y0r = 64'h3FF0000000000000;
    assign bf0_y0i = 64'h3FF0000000000000;
    assign bf1_in_r = (bf1_cnt == 0);
    assign bf1_out_v = (bf1_cnt == 9);
    assign bf1_y0r = 64'h3FF0000000000000;
    assign bf1_y0i = 64'h3FF0000000000000;
    
    // Memory model
    always @(posedge clk) begin
        mem_rd_data <= 256'h0100000000000000_0100000000000000_0100000000000000_0100000000000000;
    end
    
    // Track cycles
    reg [31:0] cycle_cnt, start_cycle;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin cycle_cnt<=0; end
        else cycle_cnt <= cycle_cnt + 1;
    end
    
    // Debug BhatMul state
    always @(posedge clk) begin
        if (u_vd.state != 5'd0) begin // Not IDLE
            $display("cy=%0d state=%0d rd=%0d fp=%0d idx=%0d bf_launch=%b bf0_done=%b bf1_done=%b",
                     cycle_cnt, u_vd.state, u_vd.rd_phase, u_vd.fp_phase, u_vd.idx,
                     u_vd.bf_launch, u_vd.bf0_done, u_vd.bf1_done);
        end
    end
    
    initial begin
        rst_n = 0; start = 0;
        #30 rst_n = 1;
        #20;
        
        $display("Starting BhatMul test with 16 coefficients...");
        start_cycle = cycle_cnt;
        start = 1;
        @(posedge clk);
        start = 0;
        
        wait(done || fail || cycle_cnt > 10000);
        $display("Done: cycles=%0d fail=%b status=%0d", cycle_cnt - start_cycle, fail, status);
        #100;
        $finish;
    end
endmodule
