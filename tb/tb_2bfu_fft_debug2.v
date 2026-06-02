`timescale 1ns/1ps

module tb_2bfu_fft_debug2;

    reg clk, rst_n;
    initial begin clk = 0; forever #5 clk = ~clk; end

    // Bus interface
    reg         bus_cs;
    reg         bus_wr;
    reg  [15:0] bus_addr;
    reg  [31:0] bus_wdata;
    wire [31:0] bus_rdata;
    wire        bus_ready;
    wire        bus_irq;
    wire        busy, done, fail;
    wire [7:0]  status;

    // Instantiate top with 2-BFU
    falconsign_top #(.ADDR_W(13), .LEVEL_W(4), .INDEX_W(10), .BFU_LANES(2)) dut (
        .clk(clk), .rst_n(rst_n),
        .bus_cs(bus_cs), .bus_wr(bus_wr),
        .bus_addr(bus_addr), .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata), .bus_ready(bus_ready),
        .bus_irq(bus_irq),
        .busy(busy), .done(done), .fail(fail), .status(status)
    );

    // Track phase transitions
    reg [3:0] prev_st;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_st <= 0;
        end else begin
            if (dut.st != prev_st) begin
                $display("T=%0t Phase: %0d -> %0d", $time, prev_st, dut.st);
                prev_st <= dut.st;
            end
        end
    end

    // Debug FFT phase - just print when FFT is active
    reg [31:0] fft_cycle_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fft_cycle_cnt <= 0;
        end else if (dut.st == 3) begin  // FC_FFT
            fft_cycle_cnt <= fft_cycle_cnt + 1;
            if (fft_cycle_cnt < 20) begin
                $display("T=%0t FFT active, cycle=%0d", $time, fft_cycle_cnt);
            end
        end else begin
            fft_cycle_cnt <= 0;
        end
    end

    // Test sequence
    initial begin
        rst_n = 0;
        bus_cs = 0;
        bus_wr = 0;
        bus_addr = 0;
        bus_wdata = 0;

        #30 rst_n = 1;
        #20;

        // Start signing (CASE=1)
        $display("=== Starting signing ===");
        @(posedge clk);
        bus_cs = 1;
        bus_wr = 1;
        bus_addr = 16'h0008;
        bus_wdata = 32'h00000002;  // force_accept=1
        @(posedge clk);
        bus_cs = 0;
        bus_wr = 0;

        @(posedge clk);
        bus_cs = 1;
        bus_wr = 1;
        bus_addr = 16'h0000;
        bus_wdata = 32'h00000001;  // start
        @(posedge clk);
        bus_cs = 0;
        bus_wr = 0;

        // Wait for completion
        wait(done || fail);
        @(posedge clk);

        $display("=== Signing complete ===");
        $display("done=%0d, fail=%0d, status=%0d", done, fail, status);
        $display("norm_sq=%0d, bound=%0d", dut.norm_sq, dut.FALCON512_BOUND_SQ);

        #100;
        $finish;
    end

    // Timeout
    initial begin
        #5000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
