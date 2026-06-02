`timescale 1ns/1ps

module tb_bfu_signal_debug;

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

    // Debug BFU signals during FFT phase
    always @(posedge clk) begin
        if (dut.st == 3) begin  // FC_FFT
            // Print BFU 0 signals
            $display("T=%0t BFU0: in_v=%b in_r=%b out_v=%b | BFU1: in_v=%b in_r=%b out_v=%b",
                     $time, 
                     dut.sh_bfu0_in_v, dut.sh_bfu0_in_r, dut.sh_bfu0_out_v,
                     dut.sh_bfu1_in_v, dut.sh_bfu1_in_r, dut.sh_bfu1_out_v);
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

        // Wait for FFT phase to start
        wait(dut.st == 3);
        #50000;  // Print 5000 cycles of debug
        $display("=== Stopping debug ===");
        $finish;
    end

endmodule
