`timescale 1ns/1ps

module tb_fft_simple;

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

    // Instantiate top with 1-BFU
    falconsign_top #(.ADDR_W(13), .LEVEL_W(4), .INDEX_W(10), .BFU_LANES(1)) dut (
        .clk(clk), .rst_n(rst_n),
        .bus_cs(bus_cs), .bus_wr(bus_wr),
        .bus_addr(bus_addr), .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata), .bus_ready(bus_ready),
        .bus_irq(bus_irq),
        .busy(busy), .done(done), .fail(fail), .status(status)
    );

    // Task to read memory
    task read_mem;
        input integer addr;
        output [255:0] data;
        begin
            case (addr & 3)
                0: data = dut.u_mem.bank0[addr >> 2];
                1: data = dut.u_mem.bank1[addr >> 2];
                2: data = dut.u_mem.bank2[addr >> 2];
                default: data = dut.u_mem.bank3[addr >> 2];
            endcase
        end
    endtask

    // Task to write memory
    task write_mem;
        input integer addr;
        input [255:0] data;
        begin
            case (addr & 3)
                0: dut.u_mem.bank0[addr >> 2] = data;
                1: dut.u_mem.bank1[addr >> 2] = data;
                2: dut.u_mem.bank2[addr >> 2] = data;
                default: dut.u_mem.bank3[addr >> 2] = data;
            endcase
        end
    endtask

    // Test sequence
    integer i;
    reg [255:0] mem_data;

    initial begin
        rst_n = 0;
        bus_cs = 0;
        bus_wr = 0;
        bus_addr = 0;
        bus_wdata = 0;

        #30 rst_n = 1;
        #20;

        // Initialize memory with simple pattern: c[i] = i (as real, imag=0)
        $display("=== Initializing memory ===");
        for (i = 0; i < 512; i = i + 1) begin
            write_mem(i, {128'd0, 64'd0, $realtobits($itor(i))});
        end

        // Verify initialization
        $display("First 4 values:");
        for (i = 0; i < 4; i = i + 1) begin
            read_mem(i, mem_data);
            $display("  [%0d] = %h + %hi", i, mem_data[63:0], mem_data[127:64]);
        end

        // Run forward FFT (opcode=3, logn=9)
        $display("");
        $display("=== Running forward FFT ===");
        @(posedge clk);
        bus_cs = 1;
        bus_wr = 1;
        bus_addr = 16'h0008;
        bus_wdata = 32'h00000000;  // No special config
        @(posedge clk);
        bus_cs = 0;
        bus_wr = 0;

        // Start FFT via direct command
        // Actually, we need to use the proper signing flow
        // Let me just check if the FFT module works correctly

        #1000;
        $display("=== Test Complete ===");
        $finish;
    end

endmodule
