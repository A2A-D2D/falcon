`timescale 1ns/1ps

module tb_norm_debug_mem;

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

    // Status
    wire        busy;
    wire        done;
    wire        fail;
    wire [7:0]  status;

    falconsign_top #(.ADDR_W(13), .LEVEL_W(4), .INDEX_W(10), .BFU_LANES(1)) dut (
        .clk(clk), .rst_n(rst_n),
        .bus_cs(bus_cs), .bus_wr(bus_wr),
        .bus_addr(bus_addr), .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata), .bus_ready(bus_ready),
        .bus_irq(bus_irq),
        .busy(busy), .done(done), .fail(fail), .status(status)
    );

    // Task to read memory
    task read_mem_word;
        input integer word_addr;
        output [255:0] data;
        begin
            case (word_addr & 3)
                0: data = dut.u_mem.bank0[word_addr >> 2];
                1: data = dut.u_mem.bank1[word_addr >> 2];
                2: data = dut.u_mem.bank2[word_addr >> 2];
                default: data = dut.u_mem.bank3[word_addr >> 2];
            endcase
        end
    endtask

    // Test sequence
    integer i;
    reg [255:0] mem_data;
    reg signed [15:0] coeff;
    reg [63:0] s2_norm, s1_norm;
    reg signed [31:0] coeff_centered;

    initial begin
        rst_n = 0;
        bus_cs = 0;
        bus_wr = 0;
        bus_addr = 0;
        bus_wdata = 0;

        #30 rst_n = 1;
        #20;

        // Load key material (same as smoke test)
        $display("=== Loading key material ===");
        // ... (hex loading would go here)

        // Start signing
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

        // Read s2 values (LAYOUT_SIG_BASE = 6912)
        $display("");
        $display("=== s2 values (first 32 coefficients) ===");
        s2_norm = 0;
        for (i = 0; i < 2; i = i + 1) begin
            read_mem_word(6912 + i, mem_data);
            $display("Word %0d: %h", i, mem_data);
            
            // Decode coefficients
            for (integer j = 0; j < 16; j = j + 1) begin
                coeff = mem_data[j*16 +: 16];
                $display("  s2[%0d] = %0d", i*16 + j, coeff);
                s2_norm = s2_norm + $unsigned(coeff * coeff);
            end
        end
        $display("s2 partial norm (32 coeffs) = %0d", s2_norm);

        // Read s1 values (LAYOUT_S1_BASE = 7488)
        $display("");
        $display("=== s1 values (first 32 coefficients) ===");
        s1_norm = 0;
        for (i = 0; i < 2; i = i + 1) begin
            read_mem_word(7488 + i, mem_data);
            $display("Word %0d: %h", i, mem_data);
            
            // Decode coefficients with center-lifting
            for (integer j = 0; j < 16; j = j + 1) begin
                coeff = mem_data[j*16 +: 16];
                if (coeff > 16'sd6144) begin
                    coeff_centered = $signed({16'd0, coeff}) - 32'sd12289;
                end else begin
                    coeff_centered = $signed({16'd0, coeff});
                end
                $display("  s1[%0d] = %0d -> %0d", i*16 + j, coeff, coeff_centered);
                s1_norm = s1_norm + $unsigned(coeff_centered * coeff_centered);
            end
        end
        $display("s1 partial norm (32 coeffs) = %0d", s1_norm);

        $display("");
        $display("=== Summary ===");
        $display("Total partial norm (32 s2 + 32 s1) = %0d", s2_norm + s1_norm);
        $display("Estimated full norm (512 coeffs) = %0d", (s2_norm + s1_norm) * 8);

        #100;
        $finish;
    end

    // Timeout
    initial begin
        #10000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
