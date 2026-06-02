`timescale 1ns/1ps
module tb_phase_debug;
    reg clk; initial begin clk=0; forever #5 clk=~clk; end
    reg rst_n; initial begin rst_n=0; #30 rst_n=1; end

    reg bus_cs, bus_wr; reg [15:0] bus_addr; reg [31:0] bus_wdata;
    wire [31:0] bus_rdata; wire bus_irq, busy, done, fail; wire [7:0] status;

    falconsign_top #(.ADDR_W(13), .LEVEL_W(4), .INDEX_W(10), .BFU_LANES(2)) dut (
        .clk(clk), .rst_n(rst_n), .bus_cs(bus_cs), .bus_wr(bus_wr),
        .bus_addr(bus_addr), .bus_wdata(bus_wdata), .bus_rdata(bus_rdata),
        .bus_ready(bus_ready), .bus_irq(bus_irq), .busy(busy), .done(done), .fail(fail), .status(status)
    );

    task bus_write; input [15:0] a; input [31:0] d;
    begin @(posedge clk); bus_cs=1; bus_wr=1; bus_addr=a; bus_wdata=d;
           @(posedge clk); bus_cs=0; bus_wr=0; wait(bus_ready); @(posedge clk); end
    endtask

    task load_hex; input [1024*8-1:0] fn; input integer base; input integer cnt;
    integer fd, n, addr; reg [255:0] w;
    begin fd=$fopen(fn,"r"); if(!fd) begin $display("ERR: %s",fn); $finish; end
          for(addr=0;addr<cnt;addr=addr+1) begin n=$fscanf(fd,"%h\n",w);
            case((base+addr)&3) 0:dut.u_mem.bank0[(base+addr)>>2]=w; 1:dut.u_mem.bank1[(base+addr)>>2]=w;
            2:dut.u_mem.bank2[(base+addr)>>2]=w; default:dut.u_mem.bank3[(base+addr)>>2]=w; endcase end
          $fclose(fd); end
    endtask

    integer cyc; reg [3:0] prev_st;

    initial begin
        bus_cs=0; bus_wr=0; bus_addr=0; bus_wdata=0; prev_st=0;
        repeat(8) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);
        // Load key material
        load_hex("b00.hex", 4864, 512); load_hex("b01.hex", 5376, 512);
        load_hex("b10.hex", 5888, 512); load_hex("b11.hex", 6400, 512);
        load_hex("tree_full_poly.hex", 1024, 2816);
        load_hex("h_ntt.hex", 7456, 32); load_hex("hm.hex", 7424, 32);
        load_hex("t0_target.hex", 0, 512); load_hex("t1_target.hex", 512, 512);
        $display("=== CASE=0: START_AT_FS + real rejection ===");
        bus_write(16'h0008, 32'h00000004); // start_at_fs=1
        bus_write(16'h0000, 32'h00000001); // start
        cyc=0;
        while (!done && !fail && cyc < 10000000) begin
            @(posedge clk); cyc=cyc+1;
            // Track state transitions
            if (dut.st != prev_st) begin
                $display("  cyc=%0d: st=%0d->%0d (FS=%0d VD=%0d) ts_busy=%0d ts_done=%0d sample_cmds=%0d",
                    cyc, prev_st, dut.st, 5, 6, dut.ts_busy, dut.ts_done, 
                    $countones(0)); // placeholder
                prev_st = dut.st;
            end
        end
        $display("Done: done=%0d fail=%0d cyc=%0d", done, fail, cyc);
        $display("FS phase cycles: %0d", 0); // will be tracked
        #100; $finish;
    end

    // Dedicated FS phase counter
    reg [31:0] fs_cycles;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fs_cycles <= 0;
        else if (dut.st == 4'd5) fs_cycles <= fs_cycles + 1;
    end
endmodule
