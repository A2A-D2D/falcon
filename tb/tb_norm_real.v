`timescale 1ns/1ps
module tb_norm_real;
    localparam [15:0] R_CR=0,R_SR=4,R_CFG=8;
    localparam T0=0,T1=512,TREE=1024,B00=4864,B01=5376,B10=5888,B11=6400,CI=7424,HB=7456;

    reg clk=0, rst_n=0;
    reg cs=0,wr=0; reg [15:0] addr=0; reg [31:0] wdata=0;
    wire [31:0] rdata; wire ready,irq; wire busy; wire done,fail; wire [7:0] status;

    falconsign_top #(.ADDR_W(13),.LEVEL_W(4),.INDEX_W(10),.BFU_LANES(2)) dut (
        .clk,.rst_n,.bus_cs(cs),.bus_wr(wr),.bus_addr(addr),.bus_wdata(wdata),
        .bus_rdata(rdata),.bus_ready(ready),.bus_irq(irq),
        .busy(busy),.done(done),.fail(fail),.status(status));

    always #5 clk=~clk;

    task load_hex; input [1024*8-1:0] fn; input integer base,n; integer fd,a; reg [255:0] w;
        begin fd=$fopen(fn,"r");
        for(a=0;a<n;a=a+1) begin $fscanf(fd,"%h\n",w);
            case((base+a)&3) 0:dut.u_mem.bank0[(base+a)>>2]=w; 1:dut.u_mem.bank1[(base+a)>>2]=w; 2:dut.u_mem.bank2[(base+a)>>2]=w; 3:dut.u_mem.bank3[(base+a)>>2]=w; endcase end
        $fclose(fd); end
    endtask

    task bus_wr_op; input [15:0] a; input [31:0] d;
        begin @(posedge clk); cs<=1;wr<=1;addr<=a;wdata<=d;
        @(posedge clk); cs<=0;wr<=0;addr<=0;wdata<=0; wait(ready); @(posedge clk); end
    endtask

    integer cyc,i; reg [31:0] sr;
    initial begin
        repeat(8) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        $display("=== Norm Check — BFU_LANES=2, REAL rejection (force_accept=0) ===");

        load_hex("b00.hex",B00,512); load_hex("b01.hex",B01,512);
        load_hex("b10.hex",B10,512); load_hex("b11.hex",B11,512);
        load_hex("tree_full_poly.hex",TREE,2816);
        load_hex("h_ntt.hex",HB,32); load_hex("hm.hex",CI,32);
        load_hex("t0_target.hex",T0,512); load_hex("t1_target.hex",T1,512);
        $display("Key loaded.");

        // Config: start_at_fs=1, force_accept=0 (REAL norm check!)
        bus_wr_op(R_CFG, 32'h00000004);
        $display("Config: start_at_fs=1 force_accept=0");

        bus_wr_op(R_CR, 32'h00000001);
        $display("Signing...");

        cyc=0; while(!done && !fail && cyc<50000000) begin @(posedge clk); cyc=cyc+1; end

        $display("");
        $display("=========================================");
        $display("  done=%0d  fail=%0d  status=0x%02h", done,fail,status);
        $display("  total_cycles=%0d", cyc);
        $display("  norm_sq=%0d  bound=%0d", dut.norm_sq, 64'd34034726);
        $display("  norm_accept=%0d", dut.norm_accept);
        if(dut.norm_sq <= 64'd34034726) $display("  *** NORM PASS ***");
        else $display("  *** NORM FAIL ***");
        $display("=========================================");

        if(done && !fail) $display("*** SIGNING PASSED ***");
        else $display("*** FAILED ***");

        #100; $finish;
    end
endmodule
