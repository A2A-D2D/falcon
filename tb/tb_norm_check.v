`timescale 1ns/1ps
// Test: BFU_LANES=2 with REAL norm check (no force_accept)
// CASE: START_AT_FS + real rejection (FS->VD->IV->FI->N1->RC)

module tb_norm_check;
    localparam [15:0] R_CR=0,R_SR=4,R_CFG=8;
    localparam integer T0=0,T1=512,TREE=1024,B00=4864,B01=5376,B10=5888,B11=6400,CI=7424,HB=7456;

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

    task bus_rd_op; input [15:0] a; output [31:0] d;
        begin @(posedge clk); cs<=1;wr<=0;addr<=a;
        @(posedge clk); cs<=0;addr<=0; wait(ready); d=rdata; @(posedge clk); end
    endtask

    reg [3:0] pst; reg [31:0] tc,ps,restart_cnt; reg [31:0] pc[0:15];
    always @(posedge clk) begin
        if(!rst_n) begin pst<=0;tc<=0;ps<=0;restart_cnt<=0; end else begin
            tc<=tc+1;
            if(dut.st!=pst) begin pc[pst]<=tc-ps; pst<=dut.st; ps<=tc; end
            if(dut.st==1 && (pst==9||pst==7)) restart_cnt<=restart_cnt+1;
        end
    end

    wire [63:0] norm_sq = dut.norm_sq;
    wire norm_accept = dut.norm_accept;
    localparam [63:0] FALCON512_BOUND_SQ = 64'd34034726;

    integer cyc,i; reg [31:0] sr;
    initial begin
        repeat(8) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        $display("=== Norm Check Test — BFU_LANES=2, REAL rejection ===");

        // Load key material
        load_hex("b00.hex",B00,512); load_hex("b01.hex",B01,512);
        load_hex("b10.hex",B10,512); load_hex("b11.hex",B11,512);
        load_hex("tree_full_poly.hex",TREE,2816);
        load_hex("h_ntt.hex",HB,32); load_hex("hm.hex",CI,32);
        load_hex("t0_target.hex",T0,512); load_hex("t1_target.hex",T1,512);
        $display("Key material loaded.");

        // Config: start_at_fs=1, force_accept=0 (REAL rejection), bypass_fs=0
        bus_wr_op(R_CFG, 32'h00000004);
        $display("Config: force_accept=0 (REAL norm check), start_at_fs=1");

        // Start
        bus_wr_op(R_CR, 32'h00000001);
        $display("Signing started...");

        cyc=0; while(!done && !fail && cyc<50000000) begin @(posedge clk); cyc=cyc+1; end

        bus_rd_op(R_SR, sr);

        $display("");
        $display("=========================================");
        $display("  Norm Check Results");
        $display("=========================================");
        $display("  done=%0d  fail=%0d  status=0x%02h", done,fail,status);
        $display("  total_cycles=%0d  restarts=%0d", cyc, restart_cnt);
        $display("  norm_sq=%0d  bound=%0d", norm_sq, FALCON512_BOUND_SQ);
        $display("  norm_accept=%0d", norm_accept);
        if(norm_sq <= FALCON512_BOUND_SQ) $display("  *** NORM PASS (norm_sq <= bound) ***");
        else $display("  *** NORM FAIL (norm_sq > bound) ***");
        $display("-----------------------------------------");
        $display("  Phase breakdown:");
        $display("    FS: %0d  VD: %0d  IV: %0d  FI: %0d  N1: %0d  RC+: %0d",
            pc[4],pc[5],pc[6],pc[7],pc[8],pc[9]+pc[10]+pc[11]+pc[12]+pc[13]);
        $display("=========================================");

        if(done && !fail) $display("*** SIGNING PASSED with real norm check ***");
        else if(fail) $display("*** SIGNING FAILED (restart exhausted or other) ***");
        else $display("*** TIMEOUT ***");

        #100; $finish;
    end
endmodule
