`timescale 1ns/1ps
// Compare BFU_LANES=1 vs BFU_LANES=2 full pipeline (CASE=1: SH->HP->FC->FS->VD->IV->FI->N1)

module tb_compare_bfu;

    // ——— BFU_LANES=2 DUT ———
    reg clk=0, rst_n=0;
    reg b2_cs=0,b2_wr=0; reg [15:0] b2_addr=0; reg [31:0] b2_wdata=0;
    wire [31:0] b2_rdata; wire b2_ready,b2_irq; wire b2_busy; wire b2_done,b2_fail; wire [7:0] b2_status;

    falconsign_top #(.ADDR_W(13),.LEVEL_W(4),.INDEX_W(10),.BFU_LANES(2)) dut2 (
        .clk,.rst_n,.bus_cs(b2_cs),.bus_wr(b2_wr),.bus_addr(b2_addr),.bus_wdata(b2_wdata),
        .bus_rdata(b2_rdata),.bus_ready(b2_ready),.bus_irq(b2_irq),
        .busy(b2_busy),.done(b2_done),.fail(b2_fail),.status(b2_status));

    always #5 clk=~clk;

    localparam [15:0] R_CR=0,R_SR=4,R_CFG=8,R_MHI=12;
    localparam integer T0=0,T1=512,TREE=1024,Z0=3840,Z1=4352,B00=4864,B01=5376,B10=5888,B11=6400,SIG=6912,CI=7424,HB=7456,S1=7488;

    task load_hex; input [1024*8-1:0] fn; input integer base,n; integer fd,addr; reg [255:0] w;
        begin fd=$fopen(fn,"r");
        for(addr=0;addr<n;addr=addr+1) begin $fscanf(fd,"%h\n",w);
            case((base+addr)&3) 0:dut2.u_mem.bank0[(base+addr)>>2]=w; 1:dut2.u_mem.bank1[(base+addr)>>2]=w; 2:dut2.u_mem.bank2[(base+addr)>>2]=w; 3:dut2.u_mem.bank3[(base+addr)>>2]=w; endcase end
        $fclose(fd); end
    endtask

    task wait_done; integer c; begin c=0; while(!b2_done && !b2_fail && c<30000000) begin @(posedge clk); c=c+1; end end endtask

    // Phase tracker
    reg [3:0] pst; reg [31:0] tc,ps; reg [31:0] pc[0:15];
    always @(posedge clk) begin
        if(!rst_n) begin pst<=0;tc<=0;ps<=0; end else begin
            tc<=tc+1;
            if(dut2.st!=pst) begin pc[pst]<=tc-ps; pst<=dut2.st; ps<=tc; end
        end
    end

    integer cyc,i;
    initial begin
        // Load keys
        load_hex("b00.hex",B00,512); load_hex("b01.hex",B01,512);
        load_hex("b10.hex",B10,512); load_hex("b11.hex",B11,512);
        load_hex("tree_full_poly.hex",TREE,2816);
        load_hex("h_ntt.hex",HB,32); load_hex("hm.hex",CI,32);

        // ——— RUN 1: BFU_LANES=2 full pipeline ———
        repeat(8) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        // Reload t0/t1 (will be overwritten by HP→FC)
        load_hex("t0_target.hex",T0,512); load_hex("t1_target.hex",T1,512);
        @(posedge clk);

        // Config: force_accept=1, restart t0/t1 at start
        b2_cs<=1;b2_wr<=1;b2_addr<=R_CFG;b2_wdata<=32'h00000002; @(posedge clk);
        b2_cs<=0;b2_wr<=0;b2_addr<=0;b2_wdata<=0; wait(b2_ready); @(posedge clk);

        // Start
        b2_cs<=1;b2_wr<=1;b2_addr<=R_CR;b2_wdata<=32'h00000001; @(posedge clk);
        b2_cs<=0;b2_wr<=0;b2_addr<=0;b2_wdata<=0; wait(b2_ready); @(posedge clk);

        wait_done();

        $display("==========================================");
        $display("  BFU_LANES=2 (Parallel 2-BFU) — Full Pipeline");
        $display("==========================================");
        $display("  done=%0d fail=%0d status=0x%02h", b2_done,b2_fail,b2_status);
        $display("  TOTAL: %0d cycles", tc);
        $display("  SH: %0d  HP: %0d  FC_FFT: %0d  FS: %0d", pc[1],pc[2],pc[3],pc[4]);
        $display("  VD(BhatMul): %0d  IV_IFFT: %0d  FI: %0d  N1_NTT: %0d", pc[5],pc[6],pc[7],pc[8]);
        $display("  RC+CN+EN+OU+SD: %0d", pc[9]+pc[10]+pc[11]+pc[12]+pc[13]);
        $display("==========================================");

        #100; $finish;
    end
endmodule
