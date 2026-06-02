`timescale 1ns/1ps
// FalconSign full chain test with BFU_LANES=2 (parallel 2-BFU)
// Runs CASE=2: START_AT_FS + force_accept (fastest path through real ffSampling)
// Reports total cycles and per-phase breakdown.

module tb_full_2bfu;
    localparam [15:0] REG_CR=16'h0000, REG_SR=16'h0004, REG_CFG=16'h0008, REG_MEM_HI=16'h000C;
    localparam integer LAYOUT_T0_BASE=0, LAYOUT_T1_BASE=512, LAYOUT_TREE_BASE=1024;
    localparam integer LAYOUT_Z0_BASE=3840, LAYOUT_Z1_BASE=4352;
    localparam integer LAYOUT_B00_BASE=4864, LAYOUT_B01_BASE=5376;
    localparam integer LAYOUT_B10_BASE=5888, LAYOUT_B11_BASE=6400;
    localparam integer LAYOUT_SIG_BASE=6912, LAYOUT_C_INT_BASE=7424;
    localparam integer LAYOUT_H_BASE=7456, LAYOUT_S1_BASE=7488;
    localparam N_WORDS=512, TREE_SIZE=2816;

    reg clk=0, rst_n=0;
    reg bus_cs=0, bus_wr=0; reg [15:0] bus_addr=0; reg [31:0] bus_wdata=0;
    wire [31:0] bus_rdata; wire bus_ready, bus_irq; wire busy; wire done, fail; wire [7:0] status;

    falconsign_top #(.ADDR_W(13),.LEVEL_W(4),.INDEX_W(10),.BFU_LANES(2)) dut (
        .clk,.rst_n,.bus_cs,.bus_wr,.bus_addr,.bus_wdata,
        .bus_rdata,.bus_ready,.bus_irq,.busy,.done,.fail,.status);

    always #5 clk=~clk;

    function [127:0] pn; input [3:0] p; case(p) 0:pn="SI";1:pn="SH";2:pn="HP";3:pn="FC";4:pn="FS";5:pn="VD";6:pn="IV";7:pn="FI";8:pn="N1";9:pn="RC";10:pn="CN";11:pn="EN";12:pn="OU";13:pn="SD";default:pn="??"; endcase endfunction

    task load_hex; input [1024*8-1:0] fn; input integer base,n; integer fd,addr; reg [255:0] w;
        begin fd=$fopen(fn,"r");
        for(addr=0;addr<n;addr=addr+1) begin $fscanf(fd,"%h\n",w);
            case((base+addr)&3) 0:dut.u_mem.bank0[(base+addr)>>2]=w; 1:dut.u_mem.bank1[(base+addr)>>2]=w; 2:dut.u_mem.bank2[(base+addr)>>2]=w; 3:dut.u_mem.bank3[(base+addr)>>2]=w; endcase end
        $fclose(fd); end
    endtask

    task bus_wr_op; input [15:0] a; input [31:0] d;
        begin @(posedge clk); bus_cs<=1;bus_wr<=1;bus_addr<=a;bus_wdata<=d;
        @(posedge clk); bus_cs<=0;bus_wr<=0;bus_addr<=0;bus_wdata<=0; wait(bus_ready); @(posedge clk); end
    endtask

    task bus_rd_op; input [15:0] a; output [31:0] d;
        begin @(posedge clk); bus_cs<=1;bus_wr<=0;bus_addr<=a;
        @(posedge clk); bus_cs<=0;bus_addr<=0; wait(bus_ready); d=bus_rdata; @(posedge clk); end
    endtask

    reg [3:0] prev_st; reg [31:0] total_cyc, ph_start;
    reg [31:0] ph_cyc[0:15];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin prev_st<=0;total_cyc<=0;ph_start<=0; end
        else begin
            total_cyc<=total_cyc+1;
            if(dut.st!=prev_st) begin ph_cyc[prev_st]<=total_cyc-ph_start; prev_st<=dut.st; ph_start<=total_cyc; end
        end
    end

    integer cyc,i; reg [31:0] sr;
    initial begin
        repeat(8) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        $display("=== FalconSign Full Chain — BFU_LANES=2 (Parallel 2-BFU) ===");
        $display("CASE: START_AT_FS + force_accept (FS->VD->IV->FI->N1->RC)");

        // Load key material
        $display("Loading key material...");
        load_hex("b00.hex",LAYOUT_B00_BASE,N_WORDS); load_hex("b01.hex",LAYOUT_B01_BASE,N_WORDS);
        load_hex("b10.hex",LAYOUT_B10_BASE,N_WORDS); load_hex("b11.hex",LAYOUT_B11_BASE,N_WORDS);
        load_hex("tree_full_poly.hex",LAYOUT_TREE_BASE,TREE_SIZE);
        load_hex("h_ntt.hex",LAYOUT_H_BASE,32); load_hex("hm.hex",LAYOUT_C_INT_BASE,32);
        load_hex("t0_target.hex",LAYOUT_T0_BASE,N_WORDS); load_hex("t1_target.hex",LAYOUT_T1_BASE,N_WORDS);
        $display("Key material loaded.");

        // Configure: start_at_fs=1, force_accept=1
        bus_wr_op(REG_CFG, 32'h00000006);
        $display("Config: start_at_fs=1 force_accept=1 bypass_fs=0");

        // Start
        bus_wr_op(REG_CR, 32'h00000001);
        $display("Signing started...");

        cyc=0; while(!done && !fail && cyc<25000000) begin @(posedge clk); cyc=cyc+1; end

        bus_rd_op(REG_SR, sr);
        $display("");
        $display("=========================================");
        $display("  RESULTS — BFU_LANES=2 (Parallel 2-BFU)");
        $display("=========================================");
        $display("  done=%0d fail=%0d status=0x%02h", done,fail,status);
        $display("  TOTAL CYCLES: %0d", cyc);
        $display("-----------------------------------------");
        $display("  Phase breakdown:");
        $display("    FS_ffSampling:   %6d", ph_cyc[4]);
        $display("    VD_BhatMul:      %6d", ph_cyc[5]);
        $display("    IV_IFFT:         %6d", ph_cyc[6]);
        $display("    FI_FprToInt:     %6d", ph_cyc[7]);
        $display("    N1_NTT:          %6d", ph_cyc[8]);
        $display("    RC+CN+EN+OU+SD:  %6d", ph_cyc[9]+ph_cyc[10]+ph_cyc[11]+ph_cyc[12]+ph_cyc[13]);
        $display("=========================================");

        if(done && !fail) $display("*** FULL CHAIN PASSED (BFU_LANES=2) ***");
        else $display("*** FAILED ***");

        #100; $finish;
    end
endmodule
