`timescale 1ns/1ps
// Clean comparison: BhatMul BFU vs no BFU
module tb_bhat_compare;
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

    // Phase tracker — correct labels
    reg [3:0] pst; reg [31:0] tc,ps; reg [31:0] pc[0:15];
    always @(posedge clk) begin
        if(!rst_n) begin pst<=0;tc<=0;ps<=0; end else begin
            tc<=tc+1;
            if(dut.st!=pst) begin pc[pst]<=tc-ps; pst<=dut.st; ps<=tc; end
        end
    end

    function [127:0] phase_name; input [3:0] p;
        case(p) 0:phase_name="SI_Idle"; 1:phase_name="SH_Hash"; 2:phase_name="HP_HTP";
            3:phase_name="FC_FFT"; 4:phase_name="TG_TargetGen"; 5:phase_name="FS_ffSampling";
            6:phase_name="VD_BhatMul"; 7:phase_name="IV_IFFT"; 8:phase_name="FI_FprToInt";
            9:phase_name="N1_NTT"; 10:phase_name="RC_RejCheck"; 11:phase_name="CN_Compress";
            12:phase_name="EN_Encode"; 13:phase_name="OU_Output"; 14:phase_name="SD_Done";
            default:phase_name="??";
        endcase
    endfunction

    integer cyc,i; reg [31:0] sr;
    initial begin
        repeat(8) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        $display("=== BhatMul BFU — Full Pipeline (start_at_fs + force_accept) ===");

        load_hex("b00.hex",B00,512); load_hex("b01.hex",B01,512);
        load_hex("b10.hex",B10,512); load_hex("b11.hex",B11,512);
        load_hex("tree_full_poly.hex",TREE,2816);
        load_hex("h_ntt.hex",HB,32); load_hex("hm.hex",CI,32);
        load_hex("t0_target.hex",T0,512); load_hex("t1_target.hex",T1,512);

        bus_wr_op(R_CFG, 32'h00000006); // start_at_fs=1 force_accept=1
        bus_wr_op(R_CR, 32'h00000001);

        cyc=0; while(!done && !fail && cyc<25000000) begin @(posedge clk); cyc=cyc+1; end

        $display("");
        $display("=========================================");
        $display("  TOTAL CYCLES: %0d  done=%b fail=%b", cyc, done, fail);
        $display("-----------------------------------------");
        // Correct labels using state encoding directly
        for(i=5;i<11;i=i+1) $display("  [st=%0d] %s: %0d", i, phase_name(i[3:0]), pc[i]);
        $display("  RC+CN+EN+OU+SD: %0d", pc[10]+pc[11]+pc[12]+pc[13]+pc[14]);
        $display("=========================================");

        if(done && !fail) $display("PASS");
        else $display("FAIL");

        #100; $finish;
    end
endmodule
