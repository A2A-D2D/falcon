// Quick check: does HP write c[0] to addr 0?
// Monitors hp_wr_en, hp_wr_addr, and address 0 after HP phase
module tb_check_hp;
    localparam [15:0] REG_CR=0, REG_SR=4, REG_CFG=8, REG_MEM_HI=12;
    localparam integer L0=0, L1=512, LT=1024, LZ0=3840, LZ1=4352;
    localparam integer LB00=4864, LB01=5376, LB10=5888, LB11=6400;
    localparam integer LSIG=6912, LCINT=7424, LH=7456, LS1=7488;
    reg clk=0, rst_n=0, bus_cs=0, bus_wr=0;
    reg [15:0] bus_addr=0; reg [31:0] bus_wdata=0;
    wire [31:0] bus_rdata; wire bus_ready, bus_irq, busy, done, fail;
    wire [7:0] status;

    falconsign_top #(.ADDR_W(13), .LEVEL_W(4), .INDEX_W(10)) dut (
        .clk(clk), .rst_n(rst_n),
        .bus_cs(bus_cs), .bus_wr(bus_wr),
        .bus_addr(bus_addr), .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata), .bus_ready(bus_ready),
        .bus_irq(bus_irq), .busy(busy), .done(done), .fail(fail), .status(status)
    );

    always #5 clk = ~clk;
    always @(posedge clk) begin
        if (dut.sn == 2 && dut.st == 1) begin // SH→HP
            $display("T=%0t SH→HP: hp_wr_en=%b hp_wr_addr=%0d", $time, dut.hp_wr_en, dut.hp_wr_addr);
        end
        if (dut.st == 2 && dut.hp_wr_en && dut.htp_coeff_valid) begin
            $display("T=%0t HP WRITE: en=%b addr=%0d data_re=%h htp_coeff=%0d",
                $time, dut.hp_wr_en, dut.hp_wr_addr, dut.hp_wr_data, dut.htp_coeff);
        end
    end

    function [255:0] peek; input integer a;
        case(a&3) 0: peek=dut.u_mem.bank0[a>>2];
        1: peek=dut.u_mem.bank1[a>>2];
        2: peek=dut.u_mem.bank2[a>>2];
        default: peek=dut.u_mem.bank3[a>>2]; endcase
    endfunction

    task load_hex; input [1024*8-1:0] fn; input integer base, nw;
        integer fd, addr; reg [255:0] w;
        begin fd=$fopen(fn,"r");
            for(addr=0;addr<nw;addr=addr+1) begin
                $fscanf(fd,"%h\n",w);
                case((base+addr)&3) 0:dut.u_mem.bank0[(base+addr)>>2]=w;
                1:dut.u_mem.bank1[(base+addr)>>2]=w;
                2:dut.u_mem.bank2[(base+addr)>>2]=w;
                default:dut.u_mem.bank3[(base+addr)>>2]=w; endcase
            end $fclose(fd); end
    endtask

    task bus_w; input [15:0] a; input [31:0] d;
        begin @(posedge clk); bus_cs<=1;bus_wr<=1;bus_addr<=a;bus_wdata<=d;
            @(posedge clk); bus_cs<=0;bus_wr<=0;bus_addr<=0;bus_wdata<=0;
            wait(bus_ready);@(posedge clk); end
    endtask

    initial begin
        repeat(8)@(posedge clk); rst_n=1; repeat(4)@(posedge clk);
        load_hex("t0_target.hex", L0, 512);
        load_hex("t1_target.hex", L1, 512);
        load_hex("b00.hex",LB00,512); load_hex("b01.hex",LB01,512);
        load_hex("b10.hex",LB10,512); load_hex("b11.hex",LB11,512);
        load_hex("tree_full_poly.hex",LT,2816);
        load_hex("h_ntt.hex",LH,32);
        load_hex("hm.hex",LCINT,32);
        bus_w(REG_CFG, 32'h02); // force_accept=1
        $display("Starting...");
        bus_w(REG_CR, 1);
        while(!done && !fail) @(posedge clk);
        $display("Done: done=%b fail=%b norm_sq=%0d", done, fail, dut.norm_sq);
        $display("HP addr0 after: %h", peek(0));
        $display("HP addr1 after: %h", peek(1));
        #100; $finish;
    end
endmodule
