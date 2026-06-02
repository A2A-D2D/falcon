`timescale 1ns/1ps
module tb_phase_profile;
    localparam [15:0] REG_CR = 16'h0000, REG_SR = 16'h0004, REG_CFG = 16'h0008;
    localparam integer LAYOUT_T0_BASE=0, LAYOUT_T1_BASE=512, LAYOUT_TREE_BASE=1024;
    localparam integer LAYOUT_Z0_BASE=3840, LAYOUT_Z1_BASE=4352;
    localparam integer LAYOUT_B00_BASE=4864, LAYOUT_B01_BASE=5376, LAYOUT_B10_BASE=5888, LAYOUT_B11_BASE=6400;
    localparam integer LAYOUT_SIG_BASE=6912, LAYOUT_C_INT_BASE=7424, LAYOUT_H_BASE=7456, LAYOUT_S1_BASE=7488;
    localparam integer N_WORDS=512, TREE_SIZE=2816;

    reg clk, rst_n, bus_cs, bus_wr;
    reg [15:0] bus_addr;
    reg [31:0] bus_wdata;
    wire [31:0] bus_rdata;
    wire bus_ready, bus_irq, busy, done, fail;
    wire [7:0] status;

    falconsign_top #(.ADDR_W(13), .LEVEL_W(4), .INDEX_W(10)) dut (
        .clk(clk), .rst_n(rst_n),
        .bus_cs(bus_cs), .bus_wr(bus_wr),
        .bus_addr(bus_addr), .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata), .bus_ready(bus_ready),
        .bus_irq(bus_irq), .busy(busy), .done(done), .fail(fail), .status(status)
    );

    initial begin clk=0; rst_n=0; #30 rst_n=1; end
    always #5 clk = ~clk;

    task load_hex;
        input [1024*8-1:0] filename;
        input integer base_addr, num_words;
        integer fd, n, addr;
        reg [255:0] word;
        begin
            fd = $fopen(filename, "r");
            if (fd==0) begin $display("ERROR: %s", filename); $finish; end
            for (addr=0; addr<num_words; addr=addr+1) begin
                n = $fscanf(fd, "%h\n", word);
                case ((base_addr+addr)&3)
                    0: dut.u_mem.bank0[(base_addr+addr)>>2] = word;
                    1: dut.u_mem.bank1[(base_addr+addr)>>2] = word;
                    2: dut.u_mem.bank2[(base_addr+addr)>>2] = word;
                    default: dut.u_mem.bank3[(base_addr+addr)>>2] = word;
                endcase
            end
            $fclose(fd);
        end
    endtask

    task bus_write;
        input [15:0] addr; input [31:0] data;
        begin @(posedge clk); bus_cs<=1; bus_wr<=1; bus_addr<=addr; bus_wdata<=data;
              @(posedge clk); bus_cs<=0; bus_wr<=0; wait(bus_ready); @(posedge clk); end
    endtask

    reg [31:0] g_cyc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) g_cyc <= 0; else g_cyc <= g_cyc + 1;
    end

    // Phase duration tracking
    reg [31:0] ph_start [0:15];
    reg [31:0] ph_dur   [0:15];
    reg [3:0]  pst;
    integer pi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pst <= 0;
            for (pi=0; pi<16; pi=pi+1) begin ph_start[pi]<=0; ph_dur[pi]<=0; end
        end else begin
            if (dut.st != pst) begin
                ph_dur[pst] <= g_cyc - ph_start[pst];
                ph_start[dut.st] <= g_cyc;
                pst <= dut.st;
            end
        end
    end

    initial begin
        bus_cs=0; bus_wr=0; bus_addr=0; bus_wdata=0;
        repeat(8) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);

        load_hex("t0_target.hex", LAYOUT_T0_BASE, N_WORDS);
        load_hex("t1_target.hex", LAYOUT_T1_BASE, N_WORDS);
        load_hex("b00.hex", LAYOUT_B00_BASE, N_WORDS);
        load_hex("b01.hex", LAYOUT_B01_BASE, N_WORDS);
        load_hex("b10.hex", LAYOUT_B10_BASE, N_WORDS);
        load_hex("b11.hex", LAYOUT_B11_BASE, N_WORDS);
        load_hex("tree_full_poly.hex", LAYOUT_TREE_BASE, TREE_SIZE);
        load_hex("h_ntt.hex", LAYOUT_H_BASE, 32);
        load_hex("hm.hex", LAYOUT_C_INT_BASE, 32);

        $display("=== Case 1: START_AT_FS + force_accept ===");
        bus_write(REG_CFG, 32'h00000006);
        bus_write(REG_CR, 32'h00000001);
        while (!done && !fail && g_cyc<10000000) @(posedge clk);

        $display("Total: %0d cy  done=%0d fail=%0d", g_cyc, done, fail);
        $display("  SI: %0d  SH: %0d  HP: %0d  FC: %0d", ph_dur[0], ph_dur[1], ph_dur[2], ph_dur[3]);
        $display("  TG: %0d  FS: %0d  VD: %0d  IV: %0d", ph_dur[4], ph_dur[5], ph_dur[6], ph_dur[7]);  // Note: st indices might not match
        $display("  FI: %0d  N1: %0d  RC: %0d", ph_dur[8], ph_dur[9], ph_dur[10]);
        $display("");
        $display("Actual st mapping:");
        $display("  st=0(SI): %0d", ph_dur[0]);
        $display("  st=1(SH): %0d", ph_dur[1]);
        $display("  st=2(HP): %0d", ph_dur[2]);
        $display("  st=3(FC): %0d", ph_dur[3]);
        $display("  st=4(FS): %0d", ph_dur[4]);
        $display("  st=5(VD): %0d", ph_dur[5]);
        $display("  st=6(IV): %0d", ph_dur[6]);
        $display("  st=7(FI): %0d", ph_dur[7]);
        $display("  st=8(N1): %0d", ph_dur[8]);
        $display("  st=9(RC): %0d", ph_dur[9]);
        $display("  st=10(CN): %0d", ph_dur[10]);
        $display("  st=11(EN): %0d", ph_dur[11]);
        $display("  st=12(OU): %0d", ph_dur[12]);
        $display("  st=13(SD): %0d", ph_dur[13]);
        $finish;
    end
endmodule
