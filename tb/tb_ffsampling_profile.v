`timescale 1ns/1ps
module tb_ffsampling_profile;
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

    // Global cycle counter
    reg [31:0] g_cyc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) g_cyc <= 0;
        else g_cyc <= g_cyc + 1;
    end

    // FS phase tracking
    reg [31:0] fs_start, fs_end;
    reg [3:0]  pst;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin pst<=0; fs_start<=0; fs_end<=0; end
        else begin
            if (dut.st != pst) begin
                if (dut.st == 4'd4) fs_start <= g_cyc;
                if (pst == 4'd4)    fs_end   <= g_cyc;
                pst <= dut.st;
            end
        end
    end

    // Per-task profiling using scheduler handshake
    reg [31:0] t_start;
    reg [3:0]  t_op;
    reg        t_active;

    // Per-opcode accumulators
    reg [31:0] op_sum [0:7];
    reg [15:0] op_n   [0:7];
    integer k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_start<=0; t_op<=0; t_active<=0;
            for (k=0; k<8; k=k+1) begin op_sum[k]<=0; op_n[k]<=0; end
        end else begin
            // Latch on task accept
            if (dut.u_ts.run_state == 3'd1 && dut.u_ts.task_valid && dut.u_ts.task_ready && !t_active) begin
                t_start <= g_cyc;
                t_op    <= dut.u_ts.task_word[67:64];
                t_active <= 1;
            end
            // Accumulate on task done
            if (t_active && dut.u_ts.task_done) begin
                if (t_op < 8) begin
                    op_sum[t_op] <= op_sum[t_op] + (g_cyc - t_start);
                    op_n[t_op]   <= op_n[t_op] + 1;
                end
                t_active <= 0;
            end
        end
    end

    // EXU state distribution during FS phase
    reg [31:0] exu_st_cnt [0:37];
    integer si;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (si=0; si<38; si=si+1) exu_st_cnt[si] <= 0;
        end else if (dut.st == 4'd4) begin
            if (dut.u_fe.state < 38)
                exu_st_cnt[dut.u_fe.state] <= exu_st_cnt[dut.u_fe.state] + 1;
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

        bus_write(REG_CFG, 32'h00000006);
        bus_write(REG_CR, 32'h00000001);

        while (!done && !fail && g_cyc<10000000) @(posedge clk);

        $display("");
        $display("=== ffSampling Performance Profile ===");
        $display("FS phase:      %0d cycles", fs_end - fs_start);
        $display("Total:         %0d cycles", g_cyc);
        $display("");
        $display("Task Breakdown (opcode -> count / total_cyc / avg_cyc):");
        $display("  READ_L10  (0): %5d  %8d  avg=%0d", op_n[0], op_sum[0], op_n[0]>0 ? op_sum[0]/op_n[0] : 0);
        $display("  SPLIT     (1): %5d  %8d  avg=%0d", op_n[1], op_sum[1], op_n[1]>0 ? op_sum[1]/op_n[1] : 0);
        $display("  ADJUST    (2): %5d  %8d  avg=%0d", op_n[2], op_sum[2], op_n[2]>0 ? op_sum[2]/op_n[2] : 0);
        $display("  SAMPLE    (3): %5d  %8d  avg=%0d", op_n[3], op_sum[3], op_n[3]>0 ? op_sum[3]/op_n[3] : 0);
        $display("  MERGE     (4): %5d  %8d  avg=%0d", op_n[4], op_sum[4], op_n[4]>0 ? op_sum[4]/op_n[4] : 0);
        $display("  COPY      (6): %5d  %8d  avg=%0d", op_n[6], op_sum[6], op_n[6]>0 ? op_sum[6]/op_n[6] : 0);
        $display("  Sum tasks:     %0d", op_n[0]+op_n[1]+op_n[2]+op_n[3]+op_n[4]+op_n[6]);
        $display("");
        $display("EXU State Distribution (FS phase):");
        $display("  ST_IDLE(0):         %8d", exu_st_cnt[0]);
        $display("  ST_READ_REQ/CAP:    %8d", exu_st_cnt[1]+exu_st_cnt[2]);
        $display("  ST_PAIR_A/B_REQ/CAP:%8d", exu_st_cnt[3]+exu_st_cnt[4]+exu_st_cnt[5]+exu_st_cnt[6]);
        $display("  ST_ADJ_T1/Z1/T0:    %8d", exu_st_cnt[7]+exu_st_cnt[8]+exu_st_cnt[9]+exu_st_cnt[10]+exu_st_cnt[11]+exu_st_cnt[12]);
        $display("  ST_FPU_REQ/WAIT:    %8d", exu_st_cnt[13]+exu_st_cnt[14]);
        $display("  ST_PAIR_WR0/WR1:    %8d", exu_st_cnt[15]+exu_st_cnt[16]);
        $display("  ST_ADJ_WR/MIRROR:   %8d", exu_st_cnt[17]+exu_st_cnt[29]);
        $display("  ST_SAMPLE_*:        %8d", exu_st_cnt[18]+exu_st_cnt[19]+exu_st_cnt[20]+exu_st_cnt[23]+exu_st_cnt[24]+exu_st_cnt[25]+exu_st_cnt[26]);
        $display("  ST_SPLIT_BUF:       %8d", exu_st_cnt[32]+exu_st_cnt[33]+exu_st_cnt[34]+exu_st_cnt[35]);
        $display("  ST_MERGE_BUF:       %8d", exu_st_cnt[35]);
        $display("  ST_COPY_REQ/CAP:    %8d", exu_st_cnt[36]+exu_st_cnt[37]);
        $display("  ST_PAIR_MIR0/MIR1:  %8d", exu_st_cnt[30]+exu_st_cnt[31]);
        $display("");
        $display("done=%0d fail=%0d", done, fail);
        $finish;
    end
endmodule
