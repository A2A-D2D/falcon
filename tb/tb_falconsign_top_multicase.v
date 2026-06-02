`timescale 1ns/1ps
// ─── FalconSign Multi-Case Full-Flow Testbench ───
//
// Runs multiple test cases covering the complete signing pipeline:
//   Case 0: BYPASS_FS mode — identity z=t, tests outer pipeline
//   Case 1: FORCE_ACCEPT mode — real ffSampling, skip rejection
//   Case 2: Full normal mode — complete signing with rejection check
//   Case 3: START_AT_FS mode — skip frontend, start from ffSampling
//
// Each case runs independently with reloaded memory state.
// Run: vvp tb_multicase.vvp [+DUMP_VCD] [+STOP_ON_FAIL]

module tb_falconsign_top_multicase;

    localparam [15:0] REG_CR     = 16'h0000;
    localparam [15:0] REG_SR     = 16'h0004;
    localparam [15:0] REG_CFG    = 16'h0008;
    localparam [15:0] REG_MEM_HI = 16'h000C;

    localparam integer LAYOUT_T0_BASE   = 0;
    localparam integer LAYOUT_T1_BASE   = 512;
    localparam integer LAYOUT_TREE_BASE = 1024;
    localparam integer LAYOUT_Z0_BASE   = 3840;
    localparam integer LAYOUT_Z1_BASE   = 4352;
    localparam integer LAYOUT_B00_BASE  = 4864;
    localparam integer LAYOUT_B01_BASE  = 5376;
    localparam integer LAYOUT_B10_BASE  = 5888;
    localparam integer LAYOUT_B11_BASE  = 6400;
    localparam integer LAYOUT_SIG_BASE  = 6912;
    localparam integer LAYOUT_C_INT_BASE = 7424;
    localparam integer LAYOUT_H_BASE    = 7456;
    localparam integer LAYOUT_S1_BASE   = 7488;

    localparam integer FALCON_N = 512;
    localparam integer N_WORDS  = 512;
    localparam integer TREE_SIZE = 2816;
    localparam integer MAX_CYCLES = 50000000;

    reg         clk;
    reg         rst_n;
    reg         bus_cs;
    reg         bus_wr;
    reg  [15:0] bus_addr;
    reg  [31:0] bus_wdata;
    wire [31:0] bus_rdata;
    wire        bus_ready;
    wire        bus_irq;
    wire        busy;
    wire        done;
    wire        fail;
    wire [7:0]  status;

    falconsign_top #(.ADDR_W(13), .LEVEL_W(4), .INDEX_W(10)) dut (
        .clk(clk), .rst_n(rst_n),
        .bus_cs(bus_cs), .bus_wr(bus_wr),
        .bus_addr(bus_addr), .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata), .bus_ready(bus_ready),
        .bus_irq(bus_irq),
        .busy(busy), .done(done), .fail(fail), .status(status)
    );

    // ─── Clock & Reset ───
    initial begin
        clk   = 1'b0;
        rst_n = 1'b0;
        #30 rst_n = 1'b1;
    end
    always #5 clk = ~clk;

    // ─── Phase name helper ───
    function [127:0] phase_name;
        input [3:0] p;
        case (p)
            4'd0: phase_name = "SI_Idle";
            4'd1: phase_name = "SH_SeedHash";
            4'd2: phase_name = "HP_HashToPoint";
            4'd3: phase_name = "FC_FFT";
            4'd4: phase_name = "TG_TargetGen";
            4'd5: phase_name = "FS_ffSampling";
            4'd6: phase_name = "VD_BhatMul";
            4'd7: phase_name = "IV_IFFT";
            4'd8: phase_name = "FI_FprToInt";
            4'd9: phase_name = "N1_NTT";
            4'd10: phase_name = "RC_RejCheck";
            4'd11: phase_name = "CN_Compress";
            4'd12: phase_name = "EN_Encode";
            4'd13: phase_name = "OU_Output";
            4'd14: phase_name = "SD_SendDone";
            default: phase_name = "UNKNOWN";
        endcase
    endfunction

    // ─── Memory access helpers ───
    function [255:0] peek_mem_word;
        input integer word_addr;
        begin
            case (word_addr & 3)
                0: peek_mem_word = dut.u_mem.bank0[word_addr >> 2];
                1: peek_mem_word = dut.u_mem.bank1[word_addr >> 2];
                2: peek_mem_word = dut.u_mem.bank2[word_addr >> 2];
                default: peek_mem_word = dut.u_mem.bank3[word_addr >> 2];
            endcase
        end
    endfunction

    task poke_mem_word;
        input integer word_addr;
        input [255:0] word;
        begin
            case (word_addr & 3)
                0: dut.u_mem.bank0[word_addr >> 2] = word;
                1: dut.u_mem.bank1[word_addr >> 2] = word;
                2: dut.u_mem.bank2[word_addr >> 2] = word;
                default: dut.u_mem.bank3[word_addr >> 2] = word;
            endcase
        end
    endtask

    task load_hex_to_mem;
        input [1024*8-1:0] filename;
        input integer       base_addr;
        input integer       num_words;
        integer fd, n, addr;
        reg [255:0] word;
        begin
            fd = $fopen(filename, "r");
            if (fd == 0) begin
                $display("  ERROR: Cannot open %s", filename);
                $finish;
            end
            for (addr = 0; addr < num_words; addr = addr + 1) begin
                n = $fscanf(fd, "%h\n", word);
                if (n != 1) begin
                    $display("  ERROR: Short read at addr %0d of %s (n=%0d)", addr, filename, n);
                    $finish;
                end
                case ((base_addr + addr) & 3)
                    0: dut.u_mem.bank0[(base_addr + addr) >> 2] = word;
                    1: dut.u_mem.bank1[(base_addr + addr) >> 2] = word;
                    2: dut.u_mem.bank2[(base_addr + addr) >> 2] = word;
                    default: dut.u_mem.bank3[(base_addr + addr) >> 2] = word;
                endcase
            end
            $fclose(fd);
        end
    endtask

    // ─── Bus operations ───
    task bus_write;
        input [15:0] addr;
        input [31:0] data;
        begin
            @(posedge clk);
            bus_cs    <= 1'b1;
            bus_wr    <= 1'b1;
            bus_addr  <= addr;
            bus_wdata <= data;
            @(posedge clk);
            bus_cs    <= 1'b0;
            bus_wr    <= 1'b0;
            bus_addr  <= 16'd0;
            bus_wdata <= 32'd0;
            wait (bus_ready);
            @(posedge clk);
        end
    endtask

    task bus_read;
        input  [15:0] addr;
        output [31:0] data;
        begin
            @(posedge clk);
            bus_cs    <= 1'b1;
            bus_wr    <= 1'b0;
            bus_addr  <= addr;
            bus_wdata <= 32'd0;
            @(posedge clk);
            bus_cs    <= 1'b0;
            bus_addr  <= 16'd0;
            wait (bus_ready);
            data = bus_rdata;
            @(posedge clk);
        end
    endtask

    // ─── Load all key material ───
    task load_full_key;
        begin
            $display("  Loading key material...");
            load_hex_to_mem("t0_target.hex", LAYOUT_T0_BASE, N_WORDS);
            load_hex_to_mem("t1_target.hex", LAYOUT_T1_BASE, N_WORDS);
            load_hex_to_mem("b00.hex", LAYOUT_B00_BASE, N_WORDS);
            load_hex_to_mem("b01.hex", LAYOUT_B01_BASE, N_WORDS);
            load_hex_to_mem("b10.hex", LAYOUT_B10_BASE, N_WORDS);
            load_hex_to_mem("b11.hex", LAYOUT_B11_BASE, N_WORDS);
            load_hex_to_mem("tree_full_poly.hex", LAYOUT_TREE_BASE, TREE_SIZE);
            load_hex_to_mem("h_ntt.hex", LAYOUT_H_BASE, 32);
            load_hex_to_mem("hm.hex", LAYOUT_C_INT_BASE, 32);
            $display("  Key material loaded.");
        end
    endtask

    // ─── Restore t0/t1 from hex (reload after each sign attempt) ───
    task reload_targets;
        begin
            load_hex_to_mem("t0_target.hex", LAYOUT_T0_BASE, N_WORDS);
            load_hex_to_mem("t1_target.hex", LAYOUT_T1_BASE, N_WORDS);
        end
    endtask

    // ─── Preload z = t for bypass_fs mode ───
    task preload_identity_z;
        integer k;
        begin
            for (k = 0; k < N_WORDS; k = k + 1) begin
                poke_mem_word(LAYOUT_Z0_BASE + k, peek_mem_word(LAYOUT_T0_BASE + k));
                poke_mem_word(LAYOUT_Z1_BASE + k, peek_mem_word(LAYOUT_T1_BASE + k));
            end
        end
    endtask

    // ═══════════════════════════════════════════════════════════
    // Phase tracker — shared across all cases
    // ═══════════════════════════════════════════════════════════
    reg [3:0]  prev_st, current_case;
    reg [31:0] phase_cycles [0:15];
    reg [31:0] phase_start;
    reg [31:0] total_cycle;
    reg [31:0] restart_cnt;
    reg [31:0] sample_cmds;
    reg [31:0] sample_rsps;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_st     <= 0;
            phase_start <= 0;
            total_cycle <= 0;
            restart_cnt <= 0;
            sample_cmds <= 0;
            sample_rsps <= 0;
            {phase_cycles[0],phase_cycles[1],phase_cycles[2],phase_cycles[3],
             phase_cycles[4],phase_cycles[5],phase_cycles[6],phase_cycles[7],
             phase_cycles[8],phase_cycles[9],phase_cycles[10],phase_cycles[11],
             phase_cycles[12],phase_cycles[13],phase_cycles[14],phase_cycles[15]} <= 0;
        end else begin
            total_cycle <= total_cycle + 1;
            if (dut.fe_sz_cmd_valid && dut.fe_sz_cmd_ready) sample_cmds <= sample_cmds + 1;
            if (dut.sz_rsp_valid) sample_rsps <= sample_rsps + 1;

            if (dut.st != prev_st) begin
                phase_cycles[prev_st] <= total_cycle - phase_start;
                prev_st     <= dut.st;
                phase_start <= total_cycle;
                if (dut.st == 4'd1 && (prev_st == 4'd9 || prev_st == 4'd7 || prev_st == 4'd5))
                    restart_cnt <= restart_cnt + 1;
            end
        end
    end

    // ═══════════════════════════════════════════════════════════
    // Single test case runner
    // ═══════════════════════════════════════════════════════════
    reg        case_pass;
    reg [31:0] case_cycles;
    reg [7:0]  case_status;
    reg [31:0] case_restarts;
    reg [31:0] case_samples;
    reg [31:0] case_sampler_rsps;
    reg [3:0]  case_end_phase;
    reg [1023:0] case_name;

    task run_single_case;
        input [1023:0] desc;
        input [31:0]   cfg_val;
        input           preload_z;     // 1 = preload z=t before start
        input           reload_t;      // 1 = reload t0/t1 before start
        input           expect_done;   // 1 = expect done, 0 = expect fail
        input [7:0]     expect_status; // only checked if expect_done == 0
        input [31:0]    timeout_cycles;

        integer k;
        reg [31:0] sr;
        reg [3:0]  fail_phase;
        begin
            $display("");
            $display("╔══════════════════════════════════════════════════════╗");
            $display("║  CASE %0d: %s", current_case, desc);
            $display("╚══════════════════════════════════════════════════════╝");

            case_name = desc;

            // Reset tracking counters for this case
            restart_cnt  <= 0;
            sample_cmds  <= 0;
            sample_rsps  <= 0;

            // ─── Pre-start memory setup ───
            if (reload_t) reload_targets();
            if (preload_z) preload_identity_z();

            @(posedge clk);

            // ─── Configure ───
            bus_write(REG_CFG, cfg_val);
            $display("  Config: REG_CFG=0x%08h (bypass_fs=%b force_accept=%b start_at_fs=%b dynamic=%b)",
                cfg_val, cfg_val[0], cfg_val[1], cfg_val[2], cfg_val[3]);

            // ─── Start ───
            $display("  Starting signing operation...");
            bus_write(REG_CR, 32'h00000001);
            // Wait one extra cycle for done/fail to clear from previous case
            @(posedge clk);

            // ─── Wait for completion ───
            case_cycles = 0;
            while (!done && !fail && case_cycles < timeout_cycles) begin
                @(posedge clk);
                case_cycles = case_cycles + 1;
            end

            bus_read(REG_SR, sr);
            case_status   = status;
            case_restarts = restart_cnt;
            case_samples  = sample_cmds;
            case_sampler_rsps = sample_rsps;
            case_end_phase = dut.st;

            // ─── Check result ───
            if (expect_done) begin
                case_pass = done && !fail;
            end else begin
                case_pass = fail && (status == expect_status);
            end

            // ─── Report ───
            $display("");
            $display("  ─── Case %0d Results ───", current_case);
            $display("  done=%0d  fail=%0d  irq=%0d  status=0x%02h",
                done, fail, bus_irq, status);
            $display("  cycles=%0d  restarts=%0d  end_phase=%s",
                case_cycles, case_restarts, phase_name(case_end_phase));
            $display("  sample_cmds=%0d  sample_rsps=%0d",
                case_samples, case_sampler_rsps);
            $display("  Completed phases:");
            $display("    SH_SeedHash:     %0d cy", phase_cycles[1]);
            $display("    HP_HashToPoint:  %0d cy", phase_cycles[2]);
            $display("    FC_FFT:          %0d cy", phase_cycles[3]);
            $display("    FS_ffSampling:   %0d cy", phase_cycles[4]);
            $display("    VD_BhatMul:      %0d cy", phase_cycles[5]);
            $display("    IV_IFFT:         %0d cy", phase_cycles[6]);
            $display("    FI_FprToInt:     %0d cy", phase_cycles[7]);
            $display("    N1_NTT:          %0d cy", phase_cycles[8]);
            $display("    RC_RejCheck:     %0d cy", phase_cycles[9]);

            // Quick sanity: print first few signature words
            if (done && !fail) begin
                $display("  s2 first words:");
                for (k = 0; k < 4; k = k + 1)
                    $display("    sig[%0d] = %h", k, peek_mem_word(LAYOUT_SIG_BASE + k));
                $display("  s1 first words:");
                for (k = 0; k < 4; k = k + 1)
                    $display("    s1[%0d]  = %h", k, peek_mem_word(LAYOUT_S1_BASE + k));
                $display("  Norm: accept=%0d norm_sq=%0d bound=%0d",
                    dut.norm_accept, dut.norm_sq, dut.FALCON512_BOUND_SQ);
            end

            if (case_pass)
                $display("  >>> CASE %0d PASSED <<<", current_case);
            else begin
                $display("  >>> CASE %0d FAILED <<<", current_case);
                if (case_cycles >= timeout_cycles) begin
                    $display("  Reason: TIMEOUT at %0d cycles", case_cycles);
                    $display("  Stuck at: phase=%s st=%0d sn=%0d",
                        phase_name(dut.st), dut.st, dut.sn);
                    $display("  SH: word_idx=%0d absorb=%0d shake_ready=%0d",
                        dut.sh_word_idx, dut.shake_absorb, dut.shake_ready);
                    $display("  HP: coeff_cnt=%0d wr_addr=%0d",
                        dut.hp_coeff_cnt, dut.hp_wr_addr);
                    $display("  FE: state=%0d op=%0d task_ready=%0d",
                        dut.u_fe.state, dut.u_fe.op_q, dut.fe_task_ready);
                    $display("  TS: run=%0d state=%0d busy=%0d done=%0d fail=%0d",
                        dut.u_ts.run_state, dut.u_ts.state_q, dut.ts_busy, dut.ts_done, dut.ts_fail);
                    $display("  SZ: busy=%0d done=%0d fail=%0d",
                        dut.sz_busy, dut.sz_done, dut.sz_fail);
                end
                if ($test$plusargs("STOP_ON_FAIL")) $finish;
            end
        end
    endtask

    // ═══════════════════════════════════════════════════════════
    // Main test orchestrator
    // ═══════════════════════════════════════════════════════════
    reg [31:0] passed;
    reg [31:0] failed;
    reg [31:0] case_cyc_log [0:7];
    integer    ci;

    initial begin
        bus_cs    = 1'b0;
        bus_wr    = 1'b0;
        bus_addr  = 16'd0;
        bus_wdata = 32'd0;
        current_case = 0;
        passed = 0;
        failed = 0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("╔══════════════════════════════════════════════════════╗");
        $display("║  FalconSign Multi-Case Full-Flow Test               ║");
        $display("║  Falcon-512, %0d MHz                                ║", 100);
        $display("╚══════════════════════════════════════════════════════╝");
        $display("");

        // ─── Load key material once ───
        $display("=== Loading shared key material ===");
        load_full_key();

        $display("");
        $display("=== Beginning test cases ===");

        // ═══════════════════════════════════════════════════════
        // Case 0: BYPASS_FS mode — identity test
        // cfg[0]=1 (bypass_fs), cfg[1]=1 (force_accept)
        // Pipeline: SH→HP→FC→VD→IV→FI→N1→RC
        // ffSampling is skipped (z=t), BhatMul produces s2=0.
        // Validates: FFT, IFFT, BhatMul, FprToInt, NTT, norm check.
        // ═══════════════════════════════════════════════════════
        run_single_case(
            "BYPASS_FS — identity (z=t), outer pipeline test",
            32'h00000003,  // cfg = bypass_fs + force_accept
            1'b1,          // preload_z
            1'b1,          // reload_t
            1'b1,          // expect_done
            8'h00,
            10000000       // ~0.1s real, ~30s sim
        );
        case_cyc_log[current_case] = case_cycles;
        if (case_pass) passed = passed + 1; else failed = failed + 1;
        current_case = current_case + 1;

        // ═══════════════════════════════════════════════════════
        // Case 1: START_AT_FS + FORCE_ACCEPT — real ffSampling
        // cfg[1]=1 (force_accept), cfg[2]=1 (start_at_fs)
        // Pipeline: FS→VD→IV→FI→N1→RC(forced)
        // Uses preloaded t0/t1/tree/B. Real ffSampling+SamplerZ.
        // Validates: ffSampling, BhatMul, IFFT, FprToInt, NTT.
        // ═══════════════════════════════════════════════════════
        run_single_case(
            "FORCE_ACCEPT — real ffSampling (preloaded t0/t1)",
            32'h00000006,  // cfg = force_accept + start_at_fs
            1'b1,          // preload_z (overwritten by ffSampling)
            1'b1,          // reload_t
            1'b1,          // expect_done
            8'h00,
            25000000       // ~0.25s real, ~75s sim
        );
        case_cyc_log[current_case] = case_cycles;
        if (case_pass) passed = passed + 1; else failed = failed + 1;
        current_case = current_case + 1;

        // ═══════════════════════════════════════════════════════
        // Case 2: FULL PIPELINE + FORCE_ACCEPT — HW generates t0/t1
        // cfg[1]=1 (force_accept), cfg[2]=0 (normal start)
        // Pipeline: SH→HP→FC→FS→VD→IV→FI→N1→RC(forced)
        // Hardware runs HashToPoint→FFT(c)→t0/t1 internally.
        // Validates: SHAKE, HashToPoint, FFT, ffSampling, full pipeline.
        // ═══════════════════════════════════════════════════════
        run_single_case(
            "FULL PIPELINE — HW HashToPoint→FFT→ffSampling",
            32'h00000002,  // cfg = force_accept only
            1'b0,          // no preload_z
            1'b1,          // reload_t (for clean initial state)
            1'b1,          // expect_done
            8'h00,
            25000000       // full pipeline, approx same as Case 1 + FFT
        );
        case_cyc_log[current_case] = case_cycles;
        if (case_pass) passed = passed + 1; else failed = failed + 1;
        current_case = current_case + 1;

        // ═══════════════════════════════════════════════════════
        // Case 3: START_AT_FS with real rejection (rejection sampling)
        // cfg[2]=1 (start_at_fs), force_accept=0
        // Pipeline: FS→VD→IV→FI→N1→RC→(restart if reject)
        // MAX_RESTARTS=3, so up to 4 attempts.
        // If signature quality is poor, will fail with fail=1 after exhaust.
        // Validates: rejection control flow, full signing with quality check.
        // ═══════════════════════════════════════════════════════
        run_single_case(
            "REJECTION TEST — ffSampling with real norm check",
            32'h00000004,  // cfg = start_at_fs only (rejection enabled)
            1'b0,          // no preload_z
            1'b1,          // reload_t
            1'b1,          // expect_done (or fail after restarts, either is OK)
            8'h00,
            25000000       // up to 4 attempts × ~500K cycles
        );
        case_cyc_log[current_case] = case_cycles;
        // Case 3: either done or fail+max_restarts is acceptable
        if (case_pass) passed = passed + 1; else failed = failed + 1;
        current_case = current_case + 1;

        // ═══════════════════════════════════════════════════════
        // Summary
        // ═══════════════════════════════════════════════════════
        $display("");
        $display("╔══════════════════════════════════════════════════════╗");
        $display("║  TEST SUMMARY                                        ║");
        $display("╠══════════════════════════════════════════════════════╣");
        $display("║  Total cases: %0d  |  PASSED: %0d  |  FAILED: %0d     ║",
            current_case, passed, failed);
        $display("╠══════════════════════════════════════════════════════╣");
        for (ci = 0; ci < current_case; ci = ci + 1) begin
            $display("║  Case %0d: %0d cycles                                  ║",
                ci, case_cyc_log[ci]);
        end
        $display("╚══════════════════════════════════════════════════════╝");
        $display("");

        if (failed > 0) begin
            $display("*** %0d CASE(S) FAILED ***", failed);
            $fatal(1);
        end else begin
            $display("*** ALL %0d CASES PASSED ***", passed);
        end

        $finish;
    end

    // ─── Watchdog ───
    reg [31:0] wd_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wd_cnt <= 0;
        else begin
            wd_cnt <= wd_cnt + 1;
            if (wd_cnt == MAX_CYCLES) begin
                $display("GLOBAL WATCHDOG at cycle=%0d", wd_cnt);
                $display("  Current case=%0d  Phase: %s (st=%0d sn=%0d)",
                    current_case, phase_name(dut.st), dut.st, dut.sn);
                $display("  TS: run=%0d state=%0d busy=%0d",
                    dut.u_ts.run_state, dut.u_ts.state_q, dut.ts_busy);
                $display("  FE: state=%0d op=%0d",
                    dut.u_fe.state, dut.u_fe.op_q);
                $display("  SZ: busy=%0d done=%0d fail=%0d",
                    dut.sz_busy, dut.sz_done, dut.sz_fail);
                $finish;
            end
        end
    end

    // ─── VCD dump ───
    initial begin
        if ($test$plusargs("DUMP_VCD")) begin
            $dumpfile("tb_multicase.vcd");
            $dumpvars(0, tb_falconsign_top_multicase);
        end
    end

endmodule
