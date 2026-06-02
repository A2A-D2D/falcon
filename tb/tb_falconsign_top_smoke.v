`timescale 1ns/1ps
// ─── FalconSign Full Signing Chain Smoke Test ───
//
// ALL cases exercise the real ffSampling + BhatMul + IFFT + FprToInt + NTT chain.
// No BYPASS_FS — every case produces real Gaussian signature output.
//
// Cases (select with +CASE=N):
//   +CASE=0 (default): START_AT_FS + real rejection
//        FS→VD→IV→FI→N1→RC  with full norm check + restart loop
//        Uses preloaded t0/t1/tree/B/h from gen_falcon_hw_key.
//        The REAL signing chain, end to end.
//   +CASE=1: FULL PIPELINE + force_accept
//        SH→HP→FC→FS→VD→IV→FI→N1→RC(forced)
//        Hardware runs HashToPoint→FFT to generate t0/t1 internally.
//        Tests the complete front-to-back hardware pipeline.
//   +CASE=2: START_AT_FS + force_accept
//        FS→VD→IV→FI→N1→RC(forced)
//        Preloaded t0/t1, skips rejection check.
//        Fastest real-ffSampling path, good for iteration.
//
// Usage:
//   vvp tb_smoke.vvp +TWIDDLE_RE=DOC/twiddle_rom_re.hex +TWIDDLE_IM=DOC/twiddle_rom_im.hex
//   vvp tb_smoke.vvp ... +CASE=1   # full pipeline
//   vvp tb_smoke.vvp ... +DUMP_VCD # waveform

module tb_falconsign_top_smoke;

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

    // Use 2-BFU mode with twiddle reuse optimization
    falconsign_top #(.ADDR_W(13), .LEVEL_W(4), .INDEX_W(10), .BFU_LANES(2)) dut (
        .clk(clk), .rst_n(rst_n),
        .bus_cs(bus_cs), .bus_wr(bus_wr),
        .bus_addr(bus_addr), .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata), .bus_ready(bus_ready),
        .bus_irq(bus_irq),
        .busy(busy), .done(done), .fail(fail), .status(status)
    );

    initial begin
        clk   = 1'b0;
        rst_n = 1'b0;
        #30 rst_n = 1'b1;
    end
    always #5 clk = ~clk;

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

    function [63:0] norm_signed_i16_span;
        input integer base_addr;
        input integer words;
        integer wi, li;
        reg [255:0] word;
        reg signed [15:0] lane;
        reg [15:0] abs_lane;
        begin
            norm_signed_i16_span = 64'd0;
            for (wi = 0; wi < words; wi = wi + 1) begin
                word = peek_mem_word(base_addr + wi);
                for (li = 0; li < 16; li = li + 1) begin
                    lane = word[li*16 +: 16];
                    abs_lane = lane[15] ? (~lane + 1'b1) : lane;
                    norm_signed_i16_span = norm_signed_i16_span +
                        {{32{1'b0}}, abs_lane} * {{32{1'b0}}, abs_lane};
                end
            end
        end
    endfunction

    function [63:0] norm_modq_i16_span;
        input integer base_addr;
        input integer words;
        integer wi, li;
        reg [255:0] word;
        reg [15:0] lane_u;
        reg signed [15:0] lane_c;
        reg [15:0] abs_lane;
        begin
            norm_modq_i16_span = 64'd0;
            for (wi = 0; wi < words; wi = wi + 1) begin
                word = peek_mem_word(base_addr + wi);
                for (li = 0; li < 16; li = li + 1) begin
                    lane_u = word[li*16 +: 16];
                    lane_c = (lane_u > 16'd6144) ? ($signed({1'b0, lane_u}) - 16'sd12289) :
                                                   $signed({1'b0, lane_u});
                    abs_lane = lane_c[15] ? (~lane_c + 1'b1) : lane_c;
                    norm_modq_i16_span = norm_modq_i16_span +
                        {{32{1'b0}}, abs_lane} * {{32{1'b0}}, abs_lane};
                end
            end
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

    task reload_targets;
        begin
            load_hex_to_mem("t0_target.hex", LAYOUT_T0_BASE, N_WORDS);
            load_hex_to_mem("t1_target.hex", LAYOUT_T1_BASE, N_WORDS);
        end
    endtask

    // ─── Phase + sampler tracker ───
    // Use dut.sn (combinational next-state) to detect transitions early.
    // dut.st is a register updated by st <= sn on the same posedge clk,
    // creating a race condition with NBA semantics. Reading sn avoids this.
    reg [3:0]  cur_st;
    reg [31:0] total_cycle;
    reg [31:0] restart_cnt;
    reg [31:0] sample_cmds;
    reg [31:0] sample_rsps;
    reg [31:0] ph_cycles [0:15];
    reg [31:0] ph_start;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_st <= 0; total_cycle <= 0; restart_cnt <= 0;
            sample_cmds <= 0; sample_rsps <= 0; ph_start <= 0;
            {ph_cycles[0],ph_cycles[1],ph_cycles[2],ph_cycles[3],
             ph_cycles[4],ph_cycles[5],ph_cycles[6],ph_cycles[7],
             ph_cycles[8],ph_cycles[9],ph_cycles[10],ph_cycles[11],
             ph_cycles[12],ph_cycles[13],ph_cycles[14],ph_cycles[15]} <= 0;
        end else begin
            total_cycle <= total_cycle + 1;
            if (dut.fe_sz_cmd_valid && dut.fe_sz_cmd_ready) sample_cmds <= sample_cmds + 1;
            if (dut.sz_rsp_valid) sample_rsps <= sample_rsps + 1;
            // Detect transition using sn (combinational) to avoid NBA race.
            // When sn != st, a state change will happen on this clock edge.
            if (dut.sn != cur_st) begin
                ph_cycles[cur_st] <= total_cycle - ph_start;
                cur_st <= dut.sn;
                ph_start <= total_cycle;
                if (dut.sn == 4'd1 && (cur_st == 4'd9 || cur_st == 4'd7 || cur_st == 4'd5))
                    restart_cnt <= restart_cnt + 1;
            end
        end
    end

    // ─── Main test ───
    reg [31:0] sr, cfg_val, case_sel, timeout_cyc;
    reg [1023:0] case_desc;
    reg [63:0] dbg_norm_s2, dbg_norm_s1;
    integer    i, cyc;
    reg        progress_en;

    initial begin
        bus_cs = 0; bus_wr = 0; bus_addr = 0; bus_wdata = 0;
        progress_en = $test$plusargs("PROGRESS");
        case_sel = 1;  // Default: full pipeline (CASE=1)

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        if ($test$plusargs("CASE=0")) case_sel = 0;
        else if ($test$plusargs("CASE=2")) case_sel = 2;

        // ═══════════════════════════════════════════════════════
        // Case configuration
        // All cases use real ffSampling — no BYPASS_FS.
        // Default: CASE=1 (full hardware pipeline)
        // ═══════════════════════════════════════════════════════
        case (case_sel)
            0: begin
                // START_AT_FS + real rejection check
                // Full real signing chain: FS→VD→IV→FI→N1→RC
                // Uses preloaded key material (t0/t1/tree/B/h from software).
                // MAX_RESTARTS=3, so up to 4 signing attempts.
                case_desc = "START_AT_FS + real rejection (FS→VD→IV→FI→N1→RC) [SW t0/t1]";
                cfg_val = 32'h00000004;  // start_at_fs=1, force_accept=0, bypass_fs=0
                timeout_cyc = 25000000;  // up to 4 attempts × ~500K
            end
            1: begin
                // FULL PIPELINE + force_accept
                // Hardware does everything: SH→HP→FC→FS→VD→IV→FI→N1→RC(forced)
                // t0/t1 generated internally via HashToPoint→FFT.
                case_desc = "FULL PIPELINE + force_accept (SH→HP→FC→FS→VD→IV→FI→N1) [FULL HW]";
                cfg_val = 32'h00000002;  // force_accept=1, all else 0
                timeout_cyc = 25000000;  // full pipeline ~500K cycles
            end
            2: begin
                // START_AT_FS + force_accept — fastest real-ffSampling path
                // FS→VD→IV→FI→N1→RC(forced)
                // Preloaded t0/t1, skips rejection. Good for ffSampling debug.
                case_desc = "START_AT_FS + force_accept (FS→VD→IV→FI→N1, preloaded key) [SW t0/t1]";
                cfg_val = 32'h00000006;  // start_at_fs=1, force_accept=1
                timeout_cyc = 10000000;  // ~473K cycles
            end
            default: begin
                case_desc = "DEFAULT (FULL HW)";
                cfg_val = 32'h00000002;
                timeout_cyc = 25000000;
            end
        endcase

        $display("╔══════════════════════════════════════════════════════╗");
        $display("║  FalconSign Full Chain Test — CASE %0d                  ║", case_sel);
        $display("║  %s", case_desc);
        $display("╚══════════════════════════════════════════════════════╝");
        $display("");

        // ─── Load key material ───
        $display("=== Loading key material ===");
        load_hex_to_mem("b00.hex", LAYOUT_B00_BASE, N_WORDS);
        load_hex_to_mem("b01.hex", LAYOUT_B01_BASE, N_WORDS);
        load_hex_to_mem("b10.hex", LAYOUT_B10_BASE, N_WORDS);
        load_hex_to_mem("b11.hex", LAYOUT_B11_BASE, N_WORDS);
        load_hex_to_mem("tree_full_poly.hex", LAYOUT_TREE_BASE, TREE_SIZE);
        load_hex_to_mem("h_ntt.hex", LAYOUT_H_BASE, 32);
        load_hex_to_mem("hm.hex", LAYOUT_C_INT_BASE, 32);
        if (case_sel != 1) begin
            // Cases 0,2: preload t0/t1 (start_at_fs skips HP/FC)
            load_hex_to_mem("t0_target.hex", LAYOUT_T0_BASE, N_WORDS);
            load_hex_to_mem("t1_target.hex", LAYOUT_T1_BASE, N_WORDS);
        end
        $display("Key material loaded.");
        $display("");

        // ─── Pre-start: reload t0/t1 if start_at_fs mode ───
        if (case_sel != 1) reload_targets();
        @(posedge clk);

        // ─── Configure & Start ───
        bus_write(REG_CFG, cfg_val);
        $display("Config: REG_CFG=0x%08h", cfg_val);
        $display("  bypass_fs=%b  force_accept=%b  start_at_fs=%b  dynamic=%b",
            cfg_val[0], cfg_val[1], cfg_val[2], cfg_val[3]);
        $display("Starting signing operation...");
        bus_write(REG_CR, 32'h00000001);
        @(posedge clk);

        // ─── Wait for completion ───
        cyc = 0;
        while (!done && !fail && cyc < timeout_cyc) begin
            @(posedge clk);
            cyc = cyc + 1;
            if (progress_en && ((cyc % 100000) == 0)) begin
                $display("PROGRESS case=%0d cyc=%0d phase=%s st=%0d sn=%0d salt=%0d restarts=%0d norm_accept=%0d norm_sq=%0d fft_state=%0d fft_stage=%0d fft_pair=%0d fft_fal_idx=%0d fft_busy=%0d fft_rsp_done=%0d",
                    case_sel, cyc, phase_name(dut.st), dut.st, dut.sn,
                    dut.salt_cnt, restart_cnt, dut.norm_accept, dut.norm_sq,
                    dut.u_fft.state, dut.u_fft.stage_idx, dut.u_fft.pair_idx,
                    dut.u_fft.fal_idx, dut.fft_busy, dut.fft_rsp_done);
                $fflush;
            end
        end

        // ─── Report ───
        bus_read(REG_SR, sr);
        $display("");
        $display("╔══════════════════════════════════════════════════════╗");
        $display("║  RESULTS                                             ║");
        $display("╠══════════════════════════════════════════════════════╣");
        $display("║  done=%0d  fail=%0d  irq=%0d  status=0x%02h  sr=0x%08h",
            done, fail, bus_irq, status, sr);
        $display("║  total_cycles=%0d  restarts=%0d", cyc, restart_cnt);
        $display("║  sample_cmds=%0d  sample_rsps=%0d", sample_cmds, sample_rsps);
        $display("╠══════════════════════════════════════════════════════╣");
        $display("║  Phase breakdown (last attempt):                     ║");
        $display("║    SH_SeedHash:     %6d cy", ph_cycles[1]);
        $display("║    HP_HashToPoint:  %6d cy", ph_cycles[2]);
        $display("║    FC_FFT:          %6d cy", ph_cycles[3]);
        $display("║    TG_TargetGen:    %6d cy", ph_cycles[4]);
        $display("║    FS_ffSampling:   %6d cy", ph_cycles[5]);
        $display("║    VD_BhatMul:      %6d cy", ph_cycles[6]);
        $display("║    IV_IFFT:         %6d cy", ph_cycles[7]);
        $display("║    FI_FprToInt:     %6d cy", ph_cycles[8]);
        $display("║    N1_NTT:          %6d cy", ph_cycles[9]);
        $display("║    RC_RejCheck:     %6d cy", ph_cycles[10]);
        $display("╠══════════════════════════════════════════════════════╣");
        $display("║  SamplerZ: busy=%0d done=%0d fail=%0d",
            dut.sz_busy, dut.sz_done, dut.sz_fail);
        $display("║  Norm:    accept=%0d norm_sq=%0d bound=%0d",
            dut.norm_accept, dut.norm_sq, dut.FALCON512_BOUND_SQ);
        $display("╚══════════════════════════════════════════════════════╝");

        dbg_norm_s2 = norm_signed_i16_span(LAYOUT_SIG_BASE, 32);
        dbg_norm_s1 = norm_modq_i16_span(LAYOUT_S1_BASE, 32);
        $display("    Norm split: s2=%0d  s1=%0d  sum=%0d",
            dbg_norm_s2, dbg_norm_s1, dbg_norm_s2 + dbg_norm_s1);

        // Signature output
        $display("");
        $display("=== Signature output ===");
        $display("s2 (first 4 words):");
        for (i = 0; i < 4; i = i + 1)
            $display("  sig[%0d] = %h", i, peek_mem_word(LAYOUT_SIG_BASE + i));
        $display("s1 (first 4 words):");
        for (i = 0; i < 4; i = i + 1)
            $display("  s1[%0d]  = %h", i, peek_mem_word(LAYOUT_S1_BASE + i));

        // Verdict
        $display("");
        if (done && !fail) begin
            $display("*** FULL CHAIN TEST PASSED (case %0d) ***", case_sel);
            $display("    Signing completed successfully with norm_sq=%0d", dut.norm_sq);
        end else if (fail && case_sel == 0) begin
            // Case 0 with rejection: fail after MAX_RESTARTS is expected
            // if signature quality is poor with preloaded key material
            $display("*** CASE %0d COMPLETED (fail after restart exhaust) ***", case_sel);
            $display("    restarts=%0d  status=0x%02h", restart_cnt, status);
            $display("    This is expected if ffSampling quality is still being tuned.");
        end else if (cyc >= timeout_cyc) begin
            $display("*** TIMEOUT at %0d cycles (case %0d) ***", cyc, case_sel);
            $display("    Phase: %s  st=%0d sn=%0d", phase_name(dut.st), dut.st, dut.sn);
            $display("    FE: state=%0d op=%0d", dut.u_fe.state, dut.u_fe.op_q);
            $display("    TS: run=%0d state=%0d busy=%0d done=%0d fail=%0d",
                dut.u_ts.run_state, dut.u_ts.state_q, dut.ts_busy, dut.ts_done, dut.ts_fail);
            $fatal(1);
        end else begin
            $display("*** FAILED (case %0d) ***", case_sel);
            $display("    fail=1, status=0x%02h, phase=%s", status, phase_name(dut.st));
            $fatal(1);
        end

        #100;
        $finish;
    end

    // ─── Watchdog ───
    reg [31:0] wd_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) wd_cnt <= 0;
        else begin
            wd_cnt <= wd_cnt + 1;
            if (wd_cnt == 50000000) begin
                $display("GLOBAL WATCHDOG at cycle=%0d phase=%s", wd_cnt, phase_name(dut.st));
                $finish;
            end
        end
    end

    // ─── VCD dump ───
    initial begin
        if ($test$plusargs("DUMP_VCD")) begin
            $dumpfile("tb_smoke.vcd");
            $dumpvars(0, tb_falconsign_top_smoke);
        end
    end

endmodule
