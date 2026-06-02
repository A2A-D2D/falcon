`timescale 1ns/1ps
// FalconSign Verify Top Integration Test
//
// Tests the verify pipeline using bypass_decode mode: pre-loads s2 and h
// into memory, then runs SHAKE(message)→HashToPoint→NTT→NormCheck.
// The test message matches the sign flow's hardcoded message.

module tb_falconsign_verify_top;

    localparam [15:0] REG_CR      = 16'h0000;
    localparam [15:0] REG_SR      = 16'h0004;
    localparam [15:0] REG_CFG     = 16'h0008;
    localparam [15:0] REG_MEM_HI  = 16'h000C;
    localparam [15:0] REG_MSG_LEN = 16'h0010;

    localparam integer VERIFY_MSG_BASE   = 256;
    localparam integer VERIFY_SIG_BASE   = 0;
    localparam integer SIG_S2_BASE       = 6912;
    localparam integer C_INT_BASE        = 7424;
    localparam integer H_BASE            = 7456;
    localparam integer S1_BASE           = 7488;

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

    falconsign_verify_top #(.ADDR_W(13)) dut (
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

    // ─── Bus tasks ───
    task bus_write;
        input [15:0] addr;
        input [31:0] data;
        begin
            @(posedge clk);
            bus_cs   = 1'b1;
            bus_wr   = 1'b1;
            bus_addr = addr;
            bus_wdata = data;
            wait(bus_ready);
            @(posedge clk);
            bus_cs   = 1'b0;
            bus_wr   = 1'b0;
        end
    endtask

    // ─── Poke memory word via direct bank access ───
    task poke_mem;
        input integer addr;
        input [255:0] data;
        reg [1:0]  bank_sel;
        reg [10:0] bank_addr;
        begin
            bank_sel  = addr % 4;
            bank_addr = addr / 4;
            case (bank_sel)
                2'd0: dut.u_mem.bank0[bank_addr] = data;
                2'd1: dut.u_mem.bank1[bank_addr] = data;
                2'd2: dut.u_mem.bank2[bank_addr] = data;
                2'd3: dut.u_mem.bank3[bank_addr] = data;
            endcase
        end
    endtask

    // ─── Peek memory word ───
    function [255:0] peek_mem;
        input integer addr;
        reg [1:0]  bank_sel;
        reg [10:0] bank_addr;
        begin
            bank_sel  = addr % 4;
            bank_addr = addr / 4;
            case (bank_sel)
                2'd0: peek_mem = dut.u_mem.bank0[bank_addr];
                2'd1: peek_mem = dut.u_mem.bank1[bank_addr];
                2'd2: peek_mem = dut.u_mem.bank2[bank_addr];
                2'd3: peek_mem = dut.u_mem.bank3[bank_addr];
            endcase
        end
    endfunction

    // ─── Load hex file to memory ───
    task load_hex;
        input [8*80-1:0] filename;
        input integer base_addr;
        input integer count;
        begin
            load_fd = $fopen(filename, "r");
            if (!load_fd) begin
                $display("ERROR: cannot open %s", filename);
                $fatal(1, "File not found");
            end
            for (load_i = 0; load_i < count; load_i = load_i + 1) begin
                load_status = $fscanf(load_fd, "%h", load_buf);
                if (load_status != 1) begin
                    $display("ERROR: read failed at word %0d of %s (status=%0d)", load_i, filename, load_status);
                    $fatal(1, "Read error");
                end
                poke_mem(base_addr + load_i, load_buf);
            end
            $fclose(load_fd);
        end
    endtask

    // ─── Hardcoded test message (32 bytes = "FALCON_SIGN_TEST_MSG_V1.0______") ───
    // Keccak absorbs byte 0 from din[7:0], so each 64-bit word is byte-reversed.
    localparam [63:0] TEST_MSG_W0 = 64'h535F4E4F434C4146;
    localparam [63:0] TEST_MSG_W1 = 64'h545345545F4E4749;
    localparam [63:0] TEST_MSG_W2 = 64'h2E31565F47534D5F;
    localparam [63:0] TEST_MSG_W3 = 64'h5F5F5F5F5F5F5F30;

    reg [31:0] phase_tracker;
    reg [31:0] total_cycle;
    reg [31:0] timeout_cycles;
    reg        test_pass;
    integer    i;
    integer    s1_mismatches;
    integer    s2_mismatches;
    // task-local variables (Verilog-2005 requires these at module scope)
    reg [255:0] load_buf;
    integer     load_fd, load_i, load_status;
    reg [8*80-1:0] h_hex;
    reg [8*80-1:0] c_hex;
    reg [8*80-1:0] s2_hex;
    reg [8*80-1:0] s1_hex;
    reg [8*80-1:0] raw_sig_hex;
    reg             use_bypass_decode_hash;
    reg             use_raw_decode;
    reg             debug_hp;
    reg [31:0]      cfg_value;

    initial begin
        bus_cs   = 1'b0;
        bus_wr   = 1'b0;
        bus_addr = 16'd0;
        bus_wdata = 32'd0;
        total_cycle  = 32'd0;
        h_hex  = "h_ntt.hex";
        c_hex  = "hm.hex";
        s2_hex = "s2_expected.hex";
        s1_hex = "s1_expected.hex";
        raw_sig_hex = "rtl_raw_sig.hex";
        use_bypass_decode_hash = 1'b0;
        use_raw_decode = 1'b0;
        debug_hp = 1'b0;
        cfg_value = 32'h00000004;

        if ($value$plusargs("H_HEX=%s", h_hex))
            $display("Using H_HEX=%s", h_hex);
        if ($value$plusargs("C_HEX=%s", c_hex))
            $display("Using C_HEX=%s", c_hex);
        if ($value$plusargs("S2_HEX=%s", s2_hex))
            $display("Using S2_HEX=%s", s2_hex);
        if ($value$plusargs("S1_HEX=%s", s1_hex))
            $display("Using S1_HEX=%s", s1_hex);
        if ($value$plusargs("RAW_SIG_HEX=%s", raw_sig_hex))
            $display("Using RAW_SIG_HEX=%s", raw_sig_hex);
        if ($test$plusargs("BYPASS_DECODE_HASH")) begin
            use_bypass_decode_hash = 1'b1;
            cfg_value = 32'h00000002;  // bit[1]=bypass_decode, run SH+HP+N1+RC
            $display("Using BYPASS_DECODE_HASH mode");
        end
        if ($test$plusargs("RAW_DECODE_MSGONLY")) begin
            use_raw_decode = 1'b1;
            cfg_value = 32'h00000008;  // bit[3]=decode raw s2, hash message-only
            $display("Using RAW_DECODE_MSGONLY mode");
        end
        if ($test$plusargs("RAW_DECODE_STANDARD")) begin
            use_raw_decode = 1'b1;
            cfg_value = 32'h00000000;  // decode raw s2, hash nonce||message
            $display("Using RAW_DECODE_STANDARD mode");
        end
        if ($test$plusargs("DEBUG_HP")) begin
            debug_hp = 1'b1;
            $display("Using DEBUG_HP tracing");
        end

        repeat(5) @(posedge clk);

        $display("=== FalconSign Verify Integration Test ===");

        // ─── Load h coefficients ───
        $display("Loading %s -> H_BASE...", h_hex);
        load_hex(h_hex, H_BASE, 32);

        // ─── Load s2 coefficients ───
        if (use_raw_decode) begin
            $display("Loading %s -> VERIFY_SIG_BASE...", raw_sig_hex);
            load_hex(raw_sig_hex, VERIFY_SIG_BASE, 22);
        end else begin
            $display("Loading %s -> SIG_S2_BASE...", s2_hex);
            load_hex(s2_hex, SIG_S2_BASE, 32);
        end

        // ─── Write test message to VERIFY_MSG_BASE ───
        // Message: 32 bytes = 4 x 64-bit, packed into one 256-bit memory word
        $display("Writing test message to VERIFY_MSG_BASE...");
        poke_mem(VERIFY_MSG_BASE + 0, {TEST_MSG_W3, TEST_MSG_W2, TEST_MSG_W1, TEST_MSG_W0});

        // ─── Optional c preload for start_at_ntt smoke mode ───
        if (!use_bypass_decode_hash && !use_raw_decode) begin
            $display("Loading %s -> C_INT_BASE...", c_hex);
            load_hex(c_hex, C_INT_BASE, 32);
        end

        // ─── Configure and start ───
        $display("Configuring REG_CFG=0x%08h...", cfg_value);
        bus_write(REG_CFG, cfg_value);

        $display("Starting verify...");
        bus_write(REG_CR, 32'd1);

        // ─── Wait for completion ───
        phase_tracker = 32'd0;
        timeout_cycles = 32'd300000;
        @(posedge clk);
        while (!done && !fail) begin
            @(posedge clk);
            total_cycle = total_cycle + 1;
            if (dut.st != phase_tracker[3:0]) begin
                phase_tracker = {28'd0, dut.st};
                case (dut.st)
                    4'd0: $display("[T=%0d] PHASE: SI_Idle", $time);
                    4'd1: $display("[T=%0d] PHASE: DS_SigDecode", $time);
                    4'd2: $display("[T=%0d] PHASE: SH_ShakeAbsorb", $time);
                    4'd3: $display("[T=%0d] PHASE: HP_HashToPoint", $time);
                    4'd4: $display("[T=%0d] PHASE: N1_NTT_Compute", $time);
                    4'd5: $display("[T=%0d] PHASE: RC_NormCheck", $time);
                    4'd6: $display("[T=%0d] PHASE: VF_VerifyFinish", $time);
                    4'd7: $display("[T=%0d] PHASE: SD_SendDone", $time);
                endcase
            end
            if (debug_hp && dut.shake_dout_valid && (dut.u_htp.idx < 10'd8)) begin
                $display("[DBG %0d] shake dout=%016h fifo_count=%0d", total_cycle, dut.shake_dout, dut.u_shake_fifo.count);
            end
            if (debug_hp && dut.htp_coeff_valid && (dut.u_htp.idx < 10'd8)) begin
                $display("[DBG %0d] htp idx=%0d coeff=%04h", total_cycle, dut.u_htp.idx, dut.htp_coeff);
            end
            if (debug_hp && dut.hp_wr_en && (dut.hp_wr_addr < (C_INT_BASE + 3))) begin
                $display("[DBG %0d] hp write addr=%0d data=%h", total_cycle, dut.hp_wr_addr, dut.hp_wr_data);
            end
            if (debug_hp && use_raw_decode && (total_cycle < 32'd80)) begin
                $display("[DBG %0d] top st=%0d sn=%0d sd_state=%0d sd_done=%0d sd_fail=%0d sd_status=%02h rd=%0d rd_addr=%0d wr=%0d wr_addr=%0d byte_off=%0d nonce_cnt=%0d coeff_idx=%0d",
                         total_cycle, dut.st, dut.sn, dut.u_sig_decode.state,
                         dut.sd_done, dut.sd_fail, dut.sd_status,
                         dut.sd_mem_rd_en, dut.sd_mem_rd_addr,
                         dut.sd_mem_wr_en, dut.sd_mem_wr_addr,
                         dut.u_sig_decode.sig_byte_off, dut.u_sig_decode.nonce_cnt,
                         dut.u_sig_decode.coeff_idx);
            end
            if (total_cycle > timeout_cycles) begin
                $display("");
                $display("ERROR: verify timeout at cycle %0d", total_cycle);
                $display("  st=%0d sn=%0d sh_f_state=%0d sh_word_idx=%0d sh_seen_busy=%0d",
                         dut.st, dut.sn, dut.sh_f_state, dut.sh_word_idx, dut.sh_seen_busy);
                $display("  shake_state=%0d shake_ready=%0d shake_dout_valid=%0d fifo_count=%0d",
                         dut.u_shake.state, dut.shake_ready, dut.shake_dout_valid, dut.u_shake_fifo.count);
                $display("  htp_idx=%0d htp_ready=%0d htp_hash_ready=%0d htp_hash_valid=%0d",
                         dut.u_htp.idx, dut.htp_ready, dut.htp_hash_ready, dut.htp_hash_valid);
                $fatal(1, "Verify timeout");
            end
        end

        $display("");
        $display("=== VERIFY COMPLETE: total_cycles=%0d ===", total_cycle);
        $display("  done=%0d  fail=%0d  irq=%0d  status=0x%02h",
                 done, fail, bus_irq, status);

        // ─── Compare c polynomial with sign flow hm.hex ───
        $display("");
        $display("=== Comparing c with sign flow %s ===", c_hex);
        load_hex(c_hex, 4000, 32);
        $display("  c[0] got=%h exp=%h %s",
                 peek_mem(C_INT_BASE + 0), peek_mem(4000 + 0),
                 (peek_mem(C_INT_BASE + 0) === peek_mem(4000 + 0)) ? "OK" : "MISMATCH");

        // ─── Compare s1 with sign flow expected ───
        $display("");
        if (use_raw_decode) begin
            $display("");
            $display("=== Comparing decoded s2 with %s ===", s2_hex);
            load_hex(s2_hex, 4064, 32);
            s2_mismatches = 0;
            for (i = 0; i < 32; i = i + 1) begin
                if (peek_mem(SIG_S2_BASE + i) !== peek_mem(4064 + i)) begin
                    s2_mismatches = s2_mismatches + 1;
                    if (s2_mismatches <= 8) begin
                        $display("  s2 mismatch[%0d]: got=%h exp=%h",
                                 i, peek_mem(SIG_S2_BASE + i), peek_mem(4064 + i));
                    end
                end
            end
            if (s2_mismatches == 0)
                $display("  decoded s2 matches expected (32/32 OK)");
            else
                $display("  decoded s2 mismatches: %0d/32", s2_mismatches);
        end

        $display("=== Comparing s1 with sign flow %s ===", s1_hex);
        load_hex(s1_hex, 4032, 32);
        s1_mismatches = 0;
        for (i = 0; i < 32; i = i + 1) begin
            if (peek_mem(S1_BASE + i) !== peek_mem(4032 + i)) begin
                $display("  s1 mismatch[%0d]: got=%h exp=%h",
                         i, peek_mem(S1_BASE + i), peek_mem(4032 + i));
                s1_mismatches = s1_mismatches + 1;
            end
        end
        if (s1_mismatches == 0)
            $display("  s1 matches sign flow expected (32/32 OK)");
        else
            $display("  s1 mismatches: %0d/32", s1_mismatches);

        // ─── Final verdict ───
        $display("");
        test_pass = (done && !fail && status == 8'h00 && s1_mismatches == 0);
        if (test_pass) begin
            $display("======================================");
            $display("  TEST PASSED: Signature verified OK");
            $display("======================================");
        end else begin
            $display("======================================");
            $display("  TEST FAILED: done=%0d fail=%0d status=0x%02h s1_mismatches=%0d",
                     done, fail, status, s1_mismatches);
            $display("======================================");
        end

        $display("");
        if (!test_pass)
            $fatal(1, "Verify test failed");
        else begin
            $display("All checks passed.");
            $finish;
        end
    end

endmodule
