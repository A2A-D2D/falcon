`timescale 1ns/1ps

module tb_norm_check_debug3;

    reg [15:0] s1_vals [0:15];
    reg signed [15:0] s2_vals [0:15];
    reg signed [31:0] s1_centered [0:15];
    reg [63:0] s1_norm, s2_norm;
    integer i;

    initial begin
        // s1[0] = 005300a02f9f00512f602f42000a2ff800c62f392f142faf012000472fc00076
        s1_vals[0]  = 16'h0076;  // 118
        s1_vals[1]  = 16'h2fc0;  // 12224
        s1_vals[2]  = 16'h0047;  // 71
        s1_vals[3]  = 16'h2faf;  // 12207
        s1_vals[4]  = 16'h2f14;  // 12052
        s1_vals[5]  = 16'h2f39;  // 12089
        s1_vals[6]  = 16'h00c6;  // 198
        s1_vals[7]  = 16'h2ff8;  // 12280
        s1_vals[8]  = 16'h000a;  // 10
        s1_vals[9]  = 16'h2f42;  // 12098
        s1_vals[10] = 16'h2f60;  // 12128
        s1_vals[11] = 16'h0051;  // 81
        s1_vals[12] = 16'h2f9f;  // 12191
        s1_vals[13] = 16'h00a0;  // 160
        s1_vals[14] = 16'h0053;  // 83
        s1_vals[15] = 16'h0000;  // 0

        // s2[0] = 001fff6200210029ffe3ff48ff6a0120ffbb0108ff99fecd0032011efed10019
        s2_vals[0]  = 16'sh0019;  // 25
        s2_vals[1]  = 16'shfed1;  // -303
        s2_vals[2]  = 16'sh011e;  // 286
        s2_vals[3]  = 16'sh0032;  // 50
        s2_vals[4]  = 16'shfecd;  // -307
        s2_vals[5]  = 16'shff99;  // -103
        s2_vals[6]  = 16'sh0108;  // 264
        s2_vals[7]  = 16'shffbb;  // -69
        s2_vals[8]  = 16'sh0120;  // 288
        s2_vals[9]  = 16'shff6a;  // -150
        s2_vals[10] = 16'shff48;  // -184
        s2_vals[11] = 16'shffe3;  // -29
        s2_vals[12] = 16'sh0029;  // 41
        s2_vals[13] = 16'sh0021;  // 33
        s2_vals[14] = 16'shff62;  // -158
        s2_vals[15] = 16'sh001f;  // 31

        // Center-lift s1 values (use 32-bit to avoid overflow)
        $display("=== s1 values (center-lifted) ===");
        s1_norm = 0;
        for (i = 0; i < 16; i = i + 1) begin
            if (s1_vals[i] > 16'd6144) begin
                s1_centered[i] = $signed({16'd0, s1_vals[i]}) - 32'sd12289;
            end else begin
                s1_centered[i] = $signed({16'd0, s1_vals[i]});
            end
            $display("  s1[%0d] = %0d -> %0d (sq=%0d)", 
                     i, s1_vals[i], s1_centered[i], 
                     s1_centered[i] * s1_centered[i]);
            s1_norm = s1_norm + $unsigned(s1_centered[i] * s1_centered[i]);
        end
        $display("  s1 partial norm (16 coeffs) = %0d", s1_norm);

        // s2 values (already signed)
        $display("");
        $display("=== s2 values ===");
        s2_norm = 0;
        for (i = 0; i < 16; i = i + 1) begin
            $display("  s2[%0d] = %0d (sq=%0d)", 
                     i, s2_vals[i], 
                     s2_vals[i] * s2_vals[i]);
            s2_norm = s2_norm + $unsigned(s2_vals[i] * s2_vals[i]);
        end
        $display("  s2 partial norm (16 coeffs) = %0d", s2_norm);

        $display("");
        $display("=== Summary ===");
        $display("  s1 partial norm (16 coeffs) = %0d", s1_norm);
        $display("  s2 partial norm (16 coeffs) = %0d", s2_norm);
        $display("  Total partial norm = %0d", s1_norm + s2_norm);
        $display("  Average per coeff = %0d", (s1_norm + s2_norm) / 32);
        $display("  Estimated full norm (512 coeffs) = %0d", (s1_norm + s2_norm) * 16);
        $display("  Bound = 34034726");
    end
endmodule
