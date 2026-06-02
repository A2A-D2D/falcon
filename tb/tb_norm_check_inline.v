`timescale 1ns/1ps

module tb_norm_check_inline;

    // Test the norm check logic directly
    reg [255:0] test_data;
    reg [63:0] word_sum_s2, word_sum_s1;
    reg signed [15:0] sum_lane;
    reg [15:0] sum_abs;
    reg [15:0] sum_center_u;
    integer sum_i;
    
    localparam [15:0] HALF_Q_U16 = 16'd6144;
    localparam signed [15:0] Q_I16 = 16'sd12289;

    initial begin
        // Test with s2[0] = 001fff6200210029ffe3ff48ff6a0120ffbb0108ff99fecd0032011efed10019
        test_data = 256'h001fff6200210029ffe3ff48ff6a0120ffbb0108ff99fecd0032011efed10019;
        
        word_sum_s2 = 64'd0;
        word_sum_s1 = 64'd0;
        
        $display("=== Testing norm check logic ===");
        $display("Test data: %h", test_data);
        $display("");
        
        for (sum_i = 0; sum_i < 16; sum_i = sum_i + 1) begin
            sum_lane = test_data[sum_i*16 +: 16];
            sum_abs = sum_lane[15] ? (~sum_lane + 1'b1) : sum_lane;
            
            $display("Lane %0d: raw=%h, signed=%0d, abs=%0d, sq=%0d", 
                     sum_i, test_data[sum_i*16 +: 16], sum_lane, sum_abs,
                     sum_abs * sum_abs);
            
            word_sum_s2 = word_sum_s2 + {{32{1'b0}}, sum_abs} * {{32{1'b0}}, sum_abs};
        end
        
        $display("");
        $display("Total word_sum_s2 = %0d", word_sum_s2);
        $display("Expected (25^2 + 303^2 + ...) = %0d", 
                 25*25 + 303*303 + 286*286 + 50*50 + 307*307 + 103*103 + 264*264 + 69*69 +
                 288*288 + 150*150 + 184*184 + 29*29 + 41*41 + 33*33 + 158*158 + 31*31);
    end
endmodule
