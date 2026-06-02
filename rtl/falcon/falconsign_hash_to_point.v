`timescale 1ns/1ps
// Module: falconsign_hash_to_point
// Purpose: rejection-sample SHAKE output into N mod-q challenge coefficients.
//
// HashToPoint: SHAKE256 output -> polynomial c in Z_q[x], q=12289.
// Falcon reference rule: consume SHAKE bytes two at a time, form
// w = (b0 << 8) | b1, accept if w < 5*q (61445), output w mod q.
module falconsign_hash_to_point #(parameter N=512)(
    input  wire        clk, rst_n,
    input  wire        start,
    output reg         ready,
    input  wire [63:0] hash_word,    // 64-bit word from SHAKE256
    input  wire        hash_valid,
    output reg         hash_ready,
    output reg [15:0]  coeff,        // one coefficient per valid cycle
    output reg         coeff_valid
);
    localparam [15:0] Q      = 16'd12289;
    localparam [15:0] FIVE_Q = 16'd61445;

    reg [9:0]  idx;       // coefficient index 0..N-1
    reg [1:0]  pair_pos;  // 2-byte pair inside the 64-bit word (0..3)
    reg [63:0] word_q;
    reg        word_valid;

    reg [15:0] sample;
    reg [15:0] sample_mod;

    // Candidate reduction for the current 16-bit SHAKE sample.
    always @(*) begin
        case (pair_pos)
            2'd0: sample = {word_q[7:0],   word_q[15:8]};
            2'd1: sample = {word_q[23:16], word_q[31:24]};
            2'd2: sample = {word_q[39:32], word_q[47:40]};
            default: sample = {word_q[55:48], word_q[63:56]};
        endcase

        sample_mod = sample;
        if (sample_mod >= Q) sample_mod = sample_mod - Q;
        if (sample_mod >= Q) sample_mod = sample_mod - Q;
        if (sample_mod >= Q) sample_mod = sample_mod - Q;
        if (sample_mod >= Q) sample_mod = sample_mod - Q;
    end

    // HashToPoint FSM. It consumes two-byte samples from each SHAKE word,
    // rejects values >= 5*q, and stores accepted coefficients.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready<=1; hash_ready<=0; coeff_valid<=0; coeff<=0;
            idx<=0; pair_pos<=0; word_q<=0; word_valid<=0;
        end else begin
            hash_ready<=0; coeff_valid<=0;

            if (start) begin
                idx<=0; pair_pos<=0; word_valid<=0;
                hash_ready<=1;
                ready<=0;
            end else if (idx >= N) begin
                ready <= 1;
                word_valid <= 0;
            end else begin
                if (!word_valid) begin
                    hash_ready <= 1;
                    if (hash_valid) begin
                        word_q <= hash_word;
                        word_valid <= 1;
                        pair_pos <= 0;
                        hash_ready <= 0;
                    end
                end else begin
                    if (sample < FIVE_Q) begin
                        coeff <= sample_mod;
                        coeff_valid <= 1;
                        idx <= idx + 1;
                    end

                    if (pair_pos == 2'd3) begin
                        word_valid <= 0;
                        pair_pos <= 0;
                        hash_ready <= 1;
                    end else begin
                        pair_pos <= pair_pos + 1'b1;
                    end
                end
            end
        end
    end

endmodule
