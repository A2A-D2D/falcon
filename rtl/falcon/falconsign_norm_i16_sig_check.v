`timescale 1ns/1ps

// Module: falconsign_norm_i16_sig_check
// Purpose: compute ||s1||^2 + ||s2||^2 from packed int16 signature words and
// compare the total with the Falcon-512 acceptance bound.
//
// Falcon signature norm check for packed int16 buffers.
// s2 is stored as signed int16; s1 is stored modulo q and center-lifted here.
//
// Optimization: time-multiplexed single multiplier (1 DSP instead of 16).
// Each word takes 16 cycles (one per lane) instead of 1 cycle.
module falconsign_norm_i16_sig_check #(
    parameter ADDR_W = 11
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              start,
    output wire              start_ready,
    input  wire [ADDR_W-1:0] s2_base,
    input  wire [ADDR_W-1:0] s1_base,
    input  wire [ADDR_W-1:0] word_count,
    input  wire [63:0]       bound_sq,

    output reg               mem_rd_en,
    output reg  [ADDR_W-1:0] mem_rd_addr,
    input  wire [255:0]      mem_rd_data,

    output reg               done,
    output reg               accept,
    output reg               fail,
    output reg  [7:0]        status,
    output reg  [63:0]       norm_sq
);

    localparam [2:0] ST_IDLE = 3'd0;
    localparam [2:0] ST_REQ  = 3'd1;
    localparam [2:0] ST_CAP  = 3'd2;
    localparam [2:0] ST_CALC = 3'd3;
    localparam [2:0] ST_DONE = 3'd4;

    localparam signed [15:0] Q_I16      = 16'sd12289;
    localparam        [15:0] HALF_Q_U16 = 16'd6144;

    reg [2:0] state;
    reg [ADDR_W-1:0] idx_q;
    reg [3:0] lane_idx;
    reg check_s1_q;
    reg reject_q;

    // Current lane extraction
    reg [255:0] word_buf;
    wire signed [15:0] cur_lane = word_buf[lane_idx*16 +: 16];
    wire [15:0] cur_lane_u = word_buf[lane_idx*16 +: 16];

    // Center-lift for s1: if unsigned value > Q/2, subtract Q
    wire signed [15:0] cur_center = (cur_lane_u > HALF_Q_U16)
        ? ($signed({1'b0, cur_lane_u}) - Q_I16)
        : $signed({1'b0, cur_lane_u});

    // Absolute value selection: s2 uses signed abs, s1 uses center-lifted abs
    wire signed [15:0] val_for_sq = check_s1_q ? cur_center : cur_lane;
    wire [15:0] abs_val = val_for_sq[15] ? (~val_for_sq + 1'b1) : val_for_sq;

    // Single multiplier (1 DSP48)
    wire [31:0] sq = {{16{1'b0}}, abs_val} * {{16{1'b0}}, abs_val};

    wire last_lane = (lane_idx == 4'd15);
    wire last_word = (idx_q == (word_count - 1'b1));

    assign start_ready = (state == ST_IDLE);

    // Memory read address
    always @(*) begin
        mem_rd_en = 1'b0;
        mem_rd_addr = (check_s1_q ? s1_base : s2_base) + idx_q;
        if (state == ST_REQ)
            mem_rd_en = 1'b1;
    end

    // Norm-check FSM: sequential lane processing (1 DSP time-multiplexed)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            idx_q      <= {ADDR_W{1'b0}};
            lane_idx   <= 4'd0;
            check_s1_q <= 1'b0;
            reject_q   <= 1'b0;
            done       <= 1'b0;
            accept     <= 1'b0;
            fail       <= 1'b0;
            status     <= 8'h00;
            norm_sq    <= 64'd0;
            word_buf   <= 256'd0;
        end else begin
            done <= 1'b0;
            fail <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (start) accept <= 1'b0;
                    status     <= 8'h00;
                    reject_q   <= 1'b0;
                    check_s1_q <= 1'b0;
                    if (start) begin
                        idx_q    <= {ADDR_W{1'b0}};
                        lane_idx <= 4'd0;
                        norm_sq  <= 64'd0;
                        if (word_count == {ADDR_W{1'b0}}) begin
                            status <= 8'hE1;
                            fail   <= 1'b1;
                            done   <= 1'b1;
                            state  <= ST_IDLE;
                        end else begin
                            state <= ST_REQ;
                        end
                    end
                end

                ST_REQ: begin
                    state <= ST_CAP;
                end

                ST_CAP: begin
                    word_buf <= mem_rd_data;
                    lane_idx <= 4'd0;
                    state    <= ST_CALC;
                end

                ST_CALC: begin
                    // Accumulate squared value for current lane
                    if (norm_sq + {{32{1'b0}}, sq} > bound_sq)
                        reject_q <= 1'b1;
                    norm_sq <= norm_sq + {{32{1'b0}}, sq};

                    if (last_lane) begin
                        if (last_word) begin
                            if (!check_s1_q) begin
                                // Finished s2, start s1
                                check_s1_q <= 1'b1;
                                idx_q      <= {ADDR_W{1'b0}};
                                lane_idx   <= 4'd0;
                                state      <= ST_REQ;
                            end else begin
                                // Finished both s2 and s1
                                state <= ST_DONE;
                            end
                        end else begin
                            // Next word
                            idx_q    <= idx_q + 1'b1;
                            lane_idx <= 4'd0;
                            state    <= ST_REQ;
                        end
                    end else begin
                        // Next lane in same word
                        lane_idx <= lane_idx + 1'b1;
                    end
                end

                ST_DONE: begin
                    done   <= 1'b1;
                    accept <= !reject_q && (norm_sq <= bound_sq);
                    status <= (!reject_q && (norm_sq <= bound_sq)) ? 8'h00 : 8'h01;
                    state  <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
