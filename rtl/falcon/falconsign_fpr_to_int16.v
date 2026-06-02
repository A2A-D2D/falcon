`timescale 1ns/1ps
// Module: falconsign_fpr_to_int16
// Purpose: convert f64 signature-domain coefficients into packed signed int16
// words for the integer NTT and norm-check path.

module falconsign_fpr_to_int16 #(
    parameter ADDR_W = 11
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              start,
    output wire              start_ready,
    input  wire [ADDR_W-1:0] src_base,
    input  wire [ADDR_W-1:0] dst_base,
    input  wire [ADDR_W-1:0] coeff_count,

    output reg               mem_rd_en,
    output reg  [ADDR_W-1:0] mem_rd_addr,
    input  wire [255:0]      mem_rd_data,
    output reg               mem_wr_en,
    output reg  [ADDR_W-1:0] mem_wr_addr,
    output reg  [255:0]      mem_wr_data,

    output reg               done,
    output reg               fail,
    output reg  [7:0]        status
);

    localparam [2:0] ST_IDLE = 3'd0;
    localparam [2:0] ST_REQ  = 3'd1;
    localparam [2:0] ST_CAP  = 3'd2;
    localparam [2:0] ST_WR   = 3'd3;
    localparam [2:0] ST_DONE = 3'd4;

    reg [2:0] state;
    reg [ADDR_W-1:0] coeff_idx;
    reg [ADDR_W-1:0] word_idx;
    reg [3:0] lane_idx;
    reg [255:0] pack_word;
    reg last_coeff_q;

    reg signed [15:0] coeff_i16;
    reg [63:0] coeff_f64;
    reg [63:0] f64_neg_v;
    reg        conv_sign;
    reg [10:0] conv_exp;
    reg [51:0] conv_frac;
    reg [63:0] conv_int_part;
    reg        conv_guard;
    reg        conv_sticky;
    integer    conv_exp_unb;
    integer    conv_rsh;
    reg signed [31:0] conv_rounded;

    wire last_lane = (lane_idx == 4'd15);
    wire last_coeff = (coeff_idx == (coeff_count - 1'b1));

    assign start_ready = (state == ST_IDLE);

    // Combinational f64-to-int16 conversion with round-to-nearest behavior.
    always @(*) begin
        coeff_f64 = coeff_idx[0] ? mem_rd_data[191:128] : mem_rd_data[63:0];
        f64_neg_v = (coeff_f64[62:0] == 63'd0) ? coeff_f64
                  : {~coeff_f64[63], coeff_f64[62:0]};
        conv_sign = f64_neg_v[63];
        conv_exp  = f64_neg_v[62:52];
        conv_frac = f64_neg_v[51:0];
        conv_int_part = 64'd0;
        conv_guard = 1'b0;
        conv_sticky = 1'b0;
        conv_exp_unb = 0;
        conv_rsh = 0;

        if (conv_exp < 11'd1022) begin
            conv_rounded = 32'sd0;
        end else if (conv_exp == 11'd1022) begin
                // |f| is in [0.5, 1). Falcon uses fpr_rint(), i.e.
                // round-to-nearest with ties to even.  Exactly +/-0.5
                // rounds to 0; any larger magnitude rounds to +/-1.
            conv_rounded = (conv_frac == 52'd0) ? 32'sd0 :
                           (conv_sign ? -32'sd1 : 32'sd1);
        end else if (conv_exp >= 11'd1054) begin
            conv_rounded = conv_sign ? -32'sd32768 : 32'sd32767;
        end else begin
            conv_exp_unb = conv_exp - 11'd1023;
            if (conv_exp_unb <= 52) begin
                conv_rsh = 52 - conv_exp_unb;
                conv_int_part = (64'd1 << conv_exp_unb) + (conv_frac >> conv_rsh);
                conv_guard = (conv_rsh > 0) ? ((conv_frac >> (conv_rsh - 1)) & 1'b1) : 1'b0;
                conv_sticky = (conv_rsh > 1) ? |(conv_frac & ((64'd1 << (conv_rsh - 1)) - 1'b1)) : 1'b0;
            end else begin
                conv_int_part = (64'd1 << conv_exp_unb) | (conv_frac << (conv_exp_unb - 52));
            end

            if (conv_guard && (conv_int_part[0] || conv_sticky))
                conv_int_part = conv_int_part + 1'b1;

            if (conv_int_part > 64'd2147483647)
                conv_rounded = conv_sign ? -32'sd2147483648 : 32'sd2147483647;
            else
                conv_rounded = conv_sign ? -$signed(conv_int_part[31:0]) : $signed(conv_int_part[31:0]);
        end

        if (conv_rounded > 32'sd32767)
            coeff_i16 = 16'sh7fff;
        else if (conv_rounded < -32'sd32768)
            coeff_i16 = -16'sd32768;
        else
            coeff_i16 = conv_rounded[15:0];
    end

    // Memory write-word assembler: places the current converted lane into the
    // packed 16-lane output word.
    always @(*) begin
        mem_rd_en   = 1'b0;
        mem_rd_addr = src_base + {1'b0, coeff_idx[ADDR_W-1:1]};
        mem_wr_en   = 1'b0;
        mem_wr_addr = dst_base + word_idx;
        mem_wr_data = pack_word;

        if ((state == ST_REQ) || (state == ST_CAP))
            mem_rd_en = 1'b1;

        if (state == ST_WR) begin
            mem_wr_en = 1'b1;
            mem_wr_data = pack_word;
        end
    end

    // Conversion FSM. It reads f64 coefficients, converts one lane at a time,
    // packs 16 int16 values, and writes one 256-bit word.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            coeff_idx    <= {ADDR_W{1'b0}};
            word_idx     <= {ADDR_W{1'b0}};
            lane_idx     <= 4'd0;
            pack_word    <= 256'd0;
            last_coeff_q <= 1'b0;
            done         <= 1'b0;
            fail         <= 1'b0;
            status       <= 8'h00;
        end else begin
            done <= 1'b0;
            fail <= 1'b0;

            case (state)
                ST_IDLE: begin
                    status <= 8'h00;
                    if (start) begin
                        coeff_idx    <= {ADDR_W{1'b0}};
                        word_idx     <= {ADDR_W{1'b0}};
                        lane_idx     <= 4'd0;
                        pack_word    <= 256'd0;
                        last_coeff_q <= 1'b0;
                        if (coeff_count == {ADDR_W{1'b0}}) begin
                            fail   <= 1'b1;
                            done   <= 1'b1;
                            status <= 8'hE2;
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
                    pack_word[lane_idx*16 +: 16] <= coeff_i16;
                    last_coeff_q <= last_coeff;

                    if (last_lane || last_coeff) begin
                        state <= ST_WR;
                    end else begin
                        coeff_idx <= coeff_idx + 1'b1;
                        lane_idx  <= lane_idx + 1'b1;
                        state     <= ST_REQ;
                    end
                end

                ST_WR: begin
                    pack_word <= 256'd0;
                    if (last_coeff_q) begin
                        state <= ST_DONE;
                    end else begin
                        coeff_idx <= coeff_idx + 1'b1;
                        word_idx  <= word_idx + 1'b1;
                        lane_idx  <= 4'd0;
                        state     <= ST_REQ;
                    end
                end

                ST_DONE: begin
                    done   <= 1'b1;
                    status <= 8'h00;
                    state  <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
