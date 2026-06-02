`timescale 1ns/1ps
// Module: falconsign_sig_decode
// Purpose: decode compressed Falcon-512 signature bytes into centered int16
// coefficients for the verify path.
//
// Decodes a Falcon-512 compressed signature from memory:
//   Header: 1 byte (0x29 = 0x20 + logn=9)
//   Nonce:  40 bytes
//   Compressed s2: variable-length bit stream of 512 signed int16 coefficients
//
// Per-coefficient encoding (big-endian bitstream order):
//   [sign:1 bit][abs_low7:7 bits][zero_run: variable 0s][terminating_1:1 bit]
//   - sign: 0=positive, 1=negative
//   - abs_low7: low 7 bits of |value|
//   - Each '0' bit adds 128 to |value|
//   - Terminating '1' bit ends this coefficient
//   - |value| range: 0..2047
//
// Memory byte layout: byte addr 0 is bits[7:0] of the first 256-bit word.
// The raw signature bytes are loaded via Port C before decode starts.

module falconsign_sig_decode #(
    parameter ADDR_W = 13
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              start,
    output wire              start_ready,
    input  wire [ADDR_W-1:0] sig_base,    // raw signature bytes in memory
    input  wire [ADDR_W-1:0] nonce_base,  // 40-byte nonce output (5 x 256-bit words)
    input  wire [ADDR_W-1:0] s2_base,     // decoded s2 output (32 x 256-bit words, 16 int16 each)

    // Port B memory
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

    localparam [3:0] ST_IDLE       = 4'd0;
    localparam [3:0] ST_RD_SIG     = 4'd1;   // issue memory read for sig word
    localparam [3:0] ST_WAIT_SIG   = 4'd2;   // wait for registered read
    localparam [3:0] ST_HEADER     = 4'd3;   // check header byte
    localparam [3:0] ST_NONCE_RD   = 4'd4;   // read nonce bytes
    localparam [3:0] ST_NONCE_WR   = 4'd5;   // write nonce word to memory
    localparam [3:0] ST_COEFF      = 4'd6;   // decode one coefficient
    localparam [3:0] ST_WR_S2      = 4'd7;   // write s2 word to memory
    localparam [3:0] ST_DONE       = 4'd8;
    localparam [3:0] ST_FAIL       = 4'd9;
    localparam [3:0] ST_LATCH_SIG  = 4'd10;  // latch registered memory read data

    localparam [7:0] FALCON_HEADER = 8'h29;  // 0x20 + logn=9

    reg [3:0]  state;
    reg [255:0] sig_word_q;    // current 256-bit signature word
    reg [5:0]   sig_byte_off;  // which byte (0..31) within sig_word_q
    reg [ADDR_W-1:0] sig_word_addr;  // next sig word address to read
    reg [5:0]   nonce_cnt;     // nonce bytes consumed (0..39)
    reg [255:0] nonce_buf;     // packed nonce bytes
    reg [4:0]   nonce_byte;    // byte index within nonce_buf (0..31)
    reg [8:0]   coeff_idx;     // coefficient index (0..511)
    reg [3:0]   lane_idx;      // lane within s2 pack word (0..15)
    reg [255:0] s2_pack_word;  // packed s2 coefficients
    reg [255:0] s2_write_word; // packed word including the current coefficient
    reg [ADDR_W-1:0] s2_wr_addr;   // next s2 write address
    reg [ADDR_W-1:0] nonce_wr_addr; // next nonce write address
    reg [255:0] nonce_write_word; // nonce word including the current byte

    // Bit accumulator
    reg [31:0]  bit_acc;
    reg [4:0]   bit_cnt;       // valid bits in bit_acc (0..32)
    reg         bit_need_rd;   // need to load next byte into accumulator

    // Coefficient decode state
    reg         coeff_sign;
    reg [10:0]  coeff_abs;
    reg [4:0]   coeff_bit_phase;  // tracks which part of coeff we're reading
    // 0=sign, 1..7=abs bits, 8=reading zero_run

    // Byte extraction
    wire [7:0] cur_byte;
    assign cur_byte = sig_word_q[(sig_byte_off * 8) +: 8];

    assign start_ready = (state == ST_IDLE);

    // ─── Combinational: next memory read address ───
    wire need_more_bytes = (state == ST_HEADER) ||
                           ((state == ST_NONCE_RD) && (nonce_cnt < 40)) ||
                           ((state == ST_COEFF) && bit_need_rd);
    wire issue_rd = (sig_byte_off == 6'd32) && need_more_bytes;

    // Decoder FSM. It skips the header/nonce, reads the compressed bitstream,
    // reconstructs signed coefficients, and writes packed int16 words.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            sig_word_q    <= 256'd0;
            sig_byte_off  <= 6'd32;  // force read on first use
            sig_word_addr <= {ADDR_W{1'b0}};
            nonce_cnt     <= 6'd0;
            nonce_buf     <= 256'd0;
            nonce_byte    <= 5'd0;
            coeff_idx     <= 9'd0;
            lane_idx      <= 4'd0;
            s2_pack_word  <= 256'd0;
            s2_write_word <= 256'd0;
            s2_wr_addr    <= {ADDR_W{1'b0}};
            nonce_wr_addr <= {ADDR_W{1'b0}};
            nonce_write_word <= 256'd0;
            bit_acc       <= 32'd0;
            bit_cnt       <= 5'd0;
            bit_need_rd   <= 1'b1;
            coeff_sign    <= 1'b0;
            coeff_abs     <= 11'd0;
            coeff_bit_phase <= 5'd0;
            mem_rd_en     <= 1'b0;
            mem_rd_addr   <= {ADDR_W{1'b0}};
            mem_wr_en     <= 1'b0;
            mem_wr_addr   <= {ADDR_W{1'b0}};
            mem_wr_data   <= 256'd0;
            done          <= 1'b0;
            fail          <= 1'b0;
            status        <= 8'h00;
        end else begin
            mem_rd_en <= 1'b0;  // default low
            mem_wr_en <= 1'b0;
            done      <= 1'b0;
            fail      <= 1'b0;

            // ─── Issue memory read when byte offset wraps ───
            case (state)
                ST_IDLE: begin
                    status <= 8'h00;
                    if (start) begin
                        sig_byte_off  <= 6'd32;  // will trigger read
                        sig_word_addr <= sig_base;
                        nonce_cnt     <= 6'd0;
                        nonce_buf     <= 256'd0;
                        nonce_byte    <= 5'd0;
                        coeff_idx     <= 9'd0;
                        lane_idx      <= 4'd0;
                        s2_pack_word  <= 256'd0;
                        s2_write_word <= 256'd0;
                        s2_wr_addr    <= s2_base;
                        nonce_wr_addr <= nonce_base;
                        nonce_write_word <= 256'd0;
                        bit_cnt       <= 5'd0;
                        bit_need_rd   <= 1'b1;
                        coeff_bit_phase <= 5'd0;
                        state         <= ST_RD_SIG;
                    end
                end

                ST_RD_SIG: begin
                    mem_rd_en   <= 1'b1;
                    mem_rd_addr <= sig_word_addr;
                    state <= ST_WAIT_SIG;
                end

                ST_WAIT_SIG: begin
                    state <= ST_LATCH_SIG;
                end

                ST_LATCH_SIG: begin
                    sig_word_q   <= mem_rd_data;
                    sig_byte_off <= 6'd0;
                    if (nonce_cnt == 6'd0 && coeff_idx == 9'd0)
                        state <= ST_HEADER;
                    else if (nonce_cnt < 6'd40)
                        state <= ST_NONCE_RD;
                    else
                        state <= ST_COEFF;
                end

                // ─── Check header byte ───
                ST_HEADER: begin
                    if (cur_byte != FALCON_HEADER) begin
                        status <= 8'hE1;
                        state  <= ST_FAIL;
                    end else begin
                        sig_byte_off <= sig_byte_off + 6'd1;
                        nonce_cnt    <= 6'd0;
                        state        <= ST_NONCE_RD;
                    end
                end

                // ─── Read nonce bytes ───
                ST_NONCE_RD: begin
                    if (sig_byte_off >= 6'd32) begin
                        sig_word_addr <= sig_word_addr + 1'b1;
                        state         <= ST_RD_SIG;
                    end else begin
                        // nonce_buf is packed as 256-bit words, byte 0 at bits[7:0]
                        nonce_write_word = nonce_buf;
                        nonce_write_word[(nonce_byte * 8) +: 8] = cur_byte;
                        nonce_buf <= nonce_write_word;
                        nonce_byte <= nonce_byte + 5'd1;
                        sig_byte_off <= sig_byte_off + 6'd1;
                        nonce_cnt    <= nonce_cnt + 6'd1;

                        if (nonce_byte == 5'd31) begin
                            mem_wr_en   <= 1'b1;
                            mem_wr_addr <= nonce_wr_addr;
                            mem_wr_data <= nonce_write_word;
                            nonce_wr_addr <= nonce_wr_addr + 1'b1;
                            nonce_byte <= 5'd0;
                            nonce_buf  <= 256'd0;
                            state      <= ST_NONCE_WR;
                        end else begin
                            if (nonce_cnt == 6'd39) begin
                                mem_wr_en   <= 1'b1;
                                mem_wr_addr <= nonce_wr_addr;
                                mem_wr_data <= nonce_write_word;
                                state        <= ST_NONCE_WR;
                            end
                        end
                    end
                end

                ST_NONCE_WR: begin
                    if (nonce_cnt == 6'd40) begin
                        // All 40 nonce bytes done; start coefficient decode
                        bit_need_rd <= 1'b1;
                        coeff_idx   <= 9'd0;
                        state       <= ST_COEFF;
                    end else begin
                        // Continue reading nonce
                        if (sig_byte_off >= 6'd32) begin
                            sig_word_addr <= sig_word_addr + 1'b1;
                            state         <= ST_RD_SIG;
                        end else begin
                            state <= ST_NONCE_RD;
                        end
                    end
                end

                // ─── Coefficient decode ───
                ST_COEFF: begin
                    // Load byte into bit accumulator when needed
                    if (bit_need_rd) begin
                        if (sig_byte_off >= 6'd32) begin
                            sig_word_addr <= sig_word_addr + 1'b1;
                            state         <= ST_RD_SIG;
                        end else begin
                            bit_acc  <= (bit_acc << 8) | {24'd0, cur_byte};
                            bit_cnt  <= bit_cnt + 5'd8;
                            sig_byte_off <= sig_byte_off + 6'd1;
                            bit_need_rd  <= 1'b0;
                        end
                    end else begin
                        case (coeff_bit_phase)
                            // Phase 0: read sign bit (1 bit)
                            5'd0: begin
                                if (bit_cnt == 5'd0) begin
                                    bit_need_rd <= 1'b1;
                                end else begin
                                    coeff_sign <= bit_acc[bit_cnt - 1];
                                    bit_cnt    <= bit_cnt - 5'd1;
                                    coeff_bit_phase <= 5'd1;
                                    coeff_abs  <= 11'd0;
                                end
                            end
                            // Phase 1..7: read abs low 7 bits (msb first)
                            5'd1, 5'd2, 5'd3, 5'd4, 5'd5, 5'd6, 5'd7: begin
                                if (bit_cnt == 5'd0) begin
                                    bit_need_rd <= 1'b1;
                                end else begin
                                    coeff_abs <= {coeff_abs[9:0], bit_acc[bit_cnt - 1]};
                                    bit_cnt   <= bit_cnt - 5'd1;
                                    coeff_bit_phase <= coeff_bit_phase + 5'd1;
                                end
                            end
                            // Phase 8+: read zero_run bits
                            default: begin
                                if (bit_cnt == 5'd0) begin
                                    bit_need_rd <= 1'b1;
                                end else begin
                                    if (bit_acc[bit_cnt - 1] == 1'b1) begin
                                        // Terminating '1' found
                                        bit_cnt   <= bit_cnt - 5'd1;
                                        // Build signed value
                                        s2_write_word = s2_pack_word;
                                        s2_write_word[(lane_idx * 16) +: 16]
                                            = coeff_sign ?
                                              (-{5'd0, coeff_abs}) :
                                              {5'd0, coeff_abs};
                                        s2_pack_word <= s2_write_word;
                                        lane_idx <= lane_idx + 4'd1;

                                        if (lane_idx == 4'd15) begin
                                            // Flush packed word
                                            mem_wr_en   <= 1'b1;
                                            mem_wr_addr <= s2_wr_addr;
                                            mem_wr_data <= s2_write_word;
                                            s2_wr_addr  <= s2_wr_addr + 1'b1;
                                            state       <= ST_WR_S2;
                                        end else if (coeff_idx == 9'd511) begin
                                            // Last coefficient, flush partial word
                                            mem_wr_en   <= 1'b1;
                                            mem_wr_addr <= s2_wr_addr;
                                            mem_wr_data <= s2_write_word;
                                            state       <= ST_WR_S2;
                                        end else begin
                                            // Next coefficient
                                            coeff_bit_phase <= 5'd0;
                                            coeff_idx  <= coeff_idx + 9'd1;
                                            bit_need_rd <= 1'b0;
                                        end
                                    end else begin
                                        // '0' bit: add 128 to abs
                                        bit_cnt   <= bit_cnt - 5'd1;
                                        coeff_abs <= coeff_abs + 11'd128;
                                        if (coeff_abs > 11'd2047) begin
                                            status <= 8'hE2;
                                            state  <= ST_FAIL;
                                        end
                                    end
                                end
                            end
                        endcase
                    end
                end

                ST_WR_S2: begin
                    if (coeff_idx == 9'd511 && lane_idx == 4'd0) begin
                        state <= ST_DONE;
                    end else begin
                        coeff_bit_phase <= 5'd0;
                        coeff_idx  <= coeff_idx + 9'd1;
                        bit_need_rd <= 1'b0;
                        state       <= ST_COEFF;
                    end
                end

                ST_DONE: begin
                    done   <= 1'b1;
                    status <= 8'h00;
                    state  <= ST_IDLE;
                end

                ST_FAIL: begin
                    done  <= 1'b1;
                    fail  <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
