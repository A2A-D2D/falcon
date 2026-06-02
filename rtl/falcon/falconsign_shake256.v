`timescale 1ns/1ps
// Module: falconsign_shake256
// Purpose: SHAKE256 absorb/squeeze wrapper around the iterative Keccak core.
// It implements Falcon's hash stream interface over 64-bit words.
//

// SHAKE256 wrapper
// Rate = 1088 bits = 17 x 64-bit words
//
// 注意：
// 当前接口没有 din_valid / din_ready，也没有 last_bytes。
// 因此默认每个输入 din 都是完整 64-bit word。
// 如果要支持任意字节长度输入，需要额外加 last_bytes 或 din_keep。

module falconsign_shake256(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        start,
    output reg         ready,

    input  wire        absorb,
    input  wire [63:0] din,
    input  wire        din_last,
    input  wire [2:0]  din_last_bytes,

    input  wire        dout_ready,
    output reg         dout_valid,
    output reg  [63:0] dout
);

    localparam ST_IDLE       = 3'd0;
    localparam ST_ABSORB     = 3'd1;
    localparam ST_SQUEEZE    = 3'd2;
    localparam ST_START_PERM = 3'd3;
    localparam ST_WAIT_BUSY  = 3'd4;
    localparam ST_WAIT_DONE  = 3'd5;
    localparam ST_PAD_EMPTY  = 3'd6;

    localparam [63:0] SHAKE_SUFFIX = 64'h0000_0000_0000_001F;
    localparam [63:0] PAD_FINAL    = 64'h8000_0000_0000_0000;

    reg [2:0] state;
    reg [2:0] after_perm;

    reg [4:0] widx;
    reg       pad_empty_pending;
    wire [63:0] partial_last_xval;

    reg [63:0] s0,s1,s2,s3,s4,s5,s6,s7,s8,s9;
    reg [63:0] s10,s11,s12,s13,s14,s15,s16,s17,s18,s19;
    reg [63:0] s20,s21,s22,s23,s24;

    reg        keccak_start;
    wire       keccak_ready;

    wire [63:0] ko0,ko1,ko2,ko3,ko4,ko5,ko6,ko7,ko8,ko9;
    wire [63:0] ko10,ko11,ko12,ko13,ko14,ko15,ko16,ko17,ko18,ko19;
    wire [63:0] ko20,ko21,ko22,ko23,ko24;

    assign partial_last_xval =
        (din_last_bytes == 3'd1) ? ((din & 64'h0000_0000_0000_00FF) ^ 64'h0000_0000_0000_1F00) :
        (din_last_bytes == 3'd2) ? ((din & 64'h0000_0000_0000_FFFF) ^ 64'h0000_0000_001F_0000) :
        (din_last_bytes == 3'd3) ? ((din & 64'h0000_0000_00FF_FFFF) ^ 64'h0000_0000_1F00_0000) :
        (din_last_bytes == 3'd4) ? ((din & 64'h0000_0000_FFFF_FFFF) ^ 64'h0000_001F_0000_0000) :
        (din_last_bytes == 3'd5) ? ((din & 64'h0000_00FF_FFFF_FFFF) ^ 64'h0000_1F00_0000_0000) :
        (din_last_bytes == 3'd6) ? ((din & 64'h0000_FFFF_FFFF_FFFF) ^ 64'h001F_0000_0000_0000) :
        (din_last_bytes == 3'd7) ? ((din & 64'h00FF_FFFF_FFFF_FFFF) ^ 64'h1F00_0000_0000_0000) :
                                    din;

    falconsign_keccak_core u_keccak(
        .clk(clk),
        .rst_n(rst_n),
        .start(keccak_start),
        .ready(keccak_ready),

        .di0(s0),   .di1(s1),   .di2(s2),   .di3(s3),   .di4(s4),
        .di5(s5),   .di6(s6),   .di7(s7),   .di8(s8),   .di9(s9),
        .di10(s10), .di11(s11), .di12(s12), .di13(s13), .di14(s14),
        .di15(s15), .di16(s16), .di17(s17), .di18(s18), .di19(s19),
        .di20(s20), .di21(s21), .di22(s22), .di23(s23), .di24(s24),

        .do0(ko0),   .do1(ko1),   .do2(ko2),   .do3(ko3),   .do4(ko4),
        .do5(ko5),   .do6(ko6),   .do7(ko7),   .do8(ko8),   .do9(ko9),
        .do10(ko10), .do11(ko11), .do12(ko12), .do13(ko13), .do14(ko14),
        .do15(ko15), .do16(ko16), .do17(ko17), .do18(ko18), .do19(ko19),
        .do20(ko20), .do21(ko21), .do22(ko22), .do23(ko23), .do24(ko24)
    );

    // Ready is high only when the wrapper can accept absorb/squeeze activity;
    // permutation states stall the caller until Keccak is done.
    always @(*) begin
        case (state)
            ST_IDLE,
            ST_ABSORB,
            ST_SQUEEZE: ready = 1'b1;
            default:    ready = 1'b0;
        endcase
    end

    // SHAKE256 control FSM. It absorbs 64-bit words with Falcon padding,
    // launches Keccak permutations, and streams rate words during squeeze.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= ST_IDLE;
            after_perm        <= ST_IDLE;
            widx              <= 5'd0;
            keccak_start      <= 1'b0;
            dout_valid        <= 1'b0;
            dout              <= 64'd0;
            pad_empty_pending <= 1'b0;
            s0  <= 64'd0; s1  <= 64'd0; s2  <= 64'd0; s3  <= 64'd0; s4  <= 64'd0;
            s5  <= 64'd0; s6  <= 64'd0; s7  <= 64'd0; s8  <= 64'd0; s9  <= 64'd0;
            s10 <= 64'd0; s11 <= 64'd0; s12 <= 64'd0; s13 <= 64'd0; s14 <= 64'd0;
            s15 <= 64'd0; s16 <= 64'd0; s17 <= 64'd0; s18 <= 64'd0; s19 <= 64'd0;
            s20 <= 64'd0; s21 <= 64'd0; s22 <= 64'd0; s23 <= 64'd0; s24 <= 64'd0;
        end else begin
            keccak_start <= 1'b0;
            if (!(dout_valid && !dout_ready))
                dout_valid <= 1'b0;

            if (start) begin
                state             <= ST_ABSORB;
                after_perm        <= ST_ABSORB;
                widx              <= 5'd0;
                pad_empty_pending <= 1'b0;
                dout_valid        <= 1'b0;
                dout              <= 64'd0;
                s0  <= 64'd0; s1  <= 64'd0; s2  <= 64'd0; s3  <= 64'd0; s4  <= 64'd0;
                s5  <= 64'd0; s6  <= 64'd0; s7  <= 64'd0; s8  <= 64'd0; s9  <= 64'd0;
                s10 <= 64'd0; s11 <= 64'd0; s12 <= 64'd0; s13 <= 64'd0; s14 <= 64'd0;
                s15 <= 64'd0; s16 <= 64'd0; s17 <= 64'd0; s18 <= 64'd0; s19 <= 64'd0;
                s20 <= 64'd0; s21 <= 64'd0; s22 <= 64'd0; s23 <= 64'd0; s24 <= 64'd0;
            end else begin
                case (state)

                    ST_IDLE: begin
                        widx <= 5'd0;
                    end

                    ST_ABSORB: begin
                        if (absorb) begin
                            if (din_last) begin
                                widx       <= 5'd0;
                                after_perm <= ST_SQUEEZE;
                                state      <= ST_START_PERM;

                                if (din_last_bytes != 3'd0) begin
                                    pad_empty_pending <= 1'b0;
                                    case (widx)
                                        5'd0: begin s0 <= s0 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd1: begin s1 <= s1 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd2: begin s2 <= s2 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd3: begin s3 <= s3 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd4: begin s4 <= s4 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd5: begin s5 <= s5 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd6: begin s6 <= s6 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd7: begin s7 <= s7 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd8: begin s8 <= s8 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd9: begin s9 <= s9 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd10: begin s10 <= s10 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd11: begin s11 <= s11 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd12: begin s12 <= s12 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd13: begin s13 <= s13 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd14: begin s14 <= s14 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd15: begin s15 <= s15 ^ partial_last_xval; s16 <= s16 ^ PAD_FINAL; end
                                        5'd16: s16 <= s16 ^ partial_last_xval ^ PAD_FINAL;
                                        default: ;
                                    endcase
                                end else if (widx == 5'd16) begin
                                    // 当前 word 正好填满一个 rate block。
                                    // 先 permute 当前 block，
                                    // 然后再吸收一个 padding-only block。
                                    pad_empty_pending <= 1'b1;
                                    s16 <= s16 ^ din;
                                end else begin
                                    pad_empty_pending <= 1'b0;
                                    case (widx)
                                        5'd0:  begin s0  <= s0  ^ din; s1  <= s1  ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd1:  begin s1  <= s1  ^ din; s2  <= s2  ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd2:  begin s2  <= s2  ^ din; s3  <= s3  ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd3:  begin s3  <= s3  ^ din; s4  <= s4  ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd4:  begin s4  <= s4  ^ din; s5  <= s5  ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd5:  begin s5  <= s5  ^ din; s6  <= s6  ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd6:  begin s6  <= s6  ^ din; s7  <= s7  ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd7:  begin s7  <= s7  ^ din; s8  <= s8  ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd8:  begin s8  <= s8  ^ din; s9  <= s9  ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd9:  begin s9  <= s9  ^ din; s10 <= s10 ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd10: begin s10 <= s10 ^ din; s11 <= s11 ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd11: begin s11 <= s11 ^ din; s12 <= s12 ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd12: begin s12 <= s12 ^ din; s13 <= s13 ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd13: begin s13 <= s13 ^ din; s14 <= s14 ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd14: begin s14 <= s14 ^ din; s15 <= s15 ^ SHAKE_SUFFIX; s16 <= s16 ^ PAD_FINAL; end
                                        5'd15: begin s15 <= s15 ^ din; s16 <= s16 ^ SHAKE_SUFFIX ^ PAD_FINAL; end
                                        default: ;
                                    endcase
                                end
                            end else if (widx == 5'd16) begin
                                s16 <= s16 ^ din;
                                widx              <= 5'd0;
                                after_perm        <= ST_ABSORB;
                                pad_empty_pending <= 1'b0;
                                state             <= ST_START_PERM;
                            end else begin
                                case (widx)
                                    5'd0:  s0  <= s0  ^ din;
                                    5'd1:  s1  <= s1  ^ din;
                                    5'd2:  s2  <= s2  ^ din;
                                    5'd3:  s3  <= s3  ^ din;
                                    5'd4:  s4  <= s4  ^ din;
                                    5'd5:  s5  <= s5  ^ din;
                                    5'd6:  s6  <= s6  ^ din;
                                    5'd7:  s7  <= s7  ^ din;
                                    5'd8:  s8  <= s8  ^ din;
                                    5'd9:  s9  <= s9  ^ din;
                                    5'd10: s10 <= s10 ^ din;
                                    5'd11: s11 <= s11 ^ din;
                                    5'd12: s12 <= s12 ^ din;
                                    5'd13: s13 <= s13 ^ din;
                                    5'd14: s14 <= s14 ^ din;
                                    5'd15: s15 <= s15 ^ din;
                                    default: ;
                                endcase
                                widx <= widx + 5'd1;
                            end
                        end
                    end

                    ST_PAD_EMPTY: begin
                        // 用于消息长度正好是 rate 整数倍的情况。
                        s0  <= s0  ^ SHAKE_SUFFIX;
                        s16 <= s16 ^ PAD_FINAL;

                        after_perm <= ST_SQUEEZE;
                        state      <= ST_START_PERM;
                    end

                    ST_SQUEEZE: begin
                        if (!absorb && dout_ready) begin
                            case (widx)
                                5'd0:  dout <= s0;
                                5'd1:  dout <= s1;
                                5'd2:  dout <= s2;
                                5'd3:  dout <= s3;
                                5'd4:  dout <= s4;
                                5'd5:  dout <= s5;
                                5'd6:  dout <= s6;
                                5'd7:  dout <= s7;
                                5'd8:  dout <= s8;
                                5'd9:  dout <= s9;
                                5'd10: dout <= s10;
                                5'd11: dout <= s11;
                                5'd12: dout <= s12;
                                5'd13: dout <= s13;
                                5'd14: dout <= s14;
                                5'd15: dout <= s15;
                                5'd16: dout <= s16;
                                default: dout <= 64'd0;
                            endcase

                            dout_valid <= 1'b1;

                            if (widx == 5'd16) begin
                                widx       <= 5'd0;
                                after_perm <= ST_SQUEEZE;
                                state      <= ST_START_PERM;
                            end else begin
                                widx <= widx + 5'd1;
                            end
                        end
                    end

                    ST_START_PERM: begin
                        // 单拍启动 Keccak core。
                        // 这里状态已经在前一拍更新完，避免 core 采到旧 s0~s24。
                        keccak_start <= 1'b1;
                        state        <= ST_WAIT_BUSY;
                    end

                    ST_WAIT_BUSY: begin
                        // 等待 core 从 ready=1 进入 busy。
                        if (!keccak_ready) begin
                            state <= ST_WAIT_DONE;
                        end
                    end

                    ST_WAIT_DONE: begin
                        if (keccak_ready) begin
                            s0  <= ko0;  s1  <= ko1;  s2  <= ko2;  s3  <= ko3;  s4  <= ko4;
                            s5  <= ko5;  s6  <= ko6;  s7  <= ko7;  s8  <= ko8;  s9  <= ko9;
                            s10 <= ko10; s11 <= ko11; s12 <= ko12; s13 <= ko13; s14 <= ko14;
                            s15 <= ko15; s16 <= ko16; s17 <= ko17; s18 <= ko18; s19 <= ko19;
                            s20 <= ko20; s21 <= ko21; s22 <= ko22; s23 <= ko23; s24 <= ko24;

                            if (pad_empty_pending) begin
                                pad_empty_pending <= 1'b0;
                                state             <= ST_PAD_EMPTY;
                            end else begin
                                state <= after_perm;
                            end
                        end
                    end

                    default: begin
                        state <= ST_IDLE;
                    end

                endcase
            end
        end
    end

endmodule
