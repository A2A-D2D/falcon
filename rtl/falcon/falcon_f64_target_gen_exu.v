`timescale 1ns/1ps

// Module: falcon_f64_target_gen_exu
// Purpose: build the two ffSampling target polynomials from FFT(c).
//
// Falcon sign_tree does not consume FFT(c) directly.  It needs:
//   t0 = FFT(c) * b11 * ( 1/q)
//   t1 = FFT(c) * b01 * (-1/q)
// This EXU performs those two complex products with the shared f64 FPU and
// writes t0/t1 into the normal signing target window.
module falcon_f64_target_gen_exu #(
    parameter ADDR_W = 13
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              start,
    output wire              start_ready,
    input  wire [ADDR_W-1:0] c_fft_base,
    input  wire [ADDR_W-1:0] t0_base,
    input  wire [ADDR_W-1:0] t1_base,
    input  wire [ADDR_W-1:0] b01_base,
    input  wire [ADDR_W-1:0] b11_base,
    input  wire [ADDR_W-1:0] word_count,

    output reg               mem_rd_en,
    output reg  [ADDR_W-1:0] mem_rd_addr,
    input  wire [255:0]      mem_rd_data,
    output reg               mem_wr_en,
    output reg  [ADDR_W-1:0] mem_wr_addr,
    output reg  [255:0]      mem_wr_data,

    output reg               fpu_req_valid,
    input  wire              fpu_req_ready,
    output reg  [3:0]        fpu_req_op,
    output reg  [63:0]       fpu_req_a,
    output reg  [63:0]       fpu_req_b,
    output reg  [63:0]       fpu_req_c,
    input  wire              fpu_rsp_valid,
    input  wire [63:0]       fpu_rsp_result,

    output reg               done,
    output reg               fail,
    output reg  [7:0]        status
);

    localparam [4:0] ST_IDLE    = 5'd0;
    localparam [4:0] ST_RD_C    = 5'd1;
    localparam [4:0] ST_RD_B01  = 5'd2;
    localparam [4:0] ST_RD_B11  = 5'd3;
    localparam [4:0] ST_WAIT1   = 5'd4;
    localparam [4:0] ST_WAIT2   = 5'd5;
    localparam [4:0] ST_CAP_C   = 5'd6;
    localparam [4:0] ST_CAP_B01 = 5'd7;
    localparam [4:0] ST_CAP_B11 = 5'd8;
    localparam [4:0] ST_FPU_REQ = 5'd9;
    localparam [4:0] ST_FPU_WAIT= 5'd10;
    localparam [4:0] ST_WR_T0   = 5'd11;
    localparam [4:0] ST_WR_T1   = 5'd12;
    localparam [4:0] ST_DONE    = 5'd13;
    localparam [4:0] ST_FAIL    = 5'd14;

    localparam [3:0] FMUL   = 4'd2;
    localparam [3:0] FMADD  = 4'd3;
    localparam [3:0] FNMADD = 4'd6;

    localparam [4:0] PH_T0_RE_A = 5'd0;
    localparam [4:0] PH_T0_RE_B = 5'd1;
    localparam [4:0] PH_T0_IM_A = 5'd2;
    localparam [4:0] PH_T0_IM_B = 5'd3;
    localparam [4:0] PH_T0_SCALE_RE = 5'd4;
    localparam [4:0] PH_T0_SCALE_IM = 5'd5;
    localparam [4:0] PH_T1_RE_A = 5'd6;
    localparam [4:0] PH_T1_RE_B = 5'd7;
    localparam [4:0] PH_T1_IM_A = 5'd8;
    localparam [4:0] PH_T1_IM_B = 5'd9;
    localparam [4:0] PH_T1_SCALE_RE = 5'd10;
    localparam [4:0] PH_T1_SCALE_IM = 5'd11;

    localparam [63:0] FPR_INV_Q     = 64'h3f1554e39097a782;
    localparam [63:0] FPR_NEG_INV_Q = 64'hbf1554e39097a782;

    reg [4:0] state;
    reg [4:0] read_return_state;
    reg [4:0] phase_q;
    reg [ADDR_W-1:0] idx_q;

    reg [63:0] c_re_q, c_im_q;
    reg [63:0] b01_re_q, b01_im_q;
    reg [63:0] b11_re_q, b11_im_q;
    reg [63:0] tmp_q;
    reg [63:0] t0_re_q, t0_im_q;
    reg [63:0] t1_re_q, t1_im_q;

    assign start_ready = (state == ST_IDLE);

    always @(*) begin
        mem_rd_en     = 1'b0;
        mem_rd_addr   = c_fft_base + idx_q;
        mem_wr_en     = 1'b0;
        mem_wr_addr   = t0_base + idx_q;
        mem_wr_data   = {128'd0, t0_im_q, t0_re_q};
        fpu_req_valid = 1'b0;
        fpu_req_op    = FMUL;
        fpu_req_a     = 64'd0;
        fpu_req_b     = 64'd0;
        fpu_req_c     = 64'd0;

        case (state)
            ST_RD_C: begin
                mem_rd_en   = 1'b1;
                mem_rd_addr = c_fft_base + idx_q[ADDR_W-1:1];
            end
            ST_RD_B01: begin
                mem_rd_en   = 1'b1;
                mem_rd_addr = b01_base + idx_q;
            end
            ST_RD_B11: begin
                mem_rd_en   = 1'b1;
                mem_rd_addr = b11_base + idx_q;
            end
            ST_FPU_REQ: begin
                fpu_req_valid = 1'b1;
                case (phase_q)
                    PH_T0_RE_A: begin fpu_req_op = FMUL;   fpu_req_a = c_re_q; fpu_req_b = b11_re_q; end
                    PH_T0_RE_B: begin fpu_req_op = FNMADD; fpu_req_a = c_im_q; fpu_req_b = b11_im_q; fpu_req_c = tmp_q; end
                    PH_T0_IM_A: begin fpu_req_op = FMUL;   fpu_req_a = c_re_q; fpu_req_b = b11_im_q; end
                    PH_T0_IM_B: begin fpu_req_op = FMADD;  fpu_req_a = c_im_q; fpu_req_b = b11_re_q; fpu_req_c = tmp_q; end
                    PH_T0_SCALE_RE: begin fpu_req_op = FMUL; fpu_req_a = t0_re_q; fpu_req_b = FPR_INV_Q; end
                    PH_T0_SCALE_IM: begin fpu_req_op = FMUL; fpu_req_a = t0_im_q; fpu_req_b = FPR_INV_Q; end
                    PH_T1_RE_A: begin fpu_req_op = FMUL;   fpu_req_a = c_re_q; fpu_req_b = b01_re_q; end
                    PH_T1_RE_B: begin fpu_req_op = FNMADD; fpu_req_a = c_im_q; fpu_req_b = b01_im_q; fpu_req_c = tmp_q; end
                    PH_T1_IM_A: begin fpu_req_op = FMUL;   fpu_req_a = c_re_q; fpu_req_b = b01_im_q; end
                    PH_T1_IM_B: begin fpu_req_op = FMADD;  fpu_req_a = c_im_q; fpu_req_b = b01_re_q; fpu_req_c = tmp_q; end
                    PH_T1_SCALE_RE: begin fpu_req_op = FMUL; fpu_req_a = t1_re_q; fpu_req_b = FPR_NEG_INV_Q; end
                    PH_T1_SCALE_IM: begin fpu_req_op = FMUL; fpu_req_a = t1_im_q; fpu_req_b = FPR_NEG_INV_Q; end
                    default: begin fpu_req_op = FMUL; end
                endcase
            end
            ST_WR_T0: begin
                mem_wr_en   = 1'b1;
                mem_wr_addr = t0_base + idx_q;
                mem_wr_data = {128'd0, t0_im_q, t0_re_q};
            end
            ST_WR_T1: begin
                mem_wr_en   = 1'b1;
                mem_wr_addr = t1_base + idx_q;
                mem_wr_data = {128'd0, t1_im_q, t1_re_q};
            end
            default: begin
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            read_return_state <= ST_IDLE;
            phase_q <= PH_T0_RE_A;
            idx_q <= {ADDR_W{1'b0}};
            c_re_q <= 64'd0; c_im_q <= 64'd0;
            b01_re_q <= 64'd0; b01_im_q <= 64'd0;
            b11_re_q <= 64'd0; b11_im_q <= 64'd0;
            tmp_q <= 64'd0;
            t0_re_q <= 64'd0; t0_im_q <= 64'd0;
            t1_re_q <= 64'd0; t1_im_q <= 64'd0;
            done <= 1'b0;
            fail <= 1'b0;
            status <= 8'h00;
        end else begin
            done <= 1'b0;
            fail <= 1'b0;

            case (state)
                ST_IDLE: begin
                    status <= 8'h00;
                    if (start) begin
                        if (word_count == {ADDR_W{1'b0}}) begin
                            status <= 8'hE1;
                            state  <= ST_FAIL;
                        end else begin
                            idx_q <= word_count - 1'b1;
                            state <= ST_RD_C;
                        end
                    end
                end

                ST_RD_C:   begin read_return_state <= ST_CAP_C;   state <= ST_WAIT1; end
                ST_RD_B01: begin read_return_state <= ST_CAP_B01; state <= ST_WAIT1; end
                ST_RD_B11: begin read_return_state <= ST_CAP_B11; state <= ST_WAIT1; end
                ST_WAIT1:  begin state <= ST_WAIT2; end
                ST_WAIT2:  begin state <= read_return_state; end

                ST_CAP_C: begin
                    if (idx_q[0]) begin
                        c_re_q <= mem_rd_data[191:128];
                        c_im_q <= mem_rd_data[255:192];
                    end else begin
                        c_re_q <= mem_rd_data[63:0];
                        c_im_q <= mem_rd_data[127:64];
                    end
                    state  <= ST_RD_B01;
                end
                ST_CAP_B01: begin
                    b01_re_q <= mem_rd_data[63:0];
                    b01_im_q <= mem_rd_data[127:64];
                    state    <= ST_RD_B11;
                end
                ST_CAP_B11: begin
                    b11_re_q <= mem_rd_data[63:0];
                    b11_im_q <= mem_rd_data[127:64];
                    phase_q  <= PH_T0_RE_A;
                    state    <= ST_FPU_REQ;
                end

                ST_FPU_REQ: begin
                    if (fpu_req_ready)
                        state <= ST_FPU_WAIT;
                end
                ST_FPU_WAIT: begin
                    if (fpu_rsp_valid) begin
                        case (phase_q)
                            PH_T0_RE_A: begin tmp_q <= fpu_rsp_result; phase_q <= PH_T0_RE_B; state <= ST_FPU_REQ; end
                            PH_T0_RE_B: begin t0_re_q <= fpu_rsp_result; phase_q <= PH_T0_IM_A; state <= ST_FPU_REQ; end
                            PH_T0_IM_A: begin tmp_q <= fpu_rsp_result; phase_q <= PH_T0_IM_B; state <= ST_FPU_REQ; end
                            PH_T0_IM_B: begin t0_im_q <= fpu_rsp_result; phase_q <= PH_T0_SCALE_RE; state <= ST_FPU_REQ; end
                            PH_T0_SCALE_RE: begin t0_re_q <= fpu_rsp_result; phase_q <= PH_T0_SCALE_IM; state <= ST_FPU_REQ; end
                            PH_T0_SCALE_IM: begin t0_im_q <= fpu_rsp_result; phase_q <= PH_T1_RE_A; state <= ST_FPU_REQ; end
                            PH_T1_RE_A: begin tmp_q <= fpu_rsp_result; phase_q <= PH_T1_RE_B; state <= ST_FPU_REQ; end
                            PH_T1_RE_B: begin t1_re_q <= fpu_rsp_result; phase_q <= PH_T1_IM_A; state <= ST_FPU_REQ; end
                            PH_T1_IM_A: begin tmp_q <= fpu_rsp_result; phase_q <= PH_T1_IM_B; state <= ST_FPU_REQ; end
                            PH_T1_IM_B: begin t1_im_q <= fpu_rsp_result; phase_q <= PH_T1_SCALE_RE; state <= ST_FPU_REQ; end
                            PH_T1_SCALE_RE: begin t1_re_q <= fpu_rsp_result; phase_q <= PH_T1_SCALE_IM; state <= ST_FPU_REQ; end
                            PH_T1_SCALE_IM: begin t1_im_q <= fpu_rsp_result; state <= ST_WR_T0; end
                            default: begin status <= 8'hE2; state <= ST_FAIL; end
                        endcase
                    end
                end

                ST_WR_T0: begin
                    state <= ST_WR_T1;
                end
                ST_WR_T1: begin
                    if (idx_q == {ADDR_W{1'b0}}) begin
                        state <= ST_DONE;
                    end else begin
                        idx_q <= idx_q - 1'b1;
                        state <= ST_RD_C;
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
                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
