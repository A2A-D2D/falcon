`timescale 1ns/1ps
// Module: falcon_f64_bhat_mul_exu
// Purpose: Falcon Bhat multiplication s2 = z0*b01 + z1*b11.
// Uses two falcon_f64_complex_bfly instances (each internal FPU)
// running in parallel for complex multiply. Then 2 FPU FADDs for sum.
//
// Throughput: ~57 cy/coeff (vs ~634 before). N=512: ~29K (vs ~325K, ~11x).
//
// BFU: a=0, b=data, w=basis → y0 = b*w = complex multiply
//   bf0: b=z0, w=b01 → m0 = z0 * b01
//   bf1: b=z1, w=b11 → m1 = z1 * b11
//   out = m0 + m1 (via shared FPU)
module falcon_f64_bhat_mul_exu #(
    parameter ADDR_W = 11
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              start,
    output wire              start_ready,
    input  wire              identity_mode,
    input  wire [ADDR_W-1:0] t_base,
    input  wire [ADDR_W-1:0] z_base,
    input  wire [ADDR_W-1:0] b00_base,
    input  wire [ADDR_W-1:0] b01_base,
    input  wire [ADDR_W-1:0] b10_base,
    input  wire [ADDR_W-1:0] b11_base,
    input  wire [ADDR_W-1:0] s2_fft_base,
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
    output reg  [7:0]        status,

    // External shared BFU ports (to top-level BFU pool)
    output wire              vd_bfu0_in_v,
    input  wire              vd_bfu0_in_r,
    output wire [63:0]       vd_bfu0_b_re, vd_bfu0_b_im,
    output wire [63:0]       vd_bfu0_w_re, vd_bfu0_w_im,
    input  wire              vd_bfu0_out_v,
    input  wire [63:0]       vd_bfu0_y0r, vd_bfu0_y0i,
    output wire              vd_bfu1_in_v,
    input  wire              vd_bfu1_in_r,
    output wire [63:0]       vd_bfu1_b_re, vd_bfu1_b_im,
    output wire [63:0]       vd_bfu1_w_re, vd_bfu1_w_im,
    input  wire              vd_bfu1_out_v,
    input  wire [63:0]       vd_bfu1_y0r, vd_bfu1_y0i
);

    localparam [3:0] FADD = 4'd0;

    // ─── FSM states ───
    localparam [4:0] ST_IDLE     = 5'd0;
    localparam [4:0] ST_RD       = 5'd1;
    localparam [4:0] ST_WAIT1    = 5'd2;
    localparam [4:0] ST_WAIT2    = 5'd3;
    localparam [4:0] ST_CAP      = 5'd4;
    localparam [4:0] ST_BFU_RUN  = 5'd5;
    localparam [4:0] ST_FPU_RE   = 5'd6;
    localparam [4:0] ST_FPU_WAIT = 5'd7;
    localparam [4:0] ST_WR       = 5'd8;
    localparam [4:0] ST_DONE_S   = 5'd9;
    localparam [4:0] ST_FAIL_S   = 5'd10;
    localparam [4:0] ST_BFU_WAIT = 5'd11;  // wait for bf_both_done

    // ─── Read sub-phases ───
    localparam [2:0] RD_Z0  = 3'd0;
    localparam [2:0] RD_Z1  = 3'd1;
    localparam [2:0] RD_B01 = 3'd2;
    localparam [2:0] RD_B11 = 3'd3;

    // ─── FPU sub-phases ───
    localparam [1:0] FP_RE = 2'd0;
    localparam [1:0] FP_IM = 2'd1;

    reg [4:0]  state;
    reg [2:0]  rd_phase;
    reg [1:0]  fp_phase;
    reg [ADDR_W-1:0] idx;

    // Input registers
    reg [63:0] z0_re, z0_im, z1_re, z1_im;
    reg [63:0] b01_re, b01_im, b11_re, b11_im;
    reg [63:0] m0_re, m0_im, m1_re, m1_im;
    reg [63:0] out_re, out_im;
    reg [63:0] pair_re_q, pair_im_q;

    wire [ADDR_W-1:0] z1_base_w = z_base + word_count;
    wire [ADDR_W-1:0] coeff_count = {1'b0, word_count[ADDR_W-1:1]};

    // ─── External shared BFU interfaces (2 lanes, instantiated at top level) ───
    // BFU 0: computes m0 = z0 * b01  (a=0, b=z0, w=b01)
    // BFU 1: computes m1 = z1 * b11  (a=0, b=z1, w=b11)
    // Ports declared in module port list above. Internal handshake wires:
    wire bf0_in_v, bf0_in_r, bf0_out_v;
    wire [63:0] bf0_y0r, bf0_y0i, bf0_y1r, bf0_y1i;
    wire bf1_in_v, bf1_in_r, bf1_out_v;
    wire [63:0] bf1_y0r, bf1_y0i, bf1_y1r, bf1_y1i;

    // Connect internal signals to external shared BFU ports
    assign vd_bfu0_in_v  = bf0_in_v;
    assign bf0_in_r      = vd_bfu0_in_r;
    assign vd_bfu0_b_re  = z0_re;   assign vd_bfu0_b_im  = z0_im;
    assign vd_bfu0_w_re  = b01_re;  assign vd_bfu0_w_im  = b01_im;
    assign bf0_out_v     = vd_bfu0_out_v;
    assign bf0_y0r       = vd_bfu0_y0r; assign bf0_y0i = vd_bfu0_y0i;

    assign vd_bfu1_in_v  = bf1_in_v;
    assign bf1_in_r      = vd_bfu1_in_r;
    assign vd_bfu1_b_re  = z1_re;   assign vd_bfu1_b_im  = z1_im;
    assign vd_bfu1_w_re  = b11_re;  assign vd_bfu1_w_im  = b11_im;
    assign bf1_out_v     = vd_bfu1_out_v;
    assign bf1_y0r       = vd_bfu1_y0r; assign bf1_y0i = vd_bfu1_y0i;

    // BFU launch: fire both in parallel, wait for both to finish
    reg bf_launch;
    reg bf0_done, bf1_done;
    wire bf_both_done = bf0_done && bf1_done;

    assign bf0_in_v = bf_launch && (state == ST_BFU_RUN);
    assign bf1_in_v = bf_launch && (state == ST_BFU_RUN);

    // ─── Combo decoder ───
    always @(*) begin
        mem_rd_en = 1'b0; mem_rd_addr = {ADDR_W{1'b0}};
        mem_wr_en = 1'b0; mem_wr_addr = s2_fft_base + {1'b0, idx[ADDR_W-1:1]};
        mem_wr_data = {out_im, out_re, pair_im_q, pair_re_q};
        fpu_req_valid = 1'b0; fpu_req_op = FADD;
        fpu_req_a = 64'd0; fpu_req_b = 64'd0; fpu_req_c = 64'd0;

        case (state)
            ST_RD: begin
                mem_rd_en = 1'b1;
                case (rd_phase)
                    RD_Z0:  mem_rd_addr = z_base + idx;
                    RD_Z1:  mem_rd_addr = z1_base_w + idx;
                    RD_B01: mem_rd_addr = b01_base + idx;
                    RD_B11: mem_rd_addr = b11_base + idx;
                    default: mem_rd_addr = z_base + idx;
                endcase
            end
            ST_FPU_RE: begin
                fpu_req_valid = 1'b1;
                fpu_req_op = FADD;
                if (fp_phase == FP_RE) begin
                    fpu_req_a = m0_re; fpu_req_b = m1_re;
                end else begin
                    fpu_req_a = m0_im; fpu_req_b = m1_im;
                end
            end
            ST_WR: begin
                mem_wr_en = idx[0];
                mem_wr_addr = s2_fft_base + {1'b0, idx[ADDR_W-1:1]};
                mem_wr_data = {out_im, out_re, pair_im_q, pair_re_q};
            end
            default: ;
        endcase
    end

    assign start_ready = (state == ST_IDLE);

    // ─── Main FSM ───
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            rd_phase <= RD_Z0; fp_phase <= FP_RE; idx <= 0;
            {z0_re,z0_im,z1_re,z1_im} <= 256'd0;
            {b01_re,b01_im,b11_re,b11_im} <= 256'd0;
            {m0_re,m0_im,m1_re,m1_im} <= 256'd0;
            {out_re,out_im} <= 128'd0;
            {pair_re_q,pair_im_q} <= 128'd0;
            bf_launch <= 0; bf0_done <= 0; bf1_done <= 0;
            done <= 0; fail <= 0; status <= 0;
        end else begin
            done <= 0; fail <= 0;

            // BFU completion tracking
            if (bf0_out_v) begin bf0_done <= 1; bf_launch <= 0; end
            if (bf1_out_v) begin bf1_done <= 1; end

            case (state)
                ST_IDLE: begin
                    status <= 0; idx <= 0;
                    if (start) begin
                        if (word_count == {ADDR_W{1'b0}}) begin
                            status <= 8'hE1; state <= ST_FAIL_S;
                        end else begin
                            pair_re_q <= 64'd0;
                            pair_im_q <= 64'd0;
                            rd_phase <= RD_Z0; state <= ST_RD;
                        end
                    end
                end

                // ─── Memory read pipeline (RD → WAIT1 → WAIT2 → CAP) ───
                ST_RD:    state <= ST_WAIT1;
                ST_WAIT1: state <= ST_WAIT2;
                ST_WAIT2: state <= ST_CAP;

                ST_CAP: begin
                    case (rd_phase)
                        RD_Z0: begin
                            z0_re <= mem_rd_data[63:0];
                            z0_im <= mem_rd_data[127:64];
                            rd_phase <= RD_Z1; state <= ST_RD;
                        end
                        RD_Z1: begin
                            z1_re <= mem_rd_data[63:0];
                            z1_im <= mem_rd_data[127:64];
                            rd_phase <= RD_B01; state <= ST_RD;
                        end
                        RD_B01: begin
                            b01_re <= mem_rd_data[63:0];
                            b01_im <= mem_rd_data[127:64];
                            rd_phase <= RD_B11; state <= ST_RD;
                        end
                        RD_B11: begin
                            b11_re <= mem_rd_data[63:0];
                            b11_im <= mem_rd_data[127:64];
                            rd_phase <= RD_Z0;
                            // All reads done → launch both BFUs
                            bf_launch <= 1; bf0_done <= 0; bf1_done <= 0;
                            state <= ST_BFU_RUN;
                        end
                        default: begin
                            rd_phase <= RD_Z0; state <= ST_RD;
                        end
                    endcase
                end

                // ─── BFU computation (both in parallel) ───
                ST_BFU_RUN: begin
                    // Wait 1 cycle for BFU to accept input, then go to wait state
                    state <= ST_BFU_WAIT;
                end

                ST_BFU_WAIT: begin
                    if (bf_both_done) begin
                        m0_re <= bf0_y0r; m0_im <= bf0_y0i;
                        m1_re <= bf1_y0r; m1_im <= bf1_y0i;
                        bf0_done <= 0; bf1_done <= 0;
                        fp_phase <= FP_RE;
                        state <= ST_FPU_RE;
                    end
                end

                // ─── FPU FADD: out_re = m0_re + m1_re, out_im = m0_im + m1_im ───
                ST_FPU_RE: begin
                    if (fpu_req_ready)
                        state <= ST_FPU_WAIT;
                end

                ST_FPU_WAIT: begin
                    if (fpu_rsp_valid) begin
                        case (fp_phase)
                            FP_RE: begin
                                out_re  <= fpu_rsp_result;
                                fp_phase <= FP_IM;
                                state <= ST_FPU_RE;
                            end
                            FP_IM: begin
                                out_im  <= fpu_rsp_result;
                                state <= ST_WR;
                            end
                            default: begin
                                out_re <= fpu_rsp_result;
                                fp_phase <= FP_IM;
                                state <= ST_FPU_RE;
                            end
                        endcase
                    end
                end

                // ─── Writeback ───
                ST_WR: begin
                    if (!idx[0]) begin
                        pair_re_q <= out_re;
                        pair_im_q <= out_im;
                    end

                    if (idx == (coeff_count - 1'b1)) begin
                        state <= ST_DONE_S;
                    end else begin
                        idx <= idx + 1'b1;
                        rd_phase <= RD_Z0;
                        state <= ST_RD;
                    end
                end

                ST_DONE_S: begin
                    done <= 1; status <= 0; state <= ST_IDLE;
                end

                ST_FAIL_S: begin
                    done <= 1; fail <= 1; state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
