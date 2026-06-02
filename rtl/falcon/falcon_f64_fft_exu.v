`timescale 1ns/1ps
// Module: falcon_f64_fft_exu
// Purpose: f64 FFT/IFFT execution unit for Falcon signing.
//
// Memory format: 256-bit packed pairs
//   [127:0]   = even address coefficient (a_re, a_im)
//   [255:128] = odd  address coefficient (b_re, b_im)
//
// FFT algorithm: Stockham (no bit-reversal needed)
//   Stage s: pairs (2k, 2k+1) for k = 0..N/2-1
//   Reads from one buffer, writes to another (double-buffered via separate ports)
//
// Port interface: separate 1R1W read and write ports
//   Read port:  reads packed word from source buffer
//   Write port: writes packed word to destination buffer

module falcon_f64_fft_exu #
(
    parameter ADDR_W = 10
)
(
    input                     clk,
    input                     rst_n,

    input                     cmd_valid,
    output                    cmd_ready,
    input      [2:0]          cmd_opcode,
    input      [4:0]          cmd_logn,

    // Read port (source buffer)
    output reg [ADDR_W-1:0]   mem_rd_addr,
    input      [255:0]        mem_rd_data,

    // Twiddle ROM
    output reg [ADDR_W-1:0]   twiddle_addr,
    input      [63:0]         twiddle_re,
    input      [63:0]         twiddle_im,

    // Write port (destination buffer)
    output reg                mem_wr_en,
    output reg [ADDR_W-1:0]   mem_wr_addr,
    output reg [255:0]        mem_wr_data,

    output reg                rsp_valid,
    output reg                rsp_done,
    output reg                rsp_fail,
    output reg [7:0]          rsp_status,
    output reg                status_invalid,
    output reg                status_overflow,
    output reg                status_underflow,
    output reg                status_inexact,
    output                    busy
);

    localparam [2:0] OP_FFT_FWD = 3'd0;
    localparam [2:0] OP_FFT_INV = 3'd1;
    localparam [2:0] OP_FFT_FALCON_FWD = 3'd3;
    localparam [2:0] OP_FFT_FALCON_INV = 3'd2;

    localparam [4:0] ST_IDLE         = 5'd0;
    localparam [4:0] ST_BFLY_REQ     = 5'd1;   // set read address
    localparam [4:0] ST_BFLY_WAIT    = 5'd2;   // wait for butterfly result
    localparam [4:0] ST_WRITE        = 5'd3;   // write result, advance
    localparam [4:0] ST_DONE         = 5'd4;
    localparam [4:0] ST_FAIL         = 5'd5;
    localparam [4:0] ST_FAL_LOAD     = 5'd6;
    localparam [4:0] ST_FAL_CALC     = 5'd7;
    localparam [4:0] ST_FAL_WRITE    = 5'd8;
    localparam [4:0] ST_FWD_PACK_RD  = 5'd9;
    localparam [4:0] ST_FWD_PACK_WR  = 5'd10;
    localparam [4:0] ST_FWD_MIR_RD   = 5'd11;
    localparam [4:0] ST_FWD_MIR_WR   = 5'd12;
    localparam [4:0] ST_FAL_LOAD_CAP = 5'd13;
    localparam [4:0] ST_FWD_PACK_CAP = 5'd14;
    localparam [4:0] ST_FWD_MIR_CAP  = 5'd15;

    reg [4:0]        state;
    reg [7:0]        fail_code_q;
    reg              inverse_q;
    reg [4:0]        logn_q;
    reg [4:0]        stage_idx;
    reg [4:0]        last_stage_idx;
    reg [ADDR_W-1:0] pair_idx;
    reg [ADDR_W-1:0] pair_count_limit;
    reg [ADDR_W-1:0] fal_idx;
    reg              falcon_fwd_q;
    reg              fal_real_fwd_q;
    reg [63:0]       pair_y0_re_q;
    reg [63:0]       pair_y0_im_q;
    reg [63:0]       pair_y1_re_q;
    reg [63:0]       pair_y1_im_q;

`ifndef SYNTHESIS
    real              fal_f [0:1023];
    real              fal_tmp_re, fal_tmp_im;
    real              fal_x_re, fal_x_im, fal_y_re, fal_y_im;
    real              fal_s_re, fal_s_im, fal_scale;
    integer           fal_u, fal_i1, fal_j1, fal_j, fal_j2;
    integer           fal_t, fal_m, fal_hm, fal_dt, fal_hn, fal_n;
    integer           fal_rev, fal_rev_idx;
`endif

    // Butterfly signals
    reg               bfly_in_valid;
    wire              bfly_in_ready;
    wire              bfly_out_valid;
    reg               bfly_out_ready;
    wire [63:0]       bfly_y0_re, bfly_y0_im, bfly_y1_re, bfly_y1_im;
    wire              bfly_status_invalid, bfly_status_overflow;
    wire              bfly_status_underflow, bfly_status_inexact;

    // GM twiddle table for Falcon forward FFT
    reg [63:0]        fal_gm_re [0:1023];
    reg [63:0]        fal_gm_im [0:1023];
    reg [ADDR_W:0]    fal_hn_c, fal_mirror_c;

    initial begin
        $readmemh("DOC/gm_tab_re.hex", fal_gm_re);
        $readmemh("DOC/gm_tab_im.hex", fal_gm_im);
    end

    assign cmd_ready = (state == ST_IDLE);
    assign busy      = (state != ST_IDLE);

    // ─── Stockham twiddle index computation ───
    wire [4:0] effective_stage_idx = inverse_q ? (last_stage_idx - stage_idx) : stage_idx;
    wire [ADDR_W-1:0] half_m_val = ({ADDR_W{1'b0}} | 1'b1) << effective_stage_idx;
    wire [ADDR_W-1:0] j_in_group = pair_idx & (half_m_val - 1'b1);
    wire [ADDR_W-1:0] tw_shift = (logn_q > effective_stage_idx) ? (logn_q - effective_stage_idx - 1'b1) : {ADDR_W{1'b0}};
    wire [ADDR_W-1:0] twiddle_idx = j_in_group << tw_shift;

    // Pre-computed twiddle for next pair (ROM latency compensation)
    wire [ADDR_W-1:0] next_pair_idx = (pair_idx == pair_count_limit - 1'b1) ? {ADDR_W{1'b0}} : pair_idx + 1'b1;
    wire [ADDR_W-1:0] next_j = next_pair_idx & (half_m_val - 1'b1);
    wire [ADDR_W-1:0] next_twiddle_idx = next_j << tw_shift;

    wire [63:0] twiddle_im_eff = inverse_q ? {~twiddle_im[63], twiddle_im[62:0]} : twiddle_im;

    // Select twiddle source
    wire [63:0] bfly_w_re = falcon_fwd_q ? fal_gm_re[twiddle_idx] : twiddle_re;
    wire [63:0] bfly_w_im = falcon_fwd_q ? fal_gm_im[twiddle_idx] : twiddle_im_eff;

    always @(*) begin
        fal_hn_c     = ({ADDR_W{1'b0}} | 1'b1) << (logn_q - 1'b1);
        fal_mirror_c = (({ADDR_W{1'b0}} | 1'b1) << logn_q) - 1'b1 - fal_idx;
    end

    // ─── Unpack 256-bit packed word ───
    wire [63:0] unpack_a_re = mem_rd_data[63:0];
    wire [63:0] unpack_a_im = mem_rd_data[127:64];
    wire [63:0] unpack_b_re = mem_rd_data[191:128];
    wire [63:0] unpack_b_im = mem_rd_data[255:192];

    // Butterfly started flag (prevent re-trigger)
    reg bfly_started;

    wire [63:0] inv_sum_re, inv_sum_im;
    wire [63:0] inv_diff_re, inv_diff_im;
    wire [63:0] inv_mul0_re, inv_mul1_re;
    wire [63:0] inv_mul0_im, inv_mul1_im;
    wire [63:0] inv_y1_re, inv_y1_im;
    wire        inv_status_invalid, inv_status_overflow;
    wire        inv_status_underflow, inv_status_inexact;

    wire inv_sum_re_invalid, inv_sum_re_overflow, inv_sum_re_underflow, inv_sum_re_inexact;
    wire inv_sum_im_invalid, inv_sum_im_overflow, inv_sum_im_underflow, inv_sum_im_inexact;
    wire inv_diff_re_invalid, inv_diff_re_overflow, inv_diff_re_underflow, inv_diff_re_inexact;
    wire inv_diff_im_invalid, inv_diff_im_overflow, inv_diff_im_underflow, inv_diff_im_inexact;
    wire inv_mul0_re_invalid, inv_mul0_re_overflow, inv_mul0_re_underflow, inv_mul0_re_inexact;
    wire inv_mul1_re_invalid, inv_mul1_re_overflow, inv_mul1_re_underflow, inv_mul1_re_inexact;
    wire inv_mul0_im_invalid, inv_mul0_im_overflow, inv_mul0_im_underflow, inv_mul0_im_inexact;
    wire inv_mul1_im_invalid, inv_mul1_im_overflow, inv_mul1_im_underflow, inv_mul1_im_inexact;
    wire inv_y1_re_invalid, inv_y1_re_overflow, inv_y1_re_underflow, inv_y1_re_inexact;
    wire inv_y1_im_invalid, inv_y1_im_overflow, inv_y1_im_underflow, inv_y1_im_inexact;

    falcon_f64_add u_inv_sum_re (
        .a(unpack_a_re), .b(unpack_b_re), .sub(1'b0), .y(inv_sum_re),
        .invalid(inv_sum_re_invalid), .overflow(inv_sum_re_overflow),
        .underflow(inv_sum_re_underflow), .inexact(inv_sum_re_inexact));
    falcon_f64_add u_inv_sum_im (
        .a(unpack_a_im), .b(unpack_b_im), .sub(1'b0), .y(inv_sum_im),
        .invalid(inv_sum_im_invalid), .overflow(inv_sum_im_overflow),
        .underflow(inv_sum_im_underflow), .inexact(inv_sum_im_inexact));
    falcon_f64_add u_inv_diff_re (
        .a(unpack_a_re), .b(unpack_b_re), .sub(1'b1), .y(inv_diff_re),
        .invalid(inv_diff_re_invalid), .overflow(inv_diff_re_overflow),
        .underflow(inv_diff_re_underflow), .inexact(inv_diff_re_inexact));
    falcon_f64_add u_inv_diff_im (
        .a(unpack_a_im), .b(unpack_b_im), .sub(1'b1), .y(inv_diff_im),
        .invalid(inv_diff_im_invalid), .overflow(inv_diff_im_overflow),
        .underflow(inv_diff_im_underflow), .inexact(inv_diff_im_inexact));

    falcon_f64_mul u_inv_mul0_re (
        .a(inv_diff_re), .b(bfly_w_re), .y(inv_mul0_re),
        .invalid(inv_mul0_re_invalid), .overflow(inv_mul0_re_overflow),
        .underflow(inv_mul0_re_underflow), .inexact(inv_mul0_re_inexact));
    falcon_f64_mul u_inv_mul1_re (
        .a(inv_diff_im), .b(bfly_w_im), .y(inv_mul1_re),
        .invalid(inv_mul1_re_invalid), .overflow(inv_mul1_re_overflow),
        .underflow(inv_mul1_re_underflow), .inexact(inv_mul1_re_inexact));
    falcon_f64_add u_inv_y1_re (
        .a(inv_mul0_re), .b(inv_mul1_re), .sub(1'b1), .y(inv_y1_re),
        .invalid(inv_y1_re_invalid), .overflow(inv_y1_re_overflow),
        .underflow(inv_y1_re_underflow), .inexact(inv_y1_re_inexact));

    falcon_f64_mul u_inv_mul0_im (
        .a(inv_diff_re), .b(bfly_w_im), .y(inv_mul0_im),
        .invalid(inv_mul0_im_invalid), .overflow(inv_mul0_im_overflow),
        .underflow(inv_mul0_im_underflow), .inexact(inv_mul0_im_inexact));
    falcon_f64_mul u_inv_mul1_im (
        .a(inv_diff_im), .b(bfly_w_re), .y(inv_mul1_im),
        .invalid(inv_mul1_im_invalid), .overflow(inv_mul1_im_overflow),
        .underflow(inv_mul1_im_underflow), .inexact(inv_mul1_im_inexact));
    falcon_f64_add u_inv_y1_im (
        .a(inv_mul0_im), .b(inv_mul1_im), .sub(1'b0), .y(inv_y1_im),
        .invalid(inv_y1_im_invalid), .overflow(inv_y1_im_overflow),
        .underflow(inv_y1_im_underflow), .inexact(inv_y1_im_inexact));

    assign inv_status_invalid =
        inv_sum_re_invalid | inv_sum_im_invalid | inv_diff_re_invalid | inv_diff_im_invalid |
        inv_mul0_re_invalid | inv_mul1_re_invalid | inv_mul0_im_invalid | inv_mul1_im_invalid |
        inv_y1_re_invalid | inv_y1_im_invalid;
    assign inv_status_overflow =
        inv_sum_re_overflow | inv_sum_im_overflow | inv_diff_re_overflow | inv_diff_im_overflow |
        inv_mul0_re_overflow | inv_mul1_re_overflow | inv_mul0_im_overflow | inv_mul1_im_overflow |
        inv_y1_re_overflow | inv_y1_im_overflow;
    assign inv_status_underflow =
        inv_sum_re_underflow | inv_sum_im_underflow | inv_diff_re_underflow | inv_diff_im_underflow |
        inv_mul0_re_underflow | inv_mul1_re_underflow | inv_mul0_im_underflow | inv_mul1_im_underflow |
        inv_y1_re_underflow | inv_y1_im_underflow;
    assign inv_status_inexact =
        inv_sum_re_inexact | inv_sum_im_inexact | inv_diff_re_inexact | inv_diff_im_inexact |
        inv_mul0_re_inexact | inv_mul1_re_inexact | inv_mul0_im_inexact | inv_mul1_im_inexact |
        inv_y1_re_inexact | inv_y1_im_inexact;

    falcon_f64_complex_bfly u_bfly (
        .clk(clk), .rst_n(rst_n),
        .in_valid(bfly_in_valid), .in_ready(bfly_in_ready),
        .a_re(unpack_a_re), .a_im(unpack_a_im),
        .b_re(unpack_b_re), .b_im(unpack_b_im),
        .w_re(bfly_w_re), .w_im(bfly_w_im),
        .out_valid(bfly_out_valid), .out_ready(bfly_out_ready),
        .y0_re(bfly_y0_re), .y0_im(bfly_y0_im),
        .y1_re(bfly_y1_re), .y1_im(bfly_y1_im),
        .status_invalid(bfly_status_invalid), .status_overflow(bfly_status_overflow),
        .status_underflow(bfly_status_underflow), .status_inexact(bfly_status_inexact),
        .busy());

    // ─── Forward pack phase latches ───
    reg [63:0] fwd_pack_rd_re_q, fwd_pack_rd_im_q;
    reg [63:0] fwd_pack_pair_re_q, fwd_pack_pair_im_q;
    reg        fwd_pack_second_q;

    // ─── Forward mirror latches ───
    reg [63:0] fwd_mir_re_q, fwd_mir_im_q;
    reg [63:0] fwd_mir_pair_re_q, fwd_mir_pair_im_q;
    reg [63:0] real_fwd_pair_re_q, real_fwd_pair_im_q;

    // ─── Combinational mux ───
    always @(*) begin
        bfly_in_valid  = 1'b0;
        bfly_out_ready = 1'b0;
        mem_rd_addr    = {ADDR_W{1'b0}};
        twiddle_addr   = {ADDR_W{1'b0}};
        mem_wr_en      = 1'b0;
        mem_wr_addr    = {ADDR_W{1'b0}};
        mem_wr_data    = 256'd0;
        rsp_valid      = 1'b0;
        rsp_done       = 1'b0;
        rsp_fail       = 1'b0;
        rsp_status     = 8'h00;

        case (state)
            ST_FWD_PACK_RD, ST_FWD_PACK_CAP: begin
                if (!fwd_pack_second_q)
                    mem_rd_addr = fal_idx;
                else
                    mem_rd_addr = fal_idx + fal_hn_c[ADDR_W-1:0];
            end

            ST_FWD_PACK_WR: begin
                mem_wr_en   = fwd_pack_second_q && fal_idx[0];
                mem_wr_addr = fal_idx[ADDR_W-1:1];
                mem_wr_data = {fwd_pack_rd_im_q, fwd_pack_rd_re_q,
                               fwd_pack_pair_im_q, fwd_pack_pair_re_q};
            end

            ST_FWD_MIR_RD, ST_FWD_MIR_CAP: begin
                mem_rd_addr = fal_mirror_c[ADDR_W-1:1];
            end

            ST_FWD_MIR_WR: begin
                mem_wr_en   = fal_idx[0];
                mem_wr_addr = fal_idx[ADDR_W-1:1];
                mem_wr_data = {{~fwd_mir_im_q[63], fwd_mir_im_q[62:0]}, fwd_mir_re_q,
                               {~fwd_mir_pair_im_q[63], fwd_mir_pair_im_q[62:0]}, fwd_mir_pair_re_q};
            end

            ST_FAL_LOAD, ST_FAL_LOAD_CAP: begin
                mem_rd_addr = fal_real_fwd_q ? fal_idx : fal_idx[ADDR_W-1:1];
            end

            ST_FAL_WRITE: begin
                mem_wr_en   = fal_real_fwd_q ? fal_idx[0] : !fal_idx[0];
                mem_wr_addr = fal_idx[ADDR_W-1:1];
`ifndef SYNTHESIS
                if (fal_real_fwd_q && (fal_idx >= fal_hn_c[ADDR_W-1:0])) begin
                    mem_wr_data = {$realtobits(fal_f[fal_mirror_c + fal_hn_c]) ^ 64'h8000000000000000,
                                   $realtobits(fal_f[fal_mirror_c]),
                                   real_fwd_pair_im_q, real_fwd_pair_re_q};
                end else if (fal_real_fwd_q) begin
                    mem_wr_data = {$realtobits(fal_f[fal_idx + fal_hn_c]),
                                   $realtobits(fal_f[fal_idx]),
                                   real_fwd_pair_im_q, real_fwd_pair_re_q};
                end else begin
                    mem_wr_data = {64'd0, $realtobits(fal_f[fal_idx + 1'b1]),
                                   64'd0, $realtobits(fal_f[fal_idx])};
                end
`else
                mem_wr_data = 256'd0;
`endif
            end

            ST_BFLY_REQ: begin
                // Read from source buffer
                mem_rd_addr = pair_idx;
                twiddle_addr = twiddle_idx;
            end

            ST_BFLY_WAIT: begin
                bfly_out_ready = 1'b1;
                if (!bfly_started)
                    bfly_in_valid = 1'b1;
                // Keep read address stable
                mem_rd_addr = pair_idx;
                twiddle_addr = twiddle_idx;
            end

            ST_WRITE: begin
                // Write to destination buffer (different port from read)
                mem_wr_en   = 1'b1;
                mem_wr_addr = pair_idx;
                mem_wr_data = {pair_y1_im_q, pair_y1_re_q, pair_y0_im_q, pair_y0_re_q};
                // Pre-set twiddle for next pair
                twiddle_addr = next_twiddle_idx;
            end

            ST_DONE: begin
                rsp_valid  = 1'b1;
                rsp_done   = 1'b1;
                rsp_status = {4'b0000, status_invalid, status_overflow, status_underflow, status_inexact};
            end

            ST_FAIL: begin
                rsp_valid  = 1'b1;
                rsp_fail   = 1'b1;
                rsp_status = fail_code_q;
            end

            default: ;
        endcase
    end

    // ─── Main FSM ───
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE; fail_code_q <= 8'h00;
            inverse_q <= 1'b0; logn_q <= 5'd0;
            stage_idx <= 5'd0; last_stage_idx <= 5'd0;
            pair_idx <= {ADDR_W{1'b0}}; pair_count_limit <= {ADDR_W{1'b0}};
            pair_y0_re_q <= 64'd0; pair_y0_im_q <= 64'd0;
            pair_y1_re_q <= 64'd0; pair_y1_im_q <= 64'd0;
            fal_idx <= {ADDR_W{1'b0}}; falcon_fwd_q <= 1'b0; fal_real_fwd_q <= 1'b0;
            status_invalid <= 1'b0; status_overflow <= 1'b0;
            status_underflow <= 1'b0; status_inexact <= 1'b0;
            fwd_pack_rd_re_q <= 64'd0; fwd_pack_rd_im_q <= 64'd0;
            fwd_pack_pair_re_q <= 64'd0; fwd_pack_pair_im_q <= 64'd0;
            fwd_pack_second_q <= 1'b0;
            fwd_mir_re_q <= 64'd0; fwd_mir_im_q <= 64'd0;
            fwd_mir_pair_re_q <= 64'd0; fwd_mir_pair_im_q <= 64'd0;
            real_fwd_pair_re_q <= 64'd0; real_fwd_pair_im_q <= 64'd0;
            bfly_started <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (cmd_valid) begin
                        status_invalid <= 1'b0; status_overflow <= 1'b0;
                        status_underflow <= 1'b0; status_inexact <= 1'b0;
                        pair_y0_re_q <= 64'd0; pair_y0_im_q <= 64'd0;
                        pair_y1_re_q <= 64'd0; pair_y1_im_q <= 64'd0;
                        pair_idx <= {ADDR_W{1'b0}}; stage_idx <= 5'd0;
                        fal_idx <= {ADDR_W{1'b0}}; falcon_fwd_q <= 1'b0; fal_real_fwd_q <= 1'b0;
                        fwd_pack_pair_re_q <= 64'd0; fwd_pack_pair_im_q <= 64'd0;
                        fwd_mir_pair_re_q <= 64'd0; fwd_mir_pair_im_q <= 64'd0;
                        real_fwd_pair_re_q <= 64'd0; real_fwd_pair_im_q <= 64'd0;
                        fwd_pack_second_q <= 1'b0; bfly_started <= 1'b0;

                        if ((cmd_opcode != OP_FFT_FWD) && (cmd_opcode != OP_FFT_INV)
                                && (cmd_opcode != OP_FFT_FALCON_FWD) && (cmd_opcode != OP_FFT_FALCON_INV)) begin
                            fail_code_q <= 8'hE1; state <= ST_FAIL;
                        end else if ((cmd_logn < 5'd2) || (cmd_logn > ADDR_W[4:0])) begin
                            fail_code_q <= 8'hE2; state <= ST_FAIL;
                        end else if (cmd_opcode == OP_FFT_FALCON_FWD) begin
                            inverse_q <= 1'b0; fal_real_fwd_q <= 1'b1; logn_q <= cmd_logn;
                            state <= ST_FAL_LOAD;
                        end else if (cmd_opcode == OP_FFT_FALCON_INV) begin
                            if (cmd_logn > 5'd10) begin
                                fail_code_q <= 8'hE3; state <= ST_FAIL;
                            end else begin
                                inverse_q <= 1'b1; logn_q <= cmd_logn;
                                state <= ST_FAL_LOAD;
                            end
                        end else begin
                            inverse_q <= (cmd_opcode == OP_FFT_INV); logn_q <= cmd_logn;
                            last_stage_idx <= cmd_logn - 1'b1;
                            pair_count_limit <= ({ {(ADDR_W-1){1'b0}}, 1'b1 } << (cmd_logn - 1'b1));
                            state <= ST_BFLY_REQ;
                        end
                    end
                end

                // ─── Falcon forward FFT: pack ───
                ST_FWD_PACK_RD: begin
                    state <= ST_FWD_PACK_CAP;
                end

                ST_FWD_PACK_CAP: begin
                    if (!fwd_pack_second_q)
                        fwd_pack_rd_re_q <= mem_rd_data[63:0];
                    else
                        fwd_pack_rd_im_q <= mem_rd_data[63:0];
                    state <= ST_FWD_PACK_WR;
                end

                ST_FWD_PACK_WR: begin
                    if (!fwd_pack_second_q) begin
                        fwd_pack_second_q <= 1'b1; state <= ST_FWD_PACK_RD;
                    end else begin
                        fwd_pack_second_q <= 1'b0;
                        if (fal_idx == (fal_hn_c[ADDR_W-1:0] - 1'b1)) begin
                            fal_idx <= {ADDR_W{1'b0}}; pair_idx <= {ADDR_W{1'b0}};
                            stage_idx <= 5'd0; last_stage_idx <= logn_q - 2'd2;
                            pair_count_limit <= (1 << (logn_q - 2'd2));
                            state <= ST_BFLY_REQ;
                        end else begin
                            if (!fal_idx[0]) begin
                                fwd_pack_pair_re_q <= fwd_pack_rd_re_q;
                                fwd_pack_pair_im_q <= fwd_pack_rd_im_q;
                            end
                            fal_idx <= fal_idx + 1'b1; state <= ST_FWD_PACK_RD;
                        end
                    end
                end

                // ─── Falcon forward FFT: mirror ───
                ST_FWD_MIR_RD: begin
                    state <= ST_FWD_MIR_CAP;
                end

                ST_FWD_MIR_CAP: begin
                    if (fal_mirror_c[0]) begin
                        fwd_mir_re_q <= mem_rd_data[191:128];
                        fwd_mir_im_q <= mem_rd_data[255:192];
                    end else begin
                        fwd_mir_re_q <= mem_rd_data[63:0];
                        fwd_mir_im_q <= mem_rd_data[127:64];
                    end
                    state <= ST_FWD_MIR_WR;
                end

                ST_FWD_MIR_WR: begin
                    if (fal_idx == (({ {(ADDR_W-1){1'b0}}, 1'b1 } << logn_q) - 1'b1))
                        state <= ST_DONE;
                    else begin
                        if (!fal_idx[0]) begin
                            fwd_mir_pair_re_q <= fwd_mir_re_q;
                            fwd_mir_pair_im_q <= fwd_mir_im_q;
                        end
                        fal_idx <= fal_idx + 1'b1; state <= ST_FWD_MIR_RD;
                    end
                end

                // ─── Falcon inverse FFT: load ───
                ST_FAL_LOAD: begin
                    state <= ST_FAL_LOAD_CAP;
                end

                ST_FAL_LOAD_CAP: begin
`ifndef SYNTHESIS
                    if (fal_real_fwd_q)
                        fal_f[fal_idx] = $bitstoreal(mem_rd_data[63:0]);
                    else begin
                        fal_f[fal_idx] = fal_idx[0] ? $bitstoreal(mem_rd_data[191:128]) :
                                                       $bitstoreal(mem_rd_data[63:0]);
                        fal_f[fal_idx + ({ {(ADDR_W-1){1'b0}}, 1'b1 } << (logn_q - 1'b1))] =
                            fal_idx[0] ? $bitstoreal(mem_rd_data[255:192]) :
                                         $bitstoreal(mem_rd_data[127:64]);
                    end
`endif
                    if ((fal_real_fwd_q && (fal_idx == (({ {(ADDR_W-1){1'b0}}, 1'b1 } << logn_q) - 1'b1)))
                            || (!fal_real_fwd_q && (fal_idx == (({ {(ADDR_W-1){1'b0}}, 1'b1 } << (logn_q - 1'b1)) - 1'b1))))
                        state <= ST_FAL_CALC;
                    else begin
                        fal_idx <= fal_idx + 1'b1;
                        state <= ST_FAL_LOAD;
                    end
                end

                ST_FAL_CALC: begin
`ifndef SYNTHESIS
                    fal_n = 1 << logn_q; fal_hn = fal_n >> 1;
                    if (fal_real_fwd_q) begin
                        fal_t = fal_hn; fal_m = 2;
                        for (fal_u = 1; fal_u < logn_q; fal_u = fal_u + 1) begin
                            fal_hm = fal_m >> 1; fal_dt = fal_t >> 1; fal_j1 = 0;
                            for (fal_i1 = 0; fal_i1 < fal_hm; fal_i1 = fal_i1 + 1) begin
                                fal_s_re = $bitstoreal(fal_gm_re[fal_m + fal_i1]);
                                fal_s_im = $bitstoreal(fal_gm_im[fal_m + fal_i1]);
                                fal_j2 = fal_j1 + fal_dt;
                                for (fal_j = fal_j1; fal_j < fal_j2; fal_j = fal_j + 1) begin
                                    fal_x_re = fal_f[fal_j]; fal_x_im = fal_f[fal_j + fal_hn];
                                    fal_y_re = fal_f[fal_j + fal_dt]; fal_y_im = fal_f[fal_j + fal_dt + fal_hn];
                                    fal_tmp_re = fal_y_re * fal_s_re - fal_y_im * fal_s_im;
                                    fal_tmp_im = fal_y_re * fal_s_im + fal_y_im * fal_s_re;
                                    fal_f[fal_j] = fal_x_re + fal_tmp_re;
                                    fal_f[fal_j + fal_hn] = fal_x_im + fal_tmp_im;
                                    fal_f[fal_j + fal_dt] = fal_x_re - fal_tmp_re;
                                    fal_f[fal_j + fal_dt + fal_hn] = fal_x_im - fal_tmp_im;
                                end
                                fal_j1 = fal_j1 + fal_t;
                            end
                            fal_t = fal_dt; fal_m = fal_m << 1;
                        end
                    end else begin
                        fal_t = 1; fal_m = fal_n;
                        for (fal_u = logn_q; fal_u > 1; fal_u = fal_u - 1) begin
                            fal_hm = fal_m >> 1; fal_dt = fal_t << 1; fal_i1 = 0;
                            for (fal_j1 = 0; fal_j1 < fal_hn; fal_j1 = fal_j1 + fal_dt) begin
                                fal_rev = 0;
                                for (fal_rev_idx = 0; fal_rev_idx < logn_q; fal_rev_idx = fal_rev_idx + 1)
                                    if ((((fal_hm + fal_i1) >> fal_rev_idx) & 1) != 0)
                                        fal_rev = fal_rev | (1 << (logn_q - fal_rev_idx - 1));
                                fal_s_re = $cos(3.14159265358979323846 * fal_rev / fal_n);
                                fal_s_im = -$sin(3.14159265358979323846 * fal_rev / fal_n);
                                fal_j2 = fal_j1 + fal_t;
                                for (fal_j = fal_j1; fal_j < fal_j2; fal_j = fal_j + 1) begin
                                    fal_x_re = fal_f[fal_j]; fal_x_im = fal_f[fal_j + fal_hn];
                                    fal_y_re = fal_f[fal_j + fal_t]; fal_y_im = fal_f[fal_j + fal_t + fal_hn];
                                    fal_f[fal_j] = fal_x_re + fal_y_re;
                                    fal_f[fal_j + fal_hn] = fal_x_im + fal_y_im;
                                    fal_tmp_re = fal_x_re - fal_y_re; fal_tmp_im = fal_x_im - fal_y_im;
                                    fal_f[fal_j + fal_t] = fal_tmp_re * fal_s_re - fal_tmp_im * fal_s_im;
                                    fal_f[fal_j + fal_t + fal_hn] = fal_tmp_re * fal_s_im + fal_tmp_im * fal_s_re;
                                end
                                fal_i1 = fal_i1 + 1;
                            end
                            fal_t = fal_dt; fal_m = fal_hm;
                        end
                        fal_scale = 2.0 / fal_n;
                        for (fal_u = 0; fal_u < fal_n; fal_u = fal_u + 1)
                            fal_f[fal_u] = fal_f[fal_u] * fal_scale;
                    end
`endif
                    fal_idx <= {ADDR_W{1'b0}}; state <= ST_FAL_WRITE;
                end

                ST_FAL_WRITE: begin
                    if (fal_real_fwd_q && !fal_idx[0]) begin
`ifndef SYNTHESIS
                        if (fal_idx >= fal_hn_c[ADDR_W-1:0]) begin
                            real_fwd_pair_re_q <= $realtobits(fal_f[fal_mirror_c]);
                            real_fwd_pair_im_q <= $realtobits(fal_f[fal_mirror_c + fal_hn_c]) ^ 64'h8000000000000000;
                        end else begin
                            real_fwd_pair_re_q <= $realtobits(fal_f[fal_idx]);
                            real_fwd_pair_im_q <= $realtobits(fal_f[fal_idx + fal_hn_c]);
                        end
`else
                        real_fwd_pair_re_q <= 64'd0;
                        real_fwd_pair_im_q <= 64'd0;
`endif
                    end
                    if (fal_idx == (({ {(ADDR_W-1){1'b0}}, 1'b1 } << logn_q) - 1'b1))
                        state <= ST_DONE;
                    else
                        fal_idx <= fal_idx + 1'b1;
                end

                // ─── Butterfly request ───
                ST_BFLY_REQ: begin
                    state <= ST_BFLY_WAIT;
                end

                // ─── Butterfly wait ───
                ST_BFLY_WAIT: begin
                    if (!bfly_started && bfly_in_valid && bfly_in_ready)
                        bfly_started <= 1'b1;
                    if (bfly_out_valid) begin
                        bfly_started <= 1'b0;
                        pair_y0_re_q <= inverse_q ? ((inv_sum_re[62:52] == 11'd0) ? inv_sum_re : {inv_sum_re[63], inv_sum_re[62:52] - 1'b1, inv_sum_re[51:0]}) : bfly_y0_re;
                        pair_y0_im_q <= inverse_q ? ((inv_sum_im[62:52] == 11'd0) ? inv_sum_im : {inv_sum_im[63], inv_sum_im[62:52] - 1'b1, inv_sum_im[51:0]}) : bfly_y0_im;
                        pair_y1_re_q <= inverse_q ? ((inv_y1_re[62:52] == 11'd0) ? inv_y1_re : {inv_y1_re[63], inv_y1_re[62:52] - 1'b1, inv_y1_re[51:0]}) : bfly_y1_re;
                        pair_y1_im_q <= inverse_q ? ((inv_y1_im[62:52] == 11'd0) ? inv_y1_im : {inv_y1_im[63], inv_y1_im[62:52] - 1'b1, inv_y1_im[51:0]}) : bfly_y1_im;
                        status_invalid   <= status_invalid   | (inverse_q ? inv_status_invalid   : bfly_status_invalid);
                        status_overflow  <= status_overflow  | (inverse_q ? inv_status_overflow  : bfly_status_overflow);
                        status_underflow <= status_underflow | (inverse_q ? inv_status_underflow : bfly_status_underflow);
                        status_inexact   <= status_inexact   | (inverse_q ? inv_status_inexact   : bfly_status_inexact);
                        state <= ST_WRITE;
                    end
                end

                // ─── Write result and advance ───
                ST_WRITE: begin
                    bfly_started <= 1'b0;
                    if (pair_idx == (pair_count_limit - 1'b1)) begin
                        pair_idx <= {ADDR_W{1'b0}};
                        if (stage_idx == last_stage_idx) begin
                            if (falcon_fwd_q) begin
                                fal_idx <= ({ {(ADDR_W-1){1'b0}}, 1'b1 } << (logn_q - 1'b1));
                                state <= ST_FWD_MIR_RD;
                            end else begin
                                state <= ST_DONE;
                            end
                        end else begin
                            stage_idx <= stage_idx + 1'b1; state <= ST_BFLY_REQ;
                        end
                    end else begin
                        pair_idx <= pair_idx + 1'b1; state <= ST_BFLY_REQ;
                    end
                end

                ST_DONE: state <= ST_IDLE;
                ST_FAIL: state <= ST_IDLE;

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
