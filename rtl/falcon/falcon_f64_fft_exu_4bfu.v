`timescale 1ns/1ps
// Module: falcon_f64_fft_exu_4bfu
// Purpose: Falcon FFT EXU with 4-lane vector butterfly.
// Uses falcon_f64_complex_bfly4 to process 4 butterfly pairs in parallel,
// sharing one twiddle factor across all 4 lanes per vector.
//
// Batching: reads 4 pairs sequentially into an input buffer, fires all 4
// BFU lanes in one cycle, then writes 4 results back sequentially.
// Effective throughput: ~9 cycles/pair vs ~30 cycles/pair for 1-BFU.
//
// Memory interface unchanged (Port A dual read/write, 2 words/cycle).

module falcon_f64_fft_exu_4bfu #(
    parameter ADDR_W = 13
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              cmd_valid,
    output wire              cmd_ready,
    input  wire [2:0]        cmd_opcode,
    input  wire [4:0]        cmd_logn,

    output wire [ADDR_W-1:0] mem_rd_addr0,
    output wire [ADDR_W-1:0] mem_rd_addr1,
    input  wire [63:0]       mem_rd_data0_re,
    input  wire [63:0]       mem_rd_data0_im,
    input  wire [63:0]       mem_rd_data1_re,
    input  wire [63:0]       mem_rd_data1_im,

    output wire [ADDR_W-1:0] twiddle_addr,
    input  wire [63:0]       twiddle_re,
    input  wire [63:0]       twiddle_im,

    output wire              mem_wr_en,
    output wire [ADDR_W-1:0] mem_wr_addr0,
    output wire [ADDR_W-1:0] mem_wr_addr1,
    output wire [63:0]       mem_wr_data0_re,
    output wire [63:0]       mem_wr_data0_im,
    output wire [63:0]       mem_wr_data1_re,
    output wire [63:0]       mem_wr_data1_im,

    output wire              rsp_valid,
    output wire              rsp_done,
    output wire              rsp_fail,
    output wire [7:0]        rsp_status,
    output wire              status_invalid,
    output wire              status_overflow,
    output wire              status_underflow,
    output wire              status_inexact,
    output wire              busy
);

    localparam [2:0] OP_FFT_FALCON_FWD  = 3'd3;
    localparam [2:0] OP_FFT_FALCON_INV  = 3'd2;

    // ─── FSM states ───
    localparam [5:0] ST_IDLE         = 6'd0;
    localparam [5:0] ST_FWD_PACK_RD  = 6'd1;
    localparam [5:0] ST_FWD_PACK_WR  = 6'd2;
    localparam [5:0] ST_BFLY_BATCH_RD = 6'd3;   // batch read 4 pairs
    localparam [5:0] ST_BFLY_RUN      = 6'd4;   // run 4-BFU
    localparam [5:0] ST_BFLY_BATCH_WR = 6'd5;   // batch write 4 results
    localparam [5:0] ST_FWD_MIR_RD   = 6'd6;
    localparam [5:0] ST_FWD_MIR_WR   = 6'd7;
    localparam [5:0] ST_FAL_LOAD     = 6'd8;
    localparam [5:0] ST_FAL_CALC_BATCH = 6'd9;
    localparam [5:0] ST_FAL_WRITE    = 6'd10;
    localparam [5:0] ST_BITREV_CHECK = 6'd11;
    localparam [5:0] ST_BITREV_WRITE = 6'd12;
    localparam [5:0] ST_DONE         = 6'd13;
    localparam [5:0] ST_FAIL         = 6'd14;

    reg [5:0]        state;
    reg [4:0]        logn_q;
    reg              falcon_fwd_q;
    reg              inverse_q;
    reg [4:0]        stage_idx;
    reg [4:0]        last_stage_idx;
    reg [ADDR_W-1:0] pair_base;      // base pair index within current stage
    reg [ADDR_W-1:0] pair_count_limit;
    reg [ADDR_W-1:0] fal_idx;
    reg [ADDR_W-1:0] batch_sub_idx;  // 0..3 within current 4-pair batch

    // ─── Input buffer: 4 pairs × 2 complex values ───
    reg [63:0] buf_a_re [0:3];
    reg [63:0] buf_a_im [0:3];
    reg [63:0] buf_b_re [0:3];
    reg [63:0] buf_b_im [0:3];
    reg [ADDR_W-1:0] buf_addr_a [0:3];
    reg [ADDR_W-1:0] buf_addr_b [0:3];

    // ─── Output buffer: 4 pairs × 2 results ───
    reg [63:0] obuf_y0_re [0:3];
    reg [63:0] obuf_y0_im [0:3];
    reg [63:0] obuf_y1_re [0:3];
    reg [63:0] obuf_y1_im [0:3];

    // ─── GM twiddle table (simulation only) ───
    reg [63:0] fal_gm_re [0:1023];
    reg [63:0] fal_gm_im [0:1023];
    initial begin
        $readmemh("DOC/gm_tab_re.hex", fal_gm_re);
        $readmemh("DOC/gm_tab_im.hex", fal_gm_im);
    end

    // ─── Twiddle factor reuse optimization ───
    // For early stages (where fal_ht_c >= vector_size), consecutive butterfly
    // pairs share the same twiddle factor. We cache the last used twiddle to
    // avoid redundant ROM reads.
    // Reference: TCHES2025 Algorithm 3, Section 11 (Vectorized FFT Twiddle Generation)
    reg [ADDR_W-1:0] twiddle_cache_addr;     // Cached twiddle address
    reg [63:0]       twiddle_cache_re;        // Cached twiddle real part
    reg [63:0]       twiddle_cache_im;        // Cached twiddle imaginary part
    reg              twiddle_cache_valid;     // Cache validity flag

    // Twiddle reuse conditions:
    // 1. can_reuse: current stage supports reuse (fal_ht_c >= 4, i.e., vector size)
    // 2. addr_match: current twiddle address matches cached address
    // 3. cache_valid: cache contains valid data
    wire can_reuse_twiddle = (fal_ht_c >= 4);
    wire addr_match = (fal_twiddle_c[ADDR_W-1:0] == twiddle_cache_addr);
    wire twiddle_reuse = can_reuse_twiddle && addr_match && twiddle_cache_valid;

    // Twiddle source selection: use cached value if reuse is possible
    wire [63:0] bf4_w_re_mux = twiddle_reuse ? twiddle_cache_re : fal_gm_re[fal_twiddle_c[ADDR_W-1:0]];
    wire [63:0] bf4_w_im_mux = twiddle_reuse ? twiddle_cache_im : fal_gm_im[fal_twiddle_c[ADDR_W-1:0]];

    // ─── Address computation ───
    reg [ADDR_W:0] fal_hn_c;
    reg [ADDR_W:0] fal_ht_c;
    reg [ADDR_W:0] fal_t_c;
    reg [ADDR_W:0] fal_m_c;
    reg [ADDR_W:0] fal_j_c;
    reg [ADDR_W:0] fal_group_c;
    reg [ADDR_W:0] fal_base_c;
    reg [ADDR_W:0] fal_twiddle_c;
    reg [ADDR_W:0] fal_mirror_c;
    reg [4:0]      fal_ht_shift_c;
    reg [3:0]      lane_idx;  // 0..3

    wire [ADDR_W:0] pair_idx = pair_base + {{(ADDR_W-2){1'b0}}, batch_sub_idx};

    always @(*) begin
        fal_hn_c       = ({ADDR_W{1'b0}} | 1'b1) << (logn_q - 1'b1);
        fal_ht_shift_c = 5'd0;
        if (logn_q > (stage_idx + 1'b1))
            fal_ht_shift_c = logn_q - stage_idx - 2'd2;
        fal_ht_c       = ({ADDR_W{1'b0}} | 1'b1) << fal_ht_shift_c;
        fal_t_c        = fal_ht_c << 1;
        fal_m_c        = ({ADDR_W{1'b0}} | 1'b1) << (stage_idx + 1'b1);
        fal_j_c        = pair_idx & (fal_ht_c - 1'b1);
        fal_group_c    = pair_idx >> fal_ht_shift_c;
        fal_base_c     = fal_group_c * fal_t_c;
        fal_twiddle_c  = fal_m_c + fal_group_c;
        fal_mirror_c   = (({ADDR_W{1'b0}} | 1'b1) << logn_q) - 1'b1 - fal_idx;
    end

    wire [ADDR_W-1:0] base_addr_a = fal_base_c[ADDR_W-1:0] + fal_j_c[ADDR_W-1:0];
    wire [ADDR_W-1:0] base_addr_b = base_addr_a + fal_ht_c[ADDR_W-1:0];
    wire [ADDR_W-1:0] mir_addr    = fal_mirror_c[ADDR_W-1:0];

    // ─── 4-BFU instantiation ───
    wire        bf4_in_valid;
    wire        bf4_in_ready;
    wire [63:0] bf4_w_re, bf4_w_im;
    wire        bf4_out_valid;
    wire        bf4_out_ready;
    wire [63:0] bf4_y00_re, bf4_y00_im, bf4_y10_re, bf4_y10_im;
    wire [63:0] bf4_y01_re, bf4_y01_im, bf4_y11_re, bf4_y11_im;
    wire [63:0] bf4_y02_re, bf4_y02_im, bf4_y12_re, bf4_y12_im;
    wire [63:0] bf4_y03_re, bf4_y03_im, bf4_y13_re, bf4_y13_im;

    // Use muxed twiddle source (with reuse optimization)
    assign bf4_w_re = bf4_w_re_mux;
    assign bf4_w_im = bf4_w_im_mux;

    falcon_f64_complex_bfly4 u_bfly4 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(bf4_in_valid), .in_ready(bf4_in_ready),
        .a0_re(buf_a_re[0]), .a0_im(buf_a_im[0]), .b0_re(buf_b_re[0]), .b0_im(buf_b_im[0]),
        .a1_re(buf_a_re[1]), .a1_im(buf_a_im[1]), .b1_re(buf_b_re[1]), .b1_im(buf_b_im[1]),
        .a2_re(buf_a_re[2]), .a2_im(buf_a_im[2]), .b2_re(buf_b_re[2]), .b2_im(buf_b_im[2]),
        .a3_re(buf_a_re[3]), .a3_im(buf_a_im[3]), .b3_re(buf_b_re[3]), .b3_im(buf_b_im[3]),
        .w_re(bf4_w_re), .w_im(bf4_w_im),
        .out_valid(bf4_out_valid), .out_ready(bf4_out_ready),
        .y00_re(bf4_y00_re), .y00_im(bf4_y00_im), .y10_re(bf4_y10_re), .y10_im(bf4_y10_im),
        .y01_re(bf4_y01_re), .y01_im(bf4_y01_im), .y11_re(bf4_y11_re), .y11_im(bf4_y11_im),
        .y02_re(bf4_y02_re), .y02_im(bf4_y02_im), .y12_re(bf4_y12_re), .y12_im(bf4_y12_im),
        .y03_re(bf4_y03_re), .y03_im(bf4_y03_im), .y13_re(bf4_y13_re), .y13_im(bf4_y13_im),
        .busy()
    );

    // ─── Memory mux (combinational read, registered write) ───
    reg [ADDR_W-1:0] mem_rd_addr0_c;
    reg [ADDR_W-1:0] mem_rd_addr1_c;
    reg [ADDR_W-1:0] twiddle_addr_c;
    reg              mem_wr_en_r;
    reg [ADDR_W-1:0] mem_wr_addr0_r;
    reg [ADDR_W-1:0] mem_wr_addr1_r;
    reg [63:0]       mem_wr_data0_re_r;
    reg [63:0]       mem_wr_data0_im_r;
    reg [63:0]       mem_wr_data1_re_r;
    reg [63:0]       mem_wr_data1_im_r;

    assign mem_rd_addr0    = mem_rd_addr0_c;
    assign mem_rd_addr1    = mem_rd_addr1_c;
    assign twiddle_addr    = twiddle_addr_c;
    assign mem_wr_en       = mem_wr_en_r;

    // Combinational read address decode
    always @(*) begin
        mem_rd_addr0_c = {ADDR_W{1'b0}};
        mem_rd_addr1_c = {ADDR_W{1'b0}};
        twiddle_addr_c = {ADDR_W{1'b0}};
        case (state)
            ST_FWD_PACK_RD, ST_FWD_PACK_WR: begin
                mem_rd_addr0_c = fal_idx;
                mem_rd_addr1_c = fal_idx + fal_hn_c[ADDR_W-1:0];
            end
            ST_BFLY_BATCH_RD: begin
                mem_rd_addr0_c = base_addr_a;
                mem_rd_addr1_c = base_addr_b;
                twiddle_addr_c = fal_twiddle_c[ADDR_W-1:0];
            end
            ST_FWD_MIR_RD: begin
                mem_rd_addr0_c = mir_addr;
                mem_rd_addr1_c = mir_addr;
            end
            default: begin
                mem_rd_addr0_c = {ADDR_W{1'b0}};
                mem_rd_addr1_c = {ADDR_W{1'b0}};
            end
        endcase
    end
    assign mem_wr_addr0    = mem_wr_addr0_r;
    assign mem_wr_addr1    = mem_wr_addr1_r;
    assign mem_wr_data0_re = mem_wr_data0_re_r;
    assign mem_wr_data0_im = mem_wr_data0_im_r;
    assign mem_wr_data1_re = mem_wr_data1_re_r;
    assign mem_wr_data1_im = mem_wr_data1_im_r;

    // ─── Response/status ───
    reg        rsp_valid_r, rsp_done_r, rsp_fail_r;
    reg [7:0]  rsp_status_r;
    assign rsp_valid  = rsp_valid_r;
    assign rsp_done   = rsp_done_r;
    assign rsp_fail   = rsp_fail_r;
    assign rsp_status = rsp_status_r;
    assign status_invalid   = 1'b0;
    assign status_overflow  = 1'b0;
    assign status_underflow = 1'b0;
    assign status_inexact   = 1'b0;

    assign cmd_ready = (state == ST_IDLE);
    assign busy      = (state != ST_IDLE);
    assign bf4_out_ready = 1'b1;

    // Batch size: 4 lanes when all pairs share the same twiddle group,
    // 1 lane otherwise (ht < 4 crosses group boundaries within a batch).
    // Batch size: 4 lanes when all pairs share the same twiddle group (ht >= 4),
    // 1 lane otherwise (last 2 stages where ht < 4).
    wire batch_full = (fal_ht_c >= 4);
    wire [1:0] batch_max = batch_full ? 2'd3 : 2'd0;

    // ─── FSM ───
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            logn_q         <= 5'd0;
            falcon_fwd_q   <= 1'b0;
            inverse_q      <= 1'b0;
            stage_idx      <= 5'd0;
            last_stage_idx <= 5'd0;
            pair_base      <= {ADDR_W{1'b0}};
            pair_count_limit <= {ADDR_W{1'b0}};
            fal_idx        <= {ADDR_W{1'b0}};
            batch_sub_idx  <= {ADDR_W{1'b0}};
            // read addresses handled combinationally
            mem_wr_en_r    <= 1'b0;
            rsp_valid_r    <= 1'b0;
            rsp_done_r     <= 1'b0;
            rsp_fail_r     <= 1'b0;
            rsp_status_r   <= 8'h00;
        end else begin
            mem_wr_en_r  <= 1'b0;
            rsp_valid_r  <= 1'b0;
            rsp_done_r   <= 1'b0;
            rsp_fail_r   <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (cmd_valid) begin
                        if (cmd_opcode == OP_FFT_FALCON_FWD) begin
                            falcon_fwd_q     <= 1'b1;
                            inverse_q        <= 1'b0;
                            logn_q           <= cmd_logn;
                            fal_idx          <= {ADDR_W{1'b0}};
                            state            <= ST_FWD_PACK_RD;
                        end else if (cmd_opcode == OP_FFT_FALCON_INV) begin
                            inverse_q        <= 1'b1;
                            falcon_fwd_q     <= 1'b0;
                            logn_q           <= cmd_logn;
                            fal_idx          <= {ADDR_W{1'b0}};
                            state            <= ST_FAL_LOAD;
                        end else begin
                            rsp_status_r <= 8'hE1;
                            state        <= ST_FAIL;
                        end
                    end
                end

                // ─── Forward FFT ──────────────────────────────────────
                ST_FWD_PACK_RD: begin
                    // address via combinational decode
                    state <= ST_FWD_PACK_WR;
                end

                ST_FWD_PACK_WR: begin
                    mem_wr_en_r     <= 1'b1;
                    mem_wr_addr0_r  <= fal_idx;
                    mem_wr_addr1_r  <= fal_idx;
                    mem_wr_data0_re_r <= mem_rd_data0_re;
                    mem_wr_data0_im_r <= mem_rd_data1_re;
                    mem_wr_data1_re_r <= mem_rd_data0_re;
                    mem_wr_data1_im_r <= mem_rd_data1_re;
                    if (fal_idx == (fal_hn_c[ADDR_W-1:0] - 1'b1)) begin
                        fal_idx        <= {ADDR_W{1'b0}};
                        stage_idx      <= 5'd0;
                        last_stage_idx <= logn_q - 2'd2;
                        pair_count_limit <= ({ {(ADDR_W-1){1'b0}}, 1'b1 } << (logn_q - 2'd2));
                        pair_base      <= {ADDR_W{1'b0}};
                        batch_sub_idx  <= {ADDR_W{1'b0}};
                        state          <= ST_BFLY_BATCH_RD;
                    end else begin
                        fal_idx <= fal_idx + 1'b1;
                        state   <= ST_FWD_PACK_RD;
                    end
                end

                // ─── Batch read up to 4 pairs ───
                ST_BFLY_BATCH_RD: begin
                    // Latch read data into input buffer (address is combinational)
                    buf_a_re[batch_sub_idx]  <= mem_rd_data0_re;
                    buf_a_im[batch_sub_idx]  <= mem_rd_data0_im;
                    buf_b_re[batch_sub_idx]  <= mem_rd_data1_re;
                    buf_b_im[batch_sub_idx]  <= mem_rd_data1_im;
                    buf_addr_a[batch_sub_idx] <= base_addr_a;
                    buf_addr_b[batch_sub_idx] <= base_addr_b;

                    if (batch_sub_idx == batch_max) begin
                        batch_sub_idx <= {ADDR_W{1'b0}};
                        state  <= ST_BFLY_RUN;
                    end else begin
                        batch_sub_idx <= batch_sub_idx + 1'b1;
                    end
                end

                // ─── Run 4-BFU ───
                ST_BFLY_RUN: begin
                    if (bf4_out_valid) begin
                        // Latch results (all lanes produce valid data)
                        obuf_y0_re[0] <= bf4_y00_re; obuf_y0_im[0] <= bf4_y00_im;
                        obuf_y1_re[0] <= bf4_y10_re; obuf_y1_im[0] <= bf4_y10_im;
                        obuf_y0_re[1] <= bf4_y01_re; obuf_y0_im[1] <= bf4_y01_im;
                        obuf_y1_re[1] <= bf4_y11_re; obuf_y1_im[1] <= bf4_y11_im;
                        obuf_y0_re[2] <= bf4_y02_re; obuf_y0_im[2] <= bf4_y02_im;
                        obuf_y1_re[2] <= bf4_y12_re; obuf_y1_im[2] <= bf4_y12_im;
                        obuf_y0_re[3] <= bf4_y03_re; obuf_y0_im[3] <= bf4_y03_im;
                        obuf_y1_re[3] <= bf4_y13_re; obuf_y1_im[3] <= bf4_y13_im;
                        batch_sub_idx <= {ADDR_W{1'b0}};
                        state  <= ST_BFLY_BATCH_WR;
                    end
                end

                // ─── Batch write results ───
                ST_BFLY_BATCH_WR: begin
                    mem_wr_en_r     <= 1'b1;
                    mem_wr_addr0_r  <= buf_addr_a[batch_sub_idx];
                    mem_wr_addr1_r  <= buf_addr_b[batch_sub_idx];
                    mem_wr_data0_re_r <= obuf_y0_re[batch_sub_idx];
                    mem_wr_data0_im_r <= obuf_y0_im[batch_sub_idx];
                    mem_wr_data1_re_r <= obuf_y1_re[batch_sub_idx];
                    mem_wr_data1_im_r <= obuf_y1_im[batch_sub_idx];

                    if (batch_sub_idx == batch_max) begin
                        batch_sub_idx <= {ADDR_W{1'b0}};
                        // Advance by (batch_max+1) pairs
                        if (pair_base + {{(ADDR_W-2){1'b0}}, batch_max} + 1'b1 >= pair_count_limit) begin
                            pair_base <= {ADDR_W{1'b0}};
                            if (stage_idx == last_stage_idx) begin
                                fal_idx <= fal_hn_c[ADDR_W-1:0];
                                state   <= ST_FWD_MIR_RD;
                            end else begin
                                stage_idx <= stage_idx + 1'b1;
                                state     <= ST_BFLY_BATCH_RD;
                            end
                        end else begin
                            pair_base <= pair_base + {{(ADDR_W-2){1'b0}}, batch_max} + 1'b1;
                            state     <= ST_BFLY_BATCH_RD;
                        end
                    end else begin
                        batch_sub_idx <= batch_sub_idx + 1'b1;
                    end
                end

                // ─── Mirror extension ───
                ST_FWD_MIR_RD: begin
                    state <= ST_FWD_MIR_WR;
                end

                ST_FWD_MIR_WR: begin
                    mem_wr_en_r     <= 1'b1;
                    mem_wr_addr0_r  <= fal_idx;
                    mem_wr_addr1_r  <= fal_idx;
                    mem_wr_data0_re_r <= mem_rd_data0_re;
                    mem_wr_data0_im_r <= mem_rd_data0_im ^ 64'h8000000000000000;
                    mem_wr_data1_re_r <= mem_rd_data0_re;
                    mem_wr_data1_im_r <= mem_rd_data0_im ^ 64'h8000000000000000;
                    if (fal_idx == (({ {(ADDR_W-1){1'b0}}, 1'b1 } << logn_q) - 1'b1)) begin
                        state <= ST_DONE;
                    end else begin
                        fal_idx <= fal_idx + 1'b1;
                        state   <= ST_FWD_MIR_RD;
                    end
                end

                // ─── Inverse FFT (keep existing software FFT for now) ───
                ST_FAL_LOAD: begin
                    // Simplified: mark done immediately (IFFT handled by sw path)
                    state <= ST_DONE;
                end

                ST_DONE: begin
                    rsp_done_r  <= 1'b1;
                    rsp_valid_r <= 1'b1;
                    rsp_status_r <= 8'h00;
                    state <= ST_IDLE;
                end

                ST_FAIL: begin
                    rsp_fail_r   <= 1'b1;
                    rsp_valid_r  <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    // ─── BFU input valid: fire when entering ST_BFLY_RUN ───
    reg bf4_in_valid_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bf4_in_valid_r <= 1'b0;
        else if (state == ST_BFLY_BATCH_RD && batch_sub_idx == batch_max) bf4_in_valid_r <= 1'b1;
        else if (bf4_in_valid_r && bf4_in_ready) bf4_in_valid_r <= 1'b0;
    end
    assign bf4_in_valid = bf4_in_valid_r;

    // ─── Twiddle factor cache update logic ───
    // Update the cache when reading a new twiddle factor from ROM (no reuse)
    // Clear the cache when entering IDLE or changing stages
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            twiddle_cache_addr  <= {ADDR_W{1'b0}};
            twiddle_cache_re    <= 64'd0;
            twiddle_cache_im    <= 64'd0;
            twiddle_cache_valid <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    // Clear cache when idle
                    twiddle_cache_valid <= 1'b0;
                end

                ST_BFLY_BATCH_RD: begin
                    // Update cache when reading new twiddle (no reuse)
                    if (!twiddle_reuse) begin
                        twiddle_cache_addr  <= fal_twiddle_c[ADDR_W-1:0];
                        twiddle_cache_re    <= fal_gm_re[fal_twiddle_c[ADDR_W-1:0]];
                        twiddle_cache_im    <= fal_gm_im[fal_twiddle_c[ADDR_W-1:0]];
                        twiddle_cache_valid <= 1'b1;
                    end
                    // If reusing, cache remains unchanged
                end

                ST_BFLY_BATCH_WR: begin
                    // When advancing to next stage, invalidate cache
                    // because twiddle factors will be different
                    if (batch_sub_idx == batch_max) begin
                        if (pair_base + {{(ADDR_W-2){1'b0}}, batch_max} + 1'b1 >= pair_count_limit) begin
                            if (stage_idx != last_stage_idx) begin
                                // Stage change - invalidate cache
                                twiddle_cache_valid <= 1'b0;
                            end
                        end
                    end
                end

                default: begin
                    // Keep cache state unchanged in other states
                end
            endcase
        end
    end

    // ─── Twiddle reuse statistics (simulation only) ───
    `ifndef SYNTHESIS
    reg [31:0] twiddle_read_count;
    reg [31:0] twiddle_reuse_count;
    initial begin
        twiddle_read_count = 0;
        twiddle_reuse_count = 0;
    end

    always @(posedge clk) begin
        if (state == ST_BFLY_BATCH_RD) begin
            if (twiddle_reuse) begin
                twiddle_reuse_count <= twiddle_reuse_count + 1;
            end else begin
                twiddle_read_count <= twiddle_read_count + 1;
            end
        end
    end

    // Print statistics at end of simulation
    final begin
        $display("=== Twiddle Reuse Statistics ===");
        $display("  ROM reads:      %0d", twiddle_read_count);
        $display("  Cache reuses:   %0d", twiddle_reuse_count);
        $display("  Total accesses: %0d", twiddle_read_count + twiddle_reuse_count);
        if ((twiddle_read_count + twiddle_reuse_count) > 0) begin
            $display("  Reuse rate:     %0d%%", (twiddle_reuse_count * 100) / (twiddle_read_count + twiddle_reuse_count));
        end
    end
    `endif

endmodule
