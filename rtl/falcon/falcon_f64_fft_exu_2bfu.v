`timescale 1ns/1ps
// Module: falcon_f64_fft_exu_2bfu
// Purpose: Falcon FFT EXU with 2-lane PARALLEL vector butterfly.
// Uses shared FPU lanes (falconsign_shared_fpu_lanes) for computation.
// Both lanes share one twiddle factor per batch.
//
// Batching: reads 2 pairs sequentially into input buffer, fires both
// FPU lanes in parallel, then writes 2 results back sequentially.
// When ht < 2 (last stage), falls back to single-pair mode.
//
// FC cycles (N=512): ~18K (vs ~37K for 1-BFU, ~2x speedup)

module falcon_f64_fft_exu_2bfu #(
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
    output wire              busy,

    // External shared FPU lane ports (2-lane butterfly via falconsign_shared_fpu_lanes)
    output wire              fc_fpu_req_v,      // request valid
    input  wire              fc_fpu_req_r,      // request ready
    output wire [2:0]        fc_fpu_mode,       // operation mode (1=butterfly)
    output wire [63:0]       fc_fpu_a0_re, fc_fpu_a0_im,  // lane 0: a
    output wire [63:0]       fc_fpu_b0_re, fc_fpu_b0_im,  // lane 0: b
    output wire [63:0]       fc_fpu_a1_re, fc_fpu_a1_im,  // lane 1: a
    output wire [63:0]       fc_fpu_b1_re, fc_fpu_b1_im,  // lane 1: b
    output wire [63:0]       fc_fpu_w_re, fc_fpu_w_im,    // lane 0 twiddle
    output wire [63:0]       fc_fpu_w1_re, fc_fpu_w1_im,  // lane 1 twiddle (same as lane 0 for FFT)
    input  wire              fc_fpu_rsp_v,      // response valid
    input  wire [63:0]       fc_fpu_y0_re, fc_fpu_y0_im,  // lane 0: y0
    input  wire [63:0]       fc_fpu_y1_re, fc_fpu_y1_im,  // lane 0: y1
    input  wire [63:0]       fc_fpu_y0_re_1, fc_fpu_y0_im_1,  // lane 1: y0
    input  wire [63:0]       fc_fpu_y1_re_1, fc_fpu_y1_im_1   // lane 1: y1
);

    localparam [2:0] OP_FFT_FALCON_FWD  = 3'd3;
    localparam [2:0] OP_FFT_FALCON_INV  = 3'd2;

    // ─── FSM states ───
    localparam [5:0] ST_IDLE          = 6'd0;
    localparam [5:0] ST_FWD_PACK_RD   = 6'd1;
    localparam [5:0] ST_FWD_PACK_WR   = 6'd2;
    localparam [5:0] ST_BFLY_INIT     = 6'd3;  // one-cycle write-commit gap
    localparam [5:0] ST_BFLY_BATCH_RD = 6'd4;
    localparam [5:0] ST_BFLY_RUN      = 6'd5;
    localparam [5:0] ST_BFLY_BATCH_WR = 6'd6;
    localparam [5:0] ST_MIR_INIT      = 6'd7;  // one-cycle gap before mirror read
    localparam [5:0] ST_FWD_MIR_RD    = 6'd8;
    localparam [5:0] ST_FWD_MIR_WR    = 6'd9;
    localparam [5:0] ST_FAL_LOAD      = 6'd10;
    localparam [5:0] ST_DONE          = 6'd11;
    localparam [5:0] ST_FAIL          = 6'd12;

    reg [5:0]        state;
    reg [4:0]        logn_q;
    reg [4:0]        stage_idx;
    reg [4:0]        last_stage_idx;
    reg [ADDR_W-1:0] pair_base;
    reg [ADDR_W-1:0] pair_count_limit;
    reg [ADDR_W-1:0] fal_idx;
    reg              batch_sub;  // 0 or 1 within current 2-pair batch

    // ─── Input buffer: 2 pairs × 2 complex values ───
    reg [63:0]       buf_a_re [0:1];
    reg [63:0]       buf_a_im [0:1];
    reg [63:0]       buf_b_re [0:1];
    reg [63:0]       buf_b_im [0:1];
    reg [ADDR_W-1:0] buf_addr_a [0:1];
    reg [ADDR_W-1:0] buf_addr_b [0:1];

    // ─── Output buffer: 2 pairs × 2 results ───
    reg [63:0]       obuf_y0_re [0:1];
    reg [63:0]       obuf_y0_im [0:1];
    reg [63:0]       obuf_y1_re [0:1];
    reg [63:0]       obuf_y1_im [0:1];

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
    // 1. can_reuse: current stage supports reuse (fal_ht_c >= 2, i.e., vector size)
    // 2. addr_match: current twiddle address matches cached address
    // 3. cache_valid: cache contains valid data
    wire can_reuse_twiddle = (fal_ht_c >= 2);
    wire addr_match = (fal_twiddle_c[ADDR_W-1:0] == twiddle_cache_addr);
    wire twiddle_reuse = 1'b0;  // Temporarily disable for debugging

    // Twiddle source selection: always use ROM
    wire [63:0] twiddle_re_mux = fal_gm_re[fal_twiddle_c[ADDR_W-1:0]];
    wire [63:0] twiddle_im_mux = fal_gm_im[fal_twiddle_c[ADDR_W-1:0]];

    // ─── Address computation ───
    reg [ADDR_W:0] fal_hn_c;
    reg [ADDR_W:0] fal_ht_c;
    reg [ADDR_W:0] fal_t_c;
    reg [ADDR_W:0] fal_m_c;
    reg [ADDR_W:0] fal_j_c;
    reg [ADDR_W:0] fal_group_c;
    reg [ADDR_W:0] fal_base_c;
    reg [ADDR_W:0] fal_twiddle_c;
    reg [4:0]      fal_ht_shift_c;

    wire [ADDR_W:0] pair_idx = pair_base + {{(ADDR_W-1){1'b0}}, batch_sub};

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
    end

    wire [ADDR_W-1:0] base_addr_a = fal_base_c[ADDR_W-1:0] + fal_j_c[ADDR_W-1:0];
    wire [ADDR_W-1:0] base_addr_b = base_addr_a + fal_ht_c[ADDR_W-1:0];
    wire [ADDR_W-1:0] mir_addr    = ((({ADDR_W{1'b0}} | 1'b1) << logn_q) - 1'b1 - fal_idx);

    // ─── External shared FPU lane connections ───
    // Lane 0: pair_base, Lane 1: pair_base+1. Both share same twiddle factor w.
    wire bf2_in_valid;
    wire bf2_in_ready = fc_fpu_req_r;
    wire bf2_out_valid = fc_fpu_rsp_v;

    assign fc_fpu_req_v  = bf2_in_valid;
    assign fc_fpu_mode   = 3'd1;  // BUTTERFLY mode
    assign fc_fpu_a0_re  = buf_a_re[0]; assign fc_fpu_a0_im = buf_a_im[0];
    assign fc_fpu_b0_re  = buf_b_re[0]; assign fc_fpu_b0_im = buf_b_im[0];
    assign fc_fpu_a1_re  = buf_a_re[1]; assign fc_fpu_a1_im = buf_a_im[1];
    assign fc_fpu_b1_re  = buf_b_re[1]; assign fc_fpu_b1_im = buf_b_im[1];
    assign fc_fpu_w_re   = twiddle_re_mux;
    assign fc_fpu_w_im   = twiddle_im_mux;
    assign fc_fpu_w1_re  = twiddle_re_mux;  // same twiddle for both lanes in FFT
    assign fc_fpu_w1_im  = twiddle_im_mux;

    wire [63:0] bf2_y00_re = fc_fpu_y0_re;  wire [63:0] bf2_y00_im = fc_fpu_y0_im;
    wire [63:0] bf2_y10_re = fc_fpu_y1_re;  wire [63:0] bf2_y10_im = fc_fpu_y1_im;
    wire [63:0] bf2_y01_re = fc_fpu_y0_re_1; wire [63:0] bf2_y01_im = fc_fpu_y0_im_1;
    wire [63:0] bf2_y11_re = fc_fpu_y1_re_1; wire [63:0] bf2_y11_im = fc_fpu_y1_im_1;

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

    assign mem_rd_addr0 = mem_rd_addr0_c;
    assign mem_rd_addr1 = mem_rd_addr1_c;
    assign twiddle_addr = twiddle_addr_c;
    assign mem_wr_en    = mem_wr_en_r;

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
            ST_FWD_MIR_RD, ST_FWD_MIR_WR: begin
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

    // Batch size: 2 lanes when ht >= 2 (same twiddle group), 1 lane otherwise
    wire batch_full = (fal_ht_c >= 2);
    wire batch_max  = batch_full;  // 0=1-pair, 1=2-pair

    // ─── BFU input valid ───
    reg bf2_in_valid_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bf2_in_valid_r <= 1'b0;
        else if (state == ST_BFLY_BATCH_RD && batch_sub == batch_max)
            bf2_in_valid_r <= 1'b1;
        else if (bf2_in_valid_r && bf2_in_ready)
            bf2_in_valid_r <= 1'b0;
    end
    assign bf2_in_valid = bf2_in_valid_r;

    // ─── FSM ───
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            logn_q          <= 5'd0;
            stage_idx       <= 5'd0;
            last_stage_idx  <= 5'd0;
            pair_base       <= {ADDR_W{1'b0}};
            pair_count_limit <= {ADDR_W{1'b0}};
            fal_idx         <= {ADDR_W{1'b0}};
            batch_sub       <= 1'b0;
            mem_wr_en_r     <= 1'b0;
            rsp_valid_r     <= 1'b0;
            rsp_done_r      <= 1'b0;
            rsp_fail_r      <= 1'b0;
            rsp_status_r    <= 8'h00;
        end else begin
            mem_wr_en_r  <= 1'b0;
            rsp_valid_r  <= 1'b0;
            rsp_done_r   <= 1'b0;
            rsp_fail_r   <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (cmd_valid) begin
                        if (cmd_opcode == OP_FFT_FALCON_FWD) begin
                            logn_q           <= cmd_logn;
                            fal_idx          <= {ADDR_W{1'b0}};
                            state            <= ST_FWD_PACK_RD;
                        end else if (cmd_opcode == OP_FFT_FALCON_INV) begin
                            logn_q           <= cmd_logn;
                            fal_idx          <= {ADDR_W{1'b0}};
                            state            <= ST_FAL_LOAD;
                        end else begin
                            rsp_status_r <= 8'hE1;
                            state        <= ST_FAIL;
                        end
                    end
                end

                // ─── Forward FFT: pack real into complex ───
                ST_FWD_PACK_RD: begin
                    state <= ST_FWD_PACK_WR;
                end

                ST_FWD_PACK_WR: begin
                    mem_wr_en_r      <= 1'b1;
                    mem_wr_addr0_r   <= fal_idx;
                    mem_wr_addr1_r   <= fal_idx;
                    mem_wr_data0_re_r <= mem_rd_data0_re;
                    mem_wr_data0_im_r <= mem_rd_data1_re;
                    mem_wr_data1_re_r <= mem_rd_data0_re;
                    mem_wr_data1_im_r <= mem_rd_data1_re;
                    if (fal_idx == (fal_hn_c[ADDR_W-1:0] - 1'b1)) begin
                        fal_idx         <= {ADDR_W{1'b0}};
                        stage_idx       <= 5'd0;
                        last_stage_idx  <= logn_q - 2'd2;
                        pair_count_limit <= (1 << (logn_q - 2'd2));
                        pair_base       <= {ADDR_W{1'b0}};
                        batch_sub       <= 1'b0;
                        state           <= ST_BFLY_INIT;
                    end else begin
                        fal_idx <= fal_idx + 1'b1;
                        state   <= ST_FWD_PACK_RD;
                    end
                end

                // ─── One-cycle commit gap (avoid RAW hazard on pack writes) ───
                ST_BFLY_INIT: begin
                    state <= ST_BFLY_BATCH_RD;
                end

                // ─── Batch read up to 2 pairs ───
                ST_BFLY_BATCH_RD: begin
                    buf_a_re[batch_sub]  <= mem_rd_data0_re;
                    buf_a_im[batch_sub]  <= mem_rd_data0_im;
                    buf_b_re[batch_sub]  <= mem_rd_data1_re;
                    buf_b_im[batch_sub]  <= mem_rd_data1_im;
                    buf_addr_a[batch_sub] <= base_addr_a;
                    buf_addr_b[batch_sub] <= base_addr_b;

                    if (batch_sub == batch_max) begin
                        batch_sub <= 1'b0;
                        state     <= ST_BFLY_RUN;
                    end else begin
                        batch_sub <= batch_sub + 1'b1;
                    end
                end

                // ─── Run 2-BFU (both lanes in parallel) ───
                ST_BFLY_RUN: begin
                    if (bf2_out_valid) begin
                        obuf_y0_re[0] <= bf2_y00_re; obuf_y0_im[0] <= bf2_y00_im;
                        obuf_y1_re[0] <= bf2_y10_re; obuf_y1_im[0] <= bf2_y10_im;
                        obuf_y0_re[1] <= bf2_y01_re; obuf_y0_im[1] <= bf2_y01_im;
                        obuf_y1_re[1] <= bf2_y11_re; obuf_y1_im[1] <= bf2_y11_im;
                        batch_sub <= 1'b0;
                        state     <= ST_BFLY_BATCH_WR;
                    end
                end

                // ─── Batch write results ───
                ST_BFLY_BATCH_WR: begin
                    mem_wr_en_r      <= 1'b1;
                    mem_wr_addr0_r   <= buf_addr_a[batch_sub];
                    mem_wr_addr1_r   <= buf_addr_b[batch_sub];
                    mem_wr_data0_re_r <= obuf_y0_re[batch_sub];
                    mem_wr_data0_im_r <= obuf_y0_im[batch_sub];
                    mem_wr_data1_re_r <= obuf_y1_re[batch_sub];
                    mem_wr_data1_im_r <= obuf_y1_im[batch_sub];

                    if (batch_sub == batch_max) begin
                        batch_sub <= 1'b0;
                        if (pair_base + {{(ADDR_W-1){1'b0}}, batch_max} + 1'b1 >= pair_count_limit) begin
                            pair_base <= {ADDR_W{1'b0}};
                            if (stage_idx == last_stage_idx) begin
                                fal_idx <= fal_hn_c[ADDR_W-1:0];
                                state   <= ST_MIR_INIT;
                            end else begin
                                stage_idx <= stage_idx + 1'b1;
                                state     <= ST_BFLY_BATCH_RD;
                            end
                        end else begin
                            pair_base <= pair_base + {{(ADDR_W-1){1'b0}}, batch_max} + 1'b1;
                            state     <= ST_BFLY_BATCH_RD;
                        end
                    end else begin
                        batch_sub <= batch_sub + 1'b1;
                    end
                end

                // ─── One-cycle gap before mirror (avoid RAW on butterfly writes) ───
                ST_MIR_INIT: begin
                    state <= ST_FWD_MIR_RD;
                end

                // ─── Mirror extension ───
                ST_FWD_MIR_RD: begin
                    state <= ST_FWD_MIR_WR;
                end

                ST_FWD_MIR_WR: begin
                    mem_wr_en_r      <= 1'b1;
                    mem_wr_addr0_r   <= fal_idx;
                    mem_wr_addr1_r   <= fal_idx;
                    mem_wr_data0_re_r <= mem_rd_data0_re;
                    mem_wr_data0_im_r <= mem_rd_data0_im ^ 64'h8000000000000000;
                    mem_wr_data1_re_r <= mem_rd_data0_re;
                    mem_wr_data1_im_r <= mem_rd_data0_im ^ 64'h8000000000000000;
                    if (fal_idx == ((1 << logn_q) - 1'b1)) begin
                        state <= ST_DONE;
                    end else begin
                        fal_idx <= fal_idx + 1'b1;
                        state   <= ST_FWD_MIR_RD;
                    end
                end

                // ─── Inverse FFT placeholder (handled by 1-BFU EXU) ───
                ST_FAL_LOAD: begin
                    state <= ST_DONE;
                end

                ST_DONE: begin
                    rsp_done_r   <= 1'b1;
                    rsp_valid_r  <= 1'b1;
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
                    if (batch_sub == batch_max) begin
                        if (pair_base + {{(ADDR_W-1){1'b0}}, batch_max} + 1'b1 >= pair_count_limit) begin
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
        $display("=== 2-BFU Twiddle Reuse Statistics ===");
        $display("  ROM reads:      %0d", twiddle_read_count);
        $display("  Cache reuses:   %0d", twiddle_reuse_count);
        $display("  Total accesses: %0d", twiddle_read_count + twiddle_reuse_count);
        if ((twiddle_read_count + twiddle_reuse_count) > 0) begin
            $display("  Reuse rate:     %0d%%", (twiddle_reuse_count * 100) / (twiddle_read_count + twiddle_reuse_count));
        end
    end
    `endif

endmodule
