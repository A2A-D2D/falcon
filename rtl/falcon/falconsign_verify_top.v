`timescale 1ns/1ps
// Module: falconsign_verify_top
// Purpose: top-level Falcon verify engine. It decodes the signature, hashes
// the message point, performs NTT-domain reconstruction, and checks the norm.
//
// FalconSign Verify Top
//
// Verifies a Falcon-512 signature against a message and public key.
//
// Verify flow (per Falcon spec):
//   1. Decode signature → nonce (40 bytes) + s2 polynomial (512 int16)
//   2. c = HashToPoint(SHAKE256(nonce || message))
//   3. Compute s1 = c - s2*h mod Q via NTT
//   4. Check ||s1||^2 + ||s2||^2 <= 34034726 → accept / reject
//
// Bus interface: same register map as sign top.
//   REG_CR   (0x0000): bit[0]=start
//   REG_SR   (0x0004): read-only status
//   REG_CFG  (0x0008): bit[0]=force_accept
//   REG_MEM_HI(0x000C): upper address bits for memory access
//   REG_MSG_LEN(0x0010): message length in bytes
//
// Memory layout:
//   VERIFY_SIG_BASE   = 0      raw signature bytes (max ~800 bytes)
//   VERIFY_NONCE_BASE = 128    decoded nonce (2 x 256-bit words)
//   VERIFY_MSG_BASE   = 256    message bytes
//   NTT_H_WORK        = 4352   h NTT work area (32 words)
//   NTT_S2_WORK       = 4384   s2 NTT work area (32 words)
//   SIG_S2_BASE       = 6912   decoded s2 (32 x 256-bit, 16 int16 each)
//   C_INT_BASE        = 7424   c polynomial (32 x 256-bit, 16 int16 each)
//   H_BASE            = 7456   h public key (32 x 256-bit, raw mod-Q int16)
//   S1_BASE           = 7488   recomputed s1 output (32 x 256-bit)

module falconsign_verify_top #(
    parameter ADDR_W  = 13,
    parameter MEM_DEPTH = 8192
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        bus_cs,
    input  wire        bus_wr,
    input  wire [15:0] bus_addr,
    input  wire [31:0] bus_wdata,
    output reg  [31:0] bus_rdata,
    output reg         bus_ready,
    output reg         bus_irq,
    output wire        busy,
    output reg         done,
    output reg         fail,
    output reg  [7:0]  status
);

    // ─── Register addresses ───
    localparam [15:0] REG_CR      = 16'h0000;
    localparam [15:0] REG_SR      = 16'h0004;
    localparam [15:0] REG_CFG     = 16'h0008;
    localparam [15:0] REG_MEM_HI  = 16'h000C;
    localparam [15:0] REG_MSG_LEN = 16'h0010;

    // ─── Memory layout ───
    localparam [ADDR_W-1:0] VERIFY_SIG_BASE   = {ADDR_W{1'b0}};
    localparam [ADDR_W-1:0] VERIFY_NONCE_BASE = {{(ADDR_W-8){1'b0}}, 8'd128};
    localparam [ADDR_W-1:0] VERIFY_MSG_BASE   = {{(ADDR_W-9){1'b0}}, 9'd256};
    localparam [ADDR_W-1:0] NTT_H_WORK        = 13'd4352;
    localparam [ADDR_W-1:0] NTT_S2_WORK       = 13'd4384;
    localparam [ADDR_W-1:0] SIG_S2_BASE       = 13'd6912;
    localparam [ADDR_W-1:0] C_INT_BASE        = 13'd7424;
    localparam [ADDR_W-1:0] H_BASE            = 13'd7456;
    localparam [ADDR_W-1:0] S1_BASE           = 13'd7488;

    localparam [63:0] FALCON512_BOUND_SQ = 64'd34034726;
    localparam [15:0] MSG_WORD_COUNT     = 16'd32;  // fixed 32-byte test message for now

    // ─── FSM states ───
    localparam [3:0] SI = 4'd0;
    localparam [3:0] DS = 4'd1;
    localparam [3:0] SH = 4'd2;
    localparam [3:0] HP = 4'd3;
    localparam [3:0] N1 = 4'd4;
    localparam [3:0] RC = 4'd5;
    localparam [3:0] VF = 4'd6;
    localparam [3:0] SD = 4'd7;

    reg [3:0] st, sn;
    reg cr_start;
    reg cfg_force_accept;
    reg cfg_bypass_decode;
    reg cfg_start_at_ntt;      // skip SH+HP, start directly at N1 phase
    reg cfg_hash_message_only; // decode s2 but hash the current bring-up message without nonce
    reg [15:0] cfg_msg_len;
    reg [1:0]  mem_addr_hi;

    // ─── Bus pending state ───
    reg          bus_pend;
    reg          bus_pend_wr;
    reg [15:0]   bus_pend_addr;
    reg [31:0]   bus_pend_data;

    // ─── Memory ───
    wire                mem_rd_en;
    wire                mem_wr_en;
    wire [ADDR_W-1:0]   mem_rd_addr;
    wire [ADDR_W-1:0]   mem_wr_addr;
    wire [255:0]        mem_rd_data;
    wire [255:0]        mem_wr_data;
    wire                mem_b_rd_en;
    wire                mem_b_wr_en;
    wire [ADDR_W-1:0]   mem_b_rd_addr;
    wire [ADDR_W-1:0]   mem_b_wr_addr;
    wire [255:0]        mem_b_rd_data;
    wire [255:0]        mem_b_wr_data;

    falconsign_memory #(.ADDR_W(ADDR_W), .DEPTH(MEM_DEPTH)) u_mem (
        .clk(clk), .rst_n(rst_n),
        .rd_en(mem_rd_en), .rd_addr(mem_rd_addr), .rd_data(mem_rd_data),
        .wr_en(mem_wr_en), .wr_addr(mem_wr_addr), .wr_data(mem_wr_data)
    );
    assign mem_b_rd_data = mem_rd_data;

    wire bus_is_reg = (bus_addr == REG_CR) || (bus_addr == REG_SR) ||
                      (bus_addr == REG_CFG) || (bus_addr == REG_MEM_HI) ||
                      (bus_addr == REG_MSG_LEN);
    assign busy           = (st != SI) && (st != SD);

    // ─── Signature Decoder ───
    wire                sd_start;
    wire                sd_ready;
    wire                sd_done;
    wire                sd_fail;
    wire [7:0]          sd_status;
    wire                sd_mem_rd_en;
    wire [ADDR_W-1:0]   sd_mem_rd_addr;
    wire                sd_mem_wr_en;
    wire [ADDR_W-1:0]   sd_mem_wr_addr;
    wire [255:0]        sd_mem_wr_data;

    falconsign_sig_decode #(.ADDR_W(ADDR_W)) u_sig_decode (
        .clk(clk), .rst_n(rst_n),
        .start(sd_start), .start_ready(sd_ready),
        .sig_base(VERIFY_SIG_BASE),
        .nonce_base(VERIFY_NONCE_BASE),
        .s2_base(SIG_S2_BASE),
        .mem_rd_en(sd_mem_rd_en), .mem_rd_addr(sd_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .mem_wr_en(sd_mem_wr_en), .mem_wr_addr(sd_mem_wr_addr),
        .mem_wr_data(sd_mem_wr_data),
        .done(sd_done), .fail(sd_fail), .status(sd_status)
    );

    // ─── SHAKE256 ───
    wire        shake_start;
    wire        shake_ready;
    reg         shake_absorb;
    reg  [63:0] shake_din;
    reg         shake_din_last;
    reg  [2:0]  shake_din_last_bytes;
    wire        shake_dout_valid;
    wire [63:0] shake_dout;
    wire        shake_fifo_wr_ready;
    wire        shake_fifo_rd_valid;
    wire [63:0] shake_fifo_rd_data;

    falconsign_shake256 u_shake(
        .clk(clk), .rst_n(rst_n),
        .start(shake_start), .ready(shake_ready),
        .absorb(shake_absorb),
        .din(shake_din), .din_last(shake_din_last),
        .din_last_bytes(shake_din_last_bytes),
        .dout_ready(shake_fifo_wr_ready),
        .dout_valid(shake_dout_valid), .dout(shake_dout)
    );

    falconsign_word_fifo #(.WIDTH(64), .DEPTH(16), .ADDR_W(4)) u_shake_fifo (
        .clk(clk), .rst_n(rst_n),
        .wr_valid(shake_dout_valid), .wr_ready(shake_fifo_wr_ready),
        .wr_data(shake_dout),
        .rd_valid(shake_fifo_rd_valid), .rd_ready(htp_hash_ready),
        .rd_data(shake_fifo_rd_data)
    );

    // ─── HashToPoint ───
    wire        htp_start;
    wire        htp_ready;
    wire [63:0] htp_hash_word;
    wire        htp_hash_valid;
    wire        htp_hash_ready;
    wire [15:0] htp_coeff;
    wire        htp_coeff_valid;

    assign htp_hash_word  = shake_fifo_rd_data;
    assign htp_hash_valid = shake_fifo_rd_valid;

    falconsign_hash_to_point #(.N(512)) u_htp(
        .clk(clk), .rst_n(rst_n),
        .start(htp_start), .ready(htp_ready),
        .hash_word(htp_hash_word), .hash_valid(htp_hash_valid),
        .hash_ready(htp_hash_ready),
        .coeff(htp_coeff), .coeff_valid(htp_coeff_valid)
    );

    // ─── HP coefficient packing → C_INT_BASE ───
    reg  [3:0]         hp_coeff_cnt;
    reg  [255:0]       hp_coeff_buf;
    reg                hp_wr_en;
    reg                hp_wr_pending;
    reg  [ADDR_W-1:0]  hp_wr_addr;
    reg  [255:0]       hp_wr_data;
    reg                hp_done;

    // ─── SHAKE feeder sub-FSM ───
    localparam [2:0] SH_F_IDLE     = 3'd0;
    localparam [2:0] SH_F_RD       = 3'd1;  // issue Port B read
    localparam [2:0] SH_F_WAIT     = 3'd2;  // wait 1 cycle for reg read
    localparam [2:0] SH_F_FEED      = 3'd3;  // drive 64-bit word onto SHAKE input
    localparam [2:0] SH_F_LAST_WAIT = 3'd4;  // wait for final permute
    localparam [2:0] SH_F_PULSE     = 3'd5;  // pulse SHAKE absorb after data is stable

    reg        sh_done_f;       // asserted when SHAKE feeding+permutation complete
    reg [2:0]  sh_f_state;
    reg [15:0] sh_word_idx;     // which total 64-bit word (0..total-1)
    reg [15:0] sh_total_words;  // total 64-bit words to feed
    reg [2:0]  sh_sub_idx;      // which 64-bit subword of 256-bit word (0..3)
    reg [ADDR_W-1:0] sh_buf_addr;  // current 256-bit word address to read
    reg [255:0] sh_buf_data;    // cached 256-bit word
    reg [15:0] sh_msg_bytes_remain; // remaining nonce+msg bytes to feed
    reg        sh_seen_busy;    // set after SHAKE leaves ready during final permute

    // ─── NTT EXU ───
    wire                ntt_start;
    wire                ntt_ready;
    wire                ntt_done;
    wire                ntt_fail;
    wire [7:0]          ntt_status;
    wire                ntt_mem_rd_en;
    wire                ntt_mem_wr_en;
    wire [ADDR_W-1:0]   ntt_mem_rd_addr;
    wire [ADDR_W-1:0]   ntt_mem_wr_addr;
    wire [255:0]        ntt_mem_wr_data;
    wire [8:0]          ntt_twiddle_rom_addr;
    wire [13:0]         ntt_twiddle_rom_data;
    wire [9:0]          ntt_psi_rom_addr;
    wire [13:0]         ntt_psi_rom_data;

    falconsign_ntt_twiddle_rom #(.ADDR_W(9)) u_ntt_tw (
        .clk(clk), .addr(ntt_twiddle_rom_addr), .data(ntt_twiddle_rom_data));

    falconsign_ntt_psi_rom #(.ADDR_W(10)) u_ntt_psi (
        .clk(clk), .addr(ntt_psi_rom_addr), .data(ntt_psi_rom_data));

    falconsign_ntt_exu #(.ADDR_W(ADDR_W)) u_ntt (
        .clk(clk), .rst_n(rst_n),
        .start(ntt_start), .start_ready(ntt_ready),
        .done(ntt_done), .fail(ntt_fail), .status(ntt_status),
        .cfg_h_base(H_BASE),
        .cfg_h_work_base(NTT_H_WORK),
        .cfg_s2_base(SIG_S2_BASE),
        .cfg_s2_work_base(NTT_S2_WORK),
        .cfg_c_base(C_INT_BASE),
        .cfg_dst_base(S1_BASE),
        .mem_rd_en(ntt_mem_rd_en), .mem_rd_addr(ntt_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .mem_wr_en(ntt_mem_wr_en), .mem_wr_addr(ntt_mem_wr_addr),
        .mem_wr_data(ntt_mem_wr_data),
        .twiddle_rom_addr(ntt_twiddle_rom_addr),
        .twiddle_rom_data(ntt_twiddle_rom_data),
        .psi_rom_addr(ntt_psi_rom_addr), .psi_rom_data(ntt_psi_rom_data)
    );

    // ─── Norm Check ───
    wire                norm_start;
    wire                norm_start_ready;
    wire                norm_done;
    wire                norm_accept;
    wire                norm_fail;
    wire [7:0]          norm_status;
    wire                norm_mem_rd_en;
    wire [ADDR_W-1:0]   norm_mem_rd_addr;
    wire [63:0]         norm_sq;

    falconsign_norm_i16_sig_check #(.ADDR_W(ADDR_W)) u_norm (
        .clk(clk), .rst_n(rst_n),
        .start(norm_start), .start_ready(norm_start_ready),
        .s2_base(SIG_S2_BASE), .s1_base(S1_BASE),
        .word_count({{(ADDR_W-6){1'b0}}, 6'd32}),
        .bound_sq(FALCON512_BOUND_SQ),
        .mem_rd_en(norm_mem_rd_en), .mem_rd_addr(norm_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .done(norm_done), .accept(norm_accept),
        .fail(norm_fail), .status(norm_status), .norm_sq(norm_sq)
    );

    // ─── Port B mux ───
    wire use_sd_portb   = (st == DS);
    wire use_sh_portb   = (st == SH) && (sh_f_state != SH_F_IDLE);
    wire use_ntt_portb  = (st == N1);
    wire use_norm_portb = (st == RC);

    assign mem_b_rd_en   = use_sd_portb   ? sd_mem_rd_en   :
                            use_sh_portb   ? (sh_f_state == SH_F_RD) :
                            use_ntt_portb  ? ntt_mem_rd_en  :
                            use_norm_portb ? norm_mem_rd_en : 1'b0;
    assign mem_b_rd_addr = use_sd_portb   ? sd_mem_rd_addr   :
                            use_sh_portb   ? sh_buf_addr      :
                            use_ntt_portb  ? ntt_mem_rd_addr  :
                            use_norm_portb ? norm_mem_rd_addr : {ADDR_W{1'b0}};
    assign mem_b_wr_en   = use_sd_portb   ? sd_mem_wr_en   :
                            use_ntt_portb  ? ntt_mem_wr_en  :
                            use_norm_portb ? 1'b0           : hp_wr_en;
    assign mem_b_wr_addr = use_sd_portb   ? sd_mem_wr_addr   :
                            use_ntt_portb  ? ntt_mem_wr_addr  : hp_wr_addr;
    assign mem_b_wr_data = use_sd_portb   ? sd_mem_wr_data   :
                            use_ntt_portb  ? ntt_mem_wr_data  : hp_wr_data;

    localparam [1:0] BUSM_IDLE  = 2'd0;
    localparam [1:0] BUSM_READ  = 2'd1;
    localparam [1:0] BUSM_CAP   = 2'd2;
    localparam [1:0] BUSM_WRITE = 2'd3;
    reg [1:0]        bus_mem_state;
    reg              bus_mem_wr_q;
    reg [ADDR_W-1:0] bus_mem_word_addr_q;
    reg [2:0]        bus_mem_lane_q;
    reg [31:0]       bus_mem_wdata_q;
    reg [255:0]      bus_mem_wr_word_q;

    wire             bus_mem_rd_en = (bus_mem_state == BUSM_READ);
    wire             bus_mem_wr_en = (bus_mem_state == BUSM_WRITE);
    wire [255:0]     bus_mem_lane_mask = ({224'd0, 32'hFFFF_FFFF} << (bus_mem_lane_q * 32));
    wire [255:0]     bus_mem_lane_data = ({224'd0, bus_mem_wdata_q} << (bus_mem_lane_q * 32));

    assign mem_rd_en   = bus_mem_rd_en ? 1'b1 : mem_b_rd_en;
    assign mem_rd_addr = bus_mem_rd_en ? bus_mem_word_addr_q : mem_b_rd_addr;
    assign mem_wr_en   = bus_mem_wr_en ? 1'b1 : mem_b_wr_en;
    assign mem_wr_addr = bus_mem_wr_en ? bus_mem_word_addr_q : mem_b_wr_addr;
    assign mem_wr_data = bus_mem_wr_en ? bus_mem_wr_word_q : mem_b_wr_data;

    // ─── Phase control signals ───
    assign sd_start    = (st != DS) && (sn == DS);
    assign shake_start = (st != SH) && (sn == SH);
    assign htp_start   = (st != HP) && (sn == HP);
    assign ntt_start   = (st != N1) && (sn == N1);
    assign norm_start  = (st != RC) && (sn == RC);

    // Hardcoded test message bytes are FALCON_SIGN_TEST_MSG_V1.0_______.
    // Keccak absorbs byte 0 from din[7:0], so each 64-bit word is byte-reversed.
    localparam [63:0] TEST_MSG_W0 = 64'h535F4E4F434C4146;
    localparam [63:0] TEST_MSG_W1 = 64'h545345545F4E4749;
    localparam [63:0] TEST_MSG_W2 = 64'h2E31565F47534D5F;
    localparam [63:0] TEST_MSG_W3 = 64'h5F5F5F5F5F5F5F30;

    // ─── Bus & FSM ───
    // Verify phase FSM: load/decode signature, hash nonce/message, run NTT
    // reconstruction, then check the Falcon norm bound.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= SI; cr_start <= 1'b0; bus_rdata <= 32'd0; bus_ready <= 1'b0;
            bus_irq <= 1'b0; done <= 1'b0; fail <= 1'b0; status <= 8'h00;
            bus_pend <= 1'b0; bus_pend_wr <= 1'b0;
            bus_pend_addr <= 16'd0; bus_pend_data <= 32'd0;
            bus_mem_state <= BUSM_IDLE;
            bus_mem_wr_q <= 1'b0;
            bus_mem_word_addr_q <= {ADDR_W{1'b0}};
            bus_mem_lane_q <= 3'd0;
            bus_mem_wdata_q <= 32'd0;
            bus_mem_wr_word_q <= 256'd0;
            cfg_force_accept <= 1'b0; cfg_bypass_decode <= 1'b0;
            cfg_start_at_ntt <= 1'b0; cfg_hash_message_only <= 1'b0;
            cfg_msg_len <= 16'd32; mem_addr_hi <= 2'd0;

            // HP packing
            hp_coeff_cnt <= 4'd0; hp_coeff_buf <= 256'd0;
            hp_wr_en <= 1'b0; hp_wr_addr <= C_INT_BASE; hp_wr_data <= 256'd0;
            hp_wr_pending <= 1'b0;
            hp_done <= 1'b0;

            // SHAKE feeder
            sh_done_f <= 1'b0; sh_f_state <= SH_F_IDLE; sh_word_idx <= 16'd0;
            sh_total_words <= 16'd0; sh_sub_idx <= 3'd0;
            sh_buf_addr <= {ADDR_W{1'b0}}; sh_buf_data <= 256'd0;
            sh_msg_bytes_remain <= 16'd0;
            sh_seen_busy <= 1'b0;
            shake_absorb <= 1'b0; shake_din <= 64'd0;
            shake_din_last <= 1'b0; shake_din_last_bytes <= 3'd0;

        end else begin
            st <= sn;
            bus_ready <= 1'b0;
            shake_absorb <= 1'b0;
            hp_wr_en <= 1'b0;
            if (hp_wr_en) begin
                hp_wr_addr <= hp_wr_addr + 1'b1;
            end
            if (hp_wr_pending) begin
                hp_wr_en <= 1'b1;
                hp_wr_pending <= 1'b0;
            end

            // ─── Bus access ───
            if (bus_cs && !bus_pend && (bus_mem_state == BUSM_IDLE)) begin
                if (bus_is_reg) begin
                    bus_ready <= 1'b1;
                    if (bus_wr) begin
                        case (bus_addr)
                            REG_CR: begin
                                cr_start <= bus_wdata[0];
                                if (st == SD) bus_irq <= 1'b0;
                            end
                            REG_CFG: begin
                                cfg_force_accept  <= bus_wdata[0];
                                cfg_bypass_decode <= bus_wdata[1];
                                cfg_start_at_ntt  <= bus_wdata[2];
                                cfg_hash_message_only <= bus_wdata[3];
                            end
                            REG_MEM_HI: mem_addr_hi <= bus_wdata[1:0];
                            REG_MSG_LEN: cfg_msg_len <= bus_wdata[15:0];
                            default: ;
                        endcase
                    end else begin
                        case (bus_addr)
                            REG_CR:     bus_rdata <= {31'd0, cr_start};
                            REG_SR:     bus_rdata <= {16'd0, status, 4'd0, fail, done, bus_irq, busy};
                            REG_CFG:    bus_rdata <= {28'd0, cfg_hash_message_only, cfg_start_at_ntt, cfg_bypass_decode, cfg_force_accept};
                            REG_MEM_HI: bus_rdata <= {30'd0, mem_addr_hi};
                            REG_MSG_LEN: bus_rdata <= {16'd0, cfg_msg_len};
                            default:    bus_rdata <= 32'd0;
                        endcase
                    end
                end else begin
                    if (!busy) begin
                        bus_mem_wr_q <= bus_wr;
                        bus_mem_word_addr_q <= {mem_addr_hi, bus_addr[15:5]};
                        bus_mem_lane_q <= bus_addr[4:2];
                        bus_mem_wdata_q <= bus_wdata;
                        bus_mem_state <= BUSM_READ;
                    end else begin
                        bus_ready <= 1'b1;
                        if (!bus_wr)
                            bus_rdata <= 32'd0;
                    end
                end
            end
            case (bus_mem_state)
                BUSM_READ: begin
                    bus_mem_state <= BUSM_CAP;
                end
                BUSM_CAP: begin
                    if (bus_mem_wr_q) begin
                        bus_mem_wr_word_q <= (mem_rd_data & ~bus_mem_lane_mask) | bus_mem_lane_data;
                        bus_mem_state <= BUSM_WRITE;
                    end else begin
                        bus_rdata <= mem_rd_data[bus_mem_lane_q * 32 +: 32];
                        bus_ready <= 1'b1;
                        bus_mem_state <= BUSM_IDLE;
                    end
                end
                BUSM_WRITE: begin
                    bus_ready <= 1'b1;
                    bus_mem_state <= BUSM_IDLE;
                end
                default: begin
                    bus_mem_state <= BUSM_IDLE;
                end
            endcase

            // Phase transitions
            case (st)
                SI: begin
                    done <= 1'b0; fail <= 1'b0; bus_irq <= 1'b0;
                    if (sn != SI) cr_start <= 1'b0;
                end
                SD: begin
                    done <= 1'b1; bus_irq <= 1'b1;
                    if (bus_cs && bus_wr && bus_addr == REG_CR) bus_irq <= 1'b0;
                end
            endcase

            // ─── DS phase: signature decode (handled by u_sig_decode) ───
            if (st == DS && sn == DS) begin
                if (sd_done) begin
                    if (sd_fail) begin
                        fail   <= 1'b1;
                        status <= sd_status;
                    end
                end
            end

            // ─── SH phase: SHAKE absorb (nonce || message) ───
            if (st == SH && sn == SH) begin
                case (sh_f_state)
                    SH_F_IDLE: begin
                        if (cfg_bypass_decode || cfg_hash_message_only) begin
                            // Bypass mode mirrors the current sign bring-up:
                            // feed the fixed 32-byte test message directly.
                            sh_buf_addr    <= VERIFY_MSG_BASE;
                            sh_total_words <= 16'd4;
                            sh_f_state     <= SH_F_FEED;
                        end else begin
                            // Normal mode: nonce (40 bytes) + message
                            sh_buf_addr  <= VERIFY_NONCE_BASE;
                            sh_total_words <= 16'd5 + (cfg_msg_len >> 3) +
                                              ((cfg_msg_len[2:0] == 3'd0) ? 16'd0 : 16'd1);
                            sh_f_state <= SH_F_RD;
                        end
                        sh_sub_idx   <= 3'd0;
                        sh_word_idx  <= 16'd0;
                        // last byte handling
                        case (cfg_msg_len[2:0])
                            3'd0: shake_din_last_bytes <= 3'd0;
                            3'd1: shake_din_last_bytes <= 3'd7;
                            3'd2: shake_din_last_bytes <= 3'd6;
                            3'd3: shake_din_last_bytes <= 3'd5;
                            3'd4: shake_din_last_bytes <= 3'd4;
                            3'd5: shake_din_last_bytes <= 3'd3;
                            3'd6: shake_din_last_bytes <= 3'd2;
                            default: shake_din_last_bytes <= 3'd1;
                        endcase
                    end

                    SH_F_RD: begin
                        // Port B read is asserted by mem_b_rd_en (mux)
                        sh_f_state <= SH_F_WAIT;
                    end

                    SH_F_WAIT: begin
                        sh_buf_data <= mem_b_rd_data;
                        sh_f_state  <= SH_F_FEED;
                    end

                    SH_F_FEED: begin
                        // Present the next 64-bit word one cycle before the
                        // absorb pulse, so the SHAKE core samples stable data.
                        if (cfg_bypass_decode || cfg_hash_message_only) begin
                            case (sh_word_idx[1:0])
                                2'd0: shake_din <= TEST_MSG_W0;
                                2'd1: shake_din <= TEST_MSG_W1;
                                2'd2: shake_din <= TEST_MSG_W2;
                                default: shake_din <= TEST_MSG_W3;
                            endcase
                        end else begin
                            case (sh_sub_idx)
                                3'd0: shake_din <= sh_buf_data[63:0];
                                3'd1: shake_din <= sh_buf_data[127:64];
                                3'd2: shake_din <= sh_buf_data[191:128];
                                3'd3: shake_din <= sh_buf_data[255:192];
                                default: shake_din <= 64'd0;
                            endcase
                        end

                        // Determine if this is the last word
                        if (sh_word_idx == (sh_total_words - 16'd1)) begin
                            shake_din_last <= 1'b1;
                        end else begin
                            shake_din_last <= 1'b0;
                        end

                        sh_f_state <= SH_F_PULSE;
                    end

                    SH_F_PULSE: begin
                        shake_absorb <= 1'b1;
                        sh_word_idx <= sh_word_idx + 16'd1;

                        if (cfg_bypass_decode || cfg_hash_message_only) begin
                            if (sh_word_idx == (sh_total_words - 16'd1)) begin
                                sh_f_state <= SH_F_LAST_WAIT;
                            end else begin
                                sh_f_state <= SH_F_FEED;
                            end
                        end else if (sh_sub_idx == 3'd3) begin
                            // Advance to next 256-bit word
                            sh_buf_addr <= sh_buf_addr + 1'b1;
                            sh_sub_idx  <= 3'd0;
                            if (sh_word_idx == (sh_total_words - 16'd1)) begin
                                // Last word fed; wait for SHAKE permute
                                sh_f_state <= SH_F_LAST_WAIT;
                            end else begin
                                sh_f_state <= SH_F_RD;
                            end
                        end else begin
                            sh_sub_idx <= sh_sub_idx + 1'b1;
                            sh_f_state <= SH_F_FEED;
                        end

                        // Switch after the fifth nonce lane. The decoded
                        // 40-byte nonce occupies one full 256-bit word plus
                        // the low 64-bit lane of the second word.
                        if (!cfg_bypass_decode && !cfg_hash_message_only && sh_word_idx == 16'd4 && sh_sub_idx == 3'd0) begin
                            sh_buf_addr <= VERIFY_MSG_BASE;
                            sh_sub_idx  <= 3'd0;
                            sh_f_state  <= SH_F_RD;
                        end
                    end

                    SH_F_LAST_WAIT: begin
                        // Wait for SHAKE to actually enter the permutation
                        // (ready low), then return to squeeze-ready.
                        if (!shake_ready) begin
                            sh_seen_busy <= 1'b1;
                        end else if (sh_seen_busy) begin
                            sh_done_f  <= 1'b1;
                            sh_f_state <= SH_F_IDLE;
                        end
                    end

                    default: sh_f_state <= SH_F_IDLE;
                endcase
            end

            // Reset SHAKE feeder state on entry to SH
            if ((st != SH) && (sn == SH)) begin
                sh_done_f  <= 1'b0;
                sh_f_state <= SH_F_IDLE;
                sh_seen_busy <= 1'b0;
            end

            // ─── HP phase: HashToPoint → c polynomial ───
            if (st == HP && sn == HP) begin
                hp_done <= 1'b0;
                if (htp_coeff_valid) begin
                    // Pack int16 coefficients into 256-bit word
                    // (skip FP64 write; verify doesn't need FFT input)
                    if (hp_coeff_cnt == 4'd15) begin
                        hp_wr_pending <= 1'b1;
                        hp_wr_data    <= {htp_coeff, hp_coeff_buf[239:0]};
                        hp_coeff_cnt <= 4'd0;
                        hp_coeff_buf <= 256'd0;
                    end else begin
                        hp_coeff_buf[(hp_coeff_cnt * 16) +: 16] <= htp_coeff;
                        hp_coeff_cnt <= hp_coeff_cnt + 1'b1;
                    end
                end
                // Detect HP done: all 512 coeffs written (32 words)
                if (hp_wr_addr == (C_INT_BASE + 32) && !hp_wr_en && !hp_wr_pending) begin
                    hp_done <= 1'b1;
                end
            end

            // Reset HP state on entry
            if ((st != HP) && (sn == HP)) begin
                hp_wr_addr   <= C_INT_BASE;
                hp_coeff_cnt <= 4'd0;
                hp_coeff_buf <= 256'd0;
                hp_wr_pending <= 1'b0;
                hp_done      <= 1'b0;
            end

            // ─── N1 phase: NTT EXU (s1 = c - s2*h mod Q) ───
            // Handled entirely by u_ntt

            // ─── RC phase: Norm check ───
            // Handled by u_norm

            // ─── VF phase: Verify Finish ───
            if (st == VF && sn == VF) begin
                done   <= 1'b1;
                if (cfg_force_accept) begin
                    fail   <= 1'b0;
                    status <= 8'h00;
                end else if (!norm_accept) begin
                    fail   <= 1'b1;
                    status <= 8'h01;
                end else begin
                    fail   <= 1'b0;
                    status <= 8'h00;
                end
            end
        end
    end

    // ─── Phase FSM (next state) ───
    // Bus read mux for control/status registers and memory debug reads.
    always @(*) begin
        sn = st;
        case (st)
            SI: if (cr_start)  sn = cfg_start_at_ntt ? N1 :
                                       cfg_bypass_decode ? SH : DS;
            DS: if (sd_done)   sn = sd_fail ? SD : SH;
            SH: if (sh_done_f) sn = HP;
            HP: if (hp_done)   sn = N1;
            N1: if (ntt_done)  sn = ntt_fail ? SD : RC;
            RC: if (norm_done) sn = VF;
            VF:                sn = SD;
            SD: if (bus_cs && bus_wr && (bus_addr == REG_CR)) sn = SI;
            default: sn = SI;
        endcase
    end

endmodule
