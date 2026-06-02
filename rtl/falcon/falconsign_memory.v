`timescale 1ns/1ps
// Module: falconsign_memory
// Purpose: shared Falcon workspace memory.
//
// Storage is 256-bit words. FFT data packs one butterfly pair per word:
//   [127:0]   = a complex value {a_im[63:0], a_re[63:0]}
//   [255:128] = b complex value {b_im[63:0], b_re[63:0]}
//
// This wrapper exposes one synchronous 256-bit read port and one synchronous
// 256-bit write port. Bus/debug access is arbitrated in falconsign_top.

module falconsign_memory #(
    parameter ADDR_W = 10,
    parameter BANK_W = 256,
    parameter DEPTH  = 1024
) (
    input  wire                clk,
    input  wire                rst_n,

    input  wire                rd_en,
    input  wire [ADDR_W-1:0]   rd_addr,
    output wire [BANK_W-1:0]   rd_data,

    input  wire                wr_en,
    input  wire [ADDR_W-1:0]   wr_addr,
    input  wire [BANK_W-1:0]   wr_data
);

    localparam BANK_SEL_W  = 2;
    localparam BANK_ADDR_W = ADDR_W - BANK_SEL_W;

    reg [BANK_W-1:0] bank0 [0:(DEPTH/4)-1];
    reg [BANK_W-1:0] bank1 [0:(DEPTH/4)-1];
    reg [BANK_W-1:0] bank2 [0:(DEPTH/4)-1];
    reg [BANK_W-1:0] bank3 [0:(DEPTH/4)-1];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < (DEPTH/4); init_i = init_i + 1) begin
            bank0[init_i] = {BANK_W{1'b0}};
            bank1[init_i] = {BANK_W{1'b0}};
            bank2[init_i] = {BANK_W{1'b0}};
            bank3[init_i] = {BANK_W{1'b0}};
        end
    end

    wire [1:0] rd_bank = rd_addr[1:0];
    wire [BANK_ADDR_W-1:0] rd_idx = rd_addr[ADDR_W-1:2];

    wire [1:0] wr_bank = wr_addr[1:0];
    wire [BANK_ADDR_W-1:0] wr_idx = wr_addr[ADDR_W-1:2];

    reg [BANK_W-1:0] rd_q;
    initial rd_q = {BANK_W{1'b0}};

    always @(posedge clk) begin
        if (rd_en) begin
            case (rd_bank)
                2'd0: rd_q <= bank0[rd_idx];
                2'd1: rd_q <= bank1[rd_idx];
                2'd2: rd_q <= bank2[rd_idx];
                default: rd_q <= bank3[rd_idx];
            endcase
        end

        if (wr_en) begin
            case (wr_bank)
                2'd0: bank0[wr_idx] <= wr_data;
                2'd1: bank1[wr_idx] <= wr_data;
                2'd2: bank2[wr_idx] <= wr_data;
                default: bank3[wr_idx] <= wr_data;
            endcase
        end
    end

    assign rd_data = rd_q;

endmodule
