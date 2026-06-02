`timescale 1ns/1ps
// Module: falconsign_ntt_psi_rom
// Purpose: ROM for psi factors used by the NTT/iNTT domain conversion path.
//
// Psi table ROM for NTT pre/post-processing: 1024 entries x 14-bit.
//   addr[8:0] = coefficient index i (0..511)
//   addr[9]   = 0: pre_mul[i]  = psi^i mod Q
//            = 1: post_mul[i] = psi^(-i) * N_inv mod Q
// Loaded from DOC/ntt_psi_table.hex (1024 lines of 4-hex-char each).
module falconsign_ntt_psi_rom #(
    parameter ADDR_W = 10
) (
    input  wire             clk,
    input  wire [ADDR_W-1:0] addr,
    output reg  [13:0]      data
);

    reg [13:0] rom [0:1023];

    // ROM contents are generated from the Falcon q=12289 psi table.
    initial begin
        $readmemh("DOC/ntt_psi_table.hex", rom);
    end

    // Combinational read keeps the NTT EXU address schedule simple.
    always @(*) begin
        data = rom[addr];
    end

endmodule
