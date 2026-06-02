`timescale 1ns/1ps
// Module: falconsign_ntt_twiddle_rom
// Purpose: ROM for q=12289 NTT butterfly twiddle factors.
//
// NTT twiddle ROM: 512 entries x 14-bit.
//   addr[7:0] = k (0..255)
//   addr[8]   = 0: forward (omega^k), 1: inverse (omega^(-k))
// Loaded from DOC/ntt_twiddle_fwd.hex (512 lines of 4-hex-char each).
module falconsign_ntt_twiddle_rom #(
    parameter ADDR_W = 9
) (
    input  wire             clk,
    input  wire [ADDR_W-1:0] addr,
    output reg  [13:0]      data
);

    reg [13:0] rom [0:511];

    // ROM contents are generated from the Falcon forward NTT twiddle table.
    initial begin
        $readmemh("DOC/ntt_twiddle_fwd.hex", rom);
    end

    // Combinational read for the current butterfly twiddle.
    always @(*) begin
        data = rom[addr];
    end

endmodule
