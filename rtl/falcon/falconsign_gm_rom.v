`timescale 1ns/1ps
// Module: falconsign_gm_rom
// Purpose: Falcon GM table ROM: split/merge twiddle factors for ffSampling EXU.
//
// Falcon GM Table ROM — split/merge twiddle factors for ffSampling EXU.
//
// Stores the official Falcon gm_tab values needed by poly_split_fft and
// poly_merge_fft, indexed by (level, idx):
//   gm_tab[ idx + hn ]   where hn = 256 >> level
//
// Address mapping (level L, pair-index idx):
//   addr = base[L] + idx
//   base[L] = 256 - (1 << (8 - L))   for L=0..7
//
//   Level 0 (root,  hn=256): addr   0..127  (128 entries)
//   Level 1        (hn=128): addr 128..191  ( 64 entries)
//   Level 2        (hn= 64): addr 192..223  ( 32 entries)
//   Level 3        (hn= 32): addr 224..239  ( 16 entries)
//   Level 4        (hn= 16): addr 240..247  (  8 entries)
//   Level 5        (hn=  8): addr 248..251  (  4 entries)
//   Level 6        (hn=  4): addr 252..253  (  2 entries)
//   Level 7        (hn=  2): addr 254       (  1 entry )
//   Level 8        (hn=  1): no twiddle needed (logn=1 straight copy)
//
//   Total: 255 entries, 8-bit address.

module falconsign_gm_rom #(
    parameter ADDR_W = 8,
    parameter DEPTH  = 255
) (
    input  wire        clk,
    input  wire [ADDR_W-1:0] addr,
    output wire [63:0] gm_re,
    output wire [63:0] gm_im
);

    reg [63:0] rom_re [0:DEPTH-1];
    reg [63:0] rom_im [0:DEPTH-1];

    // Simulation/FPGA ROM initialization. Vivado maps this memory to ROM/BRAM
    // when the hex files are available during synthesis or implementation.
    initial begin
        $readmemh("DOC/gm_rom_re.hex", rom_re);
        $readmemh("DOC/gm_rom_im.hex", rom_im);
    end

    assign gm_re = rom_re[addr];
    assign gm_im = rom_im[addr];

endmodule
