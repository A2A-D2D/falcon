`timescale 1ns/1ps
// Module: falconsign_twiddle_rom
// Purpose: f64 FFT twiddle ROM for the Falcon transform datapath.
//

module falconsign_twiddle_rom #(
    parameter ADDR_W = 8,
    parameter DEPTH  = 256
) (
    input  wire        clk,
    input  wire [ADDR_W-1:0] addr,
    output wire [63:0] twiddle_re,
    output wire [63:0] twiddle_im
);

    reg [63:0] rom_re [0:DEPTH-1];
    reg [63:0] rom_im [0:DEPTH-1];

    reg [1023:0] rom_re_path;
    reg [1023:0] rom_im_path;

    // ROM contents are loaded from the generated f64 FFT twiddle hex files.
    initial begin
        rom_re_path = "DOC/twiddle_rom_re.hex";
        rom_im_path = "DOC/twiddle_rom_im.hex";
        if (!$value$plusargs("TWIDDLE_RE=%s", rom_re_path))
            rom_re_path = "DOC/twiddle_rom_re.hex";
        if (!$value$plusargs("TWIDDLE_IM=%s", rom_im_path))
            rom_im_path = "DOC/twiddle_rom_im.hex";
        $readmemh(rom_re_path, rom_re);
        $readmemh(rom_im_path, rom_im);
    end

    // Combinational read to match FFT EXU timing (0-cycle latency expected)
    assign twiddle_re = rom_re[addr];
    assign twiddle_im = rom_im[addr];

endmodule
