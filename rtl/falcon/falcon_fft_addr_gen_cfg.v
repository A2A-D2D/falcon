`timescale 1ns/1ps
// Module: falcon_fft_addr_gen_cfg
// Purpose: combinational address generator for one FFT butterfly pair.

module falcon_fft_addr_gen_cfg #
(
    parameter ADDR_W = 10
)
(
    input      [4:0]         logn,
    input      [4:0]         stage_idx,
    input      [ADDR_W-1:0]  pair_idx,
    output reg [ADDR_W-1:0]  addr_a,
    output reg [ADDR_W-1:0]  addr_b,
    output reg [ADDR_W-1:0]  twiddle_idx
);

    reg [ADDR_W:0] half_m;
    reg [ADDR_W:0] m_size;
    reg [ADDR_W:0] j_idx;
    reg [ADDR_W:0] group_idx;
    reg [ADDR_W:0] base_idx;
    reg [4:0]      shift_amt;

    // Pure combinational address calculation for one butterfly pair.
    always @(*) begin
        half_m = ({(ADDR_W){1'b0}} | 1'b1) << stage_idx;
        m_size = half_m << 1;

        if (half_m == 0) begin
            j_idx     = 0;
            group_idx = 0;
        end else begin
            j_idx     = pair_idx & (half_m - 1'b1);
            group_idx = pair_idx >> stage_idx;
        end

        base_idx = group_idx * m_size;
        addr_a   = base_idx[ADDR_W-1:0] + j_idx[ADDR_W-1:0];
        addr_b   = addr_a + half_m[ADDR_W-1:0];

        if (logn > (stage_idx + 1'b1)) begin
            shift_amt = logn - stage_idx - 1'b1;
        end else begin
            shift_amt = 0;
        end
        twiddle_idx = j_idx[ADDR_W-1:0] << shift_amt;
    end

endmodule
