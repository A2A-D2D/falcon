`timescale 1ns/1ps
// Module: falconsign_word_fifo
// Purpose: small synchronous FIFO for 64-bit SHAKE/RNG stream words.
//

module falconsign_word_fifo #(
    parameter WIDTH  = 64,
    parameter DEPTH  = 16,
    parameter ADDR_W = 4
) (
    input  wire             clk,
    input  wire             rst_n,

    input  wire             wr_valid,
    output wire             wr_ready,
    input  wire [WIDTH-1:0] wr_data,

    output wire             rd_valid,
    input  wire             rd_ready,
    output wire [WIDTH-1:0] rd_data
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_W-1:0] wr_ptr;
    reg [ADDR_W-1:0] rd_ptr;
    reg [ADDR_W:0]   count;

    wire wr_fire = wr_valid && wr_ready;
    wire rd_fire = rd_valid && rd_ready;

    assign wr_ready = (count != DEPTH[ADDR_W:0]);
    assign rd_valid = (count != {ADDR_W+1{1'b0}});
    assign rd_data  = mem[rd_ptr];

    // Synchronous FIFO pointers and storage. Writes and reads may occur in the
    // same cycle when both sides are ready.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= {ADDR_W{1'b0}};
            rd_ptr <= {ADDR_W{1'b0}};
            count  <= {ADDR_W+1{1'b0}};
        end else begin
            if (wr_fire) begin
                mem[wr_ptr] <= wr_data;
                wr_ptr <= wr_ptr + 1'b1;
            end

            if (rd_fire) begin
                rd_ptr <= rd_ptr + 1'b1;
            end

            case ({wr_fire, rd_fire})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
