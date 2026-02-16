//`default_nettype none
`timescale 1ns/1ns

// Rutvik Ajit Jere / ECE5545 / rutvikj / Adv VLSI Design Project #1
// Project #1: This code depicts the circuitry of an ALU of a single thread in a warp, 
// in a streamlined multiprocessesor, in a GPU.

module alu(
    clk, rst, in_valid, mask_en, opcode, lane_mask, srcA, srcB,
    use_imm, imm, cmp_signed, sat_mode, pred, warp_id_i, dbg_dryrun,
    alu_out, write_en, Z, N, C, V, out_valid, illegal_opcode, warp_id_o // outputs
);

  input clk, rst, in_valid, mask_en, use_imm;
  input [2:0] opcode;
  input [3:0] lane_mask; // Lanes of the warp 
  input [31:0] srcA, srcB;
  input [7:0] imm;           
  input cmp_signed;     // 1: compute V as signed overflow; 0: V=0
  input sat_mode;       // 1: signed saturating ADD (clamp on overflow)
  input [3:0] pred;     // Per-lane predicate (ANDed with lane_mask)
  input [4:0] warp_id_i;
  input dbg_dryrun;     // 1: test input, won't commit alu_out (write_en=0)

  output wire [31:0] alu_out;
  output reg [3:0] write_en;
  output reg [3:0] Z, N, C, V;
  output reg out_valid;
  output reg illegal_opcode; //opcode = 111
  output reg [4:0] warp_id_o;

  // Initializing everything:
  reg [31:0] R_reg;
  assign alu_out = R_reg;

  wire [3:0] mask_eff = (mask_en ? lane_mask : 4'hF) & pred;
  wire [3:0] commit_mask = dbg_dryrun ? 4'h0 : mask_eff; // commit the output to memory or not

  integer i;
  reg [7:0] a, b, r;
  reg [8:0] sum; // For ADD/SUB operations
  reg       ovf;

  always @(posedge clk) begin
    if (rst) begin
      R_reg <= 32'h0;
      Z <= 4'h0;
      N <= 4'h0;
      C <= 4'h0;
      V <= 4'h0;
      write_en  <= 4'h0;
      out_valid <= 1'b0;
      illegal_opcode<= 1'b0;
      warp_id_o <= 5'd0;
    end else begin
      out_valid  <= 1'b0;
      //illegal_opcode <= 1'b0;

      if (in_valid) begin
        // default flags
        C <= 4'h0;
        V <= 4'h0;

        for (i = 0; i < 4; i = i + 1) begin  // cycle through all the lanes and compute the operation
          a = srcA[i*8 +: 8];
          b = use_imm ? imm : srcB[i*8 +: 8];

          case (opcode)
            3'b000: begin  //ADD
              sum = {1'b0, a} + {1'b0, b};
              r   = sum[7:0];

              // signed overflow
              ovf = (~(a[7]^b[7]) & (r[7]^a[7]));
              V[i] <= cmp_signed ? ovf : 1'b0;

              // unsigned carry
              C[i] <= sum[8];

              // saturate on signed overflow
              if (sat_mode && cmp_signed && ovf) begin
                if (~a[7] && ~b[7]) r = 8'h7F; 
                else                 r = 8'h80;
              end
            end

            3'b001: begin  //SUB 
              sum = {1'b0, a} - {1'b0, b};
              r   = sum[7:0];
              // Borrow
              C[i] <= (a < b);
              
              ovf = ((a[7]^b[7]) & (r[7]^a[7]));
              V[i] <= cmp_signed ? ovf : 1'b0;
            end

            3'b010: r = ~a;        // NOT
            3'b011: r = ~(a & b);  // NAND
            3'b100: r = ~(a | b);  // NOR
            3'b101: r =  (a & b);  // AND
            3'b110: r =  (a | b);  // OR

            default: begin         // 3'b111 -> illegal
              r = 8'h00;
              illegal_opcode <= 1'b1;
            end
          endcase

          // write the final result to the lane
          R_reg[i*8 +: 8] <= r;

          // Basic flags valid for all ops
          Z[i] <= (r == 8'h00);
          N[i] <= r[7];
        end

        write_en  <= commit_mask;
        out_valid <= 1'b1;      
        warp_id_o <= warp_id_i; 
      end
    end
  end

endmodule

//`default_nettype wire 