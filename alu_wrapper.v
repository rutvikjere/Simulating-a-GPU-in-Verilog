//`include "alu.v"
`timescale 1ns/1ps

module alu_fsm(
    clk,
    rst,
    start,          // single cycle pulse to start an operation
    wb_ready,       // downstream ready (lets wrapper finish)
    busy,           // high while FSM not in IDLE
    done,           // single cycle pulse when operation completes
    mask_en,
    opcode,
    lane_mask,    
    srcA,         
    srcB,         
    use_imm,
    imm,           
    cmp_signed,
    sat_mode,
    pred,           
    warp_id_i,      
    dbg_dryrun,
    alu_out,        
    write_en,      
    Z, N, C, V,   
    illegal_opcode,
    warp_id_o,     

    // FSM debug
    dbg_alu_out_valid,
    dbg_state     
);

  input clk, rst;
  input start;
  input wb_ready;
  output busy;
  output reg done;
  input mask_en;
  input [2:0] opcode;
  input [3:0] lane_mask;
  input [31:0] srcA, srcB;
  input use_imm;
  input [7:0] imm;
  input cmp_signed;
  input sat_mode;
  input [3:0] pred;
  input [4:0] warp_id_i;
  input dbg_dryrun;
  output [31:0] alu_out;
  output [3:0] write_en;
  output [3:0] Z, N, C, V;
  output illegal_opcode;
  output [4:0] warp_id_o;
  output dbg_alu_out_valid;
  output [5:0] dbg_state;

  reg mask_en_r, use_imm_r, cmp_signed_r, sat_mode_r, dbg_dryrun_r;
  reg [2:0] opcode_r;
  reg [3:0] lane_mask_r, pred_r;
  reg [7:0] imm_r;
  reg [31:0] A_r, B_r;
  reg [4:0] warp_id_r;
  reg [5:0] state, nxt;

  localparam S00_RESET     = 6'd0,
             S01_IDLE      = 6'd1,
             S02_LATCH     = 6'd2,
             S03_DECODE    = 6'd3,
             S04_CHECK_RDY = 6'd4,
             S05_ISSUE     = 6'd5,
             S06_MIC0      = 6'd6,
             S07_MIC1      = 6'd7,
             S08_MIC2      = 6'd8,
             S09_MIC3      = 6'd9,
             S10_MIC4      = 6'd10,
             S11_MIC5      = 6'd11,
             S12_MIC6      = 6'd12,
             S13_MIC7      = 6'd13,
             S14_MIC8      = 6'd14,
             S15_MIC9      = 6'd15,
             S16_MIC10     = 6'd16,
             S17_MIC11     = 6'd17,
             S18_MIC12     = 6'd18,
             S19_MIC13     = 6'd19,
             S20_MIC14     = 6'd20,
             S21_MIC15     = 6'd21,
             S22_MIC16     = 6'd22,
             S23_MIC17     = 6'd23,
             S24_MIC18     = 6'd24,
             S25_MIC19     = 6'd25,
             S26_MIC20     = 6'd26,
             S27_MIC21     = 6'd27,
             S28_MIC22     = 6'd28,
             S29_MIC23     = 6'd29,
             S30_WB_ALIGN  = 6'd30,
             S31_OUT_PULSE = 6'd31,
             S32_COMPLETE  = 6'd32,
             S33_ILLEGAL   = 6'd33,
             S34_SOFTSTALL = 6'd34,
             S35_SPARE     = 6'd35;

  assign dbg_state = state;
  assign busy = (state != S01_IDLE);

  reg  alu_in_valid;  
  wire alu_out_valid;  
  assign dbg_alu_out_valid = alu_out_valid;
  reg  result_seen;  

  alu u_alu(
    .clk        (clk),
    .rst        (rst),
    .in_valid   (alu_in_valid),
    .mask_en    (mask_en_r),
    .opcode         (opcode_r),
    .lane_mask  (lane_mask_r),
    .srcA     (A_r),
    .srcB     (B_r),
    .use_imm    (use_imm_r),
    .imm        (imm_r),
    .cmp_signed (cmp_signed_r),
    .sat_mode   (sat_mode_r),
    .pred       (pred_r),
    .warp_id_i  (warp_id_r),
    .dbg_dryrun (dbg_dryrun_r),

    .alu_out     (alu_out),
    .write_en   (write_en),
    .Z          (Z),
    .N          (N),
    .C          (C),
    .V          (V),
    .out_valid  (alu_out_valid),
    .illegal_opcode (illegal_opcode),
    .warp_id_o  (warp_id_o)
  );

  always @(posedge clk or posedge rst) begin
    if (rst) state <= S00_RESET;
    else     state <= nxt;
  end

  always @* begin
    nxt = state;
    case (state)
      S00_RESET    : nxt = S01_IDLE;
      S01_IDLE     : nxt = (start ? S02_LATCH : S01_IDLE);
      S02_LATCH    : nxt = S03_DECODE;
      S03_DECODE   : nxt = S04_CHECK_RDY;
      S04_CHECK_RDY: nxt = wb_ready ? S05_ISSUE : S04_CHECK_RDY;
      S05_ISSUE    : nxt = S06_MIC0;
      S06_MIC0     : nxt = S07_MIC1;
      S07_MIC1     : nxt = S08_MIC2;
      S08_MIC2     : nxt = S09_MIC3;
      S09_MIC3     : nxt = S10_MIC4;
      S10_MIC4     : nxt = S11_MIC5;
      S11_MIC5     : nxt = S12_MIC6;
      S12_MIC6     : nxt = S13_MIC7;
      S13_MIC7     : nxt = S14_MIC8;
      S14_MIC8     : nxt = S15_MIC9;
      S15_MIC9     : nxt = S16_MIC10;
      S16_MIC10    : nxt = S17_MIC11;
      S17_MIC11    : nxt = S18_MIC12;
      S18_MIC12    : nxt = S19_MIC13;
      S19_MIC13    : nxt = S20_MIC14;
      S20_MIC14    : nxt = S21_MIC15;
      S21_MIC15    : nxt = S22_MIC16;
      S22_MIC16    : nxt = S23_MIC17;
      S23_MIC17    : nxt = S24_MIC18;
      S24_MIC18    : nxt = S25_MIC19;
      S25_MIC19    : nxt = S26_MIC20;
      S26_MIC20    : nxt = S27_MIC21;
      S27_MIC21    : nxt = S28_MIC22;
      S28_MIC22    : nxt = S29_MIC23;
      S29_MIC23    : nxt = S30_WB_ALIGN;
      S30_WB_ALIGN : nxt = (result_seen && wb_ready) ? S31_OUT_PULSE : S30_WB_ALIGN;
      S31_OUT_PULSE: nxt = S32_COMPLETE;
      S32_COMPLETE : nxt = S01_IDLE;
      S33_ILLEGAL  : nxt = S30_WB_ALIGN;
      S34_SOFTSTALL: nxt = S01_IDLE;
      S35_SPARE    : nxt = S01_IDLE;
      default      : nxt = S01_IDLE;
    endcase
  end

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      mask_en_r    <= 1'b0;
      opcode_r     <= 3'b000;
      lane_mask_r  <= 4'h0;
      A_r          <= 32'h0;
      B_r          <= 32'h0;
      use_imm_r    <= 1'b0;
      imm_r        <= 8'h00;
      cmp_signed_r <= 1'b0;
      sat_mode_r   <= 1'b0;
      pred_r       <= 4'hF;  
      warp_id_r    <= 5'd0;
      dbg_dryrun_r <= 1'b0;
      alu_in_valid <= 1'b0;
      result_seen  <= 1'b0;
      done         <= 1'b0;
    end else begin
      alu_in_valid <= 1'b0;
      done         <= 1'b0;

      if (alu_out_valid) result_seen <= 1'b1;
      if (state == S32_COMPLETE) result_seen <= 1'b0;

      case (state)
        S02_LATCH: begin
          mask_en_r    <= mask_en;
          opcode_r     <= opcode;
          lane_mask_r  <= lane_mask;
          A_r          <= srcA;
          B_r          <= srcB;
          use_imm_r    <= use_imm;
          imm_r        <= imm;
          cmp_signed_r <= cmp_signed;
          sat_mode_r   <= sat_mode;
          pred_r       <= pred;
          warp_id_r    <= warp_id_i;
          dbg_dryrun_r <= dbg_dryrun;
        end

        S05_ISSUE: begin
          alu_in_valid <= 1'b1;
        end

        S31_OUT_PULSE: begin
          done <= 1'b1;
        end
      endcase
    end
  end

endmodule
