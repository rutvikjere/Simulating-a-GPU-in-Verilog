`timescale 1ns/1ps
//`include "alu.v"
//`include "alu_wrapper.v"

module tb_alu;

  reg clk = 0;
  always #5 clk = ~clk; // 100 MHz

  reg rst = 1;

  reg  start = 0; 
  reg  wb_ready = 1;  
  wire busy;
  wire done;

  reg mask_en = 1'b1;
  reg [2:0] opcode = 3'b000;
  reg [3:0] lane_mask = 4'hF;  
  reg [31:0] srcA = 32'h0;
  reg [31:0] srcB = 32'h0;
  reg use_imm = 1'b0;
  reg [7:0] imm = 8'h00;
  reg cmp_signed = 1'b1;
  reg sat_mode = 1'b0;
  reg [3:0] pred = 4'hF;  
  reg [4:0] warp_id_i = 5'd7;
  reg dbg_dryrun = 1'b0;

  wire [31:0] alu_out;
  wire [3:0] write_en;
  wire [3:0] Z, N, C, V;
  wire illegal_opcode;
  wire [4:0] warp_id_o;
  wire [5:0] dbg_state;
  wire dbg_alu_out_valid;

  alu_fsm alu_dut (
    .clk(clk), .rst(rst),
    .start(start), .wb_ready(wb_ready), .busy(busy), .done(done),
    .mask_en(mask_en), .opcode(opcode), .lane_mask(lane_mask),
    .srcA(srcA), .srcB(srcB),
    .use_imm(use_imm), .imm(imm), .cmp_signed(cmp_signed), .sat_mode(sat_mode),
    .pred(pred), .warp_id_i(warp_id_i), .dbg_dryrun(dbg_dryrun),

    .alu_out(alu_out), .write_en(write_en),
    .Z(Z), .N(N), .C(C), .V(V),
    .illegal_opcode(illegal_opcode), .warp_id_o(warp_id_o), .dbg_alu_out_valid(dbg_alu_out_valid),
    .dbg_state(dbg_state)
  );

  task pulse_start;
    begin
      @(negedge clk); start <= 1'b1;
      @(negedge clk); start <= 1'b0;
    end
  endtask


  // test sequence
  initial begin
    $dumpfile("simulation_output.vcd");
    $dumpvars(0, tb_alu);

    repeat (4) @(negedge clk);
    rst <= 1'b0;

    // ADD with imm test:
    srcA <= 32'h03_02_01_00;  
    use_imm <= 1'b1;
    imm <= 8'h01;
    opcode <= 3'b000; 
    lane_mask <= 4'hF; pred <= 4'hF;
    wb_ready <= 1'b1;

    wait (!busy); 
    pulse_start();
    wait (done);
    $display("[T1 ADD+imm] alu_out=%h write_en=%b  Z=%b N=%b C=%b V=%b warp_id_o=%0d", alu_out, write_en, Z, N, C, V, warp_id_o);

    // AND with B test:
    use_imm <= 1'b0;
    opcode <= 3'b101;
    srcA <= 32'hF0_F0_F0_F0;
    srcB <= 32'h0F_F0_0F_F0;
    lane_mask <= 4'b1011; 
    pred <= 4'b1110; 

    wait (!busy); 
    pulse_start();
    wait (done);
    $display("[T2 AND    ] alu_out=%h write_en=%b  Z=%b N=%b C=%b V=%b", alu_out, write_en, Z, N, C, V);

    // SUB with imm test:
    opcode <= 3'b001; 
    use_imm <= 1'b1;
    imm <= 8'h01;
    cmp_signed <= 1'b1;
    sat_mode <= 1'b0;
    srcA <= 32'h80_00_7F_01;
    wb_ready <= 1'b0;
    wait (!busy); 
    pulse_start();
    repeat (8) @(negedge clk); 
    wb_ready <= 1'b1; 
    wait (done);
    $display("[T3 SUB-1 ] alu_out=%h write_en=%b  Z=%b N=%b C=%b V=%b", alu_out, write_en, Z, N, C, V);

    // illegal opcode test:
    opcode <= 3'b111;
    use_imm <= 1'b0;
    lane_mask <= 4'hF; pred <= 4'hF;
    srcA <= 32'h12_34_56_78;
    srcB <= 32'hAA_BB_CC_DD;

    wait (!busy); 
    pulse_start();
    wait (done);
    $display("[T4 ILLEGAL] alu_out=%h write_en=%b illegal=%b", alu_out, write_en, illegal_opcode);

    $display("All tests done.");
    #20 $finish;
  end

endmodule
