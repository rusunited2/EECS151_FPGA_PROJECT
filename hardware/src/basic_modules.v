/* Basic modules file for project.

*/
`timescale 1ns/1ns

// MUX module
module TWO_INPUT_MUX(sel, in0, in1, out);
  input sel;
  input [31:0] in0, in1;
  output reg [31:0] out;
  
  always @(*) begin
	case(sel)
	  1'b0: out = in0;
	  1'b1: out = in1;
	  default: out = 0;
	endcase
  end
endmodule // TWO_INPUT_MUX

module FOUR_INPUT_MUX(sel, in0, in1, in2, in3, out);
  input [1:0] sel;
  input [31:0] in0, in1, in2, in3;
  output reg [31:0] out;
  
  always @(*) begin
	case(sel)
	  2'b00: out = in0;
	  2'b01: out = in1;
	  2'b10: out = in2;
	  2'b11: out = in3;
	  default: out = 0;
	endcase
  end
endmodule // FOUR_INPUT_MUX

module EIGHT_INPUT_MUX(sel, in0, in1, in2, in3, in4, in5, in6, in7, out);
  input [2:0] sel;
  input [31:0] in0, in1, in2, in3, in4, in5, in6, in7;
  output reg [31:0] out;
  
  always @(*) begin
	case(sel)
	  3'b000: out = in0;
	  3'b001: out = in1;
	  3'b010: out = in2;
	  3'b011: out = in3;
	  3'b100: out = in4;
	  3'b101: out = in5;
	  3'b110: out = in6;
	  3'b111: out = in7;
	  default: out = 0;
	endcase
  end
endmodule // EIGHT_INPUT_MUX

// Adder
module ADDER(in0, in1, out);
  input [31:0] in0, in1;
  output [31:0] out;
  
  assign out = in0 + in1;
endmodule // ADDER

// ALU
module ALU(alu_sel, rs1, rs2, out);
  input [3:0] alu_sel;
  input [31:0] rs1, rs2;
  output reg [31:0] out;
  
  always @(*) begin
	case(alu_sel)
	  4'b0000: out = rs1 + rs2;
	  4'b0001: out = rs1 - rs2;
	  4'b0010: out = rs1 & rs2;
	  4'b0011: out = rs1 | rs2;
	  4'b0100: out = rs1 ^ rs2;
	  4'b0101: out = rs1 << rs2[4:0];
	  4'b0110: out = rs1 >> rs2[4:0];
	  4'b0111: out = $signed(rs1) >>> rs2[4:0];
	  4'b1000: out = (rs1 < rs2) ? $signed(1) : 0; // slt
	  4'b1001: out = (rs1 < rs2) ? 1 : 0; // sltu
	  default: out = 0; // For error/default case
	endcase
  end
endmodule // ALU

// Branch Comparator
module BRANCH_COMPARATOR(rs1, rs2, br_un, br_eq, br_lt);
  input [31:0] rs1, rs2;
  input br_un;
  output br_eq;
  output reg br_lt;

  assign br_eq = rs1 == rs2;

  always @(*) begin
	case(br_un)
		1'b0: br_lt = $signed(rs1) < $signed(rs2); // ask about this
		1'b1: br_lt = rs1 < rs2;
	endcase
  end
endmodule // BRANCH_COMPARATOR

module IMM_GEN(inst, imm);
  input [31:0] inst;
  output reg [31:0] imm;

  always @(*) begin
	case(inst[6:0])
		7'b0010011: imm = {{20{inst[31]}}, inst[31:20]}; // I-type instruction
		7'b0100011: imm = {{20{inst[31]}}, inst[31:25], inst[11:7]}; // S-type instruction
		7'b1100011: imm = {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 0}; // SB-type instruction
		7'b1101111: imm = {{20{inst[31]}}, inst[19:12], inst[20], inst[30:21], 0}; // J-type instruction
		7'b0010111: imm = {inst[31:12], {12{1'b0}}}; // AUIPC instruction (what to do with bottom bits)
		7'b0110111: imm = {inst[31:12], {12{1'b0}}}; // LUI instruction
		default: imm = 32'd0;
	endcase
  end
endmodule
