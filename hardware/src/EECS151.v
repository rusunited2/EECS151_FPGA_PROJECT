/* Standard include file for EECS151.

 The "no flip-flop inference" policy.  Instead of using flip-flop and
 register inference, all EECS151/251A Verilog specifications will use
 explicit instantiation of register modules (defined below).  This
 policy will apply to lecture, discussion, lab, project, and problem
 sets.  This way of specification matches our RTL model of circuit,
 i.e., all specifications are nothing but a set of interconnected
 combinational logic blocks and state elements.  The goal is to
 simplify the use of Verilog and avoid mistakes that arise from
 specifying sequential logic.  Also, we can eliminate the explicit use
 of the non-blocking assignment "<=", and the associated confusion
 about blocking versus non-blocking.

 Here is a draft set of standard registers for EECS151.  All are
 positive edge triggered.  R and CE represent synchronous reset and
 clock enable, respectively. Both are active high.

 REGISTER 
 REGISTER_CE
 REGISTER_R
 REGISTER_R_CE
*/
`timescale 1ns/1ns
`include "opcode.vh"

// Register of D-Type Flip-flops
module REGISTER(q, d, clk);
  parameter N = 1;
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	     clk;
  initial q = {N{1'b0}};
  always @(posedge clk)
    q <= d;
endmodule // REGISTER

// Register with clock enable
module REGISTER_CE(q, d, ce, clk);
  parameter N = 1;
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	      ce, clk;
  initial q = {N{1'b0}};
  always @(posedge clk)
    if (ce) q <= d;
endmodule // REGISTER_CE

// Register with reset value
module REGISTER_R(q, d, rst, clk);
  parameter N = 1;
  parameter INIT = {N{1'b0}};
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	      rst, clk;
  initial q = INIT;
  always @(posedge clk)
    if (rst) q <= INIT;
    else q <= d;
endmodule // REGISTER_R

// Register with reset and clock enable
//  Reset works independently of clock enable
module REGISTER_R_CE(q, d, rst, ce, clk);
  parameter N = 1;
  parameter INIT = {N{1'b0}};
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	      rst, ce, clk;
  initial q = INIT;
  always @(posedge clk)
    if (rst) q <= INIT;
    else if (ce) q <= d;
endmodule // REGISTER_R_CE


/* 
 Memory Blocks.
*/

// Single-port ROM with asynchronous read
module ASYNC_ROM(q, addr);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input  [AWIDTH-1:0] addr; // address
  output [DWIDTH-1:0] q;    // read data

  (* rom_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  assign q = mem[addr];
endmodule // ASYNC_ROM

// Single-port RAM with asynchronous read
module ASYNC_RAM(q, d, addr, we, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr; // address
  input 	            we;   // write-enable
  input  [DWIDTH-1:0] d;    // write data
  output [DWIDTH-1:0] q;    // read data

  (* ram_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  always @(posedge clk) begin
    if (we)
      mem[addr] <= d;
  end

  assign q = mem[addr];
endmodule // ASYNC_RAM

// Single-port ROM with synchronous read
module SYNC_ROM(q, addr, en, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input 	            clk;
  input               en;   // ram-enable
  input  [AWIDTH-1:0] addr; // address
  output [DWIDTH-1:0] q;    // read data

  (* rom_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data_reg;
  always @(posedge clk) begin
    if (en) begin
      read_data_reg <= mem[addr];
    end
  end

  assign q = read_data_reg;
endmodule // SYNC_ROM

// Single-port RAM with synchronous read
module SYNC_RAM(q, d, addr, we, en, clk);
  parameter DWIDTH = 8;           // Data width
  parameter AWIDTH = 8;           // Address width
  parameter DEPTH  = 1 << AWIDTH; // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr; // address
  input 	            we;   // write-enable
  input               en;   // ram-enable
  input  [DWIDTH-1:0] d;    // write data
  output [DWIDTH-1:0] q;    // read data

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data_reg;
  always @(posedge clk) begin
    if (en) begin
      if (we)
        mem[addr] <= d;
      read_data_reg <= mem[addr];
    end
  end

  assign q = read_data_reg;
endmodule // SYNC_RAM

// Dual-port ROM with synchronous read
module SYNC_ROM_DP(q0, addr0, en0, q1, addr1, en1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input 	            clk;
  input               en0, en1;     // ram-enable
  input  [AWIDTH-1:0] addr0, addr1; // address
  output [DWIDTH-1:0] q0, q1;       // read data

  (* rom_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data0_reg;
  reg [DWIDTH-1:0] read_data1_reg;

  always @(posedge clk) begin
    if (en0) begin
      read_data0_reg <= mem[addr0];
    end
  end

  always @(posedge clk) begin
    if (en1) begin
      read_data1_reg <= mem[addr1];
    end
  end

  assign q0 = read_data0_reg;
  assign q1 = read_data1_reg;
endmodule // SYNC_ROM_DP

// Dual-port RAM with asynchronous read
module ASYNC_RAM_DP(q0, d0, addr0, we0, q1, d1, addr1, we1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr0, addr1; // address
  input 	            we0, we1;     // write-enable
  input  [DWIDTH-1:0] d0, d1;       // write data
  output [DWIDTH-1:0] q0, q1;       // read data

  (* ram_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  always @(posedge clk) begin
    if (we0)
      mem[addr0] <= d0;
  end

  always @(posedge clk) begin
    if (we1)
      mem[addr1] <= d1;
  end

  assign q0 = mem[addr0];
  assign q1 = mem[addr1];

endmodule // ASYNC_RAM_DP

// Dual-port RAM with synchronous read
module SYNC_RAM_DP(q0, d0, addr0, we0, en0, q1, d1, addr1, we1, en1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr0, addr1; // address
  input 	            we0, we1;     // write-enable
  input               en0, en1;     // ram-enable
  input  [DWIDTH-1:0] d0, d1;       // write data
  output [DWIDTH-1:0] q0, q1;       // read data

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data0_reg, read_data1_reg;

  always @(posedge clk) begin
    if (en0) begin
      if (we0)
        mem[addr0] <= d0;
      read_data0_reg <= mem[addr0];
    end
  end

  always @(posedge clk) begin
    if (en1) begin
      if (we1)
        mem[addr1] <= d1;
      read_data1_reg <= mem[addr1];
    end
  end

  assign q0 = read_data0_reg;
  assign q1 = read_data1_reg;

endmodule // SYNC_RAM_DP

// Single-port RAM with synchronous read with write byte-enable
module SYNC_RAM_WBE(q, d, addr, en, wbe, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input [DWIDTH-1:0]   d;    // Data input
  input [AWIDTH-1:0]   addr; // Address input
  input [DWIDTH/8-1:0] wbe;  // write-byte-enable
  input en;
  input clk;
  output [DWIDTH-1:0] q;

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data_reg;
  always @(posedge clk) begin
    if (en) begin
      for (i = 0; i < DWIDTH/8; i = i+1) begin
        if (wbe[i])
          mem[addr][i*8 +: 8] <= d[i*8 +: 8];
        end
      read_data_reg <= mem[addr];
    end
  end

  assign q = read_data_reg;
endmodule // SYNC_RAM_WBE

// Dual-port RAM with synchronous read with write byte-enable
module SYNC_RAM_DP_WBE(q0, d0, addr0, en0, wbe0, q1, d1, addr1, en1, wbe1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input clk;
  input [DWIDTH-1:0]   d0;    // Data input
  input [AWIDTH-1:0]   addr0; // Address input
  input [DWIDTH/8-1:0] wbe0;  // write-byte-enable
  input                en0;
  output [DWIDTH-1:0]  q0;

  input [DWIDTH-1:0]   d1;    // Data input
  input [AWIDTH-1:0]   addr1; // Address input
  input [DWIDTH/8-1:0] wbe1;  // write-byte-enable
  input                en1;
  output [DWIDTH-1:0]  q1;

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end


  reg [DWIDTH-1:0] read_data0_reg;
  reg [DWIDTH-1:0] read_data1_reg;

  always @(posedge clk) begin
    if (en0) begin
      for (i = 0; i < 4; i = i+1) begin
        if (wbe0[i])
          mem[addr0][i*8 +: 8] <= d0[i*8 +: 8];
      end
      read_data0_reg <= mem[addr0];
    end
  end

  always @(posedge clk) begin
    if (en1) begin
      for (i = 0; i < 4; i = i+1) begin
        if (wbe1[i])
          mem[addr1][i*8 +: 8] <= d1[i*8 +: 8];
        end
      read_data1_reg <= mem[addr1];
    end
  end

  assign q0 = read_data0_reg;
  assign q1 = read_data1_reg;

endmodule // SYNC_RAM_DP_WBE

// Multi-port RAM with two asynchronous-read ports, one synchronous-write port
module ASYNC_RAM_1W2R(d0, addr0, we0, q1, addr1, q2, addr2, clk);
  parameter DWIDTH = 8;  // Data width
  parameter AWIDTH = 8;  // Address width
  parameter DEPTH = 256; // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";
  input clk;

  input [DWIDTH-1:0] d0;    // Data input
  input [AWIDTH-1:0] addr0; // Address input
  input              we0;   // Write enable

  input [AWIDTH-1:0] addr1; // Address input
  output [DWIDTH-1:0] q1;

  input [AWIDTH-1:0] addr2; // Address input
  output [DWIDTH-1:0] q2;

  (* ram_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  always @(posedge clk) begin
    if (we0)
      mem[addr0] <= d0;
  end

  assign q1 = mem[addr1];
  assign q2 = mem[addr2];

endmodule // ASYNC_RAM_1W2R

module WF_CU(rst, instruction, rf_we, wb_sel, ldx_sel, pc_sel, br_taken);
  // needs branch logic, jalr, and jal (as inputs)
  input rst;
	input [31:0] instruction;
  input br_taken;
  // input br_taken, jalr, jal;
	output reg rf_we;
  output reg [1:0] wb_sel;
	output reg [2:0] ldx_sel, pc_sel;

  always @(*) begin
    if (rst) pc_sel = 0;
  end
 
	always @(*) begin
		case(instruction[6:2]) // R
      `OPC_ARI_RTYPE_5: begin
				ldx_sel = 0;
				wb_sel = 1;
        // if (br_taken == 1'b1 || jalr == 1'b1) pc_sel = 3;
        // else if (jal == 1'b1) pc_sel == 1; // testing this
        // else pc_sel = 2;
        pc_sel = 2;
        rf_we = 1;
      end
      `OPC_ARI_ITYPE_5: begin // I and I*
				ldx_sel = 0;
				wb_sel = 1;
				// if (br_taken == 1) pc_sel = 3;
        // else pc_sel = 2;
        pc_sel = 2;
        rf_we = 1;
      end
      `OPC_LOAD_5: begin // I type for load
				ldx_sel = 0;
				wb_sel = 0;
        rf_we = 1;
				// if (br_taken == 1) pc_sel = 3;
        // else pc_sel = 2;
        pc_sel = 2;
        case(instruction[14:12])
          3'b000: ldx_sel = 3'b100; // lb
          3'b100: ldx_sel = 3'b011; // lbu
          3'b001: ldx_sel = 3'b010; // lh
          3'b101: ldx_sel = 3'b001; // lhu
          3'b010: ldx_sel = 3'b000; // lw
          default: begin
            ldx_sel = 3'b000;
          end
        endcase
      end
      `OPC_STORE_5: begin // S-type (store)
				ldx_sel = 0;
				wb_sel = 0;
				// if (br_taken == 1) pc_sel = 3;
        // else pc_sel = 2;
        pc_sel = 2;
        rf_we = 0;
      end
      `OPC_BRANCH_5: begin // B-type
        ldx_sel = 0;
        wb_sel = 0;
        rf_we = 0;
        if (br_taken == 1) pc_sel = 3;
        else pc_sel = 2;
        // pc_sel = 2;
      end
			`OPC_JAL_5: begin // J-Type
        ldx_sel = 0;
        wb_sel = 2;
        rf_we = 1;
        // if (br_taken == 1) pc_sel = 3;
        // else pc_sel = 2;
        pc_sel = 3;
      end
      `OPC_JALR_5: begin // JALR
        ldx_sel = 0;
        wb_sel = 2;
        rf_we = 1;
        // if (br_taken == 1) pc_sel = 3;
        // else pc_sel = 2;
        pc_sel = 3;
      end
      `OPC_AUIPC_5: begin // U (AUIPC) Type
        ldx_sel = 0;
        wb_sel = 1;
        rf_we = 1;
        // if (br_taken == 1) pc_sel = 3;
        // else pc_sel = 2;
        pc_sel = 2;
      end
      `OPC_LUI_5: begin // U (LUI) Type
        ldx_sel = 0;
        wb_sel = 1;
        rf_we = 1;
        // if (br_taken == 1) pc_sel = 3;
        // else pc_sel = 2;
        pc_sel = 2;
      end
	  7'b1110011: begin // CSRR
        ldx_sel = 0;
        wb_sel = 0;
        rf_we = 0;
        // if (br_taken == 1) pc_sel = 3;
        // else pc_sel = 2;
        pc_sel = 2;
	  end
			default: begin
				ldx_sel = 0;
				wb_sel = 0;
				// pc_sel = 0;
        rf_we = 0;
			end
		endcase
	end
endmodule // WF_CU

module D_CU(instruction, pc, pc_thirty, nop_sel, orange_sel, green_sel);
	input [31:0] instruction, pc;
  // input br_taken, jalr; need br_taken and jalr for later
	output reg pc_thirty, nop_sel, orange_sel, green_sel;

	assign pc_thirty = pc[30];
 
	always @(*) begin
		case(instruction[6:0]) // Assuming no 2-cycle hazard
      7'b0110011: begin
        orange_sel = 0;
				green_sel = 0;
        // if (br_taken || jalr) nop_sel = 1;
        // else nop_sel = 0;
        nop_sel = 0;
      end
      7'b0010011: begin // I and I*
        orange_sel = 0;
				green_sel = 0;
        // if (br_taken || jalr) nop_sel = 1;
        // else nop_sel = 0;
        nop_sel = 0;
      end
      7'b0000011: begin // I type for load
        orange_sel = 0;
				green_sel = 0;
        // if (br_taken || jalr) nop_sel = 1;
        // else nop_sel = 0;
        nop_sel = 0;
      end
      7'b0100011: begin // S-type (store)
        orange_sel = 0;
				green_sel = 0;
        // if (br_taken || jalr) nop_sel = 1;
        // else nop_sel = 0;
        nop_sel = 0;
      end
      7'b1100011: begin // B-type
        orange_sel = 0;
				green_sel = 0;
        // if (br_taken || jalr) nop_sel = 1;
        // else nop_sel = 0;
        nop_sel = 0;
      end
			7'b1101111: begin // J-Type
        orange_sel = 0;
				green_sel = 0;
        // if (br_taken || jalr) nop_sel = 1;
        // else nop_sel = 0;
        nop_sel = 0;
      end
      7'b1100111: begin // JALR
        orange_sel = 0;
				green_sel = 0;
        // if (br_taken || jalr) nop_sel = 1;
        // else nop_sel = 0;
        nop_sel = 0;
      end
      7'b0010111: begin // U (AUIPC) Type
        orange_sel = 0;
				green_sel = 0;
        // if (br_taken || jalr) nop_sel = 1;
        // else nop_sel = 0;
        nop_sel = 0;
      end
      7'b0010111: begin // U (LUI) Type
        orange_sel = 0;
				green_sel = 0;
        // if (br_taken || jalr) nop_sel = 1;
        // else nop_sel = 0;
        nop_sel = 0;
      end
	  7'b1110011: begin // CSRR
	  	orange_sel = 0;
		green_sel = 0;
		nop_sel = 0;
	  end
			default: begin
				orange_sel = 0;
				green_sel = 0;
        nop_sel = 0;
			end
		endcase
	end
endmodule // D_CU

module X_CU(instruction, orange_sel, green_sel, br_un, br_eq, br_lt, a_sel, b_sel, rs2_sel, alu_sel, csr_sel, br_taken, wf_instruction);
	input [31:0] instruction, wf_instruction;
  input br_eq, br_lt;

	output reg br_un, b_sel, csr_sel;
	output reg [1:0] orange_sel, green_sel, a_sel, rs2_sel;
	output reg [3:0] alu_sel;
    output reg br_taken; // add br_taken logic

	always @(*) begin
		case(instruction[6:2])
			`OPC_ARI_RTYPE_5: begin
				case(wf_instruction[6:2])
					`OPC_ARI_RTYPE_5: begin // if R (x) and R (WF)
						if (wf_instruction[11:7] == instruction[19:15]) begin // WF_rd == X_rs1
							orange_sel = 1;
							green_sel = 0;
						end
						else if(wf_instruction[11:7] == instruction[24:20]) begin // WF_rd == X_rs2
							green_sel = 1;
							orange_sel = 0;
						end
						else begin
							orange_sel = 0;
							green_sel = 0;
						end
					end
					`OPC_ARI_ITYPE_5: begin // if R (x) and I (WF)
						if (wf_instruction[11:7] == instruction[19:15]) begin // WF_rd == X_rs1
							orange_sel = 1;
							green_sel = 0;
						end
						else if(wf_instruction[11:7] == instruction[24:20]) begin // WF_rd == X_rs2
							green_sel = 1;
							orange_sel = 0;
						end
						else begin
							orange_sel = 0;
							green_sel = 0;
						end
					end
				endcase
			end
			`OPC_ARI_ITYPE_5: begin
				case(wf_instruction[6:2])
					`OPC_ARI_RTYPE_5: begin // if I (x) and R (WF)
						if (wf_instruction[11:7] == instruction[19:15]) begin // WF_rd == X_rs1
							orange_sel = 1;
							green_sel = 0;
						end
						else begin
							orange_sel = 0;
							green_sel = 0;
						end
					end
					`OPC_ARI_ITYPE_5: begin // if I (x) and I (WF)
						if (wf_instruction[11:7] == instruction[19:15]) begin // WF_rd == X_rs1
							orange_sel = 1;
							green_sel = 0;
						end
						else begin
							orange_sel = 0;
							green_sel = 0;
						end
					end
				endcase
			end
		endcase
	end
 
	always @(*) begin
		case(instruction[6:0])
			7'b0110011: begin // If R-type AKA type = 0
				orange_sel = 0; // forwarding
				green_sel = 0; // forwarding
				br_un = 0;
				a_sel = 0; // forwarding
				b_sel = 0;
				rs2_sel = 0;
				case(instruction[14:12])
          3'b000: begin
            if (instruction[31:25] == 7'b0100000) alu_sel = 4'b0001;
            else alu_sel = 4'b0000;
          end
          3'b111: alu_sel = 4'b0010;
          3'b110: alu_sel = 4'b0011;
          3'b100: alu_sel = 4'b0100;
          3'b001: alu_sel = 4'b0101;
          3'b101: begin
            if (instruction[31:25] == 7'b0100000) alu_sel = 4'b0111;
            else alu_sel = 4'b0110;
          end
          3'b010: alu_sel = 4'b1000;
          3'b011: alu_sel = 4'b1001;
          default: begin
            alu_sel = 4'b0000;
          end
        endcase
				csr_sel = 0;
			end
			7'b0010011: begin // If I-type (I and I*)
				orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				a_sel = 0;
				b_sel = 1;
				rs2_sel = 0;
				case(instruction[14:12])
          3'b000: alu_sel = 4'b0000; // addi
          3'b111: alu_sel = 4'b0010; // andi
          3'b110: alu_sel = 4'b0011; // ori
          3'b100: alu_sel = 4'b0100; // xori
          3'b001: alu_sel = 4'b0101; // slli
          3'b101: begin
            if (instruction[31:25] == 7'b0000000) alu_sel = 4'b0110; // srli
            else alu_sel = 4'b0111; // srai
          end
          3'b010: alu_sel = 4'b1000; // slti
          3'b011: alu_sel = 4'b1001; // sltiu
        endcase
				csr_sel = 0;
			end
      7'b0000011: begin // load (I type)
        orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				a_sel = 0;
				b_sel = 1;
				rs2_sel = 0;
        alu_sel = 4'b0000;
        csr_sel = 0;
      end
      7'b0100011: begin // S type
        orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				a_sel = 0;
				b_sel = 1;
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
      end
			7'b1100011: begin // B-type
				orange_sel = 0;
				green_sel = 0;
        if (instruction[14:12] == 3'b111 || instruction[14:12] == 3'b110) br_un = 1;
        else br_un = 0;
				a_sel = 1;
				b_sel = 1;
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
			end
      7'b1101111: begin // J type (jal)
        orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				a_sel = 1;
				b_sel = 1;
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
      end
      7'b1100111: begin // JALR (I type)
        orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				a_sel = 0;
				b_sel = 0;
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
      end
      7'b0010111: begin // AUIPC
        orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				a_sel = 1;
				b_sel = 1;
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
      end
      7'b0110111: begin // LUI
        orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				a_sel = 0;
				b_sel = 1;
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
      end
	  7'b1110011: begin // CSRR
	  	        if (instruction[14:12] == 3'b001) begin // CSRRW
        			orange_sel = 0;
					green_sel = 0;
					br_un = 0;
					a_sel = 0;
					b_sel = 1;
					rs2_sel = 0;
					alu_sel = 0;
					csr_sel = 1;
				end
				else if (instruction[14:12] == 3'b101) begin // CSRRWI
        			orange_sel = 0;
					green_sel = 0;
					br_un = 0;
					a_sel = 0;
					b_sel = 1;
					rs2_sel = 0;
					alu_sel = 0;
					csr_sel = 0;
				end
	  end
			default: begin
				orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				a_sel = 0;
				b_sel = 0;
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
			end
		endcase
	end

	always @(*) begin
		if (instruction[6:0] == 7'b1100011) begin // If it is a branch inst
			case(instruction[14:12])
				3'b000: begin // beq
					if (br_eq) br_taken = 1;
					else br_taken = 0;
				end
				3'b101: begin // bge
					if (!br_lt) br_taken = 1;
					else br_taken = 0;
				end
				3'b111: begin // bgeu
					if (!br_lt) br_taken = 1;
					else br_taken = 0;
				end
				3'b100: begin // blt
					if (br_lt) br_taken = 1;
					else br_taken = 0;
				end
				3'b110: begin // bltu
					if (br_lt) br_taken = 1;
					else br_taken = 0;
				end
				3'b001: begin // bne
					if (!br_eq) br_taken = 1;
					else br_taken = 0;
				end
			endcase
		end
	end
endmodule // X_CU
