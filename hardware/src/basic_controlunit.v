module WF_CU(instruction);
	input [31:0] instruction;
	output reg rf_we;

	reg [2:0] type; // if R: type = 0
 
	always @(*) begin
		case(type) // Assuming no 2-cycle hazard
			3'd0: begin // If R-type AKA type = 0
				addr_sel = 0;
				ldx_sel = 0;
				wb_sel = 0;
				pc_sel = 0;
			end
			3'd1: begin // If I-type AKA type = 1
				addr_sel = 0;
				ldx_sel = 0;
				wb_sel = 0;
				pc_sel = 0;
			end
			3'd2: begin // If S-type AKA type = 2
				addr_sel = 0;
				ldx_sel = 0;
				wb_sel = 0;
				pc_sel = 0;
			end
			3'd3: begin // If B-type AKA type = 3
				addr_sel = 0;
				ldx_sel = 0;
				wb_sel = 0;
				pc_sel = 0;
			end
			3'd4: begin // If U-type AKA type = 4
				addr_sel = 0;
				ldx_sel = 0;
				wb_sel = 0;
				pc_sel = 0;
			end
			3'd5: begin // If J-type AKA type = 5
				addr_sel = 0;
				ldx_sel = 0;
				wb_sel = 0;
				pc_sel = 0;
			end
			default: begin
				addr_sel = 0;
				ldx_sel = 0;
				wb_sel = 0;
				pc_sel = 0;
			end
		endcase
	end
endmodule // WF_CU

module D_CU(instruction, pc, rf_we, pc_thirty, nop_sel, orange_sel, green_sel);
	input [31:0] instruction, pc;
	output reg rf_we, pc_thirty, nop_sel, orange_sel, green_sel;

	reg [2:0] type; // if R: type = 0

	assign pc_thirty = pc[30];
	// TODO: If nop then assign nop_sel to 1
	// if (nop) begin
	// assign nop_sel = 1;
	// else
	// assign nop_sel = 0;
 
	always @(*) begin
		case(type) // Assuming no 2-cycle hazard
			3'd0: begin // If R-type AKA type = 0
				rf_we = 1;
				orange_sel = 0;
				green_sel = 0;
			end
			3'd1: begin // If I-type AKA type = 1
				rf_we = 1;
				orange_sel = 0;
				green_sel = 0;
			end
			3'd2: begin // If S-type AKA type = 2
				rf_we = 0;
				orange_sel = 0;
				green_sel = 0;
			end
			3'd3: begin // If B-type AKA type = 3
				rf_we = 0;
				orange_sel = 0;
				green_sel = 0;
			end
			3'd4: begin // If U-type AKA type = 4
				rf_we = 1;
				orange_sel = 0;
				green_sel = 0;
			end
			3'd5: begin // If J-type AKA type = 5
				rf_we = 1;
				orange_sel = 0;
				green_sel = 0;
			end
			default: begin
				rf_we = 0;
				orange_sel = 0;
				green_sel = 0;
			end
		endcase
	end
endmodule // D_CU

module X_CU(instruction, orange_sel, green_sel, br_un, imm_sel, a_sel, b_sel, rs2_sel, alu_sel, csr_sel);
	input [31:0] instruction;

	output reg br_un, b_sel, csr_sel;
	output [1:0] reg orange_sel, green_sel, a_sel, rs2_sel;
	output [2:0] reg imm_sel;
	output [3:0] reg alu_sel;

	reg [2:0] type; // if R: type = 0
 
	always @(*) begin
		case(type) // Assuming no forwarding
			3'd0: begin // If R-type AKA type = 0
				orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				imm_sel = 0;
				a_sel = 0;
				b_sel = 0:
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
			end
			3'd1: begin // If I-type AKA type = 1
				orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				imm_sel = 0;
				a_sel = 0;
				b_sel = 0:
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
			end
			3'd2: begin // If S-type AKA type = 2
				orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				imm_sel = 0;
				a_sel = 0;
				b_sel = 0:
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
			end
			3'd3: begin // If B-type AKA type = 3
				orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				imm_sel = 0;
				a_sel = 0;
				b_sel = 0:
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
			end
			3'd4: begin // If U-type AKA type = 4
				orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				imm_sel = 0;
				a_sel = 0;
				b_sel = 0:
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
			end
			3'd5: begin // If J-type AKA type = 5
				orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				imm_sel = 0;
				a_sel = 0;
				b_sel = 0:
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
			end
			default: begin
				orange_sel = 0;
				green_sel = 0;
				br_un = 0;
				imm_sel = 0;
				a_sel = 0;
				b_sel = 0:
				rs2_sel = 0;
				alu_sel = 0;
				csr_sel = 0;
			end
		endcase
	end
endmodule // X_CU
