`timescale 1ns/1ns

module basic_modules_testbench();
    
    wire [31:0] adder_out;
	reg [31:0] alu_out;
    reg [31:0] in0, in1; // Adder regs
    reg [31:0] rs1, rs2; // ALU regs
    reg br_un, br_lt;
    wire br_eq;
	reg [3:0] alu_sel; // ALU regs
    reg [31:0] instruction; // for immediate generator
    reg [31:0] imm; // for immediate generator

    ADDER add_test (.in0(in0), .in1(in1), .out(adder_out));
	ALU alu_test (.alu_sel(alu_sel), .rs1(rs1), .rs2(rs2), .out(alu_out));
    BRANCH_COMPARATOR branch_test (.rs1(rs1), .rs2(rs2), .br_un(br_un), .br_eq(br_eq), .br_lt(br_lt));
    IMM_GEN immediate_test (.inst(instruction), .imm(imm));

    
    initial begin
        `ifdef IVERILOG
            $dumpfile("basic_modules_testbench.fst");
            $dumpvars(0, basic_modules_testbench);
        `endif
        `ifndef IVERILOG
            $vcdpluson;
        `endif

        $display("Testing Adder:");

        in0 = 32'd4;
        in1 = 32'd5;
		#(1000)

        assert(adder_out == 9) else $error("Out should be 9 but is %d instead", adder_out);
        if (adder_out == 9) $display("ADD: Test case 1 passed!");

        $display("Testing ALU:");
		alu_sel = 4'd0;
        rs1 = 32'd525;
        rs2 = 32'd273;
		#(1000)

        assert(alu_out == rs1 + rs2) else $error("Out should be 798 but is %d instead", alu_out);
        if (alu_out == rs1 + rs2) $display("ALU: Test case 2.1 passed!");

		alu_sel = 4'd1;
        rs1 = 32'd525;
        rs2 = 32'd273;
		#(1000)

        assert(alu_out == rs1 - rs2) else $error("Out should be 798 but is %d instead", alu_out);
        if (alu_out == rs1 - rs2) $display("ALU: Test case 2.2 passed!");

        $display("Testing Branch_comparator:");
		br_un = 1'd0;
        rs1 = 32'd1;
        rs2 = 32'd1;
		#(1000)

        assert(br_eq == 1) else $error("br_eq should be 1 but is %d instead", br_eq);
        if (br_eq == 1) $display("Branch_comparator: Test case 3 passed!");
        assert(br_lt == 0) else $error("br_lt should be 0 but is %d instead", br_lt);
        if (br_lt == 0) $display("Branch_comparator: Test case 4 passed!");


        $display("Testing Immediate Generator:");
		instruction = 32'b1111_1100_1110_0000_1000_0111_1001_0011;
		#(1000)

        assert(imm == 32'b11111111111111111111111111001110) else $error("imm should be 1111_1111_1111_1111_1111_1111_1100_1110 but is %b instead", imm);
        if (imm == 32'b11111111111111111111111111001110) $display("Imm gen: Test case 4 passed!");

        instruction = 32'b0_000000_01010_10011_000_1000_0_1100011;
        #(1000)

        assert(imm == 32'b00000000000000000000000000010000) else $error("imm should be 00000000000000000000000000010000 but is %b instead", imm);
        if (imm == 32'b00000000000000000000000000010000) $display("Imm gen: Test case 5 passed!");

        `ifndef IVERILOG
            $vcdplusoff;
        `endif
        $finish();
    end
endmodule
