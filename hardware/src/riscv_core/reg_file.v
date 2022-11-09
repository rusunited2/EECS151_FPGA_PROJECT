module reg_file (
    input clk,
    input we,
    input [4:0] ra1, ra2, wa,
    input [31:0] wd,
    output [31:0] rd1, rd2
);
    parameter DEPTH = 32;
    reg [31:0] mem [0:31];
    // assign rd1 = 32'd0;
    // assign rd2 = 32'd0;

	reg [31:0] rd1_reg;
	reg [31:0] rd2_reg;

	assign rd1 = rd1_reg;
	assign rd2 = rd2_reg;

	always @(posedge clk) begin
	// if writing to x0 just write 0 to it regardless of the value
        if (we)
            mem[wa] <= wd;
    end

    always @(*) begin
		if (ra1 == 0)// i.e. rs1 is x0
        	rd1_reg = 0;
		else
        	rd1_reg = mem[ra1];

		if (ra2 == 0)// i.e. rs2 is x0
        	rd2_reg = 0;
		else
        	rd2_reg = mem[ra2];
    end
endmodule
