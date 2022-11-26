/*
A cache module for storing branch prediction data.

Inputs: 2 asynchronous read ports and 1 synchronous write port.
Outputs: data and cache hit (for each read port)
*/

module bp_cache #(
    parameter AWIDTH=32,  // Address bit width
    parameter DWIDTH=32,  // Data bit width
    parameter LINES=128   // Number of cache lines
) (
    input clk,
    input reset,

    // IO for 1st read port
    input [AWIDTH-1:0] ra0,
    output [DWIDTH-1:0] dout0,
    output hit0,

    // IO for 2nd read port
    input [AWIDTH-1:0] ra1,
    output [DWIDTH-1:0] dout1,
    output hit1,

    // IO for write port
    input [AWIDTH-1:0] wa,
    input [DWIDTH-1:0] din,
    input we

);
	localparam integer size_entry = ((AWIDTH - $clog2(LINES)) + 1 + DWIDTH); // # of cols = 30 - 7 + 1 + 2 = 26
	localparam integer index = $clog2(LINES); // # of rows = 7
	reg [size_entry-1:0] cache [0:(index - 1)];
	reg [size_entry-1:0] read_cache_zero;
	reg [size_entry-1:0] read_cache_one;

	reg [DWIDTH-1:0] dout0_reg, dout1_reg;
	reg hit0_reg, hit1_reg;

	assign dout0 = dout0_reg;
	assign dout1 = dout1_reg;
	assign hit0 = hit0_reg;
	assign hit1 = hit1_reg;

	// Sync writes
	always @(posedge clk) begin
		if (reset) begin
			for (int i = 0; i < LINES; i++) begin
				cache[i] <= 0;
			end
		end
		else begin
			if (we) // If it's a branch instruction
				cache[wa[(index-1):0]] <= {wa[(AWIDTH-1):index], 1'b1, din};
		end
	end

	// Async reads
	always @(*) begin
		read_cache_zero = cache[ra0[(index-1):0]];
		read_cache_one = cache[ra1[(index-1):0]];

		if ((read_cache_zero[DWIDTH] == 1'b1) && (ra0[(AWIDTH-1):index] == read_cache_zero[size_entry-1:(DWIDTH+1)])) begin // If Valid == 1 AND ra0_tag == read_cache_zero_tag
			dout0_reg = read_cache_zero[(DWIDTH-1):0];
			hit0_reg = 1;
			$display("%b", dout0_reg);
		end
		else if (ra0 == ra1) begin
			dout0_reg = din;
			hit0_reg = 1;
		end
		else begin
			// $display("%b", dout0_reg);
			dout0_reg = 0;
			hit0_reg = 0; // if miss does not matter
		end

		if ((read_cache_one[DWIDTH] == 1'b1) && (ra1[(AWIDTH-1):index] == read_cache_one[size_entry-1:(DWIDTH+1)])) begin
			dout1_reg = read_cache_one[(DWIDTH-1):0];
			hit1_reg = 1;
			// $display("%b", dout0_reg);
		end
		else if (we) begin
			dout1_reg = din;
			hit1_reg = 1;
		end
		else begin
			dout1_reg = 0;
			hit1_reg = 0;
		end
	end

    // TODO: Your code
    // assign dout0 = '0;
    // assign dout1 = '0;
    // assign hit0  = 1'b0;
    // assign hit1  = 1'b0;
endmodule
