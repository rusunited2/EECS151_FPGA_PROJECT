module synchronizer #(parameter WIDTH = 1) (
    input [WIDTH-1:0] async_signal,
    input clk,
    output [WIDTH-1:0] sync_signal
);
    // TODO: Create your 2 flip-flop synchronizer here
    // This module takes in a vector of WIDTH-bit asynchronous
    // (from different clock domain or not clocked, such as button press) signals
    // and should output a vector of WIDTH-bit synchronous signals
    // that are synchronized to the input clk
	reg [WIDTH-1:0] intermediate_reg;
	reg [WIDTH-1:0] inter_to_out;

	always @(posedge clk) begin
		intermediate_reg <= async_signal;
	end

	always @(posedge clk) begin
		inter_to_out <= intermediate_reg;
	end

	assign sync_signal = inter_to_out;

    // Remove this line once you create your synchronizer
    // assign sync_signal = 0;
endmodule
