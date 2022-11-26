/*
A saturating incrementer/decrementer.
Adds +/-1 to the input with saturation to prevent overflow.
*/

module sat_updn #(
    parameter WIDTH=2
) (
    input [WIDTH-1:0] in,
    input up, // Taken
    input dn, // Not-taken

    output reg [WIDTH-1:0] out
);
	// MSB is our prediction
	always @(*) begin
		if (up) begin
			case (in)
				2'b00: out = 2'b11;
				2'b01: out = 2'b00;
				2'b10: out = 2'b11;
				2'b11: out = 2'b11;
			endcase
		end
		else if (dn) begin
			case (in)
				2'b00: out = 2'b01;
				2'b01: out = 2'b01;
				2'b10: out = 2'b01;
				2'b11: out = 2'b10;
			endcase
		end
		else begin
			out = in;
		end
	end
endmodule
