module debouncer #(
    parameter WIDTH              = 1,
    parameter SAMPLE_CNT_MAX     = 62500,
    parameter PULSE_CNT_MAX      = 200,
    parameter WRAPPING_CNT_WIDTH = $clog2(SAMPLE_CNT_MAX),
    parameter SAT_CNT_WIDTH      = $clog2(PULSE_CNT_MAX) + 1
) (
    input clk,
    input [WIDTH-1:0] glitchy_signal,
    output [WIDTH-1:0] debounced_signal
);
    // TODO: fill in neccesary logic to implement the wrapping counter and the saturating counters
    // Some initial code has been provided to you, but feel free to change it however you like
    // One wrapping counter is required
    // One saturating counter is needed for each bit of glitchy_signal
    // You need to think of the conditions for reseting, clock enable, etc. those registers
    // Refer to the block diagram in the spec

	// Sample Pulse Generator begin
	reg [WRAPPING_CNT_WIDTH-1:0] wrapping_counter = 1'd0;
	reg SP_gen_output;
	always @(posedge clk) begin
		// Wrapping counter
	    if (wrapping_counter == SAMPLE_CNT_MAX) begin // 62500 or 62499? SAMPLE_CNT_MAX or SAMPLE_CNT_MAX-1
	    	wrapping_counter <= 1'd0;
		end
        else
	      	wrapping_counter <= (wrapping_counter + 1'd1);
		// Wrapping counter

		// Comparator
		if (wrapping_counter == SAMPLE_CNT_MAX)
			SP_gen_output <= 1'd1;
		else
			SP_gen_output <= 1'd0;
		// Comparator
	end
	// Sample Pulse Generator end

    // Remove this line once you have created your debouncer
    // assign debounced_signal = 0;

	// Debouncer begin
	// reg new = 0;
	reg [WIDTH-1:0] reset;
	reg [WIDTH-1:0] enable;
	reg [SAT_CNT_WIDTH-1:0] saturating_counter [WIDTH-1:0];
	reg [WIDTH-1:0] debounced_reg;


	// For loop for multiple Saturating counter
	genvar i;

	generate
		for(i = 0; i < WIDTH; i = i + 1) begin
			always @(posedge clk) begin
				enable[i] <= SP_gen_output & glitchy_signal[i]; // Set enable

				reset[i] <= ~glitchy_signal[i]; // Set reset

				// If reset bit is set, then reset counter
				if (reset[i] == 1'd1)
					saturating_counter[i] = 1'd0;

				// If enable bit is set and reset bit is not set
				if ((enable[i] == 1) & reset[i] == 1'd0) begin
					// Saturating counter
					if (saturating_counter[i] == PULSE_CNT_MAX) begin // 62500 or 62499? SAMPLE_CNT_MAX or SAMPLE_CNT_MAX-1
						saturating_counter[i] <= saturating_counter[i];
					end
					else
					  	saturating_counter[i] <= (saturating_counter[i]+ 1);
					// Saturating counter
				end

				// Comparator
				if (saturating_counter[i] == PULSE_CNT_MAX)
					debounced_reg[i] <= 1'd1;
				else
					debounced_reg[i] <= 1'd0;
				// Comparator
			end
		end
	endgenerate

	assign debounced_signal = debounced_reg;
	// For loop end
	// Debouncer end
endmodule
