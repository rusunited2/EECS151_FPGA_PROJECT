module uart_transmitter #(
    parameter CLOCK_FREQ = 125_000_000,
    parameter BAUD_RATE = 115_200)
(
    input clk,
    input reset,

    input [7:0] data_in,
    input data_in_valid,
    output data_in_ready,

    output serial_out
);
    // See diagram in the lab guide
    localparam  SYMBOL_EDGE_TIME    =   CLOCK_FREQ / BAUD_RATE;
    localparam  CLOCK_COUNTER_WIDTH =   $clog2(SYMBOL_EDGE_TIME);

	// UART transmitter start
    localparam SAMPLE_TIME = SYMBOL_EDGE_TIME / 2;

    wire symbol_edge;
    wire sample;
    wire start;
    wire tx_running;

	reg [3:0] counter;
	reg serial_out_reg;
	reg [7:0] data_in_reg;
    reg [3:0] bit_counter;
    reg [CLOCK_COUNTER_WIDTH-1:0] clock_counter;

    //--|Signal Assignments|------------------------------------------------------

    // Goes high at every symbol edge
    /* verilator lint_off WIDTH */
    assign symbol_edge = clock_counter == (SYMBOL_EDGE_TIME - 1);
    /* lint_on */

    // Goes high halfway through each symbol
    /* verilator lint_off WIDTH */
    assign sample = clock_counter == SAMPLE_TIME;
    /* lint_on */

    // Goes high when it is time to start transmitting a new character
	assign start = data_in_valid && !tx_running;

    // Goes high while we are sending a character
    assign tx_running = bit_counter != 4'd0;

    // Outputs
    assign serial_out = serial_out_reg;
	assign data_in_ready = !tx_running;

    //--|Counters|----------------------------------------------------------------

    // Counts cycles until a single symbol is done
    always @ (posedge clk) begin
        clock_counter <= (start || reset || symbol_edge) ? 0 : clock_counter + 1;
    end

    // Counts down and up from 10 bits for every character
    always @ (posedge clk) begin
        if (reset) begin
            bit_counter <= 0;
            counter <= 10;
        end else if (start) begin
            bit_counter <= 10;
            counter <= 0;
        end else if (symbol_edge && tx_running) begin
            bit_counter <= bit_counter - 1;
            counter <= counter + 1;
        end
    end

    //--|Shift Register|----------------------------------------------------------
    always @(posedge clk) begin
		if (sample && tx_running) begin
			if (counter == 0) begin
				serial_out_reg <= 0;
			end
			else if (counter == 9) begin
				serial_out_reg <= 1;
				data_in_reg <= data_in[7:0];
			end
			else
				serial_out_reg <= data_in_reg[counter-1];
		end
		else if (!tx_running) begin
			data_in_reg <= data_in[7:0];
			serial_out_reg <= 1'b1;
		end
    end
	// UART transmitter end
endmodule