module fifo #(
    parameter WIDTH = 8,
    parameter DEPTH = 32,
    parameter POINTER_WIDTH = $clog2(DEPTH)
) (
    input clk, rst,

    // Write side
    input wr_en,
    input [WIDTH-1:0] din,
    output full,

    // Read side
    input rd_en,
    output [WIDTH-1:0] dout,
    output empty
);
    // assign full = 1'b1;
    // assign empty = 1'b0;
    // assign dout = 0;

	// FIFO implementation start
	// Defining states
	parameter EMPTY = 2'b00;
	parameter INTERMEDIATE = 2'b01;
	parameter FULL = 2'b10;

	reg [POINTER_WIDTH-1:0] rd_ptr;
	reg [POINTER_WIDTH-1:0] wr_ptr;
	reg [POINTER_WIDTH:0] data_in_fifo;
	reg [WIDTH-1:0] circular_buffer [DEPTH-1:0];
	reg din_reg[WIDTH-1:0];

	// Output regs
	reg full_reg;
	reg [WIDTH-1:0] dout_reg;
	reg empty_reg;

	reg [1:0] current_state, next_state;

	assign full = full_reg;
	assign dout = dout_reg;
	assign empty = empty_reg;

	always @(posedge clk) begin
		if (rst) begin
			empty_reg <= 1;
			full_reg <= 0;
			wr_ptr <= 0;
			rd_ptr <= 0;
			data_in_fifo <= 0;
		end
		else begin
			if (data_in_fifo == 0 && empty_reg) begin // Empty
				if (wr_en) begin
					circular_buffer[wr_ptr] <= din;

					if (wr_ptr < (2 ** POINTER_WIDTH))
						wr_ptr <= wr_ptr + 1; // Increment write pointer
					else if (wr_ptr == (2 ** POINTER_WIDTH))
						wr_ptr <= 0;
					
					data_in_fifo <= data_in_fifo + 1; // Increment data
					
					empty_reg <= 1'b0;
					full_reg <= 1'b0;
				end
			end
			else if ((data_in_fifo > 0 && data_in_fifo < (2 ** POINTER_WIDTH)) && !empty_reg && !full_reg) begin // Intermediate
				if (wr_en) begin
					circular_buffer[wr_ptr] <= din;
	
					if (wr_ptr < (2 ** POINTER_WIDTH))
						wr_ptr <= wr_ptr + 1; // Increment write pointer
					else if (wr_ptr == (2 ** POINTER_WIDTH))
						wr_ptr <= 0;
				end

				if (rd_en) begin
					dout_reg <= circular_buffer[rd_ptr];
	
					if (rd_ptr < (2 ** POINTER_WIDTH))
						rd_ptr <= rd_ptr + 1; // Increment read pointer
					else if (rd_ptr == (2 ** POINTER_WIDTH))
						rd_ptr <= 0;
				end

				// Change data_in_fifo
				if (wr_en && !rd_en)
					data_in_fifo <= data_in_fifo + 1;
				else if (!wr_en && rd_en)
					data_in_fifo <= data_in_fifo - 1;

				if ((data_in_fifo + 1 == (2 ** POINTER_WIDTH)) && wr_en && !rd_en) begin // Becoming full
					full_reg <= 1'b1;
					empty_reg <= 1'b0;
				end
				else if ((data_in_fifo - 1 == 0) && !wr_en && rd_en) begin // Becoming empty
					full_reg <= 1'b0;
					empty_reg <= 1'b1;
				end
			end
			else if ((data_in_fifo == (2 ** POINTER_WIDTH)) && full_reg) begin // Full
				if (rd_en) begin
					dout_reg <= circular_buffer[rd_ptr];

					if (rd_ptr < (2 ** POINTER_WIDTH))
						rd_ptr <= rd_ptr + 1; // Increment read pointer
					else if (rd_ptr == (2 ** POINTER_WIDTH))
						rd_ptr <= 0;
					data_in_fifo = data_in_fifo - 1; // Decrement data
					full_reg <= 1'b0;
					empty_reg <= 1'b0;
				end
			end
		end
	end
	// FIFO implementation end
endmodule
