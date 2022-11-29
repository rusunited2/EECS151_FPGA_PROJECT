`timescale 1ns/1ns
`define CLK_PERIOD 8

module bp_cache_tb();
    // Generate 125 Mhz clock
    reg clk = 0;
	reg reset, we;
	reg [1:0] din;
	reg [29:0] ra0, ra1, wa;
	// wire [31:0] dout0, dout1;
	wire [1:0] dout0, dout1;
	wire hit0, hit1;
    always #(`CLK_PERIOD/2) clk = ~clk;
	bp_cache #(
		.AWIDTH(30),  // Address bit width
		.DWIDTH(2),  // Data bit width
		.LINES(128)   // Number of cache lines
	) bp_cache (
		.clk(clk),
		.reset(reset),
		.ra0(ra0),
		.dout0(dout0),
		.hit0(hit0),
		.ra1(ra1),
		.dout1(dout1),
		.hit1(hit1),
		.wa(wa),
		.din(din),
		.we(we)
	);

    initial begin
        `ifdef IVERILOG
            $dumpfile("bp_cache_tb.fst");
            $dumpvars(0, bp_cache_tb);
        `endif
        `ifndef IVERILOG
            $vcdpluson;
            $vcdplusmemon;
        `endif

		reset = 1;
		#(1000)
		// assert(valid[0] == 1'b1) else $error("valid should be 0 but is %b instead", valid[0]);
        // if (valid[0] == 1'b0) $display("Test case 1 passed!");
		reset = 0;

		repeat (1) @(posedge clk);
		ra0 = 30'b010101010101010101010101111111;
		repeat (1) @(posedge clk);
		assert(hit0 == 1'b0) else $error("hit0 should be 0 but is %b instead", hit0); // Cache Read Miss
		if (hit0 == 1'b0) $display("Test case 1 passed! (Cache Read Miss)");
		ra0 = 30'b0;
		
		ra1 = 30'b010101010101010101010101111111;
		repeat (1) @(posedge clk);
		assert(hit1 == 1'b0) else $error("hit1 should be 0 but is %b instead", hit1); // Cache Write Miss
		if (hit1 == 1'b0) $display("Test case 2 passed! (Cache Write Miss)");
		wa = 30'b010101010101010101010101111111;
		din = 2'b01;
		we = 1;
		repeat (1) @(posedge clk);

		ra0 = 30'b010101010101010101010101111111;
		ra1 = 30'b0;
		wa = 30'b0;
		din = 2'b00;
		we = 0;
		repeat (1) @(posedge clk);
		assert(hit0 == 1'b1) else $error("hit0 should be 1 but is %b instead", hit0);
		assert(dout0 == 2'b01) else $error("dout0 should be 01 but is %b instead", hit0);
		if (hit0 == 1'b1 && dout0 == 2'b01) $display("Test case 3 passed! (Cache Read Hit)"); // Cache Read Hit

		ra1 = 30'b010101010101010101010101111111;
		repeat (1) @(posedge clk);
		assert(hit1 == 1'b1) else $error("hit1 should be 1 but is %b instead", hit1); // Cache Write Hit
		if (hit1 == 1'b1) $display("Test case 4 passed! (Cache Write Hit)");

		wa = 30'b010101010101010101010101111111;
		din = 2'b11;
		we = 1;
		repeat (1) @(posedge clk); // Cache Write Hit
		we = 0;
		ra0 = 30'b010101010101010101010101111111;
		assert(hit0 == 1'b1) else $error("hit0 should be 1 but is %b instead", hit0); // Cache Read Miss
		if (hit0 == 1'b1) $display("Test case 5 passed! (Cache Write Hit/Read Miss)");

		// Cache Read Miss
		ra0 = 30'b110101010101010101010101111111;
		repeat (1) @(posedge clk);
		assert(hit0 == 1'b0) else $error("hit0 should be 0 but is %b instead", hit0);
		if (hit0 == 1'b0) $display("Test case 6 passed! (Cache Eviction)");
		ra1 = 30'b110101010101010101010101111111;
		repeat (1) @(posedge clk);
		assert(hit1 == 1'b0) else $error("hit1 should be 0 but is %b instead", hit1);
		if (hit1 == 1'b0) $display("Test case 7 passed! (Cache Eviction)");
		wa = 30'b110101010101010101010101111111;
		din = 2'b10;
		we = 1;
		repeat (1) @(posedge clk);
		we = 0;
		ra0 = 30'b110101010101010101010101111111;
		repeat (1) @(posedge clk);
		assert(hit0 == 1'b1) else $error("hit0 should be 1 but is %b instead", hit0);
		assert(dout0 == 2'b10) else $error("dout0 should be 10 but is %b instead", dout0);
		if (hit0 == 1'b1 && dout0 == 2'b10) $display("Test case 8 passed! (Cache Read After Eviction)");



		reset = 1;
		repeat (1) @(posedge clk);
		reset = 0;
		ra0 = 30'b010101010101010101010101111111;
		repeat (1) @(posedge clk);
		assert(hit0 == 1'b0) else $error("hit0 should be 0 but is %b instead", hit0); // Cache Reset
		if (hit0 == 1'b0) $display("Test case 9 passed! (Cache Reset)");

		`ifndef IVERILOG
            $vcdplusoff;
        `endif
        $finish();
    end
endmodule

	
