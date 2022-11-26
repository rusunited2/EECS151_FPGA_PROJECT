`timescale 1ns/1ns
`define CLK_PERIOD 8

module bp_cache_tb();
    // Generate 125 Mhz clock
    reg clk = 0;
	reg reset, we;
	reg [31:0] ra0, ra1, wa, din;
	wire [31:0] dout0, dout1;
	wire hit0, hit1;
    always #(`CLK_PERIOD/2) clk = ~clk;
	bp_cache #(
		.AWIDTH(32),  // Address bit width
		.DWIDTH(32),  // Data bit width
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
		assert(dout0 == 2'b00) else $error("dout0 should be 00 but is %b instead", dout0);
        if (dout0 == 2'b00) $display("Test case 1 passed!");

		assert(dout1 == 2'b00) else $error("dout1 should be 00 but is %b instead", dout1);
        if (dout1 == 2'b00) $display("Test case 2 passed!");
		

		`ifndef IVERILOG
            $vcdplusoff;
        `endif
        $finish();
    end
endmodule

	
