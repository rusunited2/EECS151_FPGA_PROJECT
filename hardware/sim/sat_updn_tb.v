`timescale 1ns/1ns

module sat_updn_tb();
	reg [1:0] in;
	reg up, dn;
	wire [1:0] out;

	sat_updn #(
		.WIDTH(2)
	) sat_updn (
		.in(in),
		.up(up),
		.dn(dn),
		.out(out)
	);

	initial begin
        `ifdef IVERILOG
            $dumpfile("sat_updn_tb.fst");
            $dumpvars(0, sat_updn_tb);
        `endif
        `ifndef IVERILOG
            $vcdpluson;
        `endif

		in = 2'b01;
		up = 0;
		dn = 0;
		#(1000)
		assert(out == 2'b01) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b01) $display("Test case 1 passed!");

		in = 2'b01;
		up = 1;
		dn = 0;
		#(1000)
		assert(out == 2'b00) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b00) $display("Test case 2 passed!");

		in = 2'b00;
		up = 1;
		dn = 0;
		#(1000)
		assert(out == 2'b11) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b11) $display("Test case 3 passed!");

		in = 2'b11;
		up = 1;
		dn = 0;
		#(1000)
		assert(out == 2'b11) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b11) $display("Test case 4 passed!");

		in = 2'b11;
		up = 0;
		dn = 1;
		#(1000)
		assert(out == 2'b10) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b10) $display("Test case 5 passed!");

		in = 2'b11;
		up = 0;
		dn = 1;
		#(1000)
		assert(out == 2'b10) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b10) $display("Test case 6 passed!");

		in = 2'b10;
		up = 1;
		dn = 0;
		#(1000)
		assert(out == 2'b11) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b11) $display("Test case 7 passed!");

		in = 2'b11;
		up = 0;
		dn = 1;
		#(1000)
		assert(out == 2'b10) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b10) $display("Test case 8 passed!");

		in = 2'b10;
		up = 0;
		dn = 1;
		#(1000)
		assert(out == 2'b01) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b01) $display("Test case 9 passed!");

		in = 2'b01;
		up = 0;
		dn = 1;
		#(1000)
		assert(out == 2'b01) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b01) $display("Test case 10 passed!");

		in = 2'b01;
		up = 1;
		dn = 0;
		#(1000)
		assert(out == 2'b00) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b00) $display("Test case 11 passed!");

		in = 2'b00;
		up = 0;
		dn = 1;
		#(1000)
		assert(out == 2'b01) else $error("out should be 0 but is %b instead", out);
        if (out == 2'b01) $display("Test case 12 passed!");


		`ifndef IVERILOG
            $vcdplusoff;
        `endif
        $finish();
    end
endmodule
