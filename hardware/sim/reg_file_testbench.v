`timescale 1ns/1ns
`define CLK_PERIOD 8

module reg_file_testbench();
    // Generate 125 Mhz clock
    reg clk = 0;
    always #(`CLK_PERIOD/2) clk = ~clk;

    reg we;
    reg [4:0] ra1, ra2, wa;
    reg [31:0] wd;
    wire [31:0] rd1, rd2;
    reg_file rf (
        .clk(clk),
        .we(we),
        .ra1(ra1), .ra2(ra2), .wa(wa),
        .wd(wd),
        .rd1(rd1), .rd2(rd2)
    );
    
    initial begin
        `ifdef IVERILOG
            $dumpfile("basic_modules_testbench.fst");
            $dumpvars(0, basic_modules_testbench);
        `endif
        `ifndef IVERILOG
            $vcdpluson;
        `endif

        $display("Testing RegFile:");

		wd = 32'd123;
		wa = 32'd7;
		we = 1'd1;
		#(1000)

		ra1 = 0;
		ra2 = 5'd7;
		#(1000)

        assert(rd1 == 0) else $error("x0 should be 0 but is %d instead", rd1);
        if (rd1 == 0) $display("reg_file: Test case 1 passed!");

        assert(rd2 == 32'd123) else $error("x7 should be 123 but is %d instead", rd2);
        if (rd2 == 32'd123) $display("reg_file: Test case 2 passed!");

		ra1 = 5'd7;
		ra2 = 0;
		#(1000)

        assert(rd1 == 32'd123) else $error("x7 should be 123 but is %d instead", rd1);
        if (rd1 == 32'd123) $display("reg_file: Test case 3 passed!");

        assert(rd2 == 0) else $error("x0 should be 0 but is %d instead", rd2);
        if (rd2 == 0) $display("reg_file: Test case 4 passed!");

        `ifndef IVERILOG
            $vcdplusoff;
        `endif
        $finish();
    end
endmodule
