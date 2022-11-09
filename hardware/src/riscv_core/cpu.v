module cpu #(
    parameter CPU_CLOCK_FREQ = 50_000_000,
    parameter RESET_PC = 32'h4000_0000,
    parameter BAUD_RATE = 115200
) (
    input clk,
    input rst,
    input serial_in,
    output serial_out
);
    // BIOS Memory
    // Synchronous read: read takes one cycle
    // Synchronous write: write takes one cycle
    wire [11:0] bios_addra, bios_addrb;
    wire [31:0] bios_douta, bios_doutb;
    wire bios_ena, bios_enb;
    bios_mem bios_mem (
      .clk(clk),
      .ena(bios_ena),
      .addra(bios_addra),
      .douta(bios_douta),
      .enb(bios_enb),
      .addrb(bios_addrb),
      .doutb(bios_doutb)
    );

    // Data Memory
    // Synchronous read: read takes one cycle
    // Synchronous write: write takes one cycle
    // Write-byte-enable: select which of the four bytes to write
    wire [13:0] dmem_addr;
    wire [31:0] dmem_din, dmem_dout;
    wire [3:0] dmem_we;
    wire dmem_en;
    dmem dmem (
      .clk(clk),
      .en(dmem_en),
      .we(dmem_we),
      .addr(dmem_addr),
      .din(dmem_din),
      .dout(dmem_dout)
    );

    // Instruction Memory
    // Synchronous read: read takes one cycle
    // Synchronous write: write takes one cycle
    // Write-byte-enable: select which of the four bytes to write
    wire [31:0] imem_dina, imem_doutb;
    wire [13:0] imem_addra, imem_addrb;
    wire [3:0] imem_wea;
    wire imem_ena;
    imem imem (
      .clk(clk),
      .ena(imem_ena),
      .wea(imem_wea),
      .addra(imem_addra),
      .dina(imem_dina),
      .addrb(imem_addrb),
      .doutb(imem_doutb)
    );

    // Register file
    // Asynchronous read: read data is available in the same cycle
    // Synchronous write: write takes one cycle
    wire we;
    wire [4:0] ra1, ra2, wa;
    wire [31:0] wd;
    wire [31:0] rd1, rd2;
    reg_file rf (
        .clk(clk),
        .we(we),
        .ra1(ra1), .ra2(ra2), .wa(wa),
        .wd(wd),
        .rd1(rd1), .rd2(rd2)
    );

    // On-chip UART
    //// UART Receiver
    wire [7:0] uart_rx_data_out;
    wire uart_rx_data_out_valid;
    wire uart_rx_data_out_ready;
    //// UART Transmitter
    wire [7:0] uart_tx_data_in;
    wire uart_tx_data_in_valid;
    wire uart_tx_data_in_ready;
    uart #(
        .CLOCK_FREQ(CPU_CLOCK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) on_chip_uart (
        .clk(clk),
        .reset(rst),

        .serial_in(serial_in),
        .data_out(uart_rx_data_out),
        .data_out_valid(uart_rx_data_out_valid),
        .data_out_ready(uart_rx_data_out_ready),

        .serial_out(serial_out),
        .data_in(uart_tx_data_in),
        .data_in_valid(uart_tx_data_in_valid),
        .data_in_ready(uart_tx_data_in_ready)
    );

    reg [31:0] tohost_csr = 0;

    // TODO: Your code to implement a fully functioning RISC-V core
    // Add as many modules as you want
    // Feel free to move the memory modules around
	// 1. WF ------------------------------------------------------

	// 1.1: pc_mux
	wire [2:0] pc_mux_sel;
  	wire [31:0] pc_mux_in0, pc_mux_in1, pc_mux_in2, pc_mux_in3, pc_mux_in4;
	wire pc_mux_out;
	EIGHT_INPUT_MUX pc_mux (
		.sel(pc_mux_sel),
		.in0(pc_mux_in0),
		.in1(pc_mux_in1),
		.in2(pc_mux_in2),
		.in3(pc_mux_in3),
		.in4(pc_mux_in4),
		.in5(0),
		.in6(0),
		.in7(0),
		.out(pc_mux_out)
	);

	// 1.2: pc_plus_four
    wire [31:0] pc_plus_four_in0;
    wire [31:0] pc_plus_four_out;
	ADDER pc_plus_four (
		.in0(pc_plus_four_in0),
		.in1(32'd4),
		.out(pc_plus_four_out)
	);

	// 1.3: pc_register
	wire [31:0] pc_register_q, pc_register_d;
	REGISTER #(
        .N(32)
    ) pc_register (
        .q(pc_register_q),
		.d(pc_register_d),
		.clk(clk)
    );

	// Wiring for WF stage
	assign pc_mux_out = pc_plus_four_in0;
	assign pc_plus_four_out = pc_mux_in2;

	assign pc_mux_out = pc_register_d;
	assign pc_register_q = pc_mux_in4; 

	assign pc_mux_out = bios_addra;
	assign pc_mux_out = imem_addrb;


	// 2. D -------------------------------------------------------
	wire pc_thirty_mux_sel;
  	wire [31:0] pc_thirty_mux_in0, pc_thirty_mux_in1;
	wire pc_thirty_mux_out;
	TWO_INPUT_MUX pc_thirty_mux (
		.sel(pc_thirty_mux_sel),
		.in0(pc_thirty_mux_in0),
		.in1(pc_thirty_mux_in1),
		.out(pc_thirty_mux_out)
	);

	wire nop_mux_sel;
  	wire [31:0] nop_mux_in0, nop_mux_in1; // TODO: nop_mux_in1 needs to be the nop[31:0]
	wire nop_mux_out;
	TWO_INPUT_MUX nop_mux (
		.sel(nop_mux_sel),
		.in0(nop_mux_in0),
		.in1(32'b0000_0000_0000_0000_0000_0000_0001_0011), // nop_mux_in1 = addi x0, x0, 0 = 00000000000000000000000000010011
		.out(nop_mux_out)
	);
	
	// Wiring for D stage
	assign bios_douta = pc_thirty_mux_in0;
	assign imem_doutb = pc_thirty_mux_in1;
	
	assign pc_thirty_mux_out = nop_mux_in0;

	// 3. X -------------------------------------------------------
endmodule
