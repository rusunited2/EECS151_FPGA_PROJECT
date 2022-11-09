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
    // wire [7:0] uart_rx_data_out;
    // wire uart_rx_data_out_valid;
    // wire uart_rx_data_out_ready;
    // //// UART Transmitter
    // wire [7:0] uart_tx_data_in;
    // wire uart_tx_data_in_valid;
    // wire uart_tx_data_in_ready;
    // uart #(
    //     .CLOCK_FREQ(CPU_CLOCK_FREQ),
    //     .BAUD_RATE(BAUD_RATE)
    // ) on_chip_uart (
    //     .clk(clk),
    //     .reset(rst),

    //     .serial_in(serial_in),
    //     .data_out(uart_rx_data_out),
    //     .data_out_valid(uart_rx_data_out_valid),
    //     .data_out_ready(uart_rx_data_out_ready),

    //     .serial_out(serial_out),
    //     .data_in(uart_tx_data_in),
    //     .data_in_valid(uart_tx_data_in_valid),
    //     .data_in_ready(uart_tx_data_in_ready)
    // );

    reg [31:0] tohost_csr = 0;

    // TODO: Your code to implement a fully functioning RISC-V core
    // Add as many modules as you want
    // Feel free to move the memory modules around
	// 1. WF ------------------------------------------------------

	// 1.1: pc_mux
    // TO DO: Call control logic for pc_mux_sel
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
	wire [31:0] pc_register_d;
    reg [31:0] pc_register_q;
    always @(posedge clk) begin
        pc_register_q <= pc_register_d;
    end

	// Wiring for WF stage
    assign pc_plus_four_in0 = pc_mux_out;
    assign pc_mux_in2 = pc_plus_four_out;

    assign pc_register_d = pc_mux_out;
    assign pc_mux_in4 = pc_register_q;

    assign bios_addra = pc_mux_out;
    assign imem_addrb = pc_mux_out;


	// 2. D -------------------------------------------------------
	wire pc_thirty_mux_sel;
  	wire [31:0] pc_thirty_mux_in0, pc_thirty_mux_in1;
	wire [31:0] pc_thirty_mux_out;
	TWO_INPUT_MUX pc_thirty_mux (
		.sel(pc_thirty_mux_sel),
		.in0(pc_thirty_mux_in0),
		.in1(pc_thirty_mux_in1),
		.out(pc_thirty_mux_out)
	);

	wire nop_mux_sel;
  	wire [31:0] nop_mux_in0;
	wire [31:0] nop_mux_out;
	TWO_INPUT_MUX nop_mux (
		.sel(nop_mux_sel),
		.in0(nop_mux_in0),
		.in1(32'b0000_0000_0000_0000_0000_0000_0001_0011), // nop_mux_in1 = addi x0, x0, 0 = 00000000000000000000000000010011
		.out(nop_mux_out)
	);

	wire rs1_mux_sel;
  	wire [31:0] rs1_mux_in0, rs1_mux_in1;
	wire [31:0] rs1_mux_out;
	TWO_INPUT_MUX rs1_mux (
		.sel(rs1_mux_sel),
		.in0(rs1_mux_in0),
		.in1(rs1_mux_in1),
		.out(rs1_mux_out)
	);

	wire rs2_mux_sel;
  	wire [31:0] rs2_mux_in0, rs2_mux_in1;
	wire [31:0] rs2_mux_out;
	TWO_INPUT_MUX rs2_mux (
		.sel(rs2_mux_sel),
		.in0(rs2_mux_in0),
		.in1(rs2_mux_in1),
		.out(rs2_mux_out)
	);

    wire [31:0] pc_decode_register_d;
    reg [31:0] pc_decode_register_q;
    always @(posedge clk) begin
        pc_decode_register_q <= pc_decode_register_d;
    end

    wire [31:0] instruction_decode_register_d;
    reg [31:0] instruction_decode_register_q;
    always @(posedge clk) begin
        instruction_decode_register_q <= instruction_decode_register_d;
    end

    wire [31:0] rs1_register_d;
    reg [31:0] rs1_register_q;
    always @(posedge clk) begin
        rs1_register_q <= rs1_register_d;
    end

    wire [31:0] rs2_register_d;
    reg [31:0] rs2_register_q;
    always @(posedge clk) begin
        rs2_register_q <= rs2_register_d;
    end

	// jal_adder
    wire [31:0] jal_adder_in0, jal_adder_in1;
    wire [31:0] jal_adder_out;
	ADDER jal_adder (
		.in0(jal_adder_in0),
		.in1(jal_adder_in1),
		.out(jal_adder_out)
	);
	
	// Wiring for D stage
	assign pc_thirty_mux_in0 = bios_douta;
	assign pc_thirty_mux_in1 = imem_doutb;
	
	assign nop_mux_in0 = pc_thirty_mux_out;

    assign pc_decode_register_d = pc_register_q; // for pc pipeline register in decode stage
	assign instruction_decode_register_d = nop_mux_out; // for instruction pipeline register in decode stage

    // wiring to regfile
    assign ra1 = nop_mux_out[19:15];
    assign ra2 = nop_mux_out[24:20];

	assign jal_adder_in0 = pc_register_q;
	assign jal_adder_in1 = {{20{nop_mux_out[31]}}, nop_mux_out[19:12], nop_mux_out[20], nop_mux_out[30:21], 1'b0};

	assign pc_mux_in1 = jal_adder_out;

	assign rs1_mux_in0 = rd1;
	assign rs2_mux_in0 = rd2;

	assign rs1_register_d = rs1_mux_out;
	assign rs2_register_d = rs2_mux_out;

	// 3. X -------------------------------------------------------

    wire [31:0] instruction_execute_register_d;
    reg [31:0] instruction_execute_register_q;
    always @(posedge clk) begin
        instruction_execute_register_q <= instruction_execute_register_d;
    end

    // PC Pipeline Register Execute Stage
    wire [31:0] pc_execute_register_d;
    reg [31:0] pc_execute_register_q;
    always @(posedge clk) begin
        pc_execute_register_q <= pc_execute_register_d;
    end

    // ALU Pipeline Register
    wire [31:0] alu_register_d;
    reg [31:0] alu_register_q;
    always @(posedge clk) begin
        alu_register_q <= alu_register_d;
    end

	// imm_gen
	wire [31:0] imm_gen_in;
	wire [31:0] imm_gen_out;
	IMM_GEN imm_gen (
		.inst(imm_gen_in), 
		.imm(imm_gen_out)
	);

    // RS1_MUX2
	wire [1:0] rs1_mux2_sel;
  	wire [31:0] rs1_mux2_in0, rs1_mux2_in1, rs1_mux2_in2;
	wire [31:0] rs1_mux2_out;
	FOUR_INPUT_MUX rs1_mux2 (
		.sel(rs1_mux2_sel),
		.in0(rs1_mux2_in0),
		.in1(rs1_mux2_in1),
		.in2(rs1_mux2_in2),
		.in3(0),
		.out(rs1_mux2_out)
	);

    // RS2_MUX2
	wire [1:0] rs2_mux2_sel;
  	wire [31:0] rs2_mux2_in0, rs2_mux2_in1, rs2_mux2_in2;
	wire [31:0] rs2_mux2_out;
	FOUR_INPUT_MUX rs2_mux2 (
		.sel(rs2_mux2_sel),
		.in0(rs2_mux2_in0),
		.in1(rs2_mux2_in1),
		.in2(rs2_mux2_in2),
		.in3(0),
		.out(rs2_mux2_out)
	);

    // Branch Comparator
	wire [31:0] branch_comp_rs1;
	wire [31:0] branch_comp_rs2;
	wire branch_comp_br_un;
	wire branch_comp_br_eq;
    wire branch_comp_br_lt;
	BRANCH_COMPARATOR branch_comp (
		.rs1(branch_comp_rs1),
		.rs2(branch_comp_rs2),
		.br_un(branch_comp_br_un),
		.br_eq(branch_comp_br_eq),
		.br_lt(branch_comp_br_lt)
	);

    // A_MUX
    wire [1:0] a_mux_sel;
    wire [31:0] a_mux_in0, a_mux_in1, a_mux_in2;
    wire [31:0] a_mux_out;
    FOUR_INPUT_MUX a_mux (
		.sel(a_mux_sel),
		.in0(a_mux_in0),
		.in1(a_mux_in1),
		.in2(a_mux_in2),
		.in3(0),
		.out(a_mux_out)
	);

    // B_MUX
    wire [1:0] b_mux_sel;
    wire [31:0] b_mux_in0, b_mux_in1;
    wire [31:0] b_mux_out;
    TWO_INPUT_MUX b_mux (
		.sel(b_mux_sel),
		.in0(b_mux_in0),
		.in1(b_mux_in1),
		.out(b_mux_out)
	);

    // ALU
    wire [3:0] alu_sel;
    wire [31:0] alu_rs1, alu_rs2;
    wire [31:0] alu_out;
    ALU alu (
        .alu_sel(alu_sel), 
        .rs1(alu_rs1), 
        .rs2(alu_rs2), 
        .out(alu_out)
    );

    // CSR
    wire csr_mux_sel;
    wire [31:0] csr_mux_in0, csr_mux_in1;
    wire [31:0] csr_mux_out;
    TWO_INPUT_MUX csr_mux (
        .sel(csr_mux_sel),
        .in0(csr_mux_in0),
        .in1(csr_mux_in1),
        .out(csr_mux_out)
    );

    wire [31:0] csr_in;
    always @(posedge clk) begin
        tohost_csr <= csr_in;
    end

    // rs2_mux3 (MUX going to memory from rs2)
    wire [1:0] rs2_mux3_sel;
  	wire [31:0] rs2_mux3_in0, rs2_mux3_in1, rs2_mux3_in2;
	wire [31:0] rs2_mux3_out;
	FOUR_INPUT_MUX rs2_mux2 (
		.sel(rs2_mux3_sel),
		.in0(rs2_mux3_in0),
		.in1(rs2_mux3_in1),
		.in2(rs2_mux3_in2),
		.in3(0),
		.out(rs2_mux3_out)
	);

    // UART
    wire [31:0] uart_instruction, uart_addr;
    wire [7:0] uart_tx_data_in;
    wire [31:0] uart_out;
    IO_MEMORY_MAP uart (
        .clk(clk),
        .instruction(uart_instruction),
        .addr(uart_addr),
        .uart_tx_data_in(uart_tx_data_in),
        .out(uart_out)
    );

    wire [1:0] addr_mux_sel;
  	wire [31:0] addr_mux_in0, addr_mux_in1, addr_mux_in2, addr_mux_in3;
	wire addr_mux_out;
	FOUR_INPUT_MUX addr (
		.sel(addr_mux_sel),
		.in0(addr_mux_in0),
		.in1(addr_mux_in1),
		.in2(addr_mux_in2),
		.in3(addr_mux_in3),
		.out(addr_mux_out)
	);

    // For addr_mux_sel (output of memories)
    always @(*) begin
        case(alu_out[31:28])
            4'b0001: addr_mux_sel = 2'b01;
            4'b0011: addr_mux_sel = 2'b01;
            4'b0100: addr_mux_sel = 2'b00;
            4'b1000: addr_mux_sel = 2'b10;
            default: begin
                addr_mux_sel = 2'b01;
            end
        endcase
    end

	// Wiring for X stage
	assign instruction_execute_register_d = instruction_decode_register_q;
	assign imm_gen_in = instruction_decode_register_q;

    // PC Execute Register Input
    assign pc_execute_register_d = pc_decode_register_q;

    // ALU Pipeline Register Input
    assign alu_register_d = alu_out;

    // rs1_mux2 inputs
	assign rs1_mux2_in0 = rs1_register_q;
	assign rs2_mux2_in0 = rs2_register_q;

    // inputs to branch comparator
    assign branch_comp_rs1 = rs1_mux2_out;
    assign branch_comp_rs2 = rs2_mux2_out;

    // inputs to A-MUX
    assign a_mux_in0 = rs1_mux2_out;
    assign a_mux_in1 = 0; // temp
    assign a_mux_in2 = 0; // temp
    assign a_mux_sel = 0; // temp

    // inputs to B-MUX
    assign b_mux_in0 = rs2_mux2_out;
    assign b_mux_in1 = imm_gen_out;
    assign b_mux_sel = 0; // temp

    // inputs to ALU
    assign alu_rs1 = a_mux_out;
    assign alu_rs2 = b_mux_out;
    assign alu_sel = 0; // temp

    // inputs to CSR_MUX
    assign csr_mux_in0 = imm_gen_out;
    assign csr_mux_in1 = rs1_mux2_out;
    assign csr_mux_sel = 0; // temp

    // input to CSR register
    assign csr_in = csr_mux_out;

    // inputs to RS2_MUX3
    assign rs2_mux3_in0 = rs2_mux2_out;
    assign rs2_mux3_in1 = 0; // temp
    assign rs2_mux3_in2 = 0; // temp

    // input to DMEM
    assign dmem_addr = alu_out[15:2];
    assign dmem_din = rs2_mux3_out;
    assign dmem_en = 0; // temp
    assign dmem_we = 4'b1111; // temp
    // output of dmem = dmem_dout

    // input to BIOS
    assign bios_addrb = alu_out;
    assign bios_enb = 0; // temp

    // input to IMEM
    assign imem_addra = alu_out[15:2];
    assign dina = rs2_mux3_out;
    assign imem_wea = 4'b1111; // temp
    assign imem_ena = 0; // temp

    // input to UART
    assign uart_instruction = instruction_decode_register_q;
    assign uart_addr = alu_out;
    assign uart_tx_data_in = rs2_mux3_out[7:0];

    // addr MUX
    assign addr_mux_in0 = bios_doutb;
    assign addr_mux_in1 = dmem_dout;
    assign addr_mux_in2 = uart_out;
    assign addr_mux_in3 = 0;
    


endmodule
