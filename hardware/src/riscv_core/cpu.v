module cpu #(
    parameter CPU_CLOCK_FREQ = 50_000_000,
    parameter RESET_PC = 32'h4000_0000,
    parameter BAUD_RATE = 115200
) (
    input clk,
    input rst,
    input bp_enable,
    input serial_in,
    output serial_out
);
    // BIOS Memory
    // Synchronous read: read takes one cycle
    // Synchronous write: write takes one cycle
    wire [11:0] bios_addra, bios_addrb;
    wire [31:0] bios_douta, bios_doutb;
    reg bios_ena;
    reg bios_enb;
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
    reg [31:0] dmem_din;
	wire [31:0] dmem_dout;
    reg [3:0] dmem_we;
    reg dmem_en;
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
    reg [3:0] imem_wea;
    reg imem_ena;
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
    // UART Receiver
    wire [7:0] uart_rx_data_out;
    wire uart_rx_data_out_valid;
    reg uart_rx_data_out_ready; // done
    // UART Transmitter
    wire [7:0] uart_tx_data_in;
    reg uart_tx_data_in_valid; // done
    wire uart_tx_data_in_ready;
    uart #(
        .CLOCK_FREQ(CPU_CLOCK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) on_chip_uart (
        .clk(clk), // input/done
        .reset(rst), // input/done

        .serial_in(serial_in), // input/done
        .data_out(uart_rx_data_out), // output
        .data_out_valid(uart_rx_data_out_valid), // output
        .data_out_ready(uart_rx_data_out_ready), // input/done

        .serial_out(serial_out), // output/done
        .data_in(uart_tx_data_in), // input
        .data_in_valid(uart_tx_data_in_valid), // input
        .data_in_ready(uart_tx_data_in_ready) // output
    );

    reg [31:0] tohost_csr = 0;

    // TODO: Your code to implement a fully functioning RISC-V core
    // Add as many modules as you want
    // Feel free to move the memory modules around
	// 1. WF ------------------------------------------------------

	// 1.1: pc_mux
	wire [2:0] pc_mux_sel;
  	wire [31:0] pc_mux_in0, pc_mux_in1, pc_mux_in2, pc_mux_in3, pc_mux_in4;
	wire [31:0] pc_mux_out;
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
    assign pc_plus_four_in0 = pc_register_q; // Changed Nov 10
    // assign pc_mux_in2 = pc_plus_four_out;
    assign pc_mux_in0 = RESET_PC;

    assign pc_register_d = pc_mux_out;
    assign pc_mux_in4 = pc_register_q;

    assign bios_addra = pc_mux_out[15:2];
    assign imem_addrb = pc_mux_out[15:2];

    // assign bios_ena = 1; // FIX THIS
    always @(*) begin
        if (pc_mux_out[31:28] == 4'b0100) bios_ena = 1;
        else bios_ena = 0;
    end
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

    // Branch Predictor
    wire [31:0] pc_guess, pc_check;
    wire is_br_guess, is_br_check, br_taken_check, br_pred_taken;
    branch_predictor br_predictor (
        .clk(clk),
        .reset(rst),
        .pc_guess(pc_guess),
        .is_br_guess(is_br_guess),
        .pc_check(pc_check),
        .is_br_check(is_br_check),
        .br_taken_check(br_taken_check),
        .br_pred_taken(br_pred_taken)
    );

    // BR Taken MUX
    wire br_taken_mux_sel;
  	wire [31:0] br_taken_mux_in0, br_taken_mux_in1;
	wire [31:0] br_taken_mux_out;
	TWO_INPUT_MUX br_taken_mux (
		.sel(br_taken_mux_sel),
		.in0(br_taken_mux_in0),
		.in1(br_taken_mux_in1),
		.out(br_taken_mux_out)
	);

    // BP Enable MUX
    wire bp_enable_mux_sel;
  	wire [31:0] bp_enable_mux_in0, bp_enable_mux_in1;
	wire [31:0] bp_enable_mux_out;
	TWO_INPUT_MUX bp_enable_mux (
		.sel(bp_enable_mux_sel),
		.in0(bp_enable_mux_in0),
		.in1(bp_enable_mux_in1),
		.out(bp_enable_mux_out)
	);

    // Branch Prediction Taken/Not Taken MUX
    wire bp_pred_taken_mux_sel;
  	wire [31:0] bp_pred_taken_mux_in0, bp_pred_taken_mux_in1;
	wire [31:0] bp_pred_taken_mux_out;
	TWO_INPUT_MUX bp_pred_taken_mux (
		.sel(bp_pred_taken_mux_sel),
		.in0(bp_pred_taken_mux_in0),
		.in1(bp_pred_taken_mux_in1),
		.out(bp_pred_taken_mux_out)
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

    wire [31:0] br_pred_taken_register_d;
    reg [31:0] br_pred_taken_register_q;
    always @(posedge clk) begin
        br_pred_taken_register_q <= br_pred_taken_register_d;
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
	assign pc_thirty_mux_in0 = imem_doutb;
	assign pc_thirty_mux_in1 = bios_douta;
	
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

    // Wiring for Branch Predictor
    assign pc_guess = pc_register_q;
    assign is_br_guess = nop_mux_out[6:2] == `OPC_BRANCH_5;

    // BR Taken MUX Wiring
    assign br_taken_mux_sel = (br_pred_taken) && (nop_mux_out[6:2] == `OPC_BRANCH_5);
    assign br_taken_mux_in0 = pc_plus_four_out;
    assign br_taken_mux_in1 = pc_register_q + {{20{nop_mux_out[31]}}, nop_mux_out[7], nop_mux_out[30:25], nop_mux_out[11:8], 1'b0};

    // BP Enable MUX Wiring
    assign bp_enable_mux_sel = bp_enable;
    assign bp_enable_mux_in0 = pc_plus_four_out;
    assign bp_enable_mux_in1 = br_taken_mux_out;

    // PC Sel Input 2
    assign pc_mux_in2 = bp_enable_mux_out;

    // BR Pred Taken/Not Taken and BR Pred Taken Register
    assign bp_pred_taken_mux_sel= bp_enable;
    assign bp_pred_taken_mux_in0 = 0; // For no Branch prediction (always guess not taken)
    assign bp_pred_taken_mux_in1 = br_pred_taken; // Branch prediction output
    assign br_pred_taken_register_d = bp_pred_taken_mux_out;

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
    wire [1:0] csr_mux_sel;
    wire [31:0] csr_mux_in0, csr_mux_in1, csr_mux_in2;
    wire [31:0] csr_mux_out;
    FOUR_INPUT_MUX csr_mux (
		.sel(csr_mux_sel),
		.in0(csr_mux_in0),
		.in1(csr_mux_in1),
		.in2(csr_mux_in2),
		.in3(0),
		.out(csr_mux_out)
	);

    wire br_result_mux_sel;
    wire [31:0] br_result_mux_in0, br_result_mux_in1;
    wire [31:0] br_result_mux_out;
    TWO_INPUT_MUX br_result_mux (
		.sel(br_result_mux_sel),
		.in0(br_result_mux_in0),
		.in1(br_result_mux_in1),
		.out(br_result_mux_out)
	);

    wire [31:0] csr_in;
    always @(posedge clk) begin
        tohost_csr <= csr_in;
    end

    // rs2_mux3 (MUX going to memory from rs2)
    wire [1:0] rs2_mux3_sel;
  	wire [31:0] rs2_mux3_in0, rs2_mux3_in1, rs2_mux3_in2;
	wire [31:0] rs2_mux3_out;
	FOUR_INPUT_MUX rs2_mux3 (
		.sel(rs2_mux3_sel),
		.in0(rs2_mux3_in0),
		.in1(rs2_mux3_in1),
		.in2(rs2_mux3_in2),
		.in3(0),
		.out(rs2_mux3_out)
	);
    
    reg [1:0] addr_mux_sel;
  	wire [31:0] addr_mux_in0, addr_mux_in1, addr_mux_in2, addr_mux_in3;
	wire [31:0] addr_mux_out;
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
        case(alu_register_q[31:28])
            4'b0001: addr_mux_sel = 2'b01;
            4'b0011: addr_mux_sel = 2'b01;
            4'b0100: addr_mux_sel = 2'b00;
            4'b1000: addr_mux_sel = 2'b10;
            default: begin
                addr_mux_sel = 2'b01;
            end
        endcase
    end

    wire [31:0] ldx_in;
    wire [2:0] ldx_sel;
    wire [31:0] ldx_out;
	wire [31:0] ldx_alu_out;
    LDX ldx (
        .ldx_in(ldx_in), 
        .ldx_sel(ldx_sel), 
        .ldx_out(ldx_out),
		.alu_out(ldx_alu_out)
    );

    wire [31:0] pc_plus_four2_in0;
    wire [31:0] pc_plus_four2_out;
	ADDER pc_plus_four2 (
		.in0(pc_plus_four2_in0),
		.in1(32'd4),
		.out(pc_plus_four2_out)
	);

    wire [1:0] wb_mux_sel;
  	wire [31:0] wb_mux_in0, wb_mux_in1, wb_mux_in2, wb_mux_in3;
	wire [31:0] wb_mux_out;
	FOUR_INPUT_MUX wb_mux (
		.sel(wb_mux_sel),
		.in0(wb_mux_in0),
		.in1(wb_mux_in1),
		.in2(wb_mux_in2),
		.in3(wb_mux_in3),
		.out(wb_mux_out)
	);

	// Wiring for X stage
	assign instruction_execute_register_d = instruction_decode_register_q;
	assign imm_gen_in = instruction_decode_register_q;

    // PC Execute Register Input
    assign pc_execute_register_d = pc_decode_register_q;

    // ALU Pipeline Register Input
    assign alu_register_d = alu_out;

    // rs1_mux2 inputs
	assign rs1_mux2_in0 = rs1_register_q;
	assign rs1_mux2_in1 = alu_register_q; // ALU->ALU forwarding
	assign rs1_mux2_in2 = ldx_out; // MEM->ALU forwarding

	assign rs2_mux2_in0 = rs2_register_q;
	assign rs2_mux2_in1 = alu_register_q; // ALU->ALU forwarding
	assign rs2_mux2_in2 = ldx_out; // MEM->ALU forwarding

    // inputs to branch comparator
    assign branch_comp_rs1 = rs1_mux2_out;
    assign branch_comp_rs2 = rs2_mux2_out;

    // inputs to A-MUX
    assign a_mux_in0 = rs1_mux2_out;
    assign a_mux_in1 = pc_decode_register_q;
    assign a_mux_in2 = ldx_out; // temp

    // inputs to B-MUX
    assign b_mux_in0 = rs2_mux2_out;
    assign b_mux_in1 = imm_gen_out;

    // inputs to ALU
    assign alu_rs1 = a_mux_out;
    assign alu_rs2 = b_mux_out;

    // inputs to CSR_MUX
    assign csr_mux_in0 = csr_in;
    assign csr_mux_in1 = imm_gen_out;
    assign csr_mux_in2 = rs1_mux2_out;

    // input to CSR register
    assign csr_in = csr_mux_out;

    // inputs to RS2_MUX3
    assign rs2_mux3_in0 = rs2_mux2_out;
    assign rs2_mux3_in1 = alu_register_q; // temp
    assign rs2_mux3_in2 = ldx_out; // temp

    // send ALU result back to PC_SEL MUX
    // assign pc_mux_in3 = alu_out;

	// Input to ldx for lw, lh and lb
	assign ldx_alu_out = alu_register_q;


    // FORWARD DATA D TO RS1_MUX and RS2_MUX
    assign rs1_mux_in1 = wb_mux_out;
    assign rs2_mux_in1 = wb_mux_out;

    // Branch Prediction Wiring
    assign pc_check = pc_decode_register_q;
    assign is_br_check = instruction_decode_register_q[6:2] == `OPC_BRANCH_5;

    // BR Result Mux (output to PC_SEL = 3)
    assign br_result_mux_in0 = alu_out;
    assign br_result_mux_in1 = pc_decode_register_q + 4;
    assign pc_mux_in3 = br_result_mux_out;

    // --------------------------------------------MEMORY ASSIGNS


    // input to DMEM
    assign dmem_addr = alu_out[15:2];

	// Russel added this for tests 33-40 (store + LUI)
	always @(*) begin
		dmem_din = rs2_mux3_out << (8 * alu_out[1:0]);
	end

    //assign dmem_din = rs2_mux3_out;
    always @(*) begin
        case(instruction_decode_register_q[6:2])
            `OPC_LOAD_5: begin
                if (alu_out[31:28] == 4'b0001 || alu_out[31:28] == 4'b0011) dmem_en = 1;
                else dmem_en = 0;
            end
            `OPC_STORE_5: begin
                if (alu_out[31:28] == 4'b0001 || alu_out[31:28] == 4'b0011) dmem_en = 1;
                else dmem_en = 0;
            end
            default: begin
                dmem_en = 0;
            end
        endcase
    end

    // output of dmem = dmem_dout

    // input to BIOS
    assign bios_addrb = alu_out[13:2];
    always @(*) begin
        if (instruction_decode_register_q[6:2] == `OPC_LOAD_5 && alu_out[31:28] == 4'b0100) bios_enb = 1;
        else bios_enb = 0;
    end

    // input to IMEM
    assign imem_addra = alu_out[15:2]; // correct
    assign imem_dina = rs2_mux3_out; // correct

    always @(*) begin
        if (instruction_decode_register_q[6:2] == `OPC_STORE_5 && (alu_out[31:28] == 4'b0010 || alu_out[31:28] == 4'b0011) && pc_decode_register_q[30] == 1'b1) begin
            imem_ena = 1;
        end
        else begin 
            imem_ena = 0;
        end
    end

    // input to UART
    wire [31:0] uart_lw_instruction,uart_sw_instruction, uart_lw_addr, uart_sw_addr;
    reg [31:0] uart_out;

    assign uart_sw_instruction = instruction_decode_register_q;
    assign uart_lw_instruction = instruction_execute_register_q;
    assign uart_sw_addr = alu_out;
    assign uart_lw_addr = alu_register_q;
    assign uart_tx_data_in = rs2_mux3_out[7:0];

    always @(*) begin
        if (uart_lw_instruction[6:2] == `OPC_LOAD_5 && uart_lw_addr == 32'h80000004) begin
            uart_rx_data_out_ready = 1;
            uart_tx_data_in_valid = 0;
        end
        else if (uart_sw_instruction[6:2] == `OPC_STORE_5 && uart_sw_addr == 32'h80000008) begin
            uart_rx_data_out_ready = 0;
            uart_tx_data_in_valid = 1;
        end
        else begin
            uart_rx_data_out_ready = 0;
            uart_tx_data_in_valid = 0;
        end
    end

    // addr MUX input
    assign addr_mux_in0 = bios_doutb;
    assign addr_mux_in1 = dmem_dout;
    assign addr_mux_in2 = uart_out;
    assign addr_mux_in3 = 0;
    
    // ldx input
    assign ldx_in = addr_mux_out;

    // pc + 4 in execute stage input
    assign pc_plus_four2_in0 = pc_execute_register_q;

    // wb mux
    assign wb_mux_in0 = ldx_out;
    assign wb_mux_in1 = alu_register_q;
    assign wb_mux_in2 = pc_plus_four2_out;

    // writeback to regfile
    assign wa = instruction_execute_register_q[11:7];
    assign wd = wb_mux_out;
    


    // ------------------- WF CONTROL LOGIC
    wire [31:0] wf_instruction;
    wire [31:0] wf_x_instruction;
    wire wf_rf_we;
    wire [1:0] wf_wb_sel;
    wire [2:0] wf_ldx_sel, wf_pc_sel;
	wire wf_br_taken;
    wire wf_jal;
    wire wf_jalr;
    wire wf_br_pred_correct;
    WF_CU wf_cu (
        .rst(rst),
        .instruction(wf_instruction), 
        .rf_we(wf_rf_we), 
        .wb_sel(wf_wb_sel), 
        .ldx_sel(wf_ldx_sel), 
        .pc_sel(wf_pc_sel),
		.br_taken(wf_br_taken),
        .jal(wf_jal),
        .jalr(wf_jalr),
        .x_instruction(wf_x_instruction),
        .br_pred_correct(wf_br_pred_correct)
    );

    assign wf_instruction = instruction_execute_register_q; // check this if reset we need to change control logic
    assign we = wf_rf_we;
    assign wb_mux_sel = wf_wb_sel;
    assign ldx_sel = wf_ldx_sel;
    assign pc_mux_sel = wf_pc_sel;
    assign wf_jal = (nop_mux_out[6:2] == `OPC_JAL_5) ? 1 : 0;
    assign wf_jalr = (instruction_decode_register_q[6:2] == `OPC_JALR_5) ? 1 : 0;
    assign wf_x_instruction = instruction_decode_register_q;


    // ------------------ D CONTROL LOGIC
    wire [31:0] d_instruction;
    wire [31:0] d_pc;
    wire [31:0] d_wf_instruction;
    wire [31:0] d_x_instruction;
    wire d_pc_thirty, d_nop_sel, d_orange_sel, d_green_sel;
    wire d_jalr;
    wire d_br_taken;
    wire d_br_pred_correct;
    D_CU d_cu (
        .instruction(d_instruction), 
        .pc(d_pc), 
        .pc_thirty(d_pc_thirty), 
        .nop_sel(d_nop_sel), 
        .orange_sel(d_orange_sel), 
        .green_sel(d_green_sel),
        .jalr(d_jalr),
        .br_taken(d_br_taken),
        .wf_instruction(d_wf_instruction),
        .x_instruction(d_x_instruction),
        .br_pred_correct(d_br_pred_correct)
    );

    assign d_instruction = nop_mux_out;
    assign d_pc = pc_register_q;
    assign pc_thirty_mux_sel = d_pc_thirty; // TEMP (FIX THIS)
    assign nop_mux_sel = d_nop_sel;
    assign rs1_mux_sel = d_orange_sel;
    assign rs2_mux_sel = d_green_sel;
    assign d_jalr = (instruction_decode_register_q[6:2] == `OPC_JALR_5) ? 1 : 0;
    assign d_wf_instruction = instruction_execute_register_q;
    assign d_x_instruction = instruction_decode_register_q;
    
    // ------------------- EX CONTROL LOGIC
    wire [31:0] x_instruction, x_wf_instruction;
    wire x_br_eq, x_br_lt;
    wire x_br_un, x_b_sel;
    wire [1:0] x_orange_sel, x_green_sel, x_a_sel, x_rs2_sel, x_csr_sel;
    wire [3:0] x_alu_sel;
	wire x_br_taken;
    wire x_br_pred_taken, x_br_pred_correct, x_br_result;
    X_CU x_cu (
        .instruction(x_instruction), 
        .orange_sel(x_orange_sel), 
        .green_sel(x_green_sel), 
        .br_un(x_br_un), 
        .br_eq(x_br_eq), 
        .br_lt(x_br_lt), 
        .a_sel(x_a_sel), 
        .b_sel(x_b_sel), 
        .rs2_sel(x_rs2_sel), 
        .alu_sel(x_alu_sel), 
        .csr_sel(x_csr_sel),
		.br_taken(x_br_taken),
		.wf_instruction(x_wf_instruction),
        .br_pred_taken(x_br_pred_taken),
        .br_pred_correct(x_br_pred_correct),
        .br_result(x_br_result)
    );

    // EX Control Logic wires
    assign x_instruction = instruction_decode_register_q;
    assign x_br_eq = branch_comp_br_eq;
    assign x_br_lt = branch_comp_br_lt;
    assign branch_comp_br_un = x_br_un;
    assign rs1_mux2_sel = x_orange_sel;
    assign rs2_mux2_sel = x_green_sel;
    assign a_mux_sel = x_a_sel;
    assign b_mux_sel = x_b_sel;
    assign rs2_mux3_sel = x_rs2_sel;
    assign alu_sel = x_alu_sel;
    assign csr_mux_sel = x_csr_sel;
	assign wf_br_taken = x_br_taken;
	assign x_wf_instruction = instruction_execute_register_q;
    assign x_br_pred_taken = br_pred_taken_register_q;

    assign d_br_taken = x_br_taken;

    // Wiring for Branch Predictor
    assign br_taken_check = x_br_taken;

    // Wiring for BR Result Mux
    assign br_result_mux_sel = x_br_result;

    // Wiring for br_pred_correct to other modules
    assign wf_br_pred_correct = x_br_pred_correct;
    assign d_br_pred_correct = x_br_pred_correct;

    // UART Output and Counters (need to move at bottom due to br_pred_correct)
    // Cycle Counter
    reg [31:0] cycle_counter = 0;
    always @(posedge clk) begin
        if (uart_sw_addr == 32'h80000018) cycle_counter <= 0;
        else cycle_counter <= cycle_counter + 1;
    end

    // Instruction Counter
    reg [31:0] instruction_counter = 0;
    always @(posedge clk) begin
        if (uart_sw_addr == 32'h80000018) instruction_counter <= 0;
        else if (uart_sw_instruction == 32'b0000_0000_0000_0000_0000_0000_0001_0011) instruction_counter <= instruction_counter;
        else instruction_counter <= instruction_counter + 1;
    end

    // Total Branch Instructions
    reg [31:0] branch_instructions_counter = 0;
    always @(posedge clk) begin
        if (uart_sw_addr == 32'h80000018) branch_instructions_counter <= 0;
        else if (instruction_decode_register_q[6:2] == `OPC_BRANCH_5) branch_instructions_counter <= branch_instructions_counter + 1;
        else branch_instructions_counter <= branch_instructions_counter;
    end

    // Correct Predictions
    reg [31:0] correct_prediction_counter = 0;
    always @(posedge clk) begin
        if (uart_sw_addr == 32'h80000018) correct_prediction_counter <= 0;
        else if (instruction_decode_register_q[6:2] == `OPC_BRANCH_5 && x_br_pred_correct) correct_prediction_counter <= correct_prediction_counter + 1;
        else correct_prediction_counter <= correct_prediction_counter;
    end

    always @(*) begin
        case(uart_lw_addr)
            32'h80000000: uart_out = {30'b0, uart_rx_data_out_valid, uart_tx_data_in_ready};
            32'h80000004: uart_out = {24'b0, uart_rx_data_out};
            32'h80000010: uart_out = cycle_counter;
            32'h80000014: uart_out = instruction_counter;
            32'h8000001c: uart_out = branch_instructions_counter;
            32'h80000020: uart_out = correct_prediction_counter;
            default: begin
                uart_out = 32'h00000000;
            end
        endcase
    end

	// Combinational logic for dmem write enable (Russel added this for tests 33-40)
	always @(*) begin
		if (x_instruction[6:2] == `OPC_STORE_5) begin
			if (alu_out[1:0] == 0) begin
				case(x_instruction[14:12])
    				3'b000: dmem_we = 4'b0001; // temp what are these values
					3'b001: dmem_we = 4'b0011;
					3'b010: dmem_we = 4'b1111;
                    default: dmem_we = 4'b0000;
				endcase
			end
			else if (alu_out[1:0] == 1 && x_instruction[14:12] == 3'b000) begin
				dmem_we = 4'b0010;
			end
			else if (alu_out[1:0] == 2) begin
				case(x_instruction[14:12])
    				3'b000: dmem_we = 4'b0100; // temp what are these values
					3'b001: dmem_we = 4'b1100;
                    default: dmem_we = 4'b0000;
				endcase
			end
			else if (alu_out[1:0] == 3 && x_instruction[14:12] == 3'b000) begin
				dmem_we = 4'b1000;
			end
            else dmem_we = 4'b0000;
		end
        else dmem_we = 4'b0000;
	end

    // IMEM WEA
    always @(*) begin
		if (x_instruction[6:2] == `OPC_STORE_5) begin
			if (alu_out[1:0] == 0) begin
				case(x_instruction[14:12])
    				3'b000: imem_wea = 4'b0001; // temp what are these values
					3'b001: imem_wea = 4'b0011;
					3'b010: imem_wea = 4'b1111;
                    default: imem_wea = 4'b0000;
				endcase
			end
			else if (alu_out[1:0] == 1 && x_instruction[14:12] == 3'b000) begin
				imem_wea = 4'b0010;
			end
			else if (alu_out[1:0] == 2) begin
				case(x_instruction[14:12])
    				3'b000: imem_wea = 4'b0100; // temp what are these values
					3'b001: imem_wea = 4'b1100;
                    default: imem_wea = 4'b0000;
				endcase
			end
			else if (alu_out[1:0] == 3 && x_instruction[14:12] == 3'b000) begin
				imem_wea = 4'b1000; // temp what are these values
			end
            else imem_wea = 4'b0000;
		end
        else imem_wea = 4'b0000;
	end
endmodule
