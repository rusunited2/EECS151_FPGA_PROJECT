/* IO File

*/
`timescale 1ns/1ns
`include "opcode.vh"
// IO module
module IO_MEMORY_MAP #(
    parameter CPU_CLOCK_FREQ = 50_000_000,
    parameter RESET_PC = 32'h4000_0000,
    parameter BAUD_RATE = 115200
) (
    input clk,
    input rst,
    input serial_in,
    input [31:0] instruction,
    input [31:0] addr,
    input [7:0] uart_tx_data_in,
    output reg [31:0] out,
    output serial_out
);
    // On-chip UART
    //// UART Receiver
    wire [7:0] uart_rx_data_out;
    wire uart_rx_data_out_valid;
    reg uart_rx_data_out_ready;
    //// UART Transmitter
    // wire [7:0] uart_tx_data_in;
    reg uart_tx_data_in_valid;
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

    // Cycle Counter
    reg [31:0] cycle_counter = 0;
    always @(posedge clk) begin
        if (addr == 32'h80000018) cycle_counter <= 0;
        else cycle_counter <= cycle_counter + 1;
    end

    // Instruction Counter
    reg [31:0] instruction_counter = 0;
    always @(posedge clk) begin
        if (addr == 32'h80000018) cycle_counter <= 0;
        else if (instruction == 32'b0000_0000_0000_0000_0000_0000_0001_0011) instruction_counter <= instruction_counter;
        else instruction_counter <= instruction_counter + 1;
    end

    // for rx_data_out_ready and tx_in_valid
    always @(*) begin
        if (instruction[6:2] == `OPC_LOAD_5 && addr == 32'h80000004) uart_rx_data_out_ready = 1;
        else if (instruction[6:2] == `OPC_STORE_5 && addr == 32'h80000008) uart_tx_data_in_valid = 1; // can they be 1 at same time?
        else begin
            uart_rx_data_out_ready = 0;
            uart_tx_data_in_valid = 0;
        end
    end

    // for output of UART
    always @(*) begin
        case(addr)
            32'h80000000: out = {30'b0, uart_rx_data_out_valid, uart_tx_data_in_ready};
            32'h80000004: out = {24'b0, uart_rx_data_out};
            32'h80000010: out = cycle_counter;
            32'h80000014: out = instruction_counter;
            default: begin
                out = 32'h00000000;
            end
        endcase
    end

endmodule