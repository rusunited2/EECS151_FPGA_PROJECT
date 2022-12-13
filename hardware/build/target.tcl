set ABS_TOP                        /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware
set TOP                            z1top
set FPGA_PART                      xc7z020clg400-1
set_param general.maxThreads       4
set_param general.maxBackupLogs    0
set RTL { /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/EECS151.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/basic_modules.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/clocks.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/io_circuits/button_parser.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/io_circuits/debouncer.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/io_circuits/edge_detector.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/io_circuits/fifo.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/io_circuits/synchronizer.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/io_circuits/uart.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/io_circuits/uart_receiver.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/io_circuits/uart_transmitter.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/memories/bios_mem.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/memories/dmem.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/memories/imem.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/memory_io.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/riscv_core/branch_prediction/bp_cache.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/riscv_core/branch_prediction/branch_predictor.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/riscv_core/branch_prediction/sat_updn.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/riscv_core/cpu.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/riscv_core/reg_file.v /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/z1top.v }
set CONSTRAINTS { /home/cc/eecs151/fa22/class/eecs151-aaq/Desktop/fa22_fpga_team43/hardware/src/z1top.xdc }
