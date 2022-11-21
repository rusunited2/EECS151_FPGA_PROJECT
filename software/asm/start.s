.section    .start
.global     _start

_start:

# Follow a convention
# x1 = result register 1
# x2 = result register 2
# x10 = argument 1 register
# x11 = argument 2 register
# x20 = flag register

# Test ADD
li x10, 100         # Load argument 1 (rs1)
li x11, 200         # Load argument 2 (rs2)
add x1, x10, x11    # Execute the instruction being tested
li x20, 1           # Set the flag register to stop execution and inspect the result register
                    # Now we check that x1 contains 300

# Test BEQ
li x2, 100          # Set an initial value of x2
beq x0, x0, branch1 # This branch should succeed and jump to branch1
li x2, 123          # This shouldn't execute, but if it does x2 becomes an undesirable value
branch1: li x1, 500 # x1 now contains 500
li x20, 2           # Set the flag register
                    # Now we check that x1 contains 500 and x2 contains 100

# TODO: add more tests here

# TEST and
li x3, 25
li x4, 12
and x2, x3, x4
li x20, 3

# TEST and
li x1, 25
and x1, x1, x1
li x20, 4

# TEST and
li x1, -100
li x2, 90
and x1, x1, x2
li x20, 5

# TEST and
li x6, -1
and x8, x6, x6
li x20, 6

# TEST and 1
lui	x1,0xff010
addi	x1,x1,-256
lui	x2,0xf0f1
addi	x2,x2,-241
and	x14,x1,x2
li x20, 7

done: j done
