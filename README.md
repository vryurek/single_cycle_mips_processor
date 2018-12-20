# single_cycle_mips_processor
Single cycle mips processor written in Verilog for CS3421 Computer Organization at MTU.

Supported Instructions:
add,
sub,
and,
or,
beq,
jump,
lw,
sw,
addi,
ori,
andi,
addiu,
bne.

The instruction memory reads in a file named "a.out". To generate your own a.out, create an asm file of mips instructions (such as tier3test.asm). Compile the included kmips.cpp with g++ and run this command to generate the a.out file: ./kmips yourfile.asm


This file is made to simulate in modelsim and quartus.
