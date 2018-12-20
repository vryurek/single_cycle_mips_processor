//Single cycle mips processor
//Veronica Yurek 12/13/2018
module test_bench(output [31:0] result);
	reg clk;
	Processor DUT(clk, result);
	initial
	begin 
		clk = 1'b0;
	end
	
	always
	begin
		clk = 1'b1;
		#10;
		clk = 1'b0;
		#10;
	end
	
endmodule

module Processor(input CLK, output reg [31:0] Y);
	
	wire [31:0] pcin, pcout, instr, RD1, RD2, WD3, signImm,
		SrcB, ALUResult, ReadData, PCBranch, PCPlus1, 
		end_cycle, PCJump, zeroExtend, Imm;
	wire MemtoReg, MemWrite, BranchE, BranchNE, ALUSrc, Jump,
		RegDst, RegWrite, zeroE, zeroNE, carryOut1, carryOut2, muxBranchE,
		muxBranchNE, muxBranch, rt;
	wire [2:0] ALUControl;
	wire [4:0] WriteReg;
	wire [27:0] jShift;
	
	ProgramCounter pc(CLK, pcin, pcout);	//start counter
	InstructionMemory imem(pcout, instr);	//load instructions into memory
	
	RegisterFile regfile(CLK, RegWrite, instr[25:21], instr[20:16], 
		WriteReg, WD3, RD1, RD2);
	ControlUnit control(instr[31:26], instr[5:0],
		MemtoReg, MemWrite, BranchE, BranchNE, ALUSrc,
		RegDst, RegWrite, Jump, rt, ALUControl);
	
	//select writereg for A3
	mux2to1_5bit mux5bit(RegDst, instr[20:16], instr[15:11], WriteReg);
	
		
	SignExtension ext(instr[15:0], signImm);
	zeroExtend16 zeroExt(instr[15:0], zeroExtend);
	//choose between zero extension or sign for immediate value
	mux2to1 mux5(rt, signImm, zeroExtend, Imm);
	
	mux2to1 mux1(ALUSrc, RD2, Imm, SrcB);
	//alu 
	ALU alu(RD1, SrcB, ALUControl, ALUResult, zeroE, zeroNE);

	//see if program should branch
	assign muxBranchE = zeroE & BranchE;
	assign muxBranchNE = zeroNE & BranchNE;
	assign muxBranch = muxBranchE | muxBranchNE;
	
	DataMemory data(CLK, MemWrite, ALUResult, RD2, ReadData);
	mux2to1 mux2(MemtoReg, ALUResult, ReadData, WD3);	//select to writeback
	
	//determine next pc value
	adder32 pc1(pcout, 32'b1, 1'b0, PCPlus1, carryOut1);
	adder32 pcB(signImm, PCPlus1, 1'b0, PCBranch, carryOut2);
	
	mux2to1 mux3(muxBranch, PCPlus1, PCBranch, end_cycle);
	
	//determine if jump is necessary
	jumpShift jshift(instr[25:0], jShift);
	assign PCJump[31:28] = PCPlus1[31:28];
	assign PCJump[27:0] = jShift;
	mux2to1 mux4(Jump, end_cycle, PCJump, pcin);
	
	always@(WD3) 
	begin
		Y = WD3;
	end
endmodule

module jumpShift( input [25:0] in, output [27:0] out);
	assign out[25:0] = in[25:0];
	assign out [27:26] = 2'b00;
endmodule

module ControlUnit(input [5:0] opcode,
	input [5:0] funct,
	output reg MemtoReg,
	output reg MemWrite,
	output reg BranchE,
	output reg BranchNE,
	output reg ALUSrc,
	output reg RegDst,
	output reg RegWrite,
	output reg Jump,
	output reg rt,
	output reg [2:0] ALUControl);
	
	always @ ( opcode )
	begin
		MemtoReg = 1'b0;
		MemWrite = 1'b0;
		BranchE = 1'b0;
		BranchNE = 1'b0;
		ALUSrc = 1'b0;
		RegDst = 1'b0;
		RegWrite = 1'b0;
		Jump = 1'b0;
		rt = 1'b0;
		ALUControl = 3'b000;
		
		if (opcode == 6'b000000)
		begin
			RegWrite = 1'b1;
			RegDst = 1'b1;
			case (funct)
				6'b100000: ALUControl = 3'b010; // Add
				6'b100010: ALUControl = 3'b110; // Sub
				6'b100100: ALUControl = 3'b000; // And
				6'b100101: ALUControl = 3'b001; // Or
				6'b101010: ALUControl = 3'b111; // SLT

				default: ALUControl = 3'bxxx;
			endcase
		end
		else
		begin
			case (opcode)
				// LW
				6'b100011:
				begin
					RegWrite = 1'b1;
					ALUSrc = 1'b1;
					MemtoReg = 1'b1;
					ALUControl = 3'b010;
				end
				
				// SW
				6'b101011:
				begin	
					RegDst = 1'bx;
					ALUSrc = 1'b1;
					MemWrite = 1'b1;
					MemtoReg = 1'bx;
					ALUControl = 3'b010;
				end
				
				// BEQ
				6'b000100:
				begin
					RegDst = 1'bx;
					BranchE = 1'b1;
					MemtoReg = 1'bx;
					ALUControl = 3'b110;
				end

				//addi
				6'b001000:
				begin
					RegWrite = 1'b1;
					ALUSrc = 1'b1;
					ALUControl = 3'b010;
				end
				
				//jump
				6'b000010:
				begin
					RegDst = 1'bx;
					ALUSrc = 1'bx;
					BranchE = 1'bx;
					BranchNE =1'bx;
					MemtoReg = 1'bx;
					ALUControl = 3'bxxx;
					rt = 1'bx;
					Jump = 1'b1;
				end
				
				//bne
				6'b000101:
				begin
					RegDst = 1'bx;
					BranchNE = 1'b1;
					MemtoReg = 1'bx;
					ALUControl = 3'b110;
				end
				
				//addiu
				6'b001001:
				begin
					RegWrite = 1'b1;
					ALUSrc = 1'b1;
					rt = 1'b1;
					ALUControl = 3'b010;
				end
				
				//andi
				6'b001100:
				begin
					RegWrite = 1'b1;
					ALUSrc = 1'b1;
					rt = 1'b1;
				end
				
				//ori
				6'b001101:
				begin
					RegWrite = 1'b1;
					ALUSrc = 1'b1;
					rt = 1'b1;
					ALUControl = 3'b001;
				end
				
				//slti
				6'b001010:
				begin
					RegWrite = 1'b1;
					ALUSrc = 1'b1;
					ALUControl = 3'b111;
				end
				
				default:
				begin
					MemtoReg = 1'bx;
					MemWrite = 1'bx;
					BranchE = 1'bx;
					BranchNE =1'bx;
					ALUSrc = 1'bx;
					RegDst = 1'bx;
					RegWrite = 1'bx;
					rt = 1'bx;
					ALUControl = 3'bxxx;
				end
		endcase
		end // else
	end // always
endmodule
	
module RegisterFile(input CLK,
	input WE3,
	input [4:0] A1,
	input [4:0] A2,
	input [4:0] A3,
	input [31:0] WD3,
	output reg [31:0] RD1,
	output reg [31:0] RD2);
	reg [31:0] gpreg [0:31];
	integer k;
	
	//set all regs to zero
	initial
	begin
		for (k = 0; k < 32; k = k + 1)
		begin
			gpreg[k] = 0;
		end
	end
	
	always @ ( negedge CLK )
	begin
		if (WE3 == 1'b1)
		begin
			gpreg[A3] <= WD3;
		end
	end
	
	always @ (A1, A2)	//prevents write on negedge clock without missing data
	begin
		RD1 = gpreg[A1];
		RD2 = gpreg[A2];
	end
endmodule

module ProgramCounter(input CLK,
	input [31:0] dataIn,
	output reg [31:0] dataOut);
	reg [31:0] pcreg;
	
	initial
	begin
		pcreg = 32'b0;
	end
	
	always @ ( posedge CLK )
	begin
		pcreg <= dataIn;
		dataOut = pcreg;
	end
endmodule

module InstructionMemory(input [31:0] A,
	output reg [31:0] RD);
	reg [31:0] instMemory [0:1023];
	initial
	begin
		$readmemh("a.out", instMemory);	//load instructions from file
	end
	always @ ( A )
	begin
		RD <= instMemory[A];
	end
endmodule

module DataMemory(input CLK,
	input WE,
	input [31:0] A,
	input [31:0] WD,
	output reg [31:0] RD);
	reg [31:0] dataMemory [0:1023];
	
	/*initial 
	begin
		dataMemory[6] = 32'hAAAAAAAAA;
		dataMemory[1] = 32'd1;
	end*/
	
	always @ (*)
	begin
		if (WE == 1'b1)	//write enabled
		begin
			dataMemory[A] <= WD;
		end
		RD <= dataMemory[A];
	end
endmodule

module SignExtension(input [15:0] in, output [31:0] result);
	assign result = { {16{in[15]}}, in };
endmodule

module ALU(input [31:0] A,
				input [31:0] B,
				input [2:0] F,
				output [31:0] Y, output zeroE, output zeroNE);

	// Internal connections (all the wires)
	wire [31:0] w1, w0, w2, w3, mux2out, adderSum;
	wire adderCarryOut;

	// Module instantiations
	
	mux2to1 m2(F[2], B, ~B, mux2out);
	adder32 a32(A, mux2out, F[2], adderSum, adderCarryOut);
	
	assign w1 = A | mux2out;
	assign w0 = A & mux2out;
	assign w2 = adderSum;
	zeroExtender ze(adderSum[31], w3);
	
	mux4to1 m4(F[1:0], w0, w1, w2, w3, Y);
	assign zeroE = (Y == 32'b0) ? 1'b1 : 1'b0;
	assign zeroNE = (Y == 32'b0) ? 1'b0 : 1'b1;
	
	
endmodule

module mux2to1(input select,
							input [31:0] w0,
							input [31:0] w1,
							output reg [31:0] y);
	// If you use an always block, make y an output reg.
	always@(*)
	begin
		case(select)
		2'b00: y = w0;
		2'b01: y = w1;
		endcase
	end
endmodule

module mux2to1_5bit(input select,
							input [4:0] w0,
							input [4:0] w1,
							output reg [4:0] y);
	// If you use an always block, make y an output reg.
	always@(*)
	begin
		case(select)
		2'b00: y = w0;
		2'b01: y = w1;
		endcase
	end
endmodule

module mux4to1(input [1:0] select,
							input [31:0] w0,
							input [31:0] w1,
							input [31:0] w2,
							input [31:0] w3,
							output reg [31:0] y);
	// Same as the 2-to-1 mux
	always@(*)
	begin
		case(select)
		2'b00: y = w0;
		2'b01: y = w1;
		2'b10: y = w2;
		2'b11: y = w3;
		endcase
	end
endmodule

module adder32(input [31:0] A,
							input [31:0] B,
							input carryIn,
							output [31:0] sum,
							output carryOut);
							
	wire c4, c8, c12, c16, c20, c24, c28;
	CLA_4bit m1(A[3:0], B[3:0], carryIn, sum[3:0], c4);
	CLA_4bit m2(A[7:4], B[7:4], c4, sum[7:4], c8);
	CLA_4bit m3(A[11:8], B[11:8], c8, sum[11:8], c12);
	CLA_4bit m4(A[15:12], B[15:12], c12, sum[15:12], c16);
	CLA_4bit m5(A[19:16], B[19:16], c16, sum[19:16], c20);
	CLA_4bit m6(A[23:20], B[23:20], c20, sum[23:20], c24);
	CLA_4bit m7(A[27:24], B[27:24], c24, sum[27:24], c28);
	CLA_4bit m8(A[31:28], B[31:28], c28, sum[31:28], carryOut);
endmodule

module zeroExtender(input singleBit,
									output [31:0] dataOut);
	// Set to appropriate size.
	// Make dataOut a reg if necessary.
	assign dataOut = { {31{1'b0}}, singleBit}; 
endmodule

module zeroExtend16(input [15:0] in, output [31:0] out);
	assign out = { {16{1'b0}}, in};
endmodule

module CLA_4bit(input [3:0]A, B, input Cin, output [3:0]S, output Cout);
	wire [3:0]P, G, C;
	assign G = A & B;
	assign P = A ^ B;

	assign C[0] = Cin;
	assign C[1] = G[0] | (P[0] & C[0]);
	assign C[2] = G[1] | (P[1] & G[0]) | (P[1] & P[0] & C[0]);
	assign C[3] = G[2] | (P[2] & G[1]) | (P[2] & P[1] & G[0]) | (P[2] & P[1] & P[0] & C[0]);
	assign Cout = G[3] | (P[3] & G[2]) | (P[3] & P[2] & G[1]) | (P[3] & P[2] & P[1] & G[0]) | (P[3] & P[2] & P[1] & P[0] & C[0]);

	assign S = P ^ C;
endmodule

