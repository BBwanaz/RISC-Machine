module regfile(data_in,writenum,write,readnum,clk,data_out);
   input [15:0] data_in;
	input [2:0] writenum, readnum;
	input write, clk;
	output [15:0] data_out;
	
	wire [7:0] enable_wire; // Wires to enable a given register to be written to
	wire [7:0] read_select; // one hot code to be used with the MUX to select which regiseter output to
	
	 
	
	wire [7:0] DEC; // Output from our decoder
	wire [15:0] R0,R1,R2, R3, R4, R5, R6, R7; // Output reported by our first register R1
	
	assign enable_wire = {8{write}}&DEC;   // Produced 8 copies of write and anded
	
	
	Decoder Read(readnum,read_select);
	Decoder D1(writenum, DEC); // Decoder to aid us select the register that we want to write to
	
	vRegister vR0(clk, enable_wire[0], data_in, R0); // 8 different instances of each register
	vRegister vR1(clk, enable_wire[1], data_in, R1);
	vRegister vR2(clk, enable_wire[2], data_in, R2);
	vRegister vR3(clk, enable_wire[3], data_in, R3);
	vRegister vR4(clk, enable_wire[4], data_in, R4);
	vRegister vR5(clk, enable_wire[5], data_in, R5);
	vRegister vR6(clk, enable_wire[6], data_in, R6);
	vRegister vR7(clk, enable_wire[7], data_in, R7);
	
	MUX8 Out(R0,R1,R2,R3,R4,R5,R6,R7,read_select,data_out);
	
	
endmodule

// This code was borrowed from Lab 5 introduction slide page 7
module vRegister(clk, en, in , out);
   	parameter n = 16;
		input clk,en;
		input [n-1:0] in;
		output [n-1:0] out;
		
		reg [n-1:0] out;
		wire [n-1:0] next_out;
		
		assign next_out = en ? in : out;
		
		always @(posedge clk)
		    out <= next_out;
			 
endmodule

// Decoder to convert out 3 bit select value into a one hot 8 bit value
module Decoder(in , out);
  parameter n = 3;
  parameter m = 8;
  
  input [n-1:0] in;
  output [m-1:0] out;
  
  wire [m-1:0] out = 1 << in;
  
endmodule 

// Multiplexer to select the register whose output should appear on the data_out wire in out top level module
module MUX8(R0, R1, R2, R3, R4, R5,R6, R7, select, out);
   parameter k = 16;
	
	input [k-1:0] R0,R1, R2, R3, R4, R5,R6, R7;
	input [7:0] select;
	output [k-1:0] out;
	
	assign out =  ({k{select[0]}} & R0)|
	              ({k{select[1]}} & R1)|
					  ({k{select[2]}} & R2)|
					  ({k{select[3]}} & R3)|
					  ({k{select[4]}} & R4)|
					  ({k{select[5]}} & R5)|
					  ({k{select[6]}} & R6)|
					  ({k{select[7]}} & R7);
endmodule
	
	
