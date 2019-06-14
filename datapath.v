module datapath(clk,readnum,vsel,loada,loadb,shift,asel,bsel,ALUop,loadc,loads,writenum,write,sximm8/*datapath_in*/,sximm5,PC,mdata,status_out,datapath_out, data_out);
//input [15:0] datapath_in;
 input clk, write, loada, loadb, asel, bsel, loadc, loads;
 input [3:0] vsel;
 input [2:0] readnum, writenum;
 input [1:0] shift, ALUop;
 output [15:0] datapath_out;
//output Z_out;
output [15:0] data_out;
 wire [15:0] data_in, A_out, B_out, sout, Ain, Bin, ALU_out;
 

 input [8:0] PC;
 output [2:0] status_out; // changes to 3 bits from 1
 wire [2:0] status;
 input [15:0] sximm8; // sign extended 8-bit lower imediate (lower 8 bits on instruction register aka where to store to i think)
 input [15:0] sximm5; // sign extended 5-bit immediate
 input [15:0] mdata; // not needed until lab 8

 

 
  Mux4b #(16)mux4( mdata, sximm8, {7'b0,PC}, datapath_out, vsel, data_in); // <-------
  regfile REGFILE(data_in, writenum, write, readnum, clk, data_out);
  vRegister vRegA(clk, loada, data_out, A_out);
  vRegister vRegB(clk, loadb, data_out, B_out);
  shifter Bshift(B_out, shift, sout);
  
 assign Ain = asel ? 16'b0 : A_out;
 assign Bin = bsel ? sximm5 : sout; //<------
 
  ALU aluU2(Ain, Bin, ALUop, ALU_out, status); //increased functionality of ALU now also checks for over flow <-----
  vRegister vRegC(clk, loadc, ALU_out, datapath_out);
  
 assign neg = datapath_out[7] ? 1'b1 : 1'b0; // checking to see if the number is negative, if it is neg has the value 1
 
  vRegister #(3)status0(clk, loadc, status, status_out); //remains the same <-------

  
  
 
endmodule

module Mux4b(a3, a2, a1, a0, select , b);
 parameter k = 1;
 input [k-1:0] a3, a2, a1, a0;
 input [3:0] select;

 output [k-1:0] b;
 
 assign b = 	  ({k{select[0]}} & a0)|
	              ({k{select[1]}} & a1)|
					  ({k{select[2]}} & a2)|
					  ({k{select[3]}} & a3);
 
 endmodule


 