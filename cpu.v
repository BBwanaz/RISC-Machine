

// FSM state encoding

`define Reset 5'b00000 //Added States replacing Wait 
`define IF1   5'b00110
`define IF2   5'b01010
`define UpdatePC 5'b01011

// -------------------------------------------- States for STR and for LDR
`define LDR 5'b01100
`define STR 5'b01101
`define HALT 5'b01110
`define STR_OP 5'b01111
`define Add_sxim 5'b10000




//------------------ These states above replaced the wait state

`define Decode 5'b00001
`define GetA 5'b00010
`define GetB 5'b00011
`define Operate 5'b00100
`define WriteReg 5'b00101
`define WriteRn  5'b00111
`define ReadRm 5'b01000
`define OperateMOV 5'b01001

//------------------

`define LDR_wait 5'b11111
`define LDR_wait2 5'b11101
`define GetSTR_B 5'b11110
`define GetBX_B 5'b10111
`define WritePC_wait 5'b10001

`define WritePC 5'b11011


// Registers Rm, Rd, Rn encoding
`define Rn  3'b001
`define Rd  3'b010
`define Rm  3'b100

//--------------------------------------
// 5 bits in the order load_addr,reset_pc,load_pc,addr_sel,m_cmd,load_ir
`define Reset_bits 7'b0111000
`define IF1_bits 7'b0001010
`define IF2_bits 7'b0001011
`define UpdatePC_bits 7'b0011000
`define NULL 7'b0001000
`define WritePC_bits 7'b0000000
`define WriteReg_bits 7'b0000100
`define  HALT_bits 7'b0100000

// load_addr,reset_pc,load_pc,addr_sel,m_cmd,load_ir

`define Add_sxim_bits 7'b1001000
`define LDR_bits 7'b1000010
`define STR_OP_bits 7'b0000000
`define STR_bits 7'b0000100
`define GetSTR_B_bits 7'b1001000
`define GetBX_B_bits 7'b1011000

//-------------------------------------- the other bits
//vsel, w,loada, loadb, asel, bsel, loadc, loads, write,nsel
`define Add_sxim_out 15'bxxxx00001100000
`define LDR_out 15'b000000000000000
`define STR_OP_out 15'b000000010110000
`define STR_out 12'b100000000001

`define GetBX_B_wait_out 12'b000000110110


// State outputs encoding in this order:  vsel, w,loada, loadb, asel, bsel, loadc, loads, write,nsel
`define Wait_out 			15'b010010000000000
`define Decode_out 		15'b010000000000010
`define GetA_out 			12'b000101000000
`define GetB_out			12'b000100100000
`define Operate_out		12'b000100000100
`define OperateMOV_out	12'b000100010100
`define WriteReg_out    12'b000100000001
`define WriteRn_out     12'b010000000001
`define ReadRm_out 		12'b000100100000

`define GetBX_B_out     12'b000100110110

`define LDR_wait_out 12'b100000000001


`define WritePC_out 12'b001000000001 // Writes the value of PC to register Rn


//------------------------------ Branch Encodings
`define B 8'b00100000
`define BEQ 8'b00100001
`define BNE 8'b00100010
`define BLT 8'b00100011
`define BLE 8'b00100100
`define BL 8'b01011xxx
`define BX 8'b01000xxx
`define BLX 8'b01010xxx

module cpu(clk,reset,s,load,in,mdata,out,N,V,Z,w,halt, mem_cmd,mem_addr);


  input clk, reset, s, load;
  input [15:0] in, mdata;
  output [15:0] out;// output from datapath
  output N, V, Z, w, halt; // wtf is w????
  output[1:0] mem_cmd;
  output[8:0] mem_addr;
 
  wire [1:0] op, sh, ALUop;
  wire[8:0] data_address;
  wire [2:0] writenum,readnum, nsel, opcode;
  wire [15:0]Rout, sximm5, sximm8;
  wire [23:0] FSM_out;
  wire [2:0]  status;
  wire [3:0] vsel = FSM_out[14:11];
  wire addr_sel = FSM_out[18];
  wire [15:0] datapath_out;
  wire PC_select;
  
  // Not needed until lab 8 
  wire [8:0] PC,to_PC, PC_out;
  wire [15:0] mdata;
  
  assign halt = FSM_out[22];
  assign w = FSM_out[10];
  assign nsel = FSM_out[2:0];
  assign Z = status[2];
  assign N = status[1];
  assign V = status[0];
  
  assign load_addr = FSM_out[21];
  assign reset_pc = FSM_out[20];
  assign load_pc = FSM_out[19];
 
  assign mem_cmd = FSM_out[17:16];
  assign load_ir = FSM_out[15];
  
  assign mem_addr = addr_sel ? PC : data_address;
  assign out = datapath_out;
  wire [15:0] data_out;
  
  //Purely combnational logic
  
  PC_Controller Cont(clk,op, opcode, Rout[10:8], Rout[7:0], Z, N, V, PC, to_PC); // This controls whether we add sxim8 to PC or we don't
 
 
  ProgramCounter PrCo(	.clk (clk),
								.rst (reset_pc),
								.load (load_pc),
								.in (to_PC),
								.out (PC_out));
						
								
assign PC = FSM_out[23] ? data_out[8:0]-1'b1 : PC_out; // If opcode  = 010 and op is x0 then PC gets Rd

vRegister #(9)addr(clk,load_addr,datapath_out[8:0],data_address);//load_addr = FSM_out[21]




  
  vRegister InstructionReg(clk, load_ir, in , Rout);  // instantiated a register with load option NB: parameter was already 16 in module declaration
  decoderblock D1(Rout, nsel, opcode,ALUop,sximm5, sximm8, op,readnum, writenum, sh); // Decoder block instantiation. This feeds into datapath and the instruction reg
  FiniteStateMachine FSM(clk, opcode, op, reset, s, FSM_out); //The  output is in this order: w, loada, loadb, asel, bsel, loadc, loads, write,nsel
  
  datapath DP(.clk(clk),
				.readnum		(readnum), 
				.vsel			(vsel),
				.loada		(FSM_out[9]), 
				.loadb		(FSM_out[8]),
				.shift		(sh),
				.asel			(FSM_out[7]),
				.bsel			(FSM_out[6]),
				.ALUop		(ALUop),
				.loadc		(FSM_out[5]),
				.loads		(FSM_out[4]),
				.writenum	(writenum),
				.write		(FSM_out[3]),
				.sximm8		(sximm8)/*datapath_in*/,
				.sximm5     (sximm5),
				.PC			(PC-{Rout[7],Rout[7:0]}),
				.mdata		(mdata),
				.status_out     (status),
			   .datapath_out	(datapath_out),
				.data_out (data_out)
			);
  
endmodule

module decoderblock(in, nsel, opcode,ALUop,sximm5, sximm8, op,readnum, writenum, sh);

 input [15:0] in;
 input [2:0] nsel;
 output [2:0] opcode, readnum, writenum;
 output [1:0] op, sh, ALUop;
 output [15:0] sximm5, sximm8;
 wire [2:0] Rm, Rd, Rn;
 wire [2:0] MUXout;
 
 
 // Assign each signal to the specified output
 assign opcode = in[15:13];
 assign op = in[12:11];
 assign ALUop = in[12:11];
 assign sximm5 = {{11{in[4]}},in[4:0]}; // sign extend the most significant bit
 assign sximm8 = {{8{in[7]}},in[7:0]};  // sign extend the most significant bit
 assign Rm = in[2:0];
 assign Rd = in[7:5];
 assign Rn = in[10:8];
 assign sh = in[4:3];
 
 assign readnum = MUXout; // Selects which register to write to or read from
 assign writenum = MUXout;
 
 MUX3 Select(Rn, Rd, Rm,nsel,MUXout); 
 
 
endmodule

module MUX3(Rn, Rd, Rm,nsel,MUXout); // Multiplexer to select among three options
parameter k = 3;
 input [k-1:0] Rn, Rd, Rm, nsel;
 output [k-1:0] MUXout;
 
 // Selects the output depending on the value of nsel
 assign MUXout =  ({k{nsel[0]}} & Rn)|
	               ({k{nsel[1]}} & Rd)|
					   ({k{nsel[2]}} & Rm);


endmodule


module FiniteStateMachine (clk, opcode, op, reset, s, out);

 input[2:0] opcode;
 input [1:0] op;
 input reset, s, clk;
 output [23:0] out;  // output is in the order vsel, w, loada, loadb, asel, bsel, loadc, loads, write,nsel from MSB --> LSB
 
 
 reg [23:0] out; // 

 reg [4:0] next_state;
 wire [4:0] present_state;
 reg [19:0] p;
 
 
 
 
 
 vRegister #(5)State(clk,1'b1,next_state,present_state); // to assng present_state to next_state on posedge of s NB: 1'b1 because we want to always load 
 
 // Combinational logic for next state
 always @(*) begin
   casex({s,present_state,opcode,op,reset}) 	// Cancatenated 4 signals that act combinationally to determine the next state
	
	//Transitioning through states for the ADD operation
	//----------------------------------------------------------------------------------------------------------------------
	 12'bxxxxxxxxxxx1: 		 next_state = `Reset; 	 // When reset in on, we don't care what anything else is 
	
	
	 {1'bx,`HALT,6'bxxxxx0}:		 next_state = `HALT;
	 
	 
	 //---------------------------------------------------------------- Added this one

	 
	 
	 
	 
	 {1'bx,`Reset,5'bxxxxx,1'b0}: 		next_state = `IF1; 		// We only care about whether present state was Reset and reset was zero in order to go to IF1
	 {1'bx,`IF1,5'bxxxxx,1'b0}: next_state = `IF2;                // Transitioning from IF1 to IF2
	 {1'bx,`IF2,5'bxxxxx,1'b0}: next_state = `UpdatePC;            // Transitioning from IF2 to UpdatePC
	 {1'bx,`UpdatePC,5'bxxxxx,1'b0}: next_state = `Decode;        // Transitioning from UpdatePC to Decode
	 
	 //-------------------------------------------------------- Transitioning fom Decode
	 
	 
	 {1'bx,`Decode,3'b101,2'bxx,1'b0}:	 next_state = `GetA; 			// Don't care what op is and care about the rest
	 {1'bx,`Decode,3'b110,2'b10,1'b0}:   next_state = `WriteRn; // Now we care about what op is so that we know which one of the MOV are we exercuting
	 {1'bx,`Decode,3'b100,2'bxx,1'b0}:   next_state = `GetA;    // When doing STR
	 {1'bx,`Decode,3'b011,2'bxx,1'b0}:   next_state = `GetA;    // When doing LDR
	 {1'bx,`Decode,3'b110,2'b00,1'b0}: next_state = `ReadRm;        // If doing second MOV operation which reads RM
	 {1'bx,`Decode,3'b111,2'bxx,1'b0}: next_state = `HALT;
	 
	 {1'bx,`Decode,3'b010,2'b11,1'b0}: next_state = `WritePC;
	 {1'bx,`Decode,3'b010,2'b10,1'b0}: next_state = `WritePC;
	 
	 
	 {1'bx,`Decode,3'b010,2'bx0,1'b0}: next_state = `GetBX_B;
	 {1'bx,`GetBX_B,3'b010,2'bx0,1'b0}: next_state = `IF1;
	 
	 
	 {1'bx,`WritePC,3'b010,2'b1x,1'b0}: next_state = `IF1;
	 
	 
	 
	 
	 
//-------------------------------------------------------------------------- Transitions from GetA
	 {1'bx,`GetA,3'b101,2'bxx,1'b0}:			next_state = `GetB; 			// Only care about present_state and reset
	 {1'bx,`GetA,3'b011,2'bxx,1'b0}:      next_state = `Add_sxim;  // If we are doing LDR
	 {1'bx,`GetA,3'b100,2'bxx,1'b0}:       next_state = `Add_sxim;  // If we are doing STR
	
	 
//-------------------------------------------------------------------------- Transitioning from GetB
	 {1'bx,`GetB,3'bxxx,2'bxx,1'b0}:			next_state = `Operate; 	// Only care about present_state and reset
	                                                              //{1'bx,`GetB,3'b100,2'bxx,1'b0}:		next_state = `STR_OP;
//--------------------------------------------------------------------------
	 {1'bx,`Operate,5'bxxx00,1'b0}:		next_state = `WriteReg;
	 {1'bx,`Operate,5'bxxx10,1'b0}:		next_state = `WriteReg;
	 {1'bx,`Operate,5'bxxx11,1'b0}:		next_state = `WriteReg;
	 {1'bx,`Operate,5'bxxx01,1'b0}:		next_state = `IF1; // Making sure we do not write when doing CMP
	 
//------------------------------------------------------------------------- Transitioning from Add_sxim
    {1'bx,`Add_sxim,3'b011,2'bxx,1'b0}: next_state = `LDR;
	 {1'bx,`Add_sxim,3'b100,2'bxx,1'b0}: next_state = `GetSTR_B;
	

	{1'bx,`GetSTR_B,3'b100,2'bxx,1'b0}: next_state = `STR_OP; 
//------------------------------------------------------------------------- Transitioning from LDR
    {1'bx,`LDR,3'bxxx,2'bxx,1'b0}: next_state = `LDR_wait;
	 {1'bx,`LDR_wait,3'bxxx,2'bxx,1'b0}: next_state = `LDR_wait2;
	 {1'bx,`LDR_wait2,3'bxxx,2'bxx,1'b0}: next_state = `IF1;
//-------------------------------------------------------------------------Transitioning from STR
	 {1'bx,`STR,3'bxxx,2'bxx,1'b0}: next_state = `IF1;

//-------------------------------------------------------------------------Transitioning from ReadRd
    {1'bx,`STR_OP,3'bxxx,2'bxx,1'b0}: next_state = `STR;

//-------------------------------------------------------------------------Transitioning from WriteReg
	 {1'bx,`WriteReg,5'bxxxxx,1'b0}:   next_state = `IF1;
	 {1'bx,`WriteReg,5'bxxxxx,1'b0}:   next_state = `IF1;
	 
//------------------------------------------------------------------------- Transitioning WriteRn
	 {1'bx,`WriteRn,5'bxxxxx,1'b0}:   next_state = `IF1;     // Comes back to wait
	 
//------------------------------------------------------------------------- Transitioning ReadRm
	 
	 {1'bx,`ReadRm,5'bxxxxx,1'b0}:		next_state = `OperateMOV; // Since all operations are operated in the ALU
	 
//------------------------------------------------------------------------- Transitioning Operate MOV
	 {1'bx,`OperateMOV,5'bxxxxx,1'b0}: next_state = `WriteReg;
//------------------------------------------------------------------------- Default 
	 default: next_state = `IF1;
	 
endcase
end

// Combinatreional logic to assign the what happens in each state 
//the output bus is this bit order: w,vsel, loada, loadb, asel, bsel, loadc, loads, write,nsel


 always @(*) begin
  case(present_state)
    // assign output depending on the input value.
	`Reset: 			out = {1'b0,1'b0,`Reset_bits,`Wait_out}; //show that we are in wait by assigning w to be one
	`IF1:				out = {1'b0,1'b0,`IF1_bits,`Wait_out};
	`IF2: 			out = {1'b0,1'b0,`IF2_bits,`Wait_out};
	`UpdatePC:		out = {1'b0,1'b0,`UpdatePC_bits,`Wait_out};
	`Decode:    	out = {1'b0,1'b0,`NULL,`Decode_out}; 
	`GetA:			out = {1'b0,1'b0,`NULL,`GetA_out,`Rn};
	`GetB:			out = {1'b0,1'b0,`NULL,`GetB_out,`Rm};
	`Operate:		out = {1'b0,1'b0,`NULL,`Operate_out,3'b000};
	`WriteReg:  	out = {1'b0,1'b0,`WriteReg_bits,`WriteReg_out,`Rd};
	`WriteRn:   	out = {1'b0,1'b0,`NULL,`WriteRn_out,`Rn};
	`ReadRm:  		out = {1'b0,1'b0,`NULL,`ReadRm_out,`Rm};
	`OperateMOV:	out = {1'b0,1'b0,`NULL,`OperateMOV_out,3'b000};
	
//---------------------------------------------------------------
   `Add_sxim:  	out = {1'b0,1'b0,`Add_sxim_bits,`Add_sxim_out};
	`LDR:				out = {1'b0,1'b0,`LDR_bits,`LDR_out};
	`STR_OP:			out = {1'b0,1'b0,`STR_OP_bits,`STR_OP_out};
	`STR:				out = {1'b0,1'b0,`STR_bits,`STR_out,`Rd};
	`LDR_wait:     out = {1'b0,1'b0,`LDR_bits,`LDR_out};
	`LDR_wait2:     out = {1'b0,1'b0,`LDR_bits,`LDR_wait_out,`Rd};
	`GetSTR_B:   	  out = {1'b0,1'b0,`GetSTR_B_bits,`GetB_out,`Rd}; 
	`GetBX_B: 		out = {1'b1,1'b0,`GetBX_B_bits,`GetBX_B_out,`Rd};

	               
	`WritePC: 		out = {1'b0,1'b0,`WritePC_bits,`WritePC_out,`Rn};
	`HALT:			out = {1'b0,1'b1,`HALT_bits,15'b0};
	
	 default:   	out = {1'b0,1'b0,`NULL,`Wait_out};// To avoide inferred latches
	 
	endcase
end




endmodule

module ProgramCounter(clk,load,rst,in,out) ;
  input rst, clk ; // reset and clock
  input load; // added for lab 7
  input [8:0] in;
  output [8:0] out ;
  reg	[8:0] next ;
  
  vRegister #(9) count(clk, load, next, out) ;
  
  always @(*) begin
	case(rst)
  	1'b1: next = 9'b000000000 ;
  	1'b0: next = in + 1 ;
  	default: next = 9'bxxxxxxxxx;
	endcase 
  end
endmodule

module PC_Controller(clk, op, opcode, cond, im8, Z, N, V, in, out);// Out is what is coming from PC out, in is what is being fed into the PC
input clk, Z, N, V;
input [2:0] opcode, cond;
input [1:0] op;
input [7:0] im8; 
input [8:0] in;
output [8:0] out;

wire [8:0] sxim8 = {im8[7],im8};
reg [8:0] out;

always @(*) begin
 
 // Checking for opcode op and condition and then determining what value is given to next out
  
  casex({opcode,op,cond})
  `B: out = in + sxim8; // basically PC will get an input of PC + sxim8
  `BEQ: out = Z ? in + sxim8: in; // if Z = 1 PC = PC + sxim8 else PC = PC
  `BNE: out = Z ? in : in + sxim8; // if Z = 0 PC = PC + sxim8 else PC = PC
  `BLT: out = N!=V ? in + sxim8 : in; // if N!=V PC = PC + sxim8 else PC = PC
  `BLE: out = (N!=V)|Z ? in + sxim8 : in; // if N!=V PC = PC + sxim8 else PC = PC
  `BL:  out = in + sxim8;
  
  
  default: out = in;
  
  endcase
 

end
endmodule

// Test bench to test our decoder block
module Testblock; 
 reg [15:0] in;
 reg [2:0] nsel;
 wire [2:0] opcode, readnum, writenum;
 wire [1:0] op, sh, ALUop;
 wire [15:0] im5, im8;
 
 
 decoderblock  DUT(in, nsel, opcode,ALUop,im5, im8, op,readnum, writenum, sh);
 initial begin
 in = 16'b1100000111110100; // Assign arbitrary values to in and see how the decoder assigns values to the output signals
 nsel = 3'b000;
 #5; // wait 5 ns
 in = 16'b0000001110001010;
 nsel = 3'b010;
 #5;
 in = 16'b1111101111111101;
 nsel = 3'b100;
 #5;
 end
endmodule
