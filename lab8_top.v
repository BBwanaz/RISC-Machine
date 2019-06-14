`define MREAD 2'b01
`define MWRITE 2'b10

module lab8_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5, CLOCK_50);
input [3:0] KEY;
input CLOCK_50;
input [9:0] SW;
output[9:0] LEDR;
output[6:0] HEX0, HEX1,HEX2, HEX3, HEX4, HEX5;

wire [15:0] read_data, data_out;
wire [8:0] mem_addr;
wire [1:0] mem_cmd;
wire [15:0] out;
wire [15:0] dout,din;
wire [15:0] write_data;

wire clk = CLOCK_50;

RAM MEM(.clk 		(clk),
	.read_address	(mem_addr[7:0]),
	.write_address	(mem_addr[7:0]),
	.write		(write),
	.din		(write_data),
	.dout		(data_out) );

 cpu CPU( .clk (clk),
	.reset (~KEY[1]),
	.s     (~KEY[2]),
        .load  (~KEY[3]),
        .in    (read_data),//(ir),
	    .mdata (read_data),
        .out   (write_data),
        .Z     (Z),
        .N     (N),
        .V     (V),
        .w     (LEDR[9]),
		  .halt  (halt_signal),
	.mem_addr (mem_addr),
	.mem_cmd (mem_cmd) );

assign write_data = out;


wire msel = mem_addr[8] == 1'b0;
wire mread = mem_cmd == `MREAD;
wire mwrite = mem_cmd == `MWRITE;

wire tri_state_d = msel & mread;

assign write = mwrite & msel;

TriStateDriver #(16,16)drive0(tri_state_d , data_out , read_data);

//assign read_data = tri_state_d ? dout : 1'bz;


SwitchTSD switch(mem_cmd, mem_addr, sw_tsd);
TriStateDriver #(16,8)drive1(sw_tsd, SW[7:0], read_data);
//assign read_data = sw_tsd ? {{8{1'b0}},SW[7:0]} : 16'bz;

LedRREG ledr0(mem_cmd, mem_addr, led_reg);
vRegister #(8)ledr1(clk, led_reg, write_data[7:0], LEDR[7:0]);



  assign HEX5[0] = ~Z;
  assign HEX5[6] = ~N;
  assign HEX5[3] = ~V;

  // fill in sseg to display 4-bits in hexidecimal 0,1,2...9,A,B,C,D,E,F
  sseg H0(out[3:0],   HEX0);
  sseg H1(out[7:4],   HEX1);
  sseg H2(out[11:8],  HEX2);
  sseg H3(out[15:12], HEX3);
  assign HEX4 = 7'b1111111;
  assign {HEX5[2:1],HEX5[5:4]} = 4'b1111; // disabled
  assign LEDR[8] = halt_signal;
endmodule

module SwitchTSD (mem_cmd, mem_addr, sw_tsd);
input [1:0] mem_cmd;
input [8:0] mem_addr;
output sw_tsd;
wire a;
wire b;

assign a = mem_cmd == `MREAD;
assign b = mem_addr == 9'h140;

assign sw_tsd = a & b;

//assign sw_tsd = mem_cmd == `MREAD ? (mem_addr == 9'h140 ? 1'b1 : 1'b0) : 1'b0;

endmodule

module LedRREG (mem_cmd, mem_addr, led_reg);
input [1:0] mem_cmd;
input [8:0] mem_addr;
output led_reg;
wire a;
wire b;

assign a = mem_cmd == `MWRITE;
assign b = mem_addr == 9'h100;

assign led_reg = a & b;

//assign led_reg = mem_cmd == `MWRITE ? (mem_addr == 9'h100 ? 1'b1 : 1'b0) : 1'b0;

endmodule

module TriStateDriver(cond, in, out);
parameter n = 16;
parameter m = 8;
input cond;
input [m-1:0] in;
output [n-1:0] out;

assign out = cond ? {{n-m{1'b0}},in} : {n{1'bz}};

endmodule


/*module input_iface(clk, SW, ir, LEDR);
  input clk;
  input [9:0] SW;
  output [15:0] ir;
  output [7:0] LEDR;
  wire sel_sw = SW[9];  
  wire [15:0] ir_next = sel_sw ? {SW[7:0],ir[7:0]} : {ir[15:8],SW[7:0]};
  vDFF #(16) REG(clk,ir_next,ir);
  assign LEDR = sel_sw ? ir[7:0] : ir[15:8];  
endmodule         */

/*module vDFF(clk,D,Q);
  parameter n=1;
  input clk;
  input [n-1:0] D;
  output [n-1:0] Q;
  reg [n-1:0] Q;
  always @(posedge clk)
    Q <= D;
endmodule*/




module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 16; 
  parameter addr_width = 8;
  parameter filename = "lab8fig2.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @ (posedge clk) begin
    if (write)
      mem[write_address] <= din;
    dout <= mem[read_address]; // dout doesn't get din in this clock cycle 
                               // (this is due to Verilog non-blocking assignment "<=")
  end 
endmodule

module sseg(in,segs);
  input [3:0] in;
  output [6:0] segs;

  // NOTE: The code for sseg below is not complete: You can use your code from
  // Lab4 to fill this in or code from someone else's Lab4.  
  //
  // IMPORTANT:  If you *do* use someone else's Lab4 code for the seven
  // segment display you *need* to state the following three things in
  // a file README.txt that you submit with handin along with this code: 
  //
  //   1.  First and last name of student providing code
  //   2.  Student number of student providing code
  //   3.  Date and time that student provided you their code
  //
  // You must also (obviously!) have the other student's permission to use
  // their code.
  //
  // To do otherwise is considered plagiarism.
  //
  // One bit per segment. On the DE1-SoC a HEX segment is illuminated when
  // the input bit is 0. Bits 6543210 correspond to:
  //
  //    0000
  //   5    1
  //   5    1
  //    6666
  //   4    2
  //   4    2
  //    3333
  //
  // Decimal value | Hexadecimal symbol to render on (one) HEX display
  //             0 | 0
  //             1 | 1
  //             2 | 2
  //             3 | 3
  //             4 | 4
  //             5 | 5
  //             6 | 6
  //             7 | 7
  //             8 | 8
  //             9 | 9
  //            10 | A
  //            11 | b
  //            12 | C
  //            13 | d
  //            14 | E
  //            15 | F

 
reg [6:0] segs;
 
always @(*) begin
 case(in)
4'b0000: segs = 7'b1000000; // 0
4'b0001: segs = 7'b1111001; // 1
4'b0010: segs = 7'b0100100; // 2
4'b0011: segs = 7'b0110000; // 3
4'b0100: segs = 7'b0011001; // 4
4'b0101: segs = 7'b0010010; // 5
4'b0110: segs = 7'b0000010; // 6
4'b0111: segs = 7'b1111000; // 7
4'b1000: segs = 7'b0000000; // 8
4'b1001: segs = 7'b0010000; // 9
4'b1010: segs = 7'b0001000; // A
4'b1011: segs = 7'b0000011; // b
4'b1100: segs = 7'b1000110; // C
4'b1101: segs = 7'b0100001; // d
4'b1110: segs = 7'b0000110; // E
4'b1111: segs = 7'b0001110;  //F
 default segs = 7'b1000000; // default statement to avoid infered latches
endcase
end
endmodule