module ALU(Ain,Bin,ALUop,out,status);

 input [15:0] Ain, Bin;
 input [1:0] ALUop;
 output [15:0] out;
 output [2:0]status;
reg[15:0] out;
wire [7:0] as_out;



//sub = ALUop == 2'b01;
AddSub #(8)add(Ain[7:0], Bin[7:0], ALUop[0],as_out, ovf); // Add or subtract and then assign the value for overflow accordingly


always@(*)begin
// Assign output depending on the ALUop
  case (ALUop)
 2'b00: out = Ain + Bin; //{{8{as_out[7]}},as_out};
 2'b01: out = Ain - Bin; //{{8{as_out[7]}},as_out};
 2'b10: out = Ain & Bin;
 2'b11: out = ~Bin;
 default out = 16'bxxxxxxxxxxxxxxxx;
endcase
end

wire Z = (out === 16'b0) ? 1'b1:1'b0;
wire neg = out[15] ? 1'b1:1'b0;
assign status = {Z,neg,ovf};
 
endmodule


module AddSub(a, b, sub, s, ovf);
  parameter n = 8;
 input [n-1:0] a, b;
 input sub; 
 
 output [n-1:0] s;
 output ovf;
 
 wire c1, c2;
 wire ovf = c1 ^ c2;
//adding non sign bits
 Adder1 #(n-1)ai(a[n-2:0], b[n-2:0]^{n-1{sub}},sub,c1,s[n-2:0]);
//adding sign bits
 Adder1 #(1)as(a[n-1],b[n-1]^sub,c1,c2,s[n-1]);
endmodule


module Adder1(a,b,cin,cout,s);
  parameter n = 8;
 input [n-1:0] a, b;
 input cin;
 output [n-1:0] s;
 output cout;
 wire [n-1:0] s;
 wire cout;
 assign {cout, s} = a + b + cin;
endmodule


