module shifter(in,shift,sout);
// input and output declarations
  input [15:0] in;
  input [1:0] shift;
  output [15:0] sout;

  reg [15:0] sout; // Assigned in the always block
 
 
 always @(*) begin
 case(shift) // Assign sout depending on the case of shift
  2'b00: sout = in;
  2'b01: sout = {in[14:0],1'b0};
  2'b10: sout = {1'b0, in[15:1]};
  2'b11: sout = {in[15], in[15:1]};
  default sout = 16'bxxxxxxxxxxxxxxxx; // avoid inferred latches
 endcase
 end
endmodule

