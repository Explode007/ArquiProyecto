module decoder (
    output reg[15:0] regnum,
    input LE,
    input [3:0] RW
);

always @(*)
begin
 if(LE == 1'b1)
begin
    case(RW)
    4'b0000: regnum = 16'b0000000000000001;
    4'b0001: regnum = 16'b0000000000000010;
    4'b0010: regnum = 16'b0000000000000100; 
    4'b0011: regnum = 16'b0000000000001000; 
    4'b0100: regnum = 16'b0000000000010000;
    4'b0101: regnum = 16'b0000000000100000; 
    4'b0110: regnum = 16'b0000000001000000; 
    4'b0111: regnum = 16'b0000000010000000; 
    4'b1000: regnum = 16'b0000000100000000; 
    4'b1001: regnum = 16'b0000001000000000; 
    4'b1010: regnum = 16'b0000010000000000; 
    4'b1011: regnum = 16'b0000100000000000; 
    4'b1100: regnum = 16'b0001000000000000; 
    4'b1101: regnum = 16'b0010000000000000; 
    4'b1110: regnum = 16'b0100000000000000;     
    4'b1111: regnum = 16'b1000000000000000;
 endcase
end else regnum = 16'b0000000000000000;
end
endmodule