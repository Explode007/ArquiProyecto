module mux(
    output reg [31:0] Y, 
    input [3:0] R,
    input [31:0] Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15
);

always @(*) begin
case(R)
4'b0000: Y = Q0;
4'b0001: Y = Q1;
4'b0010: Y = Q2;
4'b0011: Y = Q3;
4'b0100: Y = Q4;
4'b0101: Y = Q5;
4'b0110: Y = Q6;
4'b0111: Y = Q7;
4'b1000: Y = Q8;
4'b1001: Y = Q9;
4'b1010: Y = Q10;
4'b1011: Y = Q11;
4'b1100: Y = Q12;
4'b1101: Y = Q13;
4'b1110: Y = Q14;
4'b1111: Y = Q15;
endcase
end
endmodule