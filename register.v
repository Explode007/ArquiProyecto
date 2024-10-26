module register(
    input LE, Clk,
    input [31:0] PW,
    output reg[31:0] Q
);

always @(posedge Clk)
    if(LE) Q = PW;
endmodule