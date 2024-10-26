`include "decoder.v"
`include "register.v"
`include "mux.v"

module register_file(
    input LE, Clk,
    input [31:0] PC,PW,
    input[3:0] RD,RB,RA,RW,
    output[31:0] PD,PB,PA
);

wire [15:0] regnum;
wire [31:0] Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15;


//declarando los modulos individuales las veces necesarias para hacer un three port register file y haciendo las 
//conneciones necesarias
decoder decoder1(regnum,LE,RW);

register R0(regnum[0], Clk, PW, Q0);
register R1(regnum[1], Clk, PW, Q1);
register R2(regnum[2], Clk, PW, Q2);
register R3(regnum[3], Clk, PW, Q3);
register R4(regnum[4], Clk, PW, Q4);
register R5(regnum[5], Clk, PW, Q5);
register R6(regnum[6], Clk, PW, Q6);
register R7(regnum[7], Clk, PW, Q7);
register R8(regnum[8], Clk, PW, Q8);
register R9(regnum[9], Clk, PW, Q9);
register R10(regnum[10], Clk, PW, Q10);
register R11(regnum[11], Clk, PW, Q11);
register R12(regnum[12], Clk, PW, Q12);
register R13(regnum[13], Clk, PW, Q13);
register R14(regnum[14], Clk, PW, Q14);

mux mux1(PA,RA,Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,PC);

mux mux2(PB,RB,Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,PC);

mux mux3(PD,RD,Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,PC);

endmodule