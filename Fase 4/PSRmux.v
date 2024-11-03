module PSRmux(
    input Z_PSR_in,
    input N_PSR_in,
    input C_PSR_in,
    input V_PSR_in,

    input Z_ALU_in,
    input N_ALU_in,
    input C_ALU_in,
    input V_ALU_in,
    
    input S_bit_in,
    
    output reg Z_out,
    output reg N_out,
    output reg C_out,
    output reg V_out
);

    always @(*) begin
        case (S_bit_in)
            1'b0: begin
                Z_out = Z_ALU_in;
                N_out = N_ALU_in;
                C_out = C_ALU_in;
                V_out = V_ALU_in;
            end
            1'b1: begin
                Z_out = Z_PSR_in;
                N_out = N_PSR_in;
                C_out = C_PSR_in;
                V_out = V_PSR_in;
            end
        endcase
    end
endmodule
