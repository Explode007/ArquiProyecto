//TODO: Possibly Redo this with 32 bits rather than just the necessary flags
module ProgramStatusRegister(
    input S_bit_in,
    
    //'Condition Codes'
    input Z_in,
    input N_in,
    input C_in,
    input V_in,

    //'Flags' Out
    output reg Z_out,
    output reg N_out,
    output reg C_out,
    output reg V_out
    );

    always @* begin
        if (S_bit_in) begin
            Z_out <= Z_in;
            N_out <= N_in;
            C_out <= C_in;
            V_out <= V_in;
        end
    end

endmodule
