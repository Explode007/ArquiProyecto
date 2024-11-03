module PSRmux(
    input PSR_in,
    input ALU_in,
    input S_bit_in,
    output reg Flags_out
);

    always @(*) begin
        case (S_bit_in)
        1'b0: Flags_out = ALU_in;
        1'b1: Flags_out = PSR_in;       
        endcase
    end
endmodule

