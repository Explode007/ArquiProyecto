module PC (
    output reg [31:0] PC_Out, 
    input [31:0] PC_In, 
    input E, Reset, clk 
    );

    always @ (posedge clk) begin
        if (Reset) PC_Out <= 32'b00000000000000000000000000000000;
        else if (E) PC_Out <= PC_In; 
    end
endmodule
