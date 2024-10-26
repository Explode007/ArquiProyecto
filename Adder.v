module adder(
  output reg [31:0] Adder_OUT,
  input[31:0] Adder_IN
  );
  always@(Adder_IN) 
        begin
             Adder_OUT = Adder_IN + 32'b100;
        end
endmodule
