module rom (
    output reg [31:0] I, 
    input [7:0] A 
    );

    reg [7:0] Mem [0:255];
    
    always @(A)
        I = { Mem[A], Mem[A+1], Mem[A+2], Mem[A+3] };

endmodule
