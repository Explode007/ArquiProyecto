module ram (
    output reg [31:0] DataOut,  
    input E,               
    input RW,                   
    input [8:0] A,              
    input [31:0] DataIn,        
    input Size           
);

    reg [7:0] Mem [0:255];  

  always @(E) begin
        if (RW == 0) begin  
                case (Size)
                    0: DataOut = {24'b0, Mem[A]};  
                    1: DataOut = {Mem[A], Mem[A+1], Mem[A+2], Mem[A+3]};   
                endcase
            end

        if (RW == 1) begin  
            case (Size)
                0: Mem[A] = DataIn[7:0]; 
                1: begin  
                    Mem[A] = DataIn[7:0];   
                    Mem[A+1] = DataIn[15:8];
                    Mem[A+2] = DataIn[23:16];
                    Mem[A+3] = DataIn[31:24]; 
                end
            endcase
        end
    end

endmodule
