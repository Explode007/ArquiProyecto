module ALU (
    input [3:0] Op,     
    input [31:0] A, B,
    input CIN,            
    
    output reg [31:0] Out, 
    output reg Z, N, C, V 
);
    parameter OP_ADD  = 4'b0000;
    parameter OP_ADD_CIN = 4'b0001;
    parameter OP_A_SUB_B  = 4'b0010;
    parameter OP_A_SUB_B_CIN = 4'b0011;
    parameter OP_B_SUB_A = 4'b0100;
    parameter OP_B_SUB_A_CIN = 4'b0101;
    parameter OP_AND = 4'b0110;
    parameter OP_OR  = 4'b0111;
    parameter OP_XOR = 4'b1000;
    parameter OP_A_TRANSFER = 4'b1001;
    parameter OP_B_TRANSFER = 4'b1010;
    parameter OP_NOT_B = 4'b1011;
    parameter OP_A_AND_NOT_B = 4'b1100;
  	
    //When using {C, Out}, its like we have a 33 bit 'space', so that the 33rd bit falls into C and is not lost.
  
    always @(*) begin

        C = 1'b0; //Carry 0 by default, only updates it if the instruction passes a carry bit. 
                  //Saves us from saying C =1'b0 on every non arithmetic operation. Which means we get 1 liners for all cases.
        case (Op)
        
            OP_ADD:          {C, Out} = A + B; 
            OP_ADD_CIN:      {C, Out} = A + B + CIN; 
            OP_A_SUB_B:      {C, Out} = A - B; 
            OP_A_SUB_B_CIN:  {C, Out} = A - B - CIN;
            OP_B_SUB_A:      {C, Out} = B - A;
            OP_B_SUB_A_CIN:  {C, Out} = B - A - CIN;
            OP_AND:          Out = A & B;
            OP_OR:           Out = A | B;
           	OP_XOR:          Out = A ^ B;
            OP_A_TRANSFER:   Out = A;
            OP_B_TRANSFER:   Out = B;
            OP_NOT_B:        Out = ~B;
            OP_A_AND_NOT_B:  Out = A & (~B);

            default:         {C, Out} = {1'b0, 32'b0};
        endcase

        // Flags
          Z = (Out == 32'b0);
      	  N = Out[31]; //MSB of Result is the Sign Flag
    
        // Overflow 
        if (Op == OP_ADD || Op == OP_ADD_CIN) begin
            // When adding 
            V = (A[31] == B[31]) && (A[31] != Out[31]);
          
        end else if (Op == OP_A_SUB_B || Op == OP_A_SUB_B_CIN || Op == OP_B_SUB_A || Op == OP_B_SUB_A_CIN) begin
            // When substracting 
            V = (A[31] != B[31]) && (A[31] != Out[31]);
          
        end else begin
            V = 1'b0; // Logical operation -> No overflow
        end
    end
endmodule


module Shifter_SignExtender (
    input [31:0] Rm,  
    input [11:0] I,   
    input [1:0] AM,   

    output reg [31:0] N  
);

    //Shift Types
    parameter LSL = 2'b00;  // Logical Shift Left
    parameter LSR = 2'b01;  // Logical Shift Right
    parameter ASR = 2'b10;  // Arithmetic Shift Right (preserves signed number)
    parameter ROR = 2'b11;  // Rotate Right

    //Mode Control
    parameter ROTATE_RIGHT = 2'b00;
    parameter PASS_RM = 2'b01;
    parameter ZERO_EXTEND = 2'b10;
    parameter SHIFT_RM = 2'b11;

    always @(*) begin
        case (AM)
            ROTATE_RIGHT:   N = {24'b0, I[7:0]} >> (2 * I[11:8]); // Rotate right {0x000, I[7:0]} by 2 * I[11:8] positions
            PASS_RM:        N = Rm;
            ZERO_EXTEND:    N = {20'b0, I[11:0]};                 
            SHIFT_RM: begin                                       // Shift Rm by I[11:7] positions with different shift types based on I[6:5]
                case (I[6:5]) //This is AM
                    LSL: N = Rm << I[11:7];   
                    LSR: N = Rm >> I[11:7];   
                    ASR: N = $signed(Rm) >>> I[11:7]; // >>> Triple arrow means arithmetic shift. In a signed rotate, the MSB is copied on the left of the resulting shift
                    ROR: N = (Rm << (32 - I[11:7])) | (Rm >> I[11:7]) ;  
                    default: N = 32'b0;
                endcase
            end
            default: N = 32'b0; 
        endcase
    end
endmodule

// Module I use to combine ALU and Shifter_SignExtender
module Alu_and_Shifter (
    //ALU wires
    input wire [31:0] alu_a, alu_b,    
    input wire alu_cin,
    input wire [3:0] alu_op,     

    output wire [31:0] alu_out,
    output wire Z, N, C, V,

    //Shifter wires
    input wire [31:0] shifter_rm,        
    input wire [11:0] shifter_i,
    input wire [1:0] shifter_am,         

    output wire [31:0] shifter_n
);

    ALU alu (
        .A(alu_a),                  
        .B(alu_b),  
        .CIN(alu_cin),               
        .Op(alu_op),                
        .Out(alu_out),       
        .Z(Z),                   
        .N(N),                   
        .C(C),
        .V(V)
    );


    Shifter_SignExtender shifter (
        .Rm(shifter_rm),
        .I(shifter_i),
        .AM(shifter_am),
        .N(shifter_n)
    );
endmodule


module Alu_and_Shifter_Testing;
    //ALU vars
    reg [31:0] alu_a, alu_b;
    reg alu_cin;
    reg [3:0] alu_op;     

    wire [31:0] alu_out;
    wire Z, N, C, V;

    //Shifter vars
    reg [31:0] shifter_rm;        
    reg [11:0] shifter_i;
    reg [1:0] shifter_am;         

    wire [31:0] shifter_n;

    //Test vars
    reg [3:0] testnum;
    reg clk;
    integer cycles;

    Alu_and_Shifter testmodule (
        .alu_a(alu_a),
        .alu_b(alu_b),
        .alu_cin(alu_cin),
        .alu_op(alu_op),
        .alu_out(alu_out),
        .Z(Z),                   
        .N(N),                   
        .C(C),
        .V(V),
        .shifter_rm(shifter_rm),
        .shifter_i(shifter_i),
        .shifter_am(shifter_am),
        .shifter_n(shifter_n)
    );

    //CLOCK
    initial begin 
        clk = 0;
        forever #1 clk = ~clk;
    end
        
    //TEST SETUP
    initial begin
        alu_a = 32'b10011100000000000000000000111000;  
        alu_b = 32'b10011100000000000000000000111000;
        alu_cin = 0;
        alu_op = 4'b0000;

        shifter_rm = 32'b10000100001100011111111111101010; 
        shifter_i = 12'b000101101100;                      
        shifter_am = 2'b00;                                

        testnum = 0;
        cycles = 0;
    end
    always @(*) begin
        if (testnum == 0 || testnum == 1) begin
            $monitor("Cycle: %d | TestNum: %d | ALU_Op: %b | CIN: %d | ALU Out: %b | Z: %d | N: %d | C: %d | V: %d",
                      cycles, testnum, alu_op, alu_cin, alu_out, Z, N, C, V);
        end else if (testnum == 2) begin
            $monitor("Cycle: %d | TestNum: %d | Shift Out: %b | AM: %b | I[6:5]: %b",
                      cycles, testnum, shifter_n, shifter_am, shifter_i[6:5]);
        end
    end
    //TESTING LOGIC
    always @(posedge clk) begin
        cycles = cycles + 1;
            case (testnum)
                0: begin  // ALU with CIN = 0
                    if (cycles % 2 == 0) begin
                        if (alu_op == 4'b1100) begin
                            alu_cin = 1;  //Change CIN to 1 before moving to next test
                            alu_op = 4'b0000;
                            testnum = testnum + 1;
                        end else begin
                            alu_op = alu_op + 1;
                        end
                    end
                end

                1: begin  // ALU with CIN = 1
                    if (cycles % 2 == 0) begin
                        if (alu_op == 4'b0101) begin
                            testnum = testnum + 1;  // Move to Shifter test
                        end else begin
                            alu_op = alu_op + 1;
                        end
                    end
                end

                2: begin  // Shifter test
                    if (shifter_am == 2'b11) begin
                        if (shifter_i[6:5] == 2'b11) begin
                            testnum = testnum + 1;
                        end else begin //Go through all I[6:5] Scenarios
                            shifter_i[6:5] = shifter_i[6:5]+1;
                        end
                    end else begin
                        shifter_am = shifter_am + 1;
                    end
                end

                3: begin  
                    $stop; 
                end
            endcase
    end

endmodule

