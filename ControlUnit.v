module control_unit (
    input [31:0] instruction,  
    output reg rf_en,          
    output reg [3:0] alu_op,   
    output reg Load,            // 1 for Load (LDR), 0 for Store (STR)
    output reg branch_link,    // 1 for Branch & Link (BL), 0 for Branch (B)
    output reg s_bit,          // S bit for updating the PSR (Program Status Register)
    output reg rw,             // Read/Write signal: 1 for read (load), 0 for write (store)
    output reg size,           // Size: 0 for byte, 1 for word
    output reg datamem_en, // Data memory enable for load/store
    output reg [1:0] AM
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

    parameter ROTATE_RIGHT = 2'b00;
    parameter PASS_RM = 2'b01;
    parameter ZERO_EXTEND = 2'b10;
    parameter SHIFT_RM = 2'b11;

    // Extract fields from the instruction
    wire [2:0] category = instruction[27:25]; // Instruction category field (27:25)
    wire [3:0] opcode = instruction[24:21];   // ALU opcode field (24:21)
    wire I = instruction[25];                 // Immediate bit (I) for data processing | Register/Immediate offset bit for load/store (R)
    wire S = instruction[20];
    wire bit4 = instruction[4];
    
    always @(*) begin
        // Default values
        rf_en = 0;
        alu_op = 4'b0000;
        Load = 0;
        branch_link = 0;
        s_bit = 0;
        rw = 0;
        size = 0;  // Default to word (32 bits)
        datamem_en = 0;
        
        AM = 2'b00;  // Default AM signal

        if (instruction == 32'b0) begin
            rf_en = 0;
            alu_op = 4'b0;
            Load = 0;
            AM = 0;
            branch_link = 0;
            s_bit = 0;
            rw = 0;
            size = 0;
        end else begin
            casez (category)
                3'b00z: begin  // Data Processing (00I)
                    rf_en = 1;
                    s_bit = S;  

                    //How to shifter
                    if (I) begin
                        AM = ROTATE_RIGHT;
                    end else begin
                        if (bit4 == 0) begin
                            AM = SHIFT_RM; //Normal immediate shift Rm by an Immediate
                        end
                    end
                
                    case (opcode)
                        4'b0000: alu_op = OP_ADD;           
                        4'b0001: alu_op = OP_ADD_CIN;       
                        4'b0010: alu_op = OP_A_SUB_B;       
                        4'b0011: alu_op = OP_A_SUB_B_CIN;   
                        4'b0100: alu_op = OP_B_SUB_A;       
                        4'b0101: alu_op = OP_B_SUB_A_CIN;   
                        4'b0110: alu_op = OP_AND;           
                        4'b0111: alu_op = OP_OR;            
                        4'b1000: alu_op = OP_XOR;           
                        4'b1001: alu_op = OP_A_TRANSFER;    
                        4'b1010: alu_op = OP_B_TRANSFER;    
                        4'b1011: alu_op = OP_NOT_B;           
                        4'b1100: alu_op = OP_A_AND_NOT_B;     
                        default: alu_op = 4'b0000;            
                    endcase
                end

                3'b01z: begin
                                        
                    Load = instruction[20]; // 1 for Load (LDR), 0 for Store (STR)
                    rw = instruction[20]; // 1 for read, 0 for write
                    size = instruction[22]; // Size: 1 for byte, 0 for word
                    datamem_en = 1;

                    if (I == 0) begin
                        AM = ZERO_EXTEND; // For Offset-12
                    end else begin
                        if (instruction[11:4] == 0) begin
                            AM = PASS_RM; // Register Offset
                        end else begin 
                            AM = SHIFT_RM; // Scaled Register Offset
                        end
                    end
                end

                3'b101: begin  // Branch (101)
                    branch_link = instruction[24]; // 1 for BL (Branch & Link), 0 for B (Branch)
                end

                default: begin
                    // No operation
                end
            endcase
        end
    end
endmodule
