module PPU();
    //======Simulation======//

        //======Init Stuff======//
            //Generate clock signal, toggles every 2 time units
            always begin
                clk <= 0;
                #2 clk <= ~clk;
            end
            //Preload
            integer fi, code;
            reg [7:0] data;
            reg [8:0] Address;
            initial begin
                // Open and read the test code file
                fi = $fopen("precharge.txt", "r");
                Address = 0;
                while (!$feof(fi)) begin
                    code = $fscanf(fi, "%b", data);  // Read binary instructions
                    insMem.Mem[Address] = data;      // Load instructions into ROM
                    Address = Address + 1;
                end
                $fclose(fi);  // Close the file

                // Initialize the signals
                LE <= 1'b1;
                rst <= 1'b1;
                s <= 1'b0;
                
                // Do required timed changes
                #3 rst <= 1'b0;
                #32 s <= 1'b1;
                
            end

            initial begin
                #40 $finish; //41 so it can print out 40
            end

            reg [167:0] instruction_name;

            always @ (*) begin

                case (cu_in)
                    32'b00000000000000000000000000000000: instruction_name = "NOP";
                    32'b11100010000100010000000000000000: instruction_name = "ANDS R0,R1, #0";
                    32'b11100000100000000101000110000011: instruction_name = "ADD R5,R0,R3, LSL #3";
                    32'b11100111110100010010000000000000: instruction_name = "LDRB R2, [R1,R0]";
                    32'b11100101100010100101000000000000: instruction_name = "STR R5, [R10, #0]";
                    32'b00011010111111111111111111111101: instruction_name = "BNE -3";
                    32'b11011011000000000000000000001001: instruction_name = "BLLE+9";
                    32'b11100010000000010000000000000000: instruction_name = "AND R0,R1, #0";
                    default: instruction_name = "NULL";
                endcase

            end


        //======Monitor=======//
            always begin
                #1

                $monitor("Current Time Unit = %t\nIns. = %s %b\nPC = %d\nID_STAGE: AM = %b\tS-Bit = %b\tDATAMEM_EN = %b\tRW = %b\tSize = %b\tRF_EN = %b\tALU_OP = %b\tLoad = %b\tBranch&Link = %b\tBranch = %b\nEX_STAGE: AM = %b\tS_Bit = %b\tDATAMEM-EN = %b\tR/W = %b\tSIZE = %b\tRF_EN = %b\tALU_OP = %b\tLoad = %b\nMEM_STAGE: RF_EN = %b\tDATAMEM-EN = %b\tR/W = %b\tSIZE = %b\tLoad = %b\nWB_STAGE: RF_EN = %b\n", 
                $time, instruction_name, cu_in, PC_Out, am_cu_out, s_bit_cu_out, datamem_en_cu_out, rw_cu_out, size_cu_out, rf_en_cu_out, alu_op_cu_out, Load_cu_out, branch_link_cu_out, branch_cu_out, //First Line
                am_out_idexe, s_bit_out_idexe, datamem_en_out_idexe, rw_out_idexe, size_out_idexe, rf_en_out_idexe, alu_op_out_idexe, Load_out_idexe, //Second Line
                rf_en_out_exemem, datamem_en_out_exemem, rw_out_exemem, size_out_exemem, Load_out_exemem, //Third Line
                rf_en_out_memwb //Fourth Line
                );
            end




    //==================INSTANTIATION===================//
        //Generic Wires
        reg LE;
        reg rst;
        reg clk;
        
        //SPACE FOR PHASE 4 STUFF vvv (Remember to make wires in the Pipeline)
        
        //Condition Handler

            //These are some ALU WIRES
            wire [3:0] condition_codes;

        wire TA_Ctrl_out;
        wire bl_condition_out;
        wire COND_EVAL_out;
        ConditionHandler condhandler (
            .B_in(B_signal),
            .BL_in(BL_signal),
            .I_Cond_in(condition_code),
            .Z_in(Z_out),
            .N_in(N_out),
            .C_in(C_out),
            .V_in(V_out),
            .TA_Ctrl_out(TA_Ctrl_out),
            .BL_COND_out(bl_condition_out),
            .COND_EVAL_out(COND_EVAL_out)
        );


        //PSR MUX
        wire Z_out, N_out, C_out, V_out;
        wire Z_alu, N_alu, C_alu, V_alu; //TODO: Place with ALU 
        PSRmux psrmux(
            .S_bit_in(s_bit_out_cumux),

            .Z_PSR_in(Z_psr),
            .N_PSR_in(N_psr),
            .C_PSR_in(C_psr),
            .V_PSR_in(V_psr),

            .Z_ALU_in(Z_alu),
            .N_ALU_in(N_alu),
            .C_ALU_in(C_alu),
            .V_ALU_in(V_alu),

            .Z_out(Z_out),
            .N_out(N_out),
            .C_out(C_out),
            .V_out(V_out)
        );

        //Program Status Register
        wire Z_psr, N_psr, C_psr, V_psr;
        ProgramStatusRegister PSR(
            .S_bit_in(s_bit_out_cumux),

            .Z_in(Z_alu),
            .N_in(N_alu),
            .C_in(C_alu),
            .V_in(V_alu),

            .Z_out(Z_psr),
            .N_out(N_psr),
            .C_out(C_psr),
            .V_out(V_psr)
        )
        //SPACE FOR PHASE 4 STUFF ^^^ 


        //Program Counter
        wire [31:0] PC_Out; 
        wire [31:0] PC_In; 
        PC PCReg(
            .E(LE), 
            .Reset(rst), 
            .clk(clk), 
            .PC_In(PC_In), 
            .PC_Out(PC_Out)
        );

        //Adder 
        adder add(
            .Adder_OUT(PC_In),
            .Adder_IN(PC_Out)
        );

        //Instruction Memory
        wire [31:0] insMem_Out;
        rom insMem(
            .I(insMem_Out),
            .A(PC_Out[7:0])
        );
        
        //Fetch Decode PPR
        wire [31:0] cu_in;
        if_id_reg IF_ID(
            .clk(clk),
            .load_enable(LE),
            .reset(rst),

            //INPUTS
            .instruction(insMem_Out),
            
            //OUTPUTS
            .cu_in(cu_in)
        );

        //Control Unit (Wires TO MUX)
        wire [1:0] am_cu_out;
        wire rf_en_cu_out;
        wire [3:0] alu_op_cu_out;
        wire Load_cu_out;
        wire branch_cu_out;
        wire branch_link_cu_out;
        wire s_bit_cu_out;
        wire rw_cu_out;
        wire size_cu_out;
        wire datamem_en_cu_out;
        control_unit CU(
            //INPUTS
            .instruction(cu_in),

            //OUTPUTS
            .AM(am_cu_out),
            .rf_en(rf_en_cu_out),
            .alu_op(alu_op_cu_out),
            .Load(Load_cu_out),
            .branch(branch_cu_out),
            .branch_link(branch_link_cu_out),
            .s_bit(s_bit_cu_out),
            .rw(rw_cu_out),
            .size(size_cu_out),
            .datamem_en(datamem_en_cu_out)
        );

        //Control Unit MUX 
        reg s;
        wire [1:0] am_out_cumux;
        wire rf_en_out_cumux;
        wire [3:0] alu_op_out_cumux;
        wire Load_out_cumux;
        wire branch_out_cumux;
        wire branch_link_out_cumux;
        wire s_bit_out_cumux;
        wire rw_out_cumux;
        wire size_out_cumux;
        wire datamem_en_out_cumux;
        cuMux Mux(
            .s(s),
            
            //INPUTS
            .am_in(am_cu_out),
            .rf_en_in(rf_en_cu_out),
            .alu_op_in(alu_op_cu_out),
            .Load_in(Load_cu_out),
            .branch_in(branch_cu_out),
            .branch_link_in(branch_link_cu_out),
            .s_bit_in(s_bit_cu_out),
            .rw_in(rw_cu_out),
            .size_in(size_cu_out),
            .datamem_en_in(datamem_en_cu_out),

            //OUTPUTS
            .am_out(am_out_cumux),
            .rf_en_out(rf_en_out_cumux),
            .alu_op_out(alu_op_out_cumux),
            .Load_out(Load_out_cumux),
            .branch_out(branch_out_cumux),
            .branch_link_out(branch_link_out_cumux),
            .s_bit_out(s_bit_out_cumux),
            .rw_out(rw_out_cumux),
            .size_out(size_out_cumux),
            .datamem_en_out(datamem_en_out_cumux)
        );

        //Decode Execute PPR
        wire [1:0] am_out_idexe;
        wire rf_en_out_idexe;
        wire [3:0] alu_op_out_idexe;
        wire s_bit_out_idexe;
        wire rw_out_idexe;
        wire size_out_idexe;
        wire datamem_en_out_idexe;
        wire Load_out_idexe;
        id_exe_reg ID_EXE(
            .clk(clk), 
            .reset(rst),

            //INPUTS
            .am(am_out_cumux),
            .alu_op(alu_op_out_cumux),
            .rf_en(rf_en_out_cumux),
            .s_bit(s_bit_out_cumux),
            .datamem_en(datamem_en_out_cumux),
            .readwrite(rw_out_cumux),
            .size(size_out_cumux),
            .load_instruction(Load_out_cumux),

            //OUTPUTS
            .am_out(am_out_idexe),
            .rf_en_out(rf_en_out_idexe),
            .alu_op_out(alu_op_out_idexe),
            .s_out(s_bit_out_idexe),
            .datamem_en_out(datamem_en_out_idexe),
            .readwrite_out(rw_out_idexe),
            .size_out(size_out_idexe),
            .load_instruction_out(Load_out_idexe)
        );

        //Execute Memory PPR
        wire rf_en_out_exemem;
        wire rw_out_exemem;
        wire size_out_exemem;
        wire datamem_en_out_exemem;
        wire Load_out_exemem;
        exe_mem_reg EXE_MEM(
            .clk(clk),
            .reset(rst),

            //INPUTS
            .rf_en(rf_en_out_idexe),
            .datamem_en(datamem_en_out_idexe),
            .readwrite(rw_out_idexe),
            .size(size_out_idexe),
            .load_instruction(Load_out_idexe),
            
            //OUTPUTS
            .rf_en_out(rf_en_out_exemem),
            .datamem_en_out(datamem_en_out_exemem),
            .readwrite_out(rw_out_exemem),
            .size_out(size_out_exemem),
            .load_instruction_out(Load_out_exemem)
        );

        //Memory Writeback PPR
        wire rf_en_out_memwb;
        mem_wb_reg MEM_WB(
            .clk(clk),
            .reset(rst),
            
            //INPUTS
            .rf_en(rf_en_out_exemem),
            
            //OUTPUTS
            .rf_en_out(rf_en_out_memwb)
        );

endmodule

//==================MODULES===================//

    //SPACE FOR PHASE 4 STUFF vvv
    module ConditionHandler(
        input B_in,
        input BL_in,
        input [3:0] I_Cond_in,

        input Z_in,
        input N_in,
        input C_in,
        input V_in,

        output reg TA_Ctrl_out,
        output reg BL_COND_out,
        output reg COND_EVAL_out
        );

        parameter EQUALS            = 4'b0000;
        parameter NOT_EQUALS        = 4'b0001;
        parameter CARRY_SET         = 4'b0010;
        parameter CARRY_CLEAR       = 4'b0011;
        parameter MINUS             = 4'b0100;
        parameter PLUS              = 4'b0101;
        parameter OVERFLOW          = 4'b0110;
        parameter NO_OVERFLOW       = 4'b0111;
        parameter UNSIGNED_HIGHER   = 4'b1000;
        parameter UNSIGNED_LOWER    = 4'b1001;
        parameter GREATER_EQUAL     = 4'b1010;
        parameter LESS_THAN         = 4'b1011;
        parameter GREATER_THAN      = 4'b1100;
        parameter LESS_EQUAL        = 4'b1101;
        parameter ALWAYS            = 4'b1110;
        parameter NEVER             = 4'b1111;

        always @* begin
            TA_Ctrl_out = 0;
            BL_COND_out = 0;
            COND_EVAL_out = 0;

            case (I_Cond_in)
                EQUALS:           COND_EVAL_out = Z_in;
                NOT_EQUALS:       COND_EVAL_out = ~Z_in;
                CARRY_SET:        COND_EVAL_out = C_in;
                CARRY_CLEAR:      COND_EVAL_out = ~C_in;
                MINUS:            COND_EVAL_out = N_in;
                PLUS:             COND_EVAL_out = ~N_in;
                OVERFLOW:         COND_EVAL_out = V_in;
                NO_OVERFLOW:      COND_EVAL_out = ~V_in;
                UNSIGNED_HIGHER:  COND_EVAL_out = C_in && ~Z_in;
                UNSIGNED_LOWER:   COND_EVAL_out = ~C_in || Z_in;
                GREATER_EQUAL:    COND_EVAL_out = (N_in == V_in);
                LESS_THAN:        COND_EVAL_out = (N_in != V_in);
                GREATER_THAN:     COND_EVAL_out = ~Z_in && (N_in == V_in);
                LESS_EQUAL:       COND_EVAL_out = Z_in || (N_in != V_in);
                ALWAYS:           COND_EVAL_out = 1;
                NEVER:            COND_EVAL_out = 0;
                default:          COND_EVAL_out = 0;
            endcase

            TA_Ctrl_out = (B_in || BL_in) && COND_EVAL_out;
            BL_COND_out = BL_in && COND_EVAL_out;
        end

    endmodule

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

    //SPACE FOR PHASE 4 STUFF ^^^


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

    module adder(
    output reg [31:0] Adder_OUT,
    input[31:0] Adder_IN
    );
    always@(Adder_IN) 
            begin
                Adder_OUT = Adder_IN + 32'b100;
            end
    endmodule

    module rom (
        output reg [31:0] I, 
        input [7:0] A 
        );

        reg [7:0] Mem [0:255];
        
        always @(A)
            I = { Mem[A], Mem[A+1], Mem[A+2], Mem[A+3] };

    endmodule

    module control_unit (
        input [31:0] instruction,  
        output reg rf_en,          
        output reg [3:0] alu_op,   //Final opcode sent to the ALU
        output reg Load,            // 1 for Load (LDR), 0 for Store (STR)
        output reg branch_link,    // 1 for Branch & Link (BL)
        output reg branch,         // 1 for Branch
        output reg s_bit,          // S bit for updating the PSR (Program Status Register)
        output reg rw,             // Read/Write signal: 1 for read (load), 0 for write (store)
        output reg size,           // Size: 0 for byte, 1 for word
        output reg datamem_en, // Data memory enable for load/store
        output reg [1:0] AM
        );
        //Instruction OPCODE (based on arm)
        parameter OP_ADD_INS  = 4'b0100 ;
        parameter OP_ADD_CIN_INS = 4'b0101 ;
        parameter OP_A_SUB_B_INS = 4'b0010 ;
        parameter OP_A_SUB_B_CIN_INS = 4'b0110 ;
        parameter OP_B_SUB_A_INS = 4'b0011 ;
        parameter OP_B_SUB_A_CIN_INS = 4'b0111 ;
        parameter OP_AND_INS = 4'b0000 ;
        parameter OP_OR_INS = 4'b1100 ;
        parameter OP_XOR_INS = 4'b0001 ; 
        parameter OP_A_TRANSFER_INS = 4'b1101 ; //These gotta have something to do with MOV/CMP instructions
        parameter OP_B_TRANSFER_INS = 4'b1101 ; //These gotta have something to do with MOV/CMP instructions
        parameter OP_NOT_B_INS = 4'b1111 ; //MVN
        parameter OP_A_AND_NOT_B_INS = 4'b1110 ; //BIC

        //ALU OP corresponding to phase1 requirements:
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
        wire [3:0] cu_opcode = instruction[24:21];   // ALU opcode field from instruction (24:21) (NOT Based on phase 1, based of ARM-specific)
        wire I = instruction[25];                 // Immediate bit (I) for data processing | Register/Immediate offset bit for load/store (R)
        wire S = instruction[20];
        wire bit4 = instruction[4];
        
        always @(*) begin
            // Default values
            rf_en = 0;
            alu_op = 4'b0000;
            Load = 0;
            branch = 0;
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
                branch = 0;
                branch_link = 0;
                s_bit = 0;
                rw = 0;
                size = 0;
                datamem_en = 0;

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
                    
                        case (cu_opcode)
                            OP_ADD_INS:             alu_op = OP_ADD;           
                            OP_ADD_CIN_INS:         alu_op = OP_ADD_CIN;       
                            OP_A_SUB_B_INS:         alu_op = OP_A_SUB_B;       
                            OP_A_SUB_B_CIN_INS:     alu_op = OP_A_SUB_B_CIN;   
                            OP_B_SUB_A_INS:         alu_op = OP_B_SUB_A;       
                            OP_B_SUB_A_CIN_INS:    alu_op = OP_B_SUB_A_CIN;   
                            OP_AND_INS:             alu_op = OP_AND;           
                            OP_OR_INS:              alu_op = OP_OR;            
                            OP_XOR_INS:             alu_op = OP_XOR;           
                            OP_A_TRANSFER_INS:      alu_op = OP_A_TRANSFER;    //Honestly this seems to be irrelevant
                            OP_B_TRANSFER_INS:      alu_op = OP_B_TRANSFER;    
                            OP_NOT_B_INS:           alu_op = OP_NOT_B;           
                            OP_A_AND_NOT_B_INS:     alu_op = OP_A_AND_NOT_B;     
                            default:                alu_op = 4'b1111;            
                        endcase
                    end

                    3'b01z: begin
                                            
                        Load = instruction[20]; // 1 for Load (LDR), 0 for Store (STR)
                        rw = !instruction[20]; // 1 for write, 0 for read (according to phase1)
                        size = !instruction[22]; // 0 for byte, 1 for word (according to phase1)
                        
                        datamem_en = !instruction[20];
                        rf_en = instruction[20]; // 1 When load, 0 when store


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
                        //Need two signals coming out of the branch to know if its branch or branch and link. (B & BL) from block diagram
                        //For the condition handler.
                        if (instruction[24] == 1) begin
                            branch_link = 1;
                            branch=0;
                        end else if (instruction[24] == 0) begin
                            branch_link = 0;
                            branch=1;
                        end else begin
                            branch_link = 0;
                            branch=0;
                        end
                    end

                    default: begin
                        // No operation
                    end
                endcase
            end
        end
    endmodule

    module cuMux (
        input s,

        input [1:0] am_in,
        input rf_en_in,          
        input [3:0] alu_op_in,   
        input Load_in,            
        input branch_in,
        input branch_link_in,    
        input s_bit_in,          
        input rw_in,             
        input size_in,           
        input datamem_en_in,

        output reg [1:0] am_out,         
        output reg rf_en_out,          
        output reg [3:0] alu_op_out,   
        output reg Load_out,            
        output reg branch_out,   
        output reg branch_link_out,   
        output reg s_bit_out,          
        output reg rw_out,             
        output reg size_out,           
        output reg datamem_en_out
        );  

        always @* begin
            if(s == 1'b0) begin // Pass Control Unit values when selector is 0
                am_out = am_in;
                rf_en_out = rf_en_in;
                alu_op_out = alu_op_in;
                Load_out = Load_in;
                branch_out = branch_in;
                branch_link_out = branch_link_in;
                s_bit_out = s_bit_in;
                rw_out = rw_in;
                size_out = size_in;
                datamem_en_out = datamem_en_in;
            end 
            else begin
                am_out = 2'b0;
                rf_en_out = 1'b0;
                alu_op_out = 1'b0;
                Load_out = 1'b0;
                branch_out = 1'b0;
                branch_link_out = 1'b0;
                s_bit_out = 1'b0;
                rw_out = 1'b0;
                size_out = 1'b0;
                datamem_en_out = 1'b0;
            end
        end
    endmodule

    module if_id_reg(
        input clk, load_enable,reset,
        input [31:0] instruction, 
        output reg [31:0] cu_in);

        always @ (posedge clk) begin
            if(load_enable) begin
                if(reset) begin
                    // instruction = 0; (I don't know if this was missing)
                    cu_in <= 0;

                end else begin
                    cu_in <= instruction;
                end
            end
        end
    endmodule

    module id_exe_reg(
        input clk, reset,
        input [1:0] am,
        input [3:0] alu_op,
        input rf_en,s_bit,datamem_en,readwrite,size,load_instruction,

        output reg [1:0] am_out,
        output reg [3:0] alu_op_out,
        output reg rf_en_out,s_out,datamem_en_out,readwrite_out,size_out,load_instruction_out
        );

        always @ (posedge clk) begin 
            if(reset) begin
                am_out <= 0;
                alu_op_out <= 0;
                rf_en_out <= 0;
                s_out <= 0;
                datamem_en_out <= 0;
                readwrite_out <= 0;
                size_out <= 0;
                load_instruction_out <= 0;
            end else begin
                am_out <= am;
                alu_op_out <= alu_op;
                rf_en_out <= rf_en;
                s_out <= s_bit;
                datamem_en_out <= datamem_en;
                readwrite_out <= readwrite;
                size_out <= size;
                load_instruction_out <= load_instruction;
            end
        end
    endmodule

    module exe_mem_reg(
        input clk, reset,
        input rf_en,datamem_en,readwrite,size,load_instruction,

        output reg rf_en_out,datamem_en_out,readwrite_out,size_out,load_instruction_out
        );

        always @ (posedge clk) begin
            if(reset) begin
                rf_en_out <= 0;
                datamem_en_out <= 0;
                readwrite_out <= 0;
                size_out <= 0;
                load_instruction_out <= 0;
            end else begin
                rf_en_out <= rf_en;
                datamem_en_out <= datamem_en;
                readwrite_out <= readwrite;
                size_out <= size;
                load_instruction_out <= load_instruction;
            end
        end
    endmodule

    module mem_wb_reg(
        input clk, reset,
        input rf_en,

        output reg rf_en_out
        );

        always @ (posedge clk)
            begin
            if(reset) begin
                rf_en_out <= 0;
            end else begin
                rf_en_out <= rf_en;
            end
        end
    endmodule

