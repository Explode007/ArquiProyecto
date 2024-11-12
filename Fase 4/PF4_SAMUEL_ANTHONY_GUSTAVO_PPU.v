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
        //================================================================
        //RF Mux SaveNEXTPC
            wire [31:0] RF_MUX_SAVENEXTPC_OUT;
            In2Out1MUX32(
                .In1(rd_ifid),
                .In2(4'b1110),
                .selector(bl_condition_out),
                .out(RF_MUX_SAVENEXTPC_OUT)
            );
        //================================================================
        //Operand MUXES
            wire [1:0] ASelector;
            wire [1:0] BSelector;
            wire [1:0] DSelector;
            wire [31:0] OperandA_MUX_OUT;
            wire [31:0] OperandB_MUX_OUT;
            wire [31:0] OperandD_MUX_OUT;

            //Pile these wires with Register file
            wire [31:0] Operand_A_OUT_RF;
            wire [31:0] Operand_B_OUT_RF;
            wire [31:0] Operand_D_OUT_RF;

            In4Out1MUX32 OperandAMUX(
                .In1(Operand_A_OUT_RF),
                .In2(WBTORF_MUX_out),
                .In3(dataWB_memwb),
                .In4(NextPCORAALU_MUX_OUT),

                .selector(ASelector),
                .out(OperandA_MUX_OUT),
            );
            In4Out1MUX32 OperandBMUX(
                .In1(Operand_B_OUT_RF),
                .In2(WBTORF_MUX_out),
                .In3(dataWB_memwb),
                .In4(NextPCORAALU_MUX_OUT),

                .selector(BSelector),
                .out(OperandB_MUX_OUT),
            );
            In4Out1MUX32 OperandDMUX(
                .In1(Operand_D_OUT_RF),
                .In2(WBTORF_MUX_out),
                .In3(dataWB_memwb),
                .In4(NextPCORAALU_MUX_OUT),

                .selector(DSelector),
                .out(OperandD_MUX_OUT),
            );
        //================================================================
        //WBTORF MUX
            wire [31:0] WBTORF_MUX_out;
            In2Out1MUX32 WBTORFMUX(
                .In1(AluORNextPC_out_exemem),
                .In2(mem_mux_out),
                .selector(Load_out_exemem),
                .out(WBTORF_MUX_out)
            )
        //================================================================
        //NextPCORALU MUX
            wire [31:0] NextPCORAALU_MUX_OUT;
            wire [31:0] NextPCORALUMUX_In1;
            wire NextPCORALU_CTRL_OUT;
            In2Out1MUX32 NextPCORALU(
                .In1(NextPCORALUMUX_In1),
                .In2(Alu_out),
                .selector(NextPCORALU_CTRL_OUT),
                .out(NextPCORAALU_MUX_OUT)
            );
        //================================================================
        //ALU
            wire [31:0] Alu_out;
            //TODO: Flags remake
            // wire Z_alu, N_alu, C_alu, V_alu;

            ALU alu (
                .Op(alu_op_out_idexe),
                .A(OperandA_out_idexe),
                .B(Shifter_out),
                .CIN(),

                .Out(Alu_out),
                //TODO: Flags remake            
            );
        //================================================================
        //Shifter SignExtender
            wire [31:0] Shifter_out;
            Shifter_SignExtender shifter(
                .Rm(OperandB_out_idexe),
                .I(immediate_out_idexe),
                .AM(am_out_idexe),

                .N(Shifter_out)
            );
        //================================================================
        //Condition Handler //TODO: Flags rework
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

        //================================================================
        //PSR MUX //TODO: Flags rework
            wire Z_out, N_out, C_out, V_out;
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

        //================================================================
        //Program Status Register //TODO: Flags rework
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
            );
        //================================================================
        //Program Counter
            wire [31:0] PC_Out; 
            PC PCReg(
                .E(LE), 
                .Reset(rst), 
                .clk(clk), 
                .PC_In(BranchMux_out), 
                .PC_Out(PC_Out)
            );
        //================================================================
        //BranchMUX
            wire [31:0] BranchMux_out; 
            In2Out1MUX32 BMux(
                .In1(PC_adder_out),
                .In2(BranchRel_out),
                .selector(TA_Ctrl_out),

                .out(BranchMux_out)
            );

        //================================================================
        //Branch Rel PC Adder
            wire [31:0] BranchRel_out;
            adder RelAdder(
                .Adder_IN1(PC_adder_out),
                .Adder_IN2(), //TODO: Need to input the rotated thing here

                .Adder_OUT(BranchRel_out)
            );
        //================================================================
        //PC Adder 
            wire [31:0] PC_adder_out
            adder PCAdder(
                .Adder_IN1(PC_Out),
                .Adder_IN2(32'b100),

                .Adder_OUT(PC_adder_out)
            );
        //================================================================
        //Instruction Memory
            wire [31:0] insMem_Out; //This wire carries the current instruction about to come to pipeline
            rom insMem(
                .I(insMem_Out),
                .A(PC_Out[7:0])
            );
        

        //================================================================
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

        //================================================================
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

        //================================================================
        //Fetch Decode PPR
            wire [3:0] instr_cond_ifid;
            wire [3:0] ra_ifid, rd_ifid, rb_ifid;
            wire [12:0] immediate_ifid;
            wire [23:0] branch_offset_ifid;
            wire [31:0] next_pc_out_ifid;
            wire [31:0] cu_in;
            if_id_reg IF_ID(
                .clk(clk),
                .load_enable(LE),
                .reset(rst),

                //INPUTS
                .instruction(insMem_Out),
                .next_pc_in(PC_adder_out),
                
                //OUTPUTS
                .instr_cond(instr_cond),
                .branch_offset(branch_offset_ifid),
                .ra(ra_ifid),
                .rb(rb_ifid),
                .rd(rd_ifid),
                .immediate(immediate_ifid),
                .next_pc_out(next_pc_out_ifid),
                .cu_in(cu_in),
            );

        //================================================================
        //Decode Execute PPR
            wire rf_en_out_idexe;
            wire s_bit_out_idexe;
            wire datamem_en_out_idexe;
            wire rw_out_idexe;
            wire size_out_idexe;
            wire Load_out_idexe;
            wire [1:0] am_out_idexe;
            wire [3:0] alu_op_out_idexe;
            
            wire [31:0] OperandA_out_idexe;
            wire [31:0] OperandB_out_idexe;
            wire [31:0] OperandD_out_idexe;
            wire [12:0] immediate_out_idexe;
            wire [31:0] rd_out_idexe;
            wire [3:0] instr_cond_idexe;

            
            id_exe_reg ID_EXE(
                .clk(clk), 
                .reset(rst),

                //INPUTS
                .rf_en_in(rf_en_out_cumux),
                .s_bit_in(s_bit_out_cumux),
                .datamem_en_in(datamem_en_out_cumux),
                .readwrite_in(rw_out_cumux),
                .size_in(size_out_cumux),
                .load_instruction_in(Load_out_cumux),
                .am_in(am_out_cumux),
                .alu_op_in(alu_op_out_cumux),
                .instr_cond_in(instr_cond_ifid),
                .next_pc_in(next_pc_out_ifid)
                .operand_a_in(OperandA_MUX_out),
                .operand_b_in(OperandB_MUX_out),
                .operand_d_in(OperandD_MUX_out),
                .immediate_in(immediate_ifid),
                .rd_in(RF_MUX_SAVENEXTPC_OUT),
                .NEXTORALU_ctrl_in(bl_condition_out),

                //OUTPUTS
                .rf_en_out(rf_en_out_idexe),
                .s_bit_out(s_bit_out_idexe),
                .datamem_en_out(datamem_en_out_idexe),
                .readwrite_out(rw_out_idexe),
                .size_out(size_out_idexe),
                .load_instruction_out(Load_out_idexe)
                .am_out(am_out_idexe),
                .alu_op_out(alu_op_out_idexe),
                .instr_cond_out(instr_cond_idexe),
                .next_pc_out(NextPCORALUMUX_In1),
                .operand_a_out(OperandA_out_idexe),
                .operand_b_out(OperandB_out_idexe),
                .operand_d_out(OperandD_out_idexe),
                .immediate_out(immediate_out_idexe;),
                .rd_out(rd_out_idexe),
                .NEXTORALU_ctrl_out(NextPCORALU_CTRL_OUT),
            );

        //================================================================
        //Execute Memory PPR
            wire rf_en_out_exemem;
            wire rw_out_exemem;
            wire size_out_exemem;
            wire datamem_en_out_exemem;
            wire Load_out_exemem;
            wire [31:0] AluORNextPC_out_exemem;
            wire [31:0] OperandD_out_exemem;
            wire [3:0] rd_out_exemem;
            exe_mem_reg EXE_MEM(
                .clk(clk),
                .reset(rst),

                //INPUTS
                .rf_en_in(rf_en_out_idexe),
                .datamem_en_in(datamem_en_out_idexe),
                .readwrite_in(rw_out_idexe),
                .size_in(size_out_idexe),
                .load_instruction_in(Load_out_idexe),
                .AluORNextPC_in(NextPCORAALU_MUX_OUT), 
                .OperandD_in(OperandD_out_idexe),
                .rd_in(rd_out_idexe)
                
                //OUTPUTS
                .rf_en_out(rf_en_out_exemem),
                .datamem_en_out(datamem_en_out_exemem),
                .readwrite_out(rw_out_exemem),
                .size_out(size_out_exemem),
                .load_instruction_out(Load_out_exemem),
                .AluORNextPC_out(AluORNextPC_out_exemem),
                .OperandD_out(OperandD_out_exemem),
                .rd_out(rd_out_exemem)
            );
        
        //================================================================
        //Memory Writeback PPR
            wire rf_en_out_memwb;
            wire [3:0] rd_out_memwb;
            wire [31:0] dataWB_memwb;
            mem_wb_reg MEM_WB(
                .clk(clk),
                .reset(rst),
                
                //INPUTS
                .rf_en_in(rf_en_out_exemem),
                .rd_in(rd_out_exemem),
                .mem_mux_in(WBTORF_MUX_out),
                
                //OUTPUTS
                .rf_en_out(rf_en_out_memwb),
                .rd_out(rd_out_memwb),
                .mem_mux_out(dataWB_memwb), //TODO: Goes to MUX WB TO RF
            );
        //================================================================
        //Hazard & Forwarding Unit
            wire
endmodule

//==================MODULES===================//
    module HazardForwardingUnit(
        input [31:0] rb_in,
        input [31:0] ra_in,
        input COND_EVAL_in,
        input load_instruc_in,

        input [31:0] rd_in_id,
        input [31:0] rd_in_exe,
        input [31:0] rd_in_mem,
        input [31:0] rd_in_wb,
        
        input rf_en_exe,
        input rf_en_mem,
        input rf_en_wb,

        output CU_MUX_CTRL,
        output IFID_LE_CTRL,
        output PC_LE_CTRL,
        output OperandA_MUX_CTRL,
        output OperandB_MUX_CTRL,
        output OperandD_MUX_CTRL,
        );
    
    endmodule

    //TODO: Rework flag logic
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
    
    //TODO: Rework flag logic
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
    
    //TODO: Rework flag logic
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

    module In2Out1MUX32(
        input [31:0] In1, In2;
        input selector;

        output [31:0] out;
        );  
        always @(*) begin
            if (selector) begin
                out = In1;
            end else begin
                out = In2;
            end
        end
    endmodule
    
    module In4Out1MUX32(
        input [31:0] In1, In2, In3, In4;
        input [1:0] selector;

        output [31:0] out;
        );  
        always @(*) begin
            case (selector)
            2'b00: out = In1;
            2'b01: out = In2;
            2'b10: out = In3;
            2'b11: out = In4;           
            endcase
        
        end
    endmodule

    //TODO: Not sure how to make this piece
    module RotExtRELPC(
        input [24:0] reladdin;
        output [31:0] reladdout;
        );
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
                    case (I[6:5])
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

    //TODO: Rework flag logic
    module ALU (
        input [3:0] Op,     
        input [31:0] A, B,
        input CIN,            
        
        output reg [31:0] Out, 
        //TODO: Remake flags to be [32:0] output where the first 4 bits are these flags or whatever.
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
        input[31:0] Adder_IN1, Adder_IN2
        );
        always@(Adder_IN1 or Adder_IN2) 
            begin
                Adder_OUT = Adder_IN1 + Adder_IN2;
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
        input [31:0] next_pc_in,

        output reg [3:0] instr_cond,
        output reg [3:0] ra,rd,rb,
        output reg [11:0] immediate,
        output reg [23:0] branch_offset,
        output reg [31:0] next_pc_out,
        output reg [31:0] cu_in
        );

        always @ (posedge clk) begin
            if(load_enable) begin
                if(reset) begin
                    instr_cond <= 0;
                    branch_offset <= 0;
                    ra <= 0;
                    rd <= 0;
                    rb <= 0;
                    immediate <= 0;
                    next_pc_out <= 0;
                    cu_in <= 0;

                end else begin
                    instr_cond <= instruction[31:28];
                    branch_offset <= instruction[23:0];
                    ra <= instruction[19:16];
                    rd <= instruction[15:12];
                    rb <= instruction[3:0];
                    immediate <= instruction[11:0];
                    next_pc_out <= next_pc;
                    cu_in <= instruction;
                end
            end
        end
    endmodule

    module id_exe_reg(
        input clk, reset,

        input rf_en_in, s_bit_in, datamem_en_in, readwrite_in, size_in, load_instruction_in, 
        input [1:0] am_in,
        input [3:0] alu_op_in,
        input [3:0] instr_cond_in,
        input [31:0] next_pc_in, operand_a_in, operand_b_in, operand_d_in,
        input [11:0] immediate_in,
        input [3:0] rd_in,
        input NEXTORALU_ctrl_in,

        output reg rf_en_out, s_bit_out, datamem_en_out, readwrite_out, size_out, load_instruction_out
        output reg [1:0] am_out,
        output reg [3:0] alu_op_out,
        output reg [3:0] instr_cond_out,
        output reg [31:0] next_pc_out, operand_a_out, operand_b_out, operand_d_out,
        output reg [11:0] immediate_out,
        output reg [3:0] rd_out,
        input NEXTORALU_ctrl_out,
        );

        always @ (posedge clk) begin 
            if(reset) begin
                rf_en_out <= 0;
                s_bit_out <= 0;
                datamem_en_out <= 0;
                readwrite_out <= 0;
                size_out <= 0;
                load_instruction_out <= 0;
                am_out <= 0;
                alu_op_out <= 0;
                instr_cond_out <= 0;
                next_pc_out <= 0;
                operand_a_out <= 0;
                operand_b_out <= 0;
                operand_d_out <= 0;
                immediate_out <= 0;
                rd_out <= 0;
                NEXTORALU_ctrl_out <= 0;
                
            end else begin
                rf_en_out <= rf_en_in;
                s_bit_out <= s_bit_in;
                datamem_en_out <= datamem_en_in;
                readwrite_out <= readwrite_in;
                size_out <= size_in;
                load_instruction_out <= load_instruction_in;
                am_out <= am_in;
                alu_op_out <= alu_op_in;
                instr_cond_out <= instr_cond_in;
                next_pc_out <= next_pc_in;
                operand_a_out <= operand_a_in;
                operand_b_out <= operand_b_in;
                operand_d_out <= operand_d_in;
                immediate_out <= immediate_in;
                rd_out <= rd_in;
                NEXTORALU_ctrl_out <= NEXTORALU_ctrl_in;
            end
        end
    endmodule

    module exe_mem_reg(
        input clk, reset,

        input rf_en_in ,datamem_en_in ,readwrite_in ,size_in ,load_instruction_in ,
        input [31:0] AluORNextPC_in,
        input [31:0] OperandD_in,
        input [3:0] rd_in,

        output reg [31:0] AluORNextPC_out,
        output reg [31:0] OperandD_out,
        output reg [3:0] rd_out,
        output reg rf_en_out,datamem_en_out,readwrite_out,size_out,load_instruction_out
        );

        always @ (posedge clk) begin
            if(reset) begin
                rf_en_out <= 0;
                datamem_en_out <= 0;
                readwrite_out <= 0;
                size_out <= 0;
                load_instruction_out <= 0;
                AluORNextPC_out <= 0;
                OperandD_out <= 0;
                rd_out <= 0;
            end else begin
                rf_en_out <= rf_en_in;
                datamem_en_out <= datamem_en_in;
                readwrite_out <= readwrite_in;
                size_out <= size_in;
                load_instruction_out <= load_instruction_in;
                AluORNextPC_out <= AluORNextPC_in;
                OperandD_out <= OperandD_in;
                rd_out <= rd_in;
            end
        end
    endmodule

    module mem_wb_reg(
        input clk, reset,

        input rf_en_in,
        input [3:0] rd_in,
        input [31:0] mem_mux_in,
        
        output reg rf_en_out
        output reg [31:0] mem_mux_out,
        output reg [3:0] rd_out,
        );

        always @ (posedge clk)
            begin
            if(reset) begin
                rf_en_out <= 0;
                mem_mux_out <= 0;
                rd_out <= 0;
            end else begin
                rf_en_out <= rf_en_in;
                mem_mux_out <= mem_mux_in;
                rd_out <= rd_in;
            end
        end
    endmodule

