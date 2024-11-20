module PPU();
    //======Simulation======//

        //======Init Stuff======//
            //Generate clock signal, toggles every 2 time units
            always begin
                clk <= 0;
                #2 clk <= ~clk;
            end

            always begin
                //monitor clock
                monclk <= 0;
                #1 monclk <= ~monclk;
            end
            //Preload
            integer fi, code;
            reg [7:0] data;
            reg [8:0] Address;
            initial begin

//================================= ⌄ Precharge Section ⌄ =================================//

                // Open and read the test code file
                fi = $fopen("precharge.txt", "r");
                Address = 0;
                while (!$feof(fi)) begin
                    code = $fscanf(fi, "%b", data);  // Read binary instructions
                    insMem.Mem[Address] = data;      // Load instructions into ROM
                    dataMem.Mem[Address] = data;     // Load instructions into RAM
                    Address = Address + 1;
                end
                $fclose(fi);  // Close the file

//=========================================================================================//

                // Initialize the signals
                LE <= 1'b1;
                rst <= 1'b1;
                
                
                // Do required timed changes
                #3 rst <= 1'b0;
                
            end

            initial begin
                #50 $finish; //41 so it can print out 40
            end

            reg [167:0] instruction_name;

            always @ (*) begin

                case (cu_in)
                    32'b11100011101000000101000000110100: instruction_name = "MOV R5, #52";
                    32'b11100101100101010001000000000000: instruction_name = "LDR R1, [R5, #0]";
                    32'b11100101110101010010000000000100: instruction_name = "LDRB R2, [R5, #4]";
                    32'b11100101110101010011000000000101: instruction_name = "LDRB R3, [R5, #5] ";
                    32'b11100001101100001010000000000001: instruction_name = "MOVS R10,R1 ";
                    32'b01011010000000000000000000000100: instruction_name = "BPL +4 ";
                    32'b11100000010000110110000000000010: instruction_name = "SUB R6,R3,R2";
                    32'b00000000000000000000000000000000: instruction_name = "NOP";
                    32'b11101010000000000000000000000100: instruction_name = "B +1";
                    32'b11100000100000110110000000000010: instruction_name = "ADD R6,R3,R2";
                    32'b11100101110001010110000000000110: instruction_name = "STRB R6, [R5, #6]";
                    32'b11101010111111111111111111111111: instruction_name = "B -1";
                    default: instruction_name = "NULL";
                endcase

            end


        //======Monitor=======//
        always @(posedge monclk) begin
            $display("Time = %t | PC = %d | Instruction: %s (%b)", $time, PC_Out, instruction_name, cu_in);
            $display("ID Stage: AM = %b | S-Bit = %b | DATAMEM_EN = %b | R/W = %b | Size = %b | RF_EN = %b | ALU_OP = %b | Load = %b | Branch&Link = %b | Branch = %b",
                am_cu_out, s_bit_cu_out, datamem_en_cu_out, rw_cu_out, size_cu_out, rf_en_cu_out, alu_op_cu_out, Load_cu_out, branch_link_cu_out, branch_cu_out);
            $display("EX Stage: AM = %b | S-Bit = %b | DATAMEM_EN = %b | R/W = %b | Size = %b | RF_EN = %b | ALU_OP = %b | Load = %b",
                am_out_idexe, s_bit_out_idexe, datamem_en_out_idexe, rw_out_idexe, size_out_idexe, rf_en_out_idexe, alu_op_out_idexe, Load_out_idexe);
            $display("MEM Stage: RF_EN = %b | DATAMEM_EN = %b | R/W = %b | Size = %b | Load = %b",
                rf_en_out_exemem, datamem_en_out_exemem, rw_out_exemem, size_out_exemem, Load_out_exemem);
            $display("WB Stage: RF_EN = %b", rf_en_out_memwb);
        end








    //==================INSTANTIATION===================//
        //Generic Wires
        reg LE;
        reg rst;
        reg clk;
        reg monclk;
        //================================================================
        //RF Mux SaveNEXTPC
            wire [31:0] RF_MUX_SAVENEXTPC_OUT;
            In2Out1MUX32 muxsavenextpc(
                .In1({28'b0, rd_ifid}),
                .In2({28'b0, 4'b1110}),
                .selector(bl_condition_out),
                .out(RF_MUX_SAVENEXTPC_OUT)
            );
        //================================================================
        //Operand MUXES
            wire [31:0] OperandA_MUX_OUT;
            wire [31:0] OperandB_MUX_OUT;
            wire [31:0] OperandD_MUX_OUT;
            In4Out1MUX32 OperandAMUX(
                .In1(Operand_A_OUT_RF),
                .In2(WBTORF_MUX_out),
                .In3(dataWB_memwb),
                .In4(NextPCORAALU_MUX_OUT),

                .selector(ASelector),
                .out(OperandA_MUX_OUT)
            );
            In4Out1MUX32 OperandBMUX(
                .In1(Operand_B_OUT_RF),
                .In2(WBTORF_MUX_out),
                .In3(dataWB_memwb),
                .In4(NextPCORAALU_MUX_OUT),

                .selector(BSelector),
                .out(OperandB_MUX_OUT)
            );
            In4Out1MUX32 OperandDMUX(
                .In1(Operand_D_OUT_RF),
                .In2(WBTORF_MUX_out),
                .In3(dataWB_memwb),
                .In4(NextPCORAALU_MUX_OUT),

                .selector(DSelector),
                .out(OperandD_MUX_OUT)
            );
        //================================================================
        //WBTORF MUX
            wire [31:0] WBTORF_MUX_out;
            In2Out1MUX32 WBTORFMUX(
                .In1(AluORNextPC_out_exemem),
                .In2(dataMem_Out), //Originally had mem_mux_out, changed it to dataMem_Out (as the PPU diagram says)
                .selector(Load_out_exemem),
                .out(WBTORF_MUX_out)
            );
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
            wire [31:0] Alu_flags_out;

            ALU alu (
                .Op(alu_op_out_idexe),
                .A(OperandA_out_idexe),
                .B(Shifter_out),
                .CIN(Flags_out_PSR[2]), //[2] is the C flag

                .Out(Alu_out),
                .Flags(Alu_flags_out)     
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
        //Condition Handler 
            wire TA_Ctrl_out;
            wire bl_condition_out;
            wire COND_EVAL_out;
            ConditionHandler condhandler (
                .B_in(B_signal),
                .BL_in(BL_signal),
                .I_Cond_in(instr_cond_idexe),
                .Flags_in(PSR_MUX_OUT),

                .TA_Ctrl_out(TA_Ctrl_out),
                .BL_COND_out(bl_condition_out),
                .COND_EVAL_out(COND_EVAL_out)
            );
        //================================================================
        //PSR MUX
            wire [31:0] PSR_MUX_OUT;
            In2Out1MUX32 psrmux(
                .In1(Alu_flags_out),
                .In2(Flags_out_PSR),
                .selector(s_bit_out_idexe),
                .out(PSR_MUX_OUT)
            );
        //================================================================
        //Program Status Register
            wire [31:0] Flags_out_PSR;
            ProgramStatusRegister PSR(
                .S_bit_in(s_bit_out_idexe),

                .Flags_in(Alu_flags_out),
                .Flags_out(Flags_out_PSR)
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
            wire [31:0] PC_adder_out;
            adder PCAdder(
                .Adder_IN1(PC_Out),
                .Adder_IN2(32'b00000000000000000000000000000100),

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
        //Register File
            wire [31:0] Operand_A_OUT_RF;
            wire [31:0] Operand_B_OUT_RF;
            wire [31:0] Operand_D_OUT_RF;
            register_file rf1(
                .LE(rf_en_out),
                .Clk(clk),
                .PC(next_pc_out_ifid),
                .PW(dataWB_memwb),
                .RD(rd_ifid),
                .RB(rb_ifid),
                .RA(ra_ifid),
                .RW(rd_out_memwb),
                .PD(Operand_D_OUT_RF),
                .PB(Operand_B_OUT_RF),
                .PA(Operand_A_OUT_RF)
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
                .s(CU_MUX_CTRL_OUT),
                
                //INPUTS
                .am_in(am_cu_out),
                .rf_en_in(inbetweenCUCUMUX_MUX_OUT[0]),
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
            wire [11:0] immediate_ifid;
            wire [23:0] branch_offset_ifid;
            wire [31:0] next_pc_out_ifid;
            wire [31:0] cu_in;
            if_id_reg IF_ID(
                .clk(clk),
                .load_enable(IFID_LE_CTRL_OUT),
                .reset(rst),

                //INPUTS
                .instruction(insMem_Out),
                .next_pc_in(PC_adder_out),
                
                //OUTPUTS
                .instr_cond(instr_cond_ifid),
                .branch_offset(branch_offset_ifid),
                .ra(ra_ifid),
                .rb(rb_ifid),
                .rd(rd_ifid),
                .immediate(immediate_ifid),
                .next_pc_out(next_pc_out_ifid),
                .cu_in(cu_in)
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
            wire [11:0] immediate_out_idexe;
            wire [3:0] rd_out_idexe;
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
                .next_pc_in(next_pc_out_ifid),
                .operand_a_in(OperandA_MUX_OUT),
                .operand_b_in(OperandB_MUX_OUT),
                .operand_d_in(OperandD_MUX_OUT),
                .immediate_in(immediate_ifid),
                .rd_in(RF_MUX_SAVENEXTPC_OUT[3:0]),
                .NEXTORALU_ctrl_in(bl_condition_out),

                //OUTPUTS
                .rf_en_out(rf_en_out_idexe),
                .s_bit_out(s_bit_out_idexe),
                .datamem_en_out(datamem_en_out_idexe),
                .readwrite_out(rw_out_idexe),
                .size_out(size_out_idexe),
                .load_instruction_out(Load_out_idexe),
                .am_out(am_out_idexe),
                .alu_op_out(alu_op_out_idexe),
                .instr_cond_out(instr_cond_idexe),
                .next_pc_out(NextPCORALUMUX_In1),
                .operand_a_out(OperandA_out_idexe),
                .operand_b_out(OperandB_out_idexe),
                .operand_d_out(OperandD_out_idexe),
                .immediate_out(immediate_out_idexe),
                .rd_out(rd_out_idexe),
                .NEXTORALU_ctrl_out(NextPCORALU_CTRL_OUT)
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
                .rd_in(rd_out_idexe),
                
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
                .mem_mux_out(dataWB_memwb) //TODO: Goes to MUX WB TO RF
            );
        //================================================================
        //Hazard & Forwarding Unit
            wire CU_MUX_CTRL_OUT;
            wire IFID_LE_CTRL_OUT;
            wire PC_LE_CTRL_OUT;
            wire [1:0] ASelector;
            wire [1:0] BSelector;
            wire [1:0] DSelector;
            HazardForwardingUnit HFU(
                //INPUTS
                .ra_in({28'b0, ra_ifid}),               
                .rb_in({28'b0, rb_ifid}),               
                .COND_EVAL_in(cond),       
                .load_instruc_in(COND_EVAL_out),    

                .rd_in_id({28'b0, rd_ifid}),            
                .rd_in_exe({28'b0, rd_out_idexe}),          
                .rd_in_mem({28'b0, rd_out_exemem}),          
                .rd_in_wb({28'b0, rd_out_memwb}),            

                .rf_en_exe(rf_en_out_idexe),           
                .rf_en_mem(rf_en_out_exemem),           
                .rf_en_wb(rf_en_out_memwb),            

                //OUTPUTS
                .CU_MUX_CTRL(CU_MUX_CTRL_OUT),         
                .IFID_LE_CTRL(IFID_LE_CTRL_OUT),        
                .PC_LE_CTRL(PC_LE_CTRL_OUT),          

                .OperandA_MUX_CTRL(ASelector),   
                .OperandB_MUX_CTRL(BSelector),   
                .OperandD_MUX_CTRL(DSelector)    
            );
        //================================================================
        //MUX for RF between CU and CUMUX
            wire [31:0] inbetweenCUCUMUX_MUX_OUT;
            In2Out1MUX32 inbetweencucumux(
                .In1({31'b0, rf_en_cu_out}),
                .In2({31'b0, 1'b1}),
                .selector(bl_condition_out),

                .out(inbetweenCUCUMUX_MUX_OUT)
            );
        //================================================================
        //Data Memory
            wire [31:0] dataMem_Out; //This wire goes into the Mux in the MEM Stage
            ram dataMem(
                .DataOut(dataMem_Out),
                .E(datamem_en_out_exemem),
                .RW(rw_out_exemem),
                .A(AluORNextPC_out_exemem[7:0]),
                .DataIn(OperandD_out_exemem),
                .Size(size_out_exemem)
            );
        //================================================================
endmodule
//==================MODULES===================//
    module HazardForwardingUnit(
        input [31:0] ra_in,
        input [31:0] rb_in,
        input COND_EVAL_in,
        input load_instruc_in,

        input [31:0] rd_in_id,
        input [31:0] rd_in_exe,
        input [31:0] rd_in_mem,
        input [31:0] rd_in_wb,
        
        input rf_en_exe,
        input rf_en_mem,
        input rf_en_wb,

        output reg CU_MUX_CTRL,
        output reg IFID_LE_CTRL,
        output reg PC_LE_CTRL,
        output reg [1:0] OperandA_MUX_CTRL,
        output reg [1:0] OperandB_MUX_CTRL,
        output reg [1:0] OperandD_MUX_CTRL
        );

        always @(*) begin
            // Default values
            CU_MUX_CTRL = 1'b0;
            IFID_LE_CTRL = 1'b1;
            PC_LE_CTRL = 1'b1;
            OperandA_MUX_CTRL = 2'b00; 
            OperandB_MUX_CTRL = 2'b00; 
            OperandD_MUX_CTRL = 2'b00; 

            // Stall if there’s a data hazard with a load instruction
            if (load_instruc_in && ((rd_in_exe == ra_in) || (rd_in_exe == rb_in))) begin
                CU_MUX_CTRL = 1'b1;     
                IFID_LE_CTRL = 1'b0;
                PC_LE_CTRL = 1'b0;      
            end

            // Forwarding logic for OperandA
            if (rf_en_exe && (rd_in_exe == ra_in)) begin
                OperandA_MUX_CTRL = 2'b11; // Forward from Execute stage
            end else if (rf_en_mem && (rd_in_mem == ra_in)) begin
                OperandA_MUX_CTRL = 2'b01; // Forward from Memory stage
            end else if (rf_en_wb && (rd_in_wb == ra_in)) begin
                OperandA_MUX_CTRL = 2'b10; // Forward from Write-Back stage
            end else begin
                OperandA_MUX_CTRL = 2'b00;
            end

            // Forwarding logic for OperandB
            if (rf_en_exe && (rd_in_exe == rb_in)) begin
                OperandB_MUX_CTRL = 2'b11; // Forward from Execute stage
            end else if (rf_en_mem && (rd_in_mem == rb_in)) begin
                OperandB_MUX_CTRL = 2'b01; // Forward from Memory stage
            end else if (rf_en_wb && (rd_in_wb == rb_in)) begin
                OperandB_MUX_CTRL = 2'b10; // Forward from Write-Back stage
            end else begin
                OperandB_MUX_CTRL = 2'b00;
            end

            // Forwarding logic for OperandD
            if (rf_en_exe && (rd_in_exe == rd_in_id)) begin
                OperandD_MUX_CTRL = 2'b11; // Forward from EXE stage
            end else if (rf_en_mem && (rd_in_mem == rd_in_id)) begin
                OperandD_MUX_CTRL = 2'b01; // Forward from MEM stage
            end else if (rf_en_wb && (rd_in_wb == rd_in_id)) begin
                OperandD_MUX_CTRL = 2'b10; // Forward from WB stage
            end else begin
                OperandD_MUX_CTRL = 2'b00;
            end
        end
    endmodule

    module ConditionHandler(
        input B_in,
        input BL_in,
        input [3:0] I_Cond_in,

        input [31:0] Flags_in,

        output reg TA_Ctrl_out,
        output reg BL_COND_out,
        output reg COND_EVAL_out
        );

        reg Z,N,C,V;

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

        always @(*) begin
            Z = Flags_in[0];
            N = Flags_in[1];
            C = Flags_in[2];
            V = Flags_in[3];


            TA_Ctrl_out = 0;
            BL_COND_out = 0;
            COND_EVAL_out = 0;

            case (I_Cond_in)
                EQUALS:           COND_EVAL_out = Z;
                NOT_EQUALS:       COND_EVAL_out = ~Z;
                CARRY_SET:        COND_EVAL_out = C;
                CARRY_CLEAR:      COND_EVAL_out = ~C;
                MINUS:            COND_EVAL_out = N;
                PLUS:             COND_EVAL_out = ~N;
                OVERFLOW:         COND_EVAL_out = V;
                NO_OVERFLOW:      COND_EVAL_out = ~V;
                UNSIGNED_HIGHER:  COND_EVAL_out = C && ~Z;
                UNSIGNED_LOWER:   COND_EVAL_out = ~C || Z;
                GREATER_EQUAL:    COND_EVAL_out = (N == V);
                LESS_THAN:        COND_EVAL_out = (N != V);
                GREATER_THAN:     COND_EVAL_out = ~Z && (N == V);
                LESS_EQUAL:       COND_EVAL_out = Z || (N != V);
                ALWAYS:           COND_EVAL_out = 1;
                NEVER:            COND_EVAL_out = 0;
                default:          COND_EVAL_out = 0;
            endcase

            TA_Ctrl_out = (B_in || BL_in) && COND_EVAL_out;
            BL_COND_out = BL_in && COND_EVAL_out;
        end
    endmodule
    
    module ProgramStatusRegister(
        input S_bit_in,

        input [31:0] Flags_in, 
        output reg [31:0] Flags_out 
        );

        reg Z, N, C, V;

        always @(*) begin
            if (S_bit_in) begin
            Z <= Flags_in[0];
            N <= Flags_in[1];
            C <= Flags_in[2];
            V <= Flags_in[3];
            end
            Flags_out <= {28'b0, V,C,N,Z};
        end
    endmodule

    module In2Out1MUX32(
        input [31:0] In1, In2,
        input selector,

        output reg [31:0] out
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
        input [31:0] In1, In2, In3, In4,
        input [1:0] selector,

        output reg [31:0] out
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
        input [24:0] reladdin,
        output reg [31:0] reladdout
        );
        always@(*) begin
    // Sign-extend the 24-bit offset to 32 bits
    reladdout = {{8{reladdin[23]}}, reladdin} << 2;
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

    module ALU (
        input [3:0] Op,     
        input [31:0] A, B,
        input CIN,            
        
        output reg [31:0] Out,       
        output reg [31:0] Flags

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
        reg Z, N, C, V;
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
            Flags = {28'b0, V,C,N,Z}; //New flags format
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

    module ram (
        output reg [31:0] DataOut,  
        input E,               
        input RW,                   
        input [7:0] A,              
        input [31:0] DataIn,        
        input Size           
        );

        reg [7:0] Mem [0:255];  

        always @(E) begin
                if (RW == 0) begin  
                        case (Size)
                            0: DataOut = {24'b0, Mem[A]};  //Read Byte

                            1: DataOut = {Mem[A], Mem[A+1], Mem[A+2], Mem[A+3]};   //Read Word
                        endcase
                    end

                if (RW == 1) begin  
                    case (Size)
                        0: Mem[A] = DataIn[7:0]; //Write Byte
                        
                        1: begin                 //Write Word
                            Mem[A] = DataIn[7:0];   
                            Mem[A+1] = DataIn[15:8];
                            Mem[A+2] = DataIn[23:16];
                            Mem[A+3] = DataIn[31:24]; 
                        end
                    endcase
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
                    next_pc_out <= next_pc_in;
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

        output reg rf_en_out, s_bit_out, datamem_en_out, readwrite_out, size_out, load_instruction_out,
        output reg [1:0] am_out,
        output reg [3:0] alu_op_out,
        output reg [3:0] instr_cond_out,
        output reg [31:0] next_pc_out, operand_a_out, operand_b_out, operand_d_out,
        output reg [11:0] immediate_out,
        output reg [3:0] rd_out,
        output reg NEXTORALU_ctrl_out
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
        
        output reg rf_en_out,
        output reg [31:0] mem_mux_out,
        output reg [3:0] rd_out
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

    module register_file(
        input LE, Clk,
        input [31:0] PC,PW,
        input [3:0] RD,RB,RA,RW,
        output [31:0] PD,PB,PA
        );

        wire [15:0] regnum;
        wire [31:0] Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15;


        //declarando los modulos individuales las veces necesarias para hacer un three port register file y haciendo las 
        //conneciones necesarias
        decoder decoder1(regnum,LE,RW);

        register R0(regnum[0], Clk, PW, Q0);
        register R1(regnum[1], Clk, PW, Q1);
        register R2(regnum[2], Clk, PW, Q2);
        register R3(regnum[3], Clk, PW, Q3);
        register R4(regnum[4], Clk, PW, Q4);
        register R5(regnum[5], Clk, PW, Q5);
        register R6(regnum[6], Clk, PW, Q6);
        register R7(regnum[7], Clk, PW, Q7);
        register R8(regnum[8], Clk, PW, Q8);
        register R9(regnum[9], Clk, PW, Q9);
        register R10(regnum[10], Clk, PW, Q10);
        register R11(regnum[11], Clk, PW, Q11);
        register R12(regnum[12], Clk, PW, Q12);
        register R13(regnum[13], Clk, PW, Q13);
        register R14(regnum[14], Clk, PW, Q14);

        in16out1mux mux1(PA,RA,Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,PC);

        in16out1mux mux2(PB,RB,Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,PC);

        in16out1mux mux3(PD,RD,Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,PC);
    endmodule

    module decoder (
        output reg[15:0] regnum,
        input LE,
        input [3:0] RW
        );

        always @(*)
            begin
                if(LE == 1'b1) begin
                    case(RW)
                    4'b0000: regnum = 16'b0000000000000001;
                    4'b0001: regnum = 16'b0000000000000010;
                    4'b0010: regnum = 16'b0000000000000100; 
                    4'b0011: regnum = 16'b0000000000001000; 
                    4'b0100: regnum = 16'b0000000000010000;
                    4'b0101: regnum = 16'b0000000000100000; 
                    4'b0110: regnum = 16'b0000000001000000; 
                    4'b0111: regnum = 16'b0000000010000000; 
                    4'b1000: regnum = 16'b0000000100000000; 
                    4'b1001: regnum = 16'b0000001000000000; 
                    4'b1010: regnum = 16'b0000010000000000; 
                    4'b1011: regnum = 16'b0000100000000000; 
                    4'b1100: regnum = 16'b0001000000000000; 
                    4'b1101: regnum = 16'b0010000000000000; 
                    4'b1110: regnum = 16'b0100000000000000;     
                    4'b1111: regnum = 16'b1000000000000000;
                endcase
            end else regnum = 16'b0000000000000000;
        end
    endmodule

    module in16out1mux(
        output reg [31:0] Y, 
        input [3:0] R,
        input [31:0] Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15
        );

        always @(*) begin
            case(R)
            4'b0000: Y = Q0;
            4'b0001: Y = Q1;
            4'b0010: Y = Q2;
            4'b0011: Y = Q3;
            4'b0100: Y = Q4;
            4'b0101: Y = Q5;
            4'b0110: Y = Q6;
            4'b0111: Y = Q7;
            4'b1000: Y = Q8;
            4'b1001: Y = Q9;
            4'b1010: Y = Q10;
            4'b1011: Y = Q11;
            4'b1100: Y = Q12;
            4'b1101: Y = Q13;
            4'b1110: Y = Q14;
            4'b1111: Y = Q15;
            endcase
        end
    endmodule

    module register(
        input LE, Clk,
        input [31:0] PW,
        output reg[31:0] Q
        );

        always @(posedge Clk)
            if(LE) Q <= PW;
    endmodule
