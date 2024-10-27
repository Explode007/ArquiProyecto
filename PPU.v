`include "Adder.v"
`include "ControlUnit.v"
`include "cuMux.v"
`include "PC.v"
`include "rom.v"
`include "fase3pipereg.v"

module PPU();
    reg rst;
    reg clk;
    reg s;
    //Wires & More!

    //Preload
    integer fi, code, i;
    reg [7:0] data;
    reg [8:0] Address;

    //PC Wires
    wire [31:0] PC_Out; 
    wire [31:0] PC_In; 
    reg LE;
    wire [31:0] Adder_Out;

    //ROM Wires
    wire [31:0] insMem_Out;

    //IF-ID Reg Wires
    wire [31:0] cu_in;

    //Control Unit Wires to MUX
    wire [1:0] am_cu_out;
    wire rf_en_cu_out;
    wire [3:0] alu_op_cu_out;
    wire Load_cu_out;
    wire branch_link_cu_out;
    wire s_bit_cu_out;
    wire rw_cu_out;
    wire size_cu_out;
    wire datamem_en_cu_out;

    //Control Unit Wires OUT of Mux 
    wire [1:0] am_out_cumux;
    wire rf_en_out_cumux;
    wire [3:0] alu_op_out_cumux;
    wire Load_out_cumux;
    wire branch_link_out_cumux;
    wire s_bit_out_cumux;
    wire rw_out_cumux;
    wire size_out_cumux;
    wire datamem_en_out_cumux;

    //ID_EXE OUT
    wire [1:0] am_out_idexe;
    wire rf_en_out_idexe;
    wire [3:0] alu_op_out_idexe;
    wire s_bit_out_idexe;
    wire rw_out_idexe;
    wire size_out_idexe;
    wire datamem_en_out_idexe;
    wire Load_out_idexe;

    //EXE_MEM OUT
    wire rf_en_out_exemem;
    wire rw_out_exemem;
    wire size_out_exemem;
    wire datamem_en_out_exemem;
    wire Load_out_exemem;

    //MEM_WB OUT
    wire rf_en_out_memwb;

    //Instantiations here!
    PC PCReg(
        .E(LE), 
        .Reset(rst), 
        .clk(clk), 
        .PC_In(PC_In), 
        .PC_Out(PC_Out)
    );

    adder add(
        .Adder_OUT(Adder_Out),
        .Adder_IN(PC_Out)
    );

    rom insMem(
        .I(insMem_Out),
        .A(PC_Out[7:0])
    );

    control_unit CU(
        //INPUTS
        .instruction(cu_in),

        //OUTPUTS
        .AM(am_cu_out),
        .rf_en(rf_en_cu_out),
        .alu_op(alu_op_cu_out),
        .Load(Load_cu_out),
        .branch_link(branch_link_cu_out),
        .s_bit(s_bit_cu_out),
        .rw(rw_cu_out),
        .size(size_cu_out),
        .datamem_en(datamem_en_cu_out)
    );

    cuMux Mux(
        .s(s),
        
        //INPUTS
        .am_in(am_cu_out),
        .rf_en_in(rf_en_cu_out),
        .alu_op_in(alu_op_cu_out),
        .Load_in(Load_cu_out),
        .branch_link_in(branch_link_cu_out),
        .s_bit_in(s_bit_cu_out),
        .rw_in(rw_cu_out),
        .size_in(size_cu_out),
        .datamem_en_in(datamem_en_cu_out),

        //OUTPUTS
        .rf_en_out(rf_en_out_cumux),
        .alu_op_out(alu_op_out_cumux),
        .Load_out(Load_out_cumux),
        .branch_link_out(branch_link_out_cumux),
        .s_bit_out(s_bit_out_cumux),
        .rw_out(rw_out_cumux),
        .size_out(size_out_cumux),
        .datamem_en_out(datamem_en_out_cumux)
    );

    if_id_reg IF_ID(
        .clk(clk),
        .load_enable(LE),
        .reset(rst),

        //INPUTS
        .instruction(PC_Out),
        
        //OUTPUTS
        .cu_in(cu_in)
    );

    id_exe_reg ID_EXE(
        .clk(clk), 
        .reset(rst),

        //INPUTS
        .am(am_out),
        .alu_op(alu_op_out),
        .rf_en(rf_en_out),
        .s(s_out),
        .datamem_en(datamem_en_out),
        .readwrite(readwrite_out),
        .size(size_out),
        .load_instruction(load_instruction_out),

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

    mem_wb_reg MEM_WB(
        .clk(clk),
        .reset(rst),
        
        //INPUTS
        .rf_en(rf_en_out_exemem),
        
        //OUTPUTS
        .rf_en_out(rf_en_out_memwb)
    );

    //Simulation stuff here!
    initial begin
        // Open and read the test code file
        fi = $fopen("testcode_Fase_III(2).txt", "r");
        Address = 9'b000000000;
        while (!$feof(fi)) begin
            code = $fscanf(fi, "%b", data);  // Read binary instructions
            insMem.Mem[Address] = data;      // Load instructions into ROM
            Address = Address + 1;
        end
        $fclose(fi);  // Close the file

        // Initialize the signals
        LE = 1'b1;
        clk = 0;
        rst = 1'b1;
        s = 1'b0;

        // Generate clock signal, toggles every 2 time units
        forever #2 clk = ~clk;

        // Release reset at time 3
        #3 rst = 1'b0;

        // Change the mux control signal at time 32
        #32 s = 1'b1;

        // Monitor outputs at each clock cycle
        $display("Time\tPC\tAM\tS_bit\tDATAMEM-EN\tR/W\tSIZE\tRF_EN\tALU_OP\tLOAD\tBRANCH_LINK");
        $monitor("%0t\t%d\t%b\t%b\t%b\t%b\t%b\t%b\t%b\t%b\t%b",
                $time, 
                PC_Out,  // Print PC in decimal
                AM, 
                s_bit, 
                datamem_en, 
                rw, 
                size, 
                rf_en, 
                alu_op, 
                Load, 
                branch_link
        );

        // End the simulation at time 40
        #40 $finish;
    end
endmodule