`include "Adder.v"
`include "ControlUnit.v"
`include "cuMux.v"
`include "PC.v"
`include "rom.v"
`include "pipeline_registers.v"

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
reg [31:0] next_PC;
wire [3:0] instruction_condition;
wire [23:0] branch_offset;
wire [31:0] next_pc_out;
wire [3:0] ra;
wire [3:0] rd;
wire [3:0] rb;
wire [11:0] imm_in;
wire [31:0] cu_in;

//Control Unit Wires
wire rf_en;
wire [3:0] alu_op;
wire Load;
wire branch_link;
wire s_bit;
wire rw;
wire size;
wire datamem_en;

//Control Unit Mux Wires
wire rf_en_out;
wire [3:0] alu_op_out;
wire Load_out;
wire branch_link_out;
wire s_bit_out;
wire rw_out;
wire size_out;
wire datamem_en_out;

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
    .instruction(cu_in),
    .rf_en(rf_en),
    .alu_op(alu_op),
    .Load(Load),
    .branch_link(branch_link),
    .s_bit(s_bit),
    .rw(rw),
    .size(size),
    .datamem_en(datamem_en)
);

cuMux Mux(
    .s(s),
    .rf_en_in(rf_en),
    .alu_op_in(alu_op),
    .Load_in(Load),
    .branch_link_in(branch_link),
    .s_bit_in(s_bit),
    .rw_in(rw),
    .size_in(size),
    .datamem_en_in(datamem_en),
    .rf_en_out(rf_en_out),
    .alu_op_out(alu_op_out),
    .Load_out(Load_out),
    .branch_link_out(branch_link_out),
    .s_bit_out(s_bit_out),
    .rw_out(rw_out),
    .size_out(size_out),
    .datamem_en_out(datamem_en_out)
);

if_id_reg IF_ID(
    .clk(clk),
    .load_enable(LE),
    .reset(rst),
    .instruction(PC_Out),
    .next_pc(next_PC),
    .instruction_condition(instruction_condition),
    .branch_offset(branch_offset),
    .next_pc_out(next_pc_out),
    .ra(ra),
    .rd(rd),
    .rb(rb),
    .imm_in(imm_in),
    .cu_in(cu_in)
);

id_exe_reg ID_EXE(
    .clk(clk), 
    .reset(rst),
    .mux_control,
    .instruction_condition(instruction_condition),
    .next_pc(next_pc_out),
    .operand_a,
    .operand_b,
    .operand_d,
    .rd,
    .imm_in,
    .am,
    .alu_op,
    .rf_en,
    .s,
    .datamem_en,
    .readwrite,
    .size,
    .load_instruction,
    .mux_control_out,
    .instruction_condition_out,
    .next_pc_out,
    .operand_a_out,
    .operand_b_out,
    .operand_d_out,
    .rd_out,
    .imm_in_out,
    .am_out,
    .alu_op_out,
    .rf_en_out,
    .s_out,
    .datamem_en_out,
    .readwrite_out,
    .size_out,
    .load_instruction_out
);

exe_mem_reg EXE_MEM(
    .clk(clk),
    .reset(rst),
    .alu_or_nextpc,
    .data_in,
    .rd,
    .rf_en,
    .datamem_en,
    .readwrite,
    .size,
    .load_instruction,
    .alu_or_nextpc_out,
    .data_in_out,
    .rd_out,
    .rf_en_out,
    .datamem_en_out,
    .readwrite_out,
    .size_out,
    .load_instruction_out
);

mem_wb_reg MEM_WB(
    .clk(clk),
    .reset(rst),
    .rd,
    .data,
    .rf_en,
    .rd_out,
    .data_out,
    .rf_en_out
);

//Simulation stuff here!

initial begin
    fi = $fopen("testcode_Fase_III(2).txt","r");
    Address = 9'b000000000;
    while (!$feof(fi)) begin
        code = $fscanf(fi, "%b", data);
        insMem.Mem[Address] = data;
        Address = Address + 1;
    end
    $fclose(fi);
end

initial begin
    LE = 1'b1;
end

initial begin
    clk = 0;
    forever #2 clk = ~clk;
end

initial begin
    rst = 1'b1;
    #3 rst = 1'b0;
end

initial begin
    rst = 1'b1;
    #3 rst = 1'b0;
end

initial begin
    s = 1'b0;
    #32 s = 1'b1;
end

initial begin
    #40 $finish;
end

initial begin
    $monitor("Time: %0t", 
    $time
    );
end

endmodule