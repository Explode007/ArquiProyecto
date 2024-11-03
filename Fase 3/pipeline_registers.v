module if_id_reg(
input clk, load_enable,reset,
input [31:0] instruction, 
input [31:0] next_pc,
output reg [3:0] instruction_condition, 
output reg [23:0] branch_offset, 
output reg [31:0] next_pc_out, 
output reg [3:0] ra,rd,rb,
output reg [11:0] imm_in,
output reg [31:0] cu_in);

always @ (posedge clk)
begin
if(load_enable)
begin

if(reset)
begin

instruction_condition <= 0;
branch_offset <= 0;
next_pc_out <= 0;
ra <= 0;
rd <= 0;
rb <= 0;
imm_in <= 0;
cu_in <= 0;

end else begin

instruction_condition <= instruction[31:28];
branch_offset <= instruction[21:0];
next_pc_out <= next_pc;
ra <= instruction[19:16];
rd <= instruction[15:12];
rb <= instruction[3:0];
imm_in <= instruction[23:0];
cu_in <= instruction;

end
end
end
endmodule

module id_exe_reg(
input clk, reset,mux_control,
input [3:0] instruction_condition,
input [31:0] next_pc,operand_a, operand_b, operand_d,
input [3:0] rd,
input [11:0] imm_in,
input [1:0] am,
input [3:0] alu_op,
input rf_en,s,datamem_en,readwrite,size,load_instruction,

output reg mux_control_out,
output reg [3:0] instruction_condition_out,
output reg [31:0] next_pc_out,operand_a_out, operand_b_out, operand_d_out,
output reg [3:0] rd_out,
output reg [11:0] imm_in_out,
output reg [1:0] am_out,
output reg [3:0] alu_op_out,
output reg rf_en_out,s_out,datamem_en_out,readwrite_out,size_out,load_instruction_out
);

always @ (posedge clk)
begin 
if(reset)
begin
mux_control_out <= 0;
instruction_condition_out <= 0;
next_pc_out <= 0;
operand_a_out <= 0; 
operand_b_out <= 0; 
operand_d_out <= 0;
rd_out <= 0;
imm_in_out <= 0;
am_out <= 0;
alu_op_out <= 0;
rf_en_out <= 0;
s_out <= 0;
datamem_en_out <= 0;
readwrite_out <= 0;
size_out <= 0;
load_instruction_out <= 0;

end else begin

mux_control_out <= mux_control;
instruction_condition_out <= instruction_condition;
next_pc_out <= next_pc;
operand_a_out <= operand_a; 
operand_b_out <= operand_b; 
operand_d_out <= operand_d;
rd_out <= rd;
imm_in_out <= imm_in;
am_out <= am;
alu_op_out <= alu_op;
rf_en_out <= rf_en;
s_out <= s;
datamem_en_out <= datamem_en;
readwrite_out <= readwrite;
size_out <= size;
load_instruction_out <= load_instruction;

end
end
endmodule

module exe_mem_reg(
input clk, reset,
input [31:0] alu_or_nextpc,
input [31:0] data_in,
input [3:0] rd,
input rf_en,datamem_en,readwrite,size,load_instruction,

output reg [31:0] alu_or_nextpc_out,
output reg [31:0] data_in_out,
output reg [3:0] rd_out,
output reg rf_en_out,datamem_en_out,readwrite_out,size_out,load_instruction_out
);

always @ (posedge clk)
begin
if(reset)
begin
alu_or_nextpc_out <= 0;
data_in_out <= 0;
rd_out <= 0;
rf_en_out <= 0;
datamem_en_out <= 0;
readwrite_out <= 0;
size_out <= 0;
load_instruction_out <= 0;

end else begin

alu_or_nextpc_out <= alu_or_nextpc;
data_in_out <= data_in;
rd_out <= rd;
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
input [3:0] rd,
input [31:0] data,
input rf_en,

output reg [3:0] rd_out,
output reg [31:0] data_out,
output reg rf_en_out
);

always @ (posedge clk)
begin
if(reset)
begin
rd_out <= 0;
data_out <= 0;
rf_en_out <= 0;

end else begin

rd_out <= rd;
data_out <= data;
rf_en_out <= rf_en;

end
end
endmodule










