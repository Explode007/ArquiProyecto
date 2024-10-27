module if_id_reg(
    input clk, load_enable,reset,
    input [31:0] instruction, 
    output reg [31:0] cu_in);

    always @ (posedge clk) begin
        if(load_enable) begin
            if(reset) begin
                // instruction = 0; (I don't know if this was missing)
                cu_in = 0;

            end else begin
                cu_in = instruction;
            end
        end
    end
endmodule

module id_exe_reg(
    input clk, reset,
    input [1:0] am,
    input [3:0] alu_op,
    input rf_en,s,datamem_en,readwrite,size,load_instruction,

    output reg [1:0] am_out,
    output reg [3:0] alu_op_out,
    output reg rf_en_out,s_out,datamem_en_out,readwrite_out,size_out,load_instruction_out
    );

    always @ (posedge clk) begin 
        if(reset) begin
            am_out = 0;
            alu_op_out = 0;
            rf_en_out = 0;
            s_out = 0;
            datamem_en_out = 0;
            readwrite_out = 0;
            size_out = 0;
            load_instruction_out = 0;
        end else begin
            am_out = am;
            alu_op_out = alu_op;
            rf_en_out = rf_en;
            s_out = s;
            datamem_en_out = datamem_en;
            readwrite_out = readwrite;
            size_out = size;
            load_instruction_out = load_instruction;
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
            rf_en_out = 0;
            datamem_en_out = 0;
            readwrite_out = 0;
            size_out = 0;
            load_instruction_out = 0;
        end else begin
            rf_en_out = rf_en;
            datamem_en_out = datamem_en;
            readwrite_out = readwrite;
            size_out = size;
            load_instruction_out = load_instruction;
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
            rf_en_out = 0;
        end else begin
            rf_en_out = rf_en;
        end
    end
endmodule










