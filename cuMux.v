module cuMux (
    input s,

    input am_in,
    input rf_en_in,          
    input [3:0] alu_op_in,   
    input Load_in,            
    input branch_link_in,    
    input s_bit_in,          
    input rw_in,             
    input size_in,           
    input datamem_en_in,

    output reg rf_en_out,          
    output reg [3:0] alu_op_out,   
    output reg Load_out,            
    output reg branch_link_out,   
    output reg s_bit_out,          
    output reg rw_out,             
    output reg size_out,           
    output reg datamem_en_out
);

    always @* begin
        if(s == 1'b0) begin // Pass Control Unit values when selector is 0
            am_in <= am_in;
            rf_en_out <= rf_en_in;
            alu_op_out <= alu_op_in;
            Load_out <= Load_in;
            branch_link_out <= branch_link_in;
            s_bit_out <= s_bit_in;
            rw_out <= rw_in;
            size_out <= size_in;
            datamem_en_out <= datamem_en_in;
        end 
        else begin
            am_in <= 2'b0;
            rf_en_out <= 1'b0;
            alu_op_out <= 1'b0;
            Load_out <= 1'b0;
            branch_link_out <= 1'b0;
            s_bit_out <= 1'b0;
            rw_out <= 1'b0;
            size_out <= 1'b0;
            datamem_en_out <= 1'b0;
        end
    end
endmodule