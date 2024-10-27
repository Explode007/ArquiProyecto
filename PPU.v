//This is where all the wires are created and all components instantiated, essentially codifying the PPU

`include "Adder.v"
`include "ControlUnit.v"
`include "mux.v"
`include "PC.v"
`include "rom.v"
`include "pipeline_registers.v"

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

//Instantiations here!

PC PCReg(
    .E(LE), 
    .Reset(rst), 
    .clk(clk), 
    .PC_In(PC_In), 
    .PC_Out(PC_Out)
);

Adder add(
    .Adder_OUT(Adder_Out),
    .A(PC_Out)
);

rom insMem(
    .I(PC_Out),
    .A(insMem_Out)
);

ControlUnit CU(

);

pipeline_registers PR(

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