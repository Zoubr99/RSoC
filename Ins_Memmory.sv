//`include "clk_divider.sv"

module Ins_Memmory(
    input logic CLK,
    input logic clk, 
    input logic [31:0] MEM_addr,
    output logic [31:0] MEM_dout,
    input logic rMEM_en,
    input logic [31:0] MEM_Wdata,
    input logic [3:0] MEM_Wmask
);
    
    reg [31:0] MEM [0:150000];
	//reg [31:0] MEM [0:1500];

       initial begin
       $readmemh("testing_codet219.hex",MEM);
	//$readmemh("riscv_logo.bram.hex",MEM); t39
       end

   wire [29:0] Word_addr = MEM_addr[31:2];

  always_ff @(posedge clk) begin 
    if (rMEM_en) begin
        MEM_dout <= MEM[Word_addr];
    end
  
    if(MEM_Wmask[0]) MEM[Word_addr][7:0] <= MEM_Wdata[7:0];
    
    if(MEM_Wmask[1]) MEM[Word_addr][15:8] <= MEM_Wdata[15:8];
    
    if(MEM_Wmask[2]) MEM[Word_addr][23:16] <= MEM_Wdata[23:16];

    if(MEM_Wmask[3]) MEM[Word_addr][31:24] <= MEM_Wdata[31:24];   
  end


endmodule
