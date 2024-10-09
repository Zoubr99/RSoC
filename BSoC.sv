//`include "clk_divider.sv"
//`include "emitter_uart.v"

module SoC #(parameter BRG_BASE = 32'hc000_0000)

(

        input logic CLK,
        input logic RESET,

        input logic [15:0] sw,
        output logic  [15:0] led,

        input logic  rx, 
        output logic tx

);

        wire clk;
        wire resetn;
    
    
        wire [31:0] inter_MEMaddr;
        wire [31:0] inter_MEMdout;
        wire inter_rMEMenable;
        wire [31:0] inter_MEM_Wdata;
        wire [3:0] inter_MEM_Wmask;
        wire [31:0] leds_x10;
        reg [31:0] MEM_io_leds;


        wire [31:0] inter_RAM_dout;
        wire [31:0] inter_IO_dout;
        wire [29:0] inter_MEM_Waddr = inter_MEMaddr[31:2];
        wire isIO = (inter_MEMaddr[31:24] == BRG_BASE[31:24]); // SoC
	//wire isIO = (((inter_MEMaddr[31] && inter_MEMaddr[30]) == 1) & ((inter_MEMaddr[29] && inter_MEMaddr[28]) == 0));
        wire isRAM = !isIO;
        wire inter_wMEM_en = |inter_MEM_Wmask; 

        // Basic bus 
        wire b_mmio_cs; 
        wire b_wr;      
        wire b_rd;     
        wire [20:0] b_addr;       
        wire [31:0] b_wr_data;    
        wire [31:0] b_rd_data;
        
       
        
    Ins_Memmory RAM (
        .CLK(CLK),
        .clk(clk),
        .MEM_addr(inter_MEMaddr),
        .MEM_dout(inter_RAM_dout),
        .rMEM_en(isRAM & inter_rMEMenable), 
        .MEM_Wdata(inter_MEM_Wdata),
        .MEM_Wmask({4{isRAM}} & inter_MEM_Wmask)
    );





    Processor_Core CPU(
        .clk(clk),
        .resetn(resetn),
        .MEM_addr(inter_MEMaddr),
        .MEM_dout(inter_MEMdout),
        .rMEM_en(inter_rMEMenable),
        .MEM_Wdata(inter_MEM_Wdata),
        .MEM_Wmask(inter_MEM_Wmask),
        .x10(leds_x10)
    );


    // instantiate bridge
    mcs_bridge #(.BRG_BASE(BRG_BASE))
    bridge_unit (
    .io_address(inter_MEMaddr),
    .io_read_data(inter_IO_dout),
    .io_read_strobe(isIO & inter_rMEMenable),
    .io_write_data(inter_MEM_Wdata),
    .io_write_strobe(isIO),
    
    //mmio bit signals
    .b_video_cs(),
    .b_mmio_cs(b_mmio_cs), 
    .b_wr(b_wr),
    .b_rd(b_rd),
    .b_addr(b_addr),
    .b_wr_data(b_wr_data),
    .b_rd_data(b_rd_data)                 
    );




    // instantiated i/o subsystem
   mmio_sys #(.N_SW(16),.N_LED(16)) mmio_unit (
   .clk(clk),
   .reset(resetn),
   .mmio_cs(b_mmio_cs),
   .mmio_wr(b_wr),
   .mmio_rd(b_rd),
   .mmio_addr(b_addr), 
   .mmio_wr_data(b_wr_data),
   .mmio_rd_data(b_rd_data),
   .sw(sw),
   .led(led),
   .rx(rx),
   .tx(tx)          
  );             

    assign inter_MEMdout = isRAM ? inter_RAM_dout : inter_IO_dout;    



    clk_divider #(.SLOW(1))
    clk_divider (

        .CLK(CLK),
        .RESET(RESET),
        .clk(clk),
        .resetn(resetn)

    );

    

    //assign TXD = 1'b0;

endmodule