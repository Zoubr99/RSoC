module tb;
	// declaring the tb wires which to be connected the uut
   reg clk;
   wire reset = 0; 
   wire [15:0] led;
   wire [15:0] swth;
   reg  rxd = 1'b0;
   wire txd;

	// instantiating the uut
   SoC uut(
     .CLK(clk),
     .RESET(reset),
     .sw(swth),
     .led(led),
     .rx(rxd),
     .tx(txd)
   );
	// logic: every time 1 ps passes the clock will flip and it will check if the LEDs (counter) does not equal to the prev value
	// ie reached max of 5 bits 11111 it will keep printing (overflow condition)
   logic [4:0] prev_LEDS = 0;
   initial begin
      clk = 0;
      forever begin
	 #1 clk = ~clk;
	 if(led != prev_LEDS) begin
	    $display("LEDS = %b",led);
	 end
	 prev_LEDS <= led;
      end
   end
endmodule   