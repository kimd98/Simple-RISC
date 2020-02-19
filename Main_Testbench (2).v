//This testbench was inspired by autograder test
module lab8_top_tb();
 reg clk, err;
 reg [3:0] KEY;
 reg [9:0] SW;
 wire [9:0] LEDR;
 wire [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

 //lab8 module instantiated
 lab8_top #("lab8_top_tb.txt") DUT(KEY, SW, LEDR, HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, clk);

 initial forever begin
  clk = 1'b0; #5;
  clk = 1'b1; #5;
 end
 wire break = (LEDR[8] == 1'b1);
 initial begin
  err = 0;
  KEY[1] = 1'b0; // reset asserted
  #10; // wait until next falling edge of clock
  KEY[1] = 1'b1; // reset de-asserted, PC still undefined
  while (~break) begin
   @(posedge (DUT.CPU.FSM.state == 4'b0001) or posedge break);  // wait until IF1 
   @(negedge clk); // show advance to negative edge of clock
   $display("PC = %h", DUT.CPU.PC); 
  end
  if (DUT.MEM.mem[1] !== 16'd12) begin err = 1; $display("FAILED: mem[1] wrong %b", DUT.MEM.mem[1]); $stop; end
  if (~err) $display("PASSED");
  $stop;
 end
endmodule
