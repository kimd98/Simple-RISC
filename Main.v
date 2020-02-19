`define MREAD 2'b01 //MREAD
`define MWRITE 2'b10 //MWRITE

module lab8_top(KEY, SW, LEDR, HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, CLOCK_50);
   parameter filename = "lab8fig4.txt";
   input CLOCK_50;
   input [3:0] KEY;
   input [9:0] SW;
   output [9:0] LEDR;
   output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

   wire [8:0] 	mem_addr;
   wire [15:0] 	read_data, write_data, dout, ir;
   wire [1:0] 	mem_cmd;
   wire 	msel, write, halt, LEDload, switch_load;

   //LEDR[8] is on when halt state
   assign LEDR[8:8] = halt;
   
   //Instantiating memory
   RAM #(16, 8, filename) MEM (.clk(CLOCK_50), .read_address(mem_addr[7:0]), .write_address(mem_addr[7:0]), .write(write), .din(write_data), .dout(dout));

   //Instantiating the cpu
   cpu CPU (.clk(CLOCK_50), .reset(~KEY[1]), .in(read_data), .out(write_data), .mem_addr(mem_addr), .mem_cmd(mem_cmd), .halt(halt));

   //Instantiating the input_iface 
   //input_iface IN(CLOCK_50, SW, ir, LEDR[7:0]);

   //Register that controls when the LEDs are updated
   register #(8) LEDs (.data_in(write_data[7:0]), .load(LEDload), .clk(CLOCK_50), .data_out(LEDR[7:0]));

   //Tri state buffer for output
   assign read_data = msel && (`MREAD === mem_cmd) ? dout : {16{1'bz}};
   
   //msel
   assign msel = mem_addr[8:8] === 1'b0;

   //write
   assign write = msel & (`MWRITE === mem_cmd);
   
   //Tristate buffers controlling switch inputs
   assign read_data[15:8] = switch_load ? 8'h00 : {8{1'bz}};
   assign read_data[7:0] = switch_load ? SW : {8{1'bz}};

   //Change switch_load based on mem_cmd and mem_addr (lab7 stage3)
   assign switch_load = (mem_cmd === `MREAD) & (mem_addr === 9'h140);

   //Change LEDload based on mem_cmd and mem_addr (lab7 stage3)
   assign LEDload = (mem_addr === 9'h100)&(mem_cmd === `MWRITE);
   
endmodule // lab8_top

module sseg(in,segs);
   input [3:0] in;
   output reg [6:0] segs;

   //Displays the hexdecimal version of out on the DE1-SOC
   always @(*)begin
      case(in)
	4'b0000 : segs <= 7'b1000000;
	4'b0001 : segs <= 7'b1111001;
	4'b0010 : segs <= 7'b0100100;
	4'b0011 : segs <= 7'b0110000;
	4'b0100 : segs <= 7'b0011001;
	4'b0101 : segs <= 7'b0010010;
	4'b0110 : segs <= 7'b0000010;
	4'b0111 : segs <= 7'b1111000;
	4'b1000 : segs <= 7'b0000000;
	4'b1001 : segs <= 7'b0011000;
	4'b1010 : segs <= 7'b0001000;
	4'b1011 : segs <= 7'b1000011;
	4'b1100 : segs <= 7'b1000110;
	4'b1101 : segs <= 7'b0100001;
	4'b1110 : segs <= 7'b0000110;
	4'b1111 : segs <= 7'b0001110;
	default : segs <= {7{1'bx}};
      endcase // case (in)
   end 
endmodule

//input_iface module
module input_iface(clk, SW, ir, LEDR);
  input clk;
  input [9:0] SW;
  output [15:0] ir;
  output [7:0] LEDR;
  wire sel_sw = SW[9];  
  wire [15:0] ir_next = sel_sw ? {SW[7:0],ir[7:0]} : {ir[15:8],SW[7:0]};

  //D flip-flop module instantiation
  vDFF #(16) REG(clk,ir_next,ir);

  //Assign LEDR
  assign LEDR = sel_sw ? ir[7:0] : ir[15:8];  
endmodule  

//DFF module
module vDFF(clk,D,Q);
  parameter n=1;
  input clk;
  input [n-1:0] D;
  output [n-1:0] Q;
  reg [n-1:0] Q;

  //Output changes in the rising edge of clk
  always @(posedge clk)
    Q <= D;
endmodule

