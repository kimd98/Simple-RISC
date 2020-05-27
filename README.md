# Simple RISC Machine

The Simple RISC (Reduced Instruction Set Comptuer) Machine Datapath consists of one register file constaining 8 registers, each holding 16-bits; three multiplexers; three 16-bit registers with load enable; a 1-bit register with load enable; a shifter unit; and an arithmetic logic unit(ALU).

The two key additions are made to the data path: One, a finite state machine controller to automate the process of setting control inputs to the datapath; and, two, an intruction register to control the finite state machine. In addition, it includes a read-write memory to hold instructions so we no longer need to enter each intruction using an input device (the slider switches were used to maunally control each element of the datapath and enter encoded instructions as input to the state machine). 

The interface has been extended to memory to enable communication with the outside world using memory mapped I/O. The user can read the values input on the slider switches using the LDR instruction and output a value to the red LEDs using the STR instruction.

Finally, to make the Simple RISC Machine "Turing complete", which means a program can be written to implement any algorithm that can run within the 256 word memory, two types of branch instructions were added.

*UBC CPEN211 (Lena Kim, Ken Johnson)
