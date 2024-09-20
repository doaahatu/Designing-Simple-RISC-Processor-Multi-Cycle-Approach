module tb_computer();

  reg clk, reset;
  wire [31:0] data_out_instruction, data_in, data_out, address_data;
  wire MemRead, MemWrite;
  wire [3:0] state;	
  wire [31:0] ALUresult, address_inst; 
  wire [1:0] ALUop;	
  reg [31:0] out;
  wire [31:0] rr;

  always #10ns clk = ~clk;

  initial begin
	  
	clk = 1;
    reset = 1;
	#1ns
	reset = 0;
	#1ns
	reset = 1;
	
    $monitor("instruction : 0x%x\n", data_out_instruction);
    #8000ns $finish;
  end

  computer dut (
  
    .clk(clk),
    .address_inst(address_inst),
    .data_out_instruction(data_out_instruction),
    .data_in(data_in),
    .data_out(data_out),
    .address_data(address_data),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
	.reset(reset),
	.state(state),
	.ALUresult(ALUresult),
	.ALUop(ALUop)
  );
  
endmodule 







module tb_mem();

  wire [31:0] data_out_instruction;
  reg clk;
  reg [31:0] address;

  always #10ns clk = ~clk;

  initial begin
	  
	clk = 1;
    address = 32'b0;
	
	#10ns
	address = 32'b1; 
	
	$display ("ioerfjoierhf : %d\n", 1<<31);
	
    $monitor("instruction : 0x%x\n", data_out_instruction);
    #200ns $finish;
  end

  instructionMemory inst_mem(
	
		.data(data_out_instruction), 
		.clk(clk),
		.address(address)					  
	);
  
endmodule








module computer (
	
     input clk, reset,
     output [31:0] address_inst, 
	 output [31:0] data_out_instruction,
     data_in, data_out, address_data,
  	 output MemRead,
     MemWrite,
	 output [3:0] state,
	 output [31:0] ALUresult,
	 output [1:0] ALUop
  );

	
	instructionMemory inst_mem(
	
		.data(data_out_instruction), 
		.clk(clk),
		.address(address_inst)					  
	);
	
	dataMemory data_mem(
	
		.dataOut(data_out),
		.clk(clk),
		.address(address_data),
		.dataIn(data_in),
		.memRead(MemRead), 
		.memWrite(MemWrite)
	);
	
	
	CPU cpu(
	
        .data_out(data_out),
        .data_out_instruction(data_out_instruction), 
        .clk(clk),
        .data_in(data_in),
        .address_data(address_data),
        .address_instruction(address_inst),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
		.reset(reset), 
		.state(state),
		.ALUresult(ALUresult),
		.ALUop(ALUop)
    );
	
endmodule
	





module datapath ( 
	
	input MemWriteSel, MemReg, RegSrc,
		  RegWrite, RegWrite2 ,
		  IRwrite, PCwrite,
		  StackWrite, sign_ext , clk, reset,
		  StackSelect,
		  
	input [31:0] data_out_instruction, data_out,
					  
	input [1:0] ALUsrcA, ALUsrcB,
				PCsrc, ALUop, StackALU,
	
	output [5:0] opcode, 
	output [1:0] mode,
	output [31:0] address_data, address_instruction, data_in,
	output z, n, v,
	output [31:0] ALU_result
);

	wire [3:0] Rs1, Rs2, Rd; 
	wire [31:0] pc_temp;
	
	assign pc_temp = address_instruction;
	
	wire [31:0] instruction, BUS1, BUS2, B_operand, A_operand, SP_address, 
				increment, extended_immediate, ALU_operand1, ALU_operand2,
				jumpTargetAddress, PCtype, ALU_result_buffer, decrement,
				stack_select_wire;
	
	wire[31:0] MDR_out, Register_result;
	wire [15:0] immediate;
	
	wire [3:0] Rs2_regfile;


	assign opcode = instruction[31:26];
	assign mode = instruction[1:0];
	assign Rd = instruction[25:22];
	assign Rs1 = instruction[21:18];
	assign Rs2 = instruction[17:14];
	
	assign immediate = instruction[17:2];
	assign jumpTargetAddress={address_instruction[31:26], instruction[25:0]}; 
	
	assign increment = A_operand + 1;
	assign decrement = SP_address - 1;
	
	
	flop IR (.out(instruction), .clk(clk), .writeEn(IRwrite), .in(data_out_instruction), .reset(1'b1));
	
	flop A (.out(A_operand), .clk(clk), .writeEn(1'b1), .in(BUS1), .reset(1'b1));
	
	flop B (.out(B_operand), .clk(clk), .writeEn(1'b1), .in(BUS2), .reset(1'b1)); 
	
	flop pc(.out(address_instruction) , .clk(clk), .writeEn(PCwrite), .in(PCtype) , .reset(reset));
	
	flop ALUout(.in(ALU_result), .clk(clk), .writeEn(1'b1), .out(ALU_result_buffer), .reset(1'b1));
	
	flop SP(.in(stack_select_wire), .clk(clk), .writeEn(StackWrite), .out(SP_address), .reset(reset));
	
	flop MDR(.out(MDR_out), .clk(clk), .writeEn(1'b1), .in(data_out), .reset(1'b1));
	
	
	reg_file file (
	
		.clk(clk),
		.regWrite(RegWrite), .regWrite2(RegWrite2),
		.regDst1(Rd), .regDst2(Rs1), .regSrc1(Rs1),
		.regSrc2(Rs2_regfile),
		.bus_w(Register_result), .bus2_w(increment),
		.out1(BUS1), .out2(BUS2)
	);
	
	ALU alu(
	
		.ALUop(ALUop), 
		.a(ALU_operand1), 
		.b(ALU_operand2),
		.zero(z), 
		.overflow(v), 
		.negative(n),
		.result(ALU_result) 
	);
	
	
	mux2x1 #(4) RegSrc_mux (.a(Rs2), .b(Rd), .select(RegSrc), .out(Rs2_regfile)); 
	
	mux4x1 ALUsrc1_mux (.a(pc_temp), .b(A_operand), .c(SP_address), .d(32'b0), .s(ALUsrcA), .out(ALU_operand1));
	
	mux4x1 ALUsrc2_mux (.a(B_operand), .b(extended_immediate), .c(32'b1), .d(32'b0), .s(ALUsrcB), .out(ALU_operand2));
	
	mux4x1 pcMux(.a(jumpTargetAddress), .b(ALU_result), .c(data_out), .d(ALU_result_buffer), .s(PCsrc), .out(PCtype));
	
    mux4x1 data_mem_address(.a(ALU_result_buffer), .b(decrement), .c(SP_address), .d(32'b0), .s(StackALU), .out(address_data)); 
	
    mux2x1 dataIn(.a(B_operand), .b(address_instruction), .select(MemWriteSel), .out(data_in));
	
	mux2x1 stack_select_mux (.a(ALU_result), .b(decrement), .select(StackSelect), .out(stack_select_wire));
	
	mux2x1 ALU_mux(.a(ALU_result_buffer), .b(MDR_out), .select(MemReg), .out(Register_result));
	
	extender ext(.in(immediate), .sign_ext(sign_ext), .out(extended_immediate));				  

endmodule







module CPU (
	
	input [31:0] data_out, data_out_instruction, 
	input clk, reset,
	output [31:0] data_in, address_data,
				address_instruction,
	
	
	output MemWrite, MemRead,
	output [3:0] state,
	output [31:0] ALUresult,
	output [1:0] ALUop
);

	wire [5:0] opcode;
	wire [1:0] mode;
	
	wire    MemWriteSel, MemReg, RegSrc,
		   RegWrite, RegWrite2,
		   IRwrite, PCwriteUncond,
		   StackWrite, sign_ext, PCwriteCond, PCwrite;
		   
	wire [1:0] ALUsrcA, ALUsrcB,
	PCsrc , ALUctrl, StackALU;
	
	wire z, n, v;


	datapath d1 ( 
	
		.MemWriteSel(MemWriteSel),
	 	.MemReg(MemReg), .RegSrc(RegSrc),
		.RegWrite(RegWrite), .RegWrite2(RegWrite2),
		.IRwrite(IRwrite), .PCwrite(PCwrite), .StackALU(StackALU),
		.StackWrite(StackWrite), .sign_ext(sign_ext) , .clk(clk),
		.StackSelect(StackSelect),
		  
		.data_out_instruction(data_out_instruction), .data_out(data_out),
					  
		.ALUsrcA(ALUsrcA), .ALUsrcB(ALUsrcB),
		.PCsrc(PCsrc), .ALUop(ALUop), .reset(reset),
	
		.opcode(opcode), 
		.mode(mode), 
		.address_data(address_data), 
		.address_instruction(address_instruction), .data_in(data_in),
		.z(z), .n(n), .v(v),
		.ALU_result(ALUresult)
	);
	
	
	mainController control(	 
	
		.MemWriteSel(MemWriteSel),
	 	.MemReg(MemReg), .RegSrc(RegSrc),
		.RegWrite(RegWrite), .RegWrite2(RegWrite2),
		.IRwrite(IRwrite), .PCwriteUncond(PCwriteUncond), .StackALU(StackALU),
		.StackWrite(StackWrite), .sign_ext(sign_ext),
		.MemWrite(MemWrite), .MemRead(MemRead),
		   
		   
		.ALUsrcA(ALUsrcA), .ALUsrcB(ALUsrcB),
		.PCsrc(PCsrc), .ALUctrl(ALUctrl),  
	
		.clk(clk), .opcode(opcode),
		.mode(mode), .reset(reset),
		.state(state), .StackSelect(StackSelect)
	);
	
	
	PCcontrol pc_control(

		.opcode(opcode), 
		.z(z), .v(v), .n(n),
		.pcWriteUncond(PCwriteUncond),
		.writeEn(PCwrite), 
		.state(state)
	);
	
	
	ALUcontrol ALU_control(
	
		.ALUop(ALUop),
		.opcode(opcode),
		.ALUctrl(ALUctrl)
	); 	
	
endmodule



// Immediate Extender Module
module extender (
    input [8:0] inS,
    input [4:0] inI,
    input sign_ext,
    input select_type,
    output reg [15:0] out
);

    always @* begin
        if (select_type) begin
            // S-type immediate extension (9-bit)
            if (sign_ext && inS[8])
                out = {7'h7F, inS};  // Sign-extend 9-bit to 16-bit
            else
                out = {7'b0, inS};   // Zero-extend 9-bit to 16-bit
        end else begin
            // I-type immediate extension (5-bit)
            if (sign_ext && inI[4])
                out = {11'h7FF, inI}; // Sign-extend 5-bit to 16-bit
            else
                out = {11'b0, inI};   // Zero-extend 5-bit to 16-bit
        end
    end
endmodule


// Control unit module
module mainController (	 
	
	output reg MemWriteSel, MemReg, RegSrc,
		 	  RegWrite, RegWrite2, MemRead,
		   	  IRwrite, PCwriteUncond,
		       StackWrite, sign_ext, MemWrite,
			  StackSelect,
		   
		   
	output reg [1:0] ALUsrcA, ALUsrcB,
	PCsrc, ALUctrl, StackALU,
	
	output reg [3:0] state,
	
	input clk, reset, 
	input [5:0] opcode,
	input [1:0] mode
);		 						 
	reg jmp;
	reg PUSH_Branch;
	wire logical;
	
	parameter InstructionFetch = 0, 
		  	  InstructionDecode = 1, 
		  	  AddressComputation = 2, 
		  	  LoadAccess = 3, 
		  	  StoreAccess = 4,
			  ALUcomputationR = 5,
		  	  ResultStore = 6,
			  BranchCompletion = 7,
			  IncrementReg = 8,
			  ALUcomputationI = 9, 
			  PushAccess = 10,
			  PopAccess = 11,
			  ResultStoreMem = 12,
			  Call = 13,
			  Return = 14;
				
	parameter AND = 6'b0,
			  ADD = 6'b01,
			  SUB = 6'b10,
			  ANDI= 6'b11,
			  ADDI= 6'b100,
			  LW = 6'b000101,
			  LW_POI = 6'b000110,
			  SW = 6'b000111,
			  BGT = 6'b1000,
			  BLT = 6'b1001,
			  BEQ = 6'b1010,
			  BNE = 6'b1011,
			  JMP = 6'b1100,
			  CALL = 6'b1101,
			  RET = 6'b1110,
			  PUSH = 6'b1111,
			  POP = 6'b10000,
			  Inc = 2'b01,
			  NoInc = 2'b00; 
			  

			 
			  
	always @* begin 
		
		case (opcode) 
			JMP : begin
				
			   jmp = 1;
			   PUSH_Branch = 0;
			end
			
			BGT, BLT,
			BEQ, BNE,
			PUSH, SW :  begin 
				PUSH_Branch = 1;
				jmp = 0;
			end
			
			default : begin 
				PUSH_Branch = 0;
				jmp = 0;
			end
		endcase
	end
	
	assign logical = (opcode == ANDI); 
	
	
		
	always @(posedge clk or negedge reset) begin
		
		if (!reset)
			state <= InstructionFetch;
		
		else begin
			
		case (state)  
			
			InstructionFetch : state <= InstructionDecode;
			
			InstructionDecode :  begin
				
				case (opcode)
						
					AND, ADD, SUB: state <= ALUcomputationR;   //R-type			  
					LW, SW, LW_POI : state <= AddressComputation; //memory access
									
					BEQ, BNE, BGT, BLT : state <= BranchCompletion; //Branch instruction
									
					JMP : state <= InstructionFetch; //jmp instruction	
					ANDI, ADDI: state <= ALUcomputationI;  // I-type
					POP : state <= PopAccess;	  //POP
					PUSH : state <= PushAccess;
					CALL : state <= Call;
					RET : state <= Return;
				endcase
			 end

									
		  	AddressComputation : begin
				  
				case (opcode)
					LW : state <= LoadAccess; 
					SW : state <= StoreAccess;
				endcase
			end
					
			LoadAccess : begin
				
				case (mode)
					NoInc : state <= ResultStoreMem;
					Inc : state <= IncrementReg;
				endcase
			end

		  	 StoreAccess, ResultStoreMem,
			 ResultStore, IncrementReg, PushAccess, 
			 BranchCompletion, Return,
			 Call, PopAccess : state <= InstructionFetch;
	
			 ALUcomputationI, ALUcomputationR : state <= ResultStore;
			
		endcase	
		end
	
	end
	
	always @(*) begin
		
		MemWriteSel = 0;
		MemReg = 0;
		RegSrc = 0;
		RegWrite = 0;
		RegWrite2 = 0;
		MemRead = 0;
		MemWrite = 0;
		IRwrite = 0;
		PCwriteUncond = 0;
		StackALU = 0;
		StackWrite = 0; 
		sign_ext = 0;
		PCsrc = 0;
		ALUctrl = 0;
		ALUsrcA = 0;
		ALUsrcB = 0;
		StackSelect = 0;
			
		case (state) 
			
			InstructionFetch : begin
				
				IRwrite = 2'b1;
  				PCsrc = 2'b01;
 				PCwriteUncond = 2'b1;
 				ALUsrcA=2'b00;
 				ALUsrcB=2'b10;
  				ALUctrl=2'b01;
			end
			
		  	InstructionDecode : begin
					
				sign_ext = 1;
 				ALUsrcB = 2'b01;
				ALUsrcA =   2'b00;
				ALUctrl = 2'b01;
				RegSrc = PUSH_Branch;
				PCsrc = 2'b00;
				PCwriteUncond = jmp;
				StackALU = 2'b01; 
				MemRead = 1;
			end

		  	AddressComputation : begin
				  
				sign_ext = 1;
 				ALUsrcB = 2'b01;
				ALUsrcA = 2'b01; 
				ALUctrl = 2'b01;
				RegSrc = 2'b1;
			end
			 
			LoadAccess : MemRead = 2'b1;
	 		StoreAccess :	MemWrite = 2'b1;
			 
			ALUcomputationR :	begin
				 
				ALUsrcA = 2'b01;
				ALUsrcB = 2'b00;
				ALUctrl = 2'b00;
			end
				  
			ResultStore : RegWrite = 2'b1;
				
			ResultStoreMem : begin
				
				MemReg = 2'b1;
				RegWrite = 2'b1;
			end
				  
			BranchCompletion : begin
				
				PCsrc = 2'b11;
				ALUsrcA = 2'b01;
				ALUsrcB = 2'b00;
				ALUctrl= 2'b11;
			end
			  
			IncrementReg :  begin
				  
				RegWrite = 2'b1;
				RegWrite2 = 2'b1;
				MemReg = 2'b1;
			end
			  
			ALUcomputationI : begin
				  
				ALUsrcA = 2'b01;
				ALUsrcB = 2'b01;			  
				ALUctrl = 2'b00;
				sign_ext = !logical;
			end
			
			PushAccess : begin
				
				MemWrite = 1;
				ALUsrcA = 2'b10;
				ALUsrcB = 2'b10;
				StackALU = 2'b10;
				StackWrite = 1;
				ALUctrl = 2'b01;
			end
			
			PopAccess : begin
				
				StackWrite = 1;
				RegWrite = 1;
				MemReg = 1;
				StackSelect = 1;
			end
			
			Call : begin
				
				MemWriteSel = 1;
				StackWrite = 1;
				ALUsrcA = 2'b10;
				ALUsrcB = 2'b10;
				ALUctrl = 2'b01;
				StackALU = 2'b10;
				MemWrite = 1;
				PCwriteUncond = 1;
			end
			
			Return : begin
				
				PCsrc = 2'b10;
				StackWrite = 1;
				StackALU = 2'b01;
				PCwriteUncond = 1;
				StackSelect = 1;
				MemRead = 1;
				
			end
		
		endcase
		
	end

endmodule



// ALU Control module
module ALUcontrol (
	
	input [1:0] ALUctrl,
	input [3:0] opcode,
	output reg [1:0] ALUop
); 

parameter  	   
			  AND = 0'b00,
			  ADD = 2'b01,
			  SUB = 2'b10,
			  RSB = 2'b11;
			  

	always @* begin 
		case (ALUctrl)
		
			RSB, ADD, SUB : ALUop = ALUctrl; //the operation is generated by the main control
			2'b00 : begin
				case (opcode) //determine the operation based on the opcode 
					4'b0001, 4'b0100 : ALUop = ADD; //opcode = ADDI, ADD (R-TYPE)
					4'b0010 : ALUop = SUB; //opcode = SUB R-TYPE
					4'b0000, 4'b0011 : ALUop = AND; //opcode = AND, ANDI
				endcase
			end
		endcase
	end
	
endmodule



// PC Control
module PCcontrol (

	input [3:0] opcode, 
	input z, v, n,
	input pcWriteUncond,
	input [3:0] state,
	output writeEn
);
 
	parameter BNE = 4'b1011,
			  BNEZ = 4'b1011,
	          BEQ = 4'b1010,
			  BEQZ = 4'b1010,
			  BLT = 4'b1001,
			  BLTZ = 4'b1001,
			  BGT = 4'b1000,
			  BGTZ = 4'b1000;	 
			  
	wire branch;
	
	assign branch = ((opcode == BNE) && !z) ||
	                (opcode == BNEZ) && !z) ||
					((opcode == BEQ) && z) || 
					((opcode == BEQZ) && z) ||
					((opcode ==BGT) && (!z && !(n ^ v))) ||  
					((opcode ==BGTZ) && (!z && !(n ^ v))) ||
					((opcode == BLT) && (n ^ v)) ||
					((opcode == BLTZ) && (n ^ v)); 

					
	assign writeEn = ((state == 7) && branch) || pcWriteUncond;

endmodule


// ALU Module
module ALU (
    input [1:0] ALUop,                
    input signed [15:0] a, [15:0] b,         
    output zero, overflow, negative,  // Output flags
    output reg signed [15:0] result  
);

    reg [1:0] carry;                  // Carry for overflow detection

    assign zero = (result == 0);      
    assign negative = result[15];     // Negative flag (sign bit)
    assign overflow = carry[1] ^ carry[0];  

    always @(*) begin
        case (ALUop)
            2'b00 : result = a & b;  // AND operation
            2'b01 : begin            // ADD operation
                {carry[0], result[14:0]} = a[14:0] + b[14:0];
                {carry[1], result[15]} = a[15] + b[15] + carry[0];
            end
            2'b10 : result = a - b;  // SUB operation
            2'b11 : result = b - a;  // SUB operation (b - a)
            
            default: result = 16'h0000; // Default case
        endcase
    end
endmodule



// Register File Module
module reg_file (
    input clk,
    input regWrite,
    input [3:0] regDst, regSrc1, regSrc2,
    input [15:0] bus_w,
    output reg [15:0] out1, out2
);
    
    reg [15:0] regArray [0:7] = '{0, 5596, 7612, 10040,
                             4150, 5400, 16324, 8258};

    always @(*) begin // The output is taken asynchronously                                         
        out1 = regArray[regSrc1];
        out2 = regArray[regSrc2];    
    end
    
    always @(posedge clk) begin    
        if (regWrite) begin
            regArray[regDst] <= bus_w;
        end
    end
endmodule



// Instruction Memory Module
module instructionMemory #(
    parameter WIDTH = 16
) (
    output reg [15:0] data,
    input clk,
    input [WIDTH-1:0] address
);

    reg [15:0] mem [0:65535];  // Declare the memory array

    initial begin 
        $readmemh("mul.dat", mem,  0, 11);
    end

    always @(*) begin
        data = mem[address];
    end
endmodule


// Data Memory Module
module dataMemory #(
    parameter WIDTH = 256
) (
    output reg [15:0] dataOut,
    input clk,
    input [15:0] address,
    input [15:0] dataIn,
    input memRead, memWrite
);
	 
    reg [15:0] mem [0:WIDTH-1];

    initial begin 
        mem[80] = 8;
        mem[81] = 44;
        mem[82] = 432;
        mem[83] = -122;
        mem[84] = -200;
        mem[85] = -2;
        mem[86] = 211;
        mem[87] = 12;
    end
	
    always @(posedge clk) begin
        if (memWrite) 
            mem[address] <= dataIn;
    end
		
    always @(*) begin
        if (memRead)
            dataOut = mem[address];
    end	
endmodule



// Flop Module
module flop (  
    output reg [31:0] out,
    input clk,
    input writeEn,
    input [15:0] in,
    input reset
);
    always @(posedge clk or negedge reset) begin
        if (!reset)
            out <= 0;
        else if (writeEn)
            out <= in; 
    end
endmodule
 


// 2x1 MUX Module
module mux2x1 #(
    parameter WIDTH = 16
) ( 
    input [WIDTH-1:0] a, b,     
    input select,  
    output [WIDTH-1:0] out    
);
    assign out = select ? b : a;
endmodule


// 4x1 MUX Module
module mux4x1 #(
    parameter WIDTH = 16
) (
    input [WIDTH-1:0] a, b, c, d,  	 
    input [1:0] s,	 
    output reg [WIDTH-1:0] out
);

    always @* begin 
        case (s) 
            2'b00 : out = a;
            2'b01 : out = b;
            2'b10 : out = c;
            2'b11 : out = d;
        endcase
    end
endmodule
