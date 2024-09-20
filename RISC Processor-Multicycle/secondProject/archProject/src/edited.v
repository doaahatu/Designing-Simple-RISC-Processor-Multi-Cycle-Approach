//PROJECT 2___________________

// Jana Qutosa 1210331
// Shiyar Dar Mousa 1210766
// Doaa Hatu 1211088

// __________________Immediate Extender Module __________________	
	
module extender (
    input [8:0] immediate,  // Combined input for both S-type and I-type immediates
    input sign_ext,
    input select_type,  // 1 for S-type, 0 for I-type
    output reg [15:0] out
);

    always @* begin
        if (select_type) begin
            // S-type immediate extension (9-bit)
            if (sign_ext && immediate[8])
                out = {7'h7F, immediate};  // Sign-extend 9-bit to 16-bit
            else
                out = {7'b0, immediate};   // Zero-extend 9-bit to 16-bit
        end else begin
            // I-type immediate extension (5-bit)
            if (sign_ext && immediate[4])
                out = {11'h7FF, immediate[4:0]}; // Sign-extend 5-bit to 16-bit
            else
                out = {11'b0, immediate[4:0]};   // Zero-extend 5-bit to 16-bit
        end
    end
endmodule				


// __________________Second Immediate Extender Module __________________	
	
module extender2 (
    input [15:0] memOut, 
    input selection,
    input enable,  // enabling 
    output reg [15:0] out2
);

    always @* begin
        if (enable) begin
            if(selection)
				out2 = {{8{memOut[7]}}, memOut[7:0]};
			else
			    out2 = {{8{1'b0}}, memOut[7:0]};	
        end	else begin
		     out2 = memOut[15:0];
    end	
	end
	
endmodule	


//__________________ Register File Module__________________

module reg_file (																												 
    input clk,
    input regWrite,
    input [2:0] regDst, regSrc1, regSrc2,
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







// __________________Instruction Memory Module __________________

module instructionMemory #(parameter WIDTH = 16) (
    output reg [15:0] data,
    input [WIDTH-1:0] address
);

    reg [7:0] mem [0:131071];  // 2 bytes per 16-bit instruction

    initial begin
        mem[0] = 8'h98;
        mem[1] = 8'h12;
        mem[2] = 8'h04;
        mem[3] = 8'h32;	
		
    end

    always @(*) begin
        data = {mem[address + 1], mem[address]};  // Combine two bytes to form a 16-bit instruction
    end
endmodule




// __________________Data Memory Module	 __________________ 
	
module dataMemory #(parameter WIDTH = 256) (
    output reg [15:0] dataOut,
    input clk,
    input [15:0] address,
    input [15:0] dataIn,
    input memRead, memWrite
);
	 
    reg [7:0] mem [0:WIDTH*2-1];  // Byte addressable, each 16-bit data uses 2 addresses

    initial begin 
        mem[160] = 8;
        mem[161] = 0;
        mem[162] = 44;
        mem[163] = 0;
        mem[164] = 176;  // Example data adjusted to 8-bit
        mem[165] = 1;    // 432 split into two bytes: 0x01B0
        mem[166] = 134;  // -122 split into two bytes: 0xFF86
        mem[167] = 255;
        mem[168] = 56;   // -200 split into two bytes: 0xFF38
        mem[169] = 255;
        mem[170] = 254;  // -2 split into two bytes: 0xFFFE
        mem[171] = 255;
        mem[172] = 211;  // 211 split into two bytes: 0x00D3
        mem[173] = 0;
        mem[174] = 12;
        mem[175] = 0;
    end
	
    always @(posedge clk) begin
        if (memWrite) begin
            mem[address + 1] <= dataIn[15:8];  // High byte
            mem[address] <= dataIn[7:0];  // Low byte
        end
    end
		
    always @(*) begin
        if (memRead)
            dataOut = {mem[address + 1], mem[address]};
    end	
endmodule




// _____________Flop Module_____________  

module flop (  
    output reg [15:0] out,
    input clk,
    input writeEn,
	input reset,
    input [15:0] in
);
    always @(posedge clk or negedge reset) begin
		
		if (!reset)
			out <= 0;
		
		else if (writeEn)
            out <= in; 
    end
endmodule		   


// _____________PC Control_____________     

module PCcontrol (
    input [3:0] opcode, 
    input z, v, n,
    input pcWriteUncond,
    input [3:0] state,
    output  writeEn
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
                    ((opcode == BNEZ) && !z) ||
                    ((opcode == BEQ) && z) || 
                    ((opcode == BEQZ) && z) ||
                    ((opcode == BGT) && (!z && !(n ^ v))) ||  
                    ((opcode == BGTZ) && (!z && !(n ^ v))) ||
                    ((opcode == BLT) && (n ^ v)) ||
                    ((opcode == BLTZ) && (n ^ v)); 

     assign writeEn = pcWriteUncond || ((state == 10) && branch);
    

endmodule



// __________________ Clock Module__________________


module clock(output reg clock);
	
	
initial begin 
	clock=0; 
    $display("The clock started at %0.t", $time);
end

 
always #5 begin
    clock=~clock;
	// $display("The clock turned at %0.t, %b", $time, clock);
end
					   

endmodule



// __________________ 2x1 MUX Module__________________

module mux2x1 #(
    parameter WIDTH = 16
) ( 
    input [WIDTH-1:0] a, b,     
    input select,  
    output [WIDTH-1:0] out    
);
    assign out = select ? b : a;
endmodule																					   




//__________________ 4x1 MUX Module __________________
	
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






//__________________ ALU Module __________________
	
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






//__________________ ALU Control module__________________

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







// ___________________ Control Unit Module ___________________

module controlUnit (
    input [3:0] opcode,
    input mode,
    input clk, 
    input rst,
    output reg regWrite, aluSrc2, memRead, memWrite, IRwrite, regSrc2, regDest,
    output reg [1:0] aluControl,
    output reg sign_ext, address_selection, data_in_selection, select_type, PCwriteUncond, EN, SEL,
    output reg [1:0] regSrc1, PC_selection, memReg,
    output reg [3:0] state
);

    // Define the state parameters
    parameter InstructionFetch = 0,    
              InstructionDecode = 1,    
              AddressComputation = 2,
              LoadAccess = 3,            
              StoreAccess = 4, 
              ALUcomputationI = 5,
              ALUcomputationR = 6,    
              LBuResultStore = 7,
              LBsResultStore = 8,
              ALUResultStore = 9,
              BranchCompletion = 10,
              SetValue = 11,
              Call = 12,                
              Return = 13;
              
    reg jmp;
    assign jmp = opcode == 4'b1100;
    
    always @(posedge clk or negedge rst) begin
        if (!rst)
            state <= InstructionFetch;   
        else begin
            $display("** Control Unit %0t: state = %d", $time, state);
            case (state)
                InstructionFetch: state <= InstructionDecode;
                InstructionDecode: begin
                    case (opcode)
                        4'b0000, 4'b0001, 4'b0010: state <= ALUcomputationR; // AND, ADD, SUB
                        4'b0011, 4'b0100: state <= ALUcomputationI; // ADDI, ANDI
                        4'b0101, 4'b0110: state <= AddressComputation; // LW, LBu, LBs
                        4'b0111: state <= StoreAccess; // SW
                        4'b1000, 4'b1001, 4'b1010, 4'b1011: state <= BranchCompletion; // BGT, BLT, BEQ, BNE
                        4'b1101: state <= Call; // CALL
                        4'b1110: state <= Return; // RET
                        4'b1111: state <= SetValue; // SV
                        default: state <= InstructionFetch;
                    endcase
                end
                AddressComputation: begin
                    if (opcode == 4'b0101) // LW
                        state <= LoadAccess;
                    else if (opcode == 4'b0110) // LBu or LBs
                        state <= (mode == 1'b0) ? LBuResultStore : LBsResultStore;
                    else
                        state <= InstructionFetch;
                end
                LoadAccess: state <= InstructionFetch;
                StoreAccess: state <= InstructionFetch;
                ALUcomputationR: state <= ALUResultStore;
                ALUcomputationI: state <= ALUResultStore;
                LBuResultStore: state <= InstructionFetch;
                LBsResultStore: state <= InstructionFetch;
                ALUResultStore: state <= InstructionFetch;
                BranchCompletion: state <= InstructionFetch;
                Call: state <= InstructionFetch;
                Return: state <= InstructionFetch;
                SetValue: state <= InstructionFetch;
                default: state <= InstructionFetch;
            endcase
        end
    end

    always @(*) begin
        // Default values to prevent latches
        regWrite = 0;
        aluSrc2 = 0;
        memRead = 0;
        memWrite = 0;
        memReg = 2'b00;
        PC_selection = 2'b11;
        sign_ext = 0;
        address_selection = 0;
        data_in_selection = 0;
        aluControl = 2'b00;
        regSrc2 = 0;
        regDest = 0;
        regSrc1 = 2'b00;
        IRwrite = 0;   
        PCwriteUncond = 0;

        case (state)
            InstructionFetch: begin
                IRwrite = 1;
                PC_selection = 2'b11;
                PCwriteUncond = 1;
                if (!regSrc2) begin
                    // Determine the first source register for branches
                    if (opcode == 4'b1000 || opcode == 4'b1001 || opcode == 4'b1010 || opcode == 4'b1011) begin
                        regSrc1 = (mode == 0) ? 2'b01 : 2'b00;
                    end
                    if (opcode == 4'b1110) begin
                        // Fetch R7 for RET
                        regSrc1 = 2'b10; 
                    end  
                    if (opcode == 4'b0000 || opcode == 4'b0001 || opcode == 4'b0010) begin 
                        // R-type
                        regSrc2 = 1;
                    end
                end
            end
            InstructionDecode: begin
                sign_ext = 1;
                PCwriteUncond = jmp;
            end
            AddressComputation: begin
                aluControl = 2'b01; // ADD to calculate address
                address_selection = 1;   
                regSrc1 = 2'b00;
                regSrc2 = 0;
            end
            LoadAccess: begin
                memRead = 1;
                memWrite = 0;
                regWrite = 1;
                memReg = 2'b01;
				EN = 0;
            end
            LBuResultStore: begin
                memRead = 1;
                regWrite = 1;
                memReg = 2'b10;
                sign_ext = 0; 
				EN = 1;	 
				SEL= 0;
            end
            LBsResultStore: begin
                memRead = 1;
                regWrite = 1;
                memReg = 2'b10;
                sign_ext = 1; 
				EN = 1;	
				SEL= 1;
            end
            StoreAccess: begin
                memWrite = 1;  
                memRead = 0;
                address_selection = 1;
            end
            ALUcomputationR: begin
                regWrite = 1;
                regDest = 0;
                aluSrc2 = 0;
            end
            ALUcomputationI: begin
                aluSrc2 = 1;
                regWrite = 1;
                sign_ext = 0;
                select_type = 0;
            end
            ALUResultStore: begin
                regWrite = 1;
                regSrc2 = 1; 
                memReg = 2'b10;
            end
            BranchCompletion: begin
                aluControl = 2'b11; // SUB for comparison
                sign_ext = 1;
                select_type = 0;
                PC_selection = 2'b01;
            end
            Call: begin
                PC_selection = 2'b01;  
                PCwriteUncond = 1;
            end
            Return: begin
                PC_selection = 2'b10;
                regWrite = 1;
                regDest = 1; 
                PCwriteUncond = 1;
            end
            SetValue: begin
                regWrite = 0;
                aluControl = 2'b01; 
                PC_selection = 2'b11;
                aluSrc2 = 1;           
                memWrite = 1; 
                sign_ext = 1; 
                select_type = 1;
                data_in_selection = 1;
            end
            default: begin
                IRwrite = 0;
                PC_selection = 2'b11;
                regWrite = 0;
                memRead = 0;
                memWrite = 0;
            end
        endcase
    end

endmodule



// __________________ CPU module __________________

module CPU (
    input clk, 
    input reset,
    output [15:0] address_instruction,
    output [15:0] data_out_instruction,
    input [15:0] data_in,
    output [15:0] data_out,out2,
    output [15:0] address_data,
    output MemWrite,
    output MemRead,
    output [3:0] state,
    output [15:0] ALUresult,
    output [1:0] ALUop
);

    // Internal signals
    wire [15:0] instruction, regOut1, regOut2, aluResult, memDataOut, extendedImmediate;
    wire [15:0] muxAluSrc2Out, muxMemRegOut, pc, nextPC, branchAddr, jumpAddr;
    wire [3:0] opcode;
    wire regDst, regSrc2;
    wire [1:0] aluControl, regSrcSel1, pcSel, memReg, regSrc1;
    wire regWrite, aluSrc2, memRead, memWrite, IRwrite, regDest, PCwrite;
    wire sign_ext, address_selection, data_in_selection, select_type, PCwriteUncond,EN,SEL;
    wire zero, overflow, negative, pcWriteEn, mode;

    // Control Unit
    controlUnit control (
        .opcode(opcode),
        .mode(mode),
        .clk(clk),
        .rst(reset),
        .regWrite(regWrite),
        .aluSrc2(aluSrc2),
        .memRead(memRead),
        .memWrite(memWrite),
        .IRwrite(IRwrite),
        .regSrc2(regSrc2),
        .regDest(regDest),
        .aluControl(aluControl),
        .sign_ext(sign_ext),
        .address_selection(address_selection),
        .data_in_selection(data_in_selection),
        .select_type(select_type),
        .PCwriteUncond(PCwriteUncond),
        .regSrc1(regSrcSel1),
        .PC_selection(pcSel),
        .memReg(memReg),
        .state(state),
		.EN(EN),
		.SEL(SEL)
    );

    // Datapath
    datapath dp (
        .data_in_selection(data_in_selection),
        .RegSrc2(regSrc2),
        .RegWrite(regWrite),
        .IRwrite(IRwrite),
        .PCwrite(PCwrite),
        .sign_ext(sign_ext),
        .clk(clk), 
        .RegDest(regDest),
        .ALUsrc2(aluSrc2),     
        .data_out_instruction(data_out_instruction),
        .data_out(data_out),                  
        .MemReg(memReg),
        .PC_selection(pcSel), 
        .ALUop(ALUop),
        .RegSrc1(regSrc1),
        .opcode(opcode), 
        .mode(mode),
        .address_data(address_data),
        .address_instruction(address_instruction),
        .data_in(data_in),
        .z(zero),
        .n(negative),
        .v(overflow),
        .ALU_result(aluResult),
        .address_selection(address_selection),
        .reset(reset),
		.EN(EN),
		.SEL(SEL),
		.out2(out2)
    );      
    
    // ALU Control
    ALUcontrol aluCtrl (
        .ALUctrl(aluControl),
        .opcode(opcode),
        .ALUop(ALUop)
    );

    // PC Control
    PCcontrol pcCtrl (
        .opcode(opcode),
        .z(zero),
        .v(overflow),
        .n(negative),
        .pcWriteUncond(PCwriteUncond),
        .state(state),
        .writeEn(PCwrite)
    );

endmodule
	



	
// __________________ Computer module  __________________
	
module computer (
    input clk,
    input reset,
);				

	wire [15:0] address_inst; 
    wire [15:0] data_out_instruction;
    wire [15:0] data_out; 
	wire [15:0] out2;
    wire [15:0] address_data;
    wire MemRead;
    wire MemWrite;
    wire [3:0] state;
    wire [15:0] ALUresult;
    wire [1:0] ALUop;
	wire [15:0] data_in;


	

      instructionMemory im (
		.address(address_inst),
		.data(data_out_instruction)		
    );


      dataMemory dm (
        .dataOut(data_out),
        .clk(clk),
        .address(address_data),
        .dataIn(data_in),
        .memRead(MemRead),
        .memWrite(MemWrite)
    );


      CPU cpu(
	
        .data_out(data_out),
        .out2(out2),
		.data_out_instruction(data_out_instruction), 
        .clk(clk),
		.reset(reset),
        .data_in(data_in),
        .address_data(address_data),
        .address_instruction(address_inst),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
		.state(state),
		.ALUresult(ALUresult),
		.ALUop(ALUop)
    );

	

endmodule

	




//__________________  Datapath Module  __________________
	
module datapath (
    input data_in_selection, RegSrc2,
    RegWrite, IRwrite, PCwrite,	reset,
    sign_ext , clk, RegDest, ALUsrc2, address_selection,SEL,EN,
    input [15:0] data_out_instruction, data_out, out2,
    input [1:0] MemReg, PC_selection, ALUop, RegSrc1,
    output [3:0] opcode, 
    output mode,
    output [15:0] address_data, address_instruction, data_in,
    output z, n, v,
    output [15:0] ALU_result
);

    wire [2:0] Rs1, Rs2, Rd; 
    wire [14:0] upper_bits;

    wire [15:0] pc_temp;
    //wire [15:0] temp_increment;

    wire [15:0] instruction, BUS1, BUS2, B_operand, A_operand,  
                 extended_immediate, ALU_operand1, ALU_operand2,
                 jumpTargetAddress, PCtype, ALU_result_buffer , incrementPC, BTA;

    wire [15:0] MDR_out, Register_result;                            
    wire [8:0] immediate_combined;
    
    wire [2:0] Rs1_regfile;    //mux output 
    wire [2:0] Rs2_regfile;
    wire [2:0] Rd_regfile;
	wire [15:0] nextPC;

    // Extract the upper 15 bits of the address_instruction
    assign upper_bits = address_instruction[15:1];
    assign pc_temp = address_instruction;    
    // Combine the incremented upper 15 bits with the original LSB
    assign BTA = pc_temp + extended_immediate;
    assign opcode = instruction[15:12];
    assign mode = instruction[5];
    assign Rd = instruction[11:9];
    assign Rs1 = instruction[8:6];
    assign Rs2 = instruction[5:3];
	
	assign incrementPC = address_instruction + 2;
    
    assign immediate_combined = {instruction[8:0]};
   
    // Perform the left shift (multiply by 2) and concatenation
    assign jumpTargetAddress = {address_instruction[15:12], instruction[11:1] << 1};  
    
    flop IR (.out(instruction), .clk(clk), .writeEn(IRwrite), .in(data_out_instruction), .reset(reset));    
    
    flop A (.out(A_operand), .clk(clk), .writeEn(1'b1), .in(BUS1), .reset(reset));  
    
    flop B (.out(B_operand), .clk(clk), .writeEn(1'b1), .in(BUS2), .reset(reset)); 
    
    flop pc(.out(address_instruction), .clk(clk), .writeEn(PCwrite), .in(incrementPC), .reset(reset));  
	
	flop ALUout(.out(ALU_result_buffer), .clk(clk), .writeEn(1'b1), .in(ALU_result), .reset(reset)); 
	
	flop MDR(.out(MDR_out), .clk(clk), .writeEn(1'b1), .in(data_out), .reset(reset));
    
    // Register file instantiation
    reg_file rf (
        .clk(clk),
        .regWrite(RegWrite),
        .regDst(Rd_regfile),
        .regSrc1(Rs1_regfile),
        .regSrc2(Rs2_regfile),
        .bus_w(Register_result),
        .out1(BUS1),
        .out2(BUS2)
    );

    // Extender instantiation
    extender ext (
        .immediate(immediate_combined),
        .sign_ext(sign_ext),
        .select_type(opcode[3]),  // Assume select_type is based on opcode MSB
        .out(extended_immediate)
    );

    // ALU instantiation
    ALU alu (
        .ALUop(ALUop),
        .a(A_operand),
        .b(B_operand),
        .zero(z),
        .overflow(v),
        .negative(n),
        .result(ALU_result)
    );

	 mux4x1 #(3) RS1_mux (.a(Rs1), .b(3'b0), .c(3'b1), .d(3'b0), .s(RegSrc1), .out(Rs1_regfile));
	 mux2x1 #(3) RS2_mux (.a(Rd), .b(Rs2),  .select(RegSrc2), .out(Rs2_regfile));
	 mux2x1 #(3) Rd_mux (.a(Rd), .b(3'b1),  .select(RegDest), .out(Rd_regfile));
	 mux2x1 ALUsrc2_mux (.a(B_operand), .b(extended_immediate),  .select(ALUsrc2), .out(ALU_operand2));	
	 mux2x1 address_mem_mux (.a(A_operand), .b(ALU_result_buffer),  .select(address_selection), .out(address_data));
	 mux2x1 data_in_mux (.a(B_operand), .b(extended_immediate),  .select(data_in_selection), .out(data_in));
	 mux4x1 write_mux (.a(incrementPC), .b(MDR_out), .c(ALU_result_buffer), .d(16'b0), .s(MemReg), .out(Register_result));	 
	 mux4x1 PC_mux (.a(A_operand), .b(BTA), .c(jumpTargetAddress), .d(incrementPC), .s(PC_selection), .out(PCtype));
	 extender ext1(.immediate(immediate_combined), .sign_ext(sign_ext),.select_type(1'b1), .out(extended_immediate));
	 extender ext2(.immediate(immediate_combined), .sign_ext(sign_ext),.select_type(1'b0), .out(extended_immediate)); 
	 extender2 ext3(.memOut(data_out), .selection(SEL), .enable(EN), .out2(out2)); 
	  
endmodule	



// _____________Testbench_____________

`timescale 1ns / 1ps

module testbench;

    // Parameters
    parameter CLK_PERIOD = 10; // Clock period in ns
    parameter SIM_TIME = 500; // Simulation time in ns

    // Signals for the test bench
    reg clk;
    reg reset;
    reg [15:0] data_in;
    reg memRead, memWrite;
    reg [15:0] data_out_instruction;
    reg [15:0] data_out, out2;
    reg [15:0] address_data;
    reg [3:0] state;
    reg [15:0] ALUresult;
    reg [1:0] ALUop;

    // Instantiate the computer module
     computer uut (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation
    always begin
        clk = 0;
        #((CLK_PERIOD / 2)); // Adjusted delay calculation
        clk = 1;
        #((CLK_PERIOD / 2)); // Adjusted delay calculation
    end

    // Reset initialization
    initial begin
        $dumpfile("computer_tb.vcd");
        $dumpvars(0, testbench);

        reset = 1;
        memRead = 0;
        memWrite = 0;
        data_in = 16'h0000;
        #50;
        reset = 0; // Release reset

        // Test scenario 1: Basic operation
        $display("Starting Test Scenario 1: Basic Operation");
        data_in = 16'h1234;
        memRead = 1;
        #100; 

        // Test scenario 2: Memory write operation
        $display("Starting Test Scenario 2: Memory Write");
        data_in = 16'h5678;
        memWrite = 1;
        #100; 

        // Test scenario 3: ALU operation
        $display("Starting Test Scenario 3: ALU Operation");
        data_in = 16'hABCD;
        ALUop = 2'b01; // Example ALU operation code
        #100; 

        // Test scenario 4: State transition testing
        $display("Starting Test Scenario 4: State Transition Testing");
        data_in = 16'hFFFF;
        #50; 
        data_in = 16'h0000;
        #50; 

        // End simulation
        #100;
        $display("Simulation finished at %t", $time);
        $finish;
    end

endmodule


