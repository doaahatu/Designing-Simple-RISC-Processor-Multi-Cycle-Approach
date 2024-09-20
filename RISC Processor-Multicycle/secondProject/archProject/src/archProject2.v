// PROJECT 2_____________________________________________________

// Jana Qutosa 1210331
// Shiyar Dar Mousa 1210766
// Doaa Hatu 1211088

// ____________________________________________________Testbench____________________________________________________

module testbench;

    // Inputs
    reg clk;
    reg reset;
    reg [15:0] data_in;

    // Outputs
    wire [15:0] address_inst;
    wire [15:0] data_out_instruction;
    wire [15:0] data_out;
    wire [15:0] address_data;
    wire MemRead;
    wire MemWrite;
    wire [3:0] state;
    wire [15:0] ALUresult;
    wire [1:0] ALUop;

    // Instantiate the computer module
    computer uut (
        .clk(clk),
        .reset(reset),
        .address_inst(address_inst),
        .data_out_instruction(data_out_instruction),
        .data_in(data_in),
        .data_out(data_out),
        .address_data(address_data),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .state(state),
        .ALUresult(ALUresult),
        .ALUop(ALUop)
    );

    // Clock generation
    always
        #5 clk = ~clk;

    initial begin
        // Initialize Inputs
        clk = 0;
        reset = 1;
        data_in = 16'h0000;

        // Wait 100 ns for global reset to finish
        #100;
        
        // Deassert reset
        reset = 0;

        // Add stimulus here
        #10;
        data_in = 16'h1234; // Example data input
        #10;
        data_in = 16'h5678; // Another example data input

        // Wait for a while to observe the behavior
        #100;

        // Finish the simulation
        $finish;
    end

    initial begin
        // Monitor the signals
        $monitor("Time: %d, Reset: %b, Address Inst: %h, Data Out Inst: %h, Data In: %h, Data Out: %h, Address Data: %h, MemRead: %b, MemWrite: %b, State: %b, ALU Result: %h, ALU Op: %b", 
            $time, reset, address_inst, data_out_instruction, data_in, data_out, address_data, MemRead, MemWrite, state, ALUresult, ALUop);
    end

endmodule

// _____________________________________________________Immediate Extender Module _____________________________________________________    

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

//_____________________________________________________ Register File Module_____________________________________________________

module reg_file (
    input clk,
    input regWrite,
    input [2:0] regDst, regSrc1, regSrc2,
    input [15:0] bus_w,
    output reg [15:0] out1, out2
);
    
    reg [15:0] regArray [0:7] = '{0, 5596, 7612, 10040, 4150, 5400, 16324, 8258};

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

// _____________________________________________________Instruction Memory Module  _____________________________________________________

module instructionMemory #(
    parameter WIDTH = 16
) (
    output reg [15:0] data,
    input clk,
    input [WIDTH-1:0] address
);

    reg [15:0] mem [0:65535];  // Declare the memory array

    initial begin 
        $readmemh("mul.txt", mem,  0, 11);
    end

    always @(*) begin
        data = mem[address];
    end
endmodule

// _____________________________________________________Data Memory Module _____________________________________________________ 

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

// ____________________________________________________Flop Module____________________________________________________  

module flop (  
    output reg [15:0] out,
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

// ____________________________________________________PC Control____________________________________________________     

module PCcontrol (
    input [3:0] opcode, 
    input z, v, n,
    input pcWriteUncond,
    input [3:0] state,
    output writeEn
);
 
    parameter BNE = 4'b1011,
              BNEZ = 4'b1010;

    reg BranchCond;

    always @(*) begin
        case (opcode)
            BNE: BranchCond = (z == 1'b1);
            BNEZ: BranchCond = (z == 1'b0);
            default: BranchCond = 1'b0;
        endcase
    end

    assign writeEn = (state == 4'b0101 && (pcWriteUncond || BranchCond));
endmodule

// ____________________________________________________2x1 MUX Module ____________________________________________________      

module MUX2x1 (
    input [15:0] in1, in2,
    input select,
    output reg [15:0] muxOut
);
    
    always @(*) begin
        case(select)
            1'b0: muxOut = in1;
            1'b1: muxOut = in2;
        endcase
    end
endmodule

// ____________________________________________________4x1 MUX Module ____________________________________________________ 

module MUX4x1 (
    input [15:0] in1, in2, in3, in4,
    input [1:0] select,
    output reg [15:0] muxOut
);
    
    always @(*) begin
        case(select)
            2'b00: muxOut = in1;
            2'b01: muxOut = in2;
            2'b10: muxOut = in3;
            2'b11: muxOut = in4;
        endcase
    end
endmodule

// ____________________________________________________ALU Module ____________________________________________________ 

module ALU (
    input [15:0] in1, in2,
    input [2:0] aluControl,
    output reg [15:0] aluOut,
    output z, v, n
);

    reg overflow;

    always @(*) begin
        overflow = 1'b0;
        case (aluControl)
            3'b000: aluOut = in1 + in2;
            3'b001: aluOut = in1 - in2;
            3'b010: aluOut = in1 & in2;
            3'b011: aluOut = in1 | in2;
            3'b100: aluOut = ~in1;
            3'b101: aluOut = in1 ^ in2;
            3'b110: aluOut = in1 >> 1;
            3'b111: aluOut = in1 << 1;
            default: aluOut = 16'b0;
        endcase

        if (aluControl == 3'b000) begin
            overflow = (in1[15] & in2[15] & ~aluOut[15]) | (~in1[15] & ~in2[15] & aluOut[15]);
        end
    end

    assign z = (aluOut == 0) ? 1'b1 : 1'b0;
    assign v = overflow;
    assign n = aluOut[15];

endmodule

// ____________________________________________________ALU Control Module ____________________________________________________ 

module aluControl (
    input [1:0] aluOp,
    input [3:0] opcode,
    output reg [2:0] aluControl
);

    always @(*) begin
        case(aluOp)
            2'b00: aluControl = 3'b000;  // ADD
            2'b01: aluControl = 3'b001;  // SUB
            2'b10: begin
                case(opcode)
                    4'b0000: aluControl = 3'b000;  // ADD
                    4'b0001: aluControl = 3'b001;  // SUB
                    4'b0010: aluControl = 3'b010;  // AND
                    4'b0011: aluControl = 3'b011;  // OR
                    4'b0100: aluControl = 3'b100;  // NOT
                    4'b0101: aluControl = 3'b101;  // XOR
                    4'b0110: aluControl = 3'b110;  // SHR
                    4'b0111: aluControl = 3'b111;  // SHL
                    default: aluControl = 3'b000;
                endcase
            end
            default: aluControl = 3'b000;
        endcase
    end
endmodule

// ____________________________________________________Control Unit Module ____________________________________________________ 

module control (
    input clk,
    input reset,
    input [3:0] opcode,
    output regWrite, aluSrcA, IRwrite, memRead, memWrite, pcWriteUncond,
    output [1:0] aluSrcB, aluOp, pcSrc,
    output [3:0] state
);
    reg [3:0] next_state;
    reg [10:0] controls;

    parameter FETCH = 4'b0000,
              DECODE = 4'b0001,
              EXE_REG = 4'b0010,
              MEMADR = 4'b0011,
              LDR = 4'b0100,
              STR = 4'b0101,
              EXECUTE = 4'b0110,
              JUMP = 4'b0111,
              BRANCH = 4'b1000,
              HALT = 4'b1001,
              STR_REG = 4'b1010,
              LDR_REG = 4'b1011,
              JUMP_REG = 4'b1100,
              HALT_REG = 4'b1101;

    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            next_state <= FETCH;
        end else begin
            next_state <= state;
        end
    end

    always @(*) begin
        case (state)
            FETCH: controls = 11'b00101000001;
            DECODE: controls = 11'b00000001000;
            EXE_REG: controls = 11'b10010010000;
            MEMADR: controls = 11'b01010000000;
            LDR: controls = 11'b00010001000;
            STR: controls = 11'b00000000110;
            EXECUTE: controls = 11'b00000000000;
            JUMP: controls = 11'b00000000010;
            BRANCH: controls = 11'b00000000000;
            HALT: controls = 11'b00000000000;
            STR_REG: controls = 11'b00000000000;
            LDR_REG: controls = 11'b00000000000;
            JUMP_REG: controls = 11'b00000000000;
            HALT_REG: controls = 11'b00000000000;
            default: controls = 11'b00000000000;
        endcase
    end

    assign {regWrite, aluSrcA, aluSrcB, IRwrite, memRead, memWrite, pcWriteUncond, aluOp, pcSrc} = controls;
    assign state = next_state;
endmodule

// ____________________________________________________CPU Module ____________________________________________________ 

module cpu (
    input clk,
    input reset,
    output [15:0] address_inst,
    output [15:0] data_out_instruction,
    input [15:0] data_in,
    output [15:0] data_out,
    output [15:0] address_data,
    output MemRead,
    output MemWrite,
    output [3:0] state,
    output [15:0] ALUresult,
    output [1:0] ALUop
);

    wire [15:0] pc;
    wire pcWrite, zero, overflow, negative;
    wire [2:0] aluControl;
    wire [15:0] aluOut, aluSrcA_out, aluSrcB_out;
    wire [15:0] regData1, regData2, writeData;
    wire regWrite, aluSrcA, aluSrcB, IRwrite, memRead, memWrite, pcWriteUncond;
    wire [1:0] aluOp, pcSrc;

    // Instantiate Instruction Memory
    instructionMemory instrMem (
        .data(data_out_instruction),
        .clk(clk),
        .address(pc)
    );

    // Instantiate Data Memory
    dataMemory dataMem (
        .dataOut(data_out),
        .clk(clk),
        .address(address_data),
        .dataIn(data_in),
        .memRead(MemRead),
        .memWrite(MemWrite)
    );

    // Instantiate Register File
    reg_file regFile (
        .clk(clk),
        .regWrite(regWrite),
        .regDst(data_out_instruction[11:9]),
        .regSrc1(data_out_instruction[8:6]),
        .regSrc2(data_out_instruction[5:3]),
        .bus_w(writeData),
        .out1(regData1),
        .out2(regData2)
    );

    // Instantiate Control Unit
    control ctrlUnit (
        .clk(clk),
        .reset(reset),
        .opcode(data_out_instruction[15:12]),
        .regWrite(regWrite),
        .aluSrcA(aluSrcA),
        .IRwrite(IRwrite),
        .memRead(MemRead),
        .memWrite(MemWrite),
        .pcWriteUncond(pcWriteUncond),
        .aluSrcB(aluSrcB),
        .aluOp(aluOp),
        .pcSrc(pcSrc),
        .state(state)
    );

    // Instantiate ALU Control Unit
    aluControl aluCtrl (
        .aluOp(aluOp),
        .opcode(data_out_instruction[15:12]),
        .aluControl(aluControl)
    );

    // Instantiate ALU
    ALU alu (
        .in1(aluSrcA_out),
        .in2(aluSrcB_out),
        .aluControl(aluControl),
        .aluOut(ALUresult),
        .z(zero),
        .v(overflow),
        .n(negative)
    );

    // Instantiate PC Write Logic
    pcWriteLogic pcWriteLogic (
        .z(zero),
        .opcode(data_out_instruction[15:12]),
        .state(state),
        .writeEn(pcWrite)
    );

    // Instantiate Program Counter
    PC programCounter (
        .clk(clk),
        .reset(reset),
        .writeEn(pcWrite),
        .pcIn(pc),
        .pcOut(pc)
    );

    // Instantiate 2x1 Multiplexer for ALU Source A
    MUX2x1 aluSrcA_mux (
        .in1(regData1),
        .in2(pc),
        .select(aluSrcA),
        .muxOut(aluSrcA_out)
    );

    // Instantiate 4x1 Multiplexer for ALU Source B
    MUX4x1 aluSrcB_mux (
        .in1(regData2),
        .in2(16'b1),
        .in3(data_out_instruction[5:0]),
        .in4(data_out_instruction[11:0]),
        .select(aluSrcB),
        .muxOut(aluSrcB_out)
    );

    // Instantiate 4x1 Multiplexer for Program Counter Source
    MUX4x1 pcSrc_mux (
        .in1(ALUresult),
        .in2(aluOut),
        .in3({pc[15:12], data_out_instruction[11:0]}),
        .in4(16'b0),
        .select(pcSrc),
        .muxOut(pc)
    );

endmodule
