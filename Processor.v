/*
MY-P0 Processor: RISC ISA, 32-bit, Interrupts Enabled, Word Addressable, 3-stage Processor
*/

module micro_controller 
    (
        input   clk, rst, OnInt,
        input   [1:0] CmpOut,
        input   [3:0] Rx, Ry, Rz, Opcode,
        output  DrREG, DrMEM, DrALU, DrPC, DrOFF, LdPC, LdIR, LdMAR, LdA, LdB, LdCmp, WrREG, 
                WrMEM, OPTest, ChkCmp, LdEnInt, IntAck, DrData, LdDAR,
        output  [1:0] RegSel,
        output  [2:0] ALUFunc,
        output  [3:0] RegNo
    );

    reg [5:0] State;
    reg [5:0] SEQ_ROM [0: 2**4 - 1];
    reg [5:0] CC_ROM [0: 2**2 - 1];
    reg [5:0] INT_ROM [0: 2**1 - 1];
    reg [29:0] Main_ROM [0: 2**6 - 1];

    reg [5:0] NextState;

    initial begin
        State = 6'b000000;

        CC_ROM[0] = 6'h00;
        CC_ROM[1] = 6'h1A;
        CC_ROM[2] = 6'h20;
        CC_ROM[3] = 6'h21;

        INT_ROM[0] = 6'h01;
        INT_ROM[1] = 6'h2A;

        SEQ_ROM[0] = 6'h03;
        SEQ_ROM[1] = 6'h06;
        SEQ_ROM[2] = 6'h09;
        SEQ_ROM[3] = 6'h0B;
        SEQ_ROM[4] = 6'h0F;
        SEQ_ROM[5] = 6'h16;
        SEQ_ROM[6] = 6'h13;
        SEQ_ROM[7] = 6'h15;
        SEQ_ROM[8] = 6'h16;
        SEQ_ROM[9] = 6'h23;
        SEQ_ROM[10] = 6'h1D;
        SEQ_ROM[11] = 6'h1E;
        SEQ_ROM[12] = 6'h26;
        SEQ_ROM[13] = 6'h2F;
        SEQ_ROM[14] = 6'h32;
        SEQ_ROM[15] = 6'h30;

        Main_ROM[0] = 30'h3006201;
        Main_ROM[1] = 30'h1082;
        Main_ROM[2] = 30'h1600900;
        Main_ROM[3] = 30'h84044;
        Main_ROM[4] = 30'h108045;
        Main_ROM[5] = 30'h20100;
        Main_ROM[6] = 30'h84047;
        Main_ROM[7] = 30'h108048;
        Main_ROM[8] = 30'h420100;
        Main_ROM[9] = 30'h0008404A;
        Main_ROM[10] = 30'h8405;
        Main_ROM[11] = 30'h0008404C;
        Main_ROM[12] = 30'h0000840D;
        Main_ROM[13] = 30'h0000210E;
        Main_ROM[14] = 30'h20080;
        Main_ROM[15] = 30'h4410;
        Main_ROM[16] = 30'h88051;
        Main_ROM[17] = 30'h2112;
        Main_ROM[18] = 30'h40040;
        Main_ROM[19] = 30'h000A0214;
        Main_ROM[20] = 30'h840;
        Main_ROM[21] = 30'h15;
        Main_ROM[22] = 30'h4057;
        Main_ROM[23] = 30'h88058;
        Main_ROM[24] = 30'h210119;
        Main_ROM[25] = 30'h2000000;
        Main_ROM[26] = 30'h0000421B;
        Main_ROM[27] = 30'h0000841C;
        Main_ROM[28] = 30'h900;
        Main_ROM[29] = 30'h0081005F;
        Main_ROM[30] = 30'h0089005F;
        Main_ROM[31] = 30'h2000000;
        Main_ROM[32] = 30'h84062;
        Main_ROM[33] = 30'h104062;
        Main_ROM[34] = 30'h820100;
        Main_ROM[35] = 30'h4224;
        Main_ROM[36] = 30'h8425;
        Main_ROM[37] = 30'h20100;
        Main_ROM[38] = 30'h4067;
        Main_ROM[39] = 30'h88068;
        Main_ROM[40] = 30'h008A0129;
        Main_ROM[41] = 30'h00A20100;
        Main_ROM[42] = 30'h0C1A022B;
        Main_ROM[43] = 30'h1000202C;
        Main_ROM[44] = 30'h880;
        Main_ROM[45] = 30'h0;
        Main_ROM[46] = 30'h0;
        Main_ROM[47] = 30'h4000000;
        Main_ROM[48] = 30'h180871;
        Main_ROM[49] = 30'h4000000;
        Main_ROM[50] = 30'h20000433;
        Main_ROM[51] = 30'h10020034;
        Main_ROM[52] = 30'h20080040;
    end

    always @(posedge clk)
        begin
            //if (rst) State <= 6'b000000;
            //else State <= NextState;
            State <= NextState;
        end

    always @(*)
        begin
            case({ChkCmp, OPTest})
                2'b00: NextState = Main_ROM[State][5:0];
                2'b01: NextState = SEQ_ROM[Opcode][5:0];
                2'b10: NextState = CC_ROM[CmpOut][5:0];
                2'b11: NextState = INT_ROM[OnInt][5:0];
                default: NextState = 6'bZZZZZZ;
            endcase

            DrREG <= Main_ROM[State][6];
            DrMEM <= Main_ROM[State][7];
            DrALU <= Main_ROM[State][8];
            DrPC <= Main_ROM[State][9];
            DrOFF <= Main_ROM[State][10];
            LdPC <= Main_ROM[State][11];
            LdIR <= Main_ROM[State][12];
            LdMAR <= Main_ROM[State][13];
            LdA <= Main_ROM[State][14];
            LdB <= Main_ROM[State][15];
            LdCmp <= Main_ROM[State][16];
            WrREG <= Main_ROM[State][17];
            WrMEM <= Main_ROM[State][18];
            RegSel <= Main_ROM[State][20:19];
            ALUFunc <= Main_ROM[State][23:21];
            OPTest <= Main_ROM[State][24];
            ChkCmp <= Main_ROM[State][25];
            LdEnInt <= Main_ROM[State][26];
            IntAck <= Main_ROM[State][27];
            DrData <= Main_ROM[State][28];
            LdDAR <= Main_ROM[State][29];

            case(RegSel)
                2'b00: RegNo = Rx;
                2'b01: RegNo = Ry;
                2'b10: RegNo = Rz;
                2'b11: RegNo = 4'b1100;
                default: RegNo = 4'bZZZZ;
            endcase
        end

endmodule

module ram (input [15:0] MAR, input clk, input [31:0] Din, input WrMem, output [31:0] Dout);

    reg [31:0] memory [0: 2**16 - 1];

    always @(posedge clk)
        begin
            if (WrMem) memory[MAR] <= Din;
        end
    
    always @(*)
        begin
            Dout = memory[MAR];
        end

endmodule

module alu (input [31:0] A, B, input [2:0] func, output reg [31:0] out);

    always @(*)
        begin
            case(func)
                3'b000: out = A + B; // ADD
                3'b001: out = A - B; // SUB
                3'b010: out = A ~& B; // NAND
                3'b011: out = A + 1; // A + 1
                3'b100: out = A; // PASS A
                3'b101: out = B; // PASS B
                default: out = 32'hZZZZ_ZZZZ; // Undefined
            endcase
        end

endmodule

module regfile
    (
        input clk,
        input rst,
        input WrReg,
        input [3:0] RegSel,
        input [31:0] Din,
        output [31:0] Dout
    );

    reg [31:0] Registers [0: 15];

    initial begin
        Registers[0] <= 32'h00000000;
        Registers[1] <= 32'h00000000;
        Registers[2] <= 32'h00000000;
        Registers[3] <= 32'h00000000;
        Registers[4] <= 32'h00000000;
        Registers[5] <= 32'h00000000;
        Registers[6] <= 32'h00000000;
        Registers[7] <= 32'h00000000;
		Registers[8] <= 32'h00000000;
		Registers[9] <= 32'h00000000;
		Registers[10] <= 32'h00000000;
		Registers[11] <= 32'h00000000;
		Registers[12] <= 32'h00000000;
		Registers[13] <= 32'h00000000;
		Registers[14] <= 32'h00000000;
		Registers[15] <= 32'h00000000;
    end

    always @(*) 
        begin
            Dout = Registers[RegSel];
        end

    integer i;
    always @(posedge clk)
        begin
            if (rst) // Reset
                for (i = 0; i < 16; i = i + 1) begin
                    Registers[i] <= 32'h00000000;
                end
            else begin
                if (WrReg) Registers[RegSel] <= Din;
            end
        end

endmodule

module data_path (input clk, rst);
    //ram ram1();
    //regfile4x16 regfile1();
    //alu alu1();
    //micro_controller micro_controller1();

    reg [31:0] PC, IR, MAR, A, B, DAR;
    reg [1:0] Cmp;
    reg IE;

    wire DrREG, DrMEM, DrALU, DrPC, DrOFF, LdPC, LdIR, LdMAR, LdA, LdB, LdCmp, WrREG, 
         WrMEM, OPTest, ChkCmp, LdEnInt, IntAck, DrData, LdDAR;
    wire [1:0] RegSel;
    wire [2:0] ALUFunc;
    wire [3:0] RegNo;

    micro_controller micro_controller1(
        .clk(clk),
        .rst(rst),
        .OnInt('d0), // Interrupts Disabled for now, will be enabled later with devices
        .CmpOut(Cmp[1:0]),
        .Rx(IR[27:24]),
        .Ry(IR[23:20]),
        .Rz(IR[3:0]),
        .Opcode(IR[31:28]),
        .DrREG(DrREG),
        .DrMEM(DrMEM),
        .DrALU(DrALU),
        .DrPC(DrPC),
        .DrOFF(DrOFF),
        .LdPC(LdPC),
        .LdIR(LdIR),
        .LdMAR(LdMAR),
        .LdA(LdA),
        .LdB(LdB),
        .LdCmp(LdCmp),
        .WrREG(WrREG),
        .WrMEM(WrMEM),
        .OPTest(OPTest),
        .ChkCmp(ChkCmp),
        .LdEnInt(LdEnInt),
        .IntAck(IntAck),
        .DrData(DrData),
        .LdDAR(LdDAR),
        .RegSel(RegSel),
        .ALUFunc(ALUFunc),
        .RegNo(RegNo)
    );

    wire [31:0] FromREG;
    regfile regfile1(
        .clk(clk),
        .rst(rst),
        .WrReg(WrREG),
        .RegSel(RegSel),
        .Din(DAR),
        .Dout(FromREG)
    );
    
    wire [31:0] FromMEM;
    ram ram1(
        .MAR(MAR[15:0]),
        .clk(clk),
        .Din(A),
        .WrMem(WrMEM),
        .Dout(FromMEM)
    );

    wire [31:0] FromALU;
    alu alu1(
        .A(A),
        .B(B),
        .func(ALUFunc),
        .out(FromALU)
    );

    initial begin
        PC = 32'h00000000;
        IR = 32'h00000000;
        MAR = 32'h00000000;
        A = 32'h00000000;
        B = 32'h00000000;
        Cmp = 32'h00000000;
        DAR = 32'h00000000;
        IE = 32'h00000000;
    end

    wire [31:0] Bus;

    always @(posedge clk) begin
        if (LdPC)  PC <= Bus;
        if (LdIR)  IR <= Bus;
        if (LdMAR) MAR <= Bus;
        if (LdA)   A <= Bus;
        if (LdB)   B <= Bus;
        if (LdCmp) Cmp <= Bus;
        if (LdDAR) DAR <= Bus;
        if (LdEnInt) IE <= Bus;
    end


    always @(*)
        begin
            RegSel = micro_controller1.RegSel;
            ALUFunc = micro_controller1.ALUFunc;
            RegNo = micro_controller1.RegNo;
            DrREG = micro_controller1.DrREG;
            DrMEM = micro_controller1.DrMEM;
            DrALU = micro_controller1.DrALU;
            DrPC = micro_controller1.DrPC;
            DrOFF = micro_controller1.DrOFF;
            LdPC = micro_controller1.LdPC;
            LdIR = micro_controller1.LdIR;
            LdMAR = micro_controller1.LdMAR;
            LdA = micro_controller1.LdA;
            LdB = micro_controller1.LdB;
            LdCmp = micro_controller1.LdCmp;
            WrREG = micro_controller1.WrREG;
            WrMEM = micro_controller1.WrMEM;
            OPTest = micro_controller1.OPTest;
            ChkCmp = micro_controller1.ChkCmp;
            LdEnInt = micro_controller1.LdEnInt;
            IntAck = micro_controller1.IntAck;
            DrData = micro_controller1.DrData;
            LdDAR = micro_controller1.LdDAR;

            if (DrREG) Bus = FromREG;
            else if (DrMEM) Bus = FromMEM;
            else if (DrALU) Bus = FromALU;
            else if (DrPC) Bus = ('d8 > PC ? PC + 'd8 : PC);
            else if (DrOFF) Bus = $signed(IR[19:0]);
            else if (DrData) Bus = IE;
            else Bus = 32'hZZZZ_ZZZZ;
        end

endmodule