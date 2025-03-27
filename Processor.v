/*
MY-P0 Processor: RISC ISA, 32-bit, Interrupts Enabled, Word Addressable, 3-stage Processor
*/

module micro_controller 
    (
        input   clk, rst, OnInt,
        input   [1:0] CmpOut,
        input   [3:0] Rx, Ry, Rz, Opcode,
        output wire  DrREG, DrMEM, DrALU, DrPC, DrOFF, LdPC, LdIR, LdMAR, LdA, LdB, LdCmp, WrREG, 
                WrMEM, OPTest, ChkCmp, LdEnInt, IntAck, DrData, LdDAR,
        output wire [1:0] RegSel,
        output wire [2:0] ALUFunc,
        output wire [3:0] RegNo
    );

    reg [5:0] State;
    reg [5:0] SEQ_ROM [0: 2**4 - 1];
    reg [5:0] CC_ROM [0: 2**2 - 1];
    reg [5:0] INT_ROM [0: 2**1 - 1];
    reg [29:0] Main_ROM [0: 2**6 - 1];

    wire [5:0] NextState;

    initial begin
        State = 6'b000000;

        $readmemb("main_rom.txt", Main_ROM);
		  
		  $readmemb("cc_rom.txt", CC_ROM);
		  
		  $readmemb("sequencer_rom.txt", SEQ_ROM);
		  
		  $readmemb("interrupt_rom.txt", INT_ROM);
    end

    always @(posedge clk)
        begin
            //if (rst) State <= 6'b000000;
            //else State <= NextState;
            State <= NextState;
        end

		  assign NextState = {ChkCmp, OPTest} == 2'b00	?	Main_ROM[State][5:0] :
						  {ChkCmp, OPTest} == 2'b01	?	SEQ_ROM[Opcode][5:0] :
						  {ChkCmp, OPTest} == 2'b10	?	CC_ROM[CmpOut][5:0] :
						  {ChkCmp, OPTest} == 2'b11	?	INT_ROM[OnInt][5:0] :
						  6'bZZZZZZ;

            assign DrREG = Main_ROM[State][6];
            assign DrMEM = Main_ROM[State][7];
            assign DrALU = Main_ROM[State][8];
            assign DrPC = Main_ROM[State][9];
            assign DrOFF = Main_ROM[State][10];
            assign LdPC = Main_ROM[State][11];
            assign LdIR = Main_ROM[State][12];
            assign LdMAR = Main_ROM[State][13];
            assign LdA = Main_ROM[State][14];
            assign LdB = Main_ROM[State][15];
            assign LdCmp = Main_ROM[State][16];
            assign WrREG = Main_ROM[State][17];
            assign WrMEM = Main_ROM[State][18];
            assign RegSel = Main_ROM[State][20:19];
            assign ALUFunc = Main_ROM[State][23:21];
            assign OPTest = Main_ROM[State][24];
            assign ChkCmp = Main_ROM[State][25];
            assign LdEnInt = Main_ROM[State][26];
            assign IntAck = Main_ROM[State][27];
            assign DrData = Main_ROM[State][28];
            assign LdDAR = Main_ROM[State][29];


			assign RegNo =	RegSel == 2'b00 ? Rx :
						RegSel == 2'b01 ? Ry :
						RegSel == 2'b10 ? Rz :
						RegSel == 2'b11 ? 4'b1100 :
						4'bZZZZ;

endmodule

module ram (input [15:0] MAR, input clk, input [31:0] Din, input WrMem, output wire [31:0] Dout);

    reg [31:0] memory [0: 2**16 - 1];

    always @(posedge clk)
        begin
            if (WrMem) memory[MAR] <= Din;
        end
    
    assign Dout = memory[MAR];

endmodule

module alu (input [31:0] A, B, input [2:0] func, output wire [31:0] out);

    assign out = func == 3'b000 ? A + B :
                func == 3'b001 ? A - B :
                func == 3'b010 ? ~(A & B) :
                func == 3'b011 ? A + 1 :
                func == 3'b100 ? A :
                func == 3'b101 ? B :
                32'hZZZZ_ZZZZ;
					 
endmodule

module regfile
    (
        input clk,
        input rst,
        input WrReg,
        input [3:0] RegSel,
        input [31:0] Din,
        output wire [31:0] Dout
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

    assign Dout = Registers[RegSel];

    integer i;
    always @(posedge clk) begin
            if (rst) // Reset
                for (i = 0; i < 16; i = i + 1) begin
                    Registers[i] <= 32'h00000000;
                end
            else begin
                if (WrReg) Registers[RegSel] <= Din;
            end
        end

endmodule

module timer_device
	(
		input clk, rst, inta_in,
		output timer_int, inta_out,
		output [31:0] data
	);
	
	reg [31:0] reg1;
	reg reg2, int_reg;
	
	wire reset, empty;
	
	assign timer_int = int_reg ? 1 : 1'bZ;
	
	assign inta_out = ~reset & inta_in & ~reg2;
	
	assign data = reg2 ? 32'b0 : 32'bZ;
		
	assign empty = reg1 < 32'h7d0;
	
	assign reset = ~empty & inta_in;
	
	always @(posedge clk or posedge rst) begin
		if (rst) begin 
			int_reg = 1'b0;
			reg1 = 32'b0;
			reg2 = 1'b0;
		end else begin
			int_reg = ~(empty | inta_in);
			case ({reset, empty})
				2'b00: reg1 = reg1;
				2'b01: reg1 = reg1 + 1;
				default: reg1 = 32'b0;
			endcase
			reg2 = reset;
		end
	end
	
endmodule

module distance_tracker_device
	(
		input clk, rst, inta_in,
		input [31:0] addr,
		output tracker_int, inta_out,
		output [31:0] data
	);
	
	reg [31:0] reg1;
	reg reg2, int_reg;
	reg [3:0] reg3;
	
	reg [31:0] key_buffer_rom [0: 2**4 - 1];
	
	wire reset, empty, dout;
	
	assign tracker_int = int_reg ? 1 : 1'bZ;
	
	assign inta_out = ~reset & inta_in & ~reg2;
	
	assign dout = reg2;
		
	assign empty = reg1 < 32'h5d0;
	
	assign reset = ~empty & inta_in;
	
	assign data = ~dout & addr == 32'b1 ? key_buffer_rom[reg3-1] : dout ? 32'b1 : 32'bZ;
	
	initial begin
		$readmemb("key_buffer_rom.txt", key_buffer_rom);
	end
	
	always @(posedge clk or posedge rst) begin
		if (rst) begin 
			int_reg = 1'b0;
			reg1 = 32'b0;
			reg2 = 1'b0;
			reg3 = 4'b0;
		end else begin
			int_reg = ~(empty | inta_in);
			case ({reset, empty})
				2'b00: reg1 = reg1;
				2'b01: reg1 = reg1 + 1;
				default: reg1 = 32'b0;
			endcase
			reg2 = reset;
			reg3 = reset ? reg3 + 1 : reg3;
		end
	end
	
endmodule


module Processor (input clk, rst );
    reg [31:0] PC, IR, MAR, A, B, DAR;
    reg [1:0] Cmp;
    reg IE;

    wire DrREG, DrMEM, DrALU, DrPC, DrOFF, LdPC, LdIR, LdMAR, LdA, LdB, LdCmp, WrREG, 
         WrMEM, OPTest, ChkCmp, LdEnInt, IntAck, DrData, LdDAR, OnInt;
    wire [1:0] RegSel;
    wire [2:0] ALUFunc;
    wire [3:0] RegNo;
	 wire [5:0] Opcode;
	 wire [3:0] IR_to_Rx, IR_to_Ry, IR_to_Rz;
	 wire [19:0] IR_to_OFF;
	 
	 assign Opcode = IR[31:28];
	 assign IR_to_Rx = IR[27:24];
	 assign IR_to_Ry = IR[23:20];
	 assign IR_to_Rz = IR_to_OFF[3:0];
	 assign IR_to_OFF = IR[19:0];
	 
	 wire [31:0] device_data;
	 wire inta_from_timer, timer_int, inta_from_distance_tracker, distance_tracker_int;
	 timer_device timer_device1(
		.clk(clk),
		.rst(rst),
		.inta_in(IntAck),
		.timer_int(timer_int),
		.inta_out(inta_from_timer),
		.data(device_data)
	 );
	 distance_tracker_device distance_tracker_device(
		.clk(clk),
		.rst(rst),
		.inta_in(IntAck),
		.addr(DAR),
		.tracker_int(distance_tracker_int),
		.inta_out(inta_from_distance_tracker),
		.data(device_data)
	 );

    micro_controller micro_controller1(
        .clk(clk),
        .rst(rst),
        .OnInt(OnInt), // Interrupts Disabled for now, will be enabled later with devices
        .CmpOut(Cmp),
        .Rx(IR_to_Rx),
        .Ry(IR_to_Ry),
        .Rz(IR_to_Rz),
        .Opcode(Opcode),
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
        .RegSel(RegNo),
        .Din(Bus),
        .Dout(FromREG)
    );
    
    wire [31:0] FromMEM;
	 
    ram ram1(
        .MAR(MAR[15:0]),
        .clk(clk),
        .Din(Bus),
        .WrMem(WrMEM),
        .Dout(FromMEM)
    );

    wire [31:0] FromALU;
	 wire [31:0] FromA , FromB;
	 
	 assign FromA = A;
	 assign FromB = B;
	 
    alu alu1(
        .A(FromA),
        .B(FromB),
        .func(ALUFunc),
        .out(FromALU)
    );
	 
    initial begin
        PC = 32'h00000000;
        IR = 32'h00000000;
        MAR = 32'h00000000;
        A = 32'h00000000;
        B = 32'h00000000;
        Cmp = 2'h0;
        DAR = 32'h00000000;
        IE = 1'h0;
    end

    wire [31:0] Bus;

    always @(posedge clk or posedge rst) begin
		if (rst) begin
			PC <= 'b0;
			IR <= 'b0;
			MAR <= 'b0;
			A <= 'b0;
			B <= 'b0;
			Cmp <= 'b0;
			DAR <= 'b0;
			IE <= 'b0;
		end else begin
        if (LdPC)  PC <= Bus;
        if (LdIR)  IR <= Bus;
        if (LdMAR) MAR <= Bus;
        if (LdA)   A <= Bus;
        if (LdB)   B <= Bus;
        if (LdCmp) begin 
				case (Opcode[1:0]) 
					2'b00: Cmp <= Bus > 'h0 ? {Opcode[1:1], 1'b1} : {Opcode[1:1], 1'b0};
					2'b01: Cmp <= Bus == 'h0 ? {Opcode[1:1], 1'b1} : {Opcode[1:1], 1'b0};
					2'b10: Cmp <= Bus == 'h0 ? {Opcode[1:1], 1'b1} : {Opcode[1:1], 1'b0};
					2'b11: Cmp <= Bus == 'h0 ? {Opcode[1:1], 1'b1} : {Opcode[1:1], 1'b0};
				endcase
			end
        if (LdDAR) DAR <= Bus;
        if (LdEnInt) IE <= IntAck ? 0 : ~(IR_to_Ry[0]);
		end
		end
		
		assign OnInt = IE & (( timer_int ? 1 : distance_tracker_int ? 1 : 1'bZ ) | 0);

		wire [31:0] extended_IR;
		assign extended_IR = { {12{IR_to_OFF[19]}}, IR_to_OFF };

		assign Bus = 	DrREG ? 	FromREG : 32'hZZZZ_ZZZZ;
		assign Bus = 	DrMEM ? 	FromMEM : 32'hZZZZ_ZZZZ;
		assign Bus = 	DrALU ? 	FromALU : 32'hZZZZ_ZZZZ;
		assign Bus = 	DrPC	?	('d8 > PC ? PC + 'd8 : PC) : 32'hZZZZ_ZZZZ;
		assign Bus = 	DrOFF	?	extended_IR : 32'hZZZZ_ZZZZ;
		assign Bus = 	DrData ? IE : 32'hZZZZ_ZZZZ;

endmodule