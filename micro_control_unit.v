// Project: MY-P0 Processor
// Module: Micro Control Unit
// File: micro_control_unit.v
// Created Date: 2025/04/25

module micro_control_unit
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

		$readmemb("microcode/main_rom.txt", Main_ROM);
		
		$readmemb("microcode/cc_rom.txt", CC_ROM);
		
		$readmemb("microcode/sequencer_rom.txt", SEQ_ROM);
		
		$readmemb("microcode/interrupt_rom.txt", INT_ROM);
	end

	always @(posedge clk)
		begin
			//if (rst) State <= 6'b000000;
			//else State <= NextState;
			State <= NextState;
		end

		assign NextState =	{ChkCmp, OPTest} == 2'b00	?	Main_ROM[State][5:0] :
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