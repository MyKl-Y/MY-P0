/*
MY-P0 Processor: RISC ISA, 32-bit, Interrupts Enabled, Word Addressable, 3-stage Processor
*/

`include "micro_control_unit.v"
`include "arithmetic_logic_unit.v"
`include "memory_unit.v"
`include "register_file.v"
`include "timer_device.v"
`include "distance_tracker_device.v"

module Processor (input clk, rst );
	reg [31:0] PC, IR, MAR, A, B, DAR;
	reg [1:0] Cmp;
	reg IE;

	wire	DrREG, DrMEM, DrALU, DrPC, DrOFF, LdPC, LdIR, LdMAR, LdA, LdB, LdCmp, WrREG, 
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