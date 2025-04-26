// Project: MY-P0 Processor
// Module: Arithmetic Logic Unit
// File: arithmetic_logic_unit.v
// Created Date: 2025/04/25

module alu (input [31:0] A, B, input [2:0] func, output wire [31:0] out);

	assign out = func == 3'b000 ? A + B :
				func == 3'b001 ? A - B :
				func == 3'b010 ? ~(A & B) :
				func == 3'b011 ? A + 1 :
				func == 3'b100 ? A :
				func == 3'b101 ? B :
				32'hZZZZ_ZZZZ;

endmodule