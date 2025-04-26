// Project: MY-P0 Processor
// Module: Memory Unit
// File: micro_control_unit.v
// Created Date: 2025/04/25

module memory_unit (input [15:0] MAR, input clk, input [31:0] Din, input WrMem, output wire [31:0] Dout);

	reg [31:0] memory [0: 2**16 - 1];

	always @(posedge clk)
		begin
			if (WrMem) memory[MAR] <= Din;
		end
	
	assign Dout = memory[MAR];

endmodule