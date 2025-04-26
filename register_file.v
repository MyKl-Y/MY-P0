// Project: MY-P0 Processor
// Module: Register File
// File: register_file.v
// Created Date: 2025/04/25

module register_file
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