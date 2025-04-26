// Project: MY-P0 Processor
// Module: Timer Device
// File: timer_device.v
// Created Date: 2025/04/25

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