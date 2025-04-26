// Project: MY-P0 Processor
// Module: Distance Tracker Device
// File: distance_tracker_device.v
// Created Date: 2025/04/25

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
		$readmemb("microcode/key_buffer_rom.txt", key_buffer_rom);
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