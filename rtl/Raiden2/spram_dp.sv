// SPDX-License-Identifier: GPL-3.0-or-later
// Author: Umberto Parisi (rmonic79)
// Simple dual-port BRAM (1 write + 1 read, clock comune) tramite primitivo
// altsyncram istanziato ESPLICITAMENTE -> M10K block RAM GARANTITO (Quartus non
// puo declassarlo a logic come fa con i reg-array piccoli sotto-soglia).
// Read latency = 2 cicli (address_reg_b CLOCK0 + outdata_reg_b CLOCK0). Usata per i bank del line buffer
// sprite: 1 write/ciclo (decode o clear) + 1 read/ciclo (read-side).
module spram_dp #(
	parameter DW = 14,    // data width
	parameter AW = 5      // address width (depth = 2^AW)
) (
	input  wire           clk,
	input  wire           we,
	input  wire [AW-1:0]  waddr,
	input  wire [DW-1:0]  wdata,
	input  wire [AW-1:0]  raddr,
	output wire [DW-1:0]  rdata
);
	altsyncram #(
		.operation_mode          ("DUAL_PORT"),
		.width_a                 (DW),
		.widthad_a               (AW),
		.width_b                 (DW),
		.widthad_b               (AW),
		.ram_block_type          ("M10K"),
		.outdata_reg_b           ("CLOCK0"),
		.address_reg_b           ("CLOCK0"),
		.clock_enable_input_a    ("BYPASS"),
		.clock_enable_input_b    ("BYPASS"),
		.clock_enable_output_b   ("BYPASS"),
		.power_up_uninitialized  ("FALSE")
	) u_ram (
		.clock0     (clk),
		.wren_a     (we),
		.address_a  (waddr),
		.data_a     (wdata),
		.address_b  (raddr),
		.q_b        (rdata),
		// porte inutilizzate
		.aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0),
		.byteena_a(1'b1), .byteena_b(1'b1), .clock1(1'b1), .clocken0(1'b1),
		.clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1), .data_b({DW{1'b0}}),
		.eccstatus(), .q_a(), .rden_a(1'b1), .rden_b(1'b1), .wren_b(1'b0)
	);
endmodule
