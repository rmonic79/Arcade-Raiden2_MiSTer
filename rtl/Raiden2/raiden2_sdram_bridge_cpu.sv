// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// Raiden V30 ↔ SDRAM bridge per 1 CPU (pattern Irem M72 m72.v:255-295)
// Gestisce accesso CPU a region SDRAM (ROM Main o Sub).
// Genera mem_rq_active (per CE generator stall) e sdram_req toggle.
//
// Pattern M72:
//   1. Edge-detect rising di cpu_mem_read (= nuova read SDRAM)
//   2. ls245_en alto + edge → toggle sdram_req, mem_rq_active=1
//   3. Aspetta sdram_req == sdram_ack (= dato pronto)
//   4. Latch dato in cpu_ram_rom_data, mem_rq_active=0
//
// CPU vede dato latched in cpu_ram_rom_data (stabile fino prossima request).
// Mux cpu_mem_in (top): cpu_mem_in = cpu_ram_rom_data quando nessun *_dout_valid_lat.

module raiden2_sdram_bridge_cpu
(
	input  wire        clk,
	input  wire        reset,

	// CPU interface
	input  wire        ls245_en,           // CPU sta accedendo region SDRAM
	input  wire        cpu_mem_read,       // V30 bus_read
	input  wire        cpu_mem_write,      // V30 bus_write
	input  wire [23:0] cpu_region_addr,    // sdr_addr da address_translator
	input  wire [15:0] cpu_data_out,       // V30 bus_datawrite
	input  wire  [1:0] cpu_be,             // V30 bus_be
	input  wire        cpu_writable,       // region è writable (RAM, NON ROM)

	// SDRAM controller interface (toggle protocol)
	output reg  [23:0] sdram_addr,         // SDRAM address (byte)
	output reg  [15:0] sdram_din,          // SDRAM write data
	output reg   [1:0] sdram_wr_sel,       // byte select per write
	output reg         sdram_rq,           // toggle: nuova request quando flip
	input  wire        sdram_ack,          // toggle ack: match rq quando dato pronto
	input  wire [15:0] sdram_dout,         // SDRAM read data

	// Output a CPU mux
	output reg  [15:0] cpu_ram_rom_data,   // latched data (stabile fino prossima req)
	output reg         mem_rq_active       // alto durante req in volo (CE stall)
);

reg cpu_mem_read_lat;
reg cpu_mem_write_lat;

always @(posedge clk) begin
	if (reset) begin
		mem_rq_active     <= 1'b0;
		cpu_mem_read_lat  <= 1'b0;
		cpu_mem_write_lat <= 1'b0;
		sdram_rq          <= 1'b0;
		sdram_addr        <= 24'd0;
		sdram_din         <= 16'd0;
		sdram_wr_sel      <= 2'b00;
		cpu_ram_rom_data  <= 16'd0;
	end else begin
		cpu_mem_read_lat  <= cpu_mem_read;
		cpu_mem_write_lat <= cpu_mem_write;

		if (!mem_rq_active) begin
			// Edge-detect rising di cpu_mem_read o cpu_mem_write
			if (ls245_en && ((cpu_mem_read & ~cpu_mem_read_lat) || (cpu_mem_write & ~cpu_mem_write_lat))) begin
				// Nuova SDRAM request
				sdram_addr   <= cpu_region_addr;
				sdram_wr_sel <= 2'b00;
				if (cpu_mem_write && cpu_writable) begin
					sdram_wr_sel <= cpu_be;
					sdram_din    <= cpu_data_out;
				end
				sdram_rq     <= ~sdram_rq;   // toggle
				mem_rq_active <= 1'b1;
			end
		end else if (sdram_rq == sdram_ack) begin
			// SDRAM ha completato. Latch dato e clear active.
			cpu_ram_rom_data <= sdram_dout;
			mem_rq_active    <= 1'b0;
		end
	end
end

endmodule
