// SPDX-License-Identifier: GPL-3.0-or-later
// Original: Martin Donlon (wickerwaka) - Arcade-TaitoF2 (video_path). Modified by Umberto Parisi.
//
// raiden2_rotate_fifo.sv
// FIFO che cattura write da screen_rotate (CLK_VIDEO) e li drena nel bus DDRAM
// via ddr_if. COPIA ESATTA del pattern taito_f2_baseline/rtl/video_path.sv linee 159-209.
//
// Lato screen_rotate: alza rot_we per 1 ciclo ogni pixel (a CE_PIXEL).
// Lato DDR: accumula 1024 entry, alza acquire quando non vuoto, scrive in burst.

module raiden2_rotate_fifo
(
	input              clk,

	// Da screen_rotate
	input       [28:0] rot_addr,
	input       [63:0] rot_data,
	input        [7:0] rot_be,
	input              rot_we,

	// ddr_if.to_host: questo client parla al mux
	ddr_if.to_host     ddr
);

reg [9:0] fifo_wr_addr = 0, fifo_rd_addr = 0;
reg [63:0] fifo_mem [0:1023];  // pack: {rot_addr[28:0], 1'b0, be[4], be[0], data[31:0]}
reg [63:0] fifo_out;

always @(posedge clk) begin
	if (rot_we) begin
		fifo_mem[fifo_wr_addr] <= {rot_addr, 1'b0, rot_be[4], rot_be[0], rot_data[31:0]};
		fifo_wr_addr <= fifo_wr_addr + 10'd1;
	end
end

// Read port: lettura registrata sul rd_addr corrente.
always @(posedge clk) fifo_out <= fifo_mem[fifo_rd_addr];

// Output verso ddr_if (esattamente come Taito F2)
assign ddr.read       = 0;
assign ddr.burstcnt   = 1;
assign ddr.addr       = {fifo_out[63:35], 3'b0};
assign ddr.wdata      = {fifo_out[31:0], fifo_out[31:0]};
assign ddr.byteenable = {{4{fifo_out[33]}}, {4{fifo_out[32]}}};

reg ddr_acquire = 0;
reg ddr_write = 0;
assign ddr.acquire = ddr_acquire;
assign ddr.write   = ddr_write;

always @(posedge clk) begin
	if (fifo_wr_addr != fifo_rd_addr) begin
		ddr_acquire <= 1;
	end

	if (~ddr.busy & ddr_acquire) begin
		if (ddr_write) begin
			if (fifo_rd_addr != fifo_wr_addr) begin
				fifo_rd_addr <= fifo_rd_addr + 10'd1;
			end else begin
				ddr_write   <= 0;
				ddr_acquire <= 0;
			end
		end else begin
			if (fifo_rd_addr != fifo_wr_addr) begin
				ddr_write    <= 1;
				fifo_rd_addr <= fifo_rd_addr + 10'd1;
			end
		end
	end
end

endmodule
