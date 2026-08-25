// SPDX-License-Identifier: GPL-3.0-or-later
// Original: Sorgelig - MiSTer NeoGeo (ddram). Modified by Umberto Parisi.
//
// raiden2_ddram.sv
// DDR3 backend per Raiden — solo 1 write port (sprite upload) + 1 read port 32-bit (sprite fetch).
// Pattern Sorgelig NeoGeo / darius2_ddram, ridotto al minimo.
//
// Sprite ROM (sei440 512KB) caricato qui durante ioctl_download.
// Sprite renderer legge 32-bit/word con cache 8-byte interna.

module raiden2_ddram
(
	input         DDRAM_CLK,

	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	// Write port (ioctl upload sprite ROM)
	input  [27:0] wraddr,
	input  [15:0] din,
	input         we_byte,  // 0:word write, 1:byte write
	input         we_req,
	output reg    we_ack = 0,

	// Read port: 32-bit sprite fetch
	input     [27:0] rdaddr,
	output reg [31:0] dout = 0,
	input            rd_req,
	output reg       rd_ack = 0
);

reg  [7:0] ram_burst;
reg [63:0] ram_q, next_q;
reg [63:0] ram_data;
reg [27:0] ram_address;
reg [27:0] cache_addr  = '1;  // init '1 → primo confronto fallisce, forza fetch reale
reg        ram_read = 0;
reg        ram_write = 0;
reg  [7:0] ram_wr_be;

reg [2:0]  state  = 0;

assign DDRAM_BURSTCNT = ram_burst;
assign DDRAM_BE       = ram_wr_be | {8{ram_read}};
assign DDRAM_ADDR     = {4'b0011, ram_address[27:3]}; // RAM at 0x30000000
assign DDRAM_RD       = ram_read;
assign DDRAM_DIN      = ram_data;
assign DDRAM_WE       = ram_write;

always @(posedge DDRAM_CLK) begin
	if(!DDRAM_BUSY) begin
		ram_write <= 0;
		ram_read  <= 0;

		case(state)
			0: if(we_ack != we_req) begin
					ram_data    <= we_byte ? {8{din[7:0]}} : {4{din}};
					ram_address <= wraddr;
					ram_write   <= 1;
					ram_burst   <= 1;
					ram_wr_be   <= we_byte ? (8'd1 << wraddr[2:0]) : (8'd3 << {wraddr[2:1], 1'b0});
					state       <= 1;
				end
				else if(rd_req != rd_ack) begin
					if(cache_addr[27:3] == rdaddr[27:3]) begin
						rd_ack <= rd_req;
						dout   <= ram_q[{rdaddr[2], 5'b00000} +: 32];
					end
					else if((cache_addr[27:3] + 1'd1) == rdaddr[27:3]) begin
						rd_ack      <= rd_req;
						ram_q       <= next_q;
						dout        <= next_q[{rdaddr[2], 5'b00000} +: 32];
						cache_addr  <= {rdaddr[27:3], 3'b000};
						ram_address <= {rdaddr[27:3] + 1'd1, 3'b000};
						ram_read    <= 1;
						ram_burst   <= 1;
						state       <= 3;
					end
					else begin
						ram_address <= {rdaddr[27:3], 3'b000};
						cache_addr  <= {rdaddr[27:3], 3'b000};
						ram_read    <= 1;
						ram_burst   <= 2;
						state       <= 2;
					end
				end

			1: begin
					cache_addr      <= '1;
					cache_addr[3:0] <= 0;
					we_ack <= we_req;
					state  <= 0;
				end

			2: if(DDRAM_DOUT_READY) begin
					ram_q  <= DDRAM_DOUT;
					dout   <= DDRAM_DOUT[{rdaddr[2], 5'b00000} +: 32];
					rd_ack <= rd_req;
					state  <= 3;
				end

			3: if(DDRAM_DOUT_READY) begin
					next_q <= DDRAM_DOUT;
					state  <= 0;
				end
		endcase
	end
end

endmodule
