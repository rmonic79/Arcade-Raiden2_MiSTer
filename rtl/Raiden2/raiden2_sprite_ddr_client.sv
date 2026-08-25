// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// raiden2_sprite_ddr_client.sv
// Wrapper di raiden2_ddram che espone un'interfaccia ddr_if.from_host.
// Logica interna IDENTICA a raiden2_ddram (cache 8-byte, FSM read/write).
// Solo cambia il "downstream": invece di pin DDRAM Sorgelig, parla con ddr_if.
//
// `acquire` alto = "voglio il bus". Mantenuto finché ci sono req pendenti.
// Il mux esterno decide chi vince; il vincitore vede `busy` dal DDRAM reale,
// gli altri vedono `busy=1`.

module raiden2_sprite_ddr_client
(
	input         clk,

	// Write port (ioctl upload sprite ROM)
	input  [27:0] wraddr,
	input  [15:0] din,
	input         we_byte,
	input         we_req,
	output reg    we_ack = 0,

	// Read port (32-bit sprite fetch via cache 8-byte)
	input  [27:0] rdaddr,
	output reg [31:0] dout = 0,
	input         rd_req,
	output reg    rd_ack = 0,

	// Client side = to_host (output write/addr/read, input rdata/busy)
	ddr_if.to_host ddr
);

reg  [7:0] ram_burst;
reg [63:0] ram_q, next_q;
reg [63:0] ram_data;
reg [27:0] ram_address;
reg [27:0] cache_addr  = '1;
reg        ram_read = 0;
reg        ram_write = 0;
reg  [7:0] ram_wr_be;

reg [2:0]  state = 0;

// Pending = abbiamo qualcosa da fare ⇒ acquire bus.
// IMPORTANTE: acquire sticky finché siamo in qualunque state ≠ 0 (FSM ha cmd
// in volo o aspetta DOUT_READY). Se lasciassimo cadere acquire mid-FSM, il mux
// passerebbe il bus a rotate e perderemmo i dati di sprite.
wire pending = (we_ack != we_req) || (rd_req != rd_ack);
assign ddr.acquire   = pending || (state != 3'd0);

// Map driver internals → ddr_if.
// ram_address è 28-bit (byte address). DDRAM_ADDR mappa addr[31:3] alle 29 word-lines
// di pin (DDR è word-addressed 64-bit, 8 byte/word). Quindi:
//   ddr.addr[31:3]  = pin DDRAM_ADDR (29 bit word-addr)
//   ddr.addr[2:0]   = byte offset (ignorato, BE seleziona i byte)
// Sprite ROM mappata a base 0x30000000 → top 4 bit di addr[31:28] = 4'b0011.
// ram_address[27:3] = 25 bit word-addr within range → posto in addr[27:3].
assign ddr.addr      = {4'b0011, ram_address[27:3], 3'b0};
assign ddr.wdata     = ram_data;
assign ddr.byteenable= ram_wr_be | {8{ram_read}};
assign ddr.write     = ram_write;
assign ddr.read      = ram_read;
assign ddr.burstcnt  = ram_burst;

// rdata/busy from ddr_if (input side)
wire        DDRAM_BUSY       = ddr.busy;
wire [63:0] DDRAM_DOUT       = ddr.rdata;
wire        DDRAM_DOUT_READY = ddr.rdata_ready;

always @(posedge clk) begin
	if (!DDRAM_BUSY) begin
		ram_write <= 0;
		ram_read  <= 0;

		case(state)
			0: if (we_ack != we_req) begin
					ram_data    <= we_byte ? {8{din[7:0]}} : {4{din}};
					ram_address <= wraddr;
					ram_write   <= 1;
					ram_burst   <= 1;
					ram_wr_be   <= we_byte ? (8'd1 << wraddr[2:0]) : (8'd3 << {wraddr[2:1], 1'b0});
					state       <= 1;
				end
				else if (rd_req != rd_ack) begin
					if (cache_addr[27:3] == rdaddr[27:3]) begin
						rd_ack <= rd_req;
						dout   <= ram_q[{rdaddr[2], 5'b00000} +: 32];
					end
					else if ((cache_addr[27:3] + 1'd1) == rdaddr[27:3]) begin
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

			2: if (DDRAM_DOUT_READY) begin
					ram_q  <= DDRAM_DOUT;
					dout   <= DDRAM_DOUT[{rdaddr[2], 5'b00000} +: 32];
					rd_ack <= rd_req;
					state  <= 3;
				end

			3: if (DDRAM_DOUT_READY) begin
					next_q <= DDRAM_DOUT;
					state  <= 0;
				end
		endcase
	end
end

endmodule
