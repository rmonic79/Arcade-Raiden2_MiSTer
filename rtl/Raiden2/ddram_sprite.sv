// SPDX-License-Identifier: GPL-3.0-or-later
//
// ddram_sprite.sv — DDR3 reader/writer per sprite ROM Raiden2.
// Derivato da ddram_4port.sv (Sorgelig / BoogieWings). 1 write port (download)
// + 1 read port 32-bit (sprite renderer). RAM DDR3 @ 0x30000000.
//
module ddram_sprite
(
	input         clk,

	// Write port (download sprite ROM)
	input  [27:0] wraddr,
	input  [15:0] din,
	input         we_req,
	output reg    we_ack = 0,

	// Read port 32-bit (sprite ROM fetch)
	input     [27:0] rdaddr,
	output reg [31:0] dout = 0,
	input            rd_req,
	output reg       rd_ack = 0,

	// Read port 8-bit, priorita' BASSA (OKI2 ADPCM di Raiden II).
	// ~1 MB/s contro i ~40 degli sprite: servito solo quando non c'e' una
	// richiesta sprite in coda, e con cache PROPRIA per non invalidare quella
	// sequenziale del renderer (che ha il prefetch).
	input     [27:0] rdaddr2,
	output reg  [7:0] dout2 = 0,
	input            rd2_req,
	output reg       rd2_ack = 0,

	// Lato bus = ddr_if.to_host, identico a rtl/Raiden/raiden_sprite_ddr_client.sv:
	// escono addr/read/write/burstcnt/byteenable/acquire, entrano rdata/busy/
	// rdata_ready. I pin DDRAM_* non si toccano piu' da qui: li pilota l'arbitro
	// raiden2_ddr_mux nel top, che condivide il bus con il framebuffer di rotate.
	ddr_if.to_host ddr
);

reg  [7:0] ram_burst;
reg [63:0] ram_q, next_q;
reg [63:0] ram_data;
reg [27:0] ram_address;
reg [27:0] cache_addr = '1;
reg [63:0] ram_q2;
reg [27:0] cache_addr2 = '1;
reg        ram_read = 0;
reg        ram_write = 0;
reg  [7:0] ram_wr_be;

reg [2:0]  state = 0;   // 3 bit: serve lo stato 4 (porta OKI2)

// Estrae 32 bit dalla qword 64 (sel = rdaddr[2]) e SWAPPA le due word16, per
// dare al renderer lo stesso ordine {hi_word, lo_word} big-endian che la SDRAM
// produceva (sdram_bridge: tile_data={tile_hi_word, sdram_dout0}). Senza lo
// swap gli sprite si mescolano (doc 06_ddr3_to_sdram §8/§12: byte order).
// Estrae 32 bit dalla qword 64 (sel=rdaddr[2]) e swappa le due word16 (= stato
// commit 303baf7 con sprite visibili). Doc 06 §8: ordine {hi_word, lo_word}.
// NIENTE swap per Raiden II: la region "sprites" e' ROM_REGION32_LE nativa e
// il download scrive gia' plain[15:0] a +0 e plain[31:16] a +2, quindi la
// dword in DDR3 e' identica a quella di MAME. Lo swap veniva da Seibu Cup,
// dove la ROM era caricata WORD_SWAP per un 68000 big-endian.
function [31:0] extract32_swap(input [63:0] q, input sel);
	begin
		extract32_swap = sel ? q[63:32] : q[31:0];
	end
endfunction

// acquire = "voglio il bus". STICKY per TUTTA la transazione, non solo finche'
// c'e' una richiesta pendente. Due buchi dimostrano che il solo `pending` NON
// basta e che serve il termine (state != 0):
//   - stato 3 (2o beat del burst di prefetch): rd_ack e' GIA' == rd_req, quindi
//     pending = 0 mentre un beat e' ancora in volo;
//   - ramo cache-hit+1 (sotto): rd_ack <= rd_req viene aggiornato NELLO STESSO
//     ciclo in cui parte la read, quindi il ciclo dopo pending = 0 con la read
//     appena emessa.
// In entrambi i casi, con acquire basso il mux passerebbe il bus al rotate, il
// beat arriverebbe con rdata_ready forzato a 0 e la FSM resterebbe appesa per
// sempre (sprite morti). Formula identica a raiden_sprite_ddr_client.sv:50.
wire pending = (we_ack != we_req) || (rd_req != rd_ack) || (rd2_req != rd2_ack);
assign ddr.acquire    = pending || (state != 3'd0);

assign ddr.burstcnt   = ram_burst;
assign ddr.byteenable = ram_wr_be | {8{ram_read}};
// ram_address e' un byte-address a 28 bit; il pin DDRAM_ADDR (29 bit) e'
// ddr.addr[31:3]. Quindi addr = {4'b0011, ram_address[27:3], 3'b0} produce bit
// per bit lo STESSO valore del vecchio assign DDRAM_ADDR (RAM a 0x30000000).
// NON troncare: buttare i bit alti manda la sprite ROM fuori posto.
assign ddr.addr       = {4'b0011, ram_address[27:3], 3'b0};
assign ddr.read       = ram_read;
assign ddr.wdata      = ram_data;
assign ddr.write      = ram_write;

// Alias interni: il corpo della FSM qui sotto resta IDENTICO a prima.
wire        DDRAM_BUSY       = ddr.busy;
wire [63:0] DDRAM_DOUT       = ddr.rdata;
wire        DDRAM_DOUT_READY = ddr.rdata_ready;

always @(posedge clk) begin
	if(!DDRAM_BUSY) begin
		ram_write <= 0;
		ram_read  <= 0;

		case(state)
			0: if(we_ack != we_req) begin
					ram_data    <= {4{din}};
					ram_address <= wraddr;
					ram_write   <= 1;
					ram_burst   <= 1;
					ram_wr_be   <= (8'd3 << {wraddr[2:1],1'b0});
					state       <= 1;
				end
				else if(rd_req != rd_ack) begin
					if(cache_addr[27:3] == rdaddr[27:3]) begin
						rd_ack <= rd_req;
						dout   <= extract32_swap(ram_q, rdaddr[2]);
					end
					else if((cache_addr[27:3]+1'd1) == rdaddr[27:3]) begin
						rd_ack      <= rd_req;
						ram_q       <= next_q;
						dout        <= extract32_swap(next_q, rdaddr[2]);
						cache_addr  <= {rdaddr[27:3],3'b000};
						ram_address <= {rdaddr[27:3]+1'd1,3'b000};
						ram_read    <= 1;
						ram_burst   <= 1;
						state       <= 3;
					end
					else begin
						ram_address <= {rdaddr[27:3],3'b000};
						cache_addr  <= {rdaddr[27:3],3'b000};
						ram_read    <= 1;
						ram_burst   <= 2;
						state       <= 2;
					end
				end

				// Porta 2 (OKI2): solo se non c'e' nulla di piu' urgente.
				else if(rd2_req != rd2_ack) begin
					if(cache_addr2[27:3] == rdaddr2[27:3]) begin
						rd2_ack <= rd2_req;
						dout2   <= ram_q2[{rdaddr2[2:0],3'b000} +: 8];
					end
					else begin
						ram_address <= {rdaddr2[27:3],3'b000};
						cache_addr2 <= {rdaddr2[27:3],3'b000};
						ram_read    <= 1;
						ram_burst   <= 1;
						state       <= 4;
					end
				end

			1: begin
					cache_addr      <= '1;
					cache_addr[3:0] <= 0;
					cache_addr2     <= '1;
					we_ack <= we_req;
					state  <= 0;
				end

			2: if(DDRAM_DOUT_READY) begin
					ram_q  <= DDRAM_DOUT;
					dout   <= extract32_swap(DDRAM_DOUT, rdaddr[2]);
					rd_ack <= rd_req;
					state  <= 3;
				end

			3: if(DDRAM_DOUT_READY) begin
					next_q <= DDRAM_DOUT;
					state  <= 0;
				end

			// Porta 2: burst singolo, niente prefetch (accessi radi e sparsi).
			4: if(DDRAM_DOUT_READY) begin
					ram_q2  <= DDRAM_DOUT;
					dout2   <= DDRAM_DOUT[{rdaddr2[2:0],3'b000} +: 8];
					rd2_ack <= rd2_req;
					state   <= 0;
				end
		endcase
	end
end

endmodule
