// SPDX-License-Identifier: GPL-3.0-or-later
/*  This file is part of Raiden2_MiSTer.

    Raiden2_MiSTer is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Raiden2_MiSTer is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Raiden2_MiSTer.  If not, see <http://www.gnu.org/licenses/>.

    Author: Umberto Parisi (rmonic79)
    Version: 1.0
    Date: 2026

*/

// tile_rom_arbiter — Round-robin arbiter per ROM grafica SD Gundam.
// Multiplexa i client su un singolo Port0 del sdram_bridge:
//   Client 0: BG  (kind=0)
//   Client 1: MG  (kind=1)
//   Client 2: FG  (kind=2)
//   Client 3: SPR (kind=3)
//   Client 4: TXT (kind=4)  — solo se Text non è in cache BRAM
// Tutti usano letture 32-bit dalla SDRAM.

module tile_rom_arbiter (
	input  wire        clk,
	input  wire        reset,
	input  wire        hblank,   // FG gets exclusive access during hblank

	// Renderer 0 (LEFT)
	input  wire        r0_req,
	input  wire [23:0] r0_addr,
	output reg  [31:0] r0_data,
	output reg         r0_valid,

	// Renderer 1 (CENTER)
	input  wire        r1_req,
	input  wire [23:0] r1_addr,
	output reg  [31:0] r1_data,
	output reg         r1_valid,

	// Renderer 2 (RIGHT)
	input  wire        r2_req,
	input  wire [23:0] r2_addr,
	output reg  [31:0] r2_data,
	output reg         r2_valid,

	// Sprite renderer
	input  wire        r3_req,
	input  wire [23:0] r3_addr,
	output reg  [31:0] r3_data,
	output reg         r3_valid,

	// FG text renderer
	input  wire        r4_req,
	input  wire [23:0] r4_addr,
	output reg  [31:0] r4_data,
	output reg         r4_valid,

	// To sdram_bridge Port0
	output reg         tile_req,
	output reg  [23:0] tile_addr,
	output reg   [2:0] tile_kind,       // 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
	input  wire [31:0] tile_data,
	input  wire        tile_valid
);

// =====================================================================
// Tile ROM cache — 256-entry direct-mapped
// =====================================================================
// Reduces SDRAM latency for repeated tile fetches.
// Index = addr[9:2] (8 bits → 256 entries)
// Tag   = addr[18:10] (9 bits — covers tile_code + fine_y)
// 3 cache separate per BG/MG/FG: l'addr passato dai renderer è byte-offset
// RELATIVO alla region SDRAM (la base si aggiunge in sdram_bridge), quindi
// BG/MG/FG con stesso (tile, row) producono stesso addr → cache condivisa
// dava collisioni. Cache per-layer le elimina alla radice.
// Ridotta a 64 entry (era 256): -18k FF. Index addr[7:2] (6 bit), tag addr[19:8].
// TAG FINO AL BIT 19: effective_tile_idx e' 13 bit a rom_addr[19:7] — il bit 19
// e' il BANCO BG di godzilla ($100470, idx[12]). Col vecchio tag [18:8] i tile
// dei due banchi si ALIASAVANO in cache -> false hit -> tile stantii a schermo
// quando il gioco flippa il banco (tile "che si muovono in modo strano").
// ⛔ Queste sei array vanno lette con INDIRIZZO REGISTRATO e mux DOPO i
// registri d'uscita, e scritte una per always block (doc 04 e doc 17 sez.4).
// Con la lettura combinatoria e il mux sull'indice che c'erano prima, Quartus
// non poteva usare una porta M10K normale e le replicava: 6 kbit di cache
// occupavano 96 M10K (983 kbit), il 17% dell'intero pool del chip.
(* ramstyle = "M10K,no_rw_check" *) reg [31:0] cache_data_bg [0:63];
(* ramstyle = "M10K,no_rw_check" *) reg [31:0] cache_data_mg [0:63];
(* ramstyle = "M10K,no_rw_check" *) reg [31:0] cache_data_fg [0:63];
(* ramstyle = "M10K,no_rw_check" *) reg [13:0] cache_tag_bg  [0:63];
(* ramstyle = "M10K,no_rw_check" *) reg [13:0] cache_tag_mg  [0:63];
(* ramstyle = "M10K,no_rw_check" *) reg [13:0] cache_tag_fg  [0:63];
reg [63:0] cache_valid_bg;
reg [63:0] cache_valid_mg;
reg [63:0] cache_valid_fg;

wire [5:0]  cache_idx    = tile_addr[7:2];
// TAG FINO AL BIT 21: coi banchi tile di Raiden II ($470/$6CC) l'indice e' a
// 15 bit -> rom_addr[21:7], cioe' il banco sta nei bit 19-21. Col vecchio tag
// [19:8] i tile di banchi diversi aliasavano in cache (false hit) -> tile
// stantii a schermo a ogni cambio di banco.
wire [13:0] cache_tag_in = tile_addr[21:8];

// Cache hit per client (BG=0, MG=1, FG=2; SPR=3 e TXT=4 non usano cache)
// Lettura registrata: l'indirizzo e' `cache_idx`, che viene direttamente dal
// registro tile_addr (nessun mux). Il risultato e' pronto un ciclo dopo, in
// ARB_CHECK2. tile_addr non cambia fra ARB_CHECK e ARB_CHECK2, quindi
// cache_tag_in e active_client restano validi per il confronto.
reg [31:0] cd_bg_q, cd_mg_q, cd_fg_q;
reg [13:0] ct_bg_q, ct_mg_q, ct_fg_q;
reg        cv_bg_q, cv_mg_q, cv_fg_q;
always @(posedge clk) cd_bg_q <= cache_data_bg[cache_idx];
always @(posedge clk) cd_mg_q <= cache_data_mg[cache_idx];
always @(posedge clk) cd_fg_q <= cache_data_fg[cache_idx];
always @(posedge clk) ct_bg_q <= cache_tag_bg[cache_idx];
always @(posedge clk) ct_mg_q <= cache_tag_mg[cache_idx];
always @(posedge clk) ct_fg_q <= cache_tag_fg[cache_idx];
always @(posedge clk) begin
	cv_bg_q <= cache_valid_bg[cache_idx];
	cv_mg_q <= cache_valid_mg[cache_idx];
	cv_fg_q <= cache_valid_fg[cache_idx];
end

wire cache_hit_bg = cv_bg_q && (ct_bg_q == cache_tag_in);
wire cache_hit_mg = cv_mg_q && (ct_mg_q == cache_tag_in);
wire cache_hit_fg = cv_fg_q && (ct_fg_q == cache_tag_in);

wire cache_hit = (active_client == 3'd0) ? cache_hit_bg :
                 (active_client == 3'd1) ? cache_hit_mg :
                 (active_client == 3'd2) ? cache_hit_fg :
                                            1'b0;

// mux DOPO i registri d'uscita, mai dentro l'indice della memoria
wire [31:0] cache_hit_data = (active_client == 3'd0) ? cd_bg_q :
                             (active_client == 3'd1) ? cd_mg_q :
                             (active_client == 3'd2) ? cd_fg_q :
                                                        32'd0;

// Riempimento: un always block per array (mai due array nello stesso always,
// altrimenti Quartus Lite non inferisce la M10K).
wire cache_fill = (arb_state == ARB_WAIT) && tile_valid;
always @(posedge clk) if (cache_fill && active_client == 3'd0) cache_data_bg[cache_idx] <= tile_data;
always @(posedge clk) if (cache_fill && active_client == 3'd1) cache_data_mg[cache_idx] <= tile_data;
always @(posedge clk) if (cache_fill && active_client == 3'd2) cache_data_fg[cache_idx] <= tile_data;
always @(posedge clk) if (cache_fill && active_client == 3'd0) cache_tag_bg[cache_idx]  <= cache_tag_in;
always @(posedge clk) if (cache_fill && active_client == 3'd1) cache_tag_mg[cache_idx]  <= cache_tag_in;
always @(posedge clk) if (cache_fill && active_client == 3'd2) cache_tag_fg[cache_idx]  <= cache_tag_in;

// Rising edge detection on all 5 clients
reg r0_req_prev, r1_req_prev, r2_req_prev, r3_req_prev, r4_req_prev;

// Pending request bits — set on rising edge, cleared on grant
reg [4:0] pending;

// Latched addresses
reg [23:0] r0_addr_lat, r1_addr_lat, r2_addr_lat, r3_addr_lat, r4_addr_lat;

// Round-robin priority: which client to check first (0-4)
reg [2:0] next_prio;

// Active client being served
reg [2:0] active_client;

// FSM
localparam ARB_IDLE  = 2'd0;
localparam ARB_CHECK = 2'd1;  // cache lookup result ready
localparam ARB_WAIT  = 2'd2;  // waiting SDRAM
localparam ARB_CHECK2 = 2'd3; // esito cache (lettura registrata pronta)
reg [1:0] arb_state;

// Detect rising edges
wire r0_rising = r0_req && !r0_req_prev;
wire r1_rising = r1_req && !r1_req_prev;
wire r2_rising = r2_req && !r2_req_prev;
wire r3_rising = r3_req && !r3_req_prev;
wire r4_rising = r4_req && !r4_req_prev;

// Combinational grant selection (round-robin across 5 clients)
reg [2:0] grant_id;
reg       grant_found;
reg [23:0] grant_addr;

always @(*) begin
	grant_found = 0;
	grant_id    = 3'd0;
	grant_addr  = r0_addr_lat;

	if (hblank && pending[4]) begin
		// HBlank: FG text gets exclusive access for bandwidth
		grant_found = 1;
		grant_id    = 3'd4;
		grant_addr  = r4_addr_lat;
	end else begin
		// Normal: round-robin across all 5 clients
		begin : grant_search
			integer i;
			reg [2:0] check;
			reg [3:0] check_wide;
			for (i = 0; i < 5; i = i + 1) begin
				check_wide = {1'b0, next_prio} + i[3:0];
				check = (check_wide >= 4'd5) ? check_wide[2:0] - 3'd5 : check_wide[2:0];
				if (!grant_found && pending[check]) begin
					grant_id = check;
					case (check)
						3'd0: grant_addr = r0_addr_lat;
						3'd1: grant_addr = r1_addr_lat;
						3'd2: grant_addr = r2_addr_lat;
						3'd3: grant_addr = r3_addr_lat;
						3'd4: grant_addr = r4_addr_lat;
						default: grant_addr = r0_addr_lat;
					endcase
					grant_found = 1;
				end
			end
		end
	end
end

always @(posedge clk) begin
	if (reset) begin
		cache_valid_bg <= 64'b0;
		cache_valid_mg <= 64'b0;
		cache_valid_fg <= 64'b0;
		r0_req_prev  <= 0;
		r1_req_prev  <= 0;
		r2_req_prev  <= 0;
		r3_req_prev  <= 0;
		r4_req_prev  <= 0;
		pending      <= 5'b00000;
		r0_addr_lat  <= 0;
		r1_addr_lat  <= 0;
		r2_addr_lat  <= 0;
		r3_addr_lat  <= 0;
		r4_addr_lat  <= 0;
		next_prio    <= 0;
		active_client <= 0;
		arb_state    <= ARB_IDLE;
		tile_req     <= 0;
		tile_addr    <= 0;
		tile_kind      <= 3'd0;
		r0_data      <= 0;
		r1_data      <= 0;
		r2_data      <= 0;
		r3_data      <= 0;
		r4_data      <= 0;
		r0_valid     <= 0;
		r1_valid     <= 0;
		r2_valid     <= 0;
		r3_valid     <= 0;
		r4_valid     <= 0;
	end else begin
		// Edge detection
		r0_req_prev <= r0_req;
		r1_req_prev <= r1_req;
		r2_req_prev <= r2_req;
		r3_req_prev <= r3_req;
		r4_req_prev <= r4_req;

		// Latch addresses and set pending on rising edge
		if (r0_rising) begin pending[0] <= 1'b1; r0_addr_lat <= r0_addr; end
		if (r1_rising) begin pending[1] <= 1'b1; r1_addr_lat <= r1_addr; end
		if (r2_rising) begin pending[2] <= 1'b1; r2_addr_lat <= r2_addr; end
		if (r3_rising) begin pending[3] <= 1'b1; r3_addr_lat <= r3_addr; end
		if (r4_rising) begin pending[4] <= 1'b1; r4_addr_lat <= r4_addr; end

		// Clear valid pulses (1-cycle)
		r0_valid <= 0;
		r1_valid <= 0;
		r2_valid <= 0;
		r3_valid <= 0;
		r4_valid <= 0;

		// Clear tile_req after 1 cycle (bridge detects rising edge)
		tile_req <= 0;

		case (arb_state)
			ARB_IDLE: begin
				if (grant_found) begin
					active_client  <= grant_id;
					pending[grant_id] <= 1'b0;
					// Latch grant address for cache/SDRAM use next cycle
					tile_addr      <= grant_addr;
					tile_kind      <= grant_id;       // 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
					arb_state      <= ARB_CHECK;
				end
			end

			// La lettura della cache e' registrata: qui l'indirizzo e' appena
			// stato presentato, l'esito arriva al ciclo dopo.
			ARB_CHECK: arb_state <= ARB_CHECK2;

			ARB_CHECK2: begin
				// Cache lookup result is now stable (registered addr from IDLE)
				if (cache_hit && active_client != 3'd3 && active_client != 3'd4) begin
					// Cache hit — deliver data dalla cache per-layer
					case (active_client)
						3'd0: begin r0_data <= cache_hit_data; r0_valid <= 1'b1; end
						3'd1: begin r1_data <= cache_hit_data; r1_valid <= 1'b1; end
						3'd2: begin r2_data <= cache_hit_data; r2_valid <= 1'b1; end
						default: ;
					endcase
					next_prio <= (active_client == 3'd4) ? 3'd0 : active_client + 3'd1;
					arb_state <= ARB_IDLE;
				end else begin
					// Cache miss o sprite/text — fetch da SDRAM
					tile_req  <= 1'b1;
					arb_state <= ARB_WAIT;
				end
			end

			ARB_WAIT: begin
				if (tile_valid) begin
					// Route data to requesting client
					case (active_client)
						3'd0: begin r0_data <= tile_data; r0_valid <= 1'b1; end
						3'd1: begin r1_data <= tile_data; r1_valid <= 1'b1; end
						3'd2: begin r2_data <= tile_data; r2_valid <= 1'b1; end
						3'd3: begin r3_data <= tile_data; r3_valid <= 1'b1; end
						3'd4: begin r4_data <= tile_data; r4_valid <= 1'b1; end
						default: ;
					endcase
					// I dati e i tag li scrivono gli always dedicati sopra
					// (uno per array). Qui restano i soli bit di valid.
					case (active_client)
						3'd0: cache_valid_bg[cache_idx] <= 1'b1;
						3'd1: cache_valid_mg[cache_idx] <= 1'b1;
						3'd2: cache_valid_fg[cache_idx] <= 1'b1;
						default: ;
					endcase
					// Rotate priority: next after active
					next_prio <= (active_client == 3'd4) ? 3'd0 : active_client + 3'd1;
					arb_state <= ARB_IDLE;
				end
			end

			default: arb_state <= ARB_IDLE;
		endcase
	end
end

endmodule
