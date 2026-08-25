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

/*  Tile-layer parametrico Seibu D-Con (16x16, 4bpp, 32x32).

    Modulo unico per BG / MG / FG. Differenze fra layer (da MAME dcon.cpp):

      LAYER  COLOR_BASE  TRANSP  GFX_BANK  TILE_KIND  SCROLL+128
      BG     0x400       no      no        0          sì
      MG     0x500       sì(15)  sì        1          sì
      FG     0x600       sì(15)  no        2          sì

    Parametri:
      COLOR_BASE   : 11-bit base palette
      HAS_TRANSP   : 1=pen 15 trasparente, 0=sempre opaco
      HAS_GFX_BANK : 1=tile_idx |= gfx_bank_select (per MG)
      TILE_KIND    : 3-bit kind passato a SDRAM arbiter

    Architettura: line buffer ping-pong + prefetcher SDRAM (vedi BG renderer
    cancellato — qui generalizzato).

    Decode tile (dcon_tilelayout):
      base_left  = idx*128 + row*4
      base_right = idx*128 + 64 + row*4
      gruppo (col_local 0..3 di sx o dx) — fetch 32-bit a base+offset:
        byte0 = base+0 (plane 2,3)
        byte1 = base+1 (plane 0,1)
      sub      = 3 - col[1:0]
      pen      = {byte_lo[sub+4], byte_lo[sub], byte_hi[sub+4], byte_hi[sub]}
*/

module Raiden2_tile_layer #(
	parameter [10:0] COLOR_BASE   = 11'h400,
	parameter        HAS_TRANSP   = 0,
	parameter        HAS_GFX_BANK = 0,
	parameter  [2:0] TILE_KIND    = 3'd0,
	parameter        MIRROR_H     = 0,
	parameter        MAP_HEIGHT_4 = 0,    // 0=32x32 tilemap (DCon), 1=32x16 (Blood Bros)
	parameter [12:0] TILE_CODE_OFS = 13'd0, // BB FG hardcoded +0x1000 (MAME bloodbro fg_tile_info)
	parameter        PEN_ORDER    = 0      // 0=DCBA (BG/MG), 1=BADC (FG) — verificato HW
) (
	input  wire        clk,
	input  wire        reset,
	input  wire        ce_pix,

	input  wire  [9:0] hpos,
	// Indirizzo di lettura ANTICIPATO di 1 pixel (dal top, flip incluso).
	// raiden2_mixer campiona i livelli su ce_pix_d, cioe' UN CLOCK DOPO
	// ce_pix: a quell'istante `linebuf[hpos]` vale ancora il pixel k-1,
	// perche' l'indirizzo e' cambiato sullo stesso fronte. Gli sprite hanno
	// gia' questo lookahead; i tile no, quindi uscivano indietro di un pixel
	// rispetto a sprite e finestra DE: la colonna 0 del fondo non veniva
	// disegnata e il resto appariva spostato di uno.
	input  wire  [9:0] hpos_rd,
	// vpos REALE, mai specchiata dal top: e' quella che tiene in fase il
	// prefetch e i suoi slot speciali col resto del frame. Il flip si applica
	// piu' sotto alla sola RIGA DI CONTENUTO (target_y). Specchiarla dal top
	// faceva rivivere lo slot inertizzato del top (239-300, negativo, avvolto
	// a 9 bit) e sfasava il contenuto rispetto agli sprite: stessa lezione di
	// Raiden 1 (Raiden_tile_layer.sv, "vpos REALE: il mirror riga lo fa il
	// layer").
	input  wire  [8:0] vpos,
	input  wire        flip_screen,   // DIP Flip Screen
	input  wire        de,
	input  wire        layer_en,
	input  wire        new_line,

	input  wire [15:0] scroll_x,
	input  wire [15:0] scroll_y,

	// OSD offset di rendering (debug pixel-hunting, indipendente dallo scroll content)
	input  wire signed [9:0] xoff,
	input  wire signed [9:0] yoff,

	// Solo per MG: high bits aggiunti al tile_idx
	input  wire [15:0] gfx_bank,

	// VRAM read port (CPU dual-port lato B)
	output reg  [10:0] vram_addr,
	input  wire [15:0] vram_data,

	// SDRAM tile fetch via arbiter
	output reg         rom_req,
	output reg  [23:0] rom_addr,
	input  wire [31:0] rom_data,
	input  wire        rom_valid,

	// Output pixel (combinatoriale: niente latency latch, ultimo pixel a destra
	// mostrato nello stesso ce_pix in cui il game lo richiede).
	output wire        opaque,
	output wire [10:0] pen_index
);

	// ─── Line buffers (ping-pong) ────────────────────────────────────────────
	// 12 bit/pixel: {color[3:0], 4'd0, pen[3:0]} → ricostruito al read in pen_index
	// Bit10 (extra) per "pen=15 transparent" = bit 7 del campo a 12 bit (impostato da decode)
	// Layout: [11:8]=color, [7]=transp_flag, [6:4]=000, [3:0]=pen
	(* ramstyle = "M10K,no_rw_check" *) reg [11:0] linebuf0 [0:319];
	(* ramstyle = "M10K,no_rw_check" *) reg [11:0] linebuf1 [0:319];

	// Unified write per inferenza M10K: 1 SOLO punto di scrittura per linebuf.
	reg        dec_we;
	reg  [8:0] dec_addr;
	reg [11:0] dec_data;
	reg        wr_en;
	reg  [8:0] wr_addr;
	reg [11:0] wr_data;
	always @(*) begin
		if (pf_state == PF_CLEAR) begin
			wr_en   = 1'b1;
			wr_addr = clear_idx;
			wr_data = LB_EMPTY;
		end else begin
			wr_en   = dec_we;
			wr_addr = dec_addr;
			wr_data = dec_data;
		end
	end
	always @(posedge clk) begin
		if (wr_en) begin
			if (active_buf == 1'b0) linebuf1[wr_addr] <= wr_data;
			else                    linebuf0[wr_addr] <= wr_data;
		end
	end
	reg        active_buf;

	// Prefetcher state
	reg  [4:0] tile_col_pf;
	reg  [3:0] pf_state;
	reg [11:0] pf_tile_idx;
	reg  [3:0] pf_tile_clr;
	reg        pf_side;
	reg [31:0] pf_rom_data;
	reg  [3:0] decode_step;

	localparam PF_IDLE    = 4'd0;
	localparam PF_CLEAR   = 4'd9;   // pre-clear linebuf (1 pixel/clk × 320 ciclis)
	localparam PF_VRAM_R  = 4'd1;
	localparam PF_VRAM_W  = 4'd2;
	localparam PF_VRAM_W2 = 4'd3;   // wait extra: BRAM read latency
	localparam PF_ROM_REQ = 4'd4;
	localparam PF_ROM_W   = 4'd5;
	localparam PF_DECODE  = 4'd6;
	localparam PF_NEXT    = 4'd7;
	localparam PF_DONE    = 4'd8;

	// Sentinel "pixel vuoto": transp=1 → opaque=0 al read → mostra layer sotto.
	localparam [11:0] LB_EMPTY = 12'h080;   // transp_flag bit 7 = 1, pen = 0
	reg [8:0] clear_idx;

	// Coordinate prefetch — target = riga da mostrare al new_line successivo.
	//
	// Logica corretta (non più gated VBLANK):
	//   Durante display vpos=N (N<V_VISIBLE-1): prefetch riga N+1 nel buffer
	//     non-attivo. Al new_line N→N+1: swap → display mostra riga N+1.
	//   Durante display vpos=V_VISIBLE-1: prefetch riga 0 (next frame).
	//   Durante VBLANK (vpos >= V_VISIBLE): prefetch IGNORATO (= idle), così
	//     il buffer caricato durante vpos=V_VISIBLE-1 non viene sovrascritto.
	//
	// Toggle buffer al new_line — solo quando entriamo in una riga visibile
	// (vpos=0..V_VISIBLE-1 dopo wrap).
	//
	// PROBLEMA tempistico: la prefetch parte al new_line che inizia riga N+1.
	// Durante riga N+1 il prefetch sta caricando il buffer per riga N+2.
	// Al new_line N+1→N+2 swap → buffer ora-pieno diventa attivo.
	// → FUNZIONA solo se il prefetch finisce ENTRO una riga (= 384 pixel × 16 cicli
	//    = 6144 cicli). Per 21 tile × 2 fetch × ~30 cicli = ~1260 cicli. OK ✓
	//
	// PRIMA RIGA del frame: serve buffer pre-caricato durante VBLANK precedente.
	//   Soluzione: durante riga V_VISIBLE-1 prefetch fa target_y=0 (next frame),
	//   poi VBLANK no toggle. Al wrap V_TOTAL→0 SWAP → buffer ha riga 0 pronta.
	localparam [8:0] V_VISIBLE = 9'd240;  // cupsoc 320x240
	// Flip DIP: si specchia la RIGA DI CONTENUTO, non il tempo. Durante la riga
	// di display L il prefetch prepara L+1, che a schermo capovolto e' la riga
	// sorgente (V_VISIBLE-1)-(L+1) = 238-L. Scritto come 238-vpos invece di
	// 239-(vpos+1) per non mettere due addizionatori in serie sul percorso:
	// identita' aritmetica, stessi valori bit per bit.
	wire [15:0] target_y = flip_screen
	        ? ((vpos == V_VISIBLE - 9'd1) ? 16'd239 : (16'd238 - {7'd0, vpos}))
	        : ((vpos == V_VISIBLE - 9'd1) ? 16'd0   : ({7'd0, vpos} + 16'd1));
	wire [15:0] eff_y_pf = target_y + scroll_y_l + {{6{yoff[9]}}, yoff};
	// MAP_HEIGHT_4=1 → tilemap 32×16 (Blood Bros): tile_y wrap modulo 16 (4 bit).
	// MAP_HEIGHT_4=0 → tilemap 32×32 (DCon/GundamSD): tile_y wrap modulo 32 (5 bit).
	wire  [4:0] tile_y_pf = MAP_HEIGHT_4 ? {1'b0, eff_y_pf[7:4]} : eff_y_pf[8:4];
	wire  [3:0] row_pf    = eff_y_pf[3:0];

	// gated_new_line: trigger prefetch SOLO quando new vpos è visible.
	// Durante VBLANK no swap, no nuovo prefetch.
	wire vpos_visible    = (vpos < V_VISIBLE);
	wire gated_new_line  = new_line & vpos_visible;

	// Scroll/bank LATCHATI A INIZIO RIGA (per-line). PROVATO dal disasm
	// godzilla (ISR IRQ4 $37A): gli scroll sono scritti DENTRO il vblank
	// ($452 bsr $1ED4, dopo il trigger DMA 0x14 a $444) → ogni riga visibile
	// campiona scroll(N) accoppiato al contenuto(N) copiato dallo stesso
	// vblank. Il gioco RIDISEGNA una colonna di tilemap a ogni confine di
	// 16px della camera ($B322/$B346: divu #$10 → colonna nello staging):
	// se contenuto e scroll non sono dello STESSO vblank, pop di 16px a ogni
	// attraversamento (a 8px/frame = "ogni 2 frame salta e torna").
	// ⛔ NON campionare una-volta-per-frame alla riga visibile 223: quella
	// riga precede l'ISR → scroll(N-1) con contenuto(N) = pop garantito.
	// (Il prefetch della prima riga e' spostato nel TARDO VBLANK dal top,
	// dopo l'ISR: vedi vpos_for_pf in Raiden2.sv.)
	reg [15:0] scroll_x_l, scroll_y_l;
	reg [15:0] gfx_bank_l;
	always @(posedge clk) begin
		if (reset) begin
			scroll_x_l <= 16'd0;
			scroll_y_l <= 16'd0;
			gfx_bank_l <= 16'd0;
		end else if (gated_new_line) begin
			scroll_x_l <= scroll_x;
			scroll_y_l <= scroll_y;
			gfx_bank_l <= gfx_bank;
		end
	end

	wire [4:0] first_tile_x   = scroll_x_l[8:4];
	wire [3:0] first_pixel_off = scroll_x_l[3:0];

	wire [4:0] cur_tile_x = first_tile_x + tile_col_pf;
	wire signed [10:0] dst_x_signed = ({1'b0, tile_col_pf, 4'd0}) - {7'd0, first_pixel_off} + {{1{xoff[9]}}, xoff};

	// Tile index con eventuale gfx_bank (MG ha bit 12 = 13-bit totali, BG/FG = 12 bit)
	// + TILE_CODE_OFS hardcoded (BB FG aggiunge 0x1000 secondo MAME bloodbro fg_tile_info).
	wire [12:0] tile_idx_base = HAS_GFX_BANK
	                              ? ({1'b0, pf_tile_idx} | gfx_bank_l[12:0])
	                              : {1'b0, pf_tile_idx};
	wire [12:0] effective_tile_idx = tile_idx_base + TILE_CODE_OFS;

	// ─── Prefetcher main FSM ─────────────────────────────────────────────────
	always @(posedge clk) begin
		if (reset) begin
			pf_state    <= PF_IDLE;
			tile_col_pf <= 5'd0;
			rom_req     <= 1'b0;
			vram_addr   <= 11'd0;
			active_buf  <= 1'b0;
			decode_step <= 4'd0;
			pf_side     <= 1'b0;
			clear_idx   <= 9'd0;
			dec_we      <= 1'b0;
		end else begin
			dec_we <= 1'b0;   // default: rialzato solo in PF_DECODE
			case (pf_state)
				PF_IDLE: begin
					if (gated_new_line) begin
						active_buf  <= ~active_buf;
						tile_col_pf <= 5'd0;
						pf_side     <= 1'b0;
						clear_idx   <= 9'd0;
						pf_state    <= PF_CLEAR;   // pre-clear linebuf
					end
				end

				// CLEAR: azzera 320 entry del buffer non-attivo a LB_EMPTY
				// (transp=1 → opaque=0 al read → mostra layer sotto).
				// Evita "linea persistente" da scanline precedente sui pixel
				// che il prefetch non copre (bordi sinistro/destro con scroll).
				PF_CLEAR: begin
					// write effettivo nel blocco unified-write sotto (inferenza M10K)
					if (clear_idx == 9'd319) begin
						clear_idx <= 9'd0;
						pf_state  <= PF_VRAM_R;
					end else begin
						clear_idx <= clear_idx + 9'd1;
					end
				end

				PF_VRAM_R: begin
					// Emetto vram_addr (registered alla fine di questo ciclo).
					vram_addr <= {1'b0, tile_y_pf, cur_tile_x};
					pf_state  <= PF_VRAM_W;
				end

				PF_VRAM_W: begin
					// vram_addr è effettivo all'inizio di questo ciclo.
					// La BRAM dual_port produce vram_data alla FINE di questo ciclo
					// (registered output). Quindi vram_data sarà valido in PF_VRAM_W2.
					pf_state <= PF_VRAM_W2;
				end

				PF_VRAM_W2: begin
					// vram_data ora valido (= dato di addr emesso in PF_VRAM_R).
					pf_tile_idx <= vram_data[11:0];
					pf_tile_clr <= vram_data[15:12];
					pf_state    <= PF_ROM_REQ;
				end

				PF_ROM_REQ: begin
					// rom_addr = tile_idx*128 + banco*0x80000 + side_off + row*4
					// Raiden II: tile = (vram & 0xfff) | (bank << 12) -> 15 bit
					// (raiden2_v.cpp:162-190 get_back/mid/fore_tile_info; bank 0..7
					//  = 32768 tile = la region "tiles" da 4MB, unica per i 3 layer).
					// idx*128 = idx<<7, quindi il banco pesa bank<<19 sul byte offset
					// (0x80000 = 4096 tile x 128 byte). Sommarlo qui evita di
					// allargare effective_tile_idx e vale perche' pf_tile_idx e' gia'
					// mascherato a 12 bit.
					rom_addr <= ({4'd0, effective_tile_idx, 7'd0})
					           + ({2'd0, gfx_bank_l[2:0], 19'd0})
					           + (pf_side ? 24'd64 : 24'd0)
					           + ({18'd0, row_pf, 2'd0});
					rom_req  <= 1'b1;
					pf_state <= PF_ROM_W;
				end

				PF_ROM_W: begin
					if (rom_valid) begin
						pf_rom_data <= rom_data;
						rom_req     <= 1'b0;
						decode_step <= 4'd0;
						pf_state    <= PF_DECODE;
					end
				end

				PF_DECODE: begin
					begin : decode_blk
						reg [7:0] byte_lo, byte_hi;
						reg [1:0] sub;
						reg [3:0] pen;
						reg signed [10:0] dx;
						reg        transp;
						if (decode_step[2] == 1'b0) begin
							byte_lo = pf_rom_data[31:24];
							byte_hi = pf_rom_data[23:16];
						end else begin
							byte_lo = pf_rom_data[15:8];
							byte_hi = pf_rom_data[7:0];
						end
						sub = 2'd3 - decode_step[1:0];   // 3..0 per col 0..3
						// Plane order verificato su HW: BG/MG = DCBA (PEN_ORDER=0),
						// FG = BADC (PEN_ORDER=1). Sorgenti A,B,C,D.
						begin : pen_order_blk
							reg [3:0] s;
							s[0] = byte_lo[7 - {1'b0, sub}];  // A
							s[1] = byte_lo[3 - {1'b0, sub}];  // B
							s[2] = byte_hi[7 - {1'b0, sub}];  // C
							s[3] = byte_hi[3 - {1'b0, sub}];  // D
							if (PEN_ORDER == 1)
								pen = {s[2],s[3],s[0],s[1]};  // BADC (FG)
							else
								pen = {s[0],s[1],s[2],s[3]};  // DCBA (BG/MG)
						end
						transp = (HAS_TRANSP != 0) && (pen == 4'd15);
						dx = dst_x_signed + (pf_side ? 11'sd8 : 11'sd0) + {8'd0, decode_step[2:0]};
						dec_we   <= (dx >= 0 && dx < 320);
						dec_addr <= dx[8:0];
						dec_data <= {pf_tile_clr, transp, 3'd0, pen};
					end
					if (decode_step == 4'd7) begin
						pf_state <= PF_NEXT;
					end else begin
						decode_step <= decode_step + 4'd1;
					end
				end

				PF_NEXT: begin
					if (pf_side == 1'b0) begin
						pf_side  <= 1'b1;
						pf_state <= PF_ROM_REQ;
					end else begin
						pf_side <= 1'b0;
						if (tile_col_pf == 5'd20) begin
							pf_state <= PF_DONE;
						end else begin
							tile_col_pf <= tile_col_pf + 5'd1;
							pf_state    <= PF_VRAM_R;
						end
					end
				end

				PF_DONE: begin
					if (gated_new_line) begin
						active_buf  <= ~active_buf;
						tile_col_pf <= 5'd0;
						pf_side     <= 1'b0;
						pf_state    <= PF_VRAM_R;
					end
				end

				default: pf_state <= PF_IDLE;
			endcase
		end
	end

	// ─── Read side SINCRONO (M10K) ──────────────────────────────────────────
	// 1 read dedicato per buffer (no mux dentro l'indirizzo) → M10K inferibile.
	reg [11:0] rd0_r, rd1_r;
	reg        active_buf_r;
	reg        de_r;
	reg  [9:0] hpos_r;
	always @(posedge clk) begin
		rd0_r        <= linebuf0[hpos_rd[8:0]];
		rd1_r        <= linebuf1[hpos_rd[8:0]];
		active_buf_r <= active_buf;
		de_r         <= de;
		hpos_r       <= hpos;
	end
	wire [11:0] read_data_r = active_buf_r ? rd1_r : rd0_r;
	wire  [3:0] read_color = read_data_r[11:8];
	wire        read_transp = read_data_r[7];
	wire  [3:0] read_pen   = read_data_r[3:0];

	// ⛔ NON campionare questo percorso su ce_pix: raiden2_mixer campiona i
	// livelli su ce_pix_d (un clock DOPO ce_pix) proprio perche' i tile layer
	// si aggiornano a ogni clk. Mettendo qui uno stadio su ce_pix il gate
	// (hpos_r/de_r) resta indietro di un pixel rispetto al dato e la COLONNA 0
	// non viene disegnata. Provato e scartato il 2026-08-24.
	// Gate sull'indice NON ritardato: al campionamento del mixer (ce_pix_d)
	// `hpos`/`de` valgono gia' il pixel k, come il dato letto con hpos_rd.
	// Usare hpos_r/de_r qui rimetterebbe indietro il gate di un pixel e
	// spegnerebbe di nuovo la colonna 0.
	wire pixel_active = de & layer_en & (hpos < 10'd320) & ~read_transp;
	assign opaque    = pixel_active;
	assign pen_index = pixel_active ? (COLOR_BASE + {3'd0, read_color, read_pen}) : 11'd0;

endmodule
