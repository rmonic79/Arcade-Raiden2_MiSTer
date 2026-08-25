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

/*  Sprite renderer Seibu SEI252 / RISE (Raiden2 family).

    Derivato dal sprite renderer SEI0211 di GundamSD/DCon. Formato spriteram
    quasi identico (entrambi MAME sei021x_sei0220_spr.cpp) — differenze:
      - Spriteram @ 0x105000-0x105FFF (4KB, 1024 word, 256 entry × 4 word)
      - w3 bit 15 = tile bank extra bit (denjinmk usa, Raiden2 base = 0)

    Format word (legionna_v.cpp:288-312):
      w0 bit 15      = enable (0=draw, 1=skip — TODO verify Raiden2)
      w0 bit 14      = flip_x
      w0 bit 13      = flip_y (??? non sicuro in Raiden2)
      w0 bit 12..10  = sizex (3-bit, +1) → 1..8 tile larghezza
      w0 bit 9..7    = sizey (3-bit, +1) → 1..8 tile altezza
      w0 bit 6       = tile bank bit (denjinmk) / extra priority pin (cupsoc)
      w0 bit 5..0    = color (6-bit)
      w1 bit 15..14  = priority code (2-bit → m_sprite_pri_mask[pri])
      w1 bit 13..0   = tile_code (14-bit)
      w2 bit 11..0   = X (signed 12-bit con sign extension a 9-bit)
      w3 bit 15      = tile bank extra (Denjin Makai)
      w3 bit 11..0   = Y (signed 12-bit)

    Tile size 16×16 4bpp = 128 byte/tile = 32 word 16-bit.

    Priority callback legionna_v.cpp:314-317:
      pri_mask[0] = 0x0000 (sopra tutto)
      pri_mask[1] = 0xFFF0
      pri_mask[2] = 0xFFFC
      pri_mask[3] = 0xFFFE

    Pen 15 = trasparente.

    Architettura:
      - Sprite scan FSM durante linea N: scorre 256 entry, per quelle che
        intersecano linea N+1 (target_y), fetch SDRAM dei tile coperti e
        scrive in line buffer non-attivo.
      - Read side: a hpos legge line buffer attivo, restituisce pen+pri_code.
      - Ping-pong al new_line.

    ─── Ottimizzazioni performance (no logic change) ────────────────────
    Line buffer = 8 bank interleaved da 40×14 (320 pixel totali, lane =
    dx[2:0], riga = dx[8:3]). Permette:
      - SC_DECODE in 1 ciclo invece di 8: tutti gli 8 pixel della mezza-
        riga scritti in parallelo (un write per bank).
      - SC_CLEAR in 40 cicli invece di 320: 8 entry azzerate/ciclo.
    Early-skip entry: legge solo w0/w3, valuta enable&&in_y&&layer_en,
    e fetcha w1/w2 SOLO se lo sprite è visibile. Risparmio ~3 cicli sulla
    maggioranza degli slot (disabled/off-screen). Stesso pattern Blood Bros.

    Worst case: 256 entry × max 8 sizex × 1 fetch/tile = 2048 fetch/linea.
    Realistico: 30 sprite × 4 sizex avg = 120 fetch. Banda OK.
*/

module Raiden2_sprite_renderer (
	input  wire        clk,
	input  wire        reset,
	input  wire        ce_pix,

	input  wire  [9:0] hpos,        // 0..319 logico
	input  wire  [8:0] vpos,        // 0..239 logico, REALE (mai specchiata dal top)
	// DIP Flip Screen. Il read-side X e' gia' specchiato dal top (hpos), qui
	// si specchia la RIGA BERSAGLIO dello scan: senza, in flip lo sfondo si
	// capovolgeva e gli sprite no, e sembravano andare al contrario.
	input  wire        flip_screen,
	input  wire        de,
	input  wire        layer_en,
	input  wire        new_line,

	// OSD offset di rendering (debug pixel-hunting)
	input  wire signed [9:0] xoff,
	input  wire signed [9:0] yoff,

	// Sprite RAM read port (dual-port lato B)
	output reg  [10:0] spr_addr,    // 2048 word total (512 entry × 4 word)
	input  wire [15:0] spr_data,

	// SDRAM tile fetch via arbiter (client r3, kind=3, no cache)
	output reg         rom_req,
	output reg  [23:0] rom_addr,
	input  wire [31:0] rom_data,
	input  wire        rom_valid,

	// Output pixel (combinatorio come i tile layer: stessa latenza = no shift)
	output wire        opaque,
	output wire [10:0] pen_index,   // sprite color base = 0 (palette[0..1023])
	output wire  [1:0] pri_code     // 2-bit priority dal w1
);

	// ─── Line buffer ping-pong (8 bank interleaved per parallel write) ──
	// Layout 14-bit: [13:8]=color, [7:6]=pri_code, [5:4]=00, [3:0]=pen
	// Valid pixel = pen != 0xF (sentinel "no sprite") — usiamo 0xF come "vuoto"
	// perché pen 15 reale = trasparente (skipped al draw).
	// Bank b contiene i pixel con dx[2:0]==b → indicizzato da dx[8:3] (0..39).
	// Permette 8 write paralleli in 1 ciclo (decode 8 pixel/ciclo) e clear
	// in 32 cicli invece di 256. Vista esterna: identica al singolo array.
	localparam [13:0] LB_EMPTY = 14'h003F;  // color=0, pri=0, pen=15 (trasparente)
	// Line buffer = 16 BRAM (8 lane x 2 buffer ping-pong) via spram_dp =
	// altsyncram M10K GARANTITO (no ALM). Ogni bank: 32 entry x 14 bit.
	// Write port muxato clear/decode; read port a rd_row. read latency = 1.
	reg active_buf;

	// ─── Sprite scan FSM ─────────────────────────────────────────────────────
	// Stati: IDLE → CLEAR (azzera buffer non-attivo a pen=15) → RW0/RW1/RW3
	// (early-read w0,w3) → CHECK → RW2/CHECK2/CHECK2_W2 (read w1,w2 se visibile)
	// → ROM_REQ/W → DECODE → NEXT_TX/E → DONE.
	localparam SC_IDLE      = 4'd0;
	localparam SC_CLEAR     = 4'd1;
	localparam SC_POP       = 4'd2;   // attende un match dalla scan-ombra
	localparam SC_POP_W     = 4'd3;   // dato della FIFO valido (lettura registrata)
	localparam SC_ROM_REQ   = 4'd8;
	localparam SC_ROM_W     = 4'd9;
	localparam SC_DECODE    = 4'd10;
	localparam SC_NEXT_TX   = 4'd11;
	localparam SC_NEXT_E    = 4'd12;
	localparam SC_DONE      = 4'd13;

	reg [3:0] sc_state;
	// CUPSOC: spriteram 2KB → 256 entry (MAME size/2/4 = 2048/2/4 = 256).
	reg [8:0] entry_idx;       // 0..255
	reg [5:0] clear_idx;       // 0..39 per clear buffer (8 lane in parallelo, 320 px)
	reg [15:0] sp_w0, sp_w1, sp_w2, sp_w3;
	reg        pf_side;        // 0=metà sx tile (col 0..7), 1=metà dx (col 8..15)

	// Decoded fields — MAME sei0210_device::draw (non-alt format, riga 143):
	//   if (BIT(~spriteram[i], 15)) continue;  → draw quando w0[15]==1.
	// ─── Formato SEI25X_RISE1X (MAME sei25x_rise1x_spr.cpp:88-101) ──────────
	// Raiden II NON usa il formato SEI0211 della famiglia legionna: i campi
	// stanno su bit diversi. Trascritto dal device:
	//   w0[15]    flipy        w0[14:12] sizey-1     w0[11]   flipx
	//   w0[10:8]  sizex-1      w0[7:6]   pri         w0[5:0]  color
	//   w1[15:0]  code (16 bit pieni, non 14)
	//   w2[12]    ext          w2[8:0]   X
	//   w3[8:0]   Y
	// Non esiste un bit di enable: il device salta l'entry quando il codice e'
	// zero ((code % gfx elements) == 0, riga 90).
	wire        sp_flipy  = sp_w0[15];
	wire  [2:0] sp_sizey  = sp_w0[14:12];      // +1 → 1..8 tile
	wire        sp_flipx  = sp_w0[11];
	wire  [2:0] sp_sizex  = sp_w0[10:8];       // +1
	wire  [5:0] sp_color  = sp_w0[5:0];
	// cupsoc_pri_cb (legionna_v.cpp:319): pri = (w1[15:14] & 2) | (ext & 1)
	// → livello = {w1[15], w0[6]}. Bit14 di w1 inutilizzato ("side effect of
	// using the COP sprite DMA").
	// CUPSOC: pri_cb STANDARD (legionna_v.cpp pri_cb = mask[pri]) → livello =
	// w1[15:14] pieno, NON il {w1[15],ext} di grainbow (che ha un pri_cb suo).
	wire  [1:0] sp_pri    = sp_w0[7:6];
	wire [15:0] sp_code   = sp_w1;
	wire        sp_ext    = sp_w2[12];
	// Nessun bit di enable nel formato SEI25X: code 0 = entry vuota.
	// (il test "entry vuota" e' fatto direttamente in SC_CHECK2 sul dato appena
	//  letto: qui sp_w1 non sarebbe ancora valido)
	// SEI0211 get_coordinate (sei021x_sei0220_spr.h):
	//   coord &= 0x1ff;
	//   return (coord >= 0x180) ? coord - 0x200 : coord;
	// Range effettivo: -128..+383 (positivi 0..0x17F, negativi 0x180..0x1FF).
	wire [8:0] sp_xraw = sp_w2[8:0];
	wire [8:0] sp_yraw = sp_w3[8:0];
	wire signed [10:0] sp_x_raw = (sp_xraw >= 9'h180) ? ({2'b00, sp_xraw} - 11'h200) : {2'b00, sp_xraw};
	wire signed [10:0] sp_y_raw = (sp_yraw >= 9'h180) ? ({2'b00, sp_yraw} - 11'h200) : {2'b00, sp_yraw};
	// MAME SEI0211 default m_xoffset/m_yoffset = 0 (sei021x_sei0220_spr.cpp:40-41).
	// Raiden2 NON chiama set_offset (legionna.cpp:1214) → offset 0,0.
	// +1 pixel a destra: offset verificato con pixel hunting su MAME (rmonic79).
	// L'offset (xoff+1) è REGISTRATO fuori dal path critico sp_w2→linebuffer: una
	// seconda addizione su sp_x (sp_x_raw + xoff + 1) allungava la catena combinatoria
	// più lunga del design (slack -0.5ns su sp_w2) → metastabilità sull'indirizzo del
	// line buffer → sprite scritti a X instabile = MOONWALK (martello che scivola).
	// xoff cambia solo da OSD (statico durante il rendering): 1 ciclo di latenza ok.
	reg signed [10:0] xoff_eff_r;
	always @(posedge clk) xoff_eff_r <= {xoff[9], xoff} + 11'sd1;
	wire signed [10:0] sp_x = sp_x_raw + xoff_eff_r;
	wire signed [10:0] sp_y = sp_y_raw + {yoff[9], yoff};

	wire [3:0] sp_w  = {1'b0, sp_sizex} + 4'd1;    // 1..8
	wire [3:0] sp_h  = {1'b0, sp_sizey} + 4'd1;    // 1..8

	// Target Y per la linea che stiamo prefetchando (= linea corrente + 1, wrap)
	// Flip DIP: la riga di display L+1 corrisponde alla riga sorgente
	// 239-(L+1), cioe' 238-L. Scritto cosi' invece che 239-(vpos+1) per non
	// mettere DUE addizionatori in serie davanti alla catena che porta a
	// rom_addr: era il cono peggiore del fitter (-1.185, vpos[2] ->
	// rom_addr[21]). Identita' aritmetica, stessi valori bit per bit:
	//   239 - (vpos+1) == 238 - vpos      e il caso vpos==239 va a 239.
	wire [8:0] target_y = flip_screen ? ((vpos == 9'd239) ? 9'd239 : (9'd238 - vpos))
	                                  : ((vpos == 9'd239) ? 9'd0   : (vpos + 9'd1));

	// Sprite intersect check: target_y in [sp_y, sp_y + sp_h*16)
	wire signed [10:0] dy_top = {2'b00, target_y} - sp_y;
	wire        in_y     = (dy_top >= 0) && (dy_top < {3'd0, sp_h, 4'd0});  // sp_h*16
	wire  [3:0] tile_y_in = dy_top[7:4];   // tile row 0..7 dentro sprite
	wire  [3:0] row_in    = dy_top[3:0];   // row dentro tile 0..15
	// flip Y
	wire  [3:0] eff_tile_y = sp_flipy ? (sp_h - 4'd1 - tile_y_in) : tile_y_in;
	wire  [3:0] eff_row    = sp_flipy ? (4'd15 - row_in)          : row_in;

	// Iteratore tile_x
	reg  [3:0] tile_x_pf;
	reg [31:0] pf_rom_data;
	wire  [3:0] eff_tile_x = sp_flipx ? (sp_w - 4'd1 - tile_x_pf) : tile_x_pf;

	// Tile code finale Y-MAJOR (MAME draw_internal: ax=outer, ay=inner, code++):
	//   sub_index = ax*sizey + ay  →  cur_tile = code + eff_tile_x*sp_h + eff_tile_y
	// Raiden II: NESSUN gfxbank_cb in raiden2.cpp -> il codice va usato tale e
	// quale, ed e' a 16 bit pieni (non 14 come nel formato SEI0211). La region
	// sprite e' 8MB = 0x10000 tile da 128 byte, quindi i 16 bit servono tutti.
	wire [15:0] code_banked = sp_code;
	wire [15:0] cur_tile = code_banked + ({12'd0, eff_tile_x} * {12'd0, sp_h}) + {12'd0, eff_tile_y};

	// new_line gating
	wire vpos_visible = (vpos < 9'd240);   // cupsoc: 240 righe
	wire gated_new_line = new_line & vpos_visible;

	// ─── Parallel decode (8 pixel della mezza-riga in un colpo) ──────────────
	// dcon_tilelayout: rom_data 32-bit = 8 pixel per row.
	// pf_side=0 → pixel col 0..7, pf_side=1 → col 8..15 (offset +64 byte).
	// k<4 → byte alti del word, k>=4 → byte bassi.
	//   byte_lo = pf_rom_data[31:24]/[15:8], byte_hi = [23:16]/[7:0]
	// Raiden II NON usa il tilelayout planare della famiglia legionna: usa
	// gfx_16x16x4_packed_lsb (raiden2.cpp:1058), cioe' 4bpp PACKED, un nibble
	// per pixel, 8 byte per riga da 16 pixel.
	// Con readbit() di MAME (0x80>>(bit%8)), planeoffset {0,1,2,3} e xoffset
	// {1*4,0*4,3*4,2*4,...}: pixel PARI = nibble BASSO del byte, pixel DISPARI
	// = nibble ALTO dello stesso byte, senza rimescolare bit.
	// ddram_sprite non swappa piu': la dword arriva little-endian nativa, cioe'
	//   dout[7:0]=byte B+0  [15:8]=B+1  [23:16]=B+2  [31:24]=B+3
	// e ogni byte porta due pixel: pari nel nibble BASSO, dispari nell'ALTO.
	function [3:0] pen_at;
		input integer k;
		begin
			case (k)
				0: pen_at = pf_rom_data[3:0];     // byte B+0 nibble basso
				1: pen_at = pf_rom_data[7:4];     // byte B+0 nibble alto
				2: pen_at = pf_rom_data[11:8];    // byte B+1 nibble basso
				3: pen_at = pf_rom_data[15:12];   // byte B+1 nibble alto
				4: pen_at = pf_rom_data[19:16];   // byte B+2 nibble basso
				5: pen_at = pf_rom_data[23:20];   // byte B+2 nibble alto
				6: pen_at = pf_rom_data[27:24];   // byte B+3 nibble basso
				default: pen_at = pf_rom_data[31:28]; // byte B+3 nibble alto
			endcase
		end
	endfunction

	wire [3:0] pen0 = pen_at(0);
	wire [3:0] pen1 = pen_at(1);
	wire [3:0] pen2 = pen_at(2);
	wire [3:0] pen3 = pen_at(3);
	wire [3:0] pen4 = pen_at(4);
	wire [3:0] pen5 = pen_at(5);
	wire [3:0] pen6 = pen_at(6);
	wire [3:0] pen7 = pen_at(7);

	// Posizione X del primo pixel della mezza-riga sullo schermo.
	//   dx = sp_x + tile_x_pf*16 + col_in_tile, col_in_tile = pf_side*8 + k
	//   flipX: col_in_tile = 15 - col_in_tile
	wire signed [10:0] base_x = sp_x + ({6'd0, tile_x_pf, 4'd0});

	function signed [10:0] dx_at;
		input integer k;
		reg [4:0] eff_col;
		begin
			eff_col = sp_flipx ? (5'd15 - {pf_side, k[2:0]})
			                   : {pf_side, k[2:0]};
			dx_at = base_x + {6'd0, eff_col};
		end
	endfunction

	wire signed [10:0] dx0 = dx_at(0);
	wire signed [10:0] dx1 = dx_at(1);
	wire signed [10:0] dx2 = dx_at(2);
	wire signed [10:0] dx3 = dx_at(3);
	wire signed [10:0] dx4 = dx_at(4);
	wire signed [10:0] dx5 = dx_at(5);
	wire signed [10:0] dx6 = dx_at(6);
	wire signed [10:0] dx7 = dx_at(7);

	// Mask "scrivibile": pen != 15 e dx in [0,320) (320 pixel visibili)
	wire wr0 = (pen0 != 4'd15) && (dx0 >= 0) && (dx0 < 320);
	wire wr1 = (pen1 != 4'd15) && (dx1 >= 0) && (dx1 < 320);
	wire wr2 = (pen2 != 4'd15) && (dx2 >= 0) && (dx2 < 320);
	wire wr3 = (pen3 != 4'd15) && (dx3 >= 0) && (dx3 < 320);
	wire wr4 = (pen4 != 4'd15) && (dx4 >= 0) && (dx4 < 320);
	wire wr5 = (pen5 != 4'd15) && (dx5 >= 0) && (dx5 < 320);
	wire wr6 = (pen6 != 4'd15) && (dx6 >= 0) && (dx6 < 320);
	wire wr7 = (pen7 != 4'd15) && (dx7 >= 0) && (dx7 < 320);

	// Bank di destinazione per ogni step: dx[2:0]. Lane address: dx[8:3] (0..39).
	wire [2:0] ln0 = dx0[2:0];   wire [5:0] rw0_a = dx0[8:3];
	wire [2:0] ln1 = dx1[2:0];   wire [5:0] rw1_a = dx1[8:3];
	wire [2:0] ln2 = dx2[2:0];   wire [5:0] rw2_a = dx2[8:3];
	wire [2:0] ln3 = dx3[2:0];   wire [5:0] rw3_a = dx3[8:3];
	wire [2:0] ln4 = dx4[2:0];   wire [5:0] rw4_a = dx4[8:3];
	wire [2:0] ln5 = dx5[2:0];   wire [5:0] rw5_a = dx5[8:3];
	wire [2:0] ln6 = dx6[2:0];   wire [5:0] rw6_a = dx6[8:3];
	wire [2:0] ln7 = dx7[2:0];   wire [5:0] rw7_a = dx7[8:3];

	// Dato da scrivere per ogni step (formato linebuf 14-bit)
	wire [13:0] wd0 = {sp_color, sp_pri, 2'd0, pen0};
	wire [13:0] wd1 = {sp_color, sp_pri, 2'd0, pen1};
	wire [13:0] wd2 = {sp_color, sp_pri, 2'd0, pen2};
	wire [13:0] wd3 = {sp_color, sp_pri, 2'd0, pen3};
	wire [13:0] wd4 = {sp_color, sp_pri, 2'd0, pen4};
	wire [13:0] wd5 = {sp_color, sp_pri, 2'd0, pen5};
	wire [13:0] wd6 = {sp_color, sp_pri, 2'd0, pen6};
	wire [13:0] wd7 = {sp_color, sp_pri, 2'd0, pen7};

	reg        bank_we [0:7];
	reg [5:0]  bank_row [0:7];
	reg [13:0] bank_wd  [0:7];

	always @(*) begin : bank_select
		integer b;
		for (b = 0; b < 8; b = b + 1) begin
			bank_we[b]  = 1'b0;
			bank_row[b] = 6'd0;
			bank_wd[b]  = 14'd0;
		end
		// Priority encoder: step basso vince in caso di collisione di lane (raro).
		// ── Riscritto 2026-08-16: SEMANTICA IDENTICA, cono molto piu' corto ──
		// Prima le 8 assegnazioni erano in ordine CRESCENTE e ogni step doveva
		// escludersi a mano contro TUTTI i precedenti: 28 comparatori di lane
		// (ln0==ln1, ln0==ln2, ln1==ln2, ...) in cascata davanti a bank_row, che
		// pilota l'INDIRIZZO della BRAM del line buffer. La STA della build 21
		// (output_files/critical_paths_v21c.txt) conta 24485 path negativi di
		// forma sp_w2[*] -> spram_dp:gen_bank[*].u_lb*, worst -1.287 ns: e' la
		// famiglia dominante e l'unica REALE del design, perche' questo blocco
		// gira a CLOCK PIENO (nessun clock-enable, vedi la FSM a riga 342+).
		// In un always @(*) vince l'ULTIMA assegnazione: assegnando in ordine
		// DECRESCENTE, wr0 scrive per ultimo e quindi vince su tutti — la stessa
		// identica priorita' di prima ("step basso vince"), con ZERO comparatori.
		// Equivalenza: per ogni bank b il valore finale e' quello dello step N
		// piu' PICCOLO con (wrN && lnN==b), in entrambe le forme.
		if (wr7) begin bank_we[ln7] = 1'b1; bank_row[ln7] = rw7_a; bank_wd[ln7] = wd7; end
		if (wr6) begin bank_we[ln6] = 1'b1; bank_row[ln6] = rw6_a; bank_wd[ln6] = wd6; end
		if (wr5) begin bank_we[ln5] = 1'b1; bank_row[ln5] = rw5_a; bank_wd[ln5] = wd5; end
		if (wr4) begin bank_we[ln4] = 1'b1; bank_row[ln4] = rw4_a; bank_wd[ln4] = wd4; end
		if (wr3) begin bank_we[ln3] = 1'b1; bank_row[ln3] = rw3_a; bank_wd[ln3] = wd3; end
		if (wr2) begin bank_we[ln2] = 1'b1; bank_row[ln2] = rw2_a; bank_wd[ln2] = wd2; end
		if (wr1) begin bank_we[ln1] = 1'b1; bank_row[ln1] = rw1_a; bank_wd[ln1] = wd1; end
		if (wr0) begin bank_we[ln0] = 1'b1; bank_row[ln0] = rw0_a; bank_wd[ln0] = wd0; end
	end

	// ══ SCAN-OMBRA ═══════════════════════════════════════════════════════════
	// Prima c'era UNA sola FSM: percorreva le 512 entry E disegnava. Lo scarto
	// costa 4 clock per entry = 2.048 clock dei 6.149 di riga, piu' ~600 per le
	// entry che passano il pre-filtro e vengono scartate dopo sul codice: il
	// disegno partiva con ~3.200 clock utili e nelle scene affollate la riga
	// veniva troncata -> scanline di sprite perse.
	// Ora sono DUE FSM concorrenti sulla STESSA riga bersaglio (vpos+1): lo scan
	// riempie una FIFO di match, il disegno la insegue. I 2.600 clock di
	// scansione si sovrappongono al disegno invece di precederlo.
	// Una sola FIFO, niente ping-pong: bersaglio comune, il draw e' sempre
	// dietro allo scan.
	localparam SS_IDLE=4'd0, SS_RW0=4'd1, SS_RW1=4'd2, SS_RW3=4'd3, SS_CHECK=4'd4,
	           SS_RW2=4'd5, SS_CHECK2=4'd6, SS_PUSH=4'd7, SS_DONE=4'd8;
	reg [3:0]  ss_state;
	reg [15:0] sc_w0, sc_w1, sc_w3;
	reg        scan_done;

	// geometria lato scan (solo cio' che serve a decidere in_y)
	wire  [8:0] s_yraw = sc_w3[8:0];
	wire signed [10:0] s_y_raw = (s_yraw >= 9'h180) ? ({2'b00, s_yraw} - 11'h200)
	                                                : {2'b00, s_yraw};
	wire signed [10:0] s_y  = s_y_raw + {yoff[9], yoff};
	wire  [3:0] s_h  = {1'b0, sc_w0[14:12]} + 4'd1;
	wire signed [10:0] s_dy = {2'b00, target_y} - s_y;
	wire        in_y_s = (s_dy >= 0) && (s_dy < {3'd0, s_h, 4'd0});

	// FIFO dei match: 128 x 64 bit = 1 M10K. Piena -> back-pressure sullo scan,
	// cioe' il caso peggiore resta quello di prima, mai peggio.
	(* ramstyle = "M10K,no_rw_check" *) reg [63:0] mfifo [0:127];
	reg  [7:0] fifo_wr, fifo_rd;
	reg [63:0] fifo_q;
	wire [7:0] fifo_cnt   = fifo_wr - fifo_rd;
	wire       fifo_full  = (fifo_cnt == 8'd128);
	wire       fifo_empty = (fifo_wr == fifo_rd);
	wire       fifo_push  = (ss_state == SS_PUSH) && !fifo_full;

	always @(posedge clk) if (fifo_push) mfifo[fifo_wr[6:0]] <= {sc_w0, sc_w1, spr_data, sc_w3};
	always @(posedge clk) fifo_q <= mfifo[fifo_rd[6:0]];

	always @(posedge clk) begin
		if (reset) begin
			ss_state <= SS_IDLE; entry_idx <= 9'd511; spr_addr <= 11'd0;
			fifo_wr  <= 8'd0;    scan_done <= 1'b0;
		end else if (gated_new_line) begin
			ss_state  <= SS_RW0;
			entry_idx <= 9'd511;
			spr_addr  <= {9'd511, 2'd0};
			fifo_wr   <= 8'd0;
			scan_done <= 1'b0;
		end else begin
			case (ss_state)
				SS_RW0: begin spr_addr <= {entry_idx, 2'd3}; ss_state <= SS_RW1; end
				SS_RW1: begin sc_w0 <= spr_data;             ss_state <= SS_RW3; end
				SS_RW3: begin sc_w3 <= spr_data;             ss_state <= SS_CHECK; end
				SS_CHECK: begin
					if (in_y_s && layer_en) begin
						spr_addr <= {entry_idx, 2'd1};
						ss_state <= SS_RW2;
					end else begin
						if (entry_idx == 9'd0) begin scan_done <= 1'b1; ss_state <= SS_DONE; end
						else begin
							entry_idx <= entry_idx - 9'd1;
							spr_addr  <= {entry_idx - 9'd1, 2'd0};
							ss_state  <= SS_RW0;
						end
					end
				end
				SS_RW2:   begin spr_addr <= {entry_idx, 2'd2}; ss_state <= SS_CHECK2; end
				SS_CHECK2: begin
					sc_w1 <= spr_data;                 // w1 = code
					if (spr_data == 16'd0) begin       // entry vuota: stesso test di prima
						if (entry_idx == 9'd0) begin scan_done <= 1'b1; ss_state <= SS_DONE; end
						else begin
							entry_idx <= entry_idx - 9'd1;
							spr_addr  <= {entry_idx - 9'd1, 2'd0};
							ss_state  <= SS_RW0;
						end
					end else ss_state <= SS_PUSH;
				end
				SS_PUSH: begin                          // spr_data = w2, spinto insieme
					if (!fifo_full) begin
						fifo_wr <= fifo_wr + 8'd1;
						if (entry_idx == 9'd0) begin scan_done <= 1'b1; ss_state <= SS_DONE; end
						else begin
							entry_idx <= entry_idx - 9'd1;
							spr_addr  <= {entry_idx - 9'd1, 2'd0};
							ss_state  <= SS_RW0;
						end
					end
				end
				default: ;                              // SS_IDLE / SS_DONE: fermi
			endcase
		end
	end

	always @(posedge clk) begin
		if (reset) begin
			sc_state    <= SC_IDLE;
			tile_x_pf   <= 4'd0;
			rom_req     <= 1'b0;
			active_buf  <= 1'b0;
			clear_idx   <= 6'd0;
			fifo_rd     <= 8'd0;
		end else if (gated_new_line && sc_state != SC_IDLE && sc_state != SC_DONE) begin
			// OVERFLOW DI LINEA: la scansione non ha fatto in tempo.
			// Prima new_line veniva raccolto SOLO in SC_IDLE/SC_DONE: la FSM
			// tirava dritto sulla linea vecchia e il ping-pong dei buffer si
			// disallineava, quindi il disturbo restava a schermo finche' non
			// rientrava da solo. Succede con gli sprite larghi (le facce sono
			// fino a 8 tile: 8 fetch DDR3 ciascuna) quando il budget di
			// 436*16 = 6976 cicli per linea si esaurisce.
			// L'hardware in overflow PERDE sprite, non desincronizza: qui si
			// abortisce e si riparte pulti sulla linea nuova. Si perdono gli
			// sprite di coda (indici bassi = quelli davanti, come sul chip),
			// ma il buffer resta in fase.
			active_buf <= ~active_buf;
			tile_x_pf  <= 4'd0;
			clear_idx  <= 6'd0;
			fifo_rd    <= 8'd0;
			// ⚠ OBBLIGATORIO (2026-08-18): azzerare ANCHE rom_req.
			// rom_req scende in UN SOLO punto, SC_ROM_W quando arriva rom_valid
			// (riga ~478). Se l'overflow abortisce mentre siamo in SC_ROM_W —
			// cioe' lo stato in cui la FSM passa la maggior parte del tempo —
			// rom_req resta ALTO. Al giro successivo SC_ROM_REQ rifa
			// `rom_req <= 1'b1` su un segnale gia' alto: NESSUN FRONTE.
			// tile_rom_arbiter registra le richieste solo sul fronte di salita
			// (r3_rising = r3_req && !r3_req_prev), quindi non mette nulla in
			// pending, non risponde mai, e la FSM resta appesa in SC_ROM_W.
			// Alla linea dopo riscatta l'overflow (stato != IDLE/DONE) e si
			// riblocca: gli sprite spariscono DEFINITIVAMENTE, mentre il buffer
			// continua a essere pulito a ogni linea = schermo senza sprite.
			// Sintomo riportato: "dopo che si perde, gli sprite scompaiono per
			// sempre" (la morte riempie lo schermo di esplosioni -> overflow).
			rom_req    <= 1'b0;
			sc_state   <= SC_CLEAR;
		end else begin
			case (sc_state)
				SC_IDLE: begin
					if (gated_new_line) begin
						active_buf <= ~active_buf;
						// Scan INVERSO 511→0 (verificato su HW): lo fa la scan-ombra,
						// e la FIFO conserva quell'ordine, quindi con last-write-wins
						// l'entry 0 scrive per ultima e vince → entry 0 sopra.
						tile_x_pf  <= 4'd0;
						clear_idx  <= 6'd0;
						fifo_rd    <= 8'd0;
						sc_state   <= SC_CLEAR;
					end
				end

				// CLEAR: 8 lane in parallelo → 40 cicli per 320 pixel (LB_EMPTY,
				// pen=15 trasparente). Buffer scritto = quello NON attivo.
				SC_CLEAR: begin
					// write delle BRAM pilotato fuori dalla FSM (vedi bank_*_we/addr/data
					// + istanze spram_dp). Qui solo l'avanzamento del counter.
					if (clear_idx == 6'd39) begin
						clear_idx <= 6'd0;
						sc_state  <= SC_POP;
					end else begin
						clear_idx <= clear_idx + 6'd1;
					end
				end

				// POP: aspetta un match dalla scan-ombra. Se la FIFO e' vuota ma
				// lo scan ha finito, la riga e' completa.
				SC_POP: begin
					if (!fifo_empty)      sc_state <= SC_POP_W;
					else if (scan_done)   sc_state <= SC_DONE;
				end

				// Il dato della FIFO e' registrato: qui e' valido.
				SC_POP_W: begin
					sp_w0     <= fifo_q[63:48];
					sp_w1     <= fifo_q[47:32];
					sp_w2     <= fifo_q[31:16];
					sp_w3     <= fifo_q[15:0];
					fifo_rd   <= fifo_rd + 8'd1;
					tile_x_pf <= 4'd0;
					pf_side   <= 1'b0;
					sc_state  <= SC_ROM_REQ;
				end

				SC_ROM_REQ: begin
					// gfx_16x16x4_packed_lsb: 8 byte per RIGA (16 pixel x 4 bit),
					// i 16 pixel della riga sono CONTIGUI -> meta' sinistra a +0,
					// destra a +4. (Il tilelayout planare di legionna aveva invece
					// 4 byte/riga e la meta' destra a +64: con quello ogni mezza
					// riga pescava byte di righe diverse del tile.)
					rom_addr <= ({1'b0, cur_tile, 7'd0})        // tile*128
					           + ({17'd0, eff_row, 3'd0})       // row*8
					           + (pf_side ? 24'd4 : 24'd0);     // meta' destra
					rom_req  <= 1'b1;
					sc_state <= SC_ROM_W;
				end

				SC_ROM_W: begin
					if (rom_valid) begin
						pf_rom_data <= rom_data;
						rom_req     <= 1'b0;
						sc_state    <= SC_DECODE;
					end
				end

				// DECODE 1-CICLO: 8 pixel scritti in parallelo sui 8 bank.
				SC_DECODE: begin
					sc_state <= SC_NEXT_TX;
				end

				SC_NEXT_TX: begin
					if (pf_side == 1'b0) begin
						// Appena finito metà sx → fai metà dx dello stesso tile
						pf_side  <= 1'b1;
						sc_state <= SC_ROM_REQ;
					end else begin
						// Finito anche metà dx → passa al prossimo tile_x o entry
						pf_side <= 1'b0;
						if (tile_x_pf == sp_w - 4'd1) begin
							sc_state <= SC_NEXT_E;
						end else begin
							tile_x_pf <= tile_x_pf + 4'd1;
							sc_state  <= SC_ROM_REQ;
						end
					end
				end

				SC_NEXT_E: sc_state <= SC_POP;   // prossimo match dalla FIFO

				SC_DONE: begin
					if (gated_new_line) begin
						// entry_idx e spr_addr appartengono ORA alla scan-ombra:
						// toccarli anche da qui = due driver sullo stesso reg
						// (Quartus Error 10028; Verilator non lo segnala).
						active_buf <= ~active_buf;
						tile_x_pf  <= 4'd0;
						clear_idx  <= 6'd0;
						fifo_rd    <= 8'd0;
						sc_state   <= SC_CLEAR;
					end
				end

				default: sc_state <= SC_IDLE;
			endcase
		end
	end

	// ─── Write port di ogni bank: mux CLEAR vs DECODE ───────────────────────
	// CLEAR (SC_CLEAR): scrive LB_EMPTY a clear_idx su TUTTI gli 8 bank.
	// DECODE (SC_DECODE): scrive bank_wd[b] a bank_row[b] dove bank_we[b].
	// Buffer scritto = quello NON attivo (~active_buf).
	wire        clearing = (sc_state == SC_CLEAR);
	wire        decoding = (sc_state == SC_DECODE);

	wire        b_we   [0:7];
	wire [5:0]  b_wadr [0:7];
	wire [13:0] b_wdat [0:7];

	// ── STADIO DI PIPELINE VERSO IL LINE BUFFER (2026-08-20) ──────────────
	// Il cono sp_w2 -> (calcolo dx, corsia, riga) -> priority encoder ->
	// INDIRIZZO della BRAM e' la famiglia dominante del design: 4328 path
	// negativi, worst -0.975 (build 28). Questo blocco gira a CLOCK PIENO.
	// Registrando we/addr/dato subito prima della BRAM il cono si spezza in
	// due: fino al registro, e dal registro alla memoria.
	// Semantica: la scrittura avviene UN ciclo dopo. E' assorbibile perche' il
	// line buffer e' ping-pong — si scrive il buffer NON attivo e lo si legge
	// dal frame successivo — quindi nessun lettore vede il ritardo.
	// `active_buf` viene registrato INSIEME: senza, al cambio di buffer
	// l'ultima scrittura finirebbe in quello sbagliato.
	reg         b_we_q   [0:7];
	reg  [5:0]  b_wadr_q [0:7];
	reg  [13:0] b_wdat_q [0:7];
	reg         active_buf_q;
	integer bq;
	always @(posedge clk) begin
		active_buf_q <= active_buf;
		for (bq = 0; bq < 8; bq = bq + 1) begin
			b_we_q[bq]   <= reset ? 1'b0 : b_we[bq];
			b_wadr_q[bq] <= b_wadr[bq];
			b_wdat_q[bq] <= b_wdat[bq];
		end
	end
	genvar gi;
	generate
		for (gi = 0; gi < 8; gi = gi + 1) begin : gen_wmux
			assign b_we[gi]   = clearing ? 1'b1 : (decoding & bank_we[gi]);
			assign b_wadr[gi] = clearing ? clear_idx : bank_row[gi];
			assign b_wdat[gi] = clearing ? LB_EMPTY  : bank_wd[gi];
		end
	endgenerate

	// ─── Read port: bank = hpos[2:0], row = hpos[8:3] (0..39) ───────────────
	wire [2:0] rd_lane = hpos[2:0];
	wire [5:0] rd_row  = hpos[8:3];

	// 16 BRAM (8 lane x 2 buffer ping-pong). spram_dp = altsyncram M10K garantito.
	// Write sul buffer NON attivo; read da entrambi, poi mux su active_buf.
	wire [13:0] q0 [0:7];   // output buffer 0 (per lane)
	wire [13:0] q1 [0:7];   // output buffer 1 (per lane)
	generate
		for (gi = 0; gi < 8; gi = gi + 1) begin : gen_bank
			// buffer 0
			spram_dp #(.DW(14), .AW(6)) u_lb0 (
				.clk(clk),
				.we   (b_we_q[gi] & (active_buf_q == 1'b1)),   // scrivo buf0 quando attivo e' buf1
				.waddr(b_wadr_q[gi]), .wdata(b_wdat_q[gi]),
				.raddr(rd_row),     .rdata(q0[gi])
			);
			// buffer 1
			spram_dp #(.DW(14), .AW(6)) u_lb1 (
				.clk(clk),
				.we   (b_we_q[gi] & (active_buf_q == 1'b0)),   // scrivo buf1 quando attivo e' buf0
				.waddr(b_wadr_q[gi]), .wdata(b_wdat_q[gi]),
				.raddr(rd_row),     .rdata(q1[gi])
			);
		end
	endgenerate

	// read dato = buffer attivo, lane = rd_lane (mux combinatorio su q0/q1).
	wire [13:0] read_data_c = active_buf ? q1[rd_lane] : q0[rd_lane];

	// FIX glitch sprite-solo-su-CRT: la spram_dp ha read latency 2 (address_reg +
	// outdata_reg). hpos e' costante 16 clk tra due ce_pix, ma nei PRIMI 2 clk
	// dopo un tick ce_pix rd_lane e' gia' cambiato (combinatorio) mentre q riflette
	// ancora la row precedente -> read_data_c = q[row_vecchia][lane_nuova] per 2 clk
	// = pixel transitorio errato. Output combinatorio -> il transitorio entra
	// nell'RGB grezzo; il DAC analogico (campiona sub-ce_pix) lo mostra, HDMI/hsize no.
	// Fix: campiono read_data/hpos/de @ce_pix -> al tick il dato e' quello del pixel
	// precedente, STABILE da 16 clk -> nessun transitorio. Latenza netta +1 = tile.
	reg [13:0] read_data;
	reg        de_r;
	reg  [9:0] hpos_r;
	always @(posedge clk) if (ce_pix) begin
		read_data <= read_data_c;
		de_r      <= de;
		hpos_r    <= hpos;
	end

	wire  [3:0] read_pen   = read_data[3:0];
	wire  [1:0] read_pri   = read_data[7:6];
	wire  [5:0] read_color = read_data[13:8];

	// clip a hpos<320 + pen!=15. Gate hpos_r/de_r allineati a read_data (tutti @ce_pix).
	wire spr_active = de_r & layer_en & (hpos_r < 10'd320) & (read_pen != 4'd15);
	assign opaque    = spr_active;
	// MAME gfx_legionna_spr: color base = 64*16 = 0x400 (64 colorset × 16 pen).
	assign pen_index = spr_active ? (11'h000 + {1'b0, read_color, read_pen}) : 11'd0;
	assign pri_code  = spr_active ? read_pri : 2'd0;

endmodule
