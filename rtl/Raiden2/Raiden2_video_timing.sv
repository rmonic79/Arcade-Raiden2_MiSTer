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

/*  Video timing single-screen Seibu D-Con.

    RAIDEN II — timing dal driver MAME (raiden2.cpp:1134):
        screen.set_raw(XTAL(32'000'000)/4, 512, 0, 40*8, 282, 0, 30*8)
    cioe' pixel clock 8.000 MHz, 512x282 totali, 320x240 visibili con
    blanking che parte da 0 su entrambi gli assi (nessuna finestra da
    compensare, a differenza di cupsoc che aveva visarea 8..247).

    Nel core: clk_sys 96 MHz / 12 = 8.000 MHz ESATTI.
        HSync   = 8.000 MHz / 512 = 15.625 kHz
        refresh = 15.625 kHz / 282 = 55.408 Hz

    set_raw fornisce SOLO totali e confini di blanking, non la posizione del
    sync: lo split sync/porch e' una scelta dentro il budget (H: 192 px,
    V: 42 righe), tarata perche' l'HSync venga 4.75 us come da standard 15 kHz.

    mode_60hz e' cablato a 0: un "modo 60 Hz" su questo gioco non sarebbe una
    correzione di visualizzazione ma un cambio di velocita' (+8%).
*/

module Raiden2_video_timing
(
	input  wire        clk,            // 96 MHz core
	input  wire        reset,
	input  wire        mode_60hz,      // 0=59.4Hz, 1=60.1Hz
	output reg         ce_pix,         // pixel clock enable (clk/12 = 8 MHz)
	output reg [9:0]   hpos,           // 0..HTotal-1
	output reg [9:0]   vpos,           // 0..VTotal-1
	output wire [9:0]  active_x,       // 0..319 durante area attiva
	output wire [8:0]  active_y,       // 0..239 durante area attiva
	output wire        hblank,
	output wire        vblank,
	output wire        hsync,
	output wire        vsync,
	output wire        de,             // active display enable
	// Confini dell'attivo esportati: il top ne aveva una COPIA cablata
	// (H_VIS_START_TOP/V_VIS_START_TOP) e cambiando il raster le due
	// divergevano -> i renderer disegnavano con l'origine sbagliata e il
	// contenuto si distruggeva. Misurato il 2026-08-24: 73.294 -> 49.830
	// pixel accesi spostando solo l'inizio dell'attivo. Unica fonte, qui.
	output wire [9:0]  h_vis_start,
	output wire [8:0]  v_vis_start
);

	// ── Pixel clock enable: 96 MHz / 12 = 8.000 MHz ─────────────────────────
	// Raiden II reale (raiden2.cpp:1134 set_raw): XTAL(32 MHz)/4 = 8 MHz.
	// 96/12 da' 8.000 MHz ESATTI, nessun errore di frequenza.
	// ⚠ NON accorciare questo divisore per avvicinarsi al dot clock vero
	// della scheda (7,159 MHz): meno clk per pixel = meno clk per riga, e il
	// motore sprite ha solo il 2,4% di margine. Provato il 2026-08-23 con /13
	// a 93 MHz (5915 clk/riga contro 6144): sprite persi, porzioni di gioco
	// che spariscono. Il pixel clock resta un divisore INTERO: un enable non
	// uniforme metterebbe jitter sui bordi dei pixel.
	reg [3:0] cediv;
	always @(posedge clk) begin
		if (reset) begin
			cediv  <= 4'd0;
			ce_pix <= 1'b0;
		end else if (cediv == 4'd12) begin
			cediv  <= 4'd0;
			ce_pix <= 1'b1;
		end else begin
			cediv  <= cediv + 4'd1;
			ce_pix <= 1'b0;
		end
	end

	// ── Constants timing ─────────────────────────────────────────────────────
	// MAME raiden2.cpp:1134
	//   screen.set_raw(XTAL(32'000'000)/4, 512, 0, 40*8, 282, 0, 30*8)
	// = pixel clock 8.000 MHz, 512x282 totali, 320x240 visibili.
	//   96 MHz / 12 = 8.000 MHz ESATTI (nessun errore di frequenza)
	//   HSync   = 8.000 MHz / 512 = 15.625 kHz
	//   refresh = 15.625 kHz / 282 = 55.408 Hz
	// NOTA: set_raw fornisce SOLO totali e confini di blanking, non la posizione
	// del sync. Lo split sync/porch qui sotto e' una scelta dentro il budget
	// (H: 512-320 = 192 px, V: 282-240 = 42 righe), tarata perche' l'HSync
	// venga 38 px / 8 MHz = 4.75 us, cioe' lo standard ~4.7 us dei 15 kHz.
	localparam [9:0] H_TOTAL    = 10'd473;
	localparam [9:0] H_SYNC     = 10'd35;     // 0..34     (4.739 us)
	localparam [9:0] H_BP       = 10'd59;     // 35..93    (7.989 us)
	localparam [9:0] H_VISIBLE  = 10'd320;    // 94..413   (43.333 us)
	localparam [9:0] H_FP       = 10'd59;     // 414..472  (7.989 us)

	localparam [9:0] H_VIS_START = H_SYNC + H_BP;           // 46
	localparam [9:0] H_VIS_END   = H_VIS_START + H_VISIBLE; // 366

	localparam [9:0] V_SYNC     = 10'd3;      // 0..2
	localparam [9:0] V_BP       = 10'd29;     // 3..31
	localparam [9:0] V_VISIBLE  = 10'd240;    // 32..271
	localparam [9:0] V_FP       = 10'd10;     // 272..281

	// Raiden II gira nativamente a 55.408 Hz con un campo di 282 righe.
	// MODO 60 Hz: si accorcia il CAMPO a 260 righe lasciando intatti pixel
	// clock e H_TOTAL, quindi HFreq resta 15.612 kHz (doc 10: la frequenza di
	// riga non si tocca mai) e il quadro sale a 15612/260 = 60.05 Hz. Il gioco
	// gira l'8,4% piu' veloce: e' una scelta dell'utente, non una correzione.
	// Il campo passa da 282 a 260 righe, cioe' dentro lo standard 525/60: e'
	// anche il modo di verificare cosa dipende DAVVERO dalla lunghezza del
	// campo (le 285 righe del discriminatore 525/625 diventano irraggiungibili
	// con qualunque V-Size, e le ultime righe non cadono piu' in fondo a una
	// rampa verticale tirata a 55 Hz).
	//   60 Hz: VSync 3 + BP 11 + attive 240 + FP 6 = 260
	localparam [9:0] V_BP_60     = 10'd11;
	localparam [9:0] V_FP_60     = 10'd6;
	localparam [9:0] V_TOTAL_59  = 10'd282;
	localparam [9:0] V_TOTAL_60  = V_SYNC + V_BP_60 + V_VISIBLE + V_FP_60;  // 260

	wire [9:0] V_TOTAL    = mode_60hz ? V_TOTAL_60 : V_TOTAL_59;
	wire [9:0] V_VIS_START  = V_SYNC + (mode_60hz ? V_BP_60 : V_BP);   // 14 / 32
	wire [9:0] V_VIS_END_59 = V_VIS_START + V_VISIBLE;                 // 254 / 272

	// ── HV counter ───────────────────────────────────────────────────────────
	always @(posedge clk) begin
		if (reset) begin
			hpos <= 10'd0;
			vpos <= 10'd0;
		end else if (ce_pix) begin
			if (hpos == H_TOTAL - 10'd1) begin
				hpos <= 10'd0;
				if (vpos == V_TOTAL - 10'd1) vpos <= 10'd0;
				else                         vpos <= vpos + 10'd1;
			end else begin
				hpos <= hpos + 10'd1;
			end
		end
	end

	// ── Sync, blanking, DE ───────────────────────────────────────────────────
	assign hsync  = (hpos < H_SYNC);
	assign vsync  = (vpos < V_SYNC);
	assign hblank = (hpos < H_VIS_START) || (hpos >= H_VIS_END);
	assign vblank = (vpos < V_VIS_START) || (vpos >= V_VIS_END_59);
	assign de     = ~hblank & ~vblank;

	// ── Active coordinates per renderer (0..319, 0..239) ─────────────────────
	assign h_vis_start = H_VIS_START;
	assign v_vis_start = V_VIS_START[8:0];

	assign active_x = de ? (hpos - H_VIS_START)        : 10'd0;
	assign active_y = de ? (vpos[8:0] - V_VIS_START[8:0]) : 9'd0;

endmodule
