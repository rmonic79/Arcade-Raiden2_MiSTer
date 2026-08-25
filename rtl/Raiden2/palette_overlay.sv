// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// palette_overlay.sv — visualizza N entry palette runtime sopra il video.
//
// Tap sulle scritture CPU Sub a $03000-$03FFF, costruisce shadow palette in BRAM,
// poi ne renderizza un grid colorato con indici hex sopra il framebuffer.
//
// Uso:
//   1) connettere cpu_addr/cpu_dout/cpu_be/cpu_wr/palette_memrq da Sub V30
//   2) connettere render_x/render_y dal video timing
//   3) connettere rgb_*_in dal pixel pipeline arcade
//   4) enable=1 per attivare overlay
//
// Layout overlay (default, 8x8 char):
//   Riga 0..15 (a partire da row offset 192):
//     16 entry × 1 row di pixel = 16 quadratini 8x8 colorati a riga
//     Sotto a ogni quadratino: 3-char hex index (es "010")
//   Si visualizzano 16 entry per riga × 4 righe = 64 entry totali (da pal_start)
//
// Footprint: 1 BRAM 2K × 16-bit (shadow palette).

module palette_overlay (
	input  wire        clk,
	input  wire        ce_pix,
	input  wire        enable,        // off = pass-through totale

	// ─── CPU tap (Sub V30 → palette $03000-$03FFF) ──────────────────────
	input  wire [19:0] cpu_addr,
	input  wire [15:0] cpu_dout,
	input  wire  [1:0] cpu_be,
	input  wire        cpu_wr,
	input  wire        palette_memrq,

	// ─── Selezione vista ────────────────────────────────────────────────
	input  wire [10:0] pal_start,     // primo indice palette mostrato (0..0x3F0)

	// ─── Video coords ───────────────────────────────────────────────────
	input  wire  [9:0] render_x,      // 0..319
	input  wire  [8:0] render_y,      // 0..223

	// ─── Video in/out ───────────────────────────────────────────────────
	input  wire  [7:0] rgb_r_in,
	input  wire  [7:0] rgb_g_in,
	input  wire  [7:0] rgb_b_in,
	output wire  [7:0] rgb_r_out,
	output wire  [7:0] rgb_g_out,
	output wire  [7:0] rgb_b_out
);

// ─── Shadow palette: 2048 word, una entry per scrittura CPU ─────────────
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] shadow_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] shadow_hi [0:2047];
initial begin integer i; for (i=0; i<2048; i=i+1) begin shadow_lo[i]=0; shadow_hi[i]=0; end end

// Decode write enable
wire wr_lo = palette_memrq && cpu_wr && cpu_be[0] && !cpu_be[1] && !cpu_addr[0];
wire wr_hi = palette_memrq && cpu_wr && cpu_be[0] && !cpu_be[1] &&  cpu_addr[0];
wire wr_w  = palette_memrq && cpu_wr && cpu_be[0] &&  cpu_be[1];
wire [10:0] pal_word_addr = cpu_addr[11:1];

always @(posedge clk) begin
	if (wr_lo) shadow_lo[pal_word_addr] <= cpu_dout[7:0];
	if (wr_hi) shadow_hi[pal_word_addr] <= cpu_dout[7:0];
	if (wr_w) begin
		shadow_lo[pal_word_addr] <= cpu_dout[7:0];
		shadow_hi[pal_word_addr] <= cpu_dout[15:8];
	end
end

// ─── Layout overlay ─────────────────────────────────────────────────────
// Grid 16 colonne × 4 righe = 64 entry
// Cella 16x12 px (8 colore + 4 hex 1 char rid 4x6)
// Per semplicità: cella 18x14, di cui 8x8 colore + 8x6 testo sotto
// Origine grid a (X0, Y0)
localparam [9:0] GRID_X0 = 10'd8;
localparam [8:0] GRID_Y0 = 9'd128;
localparam       CELL_W  = 18;
localparam       CELL_H  = 16;
localparam       COLS    = 16;
localparam       ROWS    = 4;
localparam       GRID_W  = COLS * CELL_W;     // 288
localparam       GRID_H  = ROWS * CELL_H;     // 64

// Bounding box overlay
wire in_grid_x = (render_x >= GRID_X0) && (render_x < GRID_X0 + GRID_W);
wire in_grid_y = (render_y >= GRID_Y0) && (render_y < GRID_Y0 + GRID_H);
wire in_grid   = enable && in_grid_x && in_grid_y;

// Coordinate locali dentro grid
wire [9:0] gx = render_x - GRID_X0;
wire [8:0] gy = render_y - GRID_Y0;
wire [3:0] cell_col = gx / CELL_W;       // 0..15
wire [1:0] cell_row = gy / CELL_H;       // 0..3
wire [4:0] cell_x   = gx % CELL_W;       // 0..17
wire [3:0] cell_y   = gy % CELL_H;       // 0..15

// Indice palette per questa cella
wire [10:0] cell_idx = pal_start + {5'd0, cell_row, cell_col};   // pal_start + row*16 + col

// ─── Lettura shadow palette per questa cella (1 ciclo BRAM) ─────────────
reg [7:0] cell_lo_r, cell_hi_r;
always @(posedge clk) begin
	cell_lo_r <= shadow_lo[cell_idx];
	cell_hi_r <= shadow_hi[cell_idx];
end

// Espandi xBGR_444 → 8-bit per canale
wire [7:0] pal_r = {cell_lo_r[3:0], cell_lo_r[3:0]};
wire [7:0] pal_g = {cell_lo_r[7:4], cell_lo_r[7:4]};
wire [7:0] pal_b = {cell_hi_r[3:0], cell_hi_r[3:0]};

// ─── Layout dentro la cella (18x16) ─────────────────────────────────────
//   x 0..7  y 0..7  : quadratino colore (8x8)
//   x 8..17 y 0..7  : nulla (margine)
//   x 0..17 y 8..14 : 3 char hex (3 × 5 px = 15 px)
//   y 15            : separator nero
wire color_box = (cell_x < 8)  && (cell_y < 8);
wire hex_zone  = (cell_y >= 8) && (cell_y < 14);
wire hex_x     = (cell_x < 15);

// Pipeline: quale char (0..2)?
wire [1:0] char_idx = cell_x[4:1] >= 5'd10 ? 2'd2 :
                      cell_x[4:1] >=  5'd5 ? 2'd1 : 2'd0;
wire [2:0] char_col = (cell_x - 5'd0 - {char_idx, 2'b00, 1'b0}) % 5;   // 0..4 dentro char
wire [2:0] char_row = cell_y[2:0] - 3'd0;                              // 0..6 dentro char (font 5x7)

// Estrai nibble (3 cifre hex per indice 12-bit). cell_idx è 11-bit, bit [11] = 0.
wire [11:0] cell_idx_12 = {1'b0, cell_idx};
wire [3:0] nibble = char_idx == 2'd0 ? cell_idx_12[11:8] :
                    char_idx == 2'd1 ? cell_idx_12[7:4]  :
                                       cell_idx_12[3:0];

// Font 5x7 per cifre hex 0..F (riga 0 = top)
function [4:0] font_row5;
	input [3:0] digit;
	input [2:0] row;
	begin
		case ({digit, row})
			// 0
			{4'h0, 3'd0}: font_row5 = 5'b01110;
			{4'h0, 3'd1}: font_row5 = 5'b10001;
			{4'h0, 3'd2}: font_row5 = 5'b10011;
			{4'h0, 3'd3}: font_row5 = 5'b10101;
			{4'h0, 3'd4}: font_row5 = 5'b11001;
			{4'h0, 3'd5}: font_row5 = 5'b10001;
			{4'h0, 3'd6}: font_row5 = 5'b01110;
			// 1
			{4'h1, 3'd0}: font_row5 = 5'b00100;
			{4'h1, 3'd1}: font_row5 = 5'b01100;
			{4'h1, 3'd2}: font_row5 = 5'b00100;
			{4'h1, 3'd3}: font_row5 = 5'b00100;
			{4'h1, 3'd4}: font_row5 = 5'b00100;
			{4'h1, 3'd5}: font_row5 = 5'b00100;
			{4'h1, 3'd6}: font_row5 = 5'b01110;
			// 2
			{4'h2, 3'd0}: font_row5 = 5'b01110;
			{4'h2, 3'd1}: font_row5 = 5'b10001;
			{4'h2, 3'd2}: font_row5 = 5'b00001;
			{4'h2, 3'd3}: font_row5 = 5'b00010;
			{4'h2, 3'd4}: font_row5 = 5'b00100;
			{4'h2, 3'd5}: font_row5 = 5'b01000;
			{4'h2, 3'd6}: font_row5 = 5'b11111;
			// 3
			{4'h3, 3'd0}: font_row5 = 5'b11111;
			{4'h3, 3'd1}: font_row5 = 5'b00010;
			{4'h3, 3'd2}: font_row5 = 5'b00100;
			{4'h3, 3'd3}: font_row5 = 5'b00010;
			{4'h3, 3'd4}: font_row5 = 5'b00001;
			{4'h3, 3'd5}: font_row5 = 5'b10001;
			{4'h3, 3'd6}: font_row5 = 5'b01110;
			// 4
			{4'h4, 3'd0}: font_row5 = 5'b00010;
			{4'h4, 3'd1}: font_row5 = 5'b00110;
			{4'h4, 3'd2}: font_row5 = 5'b01010;
			{4'h4, 3'd3}: font_row5 = 5'b10010;
			{4'h4, 3'd4}: font_row5 = 5'b11111;
			{4'h4, 3'd5}: font_row5 = 5'b00010;
			{4'h4, 3'd6}: font_row5 = 5'b00010;
			// 5
			{4'h5, 3'd0}: font_row5 = 5'b11111;
			{4'h5, 3'd1}: font_row5 = 5'b10000;
			{4'h5, 3'd2}: font_row5 = 5'b11110;
			{4'h5, 3'd3}: font_row5 = 5'b00001;
			{4'h5, 3'd4}: font_row5 = 5'b00001;
			{4'h5, 3'd5}: font_row5 = 5'b10001;
			{4'h5, 3'd6}: font_row5 = 5'b01110;
			// 6
			{4'h6, 3'd0}: font_row5 = 5'b00110;
			{4'h6, 3'd1}: font_row5 = 5'b01000;
			{4'h6, 3'd2}: font_row5 = 5'b10000;
			{4'h6, 3'd3}: font_row5 = 5'b11110;
			{4'h6, 3'd4}: font_row5 = 5'b10001;
			{4'h6, 3'd5}: font_row5 = 5'b10001;
			{4'h6, 3'd6}: font_row5 = 5'b01110;
			// 7
			{4'h7, 3'd0}: font_row5 = 5'b11111;
			{4'h7, 3'd1}: font_row5 = 5'b00001;
			{4'h7, 3'd2}: font_row5 = 5'b00010;
			{4'h7, 3'd3}: font_row5 = 5'b00100;
			{4'h7, 3'd4}: font_row5 = 5'b01000;
			{4'h7, 3'd5}: font_row5 = 5'b01000;
			{4'h7, 3'd6}: font_row5 = 5'b01000;
			// 8
			{4'h8, 3'd0}: font_row5 = 5'b01110;
			{4'h8, 3'd1}: font_row5 = 5'b10001;
			{4'h8, 3'd2}: font_row5 = 5'b10001;
			{4'h8, 3'd3}: font_row5 = 5'b01110;
			{4'h8, 3'd4}: font_row5 = 5'b10001;
			{4'h8, 3'd5}: font_row5 = 5'b10001;
			{4'h8, 3'd6}: font_row5 = 5'b01110;
			// 9
			{4'h9, 3'd0}: font_row5 = 5'b01110;
			{4'h9, 3'd1}: font_row5 = 5'b10001;
			{4'h9, 3'd2}: font_row5 = 5'b10001;
			{4'h9, 3'd3}: font_row5 = 5'b01111;
			{4'h9, 3'd4}: font_row5 = 5'b00001;
			{4'h9, 3'd5}: font_row5 = 5'b00010;
			{4'h9, 3'd6}: font_row5 = 5'b01100;
			// A
			{4'hA, 3'd0}: font_row5 = 5'b01110;
			{4'hA, 3'd1}: font_row5 = 5'b10001;
			{4'hA, 3'd2}: font_row5 = 5'b10001;
			{4'hA, 3'd3}: font_row5 = 5'b11111;
			{4'hA, 3'd4}: font_row5 = 5'b10001;
			{4'hA, 3'd5}: font_row5 = 5'b10001;
			{4'hA, 3'd6}: font_row5 = 5'b10001;
			// B
			{4'hB, 3'd0}: font_row5 = 5'b11110;
			{4'hB, 3'd1}: font_row5 = 5'b10001;
			{4'hB, 3'd2}: font_row5 = 5'b10001;
			{4'hB, 3'd3}: font_row5 = 5'b11110;
			{4'hB, 3'd4}: font_row5 = 5'b10001;
			{4'hB, 3'd5}: font_row5 = 5'b10001;
			{4'hB, 3'd6}: font_row5 = 5'b11110;
			// C
			{4'hC, 3'd0}: font_row5 = 5'b01110;
			{4'hC, 3'd1}: font_row5 = 5'b10001;
			{4'hC, 3'd2}: font_row5 = 5'b10000;
			{4'hC, 3'd3}: font_row5 = 5'b10000;
			{4'hC, 3'd4}: font_row5 = 5'b10000;
			{4'hC, 3'd5}: font_row5 = 5'b10001;
			{4'hC, 3'd6}: font_row5 = 5'b01110;
			// D
			{4'hD, 3'd0}: font_row5 = 5'b11110;
			{4'hD, 3'd1}: font_row5 = 5'b10001;
			{4'hD, 3'd2}: font_row5 = 5'b10001;
			{4'hD, 3'd3}: font_row5 = 5'b10001;
			{4'hD, 3'd4}: font_row5 = 5'b10001;
			{4'hD, 3'd5}: font_row5 = 5'b10001;
			{4'hD, 3'd6}: font_row5 = 5'b11110;
			// E
			{4'hE, 3'd0}: font_row5 = 5'b11111;
			{4'hE, 3'd1}: font_row5 = 5'b10000;
			{4'hE, 3'd2}: font_row5 = 5'b10000;
			{4'hE, 3'd3}: font_row5 = 5'b11110;
			{4'hE, 3'd4}: font_row5 = 5'b10000;
			{4'hE, 3'd5}: font_row5 = 5'b10000;
			{4'hE, 3'd6}: font_row5 = 5'b11111;
			// F
			{4'hF, 3'd0}: font_row5 = 5'b11111;
			{4'hF, 3'd1}: font_row5 = 5'b10000;
			{4'hF, 3'd2}: font_row5 = 5'b10000;
			{4'hF, 3'd3}: font_row5 = 5'b11110;
			{4'hF, 3'd4}: font_row5 = 5'b10000;
			{4'hF, 3'd5}: font_row5 = 5'b10000;
			{4'hF, 3'd6}: font_row5 = 5'b10000;
			default:      font_row5 = 5'b00000;
		endcase
	end
endfunction

wire [4:0] f_row = font_row5(nibble, char_row);
wire char_pix = hex_zone && hex_x && (char_col < 5) && f_row[4 - char_col];

// ─── Mux finale ──────────────────────────────────────────────────────────
// Priorità: char_pix (bianco) > color_box (palette color) > background (input)
reg [7:0] r_out, g_out, b_out;
always @(*) begin
	if (in_grid && char_pix) begin
		r_out = 8'hFF; g_out = 8'hFF; b_out = 8'hFF;
	end else if (in_grid && color_box) begin
		r_out = pal_r; g_out = pal_g; b_out = pal_b;
	end else if (in_grid) begin
		// area cella non-color non-text → nero (background overlay)
		r_out = 8'h00; g_out = 8'h00; b_out = 8'h00;
	end else begin
		r_out = rgb_r_in; g_out = rgb_g_in; b_out = rgb_b_in;
	end
end

assign rgb_r_out = r_out;
assign rgb_g_out = g_out;
assign rgb_b_out = b_out;

endmodule
