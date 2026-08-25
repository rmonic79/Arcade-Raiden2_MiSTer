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

/*  Text layer renderer Seibu Raiden2 (8x8, 4bpp, 64x32).

    Specifiche da MAME (legionna.cpp:1117-1126, 1167):

      // charlayout
      8x8, RGN_FRAC(1,1), 4 bpp,
      planes  = { STEP4(0, 4) }                = { 0, 4, 8, 12 }       (bit offset)
      x_bits  = { STEP4(3,-1), STEP4(4*4+3,-1)} = { 3,2,1,0, 19,18,17,16 }
      y_bits  = { STEP8(0, 4*8) }              = { 0, 32, 64, ..., 224 }
      tile size = 8 row × 32 bit = 32 byte / tile (256 bit / 4 bpp)

      // GFXDECODE
      GFXDECODE_ENTRY( "char", 0, charlayout, 48*16, 16 )
      → color base = 0x300 + offset (48*16 = 0x300, palette index)
      → MA verificato in Raiden2.sv pen_index: 0x700 + (color<<4) + pen
      → discrepanza: 48*16 = 768 = 0x300, NON 0x700. Verifica blocker.

      // get_text_tile_info
      tile  = textram[tile_index];
      color = (tile >> 12) & 0xf;
      tile  = tile & 0xfff;
      tileinfo.set(0, tile, color, 0);
      m_text_layer->set_transparent_pen(15);

    Char ROM in MAME: "char" region 64KB = ROM_COPY user1[0x10000..0x1FFFF].
    Nel nostro porting carichiamo user1 byte 0x080000..0x09FFFF via ioctl,
    e il text legge SOLO la 2a metà (ioctl 0x090000..0x09FFFF) in BRAM 64KB.

    Layout fisico byte per (row, col) di tile_idx:
      word index 16-bit = tile_idx*16 + row*2 + col_high
        dove col_high = (col >= 4) ? 1 : 0
      word 16-bit contiene plane 0/1/2/3 di 4 col:
        bit  3..0 : plane 0, col 3..0
        bit  7..4 : plane 1, col 3..0
        bit 11..8 : plane 2, col 3..0
        bit 15..12: plane 3, col 3..0
      Per col c (0..3 in word low, 0..3 in word high), pen 4-bit:
        sub = 3 - (col & 3)
        pen[0] = word[sub]
        pen[1] = word[sub + 4]
        pen[2] = word[sub + 8]
        pen[3] = word[sub + 12]

    Pen finale palette = 0x300 + (color << 4) + pen (color 0..15, 16 set).
    Trasparente se pen == 15.
*/

module Raiden2_text_renderer (
	input  wire        clk,
	input  wire        reset,
	input  wire        ce_pix,

	// Video timing
	input  wire  [9:0] hpos,        // 0..319 logico
	input  wire  [9:0] hpos_rd,     // hpos anticipato di 1 (il mixer campia su ce_pix_d)
	input  wire  [8:0] vpos,        // 0..239 logico, REALE (mai specchiata dal top)
	input  wire        flip_screen, // DIP Flip Screen: specchia la riga di contenuto
	input  wire        de,
	input  wire        layer_en,
	input  wire        new_line,

	// CRTC scroll
	input  wire [15:0] scroll_x,
	input  wire [15:0] scroll_y,

	// OSD offset di rendering
	input  wire signed [9:0] xoff,
	input  wire signed [9:0] yoff,

	// Text VRAM read port (4KB = 2Kw, 64*32 grid)
	output reg  [10:0] vram_addr,
	input  wire [15:0] vram_data,

	// Char ROM in SDRAM via tile_rom_arbiter (porta r4, gfx_kind 4 -> TXT_BASE).
	// La charrom NON sta piu' in BRAM: erano 128 M10K per 128 KB di ROM che la
	// SDRAM gia' conteneva (sdram_bridge.sv: TXT_BASE = 24'h080000).
	output reg         rom_req,
	output reg  [23:0] rom_addr,
	input  wire [31:0] rom_data,
	input  wire        rom_valid,

	output wire        opaque,
	output wire [10:0] pen_index
);

	localparam [8:0] LB_EMPTY = 9'h100;   // transp = 1

	(* ramstyle = "M10K,no_rw_check" *) reg [8:0] lb0 [0:319];
	(* ramstyle = "M10K,no_rw_check" *) reg [8:0] lb1 [0:319];

	reg        active_buf;
	reg  [8:0] clear_idx;

	// ── Coordinate del prefetch: riga N+1 durante la riga N ──────────────────
	localparam [8:0] V_VISIBLE = 9'd240;
	// Flip DIP: come nei tile layer si specchia la RIGA DI CONTENUTO, non la
	// vpos (che deve restare reale per tenere in fase prefetch e swap del
	// doppio buffer). 239-(L+1) == 238-L, scritto in un solo addizionatore per
	// ramo per non allungare il percorso.
	wire [15:0] target_y = flip_screen
	        ? ((vpos == V_VISIBLE - 9'd1) ? 16'd239 : (16'd238 - {7'd0, vpos}))
	        : ((vpos == V_VISIBLE - 9'd1) ? 16'd0   : ({7'd0, vpos} + 16'd1));
	wire [15:0] eff_y_pf  = target_y + scroll_y + {{6{yoff[9]}}, yoff};
	wire  [4:0] tile_y_pf = eff_y_pf[7:3];
	wire  [2:0] row_pf    = eff_y_pf[2:0];

	wire vpos_visible   = (vpos < V_VISIBLE);
	wire gated_new_line = new_line & vpos_visible;

	reg [15:0] scroll_x_l;
	always @(posedge clk) begin
		if (reset)               scroll_x_l <= 16'd0;
		// +1: offset della finestra che riproduce ESATTAMENTE la posizione a
		// schermo del renderer precedente. Misurato, non stimato: con questo
		// valore il frame e' bit-identico a quello del vecchio modulo (il +3
		// del vecchio compensava i suoi 3 stadi di pipeline, che qui non ci
		// sono, e la lettura del line buffer ha gia' il lookahead di 1).
		else if (gated_new_line) scroll_x_l <= scroll_x + 16'd1 + {{6{xoff[9]}}, xoff};
	end
	wire  [5:0] first_tile_x   = scroll_x_l[8:3];
	wire  [2:0] first_pixel_off = scroll_x_l[2:0];

	reg  [5:0] tile_col;                       // 0..40 (320/8 + 1)
	wire [5:0] cur_tile_x = first_tile_x + tile_col;
	wire signed [10:0] dst_x_base = ({2'd0, tile_col, 3'd0}) - {8'd0, first_pixel_off};

	// ── FSM di prefetch ──────────────────────────────────────────────────────
	localparam PF_IDLE=3'd0, PF_CLEAR=3'd1, PF_VRAM_R=3'd2, PF_VRAM_W=3'd3,
	           PF_VRAM_W2=3'd4, PF_ROM_REQ=3'd5, PF_ROM_W=3'd6, PF_WRITE=3'd7;
	reg [2:0] pf_state;
	reg [11:0] pf_tile_idx;
	reg  [3:0] pf_tile_clr;
	reg [31:0] pf_rom_data;
	reg  [2:0] wr_step;
	// Ultimo codice fetchato in QUESTA riga: se la cella successiva ha lo stesso
	// codice, i 4 byte sono gli stessi e il fetch si salta. Sul text la stessa
	// cella si ripete a lunghe corse (fondo vuoto), quindi una riga passa da 41
	// fetch a pochi: e' cio' che tiene la banda dell'arbitro ai tile layer.
	reg [11:0] last_idx;
	reg        last_ok;

	// decode di un pixel: word A = [31:16] per col 0..3, word B = [15:0] per 4..7
	wire [15:0] dec_word = wr_step[2] ? pf_rom_data[15:0] : pf_rom_data[31:16];
	wire  [1:0] dec_sub  = 2'd3 - wr_step[1:0];
	wire  [3:0] dec_pen  = { dec_word[15 - {2'd0, dec_sub}],
	                         dec_word[11 - {2'd0, dec_sub}],
	                         dec_word[ 7 - {2'd0, dec_sub}],
	                         dec_word[ 3 - {2'd0, dec_sub}] };
	wire signed [10:0] dec_dx = dst_x_base + {8'd0, wr_step};

	reg        wr_en;
	reg  [8:0] wr_addr, wr_data;
	always @(*) begin
		if (pf_state == PF_CLEAR) begin
			wr_en = 1'b1; wr_addr = clear_idx; wr_data = LB_EMPTY;
		end else if (pf_state == PF_WRITE) begin
			wr_en   = (dec_dx >= 0) && (dec_dx < 11'sd320);
			wr_addr = dec_dx[8:0];
			wr_data = {(dec_pen == 4'd15), pf_tile_clr, dec_pen};
		end else begin
			wr_en = 1'b0; wr_addr = 9'd0; wr_data = LB_EMPTY;
		end
	end
	always @(posedge clk) if (wr_en && active_buf == 1'b0) lb1[wr_addr] <= wr_data;
	always @(posedge clk) if (wr_en && active_buf == 1'b1) lb0[wr_addr] <= wr_data;

	always @(posedge clk) begin
		if (reset) begin
			pf_state <= PF_IDLE; tile_col <= 6'd0; rom_req <= 1'b0;
			last_ok <= 1'b0; last_idx <= 12'd0;
			vram_addr <= 11'd0; active_buf <= 1'b0; clear_idx <= 9'd0;
			wr_step <= 3'd0; rom_addr <= 24'd0;
		end else begin
			case (pf_state)
				PF_IDLE: if (gated_new_line) begin
					active_buf <= ~active_buf; tile_col <= 6'd0;
					clear_idx  <= 9'd0;        pf_state <= PF_CLEAR;
					last_ok    <= 1'b0;
				end
				PF_CLEAR: if (clear_idx == 9'd319) begin
					clear_idx <= 9'd0; pf_state <= PF_VRAM_R;
				end else clear_idx <= clear_idx + 9'd1;

				PF_VRAM_R: begin
					vram_addr <= {tile_y_pf, cur_tile_x};   // y*64 + x
					pf_state  <= PF_VRAM_W;
				end
				PF_VRAM_W:  pf_state <= PF_VRAM_W2;
				PF_VRAM_W2: begin
					pf_tile_idx <= vram_data[11:0];
					pf_tile_clr <= vram_data[15:12];
					// Carattere vuoto: niente fetch. Sul text la stragrande
					// maggioranza delle celle e' vuota, e ogni fetch risparmiato
					// e' banda che resta ai tile layer sullo stesso arbitro.
					if (last_ok && (vram_data[11:0] == last_idx)) begin
						wr_step  <= 3'd0;        // stessi 4 byte: riuso pf_rom_data
						pf_state <= PF_WRITE;
					end else pf_state <= PF_ROM_REQ;
				end
				PF_ROM_REQ: begin
					// word = (tile<<4)|(row<<1) -> byte = word<<1
					// word = (tile<<4)|(row<<1)|col_high  ->  byte = (tile<<5)|(row<<2)
					rom_addr <= {7'd0, pf_tile_idx, 5'd0} | {19'd0, row_pf, 2'd0};
					rom_req  <= 1'b1;   // request a LIVELLO come i tile layer
					                    // (l'arbitro rileva il fronte di salita)
					pf_state <= PF_ROM_W;
				end
				PF_ROM_W: if (rom_valid) begin
					rom_req     <= 1'b0;
					pf_rom_data <= rom_data;
					last_idx    <= pf_tile_idx;
					last_ok     <= 1'b1;
					wr_step     <= 3'd0;
					pf_state    <= PF_WRITE;
				end
				PF_WRITE: begin
					if (wr_step == 3'd7) begin
						if (tile_col == 6'd40) pf_state <= PF_IDLE;
						else begin
							tile_col <= tile_col + 6'd1;
							pf_state <= PF_VRAM_R;
						end
					end else wr_step <= wr_step + 3'd1;
				end
				default: pf_state <= PF_IDLE;
			endcase
		end
	end

	// ── Lettura: indirizzo anticipato, gate sull'indice NON ritardato ────────
	reg [8:0] rd0_r, rd1_r;
	reg       active_buf_r;
	always @(posedge clk) begin
		rd0_r        <= lb0[hpos_rd[8:0]];
		rd1_r        <= lb1[hpos_rd[8:0]];
		active_buf_r <= active_buf;
	end
	wire [8:0] read_data = active_buf_r ? rd1_r : rd0_r;

	wire pixel_active = de & layer_en & (hpos < 10'd320) & ~read_data[8];
	assign opaque    = pixel_active;
	assign pen_index = pixel_active ? (11'h700 + {3'd0, read_data[7:0]}) : 11'd0;

endmodule
