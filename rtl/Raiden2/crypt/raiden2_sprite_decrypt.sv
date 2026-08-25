// SPDX-License-Identifier: GPL-3.0-or-later
// raiden2_sprite_decrypt — decifratura della ROM sprite di Raiden II.
//
// Trascritto da MAME src/mame/seibu/r2crypt.cpp (raiden2_decrypt_sprites +
// core_decrypt) e seibu_helper.cpp (seibu_partial_carry_sum).
// Si applica a ogni WORD DA 32 BIT della region "sprites", indicizzata da i.
//
//   i1 = (i & 0xff) ^ i[15] ^ (i[20] << 8)      -> 0..511
//   i2 = (i & 0xff) ^ i[15]                     -> 0..255
//   i3 = (i >> 8) & 0xff                        -> 0..255
//   i4 = (i >> 16) & 0xf                        -> 0..15
//   v1    = bitswap32(rotl32(cipher, rotate[i1]))
//   x1lo  = (x5[i2] << 11) ^ x11[i3] ^ gm(i4)
//   x1    = { bitswap16(x1lo), x1lo }
//   out   = partial_carry_sum32(v1, x1 ^ PRE_XOR, CARRY_MASK) ^ POST_XOR
//
// gm(i4): ogni bit di i4 si espande in un nibble -> {{4{i4[3]}},...,{4{i4[0]}}}.
// Pipeline a 2 stadi: le tabelle sono BRAM con uscita registrata.
module raiden2_sprite_decrypt (
	input  wire        clk,
	input  wire [21:0] widx,     // indice della word 32-bit
	input  wire [31:0] din,      // ciphertext
	output wire [31:0] dout      // plaintext (2 cicli dopo)
);
	localparam [31:0] PRE_XOR    = 32'h60860000;
	localparam [31:0] CARRY_MASK = 32'h176c91a8;
	localparam [31:0] POST_XOR   = 32'h0f488000;

	// ─── Tabelle (estratte da r2crypt.cpp) ──────────────────────────────────
	(* ramstyle = "M10K" *) reg [4:0]  rot_tab [0:511];
	(* ramstyle = "M10K" *) reg [7:0]  x5_tab  [0:255];
	(* ramstyle = "M10K" *) reg [15:0] x11_tab [0:255];
	initial begin
		$readmemh("rtl/Raiden2/crypt/rotate_r2.hex", rot_tab);
		$readmemh("rtl/Raiden2/crypt/x5_r2.hex",     x5_tab);
		$readmemh("rtl/Raiden2/crypt/x11_r2.hex",    x11_tab);
	end

	wire [8:0] i1 = {1'b0, widx[7:0] ^ {7'd0, widx[15]}} ^ {widx[20], 8'd0};
	wire [7:0] i2 = widx[7:0] ^ {7'd0, widx[15]};
	wire [7:0] i3 = widx[15:8];
	wire [3:0] i4 = widx[19:16];

	reg [4:0]  rot_q;
	reg [7:0]  x5_q;
	reg [15:0] x11_q;
	reg [3:0]  i4_q;
	reg [31:0] din_q;
	always @(posedge clk) begin
		rot_q <= rot_tab[i1];
		x5_q  <= x5_tab [i2];
		x11_q <= x11_tab[i3];
		i4_q  <= i4;
		din_q <= din;
	end

	// ─── rotl32 variabile + bitswap32 ───────────────────────────────────────
	wire [63:0] dbl = {din_q, din_q};
	wire [31:0] rot = dbl[63 - rot_q -: 32];
	wire [31:0] v1;
	assign v1[31] = rot[25];
	assign v1[30] = rot[28];
	assign v1[29] = rot[15];
	assign v1[28] = rot[19];
	assign v1[27] = rot[ 6];
	assign v1[26] = rot[ 0];
	assign v1[25] = rot[ 3];
	assign v1[24] = rot[24];
	assign v1[23] = rot[11];
	assign v1[22] = rot[ 1];
	assign v1[21] = rot[ 2];
	assign v1[20] = rot[30];
	assign v1[19] = rot[16];
	assign v1[18] = rot[ 7];
	assign v1[17] = rot[22];
	assign v1[16] = rot[17];
	assign v1[15] = rot[31];
	assign v1[14] = rot[14];
	assign v1[13] = rot[23];
	assign v1[12] = rot[ 9];
	assign v1[11] = rot[27];
	assign v1[10] = rot[18];
	assign v1[ 9] = rot[ 4];
	assign v1[ 8] = rot[10];
	assign v1[ 7] = rot[13];
	assign v1[ 6] = rot[20];
	assign v1[ 5] = rot[ 5];
	assign v1[ 4] = rot[12];
	assign v1[ 3] = rot[ 8];
	assign v1[ 2] = rot[29];
	assign v1[ 1] = rot[26];
	assign v1[ 0] = rot[21];

	// ─── x1 ─────────────────────────────────────────────────────────────────
	wire [15:0] gm = {{4{i4_q[3]}}, {4{i4_q[2]}}, {4{i4_q[1]}}, {4{i4_q[0]}}};
	wire [15:0] x1lo = ({x5_q, 11'd0} ^ x11_q) ^ gm;
	wire [15:0] x1hi;
	assign x1hi[15] = x1lo[ 0];
	assign x1hi[14] = x1lo[ 8];
	assign x1hi[13] = x1lo[ 1];
	assign x1hi[12] = x1lo[ 9];
	assign x1hi[11] = x1lo[ 2];
	assign x1hi[10] = x1lo[10];
	assign x1hi[ 9] = x1lo[ 3];
	assign x1hi[ 8] = x1lo[11];
	assign x1hi[ 7] = x1lo[ 4];
	assign x1hi[ 6] = x1lo[12];
	assign x1hi[ 5] = x1lo[ 5];
	assign x1hi[ 4] = x1lo[13];
	assign x1hi[ 3] = x1lo[ 6];
	assign x1hi[ 2] = x1lo[14];
	assign x1hi[ 1] = x1lo[ 7];
	assign x1hi[ 0] = x1lo[15];
	wire [31:0] x1 = {x1hi, x1lo};

	// ─── somma con riporto parziale (seibu_helper.cpp:12) ───────────────────
	// Il riporto propaga solo dove CARRY_MASK ha 1; il riporto finale rientra
	// in XOR sul bit 0.
	wire [31:0] a = v1;
	wire [31:0] b = x1 ^ PRE_XOR;
	wire [32:0] cy;
	wire [31:0] sum;
	assign cy[0] = 1'b0;
	genvar g;
	generate
		for (g = 0; g < 32; g = g + 1) begin : pcs
			wire [1:0] bit_sum = {1'b0, a[g]} + {1'b0, b[g]} + {1'b0, cy[g]};
			assign sum[g]   = bit_sum[0];
			assign cy[g+1]  = CARRY_MASK[g] ? bit_sum[1] : 1'b0;
		end
	endgenerate
	assign dout = ({sum[31:1], sum[0] ^ cy[32]}) ^ POST_XOR;

endmodule
