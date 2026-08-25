// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// raiden2_decrypt — passthrough ioctl_dout con decifrato MAME init_decryption()
//
// Inserito tra hps_io e sdram_bridge. Il bridge resta identico a GundamSD.
// La ROM va in SDRAM già decifrata. CPU legge raw da SDRAM.
//
// Range crittato (riferimento raiden.cpp:1187):
//   - Main ROM section 2: MRA byte 0x020000-0x05FFFF (CPU 0xC0000-0xFFFFF)
//   - Sub ROM tutto:      MRA byte 0x060000-0x09FFFF (CPU 0xC0000-0xFFFFF)
//
// Index XOR table = bit bassi del word index (= byte_addr >> 1):
//   - Main: idx = byte_addr[4:1] (16 entry)
//   - Sub:  idx = byte_addr[3:1] ( 8 entry)
//   I range partono da 0x020000 e 0x060000 (entrambi allineati a 16 e 8
//   rispettivamente), quindi i bit bassi sono invariati rispetto al
//   word index relativo al base.
//
// Bitswap16 da MAME bitswap<16>(d, b15..b0): out[15]=d[b15], ..., out[0]=d[b0].

module raiden2_decrypt (
	input  wire [26:0] ioctl_addr_in,
	input  wire [15:0] ioctl_dout_in,
	input  wire        ioctl_wr_in,
	input  wire [15:0] ioctl_index_in,
	input  wire        ioctl_download_in,

	output wire [26:0] ioctl_addr_out,
	output wire [15:0] ioctl_dout_out,
	output wire        ioctl_wr_out,
	output wire [15:0] ioctl_index_out,
	output wire        ioctl_download_out
);

	// ── XOR tables ──────────────────────────────────────────────────────
	function automatic [15:0] xor_main(input [3:0] idx);
		case (idx)
			4'h0: xor_main = 16'h200e; 4'h1: xor_main = 16'h0006;
			4'h2: xor_main = 16'h000a; 4'h3: xor_main = 16'h0002;
			4'h4: xor_main = 16'h240e; 4'h5: xor_main = 16'h000e;
			4'h6: xor_main = 16'h04c2; 4'h7: xor_main = 16'h00c2;
			4'h8: xor_main = 16'h008c; 4'h9: xor_main = 16'h0004;
			4'ha: xor_main = 16'h0088; 4'hb: xor_main = 16'h0000;
			4'hc: xor_main = 16'h048c; 4'hd: xor_main = 16'h000c;
			4'he: xor_main = 16'h04c0; 4'hf: xor_main = 16'h00c0;
		endcase
	endfunction

	function automatic [15:0] xor_sub(input [2:0] idx);
		case (idx)
			3'd0: xor_sub = 16'h0080; 3'd1: xor_sub = 16'h0080;
			3'd2: xor_sub = 16'h0244; 3'd3: xor_sub = 16'h0288;
			3'd4: xor_sub = 16'h0288; 3'd5: xor_sub = 16'h0288;
			3'd6: xor_sub = 16'h1041; 3'd7: xor_sub = 16'h1009;
		endcase
	endfunction

	// ── Bitswap functions ───────────────────────────────────────────────
	// Main: 15,14,10,12,11,13,9,8,3,2,5,4,7,1,6,0
	function automatic [15:0] bitswap_main(input [15:0] d);
		bitswap_main = {d[15], d[14], d[10], d[12], d[11], d[13], d[9], d[8],
		                d[3],  d[2],  d[5],  d[4],  d[7],  d[1],  d[6], d[0]};
	endfunction
	// Sub: 15,14,13,9,11,10,12,8,2,0,5,4,7,3,1,6
	function automatic [15:0] bitswap_sub(input [15:0] d);
		bitswap_sub = {d[15], d[14], d[13], d[9], d[11], d[10], d[12], d[8],
		               d[2],  d[0],  d[5],  d[4], d[7],  d[3],  d[1],  d[6]};
	endfunction

	// ── Range select (MRA byte address) ─────────────────────────────────
	wire enc_main = (ioctl_addr_in >= 27'h020000) && (ioctl_addr_in < 27'h060000);
	wire enc_sub  = (ioctl_addr_in >= 27'h060000) && (ioctl_addr_in < 27'h0A0000);

	wire [3:0] idx_main = ioctl_addr_in[4:1];
	wire [2:0] idx_sub  = ioctl_addr_in[3:1];

	wire [15:0] dec_main = bitswap_main(ioctl_dout_in ^ xor_main(idx_main));
	wire [15:0] dec_sub  = bitswap_sub (ioctl_dout_in ^ xor_sub (idx_sub ));

	// ── Output selection ────────────────────────────────────────────────
	// PRODUCTION MODE: decrypt runtime per usare raiden.zip originale MAME.
	// I range enc_main (0x020000-0x05FFFF) e enc_sub (0x060000-0x09FFFF) sono
	// applicati on-the-fly via XOR + bitswap MAME init_decryption() (raiden.cpp:1187).
	// Verificato byte-by-byte vs raiden2_dec.zip (ROM gia' decifrate offline).
	wire is_rom_dl = ioctl_download_in & (ioctl_index_in == 16'd0);

	assign ioctl_dout_out = is_rom_dl ? (enc_main ? dec_main :
	                                     enc_sub  ? dec_sub  :
	                                                ioctl_dout_in)
	                                  : ioctl_dout_in;

	// Pass-through inalterati
	assign ioctl_addr_out     = ioctl_addr_in;
	assign ioctl_wr_out       = ioctl_wr_in;
	assign ioctl_index_out    = ioctl_index_in;
	assign ioctl_download_out = ioctl_download_in;

endmodule
