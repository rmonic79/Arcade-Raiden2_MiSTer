// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// sei80bu - SEI80BU Encrypted Z80 Interface
// Implementazione esatta da MAME mame/src/devices/machine/sei80bu.cpp:
//   - data_r:    5 XOR + 2 bitswap (sempre)
//   - opcode_r:  8 XOR + 4 bitswap (data + 3 XOR e 2 bitswap extra solo M1)
// Determinato runtime da z80_m1 (m1=1 → opcode, m1=0 → data).
//
// Usato in Raiden Z80 audio: ROM encryptata, opcode/data path diversi.

module sei80bu (
	input         clk,
	input  [15:0] z80_rom_addr,
	input   [7:0] z80_rom_data,
	input         z80_rom_ok,
	input         z80_rom_cs,
	input         z80_m1,

	output reg  [7:0] decrypt_rom_data,
	output reg        decrypt_rom_ok
);

	wire [15:0] a = z80_rom_addr;

	// ── XOR (5 base + 3 solo opcode) ────────────────────────────────────────
	reg [7:0] s_xor;
	always @(*) begin
		s_xor = z80_rom_data;
		// XOR comuni (data + opcode)
		if ( a[ 9] &  a[ 8])           s_xor[7] = s_xor[7] ^ 1'b1;  // 0x80
		if ( a[11] &  a[ 4] &  a[ 1])  s_xor[6] = s_xor[6] ^ 1'b1;  // 0x40
		if ( a[11] & ~a[ 8] &  a[ 1])  s_xor[2] = s_xor[2] ^ 1'b1;  // 0x04
		if ( a[13] & ~a[ 6] &  a[ 4])  s_xor[1] = s_xor[1] ^ 1'b1;  // 0x02
		if (~a[11] &  a[ 9] &  a[ 2])  s_xor[0] = s_xor[0] ^ 1'b1;  // 0x01
		// XOR solo opcode (M1)
		if (z80_m1) begin
			if (~a[13] &  a[12])       s_xor[5] = s_xor[5] ^ 1'b1;  // 0x20
			if (~a[ 6] &  a[ 1])       s_xor[4] = s_xor[4] ^ 1'b1;  // 0x10
			if (~a[12] &  a[ 2])       s_xor[3] = s_xor[3] ^ 1'b1;  // 0x08
		end
	end

	// ── Bitswap (2 base + 2 solo opcode) ────────────────────────────────────
	// 1) Sempre: (a[13]&a[4]) → swap bit[1]↔bit[0]
	// 2) Sempre: (a[8] &a[4]) → swap bit[3]↔bit[2]
	// 3) Solo opcode: (a[12]&a[9]) → swap bit[5]↔bit[4]
	// 4) Solo opcode: (a[11]&~a[6]) → swap bit[7]↔bit[6]
	reg [7:0] s_bs1, s_bs2, s_bs3, s_bs4;
	always @(*) begin
		s_bs1 = (a[13] & a[ 4]) ? {s_xor[7:2], s_xor[0], s_xor[1]} : s_xor;
		s_bs2 = (a[ 8] & a[ 4]) ? {s_bs1[7:4], s_bs1[2], s_bs1[3], s_bs1[1:0]} : s_bs1;
		s_bs3 = (z80_m1 && (a[12] & a[ 9])) ? {s_bs2[7:6], s_bs2[4], s_bs2[5], s_bs2[3:0]} : s_bs2;
		s_bs4 = (z80_m1 && (a[11] & ~a[ 6])) ? {s_bs3[6], s_bs3[7], s_bs3[5:0]} : s_bs3;
	end

	always @(posedge clk) begin
		if (z80_rom_cs && z80_rom_ok) begin
			decrypt_rom_data <= s_bs4;
			decrypt_rom_ok   <= 1'b1;
		end else begin
			decrypt_rom_ok   <= 1'b0;
		end
	end

endmodule
