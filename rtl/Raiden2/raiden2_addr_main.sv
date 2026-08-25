// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// Raiden II Main V30 address translator (pattern Irem M72 pal.sv).
// Decodifica address V30 → memrq per region. Mappa MAME raiden2_mem
// (raiden2.cpp:626-646), fine-decode del blocco $400-$7FF trascritto dalla
// maincpu_map della repo _old (quella che bootava):
//   $00000-$003FF  RAM (IVT)                 → ram_memrq
//   $00400-$006FF  COP SEI300                → cop_memrq (meno i ritagli sotto)
//     $00470-$00471  cop_tile_bank_2 (cbank) → cbank_memrq
//     $00600-$0063F  Seibu CRTC              → crtc_memrq
//     $00640-$006BF  finestra obj            → obj_memrq
//     $0068E-$0068F  latch buffered spriteram→ sprbuf_memrq (dentro obj)
//     $006CA-$006CB  bank ROM                → bankw_memrq
//     $006CC-$006CD  tile bank               → tbank_memrq
//   $00700-$0071F  Seibu sound (umask 00FF)  → sound_memrq
//   $00740-$00741  DSW1                      → dsw_memrq
//   $00744-$00745  P1+P2                     → p1p2_memrq
//   $00748-$00749  P3+P4                     → p3p4_memrq
//   $0074C-$0074D  SYSTEM                    → sys_memrq
//   $0075C-$0075D  DSW2                      → dsw2_memrq
//   $00762-$00763  sprite_prot_dst1_r        → sprdst_memrq
//   $00800-$0BFFF  RAM                       → ram_memrq
//   $0C000-$0CFFF  spriteram (4KB)           → sprite_memrq
//   $0D000-$0D7FF  BG                        → bg_memrq
//   $0D800-$0DFFF  FG                        → fg_memrq
//   $0E000-$0E7FF  MG                        → mg_memrq
//   $0E800-$0F7FF  TEXT (4KB, non allineata) → text_memrq
//   $0F800-$0FFFF  RAM (stack)               → ram_memrq
//   $10000-$1EFFF  RAM                       → ram_memrq
//   $1F000-$1FFFF  palette (4KB)             → pal_memrq
//   $20000-$3FFFF  ROM bancata (rom_bank)    → ls245_en
//   $40000-$FFFFF  ROM diretta               → ls245_en
//
// ls245_en alto SOLO per le region ROM (SDRAM). Il resto è BRAM/registri
// interni → niente stall.
// sdr_addr: byte addr dentro l'immagine ROM da 1MB (MAIN a base 0 in SDRAM,
// layout della _old). Finestra bancata: {rom_bank, A[16:0]} come la _old.

module raiden2_addr_main
(
	input  logic [19:0] A,
	input  logic        DBEN,     // = rd | wr (bus enable: pulse durante accesso)
	input  logic        rom_bank, // da write $6CB (reset 1, ~wdata[15])
	// Raiden DX: mappa programma DIVERSA (MAME raidendx_mem).
	//   Raiden II: $20000-$3FFFF bancata, 2 pagine da 128K a base 0, sel. $6C9 bit7
	//   Raiden DX: $20000-$2FFFF bancata, 16 pagine da 64K nel 2o MB, sel. $470[15:12]
	//              $30000-$FFFFF ROM diretta (Raiden II: da $40000)
	input  logic        board_dx,
	input  logic  [3:0] dx_prg_bank,

	// SDRAM ROM region
	output logic        ls245_en,
	output logic [23:0] sdr_addr,

	// BRAM / registri (combinatorio)
	output logic        ram_memrq,
	output logic        cop_memrq,
	output logic        cbank_memrq,
	output logic        crtc_memrq,
	output logic        obj_memrq,
	output logic        sprbuf_memrq,
	output logic        bankw_memrq,
	output logic        tbank_memrq,
	output logic        sound_memrq,
	output logic        dsw_memrq,
	output logic        p1p2_memrq,
	output logic        p3p4_memrq,
	output logic        sys_memrq,
	output logic        dsw2_memrq,
	output logic        sprdst_memrq,
	output logic        sprite_memrq,
	output logic        bg_memrq,
	output logic        fg_memrq,
	output logic        mg_memrq,
	output logic        text_memrq,
	output logic        pal_memrq
);

always_comb begin
	ram_memrq    = 1'b0;
	cop_memrq    = 1'b0;
	cbank_memrq  = 1'b0;
	crtc_memrq   = 1'b0;
	obj_memrq    = 1'b0;
	sprbuf_memrq = 1'b0;
	bankw_memrq  = 1'b0;
	tbank_memrq  = 1'b0;
	sound_memrq  = 1'b0;
	dsw_memrq    = 1'b0;
	p1p2_memrq   = 1'b0;
	p3p4_memrq   = 1'b0;
	sys_memrq    = 1'b0;
	dsw2_memrq   = 1'b0;
	sprdst_memrq = 1'b0;
	sprite_memrq = 1'b0;
	bg_memrq     = 1'b0;
	fg_memrq     = 1'b0;
	mg_memrq     = 1'b0;
	text_memrq   = 1'b0;
	pal_memrq    = 1'b0;
	ls245_en     = 1'b0;
	sdr_addr     = 24'd0;

	if (DBEN) begin
		if (A < 20'h00400) begin
			ram_memrq = 1'b1;                       // RAM bassa (IVT)
		end else if (A < 20'h00700) begin
			// blocco COP $400-$6FF con i ritagli della _old (maincpu_map:146-162)
			if (board_dx && (A >= 20'h004D0) && (A < 20'h004D8)) begin
				// Raiden DX: map(0x004d0,0x004d7).ram(). Senza questo ritaglio
				// quelle scritture finiscono al COP come se fossero comandi.
				ram_memrq = 1'b1;
			end else if (A[9:1] == 9'h038) begin    // $470-$471
				cbank_memrq = 1'b1;
			end else if ((A >= 20'h00600) && (A < 20'h00640)) begin
				crtc_memrq = 1'b1;
			end else if ((A >= 20'h0068E) && (A < 20'h00690)) begin
				sprbuf_memrq = 1'b1;                // dentro obj: estratto a parte
			end else if ((A >= 20'h00640) && (A < 20'h006C0)) begin
				obj_memrq = 1'b1;
			end else if ((A >= 20'h006CA) && (A < 20'h006CC)) begin
				bankw_memrq = 1'b1;
			end else if ((A >= 20'h006CC) && (A < 20'h006CE)) begin
				tbank_memrq = 1'b1;
			end else begin
				cop_memrq = 1'b1;                   // include $6FC/$6FE
			end
		end else if ((A >= 20'h00700) && (A < 20'h00720)) begin
			sound_memrq = 1'b1;
		end else if ((A >= 20'h00740) && (A < 20'h00742)) begin
			dsw_memrq = 1'b1;
		end else if ((A >= 20'h00744) && (A < 20'h00746)) begin
			p1p2_memrq = 1'b1;
		end else if ((A >= 20'h00748) && (A < 20'h0074A)) begin
			p3p4_memrq = 1'b1;
		end else if ((A >= 20'h0074C) && (A < 20'h0074E)) begin
			sys_memrq = 1'b1;
		end else if ((A >= 20'h0075C) && (A < 20'h0075E)) begin
			dsw2_memrq = 1'b1;
		end else if ((A >= 20'h00762) && (A < 20'h00764)) begin
			sprdst_memrq = 1'b1;
		end else if (A < 20'h0C000) begin
			ram_memrq = 1'b1;                       // $800-$BFFF (i buchi IO sopra
			                                        // sono già usciti prima)
		end else if (A < 20'h0D000) begin
			sprite_memrq = 1'b1;
		end else if (A < 20'h0D800) begin
			bg_memrq = 1'b1;
		end else if (A < 20'h0E000) begin
			fg_memrq = 1'b1;
		end else if (A < 20'h0E800) begin
			mg_memrq = 1'b1;
		end else if (A < 20'h0F800) begin
			text_memrq = 1'b1;
		end else if (A < 20'h10000) begin
			ram_memrq = 1'b1;                       // stack
		end else if (A < 20'h1F000) begin
			ram_memrq = 1'b1;
		end else if (A < 20'h20000) begin
			pal_memrq = 1'b1;
		end else if (A < 20'h40000) begin
			ls245_en = 1'b1;
			if (board_dx)
				// $20000-$2FFFF: pagina da 64K del 2o MB del programma, che in
				// SDRAM sta a byte $600000 (zona libera oltre OKI1): la mappa
				// delle altre region resta identica a Raiden II.
				// $30000-$3FFFF: gia' ROM diretta su DX.
				sdr_addr = (A < 20'h30000) ? {4'h6, dx_prg_bank, A[15:0]}
				                           : {4'd0, A};
			else
				// ROM bancata: metà da 128KB dell'immagine scelta da rom_bank
				sdr_addr = {6'd0, rom_bank, A[16:0]};
		end else begin
			// $40000-$FFFFF: ROM diretta (immagine 1MB a base 0)
			ls245_en = 1'b1;
			sdr_addr = {4'd0, A};
		end
	end
end

endmodule
