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

/*  Audio subsystem Raiden II (Z80 + YM2151 + 2x OKI6295)

    Spec da MAME (legionna.cpp godzilla machine config :1295-1355):
      - Z80 @ 14.318180/4 = 3.579545 MHz (seibu_sound_map + godzilla_sound_io_map)
      - YM2151/OPM (jt51) @ 28.63636/8 = 3.579545 MHz
      - OKI M6295 (jt6295) @ 20MHz/20 = 1.000 MHz, PIN7=HIGH, ROM 512KB
        bancata da IO port 0 (godzilla_oki_bank_w)

    Z80 memory map: seibu_sound_map standard + IO map godzilla (port 0).
    IRQ Z80 (IM0): RST10 ← YM2151 IRQ (timer OPM), RST18 ← main wakeup.
*/

module Raiden2_audio_z80 #(parameter SS_IDX_ZRAM = -1, parameter SS_IDX_Z80 = -1,
                           parameter SS_IDX_GLUE = -1, parameter SS_IDX_YMSH = -1) (
	input  wire        clk,
	input  wire        reset,
	input  wire        pause,
	input  wire  [1:0] clk_sel,

	input  wire        ioctl_download,
	input  wire        ioctl_wr,
	input  wire [26:0] ioctl_addr,
	input  wire [15:0] ioctl_dout,

	input  wire        snd_cs,
	input  wire  [4:1] snd_addr,
	input  wire        snd_wr,
	input  wire        snd_rd,
	input  wire [15:0] snd_wdata,
	output wire [15:0] snd_rdata,
	input  wire        snd_nmi_n,
	input  wire        snd_reset_in,

	input  wire  [7:0] coin_input,

	// OKI ADPCM ROM bridge (SDRAM). Raiden2 pcm.922 = 512KB: bit 18 = bank
	// (Z80 IO port 0, MAME godzilla_oki_bank_w: set_rom_bank(data&1)).
	output wire [18:0] oki_rom_addr,
	input  wire  [7:0] oki_rom_data,
	input  wire        oki_rom_ok,
	// Secondo OKI (MAME raiden2.cpp:1151): ROM su DDR3, porta a priorita' bassa.
	output wire [18:0] oki2_rom_addr,
	input  wire  [7:0] oki2_rom_data,
	input  wire        oki2_rom_ok,

	// Volumi da OSD: 0=Default(tarato), 1=Mute, 2+ = % del Default.
	// QUATTRO bit come Raiden: la stringa OSD dichiara 16 voci (fino a 1000%)
	// e prima al modulo ne arrivavano 3 -> da "200%" in su il bit alto cadeva
	// e le voci aliasavano su Default..200%: meta' menu non faceva nulla.
	input  wire  [3:0] fm_vol_sel,
	input  wire  [3:0] oki_vol_sel,      // master OKI1
	input  wire  [3:0] oki2_vol_sel,     // master OKI2 (MAME instrada 0.40 CIASCUNO)
	input  wire  [2:0] oki_ch_vol_sel0,
	input  wire  [2:0] oki_ch_vol_sel1,
	input  wire  [2:0] oki_ch_vol_sel2,
	input  wire  [2:0] oki_ch_vol_sel3,
	// Le quattro voci del SECONDO OKI: erano cablate a 8'h10 dentro il modulo
	// e non avevano nessuna voce nell'OSD.
	input  wire  [2:0] oki2_ch_vol_sel0,
	input  wire  [2:0] oki2_ch_vol_sel1,
	input  wire  [2:0] oki2_ch_vol_sel2,
	input  wire  [2:0] oki2_ch_vol_sel3,
	// FM per-canale: il YM2151 ha OTTO canali. Le porte di volume sono state
	// aggiunte a jt51 (jt51_acc.v) seguendo lo schema gia' usato su jtopl,
	// e provate con sim_jt51.sh: a unita' l'uscita e' identica bit per bit
	// all'originale, e mutare un canale azzera SOLO quello.
	input  wire  [2:0] fm_ch_vol_sel0,
	input  wire  [2:0] fm_ch_vol_sel1,
	input  wire  [2:0] fm_ch_vol_sel2,
	input  wire  [2:0] fm_ch_vol_sel3,
	input  wire  [2:0] fm_ch_vol_sel4,
	input  wire  [2:0] fm_ch_vol_sel5,
	input  wire  [2:0] fm_ch_vol_sel6,
	input  wire  [2:0] fm_ch_vol_sel7,

	output reg signed [15:0] audio_l,
	output reg signed [15:0] audio_r,

	// ─── Savestate: QUATTRO blocchi, come Raiden (dove i savestate funzionano)
	//   ss_zram  RAM Z80 2K x 8
	//   ss_z80   i 212 bit di stato interno del T80 (REG in, DIR/DIRSet out)
	//   ss_ymsh  shadow dei 256 registri del YM2151 + replay al ripristino
	//   ss_glue  soundlatch/pending/IRQ/vettore/banco/address-latch FM
	//            (ULTIMO indice: il suo commit fa da trigger al replay)
	// z80_ss_ready: Z80 parcheggiato a confine istruzione, fa da gate al DMA.
	ssbus_if.slave     ss_zram,
	ssbus_if.slave     ss_z80,
	ssbus_if.slave     ss_glue,
	ssbus_if.slave     ss_ymsh,
	output wire        z80_ss_ready
);

	// ─── Clock enable: clk_sys 96 MHz → Z80/YM 3,579545 MHz, OKI 1,022727 MHz
	// Sulla scheda i due clock nascono dallo STESSO quarzo da 28,636363 MHz:
	//   Z80 e YM2151 = 28,636363/8  = 3,579545454 MHz
	//   OKI M6295    = 28,636363/28 = 1,022727272 MHz  (PIN7_HIGH)
	// Da 96 MHz nessuno dei due e' un divisore intero, ma i rapporti sono
	// ESATTI:  3,579545454/96 = 105/2816   e   1,022727272/96 = 15/1408.
	// Accumulatore frazionario: somma il numeratore a ogni clk, scatta e
	// sottrae il denominatore. Nessuna deriva, frequenza media esatta.
	// ⚠ La somma va SEMPRE su un bit in piu' dell'accumulatore: se il totale
	// rientra nei bit dell'accumulatore il confronto non scatta piu', il clock
	// enable si perde e il processore SI FERMA (2026-08-23, costo due build).
	localparam [12:0] YM_DEN  = 13'd2816;
	localparam [12:0] YM_NUM  = 13'd105;
	reg  [11:0] cen_z80_acc;
	reg         cen_z80;
	wire [12:0] cen_z80_sum = {1'b0, cen_z80_acc} + YM_NUM;
	always @(posedge clk) begin
		if (reset) begin
			cen_z80_acc <= 12'd0;
			cen_z80     <= 1'b0;
		end else if (cen_z80_sum >= YM_DEN) begin
			cen_z80_acc <= cen_z80_sum[11:0] - YM_DEN[11:0];
			cen_z80     <= 1'b1;
		end else begin
			cen_z80_acc <= cen_z80_sum[11:0];
			cen_z80     <= 1'b0;
		end
	end

	localparam [11:0] OKI_DEN = 12'd1408;
	localparam [11:0] OKI_NUM = 12'd15;
	reg  [10:0] cen_oki_acc;
	reg         cen_oki;
	wire [11:0] cen_oki_sum = {1'b0, cen_oki_acc} + OKI_NUM;
	always @(posedge clk) begin
		if (reset) begin
			cen_oki_acc <= 11'd0;
			cen_oki     <= 1'b0;
		end else if (cen_oki_sum >= OKI_DEN) begin
			cen_oki_acc <= cen_oki_sum[10:0] - OKI_DEN[10:0];
			cen_oki     <= 1'b1;
		end else begin
			cen_oki_acc <= cen_oki_sum[10:0];
			cen_oki     <= 1'b0;
		end
	end

	// ─── Park dello Z80 a confine istruzione (cattura deterministica) ───────
	// M1 basso & MREQ basso & IORQ alto = SOLO il fetch M1, prima che il PC
	// venga incrementato: e' il confine architetturale esatto. Refresh escluso
	// (M1 alto a T3), IACK escluso (IORQ basso), cicli di memoria esclusi.
	reg z80_parked;
	always @(posedge clk) begin
		if (reset || !pause)
			z80_parked <= 1'b0;
		else if (!z80_parked && !z80_m1_n && !z80_mreq_n && z80_iorq_n)
			z80_parked <= 1'b1;
	end
	assign z80_ss_ready = z80_parked;

	// Z80 e FM si fermano solo a park avvenuto (qualche us dopo la pausa);
	// gli OKI seguono la pausa e basta.
	wire cen_z80_g = cen_z80 & ~z80_parked;
	wire cen_oki_g = cen_oki & ~pause;

	// ─── Z80 signals ─────────────────────────────────────────────────────────
	wire [15:0] z80_addr;
	wire  [7:0] z80_dout;
	reg   [7:0] z80_din;
	wire        z80_mreq_n, z80_iorq_n, z80_rd_n, z80_wr_n, z80_m1_n;
	wire        z80_int_n;
	wire        z80_busak_n, z80_halt_n;

	wire rom_lo_cs   = ~z80_mreq_n && (z80_addr[15:13] == 3'b000);
	wire rom_hi_cs   = ~z80_mreq_n && (z80_addr[15] == 1'b1);
	wire ram_cs      = ~z80_mreq_n && (z80_addr[15:11] == 5'b00100);
	wire reg_cs      = ~z80_mreq_n && (z80_addr[15:5] == 11'h200);
	// MAME raiden2_sound_map: $6000 = oki1, $6002 = oki2 -> discrimina addr[1].
	wire oki_blk     = ~z80_mreq_n && (z80_addr[15:12] == 4'h6);
	wire oki_cs      = oki_blk && ~z80_addr[1];
	wire oki2_cs     = oki_blk &&  z80_addr[1];

	// ─── ROM Z80 — Blood Bros bb_07 32KB, layout BRAM 64KB con alias bank 1 ─
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] z80_rom_lo [0:32767];
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] z80_rom_hi [0:32767];
	reg [7:0] z80_rom_lo_q = 8'h00, z80_rom_hi_q = 8'h00;

	// Raiden2: audiocpu z80 a ioctl 0x9A0000..0x9AFFFF (8.016 raw 64KB ESATTI).
	// ATTENZIONE: la finestra DEVE essere 64KB. Con < 0x9C0000 il padding MRA
	// `<part repeat="0x10000">00</part>` a 0x9B0000-0x9BFFFF wrappava su
	// ioctl_addr[15:1] e AZZERAVA l'intera ROM Z80 → Z80 morto, handshake
	// sound morto, attract senza musica e fuori sequenza.
	wire z80_rom_dl_wr =
		ioctl_download && ioctl_wr && (ioctl_addr >= 27'h560000) && (ioctl_addr < 27'h570000);
	wire [14:0] z80_rom_dl_word = ioctl_addr[15:1];

	reg rom_bank;

	wire [15:0] z80_rom_byte_addr =
		rom_lo_cs              ? z80_addr :
		(rom_hi_cs & ~rom_bank) ? z80_addr :
		(rom_hi_cs &  rom_bank) ? {1'b0, z80_addr[14:0]} :
		                          z80_addr;

	reg z80_addr_lsb_d;
	always @(posedge clk) begin
		if (z80_rom_dl_wr) begin
			z80_rom_lo[z80_rom_dl_word] <= ioctl_dout[7:0];
			z80_rom_hi[z80_rom_dl_word] <= ioctl_dout[15:8];
		end
		z80_rom_lo_q   <= z80_rom_lo[z80_rom_byte_addr[15:1]];
		z80_rom_hi_q   <= z80_rom_hi[z80_rom_byte_addr[15:1]];
		z80_addr_lsb_d <= z80_rom_byte_addr[0];
	end

	wire [7:0] z80_rom_q = z80_addr_lsb_d ? z80_rom_hi_q : z80_rom_lo_q;

	// ─── RAM Z80 2KB ─────────────────────────────────────────────────────────
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] z80_ram [0:2047];
	reg [7:0] z80_ram_q = 8'h00;

	// synthesis translate_off
	integer z80_ram_init_i;
	initial begin
		for (z80_ram_init_i = 0; z80_ram_init_i < 2048; z80_ram_init_i = z80_ram_init_i + 1)
			z80_ram[z80_ram_init_i] = 8'h00;
	end
	// synthesis translate_on

	// Savestate adaptor sulla porta CPU della z80_ram (8 bit). Copia 1:1 del
	// cablaggio di Raiden_audio_z80.sv (ss_ram_adaptor WIDTH=8, WIDTHAD=11).
	wire        zram_wren_cpu = ram_cs && !z80_wr_n;
	wire [10:0] zram_idx;
	wire        zram_wren;
	wire  [7:0] zram_wdata_eff;
	ss_ram_adaptor #(.WIDTH(8), .WIDTHAD(11), .SS_IDX(SS_IDX_ZRAM)) u_ss_zram (
		.clk(clk), .wren_in(zram_wren_cpu), .addr_in(z80_addr[10:0]), .wdata_in(z80_dout),
		.wren_out(zram_wren), .addr_out(zram_idx), .wdata_out(zram_wdata_eff),
		.q_in(z80_ram_q), .ssbus(ss_zram)
	);
	always @(posedge clk) begin
		if (zram_wren) z80_ram[zram_idx] <= zram_wdata_eff;
		z80_ram_q <= z80_ram[zram_idx];
	end

	// ─── Sub-region decoder ──────────────────────────────────────────────────
	wire is_pending_w   = reg_cs && (z80_addr[4:0] == 5'h00) && !z80_wr_n;
	wire is_irq_clear   = reg_cs && (z80_addr[4:0] == 5'h01) && !z80_wr_n;
	wire is_rst10_ack   = reg_cs && (z80_addr[4:0] == 5'h02) && !z80_wr_n;
	wire is_rst18_ack   = reg_cs && (z80_addr[4:0] == 5'h03) && !z80_wr_n;
	// raiden2_sound_map (raiden2.cpp:738) sposta bank_w a $401A: la mappa Seibu
	// CONDIVISA lo ha a $4007, che qui NON e' mappato. Confermato sul ROM sonoro:
	// routine a $1116 = LD ($401A),A, e nessun 'LD ($4007),A' in tutti i 64KB.
	wire is_bank_w      = reg_cs && (z80_addr[4:0] == 5'h1A) && !z80_wr_n;
	wire is_ym_access   = reg_cs && (z80_addr[4:1] == 4'h4);
	wire is_ym_w        = is_ym_access && !z80_wr_n;
	wire is_ym_r        = is_ym_access && !z80_rd_n;
	wire is_latch_lo_r  = reg_cs && (z80_addr[4:0] == 5'h10) && !z80_rd_n;
	wire is_latch_hi_r  = reg_cs && (z80_addr[4:0] == 5'h11) && !z80_rd_n;
	wire is_pending_r   = reg_cs && (z80_addr[4:0] == 5'h12) && !z80_rd_n;
	wire is_coin_r      = reg_cs && (z80_addr[4:0] == 5'h13) && !z80_rd_n;
	wire is_data_lo_w   = reg_cs && (z80_addr[4:0] == 5'h18) && !z80_wr_n;
	wire is_data_hi_w   = reg_cs && (z80_addr[4:0] == 5'h19) && !z80_wr_n;
	wire is_coin_w      = reg_cs && (z80_addr[4:0] == 5'h1B) && !z80_wr_n;

	always @(posedge clk) begin
		if (reset)
			rom_bank <= 1'b0;
		else if (glue_wr)
			rom_bank <= glue_out[0];
		else if (cen_z80 && is_bank_w)
			rom_bank <= z80_dout[0];
	end

	// ─── Soundlatch main↔sub state ───────────────────────────────────────────
	reg [7:0] main2sub [0:1];
	reg [7:0] sub2main [0:1];
	reg       main2sub_pending;
	reg       sub2main_pending;

	// composto in fondo al file (dopo le dichiarazioni): vedi `assign glue_in`
	// ─── Savestate glue: tutto lo stato sparso fuori dalle memorie ──────────
	// 73 bit: [0]=rom_bank [8:1]=m2s0 [16:9]=m2s1 [24:17]=s2m0 [32:25]=s2m1
	// [33]=m2s_pend [34]=s2m_pend [35]=rst10_irq [36]=rst10_srv [37]=rst18_irq
	// [38]=rst18_srv [46:39]=iack_vector_latched [54:47]=ym_addr_sel
	// [55]=ym_irq_d [56]=iack_active_d
	// [57]=oki1_cmd_pend [64:58]=oki1_phrase [65]=oki2_cmd_pend [72:66]=oki2_phrase
	wire [72:0] glue_out;
	wire        glue_wr;
	wire [72:0] glue_in;
	reg         oki_cmd_pending_r,  oki2_cmd_pending_r;  // 1o byte scritto, atteso il 2o
	reg  [6:0]  oki_phrase_r,       oki2_phrase_r;
	reg  [7:0]  ym_addr_sel;   // address latch del YM2151 (snoop, vedi shadow)
	auto_save_adaptor #(.N_BITS(73), .SS_IDX(SS_IDX_GLUE)) u_ss_glue (
		.clk(clk), .ssbus(ss_glue),
		.bits_in(glue_in), .bits_out(glue_out), .bits_wr(glue_wr)
	);

	// ─── IRQ controller (IM0 RST10/RST18) ────────────────────────────────────
	reg rst10_irq, rst10_service;
	reg rst18_irq, rst18_service;
	wire ym_irq_n;
	wire ym_irq = ~ym_irq_n;
	reg  ym_irq_d;

	wire iack_active = ~z80_m1_n && ~z80_iorq_n;
	reg  iack_active_d;
	reg  [7:0] iack_vector_latched;
	wire [7:0] iack_vector_now =
	    (rst18_irq && !rst18_service) ? 8'hDF :
	    (rst10_irq && !rst10_service) ? 8'hD7 :
	                                    8'h00;
	always @(posedge clk) begin
		if (reset) begin
			iack_active_d       <= 1'b0;
			iack_vector_latched <= 8'h00;
		end else begin
			iack_active_d <= iack_active;
			if (glue_wr) begin
				iack_vector_latched <= glue_out[46:39];
				iack_active_d       <= glue_out[56];
			end else if (iack_active && !iack_active_d) begin
				iack_vector_latched <= iack_vector_now;
			end
		end
	end
	wire [7:0] iack_vector = iack_active_d ? iack_vector_latched : iack_vector_now;

	wire irq_active = (rst10_irq && !rst10_service) || (rst18_irq && !rst18_service);
	assign z80_int_n = ~irq_active;

	always @(posedge clk) begin
		if (reset) begin
			rst10_irq     <= 1'b0;
			rst10_service <= 1'b0;
			rst18_irq     <= 1'b0;
			rst18_service <= 1'b0;
			ym_irq_d      <= 1'b0;
		end else if (glue_wr) begin
			rst10_irq     <= glue_out[35];
			rst10_service <= glue_out[36];
			rst18_irq     <= glue_out[37];
			rst18_service <= glue_out[38];
			ym_irq_d      <= glue_out[55];
		end else begin
			ym_irq_d <= ym_irq;
			if (ym_irq && !ym_irq_d)        rst10_irq <= 1'b1;
			else if (!ym_irq && ym_irq_d)   rst10_irq <= 1'b0;

			if (snd_cs && snd_wr && snd_addr == 4'd4)
				rst18_irq <= 1'b1;

			if (iack_active_d && !iack_active) begin
				if (iack_vector_latched == 8'hDF) begin
					rst18_service <= 1'b1;
					rst18_irq     <= 1'b0;
				end else if (iack_vector_latched == 8'hD7) begin
					rst10_service <= 1'b1;
				end
			end

			if (cen_z80) begin
				if (is_irq_clear)  rst18_service <= 1'b0;
				if (is_rst10_ack)  rst10_service <= 1'b0;
				if (is_rst18_ack)  rst18_service <= 1'b0;
			end
		end
	end

	// ─── Soundlatch main_w/r logic ──────────────────────────────────────────
`ifdef MISTER_SIM
	// SIM MODE: stub Z80 — simula coin1 + start1 a tempi prestabiliti per
	// avanzare oltre la INTRO/title screen del game.
	//
	// SEIBU coin protocol: main scrive a 0x100702 (snd_addr=1, "cmd_b"),
	// Z80 risponde tramite sub2main[0] (letto da main 68K a 0x100704).
	// Valore 0xA0 = idle (no coin), 0xA1 = coin1 inserted.
	//
	// Plan:
	//   - Per ~10M tick (≈ 100ms sim) restituisco 0xA0 (idle, attract mode)
	//   - Per i successivi 5M tick restituisco 0xA1 (coin1 inserted)
	//   - Poi di nuovo 0xA0 (release)
	//   - Poi 0xA1 di nuovo (start1 — same encoding tipicamente)
	//   - Poi 0xA0 fisso (gioco in corso)
	reg [27:0] sim_phase_cnt = 0;
	reg [7:0]  sim_coin_resp = 8'hA0;
	always @(posedge clk) begin
		if (reset) begin
			sim_phase_cnt <= 0;
			sim_coin_resp <= 8'hA0;
		end else begin
			sim_phase_cnt <= sim_phase_cnt + 1;
			// 50M tick (~520ms @96MHz) → press coin1
			if      (sim_phase_cnt == 28'd50_000_000) sim_coin_resp <= 8'hA1;
			else if (sim_phase_cnt == 28'd55_000_000) sim_coin_resp <= 8'hA0;
			// 70M tick → press start1
			else if (sim_phase_cnt == 28'd70_000_000) sim_coin_resp <= 8'hA1;
			else if (sim_phase_cnt == 28'd75_000_000) sim_coin_resp <= 8'hA0;
		end
	end

	always @(posedge clk) begin
		if (reset) begin
			main2sub[0]      <= 8'd0;
			main2sub[1]      <= 8'd0;
			sub2main[0]      <= 8'd0;
			sub2main[1]      <= 8'd0;
			main2sub_pending <= 1'b0;
			sub2main_pending <= 1'b0;
		end else if (glue_wr) begin
			main2sub[0]      <= glue_out[8:1];
			main2sub[1]      <= glue_out[16:9];
			sub2main[0]      <= glue_out[24:17];
			sub2main[1]      <= glue_out[32:25];
			main2sub_pending <= glue_out[33];
			sub2main_pending <= glue_out[34];
		end else begin
			if (snd_cs && snd_wr) begin
				case (snd_addr)
					4'd0: begin
						main2sub[0]      <= snd_wdata[7:0];
						main2sub_pending <= 1'b1;
					end
					4'd1: begin
						// SEIBU cmd: simula Z80 response. Il 68k legge STATUS da
						// $10070C=sub2main[1] e STROBE/coin-bitmask da $100708=sub2main[0]
						// ($CCA: cmpi.b #$A0,$108050(=sub2main[1]); tst.b $108052(=sub2main[0])).
						// PRIMA era invertito (status in [0]) -> coin mai accreditata in sim.
						main2sub[1]      <= snd_wdata[7:0];
						sub2main[1]      <= sim_coin_resp;   // STATUS $A0/$A1 -> $10070C
						sub2main[0]      <= (sim_coin_resp == 8'hA1) ? 8'h01 : 8'h00; // coin strobe bit0 -> $100708
						sub2main_pending <= 1'b1;
						main2sub_pending <= 1'b0;
					end
					4'd2, 4'd6: begin
						sub2main_pending <= 1'b0;
						main2sub_pending <= 1'b1;
					end
					default: ;
				endcase
			end
		end
	end
`else
	always @(posedge clk) begin
		if (reset) begin
			main2sub[0]      <= 8'd0;
			main2sub[1]      <= 8'd0;
			sub2main[0]      <= 8'd0;
			sub2main[1]      <= 8'd0;
			main2sub_pending <= 1'b0;
			sub2main_pending <= 1'b0;
		end else begin
			if (snd_cs && snd_wr) begin
				case (snd_addr)
					4'd0: main2sub[0] <= snd_wdata[7:0];
					4'd1: main2sub[1] <= snd_wdata[7:0];
					4'd2, 4'd6: begin
						sub2main_pending <= 1'b0;
						main2sub_pending <= 1'b1;
					end
					default: ;
				endcase
			end
			if (cen_z80) begin
				if (is_data_lo_w) sub2main[0] <= z80_dout;
				if (is_data_hi_w) sub2main[1] <= z80_dout;
				if (is_pending_w) begin
					main2sub_pending <= 1'b0;
					sub2main_pending <= 1'b1;
				end
			end
		end
	end
`endif

	// MAME seibu_sound_device::main_r (seibusound.cpp:279):
	//   case 2,3: return sub2main[offset-2]   (soundlatch data: coin/input dal Z80)
	//   case 5:   return main2sub_pending ? 1 : 0
	//   default:  return 0xff
	// snd_addr ora E' l'offset seibu (0..7) = bus_addr[4:2]. Mappa 68k:
	//   $100708->2 sub2main[0], $10070C->3 sub2main[1], $100714->5 main2sub_pending.
	// Il 68k a $B1E/$B24 legge $100708/$10070C = sub2main (risposta coin Z80 $A0/$A1);
	// a $B10 legge $100714 = main2sub_pending (BTST#0: 0 a riposo -> boot prosegue).
	wire [7:0] main_r_data =
		(snd_addr == 4'd2)  ? sub2main[0] :
		(snd_addr == 4'd3)  ? sub2main[1] :
		(snd_addr == 4'd5)  ? {7'd0, main2sub_pending} :
		                       8'hFF;
	assign snd_rdata = {8'h00, main_r_data};

	// ─── Volume da OSD (identico a Raiden) ──────────────────────────────────
	// Guadagni di partenza tarati, NON unita': l'OKI sta piu' alto dell'FM.
	// Raiden usa 0x40/0x70, ma sono tarati sul mix di Raiden: su Cup Soccer
	// il livello assoluto risultava troppo alto. Qui si tiene l'FM al valore
	// gia' collaudato (1.0x) e si applica solo il RAPPORTO di Raiden, che e'
	// la parte che serviva: OKI 1.75x l'FM.
	// Tarati a orecchio sul core: il rapporto giusto e' OKI ~0.875x l'FM (il
	// 1.75x di Raiden mandava il meter in rosso, perche' INTERPOL(1) porta gia'
	// guadagno suo). Poi entrambi scesi di ~3 dB.
	// Q4.4 -> passo 1/16: il rapporto 7/8 e' esatto solo a 0 dB (16/14) e a
	// -6 dB (8/7); a -3 dB la coppia migliore e' 11/10 (errore +0.3 dB).
	// OKI: 0x0A, il valore di prima (per chip).
	// FM: alzato da 0x0B a 0x0D su indicazione dell'utente = +1.6 dB
	// sulla sola musica. Il volume lo valuta lui a orecchio, non si misura.
	localparam [7:0] DEF_GAIN_FM  = 8'h0D;   // FM  0.8125x (era 0x0B = 0.6875x)
	localparam [7:0] DEF_GAIN_OKI = 8'h0A;   // OKI 0.625x  (invariato)

	// Scala a 4 bit, 1:1 con le 16 voci dichiarate nella stringa OSD.
	// Stessa semantica di Raiden: 0=Default, 1=Mute, 2+ = % del Default.
	function [11:0] osd_mul_aud;
		input [3:0] sel;
		case (sel)
			4'd2:  osd_mul_aud = 12'd64;    // 25%
			4'd3:  osd_mul_aud = 12'd128;   // 50%
			4'd4:  osd_mul_aud = 12'd192;   // 75%
			4'd5:  osd_mul_aud = 12'd256;   // 100%
			4'd6:  osd_mul_aud = 12'd320;   // 125%
			4'd7:  osd_mul_aud = 12'd384;   // 150%
			4'd8:  osd_mul_aud = 12'd512;   // 200%
			4'd9:  osd_mul_aud = 12'd640;   // 250%
			4'd10: osd_mul_aud = 12'd768;   // 300%
			4'd11: osd_mul_aud = 12'd1024;  // 400%
			4'd12: osd_mul_aud = 12'd1280;  // 500%
			4'd13: osd_mul_aud = 12'd1792;  // 700%
			4'd14: osd_mul_aud = 12'd2560;  // 1000%
			default: osd_mul_aud = 12'd256;
		endcase
	endfunction

	// sel: 0=Default(tarato), 1=Mute, 2+ = % del Default.
	function [7:0] gain_resolve;
		input [3:0] sel;
		input [7:0] def_g;
		reg [19:0] scaled;
		begin
			case (sel)
				4'd0: gain_resolve = def_g;    // Default (tarato)
				4'd1: gain_resolve = 8'h00;    // Mute
				default: begin
					scaled = def_g * osd_mul_aud(sel);
					gain_resolve = (scaled[19:8] > 12'hFF) ? 8'hFF : scaled[15:8];
				end
			endcase
		end
	endfunction

	// Per-canale: 3 bit, 8 voci = le prime otto della tabella dei master.
	// Serve a far entrare nella parola di stato anche i 32 slot di savestate:
	// oltre il 150% un trim di canale non serve, mentre sui master la scala
	// intera resta.
	function [7:0] gain_resolve_ch;
		input [2:0] sel;
		input [7:0] def_g;
		begin
			gain_resolve_ch = gain_resolve({1'b0, sel}, def_g);
		end
	endfunction

	// ─── YM2151 (jt51) mono — Raiden II usa OPM, non OPL2 ───────────────────
	// MAME raiden2.cpp base_ym2151: ym2151_device @ XTAL(28'636'363)/8, route
	// ALL_OUTPUTS 0.50 su mono. Il chip e' quello di Godzilla/Rainbow, non
	// quello di Seibu Cup: qui non esistono i volumi per-canale del jtopl2.
	// cen_p1: jt51 vuole un secondo enable a meta' rate del cen principale.
	reg  cen_fm_half;
	always @(posedge clk) begin
		if (reset) cen_fm_half <= 1'b0;
		else if (cen_z80_g) cen_fm_half <= ~cen_fm_half;
	end
	wire cen_fm_p1_g = cen_z80_g & cen_fm_half;

	// jt51 campiona la write su UN solo impulso con addr/dato stabili.
	reg  ym_wr_d;
	always @(posedge clk) ym_wr_d <= is_ym_w;
	wire ym_wr_pulse = is_ym_w & ~ym_wr_d;

	wire [7:0] ym_dout;
	wire signed [15:0] ym_left, ym_right;
	// Gain per-canale FM (default 0x10 = unita' -> mix identico a prima)
	reg [7:0] fm_chvol0, fm_chvol1, fm_chvol2, fm_chvol3,
	          fm_chvol4, fm_chvol5, fm_chvol6, fm_chvol7;
	always @(posedge clk) begin
		fm_chvol0 <= gain_resolve_ch(fm_ch_vol_sel0, 8'h10);
		fm_chvol1 <= gain_resolve_ch(fm_ch_vol_sel1, 8'h10);
		fm_chvol2 <= gain_resolve_ch(fm_ch_vol_sel2, 8'h10);
		fm_chvol3 <= gain_resolve_ch(fm_ch_vol_sel3, 8'h10);
		fm_chvol4 <= gain_resolve_ch(fm_ch_vol_sel4, 8'h10);
		fm_chvol5 <= gain_resolve_ch(fm_ch_vol_sel5, 8'h10);
		fm_chvol6 <= gain_resolve_ch(fm_ch_vol_sel6, 8'h10);
		fm_chvol7 <= gain_resolve_ch(fm_ch_vol_sel7, 8'h10);
	end

	// ─── Shadow dei 256 registri YM2151 + REPLAY al ripristino ──────────────
	// Snoop: ogni write dello Z80 al chip aggiorna shadow[reg] e l'address
	// latch. Al restore (glue_wr = commit dell'ULTIMA sezione, quindi la
	// shadow e' gia' tornata) la FSM riscrive tutti i 256 registri nel jt51 a
	// passo cen, con lo Z80 in WAIT: timbri, note e timer tornano, e la musica
	// riprende dal punto esatto invece di ripartire muta.
	wire       ym_wr_cen = cen_z80_g & is_ym_w;
	wire       ymsh_wren;
	wire [7:0] ymsh_idx, ymsh_wdata;
	reg  [7:0] ymsh_q;
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] ym_shadow [0:255];

	always @(posedge clk) begin
		if (reset)                          ym_addr_sel <= 8'd0;
		else if (glue_wr)                   ym_addr_sel <= glue_out[54:47];
		else if (ym_wr_cen && !z80_addr[0]) ym_addr_sel <= z80_dout;
	end

	ss_ram_adaptor #(.WIDTH(8), .WIDTHAD(8), .SS_IDX(SS_IDX_YMSH)) u_ss_ymsh (
		.clk(clk), .wren_in(ym_wr_cen & z80_addr[0]), .addr_in(ym_addr_sel), .wdata_in(z80_dout),
		.wren_out(ymsh_wren), .addr_out(ymsh_idx), .wdata_out(ymsh_wdata),
		.q_in(ymsh_q), .ssbus(ss_ymsh)
	);

	reg       rp_active;
	reg       rp_pre;     // passo iniziale: pulisce i flag timer stantii
	reg       rp_final;   // passo finale: rimette l'address latch del chip
	reg [7:0] rp_reg;
	reg [1:0] rp_ph;      // 0=scrivi addr, 1=attesa, 2=scrivi dato, 3=attesa lunga
	reg [6:0] rp_wait;
	// La lettura di save esclude il replay COMBINATORIAMENTE: senza, la prima
	// word del chunk verrebbe salvata da rp_reg (sfasata di uno).
	wire ymsh_ss_rd = ss_ymsh.access(SS_IDX_YMSH) && ss_ymsh.read;
	wire [7:0] ymsh_raddr = (rp_active && !ymsh_ss_rd) ? rp_reg : ymsh_idx;
	always @(posedge clk) begin
		if (ymsh_wren) ym_shadow[ymsh_idx] <= ymsh_wdata;
		ymsh_q <= ym_shadow[ymsh_raddr];
	end

	always @(posedge clk) begin
		if (reset) begin
			rp_active <= 1'b0; rp_pre <= 1'b0; rp_final <= 1'b0;
			rp_reg    <= 8'd0; rp_ph  <= 2'd0; rp_wait  <= 7'd0;
		end else if (glue_wr) begin
			rp_active <= 1'b1; rp_pre <= 1'b1; rp_final <= 1'b0;
			rp_reg    <= 8'd0; rp_ph  <= 2'd0; rp_wait  <= 7'd0;
		end else if (rp_active && ymsh_ss_rd) begin
			rp_active <= 1'b0;   // save partito durante il replay: si abortisce
		end else if (rp_active && cen_z80_g) begin
			case (rp_ph)
				2'd0: begin rp_ph <= 2'd1; rp_wait <= 7'd8;   end
				2'd1: begin
					if (|rp_wait) rp_wait <= rp_wait - 1'b1;
					else if (rp_final) rp_active <= 1'b0;
					else               rp_ph     <= 2'd2;
				end
				2'd2: begin rp_ph <= 2'd3; rp_wait <= 7'd100; end
				2'd3: begin
					if (|rp_wait) rp_wait <= rp_wait - 1'b1;
					else begin
						rp_ph <= 2'd0;
						if (rp_pre) rp_pre <= 1'b0;
						else begin
							rp_reg <= rp_reg + 1'b1;
							if (rp_reg == 8'd255) rp_final <= 1'b1;
						end
					end
				end
			endcase
		end
	end

	// Bus del FM: durante il replay comanda la FSM (Z80 in WAIT).
	// Passo iniziale su YM2151 = registro $14 (controllo timer/IRQ) con i bit
	// F Reset A/B a 1: azzera i flag di timer rimasti dal pre-load. Sul YM3812
	// di Raiden l'equivalente e' reg 4 <= 0x80.
	wire       rp_wr  = rp_active && (rp_ph == 2'd0 || rp_ph == 2'd2);
	wire       rp_a0  = (rp_ph == 2'd2);
	wire [7:0] rp_din = rp_a0    ? (rp_pre ? 8'h30 : ymsh_q) :
	                    rp_pre   ? 8'h14 :
	                    rp_final ? ym_addr_sel : rp_reg;

	jt51 u_jt51 (
		.rst    (reset),
		.clk    (clk),
		.cen    (cen_z80_g),
		.cen_p1 (cen_fm_p1_g),
		.cs_n   (rp_active ? ~rp_wr : ~is_ym_access),
		.wr_n   (rp_active ? ~rp_wr : ~ym_wr_pulse),
		.a0     (rp_active ? rp_a0  : z80_addr[0]),
		.din    (rp_active ? rp_din : z80_dout),
		.fmvol0 (fm_chvol0), .fmvol1 (fm_chvol1),
		.fmvol2 (fm_chvol2), .fmvol3 (fm_chvol3),
		.fmvol4 (fm_chvol4), .fmvol5 (fm_chvol5),
		.fmvol6 (fm_chvol6), .fmvol7 (fm_chvol7),
		.dout   (ym_dout),
		.ct1    (),
		.ct2    (),
		.irq_n  (ym_irq_n),
		.sample (),
		// Uscite DEL DAC, non la somma esatta. ymfm (ym2151::generate):
		//   m_fm.output(...)            <- somma dei canali, "no intermediate clipping"
		//   output->roundtrip_fp()      <- troncatura del DAC esterno YM3012
		//                                  (mantissa 10 + esponente 3)
		// In jt51 quel round-trip sono left/right (lin2exp -> exp2lin su xleft);
		// xleft/xright sono la somma esatta, cioe' il chip SENZA il suo DAC.
		// Misurato prima di cambiare: il clamp a 16 bit dentro jt51 (l'unica cosa
		// che rendeva diversi i due percorsi) su 1.302.520 somme di canale di
		// gioco reale non e' scattato NEMMENO UNA VOLTA, quindi qui left/right
		// sono esattamente "somma non tagliata + DAC" come in ymfm.
		.left   (ym_left),
		.right  (ym_right),
		.xleft  (),
		.xright ()
	);
	wire signed [15:0] ym_snd = (ym_left >>> 1) + (ym_right >>> 1);

	// ─── OKI: cupsoc usa seibu_sound_map standard SENZA bank register
	// (il godzilla_sound_io_map con OUT(0) non esiste qui: rb-ad.922 = 128KB,
	// niente banking). oki_bank fisso a 0.
	wire oki_bank = 1'b0;

	// ─── Stato delle voci OKI al ripristino ─────────────────────────────────
	// Lo stato interno dei 6295 non e' salvabile: al restore un campione in
	// riproduzione andrebbe avanti (effetto stantio). Al commit del glue si
	// scrivono tre comandi sul bus di ENTRAMBI i chip (Z80 fermo): 0x00
	// neutralizza un eventuale primo byte gia' nel chip, 0x78 ferma tutte e
	// quattro le voci, e se il salvataggio era capitato IN MEZZO ai due byte
	// di un comando si riscrive {1,phrase} per riarmarlo — il secondo byte lo
	// scrivera' lo Z80 ripristinato. In gioco normale non si vede.
	reg [3:0] okistop_cnt;
	always @(posedge clk) begin
		if (reset)             okistop_cnt <= 4'd0;
		else if (glue_wr)      okistop_cnt <= 4'd11;
		else if (|okistop_cnt) okistop_cnt <= okistop_cnt - 1'b1;
	end
	wire ok_wrA = (okistop_cnt==4'd11)||(okistop_cnt==4'd10);   // 0x00
	wire ok_wrB = (okistop_cnt==4'd7) ||(okistop_cnt==4'd6);    // 0x78
	wire ok_wrC1 = ((okistop_cnt==4'd3)||(okistop_cnt==4'd2)) && oki_cmd_pending_r;
	wire ok_wrC2 = ((okistop_cnt==4'd3)||(okistop_cnt==4'd2)) && oki2_cmd_pending_r;
	wire       okistop_wr   = ok_wrA | ok_wrB | ok_wrC1;
	wire       okistop2_wr  = ok_wrA | ok_wrB | ok_wrC2;
	wire [7:0] okistop_din  = ok_wrA ? 8'h00 : ok_wrB ? 8'h78 : {1'b1, oki_phrase_r};
	wire [7:0] okistop2_din = ok_wrA ? 8'h00 : ok_wrB ? 8'h78 : {1'b1, oki2_phrase_r};

	// Snoop del protocollo a due byte, uno per chip
	reg oki_wrline_d, oki2_wrline_d;
	always @(posedge clk) begin
		oki_wrline_d  <= (oki_cs  & ~z80_wr_n);
		oki2_wrline_d <= (oki2_cs & ~z80_wr_n);
		if (reset) begin
			oki_cmd_pending_r  <= 1'b0;  oki_phrase_r  <= 7'd0;
			oki2_cmd_pending_r <= 1'b0;  oki2_phrase_r <= 7'd0;
		end else if (glue_wr) begin
			oki_cmd_pending_r  <= glue_out[57];  oki_phrase_r  <= glue_out[64:58];
			oki2_cmd_pending_r <= glue_out[65];  oki2_phrase_r <= glue_out[72:66];
		end else begin
			if ((oki_cs & ~z80_wr_n) & ~oki_wrline_d) begin
				if (!oki_cmd_pending_r && z80_dout[7]) begin
					oki_cmd_pending_r <= 1'b1;
					oki_phrase_r      <= z80_dout[6:0];
				end else oki_cmd_pending_r <= 1'b0;
			end
			if ((oki2_cs & ~z80_wr_n) & ~oki2_wrline_d) begin
				if (!oki2_cmd_pending_r && z80_dout[7]) begin
					oki2_cmd_pending_r <= 1'b1;
					oki2_phrase_r      <= z80_dout[6:0];
				end else oki2_cmd_pending_r <= 1'b0;
			end
		end
	end

	// ─── OKI M6295 (jt6295) — MAME raiden2.cpp: XTAL 28.63636/28 = 1.0227 MHz,
	// PIN7_HIGH, DUE chip instradati 0.40 ciascuno.
	wire [7:0] oki_dout;
	wire signed [13:0] oki_sound;
	wire        oki_sample;
	wire [17:0] oki_addr_int;

	// Gain per-voce OKI (default 0x10 = unita' -> mix identico a prima)
	reg [7:0] oki_chvol0, oki_chvol1, oki_chvol2, oki_chvol3;
	reg [7:0] oki2_chvol0, oki2_chvol1, oki2_chvol2, oki2_chvol3;
	always @(posedge clk) begin
		oki_chvol0 <= gain_resolve_ch(oki_ch_vol_sel0, 8'h10);
		oki_chvol1 <= gain_resolve_ch(oki_ch_vol_sel1, 8'h10);
		oki_chvol2 <= gain_resolve_ch(oki_ch_vol_sel2, 8'h10);
		oki_chvol3 <= gain_resolve_ch(oki_ch_vol_sel3, 8'h10);
		oki2_chvol0 <= gain_resolve_ch(oki2_ch_vol_sel0, 8'h10);
		oki2_chvol1 <= gain_resolve_ch(oki2_ch_vol_sel1, 8'h10);
		oki2_chvol2 <= gain_resolve_ch(oki2_ch_vol_sel2, 8'h10);
		oki2_chvol3 <= gain_resolve_ch(oki2_ch_vol_sel3, 8'h10);
	end

	// INTERPOL(1) come Raiden: upsampler FIR 4x (jt6295_up4.hex nella root del
	// progetto) + compensazione x4 dell'accumulatore. Costa 1 M10K e 1 DSP.
	jt6295 #(.INTERPOL(1)) u_jt6295 (
		.rst       (reset),
		.clk       (clk),
		.cen       (cen_oki_g),
		.ss        (1'b1),                // PIN7 = HIGH
		.wrn       (okistop_wr ? 1'b0 : ~(oki_cs & ~z80_wr_n)),
		.din       (okistop_wr ? okistop_din : z80_dout),
		.dout      (oki_dout),
		.rom_addr  (oki_addr_int),
		.rom_data  (oki_rom_data),
		.rom_ok    (oki_rom_ok),
		.chvol0    (oki_chvol0),
		.chvol1    (oki_chvol1),
		.chvol2    (oki_chvol2),
		.chvol3    (oki_chvol3),
		.sound     (oki_sound),
		.sample    (oki_sample)
	);
	assign oki_rom_addr = {oki_bank, oki_addr_int};

	// ─── Secondo OKI M6295 — identico al primo, ROM separata su DDR3 ────────
	wire [7:0] oki2_dout;
	wire signed [13:0] oki2_sound;
	wire        oki2_sample;
	wire [17:0] oki2_addr_int;
	jt6295 #(.INTERPOL(1)) u_jt6295_2 (
		.rst       (reset),
		.clk       (clk),
		.cen       (cen_oki_g),
		.ss        (1'b1),                // PIN7 = HIGH
		.wrn       (okistop2_wr ? 1'b0 : ~(oki2_cs & ~z80_wr_n)),
		.din       (okistop2_wr ? okistop2_din : z80_dout),
		.dout      (oki2_dout),
		.rom_addr  (oki2_addr_int),
		.rom_data  (oki2_rom_data),
		.rom_ok    (oki2_rom_ok),
		.chvol0    (oki2_chvol0), .chvol1 (oki2_chvol1),
		.chvol2    (oki2_chvol2), .chvol3 (oki2_chvol3),
		.sound     (oki2_sound),
		.sample    (oki2_sample)
	);
	assign oki2_rom_addr = {1'b0, oki2_addr_int};

	// ─── Z80 din mux (pending_r = sub2main_pending, verified DCon) ──────────
	always @(*) begin
		if (iack_active)         z80_din = iack_vector;
		else if (rom_lo_cs)      z80_din = z80_rom_q;
		else if (rom_hi_cs)      z80_din = z80_rom_q;
		else if (ram_cs)         z80_din = z80_ram_q;
		else if (is_ym_r)        z80_din = ym_dout;
		else if (is_latch_lo_r)  z80_din = main2sub[0];
		else if (is_latch_hi_r)  z80_din = main2sub[1];
		else if (is_pending_r)   z80_din = {7'd0, sub2main_pending};
		else if (is_coin_r)      z80_din = coin_input;
		else if (oki_cs)         z80_din = oki_dout;
		else if (oki2_cs)        z80_din = oki2_dout;
		else                     z80_din = 8'hFF;
	end

	// ─── Ripristino DETERMINISTICO dei registri Z80 ─────────────────────────
	// DIRSet a CPU libera caricherebbe i registri a meta' istruzione: una
	// istruzione spazzatura e esito variabile. Quindi: alla prima write di
	// restore su un nostro blocco lo Z80 va in RESET (confine pulito); al
	// commit finale (glue_wr) si rilascia il reset e si da' DIRSet il ciclo
	// dopo, su CPU vergine. Timeout di sicurezza: un salvataggio vecchio
	// senza il blocco glue non lascia lo Z80 in reset per sempre.
	reg        restoring;
	reg        dirset_arm;
	reg [21:0] restore_tmo;
	wire restore_wr_any = (ss_zram.access(SS_IDX_ZRAM) & ss_zram.write)
	                    | (ss_z80.access(SS_IDX_Z80)   & ss_z80.write)
	                    | (ss_ymsh.access(SS_IDX_YMSH) & ss_ymsh.write)
	                    | (ss_glue.access(SS_IDX_GLUE) & ss_glue.write);
	always @(posedge clk) begin
		if (reset) begin
			restoring <= 1'b0; dirset_arm <= 1'b0; restore_tmo <= 22'd0;
		end else begin
			dirset_arm <= 1'b0;
			if (glue_wr) begin
				restoring  <= 1'b0;
				dirset_arm <= 1'b1;
			end else if (restore_wr_any) begin
				restoring   <= 1'b1;
				restore_tmo <= 22'd0;
			end else if (restoring) begin
				restore_tmo <= restore_tmo + 1'b1;
				if (&restore_tmo) restoring <= 1'b0;
			end
		end
	end

	wire [211:0] z80_reg_out;
	wire [211:0] z80_dir;
	wire         z80_dir_set;
	auto_save_adaptor #(.N_BITS(212), .SS_IDX(SS_IDX_Z80)) u_ss_z80 (
		.clk(clk), .ssbus(ss_z80),
		.bits_in(z80_reg_out), .bits_out(z80_dir), .bits_wr(z80_dir_set)
	);

	// ─── T80s Z80 core ───────────────────────────────────────────────────────
	wire t80_busrq_n   = 1'b1;
	wire t80_wait_n    = ~rp_active;   // Z80 in WAIT durante il replay del FM
	wire t80_nmi_n     = 1'b1;
	wire t80_reset_n   = ~reset & ~snd_reset_in & ~restoring;

	T80s u_z80 (
		.RESET_n (t80_reset_n),
		.CLK     (clk),
		.CEN     (cen_z80_g),
		.WAIT_n  (t80_wait_n),
		.INT_n   (z80_int_n),
		.NMI_n   (t80_nmi_n),
		.BUSRQ_n (t80_busrq_n),
		.M1_n    (z80_m1_n),
		.MREQ_n  (z80_mreq_n),
		.IORQ_n  (z80_iorq_n),
		.RD_n    (z80_rd_n),
		.WR_n    (z80_wr_n),
		.RFSH_n  (),
		.HALT_n  (z80_halt_n),
		.BUSAK_n (z80_busak_n),
		.OUT0    (1'b0),
		.A       (z80_addr),
		.DI      (z80_din),
		.DO      (z80_dout),
		.REG     (z80_reg_out),
		.DIRSet  (dirset_arm),
		.DIR     (z80_dir)
	);

	assign glue_in = { oki2_phrase_r, oki2_cmd_pending_r,
	                   oki_phrase_r,  oki_cmd_pending_r,
	                   iack_active_d, ym_irq_d, ym_addr_sel, iack_vector_latched,
	                   rst18_service, rst18_irq, rst10_service, rst10_irq,
	                   sub2main_pending, main2sub_pending,
	                   sub2main[1], sub2main[0], main2sub[1], main2sub[0],
	                   rom_bank };

	// ─── Mixer audio: YM2151 mono + OKI mono → AUDIO_L/R (schema Raiden) ─────
	// Registrati: erano il percorso critico (status[86] dell'OSD entrava
	// combinatorio in gain_resolve -> moltiplicatore audio -> soft-clip).
	reg [7:0] fm_gain, oki_gain, oki2_gain;
	always @(posedge clk) begin
		fm_gain   <= gain_resolve(fm_vol_sel,   DEF_GAIN_FM);
		oki_gain  <= gain_resolve(oki_vol_sel,  DEF_GAIN_OKI);
		oki2_gain <= gain_resolve(oki2_vol_sel, DEF_GAIN_OKI);
	end

	// ─── Somma larga + soft-clip sui SOLI picchi (no crackling, niente tagli) ─
	// Somma su bus largo (stessi gain), poi soft-clip SOLO in cima. Sotto TH
	// (~-1 dBFS) tutto e' LINEARE = IDENTICO al mixer pulito: bassi, medi, corpi
	// e code delle esplosioni INTATTI. SOLO le punte che nel pulito clippavano
	// dure (= il crackling) vengono arrotondate dolci (parabola tangente in TH,
	// slope 0 al tetto = niente gradino) verso un tetto appena sotto il fondo
	// scala. Waveshaper STATICO -> niente pumping, dinamica intatta.
	// Due OKI (MAME li instrada entrambi a 0.40 contro 0.50 dell'FM): si sommano
	// a 17 bit PRIMA del guadagno, poi una sola moltiplicazione. Il bus va da 26
	// a 27 bit perche' ora le sorgenti sono tre.
	// I due OKI hanno ORA il loro guadagno master indipendente: in MAME sono
	// due device distinti instradati 0.40 ciascuno, e nell'OSD sono due voci.
	// Prima venivano sommati PRIMA del guadagno e ne condividevano uno solo.
	wire signed [15:0] oki_ext16  = {oki_sound,  2'b0};   // 14->16bit x4
	wire signed [15:0] oki2_ext16 = {oki2_sound, 2'b0};
	wire signed [24:0] fm_scaled   = ym_snd     * $signed({1'b0, fm_gain});
	wire signed [24:0] oki1_scaled = oki_ext16  * $signed({1'b0, oki_gain});
	wire signed [24:0] oki2_scaled = oki2_ext16 * $signed({1'b0, oki2_gain});
	wire signed [26:0] mix_sum    = $signed({{2{fm_scaled[24]}},   fm_scaled})
	                              + $signed({{2{oki1_scaled[24]}}, oki1_scaled})
	                              + $signed({{2{oki2_scaled[24]}}, oki2_scaled});
	wire signed [26:0] mix_ovr    = mix_sum >>> 4;                 // scala sample (gain Q4.4)

	// Limitatore: vedi raiden2_audio_limiter.sv. Sotto il ginocchio passa
	// identico al mixer pulito; sopra comprime senza mai diventare un muro.
	wire signed [15:0] mix_out;
	raiden2_audio_limiter #(.W(27), .TH(20480)) u_lim (
		.mix_in (mix_ovr),
		.mix_out(mix_out)
	);

	always @(posedge clk) begin
		if (reset) begin
			audio_l <= 16'sd0;
			audio_r <= 16'sd0;
		end else begin
			audio_l <= mix_out;
			audio_r <= mix_out;
		end
	end

endmodule
