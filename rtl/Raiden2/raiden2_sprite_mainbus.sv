// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// Raiden sprite RAM bus (pattern Irem M72 board_b_d-style + BUFFERED_SPRITERAM16)
// Gestisce spriteram (Main side, $07000-$07FFF, 4KB = 2K word) con:
// - Porta A: Main V30 R/W
// - Buffer: copia spriteram → buf su rising vblank (MAME BUFFERED_SPRITERAM16)
// - Porta B (buffer): renderer R-only
//
// MAME memory map Main:
//   $07000-$07FFF  spriteram (4KB) — share "spriteram" (BUFFERED_SPRITERAM16)
//
// Pattern M72: DOUT_VALID = MRD & sprite_memrq.

module raiden2_sprite_mainbus #(parameter SS_IDX = -1)
(
	input  wire        clk,
	input  wire        reset,

	// CPU Main interface
	input  wire [19:0] cpu_addr,
	input  wire        cpu_rd,
	input  wire        cpu_wr,
	input  wire  [1:0] cpu_be,
	input  wire [15:0] cpu_dout,

	// memrq da raiden2_addr_main
	input  wire        sprite_memrq,

	// CPU read mux output
	output wire [15:0] DOUT,
	output wire        DOUT_VALID,

	// Vblank rising → copia spriteram → buffer
	input  wire        vblank_rising,

	// Renderer porta B (read buffer)
	input  wire [10:0] spr_vram_addr,
	output wire [15:0] spr_vram_data,

	// Savestate slave. Mappa piatta: [11]=0 spr_lo/hi (bank CPU), 1 = BUFFER
	// visualizzato. Il buffer NON si ricostruisce da solo: il trigger della copia
	// e' una write della CPU a $68E (BUFFERED_SPRITERAM16), non il vblank
	// dell'hardware — senza salvarlo, dopo un restore restano a schermo gli
	// sprite della partita in corso finche' il gioco non ritriggera.
	ssbus_if.slave     ss_spr,
	// stato della FSM di copia (lo salva il blocco banchi del top).
	// SOLO copy_idx: cp_lo_q/cp_hi_q sono il registro di USCITA della BRAM, e
	// mettergli davanti un mux di ripristino li stacca dalla memoria — Quartus
	// allora replica l'array in registri per servire quella lettura: 32.768
	// flip-flop e un mux 2048:1, misurati in 16.445 ALM. Non vale un byte.
	// Se il salvataggio cade in mezzo alla copia, al ripristino una sola word
	// del buffer viene riscritta con il valore vecchio della pipeline: il
	// buffer lo ripristiniamo comunque per intero, e il primo trigger del gioco
	// rifa' la copia da capo.
	output wire [11:0] ss_copy_state,
	input  wire [11:0] ss_copy_state_in,
	input  wire        ss_copy_wr
);

// Word index (4KB / 2 byte = 2K word)
wire [10:0] spr_word_addr = cpu_addr[11:1];

// M72-native byte lanes: cpu_be[0]=lane bassa, cpu_be[1]=lane alta; cpu_dout già allineato.
wire cpu_we_lo = cpu_wr && cpu_be[0];
wire cpu_we_hi = cpu_wr && cpu_be[1];

// ─── Sprite RAM CPU bank ────────────────────────────────────────────────
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] spr_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] spr_hi [0:2047];
initial begin integer i; for (i=0; i<2048; i=i+1) begin spr_lo[i]=0; spr_hi[i]=0; end end

reg  [15:0] spr_cpu_rdata;      // q verso il ssbus (mux di regione)
reg  [15:0] spr_stage_q;        // lettura del bank CPU (= DOUT)
wire        spr_we_lo_cpu = sprite_memrq && cpu_we_lo;
wire        spr_we_hi_cpu = sprite_memrq && cpu_we_hi;
wire [15:0] spr_wdata_cpu = cpu_dout;
wire [11:0] ssp_addr;
wire        ssp_we_lo, ssp_we_hi;
wire [15:0] ssp_wdata;
wire        ssp_we  = ssp_we_lo | ssp_we_hi;
wire        ssp_sel = ss_spr.access(SS_IDX);
ss_ram16_adaptor #(.WIDTHAD(12), .SS_IDX(SS_IDX)) u_ss_spr (
	.clk(clk), .we_lo_in(1'b0), .we_hi_in(1'b0),
	.addr_in(12'd0), .wdata_in(16'd0),
	.we_lo_out(ssp_we_lo), .we_hi_out(ssp_we_hi), .addr_out(ssp_addr),
	.wdata_out(ssp_wdata), .q_in(spr_cpu_rdata), .ssbus(ss_spr)
);
wire [10:0] spr_idx      = ssp_sel ? ssp_addr[10:0] : spr_word_addr;
wire        spr_we_lo    = ssp_sel ? (ssp_we & ~ssp_addr[11]) : spr_we_lo_cpu;
wire        spr_we_hi    = ssp_sel ? (ssp_we & ~ssp_addr[11]) : spr_we_hi_cpu;
wire [15:0] spr_wdata_eff= ssp_sel ? ssp_wdata : spr_wdata_cpu;
always @(posedge clk) begin
	if (spr_we_lo) spr_lo[spr_idx] <= spr_wdata_eff[7:0];
	if (spr_we_hi) spr_hi[spr_idx] <= spr_wdata_eff[15:8];
	spr_stage_q <= {spr_hi[spr_idx], spr_lo[spr_idx]};
end

// Buffer UNICO (N-1), schema COPIATO dalla _old HW-perfetta
// (Raiden2_main_top.sv:876-919): il copy compensa la latenza SINCRONA della
// lettura M10K con la pipeline src=idx / dst=idx-1. Il vecchio codice qui
// copiava buf[k] <= spriteram[k] nello stesso ciclo: in silicio (indirizzo
// registrato) arriva spriteram[k-1] -> TUTTE le entry shiftate di una word
// -> code/color/X/Y mescolati = sprite sballati. Il doppio buffer N-2 era
// un esperimento dichiarato ("v110: test"): via, come la _old. 2026-08-13.
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] spr_lo_buf [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] spr_hi_buf [0:2047];
initial begin integer i; for (i=0; i<2048; i=i+1) begin spr_lo_buf[i]=0; spr_hi_buf[i]=0; end end

// Copy FSM: trigger = vblank_rising (nel top = sprbuf_trig, write CPU $68E,
// BUFFERED_SPRITERAM16 come MAME). idx 0..2048: a idx=k il dato letto al
// ciclo prima (mem[k-1]) viene scritto a dst=k-1.
reg [11:0] copy_idx = 12'd2100;    // idle: > 2048
wire       copying  = (copy_idx < 12'd2049);
reg  [7:0] cp_lo_q, cp_hi_q;
wire [10:0] cp_src = copy_idx[10:0];
wire [10:0] cp_dst = copy_idx[10:0] - 11'd1;
wire        cp_we  = copying && (copy_idx >= 12'd1);
wire        buf_we = ssp_sel ? (ssp_we & ssp_addr[11]) : cp_we;
wire [10:0] buf_a  = ssp_sel ? ssp_addr[10:0] : cp_dst;
wire [15:0] buf_d  = ssp_sel ? ssp_wdata : {cp_hi_q, cp_lo_q};
always @(posedge clk) begin
	cp_lo_q <= spr_lo[cp_src];
	cp_hi_q <= spr_hi[cp_src];
	if (buf_we) begin
		spr_lo_buf[buf_a] <= buf_d[7:0];
		spr_hi_buf[buf_a] <= buf_d[15:8];
	end
	if (reset)              copy_idx <= 12'd2100;
	else if (ss_copy_wr)    copy_idx <= ss_copy_state_in;
	else if (vblank_rising) copy_idx <= 12'd0;
	else if (copying)       copy_idx <= copy_idx + 12'd1;
end
assign ss_copy_state = copy_idx;

// Renderer porta B → legge il buffer (N-1, come la _old)
reg [15:0] spr_vram_rdata;
wire [10:0] buf_rd_a = ssp_sel ? ssp_addr[10:0] : spr_vram_addr;
always @(posedge clk) spr_vram_rdata <= {spr_hi_buf[buf_rd_a], spr_lo_buf[buf_rd_a]};
assign spr_vram_data = spr_vram_rdata;

// q verso l'adattatore: regione ritardata di un ciclo come le letture
reg ssp_reg_r;
always @(posedge clk) ssp_reg_r <= ssp_addr[11];
always @(*) spr_cpu_rdata = ssp_reg_r ? spr_vram_rdata : spr_stage_q;

// ─── DOUT_VALID + DOUT (pattern M72) ────────────────────────────────────
reg sprite_rd_lat;

always @(posedge clk) begin
	if (reset) begin
		sprite_rd_lat   <= 1'b0;
	end else begin
		sprite_rd_lat   <= cpu_rd & sprite_memrq;
	end
end

// Core M72 lane-aware: word naturale, il core seleziona il byte (niente swap).
assign DOUT_VALID = sprite_rd_lat;
assign DOUT       = spr_stage_q;

endmodule
