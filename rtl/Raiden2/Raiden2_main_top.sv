// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// Raiden2_main_top — Main V30 + memory map M72-style.
// Refactor pattern Irem M72:
//  - raiden2_addr_main: address translator (memrq + ls245_en + sdr_addr)
//  - raiden2_ce_gen:    CE generator (stall su mem_rq_active + ls245_en)
//  - raiden2_sdram_bridge_cpu: ls245_en→toggle SDRAM req, latch dout
//  - raiden2_sprite_mainbus: spriteram + buffer (BUFFERED_SPRITERAM16)
//  - Main RAM 28KB BRAM interno (pattern M72 per region writable interna)
//  - shared RAM 4KB: instanziata in TOP (modulo raiden2_shared_ram), porte esposte
//  - DOUT_VALID mux per cpu_din (pattern M72 m72.v:319-329)

module Raiden2_main_top #(parameter SS_IDX_SPR = -1, parameter SS_IDX_CPU = -1,
                          parameter SS_IDX_BANK = -1,
                          parameter SS_IDX_COPA = -1, parameter SS_IDX_COPR = -1) (
	input  wire        board_dx,   // 0 = Raiden II, 1 = Raiden DX (MRA index=1)
	input  wire        clk,
	input  wire        reset,
	input  wire        pause,
	output wire        cpu_idle,        // 1 = V30 a confine istruzione (savestate boundary)
	// OSD CPU speed select (legacy port, ignorato — sempre 10 MHz)
	input  wire  [2:0] clk_sel,
	// Inputs HW
	input  wire  [7:0] p1_input,
	input  wire  [7:0] p2_input,
	input  wire [15:0] dsw_input,
	input  wire [15:0] dsw2_input,     // $75C
	input  wire [15:0] system_input,   // $74C (START1/2 bit0/1, attivi bassi)
	// SDRAM ROM Main bridge (Raiden2.sv toplevel)
	input  wire [15:0] main_rom_rdata,
	input  wire        main_rom_ready,
	output wire [23:0] main_rom_addr,
	output wire        main_rom_req,
	// VBLANK per IRQ
	input  wire        vblank_in,
	// ioctl_download (legacy, non usato qui)
	input  wire        ioctl_download,
	// Layer control output
	output wire        ctrl_bg_en,
	output wire        ctrl_fg_en,
	output wire        ctrl_tx_en,
	output wire        ctrl_sp_en,
	output wire        ctrl_flipscreen,
	// Scroll registers $0F000-$0F03F flat 32 word
	output wire [511:0] scroll_words_flat,
	// Sound stub
	output wire        snd_cs,
	output wire  [4:1] snd_addr,
	output wire        snd_wr,
	output wire        snd_rd,
	output wire [15:0] snd_wdata,
	input  wire [15:0] snd_rdata,
	// VRAM read ports (text + sprite renderer)
	input  wire [10:0] text_vram_addr,
	output wire [15:0] text_vram_data,
	input  wire [10:0] spr_vram_addr,
	output wire [15:0] spr_vram_data,
	// VRAM tilemap BG/FG + palette (Raiden II: vivono sul MAIN, niente sub-CPU).
	// MG per ora e' solo storage CPU-side (renderer in fase video).
	input  wire [10:0] bg_vram_addr,
	output wire [15:0] bg_vram_data,
	input  wire [10:0] fg_vram_addr,
	output wire [15:0] fg_vram_data,
	input  wire [10:0] mg_vram_addr,
	output wire [15:0] mg_vram_data,
	input  wire [10:0] pal_vram_addr,
	output wire [15:0] pal_vram_data,
	ssbus_if.slave     ss_bg,        // SS_IDX 4 (ex sub)
	ssbus_if.slave     ss_fg,        // SS_IDX 5 (ex sub)
	ssbus_if.slave     ss_pal,       // SS_IDX 6 (ex sub)
	ssbus_if.slave     ss_crtc,      // SS_IDX 9 (ex sub-V30): registri CRTC
	// COP → ROM main via porta sub del bridge SDRAM (hitbox b100/b900):
	// la cache sub (rom_cache) sta nel top, protocollo pulse/ready compatibile.
	output wire [23:0] cop_rom_addr,
	output wire        cop_rom_req,
	input  wire [15:0] cop_rom_rdata,
	input  wire        cop_rom_ready,
	output wire        cop_busy_out,   // per ss_ready del top (COP fermo)
	// Seibu CRTC → scroll/enable/flip per i renderer (sostituiscono i
	// registri $E006/$F000 di Raiden 1, che su Raiden II non esistono)
	output wire        crtc_layer_en_bg,
	output wire        crtc_layer_en_mg,
	output wire        crtc_layer_en_fg,
	output wire        crtc_layer_en_text,
	output wire        crtc_layer_en_spr,
	output wire        crtc_flip_screen,
	output wire [15:0] crtc_scroll_bg_x,  output wire [15:0] crtc_scroll_bg_y,
	output wire [15:0] crtc_scroll_mg_x,  output wire [15:0] crtc_scroll_mg_y,
	output wire [15:0] crtc_scroll_fg_x,  output wire [15:0] crtc_scroll_fg_y,
	output wire [15:0] crtc_base_bg_x,    output wire [15:0] crtc_base_bg_y,
	output wire [15:0] crtc_base_mg_x,    output wire [15:0] crtc_base_mg_y,
	output wire [15:0] crtc_base_fg_x,    output wire [15:0] crtc_base_fg_y,
	output wire [15:0] crtc_base_txt_x,
	output wire [15:0] crtc_base_txt_y,
	// Banchi tile ($6CC + $470) impacchettati per i renderer:
	// [2:0]=BG, [6:4]=MID, [10:8]=FG (layout della _old maincpu_map)
	output wire [15:0] gfx_bank,
	// Shared RAM Main↔Sub bridge — Main side (porte verso top: raiden2_shared_ram)
	output wire [11:1] main_shared_addr,
	output wire        main_shared_cs,
	output wire  [1:0] main_shared_we,
	output wire [15:0] main_shared_wdata,
	input  wire [15:0] main_shared_rdata,
	// Probe
	output wire        dbg_irq_pending,
	// Savestate slaves (ssbus). Solo le BRAM che vivono in questo modulo.
	ssbus_if.slave     ss_workram,   // ram_lo/hi 14K (SS_IDX 0)
	ssbus_if.slave     ss_txt,       // txt_lo/hi 1K  (SS_IDX 1)
	ssbus_if.slave     ss_scroll,    // scroll_ram 32w (SS_IDX 2)
	ssbus_if.slave     ss_spr,       // spr_lo/hi 2K  (SS_IDX 7, → sprite_mainbus)
	ssbus_if.slave     ss_cpu,       // V30 main regs (SS_IDX 8, → cpu_v30_bridge)
	ssbus_if.slave     ss_bank,      // registri di BANCO (SS_IDX 3): vedi sotto
	ssbus_if.slave     ss_copa,      // COP: array (tabella microcodici, banchi DMA)
	ssbus_if.slave     ss_copr,      // COP: i 47 registri scalari
	input  wire        ss_cpu_reload // reset CPU coordinato (post-load)
);

// ─── V30 CPU bus ────────────────────────────────────────────────────────
wire [19:0] cpu_addr;
wire        cpu_rd, cpu_wr;
wire  [1:0] cpu_be;
wire [15:0] cpu_dout;
reg  [15:0] cpu_din;

// ─── IRQ handler (vblank rising → vector $0C8) ──────────────────────────
reg  vblank_d;
reg  irq_pending;
wire cpu_irq_active;
always @(posedge clk) begin
	if (reset) begin
		vblank_d    <= 1'b0;
		irq_pending <= 1'b0;
	end else begin
		vblank_d <= vblank_in;
		if (bank_wr) irq_pending <= bank_out[37];
		else begin
			if (vblank_in && !vblank_d)        irq_pending <= 1'b1;
			if (cpu_irq_active && irq_pending) irq_pending <= 1'b0;
		end
	end
end
assign dbg_irq_pending = irq_pending;

// ─── Address translator (M72 pal.sv pattern) ────────────────────────────
wire        ls245_en;
wire [23:0] sdr_addr;
wire        ram_memrq, cop_memrq, cbank_memrq, crtc_memrq, obj_memrq;
wire        sprbuf_memrq, bankw_memrq, tbank_memrq, sound_memrq;
wire        dsw_memrq, p1p2_memrq, p3p4_memrq, sys_memrq, dsw2_memrq;
wire        sprdst_memrq, sprite_memrq, bg_memrq, fg_memrq, mg_memrq;
wire        text_memrq, pal_memrq;
// Regioni Raiden 1 non esistenti su Raiden II: inerti (ctrl resta al reset =
// layer abilitati; scroll mai scritto; shared RAM sparisce col sub-CPU).
wire        shared_memrq = 1'b0;
wire        ctrl_memrq   = 1'b0;
wire        scroll_memrq = 1'b0;
wire        DBEN = cpu_rd | cpu_wr;

raiden2_addr_main u_addr (
	.A             (cpu_addr),
	.DBEN          (DBEN),
	.rom_bank      (rom_bank),
	.board_dx      (board_dx),
	.dx_prg_bank   (dx_bank_from_470 ? cop_bank[15:12] : {3'd0, rom_bank}),
	.ls245_en      (ls245_en),
	.sdr_addr      (sdr_addr),
	.ram_memrq     (ram_memrq),
	.cop_memrq     (cop_memrq),
	.cbank_memrq   (cbank_memrq),
	.crtc_memrq    (crtc_memrq),
	.obj_memrq     (obj_memrq),
	.sprbuf_memrq  (sprbuf_memrq),
	.bankw_memrq   (bankw_memrq),
	.tbank_memrq   (tbank_memrq),
	.sound_memrq   (sound_memrq),
	.dsw_memrq     (dsw_memrq),
	.p1p2_memrq    (p1p2_memrq),
	.p3p4_memrq    (p3p4_memrq),
	.sys_memrq     (sys_memrq),
	.dsw2_memrq    (dsw2_memrq),
	.sprdst_memrq  (sprdst_memrq),
	.sprite_memrq  (sprite_memrq),
	.bg_memrq      (bg_memrq),
	.fg_memrq      (fg_memrq),
	.mg_memrq      (mg_memrq),
	.text_memrq    (text_memrq),
	.pal_memrq     (pal_memrq)
);

// ─── Bank ROM $6CB + tile bank $6CC / $470 (trascritti dalla _old map) ──
// raiden2_bank_w: byte DISPARI di $6CA → dato nel byte ALTO della word.
// MACHINE_RESET parte da entry 1; il gioco scrive solo $0000/$8000 →
// rom_bank = ~wdata[15].
reg        rom_bank;
reg        dx_bank_from_470;   // DX: quale registro ha scritto per ultimo il banco
// ─── Savestate dei registri di BANCO ────────────────────────────────────
// Stanno FUORI dalle memorie e fuori dalla CPU, e su Raiden 1 non esistono
// (li' il main non banca): niente da copiare, e' stato proprio di questa
// scheda. rom_bank e cop_bank[15:12] scelgono la pagina da 64 KB che la CPU
// sta ESEGUENDO: senza salvarli, al ripristino la CPU riparte sulla pagina
// sbagliata ed esegue spazzatura — sembra la CPU rotta, ma e' la finestra
// sotto di lei ad essere cambiata.
// 38 bit: [0]=rom_bank [1]=dx_bank_from_470 [17:2]=cop_bank [25:18]=tbank_reg
//         [28:26]=gfx_bank_r[10:8] (banco fg: e' un latch, non si ricalcola)
//         [36:29]=ctrl_reg ($0E006: enable BG/FG/TX/SPR + flipscreen — senza
//                 di lui al ripristino restano accesi i layer della partita
//                 in corso, non quelli dello slot)
//         [37]=irq_pending (IRQ vblank in attesa al momento della cattura)
//         [49:38]=copy_idx della FSM che copia sprite RAM -> buffer
wire [49:0] bank_out;
wire        bank_wr;
wire [49:0] bank_in;
auto_save_adaptor #(.N_BITS(50), .SS_IDX(SS_IDX_BANK)) u_ss_bank (
	.clk(clk), .ssbus(ss_bank),
	.bits_in(bank_in), .bits_out(bank_out), .bits_wr(bank_wr)
);

reg [15:0] cop_bank;   // $470 cop_tile_bank_2: COMBINE_DATA + readback (RMW)
reg  [7:0] tbank_reg;  // $6CC tile_bank_01 (byte basso; consumer in fase video)

// COP cmd 0x7E05 (SOLO Raiden DX): write_byte($470, read_byte(cop_regs[4])).
// Lo strobe dura un ciclo ce_cop, cioe' piu' di un clk: si prende il FRONTE.
// board_dx e' il gate: su Raiden II il comando non esiste nemmeno nella
// tabella microcodici del ROM, quindi lo strobe non si alza mai — ma il gate
// resta esplicito perche' la semantica del byte basso di $470 (bit 5-4 = fg)
// e' DX-only: su R2 il banco fg viene dai bit 15-14 del byte alto.
wire       cop_bank_wr_w;
wire [7:0] cop_bank_byte_w;
reg        cop_bank_wr_d;
wire       cop_bank_wr_rise = board_dx & cop_bank_wr_w & ~cop_bank_wr_d;
always @(posedge clk) cop_bank_wr_d <= reset ? 1'b0 : cop_bank_wr_w;
always @(posedge clk) begin
	if (reset) begin
		rom_bank  <= 1'b1;
		dx_bank_from_470 <= 1'b1;   // MACHINE_RESET raidendx: entry 0 = cop_bank>>12 = 0
		cop_bank  <= 16'h0000;
		tbank_reg <= 8'h00;
	end else if (bank_wr) begin
		rom_bank         <= bank_out[0];
		dx_bank_from_470 <= bank_out[1];
		cop_bank         <= bank_out[17:2];
		tbank_reg        <= bank_out[25:18];
	end else begin
		if (bankw_memrq && cpu_wr && cpu_be[1]) rom_bank <= ~cpu_dout[15];
		// Raiden DX: il banco programma lo scrivono DUE registri e vale
		// l'ULTIMO (MAME: raiden2_bank_w da $6CB fa set_entry(BIT(~data,7)),
		// raidendx_cop_bank_2_w da $470 fa set_entry(cop_bank>>12); su DX
		// restano mappati entrambi). Prima usavamo SEMPRE $470: se il gioco
		// bancava da $6CB eseguivamo dal banco sbagliato.
		if (board_dx) begin
			if (cbank_memrq && cpu_wr && cpu_be[1]) dx_bank_from_470 <= 1'b1;
			else if (bankw_memrq && cpu_wr && cpu_be[1]) dx_bank_from_470 <= 1'b0;
		end
		if (cbank_memrq && cpu_wr) begin
			if (cpu_be[0]) cop_bank[7:0]  <= cpu_dout[7:0];
			if (cpu_be[1]) cop_bank[15:8] <= cpu_dout[15:8];
		end
		// cmd 0x7E05: come una write CPU del solo byte BASSO di $470. MAME
		// riapplica anche set_entry(cop_bank>>12), che una write a byte non
		// cambia: qui basta marcare che l'ultimo a scrivere il banco e' $470.
		if (cop_bank_wr_rise) begin
			cop_bank[7:0]    <= cop_bank_byte_w;
			dx_bank_from_470 <= 1'b1;
		end
		if (tbank_memrq && cpu_wr && cpu_be[0]) tbank_reg <= cpu_dout[7:0];
	end
end
// Packing banchi tile (raiden2_v.cpp, trascritto dalla _old maincpu_map):
//   $6CC byte basso: bg = (d&1)<<1 → 0/2 ; mid = 1|(d&2) → 1/3
//   $470 byte alto : fg = 4|(d>>14) → 4..7 (aggiornato solo con be[1],
//   qui ricomposto dal registro cop_bank già latchato)
// Reset MAME bank_reset(0,6,1,0): bg=0 fg=6 mid=1.
reg [10:0] gfx_bank_r;
always @(posedge clk) begin
	if (reset) gfx_bank_r <= {3'd6, 1'b0, 3'd1, 1'b0, 3'd0};   // fg=6 mid=1 bg=0
	else if (bank_wr) gfx_bank_r[10:8] <= bank_out[28:26];     // gli altri campi
	                                                           // si ricalcolano da tbank_reg
	else begin
		gfx_bank_r[2:0]  <= {1'b0, tbank_reg[0], 1'b0};        // bg  = (d&1)<<1
		gfx_bank_r[6:4]  <= {1'b0, tbank_reg[1], 1'b1};        // mid = 1 | (d&2)
		// fg SOLO su write $470 be[1] (semantica commit/MAME: bank_reset tiene
		// fg=6 finche' il gioco non scrive cbank; ricalcolarlo ogni clock lo
		// forzava a 4 subito dopo il reset).
		// Raiden II: fg = 4 | (d>>14), aggiornato solo con be[1].
		// Raiden DX : fg = 4 | ((cop_bank>>4)&3) -> bit 5-4, e MAME lo aggiorna
		// a OGNI write di $470 (raiden2_v.cpp raidendx_cop_bank_2_w).
		if (cbank_memrq && cpu_wr) begin
			if (board_dx)
				gfx_bank_r[10:8] <= {1'b1, cpu_be[0] ? cpu_dout[5:4] : cop_bank[5:4]};
			else if (cpu_be[1])
				gfx_bank_r[10:8] <= {1'b1, cpu_dout[15:14]};
		end
		// cmd 0x7E05 (solo DX): stessa formula della write CPU a byte basso,
		// fg = 4 | ((cop_bank>>4)&3), sul byte che arriva dal COP.
		else if (cop_bank_wr_rise)
			gfx_bank_r[10:8] <= {1'b1, cop_bank_byte_w[5:4]};
	end
end
assign bank_in = { spr_copy_state, irq_pending, ctrl_reg, gfx_bank_r[10:8], tbank_reg, cop_bank, dx_bank_from_470, rom_bank };
assign gfx_bank = {5'd0, gfx_bank_r};

// ─── Latch buffered spriteram: write CPU a $68E (raiden2.cpp:623) ───────
// NON a vblank come Raiden 1: e' il GIOCO a decidere quando fotografare la
// lista sprite.
wire [11:0] spr_copy_state;
reg sprbuf_wr_d;
always @(posedge clk) begin
	if (reset) sprbuf_wr_d <= 1'b0;
	else       sprbuf_wr_d <= sprbuf_memrq && cpu_wr;
end
wire sprbuf_trig = (sprbuf_memrq && cpu_wr) && !sprbuf_wr_d;

// ─── SDRAM bridge (mem_rq_active FSM M72) ───────────────────────────────
// Interfaccia con bridge top-level (sdram_bridge.sv esterno) tramite porte
// main_rom_*. Adapter qui converte: ls245_en/sdr_addr → main_rom_req/addr.
// Toggle protocol locale (sdram_rq/sdram_ack) NON usato — usiamo direttamente
// il pattern level del bridge top: req=1 mentre wait, ready=pulse 1-cycle.
//
// Stall CE gen: mem_rq_active locale = ls245_en in volo finché ready.
reg main_rq_active;
reg main_rd_lat;
reg [15:0] main_ram_rom_data;
reg [23:0] main_rom_addr_lat;   // pattern M72 m72.v:262-269: addr LATCHED nel FSM
always @(posedge clk) begin
	if (reset) begin
		main_rq_active    <= 1'b0;
		main_rd_lat       <= 1'b0;
		main_ram_rom_data <= 16'd0;
		main_rom_addr_lat <= 24'd0;
	end else begin
		main_rd_lat <= cpu_rd;
		if (!main_rq_active) begin
			if (ls245_en && cpu_rd && !main_rd_lat) begin
				// Rising edge cpu_rd in ROM region → start fetch
				main_rq_active    <= 1'b1;
				main_rom_addr_lat <= sdr_addr;      // latch addr — non passare-attraverso
			end
		end else if (main_rom_ready) begin
			main_ram_rom_data <= main_rom_rdata;
			main_rq_active    <= 1'b0;
		end
	end
end
// Adapter al bridge top: addr/req STABILI per tutta la durata della req
assign main_rom_addr = main_rom_addr_lat;
assign main_rom_req  = main_rq_active;

// ─── CE generator pattern M72 ───────────────────────────────────────────
wire ce, ce_4x;
// ── ROM SETTLE (2026-08-19) — porta la fix ba9cf5e di Raiden 1 ────────────
// LA CORSA DEL FETCH: il BIU campiona rdata_q, che nel bus vale cpu_din del
// clock PRECEDENTE. Alla fine di una lettura ROM ci sono DUE registri in fila:
//   main_rom_ready -> main_ram_rom_data (1 clk) -> cpu_din -> rdata_q (1 clk)
// Se lo stallo del CE cade insieme a main_rq_active, la CPU riparte mentre il
// dato sta ancora attraversando quella catena e consuma la parola VECCHIA.
// Con i wait-state (READY) il margine c'era; togliendoli — obbligatorio per
// l'ucore, che campiona il dato a T2 — il buco resta scoperto.
// MISURATO qui a variabile singola: riferimento OK (46 DMA 0x14, src 033F);
// + ce_half=ce+2 OK; + READY fisso ROMPE (1 DMA, src 0000); una coda di UN
// solo clock non basta. Servono DUE clock: uno per registro.
reg [1:0] rom_settle;
always @(posedge clk) begin
	if (reset)                                 rom_settle <= 2'd0;
	else if (main_rq_active && main_rom_ready) rom_settle <= 2'd2;
	else if (rom_settle != 2'd0)               rom_settle <= rom_settle - 2'd1;
end
// `rom_pending`: copre la finestra fra il FRONTE della lettura ROM e il clock
// in cui `main_rq_active` sale (la richiesta viene presa in carico un clock
// dopo). Senza, quel clock resta scoperto e la CPU avanza mentre il fetch non
// e' ancora partito.
// ⚠ Si spegne su `main_rq_active`, NON su `cpu_rd`: e' la differenza che
// evita l'autoalimentazione della formula di Raiden 1 (stallo alto -> la CPU
// non avanza -> cpu_rd resta basso -> stallo alto per sempre, misurato: CPU
// ferma a 133M). Qui lo stallo e' uno STATO con inizio e fine definiti.
reg rom_pending;
always @(posedge clk) begin
	if (reset)                                    rom_pending <= 1'b0;
	else if (ls245_en && cpu_rd && !main_rd_lat)  rom_pending <= 1'b1;
	else if (main_rq_active)                      rom_pending <= 1'b0;
end
// Catena completa: partenza -> in volo -> assestamento del dato (2 registri).
// ⭐ INCLUDE LO STALLO DEL COP (2026-08-19) — questo era il buco vero.
// `rom_wait = main_rq_active | cop_cpu_stall_w` (riga ~323) alimenta READY del
// core. Con i wait-state fermava la CPU anche quando il COP si prende la porta
// della Main RAM (mr_addr_mux la cede a lui mentre e' busy). Con READY FISSO
// ALTO — obbligatorio per l'ucore — quel segnale viene IGNORATO: il COP prende
// la porta, la CPU NON e' piu' fermata da nessuno e legge dalla RAM mentre
// l'indirizzo sulla porta e' quello del COP => dati altrui => il gioco deraglia.
// Il CE-stall non compensava perche' il ce_gen riceveva SOLO la richiesta ROM:
// dello stallo del COP non sapeva nulla.
// (Il commento a riga ~364 "la CPU intanto e' ferma (cpu_stall dentro
//  rom_wait) -> nessuna finestra di write perse" vale SOLO con i wait-state.)
wire main_stall_eff = rom_pending | main_rq_active | (rom_settle != 2'd0)
                    | cop_cpu_stall_w;

raiden2_ce_gen u_ce (
	.clk           (clk),
	.reset         (reset),
	.pause         (pause),
	.clk_sel       (clk_sel),
	.ls245_en      (ls245_en),
	.cpu_rd        (cpu_rd),
	.rd_lat        (main_rd_lat),
	.mem_rq_active (main_stall_eff),
	.ce            (ce),
	.ce_4x         (ce_4x)
);

// ─── V30 CPU bridge (cpu.vhd M72) ───────────────────────────────────────
cpu_v30_bridge #(.SS_IDX(SS_IDX_CPU)) u_cpu (
	.clk           (clk),
	.ce            (ce),
	.ce_4x         (ce_4x),
	.reset         (reset),
	// rom_wait = stall SDRAM + stall COP (la CPU aspetta finche' il COP ha
	// committato, come la _old con bus_rq_active = map | cop_stall).
	.rom_wait      (main_rq_active | cop_cpu_stall_w),
	.bus_addr      (cpu_addr),
	.bus_read      (cpu_rd),
	.bus_write     (cpu_wr),
	.bus_be        (cpu_be),
	.bus_dout      (cpu_dout),
	.bus_din       (cpu_din),
	.irq_req       (irq_pending),
	// MAME raiden2.cpp:1126-1127: irq0_line_hold + vector_r 0xC0/4. Il bridge
	// vuole l'indirizzo byte nella IVT: 0xC0 (Raiden 1 usava 0xC8).
	.irq_vector    (10'h0C0),
	.cpu_idle      (cpu_idle),
	.cpu_halt      (),
	.cpu_irqrequest(cpu_irq_active),
	.cpu_prefix    (),
	.ss            (ss_cpu),
	.ss_cpu_reload (ss_cpu_reload)
);

// ─── Main RAM 128KB (64K word) — unificata come la _old (Raiden II ha RAM
// sparsa in $0-$3FF/$800-$BFFF/$F800-$FFFF/$10000-$1EFFF: una BRAM unica
// indicizzata da A[16:1], le finestre video/IO sono escluse dal memrq).
// Load .mem zeros per init garantita (M10K no_rw_check NON azzera).
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] ram_lo [0:65535];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] ram_hi [0:65535];
initial begin
	$readmemh("main_ram_zeros_64k.mem", ram_lo);
	$readmemh("main_ram_zeros_64k.mem", ram_hi);
end

wire [15:0] ram_word_addr = cpu_addr[16:1];
// M72-native byte lanes: cpu_be[0]=lane bassa (A0==0), cpu_be[1]=lane alta.
// cpu_dout è già sulla lane giusta (niente shuffle/replica). Usati da ram/txt/scroll.
wire cpu_we_lo = cpu_wr && cpu_be[0];
wire cpu_we_hi = cpu_wr && cpu_be[1];
// write-enable work RAM gated su ram_memrq, per l'adaptor savestate
wire ram_we_lo_cpu = ram_memrq && cpu_we_lo;
wire ram_we_hi_cpu = ram_memrq && cpu_we_hi;
wire [15:0] ram_wdata_cpu = cpu_dout;
// Porta Main RAM condivisa CPU/COP (pattern _old main_top): quando il COP
// e' busy la porta e' sua (write ai suoi indirizzi, read da dma_src_byte);
// la CPU intanto e' ferma (cpu_stall dentro rom_wait) → nessuna finestra di
// write perse per costruzione.
wire [15:0] mr_addr_mux  = cop_dma_busy_w ? (cop_dma_ram_we ? cop_dma_ram_addr
                                                            : cop_dma_src_byte[16:1])
                                          : ram_word_addr;
wire        mr_we_lo_mux = cop_dma_busy_w ? (cop_dma_ram_we & cop_dma_ram_be[0])
                                          : ram_we_lo_cpu;
wire        mr_we_hi_mux = cop_dma_busy_w ? (cop_dma_ram_we & cop_dma_ram_be[1])
                                          : ram_we_hi_cpu;
wire [15:0] mr_wdata_mux = cop_dma_busy_w ? cop_dma_ram_wdata : ram_wdata_cpu;

// Savestate adaptor in serie sulla porta CPU (ZERO BRAM): SS idle → segnali gioco;
// durante SS → porta dirottata al ssbus (SS_IDX_WORKRAM).
reg [15:0] ram_rdata;
wire [15:0] ram_idx;
wire        ram_we_lo, ram_we_hi;
wire [15:0] ram_wdata_eff;
ss_ram16_adaptor #(.WIDTHAD(16), .SS_IDX(0)) u_ss_workram (
	.clk      (clk),
	.we_lo_in (mr_we_lo_mux),
	.we_hi_in (mr_we_hi_mux),
	.addr_in  (mr_addr_mux),
	.wdata_in (mr_wdata_mux),
	.we_lo_out(ram_we_lo),
	.we_hi_out(ram_we_hi),
	.addr_out (ram_idx),
	.wdata_out(ram_wdata_eff),
	.q_in     (ram_rdata),
	.ssbus    (ss_workram)
);
always @(posedge clk) if (ram_we_lo) ram_lo[ram_idx] <= ram_wdata_eff[7:0];
always @(posedge clk) if (ram_we_hi) ram_hi[ram_idx] <= ram_wdata_eff[15:8];
always @(posedge clk) ram_rdata <= {ram_hi[ram_idx], ram_lo[ram_idx]};

// ─── Sprite RAM Main bus (BUFFERED_SPRITERAM16) ─────────────────────────
wire [15:0] spr_DOUT;
wire        spr_DOUT_VALID;
wire vblank_rising_main = vblank_in & ~vblank_d;
// Fill COP 0x118 verso la spriteram ($0C000, word $06000-$067FF): iniettato
// come write "CPU" nel mainbus — la CPU vera e' ferma durante il busy COP.
wire spr_fill_w = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h06000)
                                  & (cop_dma_fill_addr < 23'h06800);
// ── Write della sprcpt verso la SPRITERAM (2026-08-13) ────────────────────
// Il gioco costruisce UNA sola display list in $0C000-$0CFFF (spriteram),
// alternando CPU e COP: carica il puntatore in spr_dst1 ($6C6, ROM $0AB179),
// lascia che la sprcpt aggiunga le sue entry, poi rilegge da $762 dove il COP
// e' arrivato e riprende da li' (ROM $0AB2EC-$0AB2F0). La sprcpt pero' scrive
// via dma_ram_we, che finisce nella MAIN RAM: le sue entry cadevano in
// un'ombra che nessuno legge, mentre il puntatore avanzava lo stesso ->
// buchi nella lista, entry vecchie che restano, ordine sprite sbagliato.
// (Nel core _old $762 non era nemmeno mappato: il puntatore non avanzava e la
// CPU riscriveva sopra, quindi il difetto restava invisibile.)
// Qui le write COP che cadono nella finestra spriteram (word $6000-$67FF =
// byte $C000-$CFFF) vengono instradate al modulo spriteram, esattamente come
// gia' si fa per il fill 0x118. Non c'e' conflitto con la lettura sorgente del
// COP: gli stati SP_WR_H1/H2/X/Y non emettono dma_src_byte.
wire spr_cop_w = cop_dma_busy_w & cop_dma_ram_we
               & (cop_dma_ram_addr >= 16'h6000) & (cop_dma_ram_addr < 16'h6800);
// Hijack porta A durante il busy COP (pattern _old u_spr_ram_cpu): l'indirizzo
// presenta il src del DMA → DOUT = M[src] con 1 ciclo di latenza (stessa
// semantica delle stage_b). rd/wr/be azzerati: niente DOUT_VALID spurio e le
// write CPU stallate si completano al rilascio (non perse, come la fill).
// Priorita': fill > write sprcpt > lettura sorgente COP > CPU.
raiden2_sprite_mainbus #(.SS_IDX(SS_IDX_SPR)) u_spr_bus (
	.clk           (clk),
	.reset         (reset),
	.cpu_addr      (spr_fill_w      ? {8'h0C, cop_dma_fill_addr[10:0], 1'b0} :
	                spr_cop_w       ? {8'h0C, cop_dma_ram_addr[10:0], 1'b0}  :
	                cop_dma_busy_w  ? {8'h0C, cop_dma_src_byte[11:1], 1'b0}  : cpu_addr),
	.cpu_rd        (spr_fill_w ? 1'b0 : (cop_dma_busy_w ? 1'b0 : cpu_rd)),
	.cpu_wr        (spr_fill_w ? 1'b1 : (spr_cop_w ? 1'b1 : (cop_dma_busy_w ? 1'b0 : cpu_wr))),
	.cpu_be        (spr_fill_w ? 2'b11 : (spr_cop_w ? cop_dma_ram_be : (cop_dma_busy_w ? 2'b00 : cpu_be))),
	.cpu_dout      (spr_fill_w ? cop_dma_fill_wdata : (spr_cop_w ? cop_dma_ram_wdata : cpu_dout)),
	.sprite_memrq  (spr_fill_w ? 1'b1 : (spr_cop_w ? 1'b1 : (cop_dma_busy_w ? 1'b0 : sprite_memrq))),
	.DOUT          (spr_DOUT),
	.DOUT_VALID    (spr_DOUT_VALID),
	// Raiden II: la copia nel buffer parte dalla write CPU a $68E, non dal
	// vblank (BUFFERED_SPRITERAM16 comandata dal gioco, raiden2.cpp:623).
	.vblank_rising (sprbuf_trig),
	.ss_copy_state    (spr_copy_state),
	.ss_copy_state_in (bank_out[49:38]),
	.ss_copy_wr       (bank_wr),
	.spr_vram_addr (spr_vram_addr),
	.spr_vram_data (spr_vram_data),
	.ss_spr        (ss_spr)
);

// ─── Text RAM 2KB (1K word) — Main scrive, renderer legge ──────────────
// v114 textram double buffer (pattern sprite_mainbus BUFFERED_SPRITERAM16):
// CPU bank txt_lo/txt_hi <- CPU writes
// Buffer txt_lo_buf/txt_hi_buf <- copy parallel da CPU bank su vblank_rising
// Renderer legge dal buffer = snapshot stabile frame, no race mid-frame.
// Equivale a MAME tilemap render fine-frame.
// Raiden II: TEXT 4KB (2K word) a $E800-$F7FF, regione NON allineata a 4K:
// $E800 & $FFF = $800 → offset word = cpu_addr[11:1] - $400 (wrap a 11 bit,
// stessa formula della _old).
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] txt_lo [0:2048-1];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] txt_hi [0:2048-1];
initial begin integer i; for (i=0; i<2048; i=i+1) begin txt_lo[i]=0; txt_hi[i]=0; end end

(* ramstyle = "M10K,no_rw_check" *) reg [7:0] txt_lo_buf [0:2048-1];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] txt_hi_buf [0:2048-1];
initial begin integer i; for (i=0; i<2048; i=i+1) begin txt_lo_buf[i]=0; txt_hi_buf[i]=0; end end

wire [10:0] txt_word_addr = cpu_addr[11:1] - 11'h400;
wire txt_we_lo_cpu = text_memrq && cpu_we_lo;
wire txt_we_hi_cpu = text_memrq && cpu_we_hi;
wire [7:0] txt_din_lo = cpu_dout[7:0];
wire [7:0] txt_din_hi = cpu_dout[15:8];

// Savestate SS_IDX 1 — mappa piatta a 4 regioni da 2K word:
//   [12:11]=0 staging txt   1 renderer txt (quello che si VEDE)
//           2 staging mg    3 renderer mg
// Il renderer e' indispensabile: e' scritto solo dal DMA 0x14 del gioco, quindi
// al ripristino resterebbe il frame della partita in corso.
// L'adattatore e' una porta pura; l'indirizzo di ogni memoria resta il suo e
// viene dirottato SOLO mentre il ssbus accede (il gioco e' in pausa).
reg  [15:0] txt_ss_rdata;
wire [12:0] ssv_addr;
wire        ssv_we_lo, ssv_we_hi;
wire [15:0] ssv_wdata;
wire        ssv_we  = ssv_we_lo | ssv_we_hi;
wire        ssv_sel = ss_txt.access(1);
wire  [1:0] ssv_reg = ssv_addr[12:11];
ss_ram16_adaptor #(.WIDTHAD(13), .SS_IDX(1)) u_ss_txt (
	.clk      (clk),
	.we_lo_in (1'b0),
	.we_hi_in (1'b0),
	.addr_in  (13'd0),
	.wdata_in (16'd0),
	.we_lo_out(ssv_we_lo),
	.we_hi_out(ssv_we_hi),
	.addr_out (ssv_addr),
	.wdata_out(ssv_wdata),
	.q_in     (txt_ss_rdata),
	.ssbus    (ss_txt)
);
wire [10:0] txt_idx = ssv_sel ? ssv_addr[10:0]
                    : (txt_fill ? (cop_dma_fill_addr[10:0] - 11'h400) : txt_word_addr);
wire txt_we_lo = ssv_sel ? (ssv_we & (ssv_reg == 2'd0)) : (txt_fill | txt_we_lo_cpu);
wire txt_we_hi = ssv_sel ? (ssv_we & (ssv_reg == 2'd0)) : (txt_fill | txt_we_hi_cpu);
wire [15:0] txt_wdata_eff = ssv_sel ? ssv_wdata
                          : (txt_fill ? cop_dma_fill_wdata : {txt_din_hi, txt_din_lo});
reg [15:0] txt_stage_q;
always @(posedge clk) txt_stage_q <= {txt_hi[txt_idx], txt_lo[txt_idx]};

// CPU bank: 1 always per array M10K (m10k_pattern_separate)
always @(posedge clk) begin
	if (txt_we_lo) txt_lo[txt_idx] <= txt_wdata_eff[7:0];
end
always @(posedge clk) begin
	if (txt_we_hi) txt_hi[txt_idx] <= txt_wdata_eff[15:8];
end

// Porta B dello staging: sorgente del DMA 0x14 del COP.
always @(posedge clk) txt_stage_b <= {txt_hi[cop_dma_src_byte[11:1] - 11'h400],
                                      txt_lo[cop_dma_src_byte[11:1] - 11'h400]};

// I buffer sono il RENDERER: scritti SOLO dal DMA 0x14 (offset 0xC00-0x13FF)
// — la "copia" la comanda il gioco, non il vblank (pattern _old u_txt_rend).
wire [10:0] txt_rend_waddr = cop_vram_off[10:0] - 11'h400;
wire        txt_rend_we = ssv_sel ? (ssv_we & (ssv_reg == 2'd1)) : cop_to_txt;
wire [10:0] txt_rend_a  = ssv_sel ? ssv_addr[10:0] : txt_rend_waddr;
wire [15:0] txt_rend_d  = ssv_sel ? ssv_wdata : cop_src_mux;
always @(posedge clk) if (txt_rend_we) txt_lo_buf[txt_rend_a] <= txt_rend_d[7:0];
always @(posedge clk) if (txt_rend_we) txt_hi_buf[txt_rend_a] <= txt_rend_d[15:8];

// Renderer read dal rend (in savestate la porta di lettura serve al salvataggio)
reg [7:0] txt_lo_rd, txt_hi_rd;
wire [10:0] txt_rd_a = ssv_sel ? ssv_addr[10:0] : text_vram_addr[10:0];
always @(posedge clk) txt_lo_rd <= txt_lo_buf[txt_rd_a];
always @(posedge clk) txt_hi_rd <= txt_hi_buf[txt_rd_a];

assign text_vram_data = {txt_hi_rd, txt_lo_rd};

// ─── BG/FG/MG VRAM + Palette (Raiden II, lato CPU) ─────────────────────
// BG $D000-$D7FF, FG $D800-$DFFF, MG $E000-$E7FF: 1K word ciascuna (array a
// 2K per compatibilità con le porte renderer [10:0]). PAL $1F000-$1FFFF: 2K
// word. Stesso pattern del text: CPU bank + adaptor savestate; il renderer
// legge live (i renderer definitivi arrivano con la fase video/CRTC).
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] bg_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] bg_hi [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] fg_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] fg_hi [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mg_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mg_hi [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] pal_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] pal_hi [0:2047];
initial begin integer i; for (i=0; i<2048; i=i+1) begin
	bg_lo[i]=0; bg_hi[i]=0; fg_lo[i]=0; fg_hi[i]=0;
	mg_lo[i]=0; mg_hi[i]=0; pal_lo[i]=0; pal_hi[i]=0;
end end

wire [10:0] bgfgmg_word_addr = {1'b0, cpu_addr[10:1]};
wire [10:0] pal_word_addr    = cpu_addr[11:1];

// ── SPLIT STAGING/RENDER (semantica MAME 2-stadi, pattern _old) ────────
// STAGING (array qui sotto) = host RAM: porta A = CPU (+fill COP), porta B
// = sorgente del DMA. RENDERER (array _rend) = scritti SOLO dal DMA
// 0x14/0x15, letti dal video: coppia contenuto/scroll atomica come MAME.

// BG — STAGING SS_IDX 4
// SS_IDX 4 — [11]=0 staging, 1 renderer
reg  [15:0] bg_ss_rdata;
wire [11:0] ssbg_addr;
wire        ssbg_we_lo, ssbg_we_hi;
wire [15:0] ssbg_wdata;
wire        ssbg_we  = ssbg_we_lo | ssbg_we_hi;
wire        ssbg_sel = ss_bg.access(4);
ss_ram16_adaptor #(.WIDTHAD(12), .SS_IDX(4)) u_ss_bg (
	.clk(clk),
	.we_lo_in(1'b0), .we_hi_in(1'b0), .addr_in(12'd0), .wdata_in(16'd0),
	.we_lo_out(ssbg_we_lo), .we_hi_out(ssbg_we_hi), .addr_out(ssbg_addr),
	.wdata_out(ssbg_wdata), .q_in(bg_ss_rdata), .ssbus(ss_bg)
);
wire [10:0] bg_idx = ssbg_sel ? ssbg_addr[10:0]
                   : (bg_fill ? {1'b0, cop_dma_fill_addr[9:0]} : bgfgmg_word_addr);
wire bg_we_lo = ssbg_sel ? (ssbg_we & ~ssbg_addr[11]) : (bg_fill | (bg_memrq && cpu_we_lo));
wire bg_we_hi = ssbg_sel ? (ssbg_we & ~ssbg_addr[11]) : (bg_fill | (bg_memrq && cpu_we_hi));
wire [15:0] bg_wdata_eff = ssbg_sel ? ssbg_wdata : (bg_fill ? cop_dma_fill_wdata : cpu_dout);
reg [15:0] bg_stage_q;
always @(posedge clk) if (bg_we_lo) bg_lo[bg_idx] <= bg_wdata_eff[7:0];
always @(posedge clk) if (bg_we_hi) bg_hi[bg_idx] <= bg_wdata_eff[15:8];
always @(posedge clk) bg_stage_q <= {bg_hi[bg_idx], bg_lo[bg_idx]};
always @(posedge clk) bg_stage_b <= {bg_hi[{1'b0, cop_dma_src_byte[10:1]}],
                                     bg_lo[{1'b0, cop_dma_src_byte[10:1]}]};
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] bg_rend_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] bg_rend_hi [0:2047];
initial begin integer i; for (i=0; i<2048; i=i+1) begin bg_rend_lo[i]=0; bg_rend_hi[i]=0; end end
wire        bg_rend_we = ssbg_sel ? (ssbg_we & ssbg_addr[11]) : cop_to_bg;
wire [10:0] bg_rend_a  = ssbg_sel ? ssbg_addr[10:0] : cop_vram_off[10:0];
wire [15:0] bg_rend_d  = ssbg_sel ? ssbg_wdata : cop_src_mux;
always @(posedge clk) if (bg_rend_we) bg_rend_lo[bg_rend_a] <= bg_rend_d[7:0];
always @(posedge clk) if (bg_rend_we) bg_rend_hi[bg_rend_a] <= bg_rend_d[15:8];
reg [15:0] bg_rd_q;
wire [10:0] bg_rd_a = ssbg_sel ? ssbg_addr[10:0] : bg_vram_addr;
always @(posedge clk) bg_rd_q <= {bg_rend_hi[bg_rd_a], bg_rend_lo[bg_rd_a]};
assign bg_vram_data = bg_rd_q;
reg ssbg_reg_r;
always @(posedge clk) ssbg_reg_r <= ssbg_addr[11];
always @(*) bg_ss_rdata = ssbg_reg_r ? bg_rd_q : bg_stage_q;

// FG — STAGING SS_IDX 5
// SS_IDX 5 — [11]=0 staging, 1 renderer
reg  [15:0] fg_ss_rdata;
wire [11:0] ssfg_addr;
wire        ssfg_we_lo, ssfg_we_hi;
wire [15:0] ssfg_wdata;
wire        ssfg_we  = ssfg_we_lo | ssfg_we_hi;
wire        ssfg_sel = ss_fg.access(5);
ss_ram16_adaptor #(.WIDTHAD(12), .SS_IDX(5)) u_ss_fg (
	.clk(clk),
	.we_lo_in(1'b0), .we_hi_in(1'b0), .addr_in(12'd0), .wdata_in(16'd0),
	.we_lo_out(ssfg_we_lo), .we_hi_out(ssfg_we_hi), .addr_out(ssfg_addr),
	.wdata_out(ssfg_wdata), .q_in(fg_ss_rdata), .ssbus(ss_fg)
);
wire [10:0] fg_idx = ssfg_sel ? ssfg_addr[10:0]
                   : (fg_fill ? {1'b0, cop_dma_fill_addr[9:0]} : bgfgmg_word_addr);
wire fg_we_lo = ssfg_sel ? (ssfg_we & ~ssfg_addr[11]) : (fg_fill | (fg_memrq && cpu_we_lo));
wire fg_we_hi = ssfg_sel ? (ssfg_we & ~ssfg_addr[11]) : (fg_fill | (fg_memrq && cpu_we_hi));
wire [15:0] fg_wdata_eff = ssfg_sel ? ssfg_wdata : (fg_fill ? cop_dma_fill_wdata : cpu_dout);
reg [15:0] fg_stage_q;
always @(posedge clk) if (fg_we_lo) fg_lo[fg_idx] <= fg_wdata_eff[7:0];
always @(posedge clk) if (fg_we_hi) fg_hi[fg_idx] <= fg_wdata_eff[15:8];
always @(posedge clk) fg_stage_q <= {fg_hi[fg_idx], fg_lo[fg_idx]};
always @(posedge clk) fg_stage_b <= {fg_hi[{1'b0, cop_dma_src_byte[10:1]}],
                                     fg_lo[{1'b0, cop_dma_src_byte[10:1]}]};
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] fg_rend_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] fg_rend_hi [0:2047];
initial begin integer i; for (i=0; i<2048; i=i+1) begin fg_rend_lo[i]=0; fg_rend_hi[i]=0; end end
wire [10:0] fg_rend_waddr = cop_vram_off[10:0] - 11'h400;
wire        fg_rend_we = ssfg_sel ? (ssfg_we & ssfg_addr[11]) : cop_to_fg;
wire [10:0] fg_rend_a  = ssfg_sel ? ssfg_addr[10:0] : fg_rend_waddr;
wire [15:0] fg_rend_d  = ssfg_sel ? ssfg_wdata : cop_src_mux;
always @(posedge clk) if (fg_rend_we) fg_rend_lo[fg_rend_a] <= fg_rend_d[7:0];
always @(posedge clk) if (fg_rend_we) fg_rend_hi[fg_rend_a] <= fg_rend_d[15:8];
reg [15:0] fg_rd_q;
wire [10:0] fg_rd_a = ssfg_sel ? ssfg_addr[10:0] : fg_vram_addr;
always @(posedge clk) fg_rd_q <= {fg_rend_hi[fg_rd_a], fg_rend_lo[fg_rd_a]};
assign fg_vram_data = fg_rd_q;
reg ssfg_reg_r;
always @(posedge clk) ssfg_reg_r <= ssfg_addr[11];
always @(*) fg_ss_rdata = ssfg_reg_r ? fg_rd_q : fg_stage_q;

// MG — STAGING senza SS (TODO idx quando si rinumera la mappa)
reg [15:0] mg_rdata_cpu;
wire        mg_st_we_lo = ssv_sel ? (ssv_we & (ssv_reg == 2'd2)) : (mg_fill | (mg_memrq && cpu_we_lo));
wire        mg_st_we_hi = ssv_sel ? (ssv_we & (ssv_reg == 2'd2)) : (mg_fill | (mg_memrq && cpu_we_hi));
wire [10:0] mg_st_addr  = ssv_sel ? ssv_addr[10:0]
                        : (mg_fill ? {1'b0, cop_dma_fill_addr[9:0]} : bgfgmg_word_addr);
wire [15:0] mg_st_wdata = ssv_sel ? ssv_wdata : (mg_fill ? cop_dma_fill_wdata : cpu_dout);
always @(posedge clk) if (mg_st_we_lo) mg_lo[mg_st_addr] <= mg_st_wdata[7:0];
always @(posedge clk) if (mg_st_we_hi) mg_hi[mg_st_addr] <= mg_st_wdata[15:8];
always @(posedge clk) mg_rdata_cpu <= {mg_hi[mg_st_addr], mg_lo[mg_st_addr]};
always @(posedge clk) mg_stage_b <= {mg_hi[{1'b0, cop_dma_src_byte[10:1]}],
                                     mg_lo[{1'b0, cop_dma_src_byte[10:1]}]};
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mg_rend_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mg_rend_hi [0:2047];
initial begin integer i; for (i=0; i<2048; i=i+1) begin mg_rend_lo[i]=0; mg_rend_hi[i]=0; end end
wire        mg_rend_we = ssv_sel ? (ssv_we & (ssv_reg == 2'd3)) : cop_to_mg;
wire [10:0] mg_rend_a  = ssv_sel ? ssv_addr[10:0] : cop_vram_off[10:0];
wire [15:0] mg_rend_d  = ssv_sel ? ssv_wdata : cop_src_mux;
always @(posedge clk) if (mg_rend_we) mg_rend_lo[mg_rend_a] <= mg_rend_d[7:0];
always @(posedge clk) if (mg_rend_we) mg_rend_hi[mg_rend_a] <= mg_rend_d[15:8];
reg [15:0] mg_rd_q;
wire [10:0] mg_rd_a = ssv_sel ? ssv_addr[10:0] : mg_vram_addr;
always @(posedge clk) mg_rd_q <= {mg_rend_hi[mg_rd_a], mg_rend_lo[mg_rd_a]};
assign mg_vram_data = mg_rd_q;

// dato di ritorno dell'adattatore SS_IDX 1 (le q sono gia' registrate: qui
// resta solo la scelta della regione, sull'indirizzo ritardato di un ciclo)
reg [1:0] ssv_reg_r;
always @(posedge clk) ssv_reg_r <= ssv_reg;
always @(*) begin
	case (ssv_reg_r)
		2'd0: txt_ss_rdata = txt_stage_q;
		2'd1: txt_ss_rdata = {txt_hi_rd, txt_lo_rd};
		2'd2: txt_ss_rdata = mg_rdata_cpu;
		2'd3: txt_ss_rdata = mg_rd_q;
	endcase
end

// PAL — STAGING SS_IDX 6; renderer scritto SOLO dal DMA 0x15 (niente mirror
// CPU→renderer: il flash pre-fade era il bug documentato della _old)
// SS_IDX 6 — [11]=0 staging, 1 palette VISIBILE (renderer)
reg  [15:0] pal_ss_rdata;
wire [11:0] sspal_addr;
wire        sspal_we_lo, sspal_we_hi;
wire [15:0] sspal_wdata;
wire        sspal_we  = sspal_we_lo | sspal_we_hi;
wire        sspal_sel = ss_pal.access(6);
ss_ram16_adaptor #(.WIDTHAD(12), .SS_IDX(6)) u_ss_pal (
	.clk(clk),
	.we_lo_in(1'b0), .we_hi_in(1'b0), .addr_in(12'd0), .wdata_in(16'd0),
	.we_lo_out(sspal_we_lo), .we_hi_out(sspal_we_hi), .addr_out(sspal_addr),
	.wdata_out(sspal_wdata), .q_in(pal_ss_rdata), .ssbus(ss_pal)
);
wire [10:0] pal_idx = sspal_sel ? sspal_addr[10:0]
                    : (pal_fill ? cop_dma_fill_addr[10:0] : pal_word_addr);
wire pal_we_lo = sspal_sel ? (sspal_we & ~sspal_addr[11]) : (pal_fill | (pal_memrq && cpu_we_lo));
wire pal_we_hi = sspal_sel ? (sspal_we & ~sspal_addr[11]) : (pal_fill | (pal_memrq && cpu_we_hi));
wire [15:0] pal_wdata_eff = sspal_sel ? sspal_wdata : (pal_fill ? cop_dma_fill_wdata : cpu_dout);
reg [15:0] pal_stage_q;
always @(posedge clk) if (pal_we_lo) pal_lo[pal_idx] <= pal_wdata_eff[7:0];
always @(posedge clk) if (pal_we_hi) pal_hi[pal_idx] <= pal_wdata_eff[15:8];
always @(posedge clk) pal_stage_q <= {pal_hi[pal_idx], pal_lo[pal_idx]};
always @(posedge clk) pal_stage_b <= {pal_hi[cop_dma_src_byte[11:1]],
                                      pal_lo[cop_dma_src_byte[11:1]]};
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] pal_rend_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] pal_rend_hi [0:2047];
initial begin integer i; for (i=0; i<2048; i=i+1) begin pal_rend_lo[i]=0; pal_rend_hi[i]=0; end end
wire pal_rend_we = cop_dma_busy_w & cop_dma_pal_we;
// Semantica COMMIT (u_pal unico, mux busy): la write CPU raggiunge la palette
// VISIBILE direttamente (oltre alla staging per i src dei fade); il DMA 0x15
// ha priorita' (durante busy la CPU e' comunque stallata). Il render della
// palette NON dipende piu' dal solo DMA — come nella configurazione che ha
// bootato.
wire        pal_rd_we_lo = sspal_sel ? (sspal_we & sspal_addr[11])
                          : (pal_rend_we | (pal_memrq && cpu_we_lo));
wire        pal_rd_we_hi = sspal_sel ? (sspal_we & sspal_addr[11])
                          : (pal_rend_we | (pal_memrq && cpu_we_hi));
wire [10:0] pal_rend_a   = sspal_sel ? sspal_addr[10:0]
                          : (pal_rend_we ? cop_dma_pal_addr : pal_word_addr);
wire [15:0] pal_rend_d   = sspal_sel ? sspal_wdata
                          : (pal_rend_we ? cop_dma_pal_wdata : cpu_dout);
always @(posedge clk) if (pal_rd_we_lo) pal_rend_lo[pal_rend_a] <= pal_rend_d[7:0];
always @(posedge clk) if (pal_rd_we_hi) pal_rend_hi[pal_rend_a] <= pal_rend_d[15:8];
reg [15:0] pal_rd_q;
wire [10:0] pal_rd_a = sspal_sel ? sspal_addr[10:0] : pal_vram_addr;
always @(posedge clk) pal_rd_q <= {pal_rend_hi[pal_rd_a], pal_rend_lo[pal_rd_a]};
assign pal_vram_data = pal_rd_q;
reg sspal_reg_r;
always @(posedge clk) sspal_reg_r <= sspal_addr[11];
always @(*) pal_ss_rdata = sspal_reg_r ? pal_rd_q : pal_stage_q;

// ─── Seibu CRTC ($600-$63F) — trascritto dal game_top della _old ───────
wire [15:0] crtc_rdata;
wire        crtc_dyn_size;
Raiden2_seibu_crtc #(.SS_IDX(9)) u_crtc (
	.clk(clk), .reset(reset),
	.cs(crtc_memrq),
	.wr(crtc_memrq && cpu_wr),
	.rd(crtc_memrq && cpu_rd),
	.addr(cpu_addr[6:1]),
	.alt_regs(board_dx),
	.dsn(~cpu_be),
	.wdata(cpu_dout),
	.rdata(crtc_rdata),
	.layer_en_bg(crtc_layer_en_bg), .layer_en_mg(crtc_layer_en_mg),
	.layer_en_fg(crtc_layer_en_fg), .layer_en_text(crtc_layer_en_text),
	.layer_en_spr(crtc_layer_en_spr),
	.scroll_bg_x(crtc_scroll_bg_x), .scroll_bg_y(crtc_scroll_bg_y),
	.scroll_mg_x(crtc_scroll_mg_x), .scroll_mg_y(crtc_scroll_mg_y),
	.scroll_fg_x(crtc_scroll_fg_x), .scroll_fg_y(crtc_scroll_fg_y),
	.scroll_base_bg_x(crtc_base_bg_x),   .scroll_base_bg_y(crtc_base_bg_y),
	.scroll_base_mg_x(crtc_base_mg_x),   .scroll_base_mg_y(crtc_base_mg_y),
	.scroll_base_fg_x(crtc_base_fg_x),   .scroll_base_fg_y(crtc_base_fg_y),
	.scroll_base_text_x(crtc_base_txt_x),
	.scroll_base_text_y(crtc_base_txt_y),
	.flip_screen(crtc_flip_screen), .dyn_layer_size(crtc_dyn_size),
	.ss_crtc(ss_crtc)
);

// ─── COP3 SEI300 ($400-$6FF + $762) — versione COMMIT della _old ───────
// FASE 1 (questa): lato CPU completo — registri, macro/math, trigger,
// readback, stall. Le porte DMA verso memoria/video sono INERTI (rdata=0,
// rom_ready=1 così le FSM hitbox/c480 completano senza appendersi); il
// cablaggio mr/src/vram/pal/fill/spr arriva nella fase 2 con lo split
// staging/render (pattern _old). cpu_stall entra in rom_wait: invariante
// ciclo-per-ciclo garantita a monte (la CPU e' ferma quando il COP e' busy).
wire        cop_cs_w    = cop_memrq | sprdst_memrq;
wire [15:0] cop_rdata_w;
wire        cop_dma_busy_w;
wire        cop_cpu_stall_w;
// Porte DMA verso la Main RAM (mux più sotto, pattern _old)
wire [15:0] cop_dma_ram_addr;
wire [15:0] cop_dma_ram_wdata;
wire  [1:0] cop_dma_ram_be;
wire        cop_dma_ram_we;
wire [23:0] cop_dma_src_byte;
assign cop_busy_out = cop_dma_busy_w;
Raiden2_cop3 #(.SS_IDX_ARR(SS_IDX_COPA), .SS_IDX_REG(SS_IDX_COPR)) u_cop3 (
	.ss_arr(ss_copa), .ss_reg(ss_copr),
	.clk(clk), .reset(reset),
	.cs(cop_cs_w),
	.wr(cop_memrq && cpu_wr),
	.rd(cop_cs_w && cpu_rd),
	.addr(cpu_addr[10:1] - 10'h200),   // word offset da $400 ($762 → $1B1)
	.dsn(~cpu_be),
	.wdata(cpu_dout),
	.rdata(cop_rdata_w),
	.dma_ram_addr(cop_dma_ram_addr), .dma_ram_wdata(cop_dma_ram_wdata),
	.dma_ram_be(cop_dma_ram_be),
	.dma_ram_rdata(ram_rdata),
	.dma_ram_we(cop_dma_ram_we),
	.dma_src_byte(cop_dma_src_byte),
	.dma_src_rdata(cop_src_mux_q), // mux per regione, REGISTRATO: latenza 2 stati (vedi nota)
	.cop_rom_addr(cop_rom_addr), .cop_rom_req(cop_rom_req),
	.cop_rom_rdata(cop_rom_rdata),
	.cop_rom_ready(cop_rom_ready),
	.dma_spr_addr(), .dma_spr_wdata(), .dma_spr_we(),   // mai asserite nel commit
	.dma_vram_addr(), .dma_vram_wdata(), .dma_vram_we(),  // versione registrata: non usata (si usa _now)
	.dma_vram_addr_now(cop_dma_vram_addr_now),
	.dma_vram_we_now(cop_dma_vram_we_now),
	.dma_pal_addr(cop_dma_pal_addr), .dma_pal_wdata(cop_dma_pal_wdata),
	.dma_pal_we(cop_dma_pal_we),
	.dma_pal_stage_we(),                                 // mai asserita nel commit
	.cop_bank_wr(cop_bank_wr_w), .cop_bank_byte(cop_bank_byte_w),
	.dma_fill_we(cop_dma_fill_we), .dma_fill_addr(cop_dma_fill_addr),
	.dma_fill_wdata(cop_dma_fill_wdata),
	.dma_busy(cop_dma_busy_w),
	.cpu_stall(cop_cpu_stall_w)
);

// ─── DMA video del COP (pattern _old game_top) ─────────────────────────
// 0x14 tilemap: staging → rend, offset 0x000 BG / 0x400 FG / 0x800 MG /
// 0xC00-0x13FF TXT; dato = cop_src_rdata (valido in D_WRITE, semantica _now).
wire [12:0] cop_dma_vram_addr_now;
wire        cop_dma_vram_we_now;
wire [10:0] cop_dma_pal_addr;
wire [15:0] cop_dma_pal_wdata;
wire        cop_dma_pal_we;
wire        cop_dma_fill_we;
wire [22:0] cop_dma_fill_addr;
wire [15:0] cop_dma_fill_wdata;
wire [12:0] cop_vram_off = cop_dma_vram_addr_now;
wire cop_to_bg  = cop_dma_busy_w & cop_dma_vram_we_now & (cop_vram_off < 13'h0400);
wire cop_to_fg  = cop_dma_busy_w & cop_dma_vram_we_now & (cop_vram_off >= 13'h0400) & (cop_vram_off < 13'h0800);
wire cop_to_mg  = cop_dma_busy_w & cop_dma_vram_we_now & (cop_vram_off >= 13'h0800) & (cop_vram_off < 13'h0C00);
wire cop_to_txt = cop_dma_busy_w & cop_dma_vram_we_now & (cop_vram_off >= 13'h0C00) & (cop_vram_off < 13'h1400);

// Fill 0x116/0x118 per regione (finestre WORD della mappa raiden2_mem:
// $0C000 spr, $0D000 bg, $0D800 fg, $0E000 mg, $0E800 txt, $1F000 pal)
wire bg_fill  = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h06800) & (cop_dma_fill_addr < 23'h06C00);
wire fg_fill  = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h06C00) & (cop_dma_fill_addr < 23'h07000);
wire mg_fill  = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h07000) & (cop_dma_fill_addr < 23'h07400);
wire txt_fill = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h07400) & (cop_dma_fill_addr < 23'h07C00);
wire pal_fill = cop_dma_fill_we & (cop_dma_fill_addr >= 23'h0F800) & (cop_dma_fill_addr < 23'h10000);
// spr_fill ($06000-$067FF): TODO porta write COP su sprite_mainbus

// Sorgente DMA per regione (src mux, _old main_top:544-568): porta B delle
// STAGING; default = Main RAM.
wire [12:0] cop_src_top13 = cop_dma_src_byte[23:11];
reg  [15:0] bg_stage_b, fg_stage_b, mg_stage_b, txt_stage_b, pal_stage_b;
reg  [15:0] cop_src_mux;
always @(*) begin
	case (cop_src_top13)
		13'h01A:          cop_src_mux = bg_stage_b;    // $0D000-$0D7FF
		13'h01B:          cop_src_mux = fg_stage_b;    // $0D800-$0DFFF
		13'h01C:          cop_src_mux = mg_stage_b;    // $0E000-$0E7FF
		13'h01D, 13'h01E: cop_src_mux = txt_stage_b;   // $0E800-$0F7FF
		13'h03E, 13'h03F: cop_src_mux = pal_stage_b;   // $1F000-$1FFFF
		13'h018, 13'h019: cop_src_mux = spr_DOUT;      // $0C000-$0CFFF (porta A hijack, pattern _old)
		default:          cop_src_mux = ram_rdata;     // resto = Main RAM
	endcase
end

// ── Registro di allineamento per la FSM del COP (2026-08-13) ──────────────
// Il cop3 e' scritto per una latenza di lettura di 2 STATI: emette
// dma_src_byte nello stato N e consuma dma_src_rdata nello stato N+2 (vedi
// dispatch 0205: emette +0x04, primo consumo due stati dopo).
// Le BRAM qui hanno latenza 1 CLOCK. Nel gold la FSM gira a clock pieno =>
// 1 clock = 1 stato => il codice vede 2 stati e funziona.
// Qui la FSM avanza su ce_cop (/2): 1 clock = MEZZO stato => la latenza
// vista scende a 1 STATO e OGNI catena di letture back-to-back del COP
// latcha gli operandi SHIFTATI di una posizione (0205 integratore di
// posizione, 0905 gravita', 138E/338E atan, 3BB0, 2A05, mul/div, sprcpt:
// sp_x/sp_y/head1/head2 scambiati = sprite sballati; il 0205 in retroazione
// diverge e finisce per scrivere strutture-oggetto sopra la IVT).
// Un registro sul DATO riporta la latenza a 2 stati esatti = semantica gold,
// senza toccare il cop3 (identico al gold) e senza rimuovere ce_cop (che
// serve al timing). Le catene gia' corrette (loop DMA 0x14/0x15, che ha uno
// stato senza emissione) restano corrette: il dato resta valido piu' a lungo.
reg [15:0] cop_src_mux_q;
always @(posedge clk) cop_src_mux_q <= cop_src_mux;

// ─── Scroll RAM ($0F000-$0F03F: 32 word) ───────────────────────────────
reg [15:0] scroll_ram [0:31];
initial begin integer i; for (i=0; i<32; i=i+1) scroll_ram[i] = 16'd0; end
wire [4:0] scroll_word_addr = cpu_addr[5:1];
// wdata scroll: nel caso word entrambi i byte, altrimenti byte replicato su lo
wire        scroll_wren_cpu = scroll_memrq && (cpu_we_lo || cpu_we_hi);
wire [15:0] scroll_wdata_cpu = cpu_dout;
// byte-enable per write parziale: durante SS scriviamo word intera (ssbus).
wire [1:0]  scroll_be_cpu = {cpu_be[1], cpu_be[0]};

reg  [15:0] scroll_ss_rdata;
wire        scroll_wren;
wire [4:0]  scroll_idx;
wire [15:0] scroll_wdata_eff;
ss_ram_adaptor #(.WIDTH(16), .WIDTHAD(5), .SS_IDX(2)) u_ss_scroll (
	.clk      (clk),
	.wren_in  (scroll_wren_cpu),
	.addr_in  (scroll_word_addr),
	.wdata_in (scroll_wdata_cpu),
	.wren_out (scroll_wren),
	.addr_out (scroll_idx),
	.wdata_out(scroll_wdata_eff),
	.q_in     (scroll_ss_rdata),
	.ssbus    (ss_scroll)
);
wire        scroll_ss_sel = ss_scroll.access(2);
wire [1:0]  scroll_be_eff = scroll_ss_sel ? 2'b11 : scroll_be_cpu;
always @(posedge clk) begin
	if (scroll_wren) begin
		if (scroll_be_eff[0]) scroll_ram[scroll_idx][7:0]  <= scroll_wdata_eff[7:0];
		if (scroll_be_eff[1]) scroll_ram[scroll_idx][15:8] <= scroll_wdata_eff[15:8];
	end
	scroll_ss_rdata <= scroll_ram[scroll_idx];
end
genvar gi;
generate
	for (gi = 0; gi < 32; gi = gi + 1) begin : g_scroll_export
		assign scroll_words_flat[gi*16 +: 16] = scroll_ram[gi];
	end
endgenerate

// ─── control_w $0E006 (8-bit) ──────────────────────────────────────────
// MAME bit 0: BG dis, bit 1: FG dis, bit 2: TX dis, bit 3: SPR dis
//      bit 6: flipscreen
reg [7:0] ctrl_reg;
always @(posedge clk) begin
	if (reset) ctrl_reg <= 8'h0F;
	else if (bank_wr) ctrl_reg <= bank_out[36:29];
	else if (ctrl_memrq && cpu_wr && cpu_be[0]) ctrl_reg <= cpu_dout[7:0];
end
assign ctrl_bg_en      = ~ctrl_reg[0];
assign ctrl_fg_en      = ~ctrl_reg[1];
assign ctrl_tx_en      = ~ctrl_reg[2];
assign ctrl_sp_en      = ~ctrl_reg[3];
assign ctrl_flipscreen =  ctrl_reg[6];

// ─── Sound seibu ($00700-$0071F, 8-bit umask 00FF) ─────────────────────
// MAME main_r/main_w(offset>>1): registro seibu = (byte-$700)/4 =
// cpu_addr[4:2] (fix coin della _old: con [4:1] ogni registro tranne 0 era
// misaddressato e la coin non veniva mai letta).
assign snd_cs     = sound_memrq;
assign snd_addr   = {1'b0, cpu_addr[4:2]};
assign snd_wr     = sound_memrq && cpu_wr && cpu_be[0];
assign snd_rd     = sound_memrq && cpu_rd && cpu_be[0];
assign snd_wdata  = cpu_dout;

// ─── Shared RAM bridge (Main side, modulo fisico in TOP) ──────────────
assign main_shared_addr  = cpu_addr[11:1];
assign main_shared_cs    = shared_memrq;
assign main_shared_we    = (shared_memrq && cpu_wr) ? {cpu_be[1], cpu_be[0]} : 2'b00;
assign main_shared_wdata = cpu_dout;

// ─── DOUT_VALID mux for cpu_din (pattern M72 m72.v:319-329) ────────────
// Latch 1-cycle dei memrq per allinearsi con BRAM 1-cycle latency.
reg ram_rd_lat, text_rd_lat, p1p2_rd_lat, dsw_rd_lat, sound_rd_lat;
reg shared_rd_lat;
reg p3p4_rd_lat, sys_rd_lat, dsw2_rd_lat, cbank_rd_lat, iohole_rd_lat;
reg bg_rd_lat, fg_rd_lat, mg_rd_lat, pal_rd_lat, crtc_rd_lat, cop_rd_lat;
reg [15:0] p1p2_data_lat, dsw_data_lat, sound_data_lat;
reg [15:0] p3p4_data_lat, sys_data_lat, dsw2_data_lat;

always @(posedge clk) begin
	if (reset) begin
		ram_rd_lat       <= 1'b0;
		text_rd_lat      <= 1'b0;
		p1p2_rd_lat      <= 1'b0;
		dsw_rd_lat       <= 1'b0;
		sound_rd_lat     <= 1'b0;
		shared_rd_lat    <= 1'b0;
		p3p4_rd_lat      <= 1'b0;
		sys_rd_lat       <= 1'b0;
		dsw2_rd_lat      <= 1'b0;
		cbank_rd_lat     <= 1'b0;
		iohole_rd_lat    <= 1'b0;
		bg_rd_lat        <= 1'b0;
		fg_rd_lat        <= 1'b0;
		mg_rd_lat        <= 1'b0;
		pal_rd_lat       <= 1'b0;
		crtc_rd_lat      <= 1'b0;
		cop_rd_lat       <= 1'b0;
		p1p2_data_lat    <= 16'd0;
		dsw_data_lat     <= 16'd0;
		sound_data_lat   <= 16'h00FF;
		p3p4_data_lat    <= 16'hFFFF;
		sys_data_lat     <= 16'hFFFF;
		dsw2_data_lat    <= 16'hFFFF;
	end else begin
		ram_rd_lat      <= cpu_rd & ram_memrq;
		text_rd_lat     <= cpu_rd & text_memrq;
		p1p2_rd_lat     <= cpu_rd & p1p2_memrq;
		dsw_rd_lat      <= cpu_rd & dsw_memrq;
		sound_rd_lat    <= cpu_rd & sound_memrq;
		shared_rd_lat   <= cpu_rd & shared_memrq;
		p3p4_rd_lat     <= cpu_rd & p3p4_memrq;
		sys_rd_lat      <= cpu_rd & sys_memrq;
		dsw2_rd_lat     <= cpu_rd & dsw2_memrq;
		cbank_rd_lat    <= cpu_rd & cbank_memrq;
		bg_rd_lat       <= cpu_rd & bg_memrq;
		fg_rd_lat       <= cpu_rd & fg_memrq;
		mg_rd_lat       <= cpu_rd & mg_memrq;
		pal_rd_lat      <= cpu_rd & pal_memrq;
		crtc_rd_lat     <= cpu_rd & crtc_memrq;
		cop_rd_lat      <= cpu_rd & (cop_memrq | sprdst_memrq);
		// Regioni senza modulo (obj/tbank/bankw read): open bus.
		iohole_rd_lat   <= cpu_rd & (obj_memrq | tbank_memrq | bankw_memrq);
		// IO data latch
		p1p2_data_lat   <= {p2_input, p1_input};
		dsw_data_lat    <= dsw_input;
		sound_data_lat  <= {8'hFF, snd_rdata[7:0]};
		// P3P4: Raiden II non ha P3/P4 — riposo FFFF.
		p3p4_data_lat   <= 16'hFFFF;
		sys_data_lat    <= system_input;
		// $75C: il commit NON lo mappava (default unmapped = FFFF) e la MRA
		// manda una sola word DIP ($740). Rispondere dip_sw2=0000 (attivo
		// basso = "tutto premuto") era una mia divergenza.
		dsw2_data_lat   <= 16'hFFFF;
	end
end

// Core M72 lane-aware: ritorna la word naturale, il core seleziona il byte
// (cpu_be/A0) internamente — niente byte_align.
// Pattern M72: priority mux su _valid_lat. Fallback a SDRAM (main_ram_rom_data).
always @(*) begin
	// ⚠ PRIORITA' AL FETCH ROM IN CORSO (2026-08-19)
	// Il dato ROM era il FALLBACK (ultimo else): durante l'attesa di una lettura
	// dalla ROM qualunque altro ramo attivo dirotta cpu_din, e `rdata_q` nel bus
	// (registrato a CLOCK PIENO, non su ce) cattura l'intruso. Con i wait-state
	// il core campionava nel momento giusto; con READY fisso — obbligatorio per
	// l'ucore, che campiona il dato a T2 — prende quello che passa.
	// MISURATO: cpu_din cambia 2.006.751 volte MENTRE lo stallo e' attivo.
	// Finche' il fetch ROM e' in volo (main_stall_eff) la sorgente e' UNA sola.
	// SOLO per gli accessi in regione ROM: `main_stall_eff` da solo dirotterebbe
	// anche le letture di RAM/periferiche sul dato ROM. `ls245_en` e'
	// combinatorio sull'indirizzo, che durante lo stallo e' stabile.
	if      (main_stall_eff && ls245_en) cpu_din = main_ram_rom_data;
	else if (spr_DOUT_VALID)  cpu_din = spr_DOUT;
	else if (ram_rd_lat)      cpu_din = ram_rdata;
	else if (shared_rd_lat)   cpu_din = main_shared_rdata;
	else if (text_rd_lat)     cpu_din = txt_ss_rdata;      // Raiden II: text leggibile
	else if (p1p2_rd_lat)     cpu_din = p1p2_data_lat;
	else if (dsw_rd_lat)      cpu_din = dsw_data_lat;
	else if (p3p4_rd_lat)     cpu_din = p3p4_data_lat;
	else if (sys_rd_lat)      cpu_din = sys_data_lat;
	else if (dsw2_rd_lat)     cpu_din = dsw2_data_lat;
	else if (cbank_rd_lat)    cpu_din = cop_bank;          // $470 RMW readback
	else if (bg_rd_lat)       cpu_din = bg_ss_rdata;
	else if (fg_rd_lat)       cpu_din = fg_ss_rdata;
	else if (mg_rd_lat)       cpu_din = mg_rdata_cpu;
	else if (pal_rd_lat)      cpu_din = pal_ss_rdata;
	else if (crtc_rd_lat)     cpu_din = crtc_rdata;
	else if (cop_rd_lat)      cpu_din = cop_rdata_w;
	else if (sound_rd_lat)    cpu_din = sound_data_lat;
	else if (iohole_rd_lat)   cpu_din = 16'hFFFF;          // open bus (moduli in arrivo)
	else                       cpu_din = main_ram_rom_data;   // fallback ROM (SDRAM)
end

endmodule
