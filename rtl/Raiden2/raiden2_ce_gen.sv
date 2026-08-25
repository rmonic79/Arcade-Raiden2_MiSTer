// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// Raiden II V30 CE generator — accumulatore frazionario, passo da OSD.
//
// clk_sys = 96 MHz. V30 della scheda = 16 MHz esatti (quarzo 32 MHz /2).
//
// clk_sel[2:0] (da OSD P2O[22:20]) — num/120 di 96 MHz, tutti ESATTI:
//   0 → 20/120 = 16,0 MHz  ← spec scheda (quarzo 32 MHz /2), default
//   1 → 10/120 =  8,0 MHz
//   2 → 12/120 =  9,6 MHz
//   3 → 15/120 = 12,0 MHz
//   4 → 24/120 = 19,2 MHz
//   5 → 30/120 = 24,0 MHz
//
// ce_4x = OGNI clk_sys (pattern R-Type: ce_4x è il clock interno della CPU,
// non un divisore). cpu.vhd richiede ce_4x ogni clk per il microcode/prefetch.
//
// Stall: durante accesso SDRAM in volo (mem_rq_active) il ciclo non parte, ma
// il riferimento nominale continua a maturare crediti (vedi piu' sotto).

module raiden2_ce_gen
(
	input  wire       clk,
	input  wire       reset,
	input  wire       pause,
	input  wire [2:0] clk_sel,
	input  wire       ls245_en,
	input  wire       cpu_rd,         // lettura CPU in corso (livello)
	input  wire       rd_lat,         // read-latch del FSM ROM (cpu_rd ritardato 1 clk)
	input  wire       mem_rq_active,
	// maxfan (2026-08-16): stesso motivo di ce_half in cpu_v30_bridge.sv — `ce`
	// abilita direttamente i registri di v30_eu/v30_biu (il ramo `else if (ce)`
	// di v30_eu.sv:2078) e alimenta ce_half. Replicandolo, ogni gruppo di
	// registri ha la sua copia locale invece di un unico driver che attraversa
	// il chip. Duplicare un registro non ne cambia il valore ne' la fase.
	(* maxfan = 24 *) output reg ce,
	output reg        ce_4x
);

// clk_sys = 96 MHz. Il V30 della scheda gira a 16,000000 MHz (quarzo 32 MHz
// diviso 2, "verified on pcb"), quindi il rapporto esatto e' 16/96 = 20/120.
// Accumulatore: somma il numeratore a ogni ciclo, scatta e sottrae il
// denominatore quando lo raggiunge. Nessuna deriva: la frequenza media vale
// esattamente num/den x clk_sys, anche per i passi che NON sono divisori
// interi di 96 (9,6 e 19,2 MHz).
// La CPU tollera il clock enable non uniforme; il pixel clock no, ed e' per
// questo che al video e' rimasto il divisore intero.
reg  [11:0] ce_acc;
reg  [11:0] ce_num;
localparam [12:0] CE_DEN = 13'd120;
// ⚠ La somma va SEMPRE su un bit in piu' dell'accumulatore: acc+num supera
// il denominatore per costruzione, e se il totale rientra nei bit
// dell'accumulatore il confronto >= CE_DEN non scatta piu' come deve, il
// clock enable si perde e la CPU SI FERMA (sintomo: schermo nero, niente
// musica, resta solo il suono del coin). Sbagliato cosi' il 2026-08-23,
// costo due build.
wire [12:0] ce_sum = {1'b0, ce_acc} + {1'b0, ce_num};

always @(*) begin
	case (clk_sel)
		3'd0: ce_num = 12'd20;    // 16,000 MHz spec scheda (32MHz/2)  <-- default
		3'd1: ce_num = 12'd10;    //  8,000 MHz
		3'd2: ce_num = 12'd12;    //  9,600 MHz
		3'd3: ce_num = 12'd15;    // 12,000 MHz
		3'd4: ce_num = 12'd24;    // 19,200 MHz
		3'd5: ce_num = 12'd30;    // 24,000 MHz
		default: ce_num = 12'd20;
	endcase
end

// ── CE-STALL + CATCH-UP A CREDITI (2026-08-18, porting UCORE) ─────────────
// PRIMA: stall = 1'b0 e la latenza SDRAM era gestita dal READY/Tw del core.
// Con l'UCORE quella strategia NON funziona, ed e' documentato: il suo BIU
// campiona il DATO a T2 (v30u_biu F57: "cur_data = ad_i at T2"); READY ritarda
// solo il COMPLETAMENTO del ciclo, non il campionamento. Con la SDRAM reale il
// dato arriva DOPO T2 -> in coda entrano byte 0000 -> istruzioni fantasma ->
// il boot deraglia. Misurato qui in sim: la CPU fa 2.4M accessi (e' viva) ma
// gira al ~30% della velocita' nominale e non accende MAI un IRQ.
// Il riferimento M72 dell'ucore fa cosi': "an SDRAM stall defers CPU cycles"
// (CE-stall, mai Tw reali). Raiden 1 gira cosi' su hardware.
// ⚠ ATTENZIONE ai segnali: `ls245_en` NON e' "accesso ROM in corso", e'
// puramente COMBINATORIO SULL'INDIRIZZO (raiden2_addr_main.sv:154/158: vale 1
// per tutta la regione ROM $20000-$FFFFF). Siccome il codice del gioco STA in
// ROM, e' alto quasi sempre: usare `ls245_en & ~rd_lat` come fa Raiden 1
// produce uno STALLO PERENNE. Misurato: stall 34% del tempo contro richieste
// SDRAM reali allo 0.4%, e ce emessi 1/32 del dovuto (CPU a ~0.5 MHz).
// La condizione giusta QUI e' quella con cui il nostro adapter fa partire la
// richiesta (Raiden2_main_top.sv:250): ls245_en && cpu_rd && !main_rd_lat.
// Quindi: stallo mentre la richiesta e' in volo (mem_rq_active) OPPURE
// nell'istante esatto in cui sta per partire (fronte di cpu_rd in regione ROM),
// che copre il clock prima che mem_rq_active salga.
// PROVA 2026-08-19: stallo per TUTTA la lettura ROM, non solo sul fronte.
// Con `& ~rd_lat` lo stallo durava un solo clock (l'istante del fronte di
// cpu_rd): se il dato SDRAM arriva dopo, la CPU e' gia' ripartita e
// campiona a T2 un valore non ancora valido. mem_rq_active copre la
// richiesta in volo, ma sale un clock DOPO il fronte: fra i due c'e' un
// buco. Tenendo `ls245_en & cpu_rd` il buco si chiude.
`ifndef CE_STALL_ON
// CONTORNO ORIGINALE (pre-ucore): nessun CE-stall, la latenza SDRAM era
// gestita dal READY/Tw del core. Serve per il confronto A/B.
wire stall    = 1'b0;
`else
// Lo stallo deve coprire il FRONTE della lettura ROM E il clock successivo:
// mem_rq_active (richiesta in volo) sale un clock DOPO il fronte, quindi fra
// i due ci sarebbe un clock scoperto in cui la CPU riparte mentre il dato
// non e' ancora arrivato. `stall_bridge` fa da ponte su quel clock.
// ⛔ NON usare `ls245_en & cpu_rd` senza `~rd_lat`: lo stallo diventa
//    permanente (la CPU non completa la lettura, cpu_rd non scende mai) e
//    il core si ferma al primo fetch. Misurato: 1 solo accesso bus.
// Lo stallo e' ORA un solo stato, calcolato in Raiden2_main_top.sv
// (`main_stall_eff` = rom_pending | main_rq_active | rom_settle) e passato
// su `mem_rq_active`. Copre partenza + volo + assestamento del dato.
// ⛔ NON aggiungere qui termini con `ls245_en`/`cpu_rd`: `ls245_en` e'
//    combinatorio sull'INDIRIZZO (alto per tutta la ROM) e un termine che
//    dipende da `cpu_rd` si autoalimenta -> CPU ferma per sempre (misurato).
wire stall    = mem_rq_active;
`endif

// Il CE-stall DIFFERISCE i cicli, non li perde: il riferimento nominale
// continua a maturare crediti anche durante lo stall, e appena il bus si
// libera il core li recupera a raffica (1 CE ogni CE_GAP_MIN clk). Senza
// questo, ogni miss SDRAM sarebbe velocita' persa per sempre.
// CE_GAP_MIN=5: e' il valore che tiene VERE le multicycle CE dichiarate in
// Template.sdc. Con ce_half = ce+2 (richiesto dall'ucore) l'arco
// ce_half -> ce successivo vale (GAP-2) = 3 periodi, non 5: percio' il
// multicycle addr_lat/lat_type -> v30_core va portato a 3 (era 4, calcolato
// sul ce periodico ogni 6). Dichiarare 4 qui sarebbe una maschera FALSA.
localparam [2:0] CE_GAP_MIN    = 3'd5;
// Tetto ai crediti: 8 = 2 cicli di bus. Uno stall reale dura quanto un accesso
// SDRAM; il tetto serve solo a impedire raffiche lunghe in contese patologiche.
localparam [5:0] CE_CREDIT_MAX = 6'd8;

reg [5:0] ce_credit;   // cicli differiti da recuperare (satura a CE_CREDIT_MAX)
reg [2:0] ce_gap;      // clk dall'ultimo CE emesso (satura a 7)

// earn : il riferimento nominale matura un ciclo (matura ANCHE sotto stall)
// spend: il ciclo puo' partire (bus libero + distanza minima rispettata).
//        Il credito maturato ORA e' spendibile nello stesso clk, quindi SENZA
//        stall il comportamento e' identico al divisore semplice di prima.
wire earn  = ~pause & (ce_sum >= CE_DEN);
wire spend = ~pause & ~stall & (ce_gap >= CE_GAP_MIN) & ((ce_credit != 6'd0) | earn);

always @(posedge clk) begin
	if (reset) begin
		ce_acc    <= 12'd0;
		ce        <= 1'b0;
		ce_4x     <= 1'b0;
		ce_credit <= 6'd0;
		ce_gap    <= CE_GAP_MIN;
	end else begin
		ce    <= 1'b0;
		ce_4x <= ~pause;          // ogni clk fuori pausa (invariato)

		// riferimento nominale: NON si ferma durante lo stall (cicli differiti)
		if (~pause) ce_acc <= (ce_sum >= CE_DEN) ? (ce_sum - CE_DEN) : ce_sum[11:0];

		// distanza dall'ultimo CE emesso
		if (ce_gap != 3'd7) ce_gap <= ce_gap + 3'd1;

		if (spend) begin
			ce     <= 1'b1;
			ce_gap <= 3'd0;
		end

		case ({earn, spend})
			2'b10: if (ce_credit != CE_CREDIT_MAX) ce_credit <= ce_credit + 6'd1;
			2'b01: if (ce_credit != 6'd0)          ce_credit <= ce_credit - 6'd1;
			default: ;   // 00 = fermo, 11 = maturato e speso subito
		endcase
	end
end

endmodule
