// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
/*  raiden2_v30_ss.sv — bridge savestate per la CPU V30 (core WonderSwan/FPGAzumSpass).
    GPL-3.

    Il V30 (cpu.vhd) espone un bus savestate interno (SSBUS, pattern eReg_SS di
    FPGAzumSpass): 4 registri addressabili (Adr 0-3, 64 bit ognuno) che contengono
    lo stato architetturale:
      Adr0 = DX_CX_AX_IP   Adr1 = SI_BP_SP_BX
      Adr2 = SS_CS_ES_DI   Adr3 = F_DS (32 bit usati)
    - SAVE: metto SSBUS_Adr=N, leggo SSBUS_Dout = registro N.
    - LOAD: SSBUS_Adr=N, SSBUS_Din=valore, SSBUS_wren=1 → scrive il registro N.
      Poi cpu.vhd ricarica reg_ip/ax/... da SS_CPU1..4 SU RESET (riga 577).

    Questo modulo espone i 4 registri (256 bit, di cui 224 significativi) sul ssbus
    del savestate come uno slave. memory_stream li salva/carica come le RAM.

    TIMING DEL RESTORE (audit 2026-07-15): il reset della CPU NON deve scattare quando
    QUESTO slave finisce di caricare (a metà load, mentre gli slave successivi sono
    ancora in caricamento). Le CPU ripartirebbero a stato di sistema incoerente
    (freeze; la sub CPU pilota la palette → bianco e nero). Il reset è pilotato
    ESTERNAMENTE da `ss_cpu_reload` (dal raiden2_ss_manager), che lo alza SOLO quando
    l'INTERO load è finito (ss_busy 1→0) e a gioco fermo. Pattern BoogieWings
    (ss_m68k, stato dedicato SST_RESTORE_HOLD_RST post-load).

    Sul commit (ss.addr==N_WORDS): NIENTE — i 4 registri sono già in load_acc (scritti
    dalle word 0..15). L'applicazione avviene su ss_cpu_reload.
*/

`timescale 1ns / 1ps

module raiden2_v30_ss #(parameter SS_IDX = -1) (
	input  wire        clk,
	input  wire        reset,

	// reset CPU coordinato dal manager: alto per tutta la fase di reload post-load.
	// Mentre è alto: scrivo i 4 registri via SSBUS e tengo ss_reset_cpu alto.
	input  wire        ss_cpu_reload,

	// verso la CPU V30 (SSBUS interno)
	output reg  [63:0] ssbus_din,     // → SSBUS_Din
	output reg   [6:0] ssbus_adr,     // → SSBUS_Adr
	output reg         ssbus_wren,    // → SSBUS_wren
	input  wire [63:0] ssbus_dout,    // ← SSBUS_Dout (registro selezionato da adr)

	// impulso reset CPU (ricarica reg da SS_CPU su reset). Alto per tutta la reload.
	output reg         ss_reset_cpu,

	// slave del bus savestate del sistema
	ssbus_if.slave     ss
);

// 4 registri × 4 word(16b) = 16 word esposte sul ssbus (indirizzi 0..15).
localparam N_WORDS = 16;

reg [63:0] save_snap [0:3];   // snapshot per il save (letto da ssbus_dout)
reg [63:0] load_acc  [0:3];   // accumulatore per il load (riempito dalle word ssbus)

// Scan continuo per il SAVE: ciclo ssbus_adr 0..3 e latcho ssbus_dout (registro
// corrente) in save_snap. Sospeso durante la reload (priorità alla scrittura reg).
reg [1:0] scan_adr = 0;

// FSM di RELOAD: partita da ss_cpu_reload (esterno). Scrive le 4 word CPU1..4 su
// Adr 0..3 (una per ciclo), tiene ss_reset_cpu alto per TUTTA la durata di
// ss_cpu_reload → la CPU ricarica i reg da SS_CPU su reset (cpu.vhd:577) quando
// tutti i Dout_buffer sono stabili (ss_cpu_reload dura ampiamente più delle 4 write).
reg [2:0] wr_cnt;         // 0..3 indice word scritta; >=4 = write finite (solo reset)
reg       reload_d;       // edge detect di ss_cpu_reload

always @(posedge clk) begin
	ss.setup(SS_IDX, N_WORDS + 1, 1);   // +1 word "commit" (compat. layout auto_save_adaptor)
	ssbus_wren   <= 1'b0;
	ss_reset_cpu <= 1'b0;
	reload_d     <= ss_cpu_reload;

	if (ss_cpu_reload) begin
		// --- RELOAD: scrivi i 4 registri e tieni reset CPU alto ---
		ss_reset_cpu <= 1'b1;
		if (~reload_d) begin
			// rising edge: prima write (word 0) subito, contatore a 1
			ssbus_adr  <= 7'd0;
			ssbus_din  <= load_acc[0];
			ssbus_wren <= 1'b1;
			wr_cnt     <= 3'd1;
		end else if (wr_cnt < 3'd4) begin
			// write 1,2,3 nei cicli seguenti
			ssbus_adr  <= {5'd0, wr_cnt[1:0]};
			ssbus_din  <= load_acc[wr_cnt[1:0]];
			ssbus_wren <= 1'b1;
			wr_cnt     <= wr_cnt + 3'd1;
		end
		// dopo le 4 write (wr_cnt==4): ss_reset_cpu resta alto fino a fine
		// ss_cpu_reload → la CPU ricarica i reg stabili, riparte pulita al rilascio.
	end else begin
		// --- SAVE scan continuo ---
		// ssbus_dout (BUS_Dout eReg_SS) è COMBINATORIO su ssbus_adr (reg): riflette il
		// registro il cui Adr == ssbus_adr ATTUALE. Latcho save_snap[ssbus_adr], NON
		// scan_adr (il PROSSIMO), altrimenti salverei reg scan_adr-1 → shift di 1.
		save_snap[ssbus_adr[1:0]] <= ssbus_dout;
		ssbus_adr <= {5'd0, scan_adr};
		scan_adr  <= scan_adr + 2'd1;
	end

	// --- interfaccia col ssbus del sistema (save/load) ---
	// LOAD: le word 0..15 riempiono load_acc; il commit (addr==N_WORDS) è ignorato
	// (il reset è pilotato da ss_cpu_reload esterno, a load completo).
	if (ss.access(SS_IDX)) begin
		if (ss.write) begin
			if (ss.addr != N_WORDS)
				load_acc[ss.addr[3:2]][ ss.addr[1:0]*16 +: 16 ] <= ss.data[15:0];
			ss.write_ack(SS_IDX);
		end else if (ss.read) begin
			ss.read_response(SS_IDX, {48'd0, save_snap[ss.addr[3:2]][ ss.addr[1:0]*16 +: 16 ]});
		end
	end
end

endmodule
