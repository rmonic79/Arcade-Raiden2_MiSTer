// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
/*  raiden2_ss_manager.sv — coordinatore savestate frame-aligned.
    GPL-3.

    Problema #1 (risolto): collegare ss_save/ss_load direttamente a save_state_data
    fa partire il DMA A METÀ FRAME → cattura incoerente. Soluzione: pausa PRIMA
    (frame-aligned al vblank), POI trigger del DMA.

    Problema #2 (RESTORE rotto, audit 2026-07-15): il reset delle CPU V30 NON deve
    scattare quando lo slave CPU finisce di caricare (a METÀ load, mentre gli slave
    successivi — sub-CPU, z80_ram — sono ancora in caricamento). Le CPU ripartirebbero
    a stato di sistema INCOERENTE (freeze video; la sub CPU pilota la palette → b&w).
    Pattern BoogieWings (ss_m68k SST_RESTORE_HOLD_RST): reset CPU in fase DEDICATA,
    DOPO che l'intero load è finito (ss_busy 1→0), a gioco ANCORA fermo.
    → dopo il DMA di load, entro nello stato S_RELOAD: pulso ss_cpu_reload esteso a
      ENTRAMBE le CPU (che ricaricano i reg da SS_CPU su reset), poi sblocco la pausa.

    Sequenza:
      SAVE:  IDLE →(ss_save)→ WAIT(paused_safe) → TRIG(write_start) → RUN(busy) → IDLE
      LOAD:  IDLE →(ss_load)→ WAIT(paused_safe) → TRIG(read_start)  → RUN(busy)
                 → RELOAD(ss_cpu_reload N cicli, RAM tutte coerenti) → IDLE
      ss_pause resta alto per tutta la sequenza (fino a fine RELOAD nel load).
*/

`timescale 1ns / 1ps

module raiden2_ss_manager (
	input  wire clk,
	input  wire reset,

	input  wire ss_save,        // pulse da savestate_ui
	input  wire ss_load,        // pulse da savestate_ui
	input  wire paused_safe,    // pausa frame-aligned effettiva (dal top)
	input  wire ss_busy,        // save_state_data.busy (DMA in corso)
	input  wire slot_empty,     // 1 = il load ha trovato uno slot MAI SCRITTO

	output reg  ss_pause,       // richiesta pausa (va in OR nel paused_safe del top)
	output reg  write_start,    // → save_state_data.write_start (pulse)
	output reg  read_start,     // → save_state_data.read_start  (pulse)
	output reg  ss_cpu_reload   // reset CPU esteso post-load (→ tutte le CPU V30)
);

localparam S_IDLE = 3'd0, S_WAIT = 3'd1, S_TRIG = 3'd2, S_RUN = 3'd3, S_RELOAD = 3'd4;
reg [2:0] state;
reg       is_load;             // 1=load, 0=save

// durata del reset CPU post-load: abbastanza da coprire la scrittura dei 4 registri
// via SSBUS + margine Dout_buffer stabile. 8 cicli = ampio (le write sono 4).
localparam RELOAD_CYCLES = 5'd8;
reg [4:0] reload_cnt;

always @(posedge clk) begin
	if (reset) begin
		state         <= S_IDLE;
		ss_pause      <= 1'b0;
		write_start   <= 1'b0;
		read_start    <= 1'b0;
		ss_cpu_reload <= 1'b0;
		is_load       <= 1'b0;
		reload_cnt    <= 5'd0;
	end else begin
		write_start <= 1'b0;   // default: pulse di 1 ciclo
		read_start  <= 1'b0;

		case (state)
			S_IDLE: begin
				if (ss_save | ss_load) begin
					ss_pause <= 1'b1;          // richiedi pausa
					is_load  <= ss_load;
					state    <= S_WAIT;
				end
			end

			S_WAIT: begin
				// aspetta che la pausa sia EFFETTIVA (frame-aligned, gioco fermo)
				if (paused_safe) begin
					if (is_load) read_start  <= 1'b1;   // trigger memory_stream
					else         write_start <= 1'b1;
					state <= S_TRIG;
				end
			end

			S_TRIG: begin
				// aspetta che save_state_data alzi busy (DMA partito)
				if (ss_busy) state <= S_RUN;
			end

			S_RUN: begin
				// DMA in corso: mantieni la pausa finché busy è alto
				if (~ss_busy) begin
					if (is_load && !slot_empty) begin
						// LOAD finito: TUTTE le RAM sono coerenti. Ora (e SOLO ora)
						// resetto le CPU perché ricarichino i registri da SS_CPU.
						ss_cpu_reload <= 1'b1;
						reload_cnt    <= 5'd0;
						state         <= S_RELOAD;
					end else if (is_load) begin
						// SLOT VUOTO: memory_stream non ha scritto NIENTE nelle RAM.
						// Resettare le CPU qui significherebbe ripartire con lo stato
						// corrente a meta' — e' il caricamento a vuoto che rompeva il
						// gioco. Si sblocca e basta: la partita prosegue intatta.
						ss_pause <= 1'b0;
						state    <= S_IDLE;
					end else begin
						ss_pause <= 1'b0;   // SAVE finito → sblocca subito
						state    <= S_IDLE;
					end
				end
			end

			S_RELOAD: begin
				// tieni il reset CPU per RELOAD_CYCLES, a gioco ancora fermo (ss_pause alto),
				// poi rilascia. Le CPU ripartono da stato coerente con TUTTE le RAM caricate.
				if (reload_cnt == RELOAD_CYCLES - 1) begin
					ss_cpu_reload <= 1'b0;
					ss_pause      <= 1'b0;   // ora sblocca il gioco
					state         <= S_IDLE;
				end else begin
					reload_cnt <= reload_cnt + 5'd1;
				end
			end
		endcase
	end
end

endmodule
