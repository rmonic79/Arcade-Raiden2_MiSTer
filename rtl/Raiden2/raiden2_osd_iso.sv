// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// raiden2_osd_iso — Isolatore OSD → CPU.
//
// Tutti i bit OSD usati dalle CPU passano qui. Vengono registrati con 2-FF
// synchronizer su clk_sys, con attributi preserve + SYNCHRONIZER_IDENTIFICATION.
// Risultato: i FF dei segnali CPU-side hanno un fanin LOCALE (questo modulo),
// non più dipendente da come Quartus rerouta status[127:0] di hps_io.
//
// Aggiungere/togliere bit OSD altrove NON sposta più il P&R dei segnali CPU.
//
// Ogni nuovo bit OSD destinato alla CPU va aggiunto SOLO qui (input + sync +
// output), così la regola "OSD non tocca CPU" resta valida per sempre.

module raiden2_osd_iso
(
	input  wire       clk,

	// ── Ingressi raw da hps_io / wires top ─────────────────────────────────
	input  wire       pause_in,
	input  wire [2:0] main_clk_sel_in,
	input  wire [2:0] sub_clk_sel_in,

	// ── Uscite stabilizzate (verso CPU main/sub) ───────────────────────────
	output wire       pause_out,
	output wire [2:0] main_clk_sel_out,
	output wire [2:0] sub_clk_sel_out
);

(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *)
reg pause_s1, pause_s2;
(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *)
reg [2:0] main_clk_sel_s1, main_clk_sel_s2;
(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *)
reg [2:0] sub_clk_sel_s1,  sub_clk_sel_s2;

always @(posedge clk) begin
	pause_s1        <= pause_in;
	pause_s2        <= pause_s1;
	main_clk_sel_s1 <= main_clk_sel_in;
	main_clk_sel_s2 <= main_clk_sel_s1;
	sub_clk_sel_s1  <= sub_clk_sel_in;
	sub_clk_sel_s2  <= sub_clk_sel_s1;
end

assign pause_out        = pause_s2;
assign main_clk_sel_out = main_clk_sel_s2;
assign sub_clk_sel_out  = sub_clk_sel_s2;

endmodule
