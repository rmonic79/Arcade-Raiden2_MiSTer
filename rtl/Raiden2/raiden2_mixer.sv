// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
/*  raiden2_mixer.sv — compositore video con blending di palette.

    ESTRATTO da Raiden2.sv perche' era DUPLICATO fra il core e il banco di
    simulazione: la copia del banco decodificava la palette a 444 mentre il
    core usa 555, e la divergenza e' rimasta invisibile per mesi. Un modulo
    solo, usato da entrambi, e testabile a parte (sim/tb_mixer.cpp).

    MODELLO (MAME raiden2_v.cpp screen_update + blend_layer):
    si parte dal fondale NERO e si attraversano gli 8 livelli dal FONDO in
    avanti — spr0, BG, spr1, MG, spr2, FG, spr3, TEXT. Per ogni livello che
    ha un pixel opaco:
        se il suo indice di palette e' nella tabella -> il colore si MESCOLA
        al 50% con quanto accumulato finora;
        altrimenti -> lo SOSTITUISCE.
    Quindi DUE elementi trasparenti sovrapposti si COMPONGONO: il secondo
    mescola con il risultato del primo, non col colore grezzo sotto.

    La palette ha una sola porta di lettura utile per pixel, quindi
    l'accumulo si fa in SEQUENZA: 12 clock per pixel bastano per gli 8
    livelli. Uno stato scorre i livelli dal fondo, emette l'indirizzo di
    palette e accumula il colore.
*/

module raiden2_mixer (
	input  wire        clk,
	input  wire        ce_pix,

	// livelli, dal fondo in avanti: spr0, BG, spr1, MG, spr2, FG, spr3, TEXT
	input  wire [10:0] spr_pen,
	input  wire        spr_pri0, spr_pri1, spr_pri2, spr_pri3,
	input  wire [10:0] bg_pen,   input wire bg_opaque,
	input  wire [10:0] mg_pen,   input wire mg_opaque,
	input  wire [10:0] fg_pen,   input wire fg_opaque,
	input  wire [10:0] text_pen, input wire text_opaque,

	input  wire        video_de,

	// porta palette: indirizzo fuori, parola dentro DUE clock dopo
	output reg  [10:0] pal_addr,
	input  wire [15:0] pal_word,

	output wire  [7:0] rgb_r, rgb_g, rgb_b
);

// ── tabella: quali indici si mescolano (127 su 2048, da MAME) ───────────
// PERCORSO ESPLICITO IN SINTESI: col nome nudo Quartus non la troverebbe
// (nessun SEARCH_PATH copre rtl/Raiden2) e resterebbe TUTTA A ZERO, cioe'
// blending inerte sul silicio ma perfetto in simulazione.
(* ramstyle = "MLAB" *) reg blend_tab [0:2047];
`ifdef SYNTHESIS
initial $readmemb("rtl/Raiden2/raiden2_blend.mem", blend_tab);
`else
initial $readmemb("raiden2_blend.mem", blend_tab);
initial begin
	#0;
	if (blend_tab[11'h380] !== 1'b1 || blend_tab[11'h000] !== 1'b0)
		$fatal(1, "raiden2_blend.mem NON caricata");
end
`endif

// ── i livelli campionati a inizio pixel, dal FONDO in avanti ────────────
reg [10:0] pen_q  [0:7];
reg        vld_q  [0:7];
reg  [2:0] step;          // livello in lavorazione, 0 = il piu' in fondo
reg        busy;
reg  [7:0] acc_r, acc_g, acc_b;
// valore che l'accumulatore assume IN QUESTO ciclo: serve a latchare il
// risultato nello stesso istante in cui l'ultimo livello viene accumulato.
wire mixing_now = rd_vld[1] && vld_q[rd_step_d2];
wire blend_now  = blend_tab[pen_q[rd_step_d2]];
wire [7:0] acc_r_nxt = !mixing_now ? acc_r : (blend_now ? (({1'b0,acc_r}+{1'b0,pw_r})>>1) : pw_r);
wire [7:0] acc_g_nxt = !mixing_now ? acc_g : (blend_now ? (({1'b0,acc_g}+{1'b0,pw_g})>>1) : pw_g);
wire [7:0] acc_b_nxt = !mixing_now ? acc_b : (blend_now ? (({1'b0,acc_b}+{1'b0,pw_b})>>1) : pw_b);
reg        de_q;

// palette xBGR_555 -> RGB888 (espansione MAME palette_device)
wire [7:0] pw_r = {pal_word[ 4: 0], pal_word[ 4: 2]};
wire [7:0] pw_g = {pal_word[ 9: 5], pal_word[ 9: 7]};
wire [7:0] pw_b = {pal_word[14:10], pal_word[14:12]};

// la parola arriva DUE clock dopo l'indirizzo: pipeline di validita'
reg [1:0] rd_vld;
// I renderer aggiornano le uscite SUL fronte di ce_pix: campionarle li'
// prenderebbe il pixel PRECEDENTE (immagine spostata di un pixel — segnalato
// dall'utente sul text). Si campiona UN CICLO DOPO, quando sono stabili.
reg ce_pix_d;
reg [2:0] rd_step_d1, rd_step_d2;

integer k;
always @(posedge clk) begin
	rd_vld     <= {rd_vld[0], busy};
	rd_step_d1 <= step;
	rd_step_d2 <= rd_step_d1;

	ce_pix_d <= ce_pix;

	// I renderer aggiornano le uscite SUL fronte di ce_pix: campionarle li'
	// prenderebbe il valore PRECEDENTE (semantica del campionamento su fronte),
	// cioe' il pixel N-1 -> immagine spostata di un pixel. Si campiona UN CICLO
	// DOPO, quando valgono il pixel corrente. Restano 15 clock dei 16 del
	// pixel, e l'accumulo degli 8 livelli finisce all'undicesimo.
	if (ce_pix) begin
		acc_r <= 8'h00; acc_g <= 8'h00; acc_b <= 8'h00;   // fondale nero
		busy  <= 1'b0;
		rd_vld <= 2'b00;
	end else if (ce_pix_d) begin
		pen_q[0] <= spr_pen;  vld_q[0] <= spr_pri0;
		pen_q[1] <= bg_pen;   vld_q[1] <= bg_opaque;
		pen_q[2] <= spr_pen;  vld_q[2] <= spr_pri1;
		pen_q[3] <= mg_pen;   vld_q[3] <= mg_opaque;
		pen_q[4] <= spr_pen;  vld_q[4] <= spr_pri2;
		pen_q[5] <= fg_pen;   vld_q[5] <= fg_opaque;
		pen_q[6] <= spr_pen;  vld_q[6] <= spr_pri3;
		pen_q[7] <= text_pen; vld_q[7] <= text_opaque;
		de_q  <= video_de;
		step  <= 3'd0;
		busy  <= 1'b1;
	end else if (busy) begin
		if (step == 3'd7) busy <= 1'b0;
		else              step <= step + 3'd1;
	end

	// indirizzo del livello in lavorazione
	pal_addr <= pen_q[step];

	// accumulo: la parola letta appartiene al livello di due clock fa
	if (rd_vld[1] && vld_q[rd_step_d2]) begin
		if (blend_tab[pen_q[rd_step_d2]]) begin
			// SOMMA A 9 BIT: con due operandi a 8 bit e destinazione a 8 bit
			// Verilog valuta l'addizione a 8 bit e BUTTA IL RIPORTO, quindi
			// ogni canale la cui somma supera 255 usciva 128 piu' scuro.
			// Trovato dal test contro il modello MAME (sim/tb_mixer.cpp).
			acc_r <= ({1'b0, acc_r} + {1'b0, pw_r}) >> 1;   // 50% con l'ACCUMULATO
			acc_g <= ({1'b0, acc_g} + {1'b0, pw_g}) >> 1;
			acc_b <= ({1'b0, acc_b} + {1'b0, pw_b}) >> 1;
		end else begin
			acc_r <= pw_r;                  // opaco: sostituisce
			acc_g <= pw_g;
			acc_b <= pw_b;
		end
	end

end

// USCITA TENUTA PER TUTTO IL PIXEL. L'accumulatore viene azzerato a ce_pix e
// ricostruito in 10 clock: esporlo direttamente farebbe vedere l'ACCUMULO IN
// CORSO per gran parte del periodo, cioe' un valore che non appartiene al
// pixel corrente -> immagine spostata. Il risultato si latcha quando l'ultimo
// livello e' stato accumulato, e resta stabile fino al pixel dopo: all'istante
// di campionamento vale esattamente quanto valeva col compositore precedente,
// senza aggiungere latenza.
reg [7:0] hold_r, hold_g, hold_b;
reg       hold_de;
always @(posedge clk) begin
	if (rd_vld[1] && (rd_step_d2 == 3'd7)) begin
		hold_r  <= acc_r_nxt;
		hold_g  <= acc_g_nxt;
		hold_b  <= acc_b_nxt;
		hold_de <= de_q;
	end
end

// (commento storico) uscita combinatoria dall'accumulatore: nel compositore
// precedente `video_r` era un wire, quindi al fronte di ce_pix N+1 lo scaler
// campionava il pixel N — un pixel di latenza gia' presente. `acc` viene
// azzerato solo AL ce_pix successivo, quindi al fronte tiene ancora il
// risultato del pixel appena chiuso: stessa identica latenza di prima, e
// nessuna compensazione da fare sui sincronismi.
assign rgb_r = hold_de ? hold_r : 8'h00;
assign rgb_g = hold_de ? hold_g : 8'h00;
assign rgb_b = hold_de ? hold_b : 8'h00;

endmodule
