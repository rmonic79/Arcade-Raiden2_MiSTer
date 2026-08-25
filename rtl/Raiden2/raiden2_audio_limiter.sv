// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// raiden2_audio_limiter.sv — limitatore del mixer audio.
//
// PERCHE' ESISTE. Il limitatore di prima era una parabola valida per soli
// 2R = 4096 punti sopra TH = 29000: oltre 33096 diventava un MURO fisso a
// 31048. Con tre sorgenti (FM + due OKI) il mix supera il fondo scala gia'
// con una esplosione sopra la musica, quindi ci restava inchiodato = onda
// quadra. Sul banco: 93808 valori su 160001 finivano tutti sullo stesso
// numero. E' quello il gracchio delle esplosioni e la cupezza della musica
// che ci sta sotto.
//
// LA CURVA.  x = |mix|
//    x <= TH : y = x                      identita' ESATTA, bit per bit
//    x >  TH : y = TH + K*(1 - e^(-(x-TH)/K))   con K = FS - TH
// Scelta di K: la derivata in TH vale esattamente 1, quindi la curva si
// raccorda alla retta senza spigolo (C1), e' monotona, e ha come asintoto
// FS = 32767 — non ci arriva mai, quindi non c'e' nessun muro.
// Una parabola non poteva farlo: per coprire un eccesso di 2x servirebbe TH
// negativa. Una spezzata a pendenze dimezzate nemmeno: cambia pendenza di
// colpo in 5 punti e spreca l'escursione disponibile.
//
// REALIZZAZIONE. Tabella di 64 punti a passo 2048 (copre |mix| fino a 131071,
// il nostro picco e' ~67500) con interpolazione lineare fra i punti: un
// moltiplicatore 11x17 e due somme. Sotto TH non passa dalla tabella, cosi'
// l'identita' e' esatta e non approssimata.
//
// Tabella generata da: y = TH + K*(1-exp(-(x-TH)/K)), TH=20480, FS=32767.

`timescale 1ns / 1ps

module raiden2_audio_limiter #(
    parameter W  = 27,           // larghezza del bus di mix in ingresso
    parameter TH = 20480           // ginocchio (cade su un punto della tabella)
) (
    input  wire signed [W-1:0] mix_in,
    output wire signed  [15:0] mix_out
);

localparam signed [W-1:0] TH_S = TH;
localparam        [16:0]  FS   = 17'd32767;

wire signed [W-1:0] amag_s = mix_in[W-1] ? -mix_in : mix_in;
wire        [W-1:0] amag   = amag_s;

reg [16:0] LUT [0:63];
initial begin
	LUT[6'd 0] = 17'd0;
	LUT[6'd 1] = 17'd2048;
	LUT[6'd 2] = 17'd4096;
	LUT[6'd 3] = 17'd6144;
	LUT[6'd 4] = 17'd8192;
	LUT[6'd 5] = 17'd10240;
	LUT[6'd 6] = 17'd12288;
	LUT[6'd 7] = 17'd14336;
	LUT[6'd 8] = 17'd16384;
	LUT[6'd 9] = 17'd18432;
	LUT[6'd10] = 17'd20480;
	LUT[6'd11] = 17'd22366;
	LUT[6'd12] = 17'd23963;
	LUT[6'd13] = 17'd25315;
	LUT[6'd14] = 17'd26459;
	LUT[6'd15] = 17'd27427;
	LUT[6'd16] = 17'd28247;
	LUT[6'd17] = 17'd28941;
	LUT[6'd18] = 17'd29529;
	LUT[6'd19] = 17'd30026;
	LUT[6'd20] = 17'd30447;
	LUT[6'd21] = 17'd30803;
	LUT[6'd22] = 17'd31104;
	LUT[6'd23] = 17'd31360;
	LUT[6'd24] = 17'd31576;
	LUT[6'd25] = 17'd31759;
	LUT[6'd26] = 17'd31913;
	LUT[6'd27] = 17'd32044;
	LUT[6'd28] = 17'd32155;
	LUT[6'd29] = 17'd32249;
	LUT[6'd30] = 17'd32329;
	LUT[6'd31] = 17'd32396;
	LUT[6'd32] = 17'd32453;
	LUT[6'd33] = 17'd32501;
	LUT[6'd34] = 17'd32542;
	LUT[6'd35] = 17'd32577;
	LUT[6'd36] = 17'd32606;
	LUT[6'd37] = 17'd32631;
	LUT[6'd38] = 17'd32652;
	LUT[6'd39] = 17'd32669;
	LUT[6'd40] = 17'd32684;
	LUT[6'd41] = 17'd32697;
	LUT[6'd42] = 17'd32708;
	LUT[6'd43] = 17'd32717;
	LUT[6'd44] = 17'd32725;
	LUT[6'd45] = 17'd32731;
	LUT[6'd46] = 17'd32737;
	LUT[6'd47] = 17'd32741;
	LUT[6'd48] = 17'd32745;
	LUT[6'd49] = 17'd32749;
	LUT[6'd50] = 17'd32751;
	LUT[6'd51] = 17'd32754;
	LUT[6'd52] = 17'd32756;
	LUT[6'd53] = 17'd32758;
	LUT[6'd54] = 17'd32759;
	LUT[6'd55] = 17'd32760;
	LUT[6'd56] = 17'd32761;
	LUT[6'd57] = 17'd32762;
	LUT[6'd58] = 17'd32763;
	LUT[6'd59] = 17'd32764;
	LUT[6'd60] = 17'd32764;
	LUT[6'd61] = 17'd32765;
	LUT[6'd62] = 17'd32765;
	LUT[6'd63] = 17'd32765;
end

// indice = amag[16:11], frazione = amag[10:0]
wire        oltre = |amag[W-1:17];                 // fuori tabella -> tetto
wire  [5:0] idx   = amag[16:11];
wire [10:0] frac  = amag[10:0];
wire [16:0] y0    = LUT[idx];
wire [16:0] y1    = (idx == 6'd63) ? FS : LUT[idx + 6'd1];
wire [16:0] dy    = y1 - y0;
wire [27:0] interp= dy * frac;                     // 17x11
wire [16:0] ycurve= y0 + interp[27:11];

// sotto il ginocchio NON si passa dalla tabella: identita' esatta
wire [16:0] yabs0 = (amag_s <= TH_S) ? amag[16:0] : (oltre ? FS : ycurve);
wire [16:0] yabs  = (yabs0 > FS) ? FS : yabs0;     // rete di sicurezza
wire signed [16:0] yfull = mix_in[W-1] ? -$signed(yabs) : $signed(yabs);

assign mix_out = yfull[15:0];

endmodule
