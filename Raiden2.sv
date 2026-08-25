// SPDX-License-Identifier: GPL-3.0-or-later
//
// Raiden (Seibu Kaihatsu, 1990) - MiSTer FPGA core
// Copyright (c) Umberto Parisi (rmonic79)
// Based on the MiSTer Template by Sorgelig.

module emu
(
	input         CLK_50M,
	input         RESET,
	inout  [48:0] HPS_BUS,
	output        CLK_VIDEO,
	output        CE_PIXEL,
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,
	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER,
	output        VGA_DISABLE,
	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,
`ifdef MISTER_FB_PALETTE
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,
	output  [1:0] BUTTONS,

	input         CLK_AUDIO,
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,
	output  [1:0] AUDIO_MIX,

	inout   [3:0] ADC_BUS,

	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

///////// Unused ports /////////
assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
// DDRAM gestito da raiden2_ddram (sprite ROM in DDR3) + screen_rotate (FB writes).
// Mux a fine modulo: sprite_rom_cache=read 0x30000000, rotate_fb=write 0x24000000.
assign DDRAM_CLK = clk_sys;
assign FB_FORCE_BLANK = 0;

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_DISABLE = 0;
// Pause: toggle on rising edge of joy[12] (standard MiSTer pause bit)
reg pause_toggle;
reg joy_pause_prev;
always @(posedge clk_sys) begin
	if (reset) begin
		pause_toggle <= 1'b0;
		joy_pause_prev <= 1'b0;
	end else begin
		joy_pause_prev <= joy0[12] | joy1[12];
		if ((joy0[12] | joy1[12]) && !joy_pause_prev)
			pause_toggle <= ~pause_toggle;
	end
end
wire pause = pause_toggle;  // solo pad (OSD pause rimosso, pattern Darius2)

// --- VBlank-synced pause (frame-aligned, pattern Darius2 F2) ---
// pause raw asincrono → paused_safe registrato che cambia SOLO al rising edge
// vblank. Sincronizza pause boundary su tutti i moduli (CPU cen, audio cen).
// Necessario per evitare race a metà bus cycle / scanline / DDR3 transaction.
wire ss_busy;       // savestate DMA in corso — da save_state_data.busy
wire ss_mgr_pause;  // richiesta pausa dal coordinatore (raiden2_ss_manager)
// Coordinamento frame-aligned (pattern BoogieWings/F2): alla pressione il manager
// alza ss_mgr_pause SUBITO; paused_safe sale al vblank successivo (confine frame);
// solo allora il manager pulsa il DMA. ss_mgr_pause resta alto per tutto il DMA
// (fino a ss_busy basso) → paused_safe resta alto → stato COERENTE per il memory_stream.
// Il memory_stream dirotta le porte BRAM al ssbus (adaptor), no race.
reg vblank_prev_safe;
reg paused_safe_r;
always @(posedge clk_sys) begin
	if (reset) begin
		vblank_prev_safe <= 1'b0;
		paused_safe_r    <= 1'b0;
	end else begin
		vblank_prev_safe <= VBlank;
		// una volta alto per SS resta alto finché il manager tiene la pausa
		// (ss_mgr_pause), che scende solo a DMA finito → paused_safe coerente.
		if (ss_mgr_pause & paused_safe_r)
			paused_safe_r <= 1'b1;
		else if (VBlank && !vblank_prev_safe)
			paused_safe_r <= pause | ss_mgr_pause;
	end
end
wire paused_safe = paused_safe_r;
assign HDMI_FREEZE = 1'b0;  // overlay pause renderizzato real-time, no freeze scaler
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;  // signed audio
wire signed [15:0] game_audio_l, game_audio_r;
assign AUDIO_L = game_audio_l;
assign AUDIO_R = game_audio_r;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[122:121];

// Offset layer COPIATI dalla _old (video HW-perfetto, Raiden2.sv:446-463):
// TRIM0 = 0 per TUTTI i layer ("esattamente il valore con cui il core
// girava"), sprite xoff = -1 ("base famiglia, match MAME su HW"),
// txt xoff = -1. I +16/-16 precedenti erano una deduzione sbagliata
// ("valori runtime attuali") da bit OSD morti: sballavano la composizione
// di 16 px tra i layer. 2026-08-13.
wire signed [9:0] osd_l0_xoff  = 10'sd0;
wire signed [9:0] osd_l0_yoff  = 10'sd0;
wire signed [9:0] osd_spr_xoff = -10'sd1;
wire signed [9:0] osd_spr_yoff = 10'sd0;
wire signed [9:0] osd_fg_xoff  = 10'sd0;
wire signed [9:0] osd_fg_yoff  = 10'sd0;
wire signed [9:0] osd_txt_xoff = -10'sd1;
wire signed [9:0] osd_txt_yoff = 10'sd0;
wire signed [9:0] osd_bg_xoff  = osd_l0_xoff;
wire signed [9:0] osd_bg_yoff  = osd_l0_yoff;

`include "build_id.v"
localparam CONF_STR = {
	"Raiden2;SS3E000000:200000;",
	"-;",
	"O[109:105],Savestate Slot,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32;",
	"R[110],Save state (Alt-F1);",
	"R[111],Restore state (F1);",
	"-;",
	"P1,Video;",
	"P1O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[7:5],Scale,Normal,V-Integer,HV-Integer,Narrower HV-Integer;",
	"P1O[2:1],Rotate,No,CCW (TATE),CW;",
	"P1O[3],Flip 180,Off,On;",
	"P1O[19],Refresh Rate,Original 55.4Hz,60Hz;",
	"P1O[79],CRT Adjust,Off,On;",
	"H1P1O[66:62],CRT H-Size,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[104:98],CRT H-Position,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[61:56],CRT V-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[95:92],CRT V-Size,0,+1,+2,+3,+4,+5,+6,+7,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[96],CRT V-Size Mode,PVM,Cabinet;",
	"-;",
	"O[18],Clean Pause,Off,On;",
	"O[30],Player,1P,2P;",
	"-;",
	"DIP;",
	"-;",
	"P3,Audio;",
	"P3O[87:84],FM Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[91:88],OKI1 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[11:8],OKI2 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[69:67],OKI1 Ch1 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[72:70],OKI1 Ch2 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[75:73],OKI1 Ch3 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[78:76],OKI1 Ch4 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[14:12],OKI2 Ch1 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[17:15],OKI2 Ch2 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[22:20],OKI2 Ch3 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[25:23],OKI2 Ch4 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[42:40],FM Ch1 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[45:43],FM Ch2 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[48:46],FM Ch3 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[51:49],FM Ch4 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[114:112],FM Ch5 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[117:115],FM Ch6 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[120:118],FM Ch7 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"P3O[125:123],FM Ch8 Volume,Default,Mute,25%,50%,75%,100%,125%,150%;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"-;",
	// J1: bit 4=Fire(A), 5=Bomb(B), 6,7,8,9=unused, 10=Start, 11=Coin, 12=Pause.
	// Start 2P/Coin 2P rimossi: per giocare P2 con 1 pad usa OSD "Controls: Swap 1P/2P".
	"J1,Fire,Bomb,-,-,-,-,Start,Coin,Pause;",
	"jn,A,B,,,,,Start,R,L;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
wire [10:0] ps2_key;
wire [15:0] joy0, joy1;
// ioctl_* come uscono da hps_io (RAW dalla MRA, encrypted dove serve)
wire        ioctl_download_raw;
wire [15:0] ioctl_index_raw;
wire        ioctl_wr_raw;
wire [26:0] ioctl_addr_raw;
wire [15:0] ioctl_dout_raw;
wire        ioctl_wait;

hps_io #(.CONF_STR(CONF_STR), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),
	.forced_scandoubler(forced_scandoubler),
	.buttons(buttons),
	.status(status),
	.status_menumask({14'd0, ~status[79], 1'b0}),   // H1: gruppo CRT visibile solo con Adjust On
	.ps2_key(ps2_key),
	.joystick_0(joy0),
	.joystick_1(joy1),
	.ioctl_download(ioctl_download_raw),
	.ioctl_index(ioctl_index_raw),
	.ioctl_wr(ioctl_wr_raw),
	.ioctl_addr(ioctl_addr_raw),
	.ioctl_dout(ioctl_dout_raw),
	.ioctl_wait(ioctl_wait)
);

// === Savestate UI: trigger save/load da tasti (Alt+F1-F4 / F1-F4), gamepad, OSD ===
wire       ss_save, ss_load;
wire [4:0] ss_slot;        // 32 slot: [4:3]=regione (file .ss1-.ss4), [2:0]=sotto-slot
wire [15:0] joy_all = joy0 | joy1;
savestate_ui #(.INFO_TIMEOUT_BITS(25)) u_ss_ui (
	.clk         (clk_sys),
	.ps2_key     (ps2_key),
	.allow_ss    (1'b1),
	.joySS       (joy_all[13]),   // Select
	.joyRight    (joy_all[0]),
	.joyLeft     (joy_all[1]),
	.joyDown     (joy_all[2]),
	.joyUp       (joy_all[3]),
	.joyStart    (joy_all[12]),
	.joyRewind   (1'b0),
	.rewindEnable(1'b0),
	.status_slot (status[109:105]),   // 32 slot: la lista OSD e' O[109:105]
	.autoincslot (1'b0),
	.OSD_saveload(status[111:110]),  // R[110]=save, R[111]=restore
	.ss_save     (ss_save),
	.ss_load     (ss_load),
	.ss_info_req (),
	.ss_info     (),
	.statusUpdate(),
	.selected_slot(ss_slot)
);

// Raiden II: il programma main V30 e' IN CHIARO (raiden2.cpp non ha
// init_decryption). Il decrypt di Raiden 1 qui corrompeva $20000-$9FFFF
// del main. Stream ioctl passa raw; l'unica cifratura e' sugli sprite
// (crypt/raiden2_sprite_decrypt, piu' avanti).
wire        ioctl_download = ioctl_download_raw;
wire [15:0] ioctl_index    = ioctl_index_raw;
wire        ioctl_wr       = ioctl_wr_raw;
wire [26:0] ioctl_addr     = ioctl_addr_raw;
wire [15:0] ioctl_dout     = ioctl_dout_raw;

// --- Joystick to Raiden input mapping ---
// MAME P1_P2 port ($E0002): active low.
// Low byte P1 / high byte P2: bit0=U, bit1=D, bit2=L, bit3=R, bit4=Btn1, bit5=Btn2.
// MiSTer joy bits: joy[0]=R, joy[1]=L, joy[2]=D, joy[3]=U, joy[4]=A, joy[5]=B.
// Raiden P1_P2 layout MAME (raiden.cpp:589-605):
//   bit 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT, 4=BTN1(Fire), 5=BTN2(Bomb), 6=unused, 7=START1
//   bit 8-15 = P2 (stesso layout)
// MiSTer joy bits: 0=R, 1=L, 2=D, 3=U, 4=A=Fire, 5=B=Bomb, 10=Start
// Active LOW.
// Player 1P/2P (OSD O[30]): con 1 solo pad + "2P", il pad guida la nave P2.
wire        swap_pl = status[30];
wire [15:0] jp1 = swap_pl ? joy1 : joy0;
wire [15:0] jp2 = swap_pl ? joy0 : joy1;
wire [7:0] p1_input = {~jp1[10], 1'b1, ~jp1[5], ~jp1[4], ~jp1[0], ~jp1[1], ~jp1[2], ~jp1[3]};
wire [7:0] p2_input = {~jp2[10], 1'b1, ~jp2[5], ~jp2[4], ~jp2[0], ~jp2[1], ~jp2[2], ~jp2[3]};
wire [15:0] p1_p2_input = {p2_input, p1_input};

// Raiden NON ha system_input separato. P1_P2 inglobano già Start (bit 7=START1,
// bit 15=START2). Lo costruisco in p1_p2_input più sopra. Coin va via SEIBU_COIN_INPUTS
// (Z80 path → soundlatch). Service in coin_input bit (Z80 leggerà il segnale).
// SYSTEM ($74C): bit0/1 = START1/2 attivi bassi; bit8/9 (START3/4) a riposo.
wire [15:0] system_input16 = {6'h3F, 1'b1, 1'b1, 6'h3F, ~jp2[10], ~jp1[10]};

// Seibu coin input: Raiden II usa SEIBU_COIN_INPUTS_INVERT (raiden2.cpp:768,
// tutti i bit ATTIVI BASSI, a riposo 0xFF — confermato dal ROM nella _old:
// il ramo che accredita e' quello con AND == 0).
wire [7:0] coin_input = ~{6'd0, jp2[11], jp1[11]};

// DIP switches — loaded from MRA via ioctl (index 254)
// Raiden II: DSW1 ($740, word 0) + DSW2 ($75C, word 1). Active-LOW.
reg [15:0] dip_sw  = 16'hFFFF;
reg [15:0] dip_sw2 = 16'hFFFF;
always @(posedge clk_sys)
	if (ioctl_wr && (ioctl_index == 16'd254)) begin
		if      (!ioctl_addr[26:1])         dip_sw  <= ioctl_dout;
		else if (ioctl_addr[26:1] == 26'd1) dip_sw2 <= ioctl_dout;
	end

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;    // 80 MHz: monolitico
wire pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.locked(pll_locked)
);

// Game reset: include ioctl_download + reset_hold 22-bit (~43 ms @96 MHz).
// Pattern documentato: project_raiden2_download_stretch_useful.md (commit 213d572).
// Senza hold il V30 esce da reset prima che SDRAM sia stabile → primo fetch a
// 0xFFFF0 ritorna garbage → schermo nero. Revert (8dd29a2) = regressione.
wire reset_cause = RESET | status[0] | buttons[1] | ~pll_locked | ioctl_download;
reg [21:0] reset_hold_cnt = 22'h3FFFFF;
always @(posedge clk_sys) begin
	if (reset_cause)                  reset_hold_cnt <= 22'h3FFFFF;
	else if (reset_hold_cnt != 22'd0) reset_hold_cnt <= reset_hold_cnt - 22'd1;
end
// ── Reset REGISTRATO (2026-08-14) ────────────────────────────────────────
// Prima: wire reset = (reset_hold_cnt != 22'd0). Quel confronto a 22 bit era
// COMBINATORIO davanti al reset di OGNI flip-flop del core: nella STA della
// build 20 e' il cono peggiore in assoluto e da solo produce 146 dei 250 path
// falliti — 101 verso i registri interni del V30 (worst -1.747 ns) e 45 verso
// l'audio (jt51). Un reset che arriva in ritardo ai registri della CPU li fa
// uscire dal reset in cicli diversi: la CPU parte incoerente = schermo nero,
// e siccome dipende dal routing il risultato cambia a ogni fit (build 18
// -1.030 / 19 -1.276 / 20 -1.747: la 20 e' nera con logica equivalente).
// Registrandolo, l'OR a 22 bit alimenta UN solo carico e i consumatori vedono
// un path registro->registro; maxfan lascia replicare il registro per zona.
// Semantica: rilascio (e asserzione) ritardati di 1 clock su un hold di ~43 ms.
// NON tocca reset_cause (ioctl_download resta dentro) ne' bridge/video reset.
wire reset_pre = (reset_hold_cnt != 22'd0);
(* maxfan = 32 *) reg reset = 1'b1;
always @(posedge clk_sys) reset <= reset_pre;
// Bridge reset: ONLY pll_locked — bridge must run during download
// (revert dd86f8b: includere user reset causa mismatch sdram_ack/sdram_req,
// SDRAM controller non si resetta → bridge in reset vs SDRAM running → stuck)
wire bridge_reset = ~pll_locked;
// Video reset: ONLY pll_locked — CRT needs sync always
wire video_reset = ~pll_locked;

///////////////////////   SDRAM   ///////////////////////////////

// Genesis 4-port SDRAM controller (Sorgelig + donor bridge)
// Port 0: graphics ROM + download
// Port 1: main 68000 ROM
// Port 2: temporarily unused donor ROM path
// Port 3: audio/sample ROM path

wire [24:1] sd_addr0, sd_addr1, sd_addr2, sd_addr3;
wire [15:0] sd_din0, sd_din1, sd_din2, sd_din3;
wire        sd_wrl0, sd_wrh0, sd_wrl1, sd_wrh1, sd_wrl2, sd_wrh2, sd_wrl3, sd_wrh3;
wire        sd_req0, sd_req1, sd_req2, sd_req3;
wire        sd_ack0, sd_ack1, sd_ack2, sd_ack3;
wire [15:0] sd_dout0, sd_dout1, sd_dout2, sd_dout3;
wire        sdram_ready;

// OKI ADPCM ROM bridge ↔ jt6295 (via main_top)
wire [18:0] oki_rom_addr;   // Raiden II: OKI1 256KB+banking → 19 bit (bridge _old)
wire  [7:0] oki_rom_data;
wire        oki_rom_ok;

sdram sdram_ctrl
(
	.SDRAM_DQ(SDRAM_DQ),
	.SDRAM_A(SDRAM_A),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_BA(SDRAM_BA),
	.SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nWE(SDRAM_nWE),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_CLK(SDRAM_CLK),
	.SDRAM_CKE(SDRAM_CKE),

	.init(~pll_locked),
	.clk(clk_sys),
	.prio_mode(status[35:34]),
	.ready(sdram_ready),

	.addr0(sd_addr0), .wrl0(sd_wrl0), .wrh0(sd_wrh0),
	.din0(sd_din0), .dout0(sd_dout0), .req0(sd_req0), .ack0(sd_ack0),

	.addr1(sd_addr1), .wrl1(sd_wrl1), .wrh1(sd_wrh1),
	.din1(sd_din1), .dout1(sd_dout1), .req1(sd_req1), .ack1(sd_ack1),

	.addr2(sd_addr2), .wrl2(sd_wrl2), .wrh2(sd_wrh2),
	.din2(sd_din2), .dout2(sd_dout2), .req2(sd_req2), .ack2(sd_ack2),

	.addr3(sd_addr3), .wrl3(sd_wrl3), .wrh3(sd_wrh3),
	.din3(sd_din3), .dout3(sd_dout3), .req3(sd_req3), .ack3(sd_ack3)
);

///////////////////////   BRIDGE   ///////////////////////////////

// Bridge between game logic (level protocol) and Genesis SDRAM (toggle protocol)
wire [23:0] game_tile_addr, game_main_addr, game_sub_addr;
wire        game_tile_req, game_main_req, game_sub_req;
wire  [2:0] game_tile_kind;     // 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
wire [31:0] game_tile_data;
wire        game_tile_valid;
wire [15:0] game_main_data, game_sub_data;
// Audio Z80 ROM removed from SDRAM — will use BRAM when audio implemented
wire        game_main_ready, game_sub_ready;

// BYPASS rom_cache (pattern WonderSwan_MiSTer): CPU bus_read direttamente sul
// bridge SDRAM. WonderSwan non usa cache — sdram controller registra dout, dato
// stabile fino al prossimo accesso. La cache introduceva 1 ciclo di latenza
// extra + cpu_ready pulse 1 ciclo che la CPU V30 (10 MHz → ~9 cicli clk_sys
// per ce) poteva facilmente perdere.
wire [23:0] bridge_main_addr = game_main_addr;
wire        bridge_main_req  = game_main_req;
wire [15:0] bridge_main_data;
wire        bridge_main_ready;
assign game_main_data  = bridge_main_data;
assign game_main_ready = bridge_main_ready;

// SUB ROM in SDRAM (porta 2), non piu' in BRAM. Il download scrive gia' tutto
// in SDRAM (Sub @ word offset SUB_BASE=0x030000); il Sub V30 legge via bridge
// come il Main (READY/Tw per la latenza SDRAM). Rimosso il duplicato BRAM
// (256KB = ~205 M10K liberati).
wire [15:0] bridge_sub_data;
wire        bridge_sub_ready;
wire [23:0] sub_cache_sdram_addr;
wire        sub_cache_sdram_req;

// rom_cache: la maggior parte dei fetch del Sub in 1 ciclo (niente SDRAM) ->
// il Sub non ruba banda alla grafica -> niente nero da contesa.
// CACHE_BITS 13 = 16KB (8192x16): miss rarissimi -> timing Sub quasi-BRAM.
// Con 9 (1KB) il thrash nelle scene pesanti rallentava/jitterava il Sub ->
// race mailbox main<->sub -> leak slot sprite (detriti fissi a schermo, stage 3).
rom_cache #(.CACHE_BITS(13)) u_sub_cache (
	.clk(clk_sys), .reset(bridge_reset),
	.cpu_addr(game_sub_addr), .cpu_req(game_sub_req),
	.cpu_data(game_sub_data), .cpu_ready(game_sub_ready),
	.sdram_addr(sub_cache_sdram_addr), .sdram_req(sub_cache_sdram_req),
	.sdram_data(bridge_sub_data), .sdram_ready(bridge_sub_ready)
);

// ── Board select (MRA <rom index="1">) ───────────────────────────────────
// L'HPS scarica index=1 PRIMA delle ROM (index=0); i DIP (index=254) arrivano
// DOPO, quindi non servirebbero: la variante deve essere nota gia' durante il
// download. Default 0 = Raiden II, cosi' una MRA priva della regione continua
// a funzionare. NON azzerato dal reset di gioco: un soft reset dall'OSD non
// deve riportare il core alla variante base.
reg board_dx = 1'b0;
always @(posedge clk_sys) begin
	// (a) byte esplicito della MRA
	if (ioctl_wr && (ioctl_index == 16'd1)) board_dx <= ioctl_dout[0];
	// (b) AUTORICONOSCIMENTO dal flusso, indipendente dalla regione index=1:
	// SOLO Raiden DX scrive oltre $F00000 (li' sta il 2o MB di programma).
	// Lo stream di Raiden II finisce a $E40000. Serve perche' se per qualsiasi
	// motivo la regione index=1 non arriva, DX girerebbe con la mappa di
	// Raiden II e il CRTC non scambiato = schermo nero silenzioso.
	if (ioctl_download && ioctl_wr && (ioctl_index == 16'd0)
	    && (ioctl_addr >= 27'hF00000)) board_dx <= 1'b1;
end

sdram_bridge bridge
(
	.board_dx(board_dx),
	.clk(clk_sys),
	.reset(bridge_reset),
	.sdram_ready(sdram_ready),

	// HPS download
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),

	// Game: Tile ROM (32-bit)
	.tile_byte_addr(game_tile_addr),
	.tile_req(game_tile_req),
	.gfx_kind(game_tile_kind),
	.tile_data(game_tile_data),
	.tile_valid(game_tile_valid),

	// Game: Main CPU ROM (16-bit)
	.main_byte_addr(bridge_main_addr),
	.main_req(bridge_main_req),
	.main_data(bridge_main_data),
	.main_ready(bridge_main_ready),

	// Sub V30 ROM in SDRAM (porta 2) via rom_cache.
	.sub_byte_addr(sub_cache_sdram_addr),
	.sub_req(sub_cache_sdram_req),
	.sub_data(bridge_sub_data),
	.sub_ready(bridge_sub_ready),

	// OKI ADPCM ROM (port 3)
	.oki_byte_addr(oki_rom_addr),
	.oki_data(oki_rom_data),
	.oki_ok(oki_rom_ok),

	// SDRAM ports
	.sdram_addr0(sd_addr0), .sdram_din0(sd_din0),
	.sdram_wrl0(sd_wrl0), .sdram_wrh0(sd_wrh0),
	.sdram_req0(sd_req0), .sdram_ack0(sd_ack0), .sdram_dout0(sd_dout0),

	.sdram_addr1(sd_addr1), .sdram_din1(sd_din1),
	.sdram_wrl1(sd_wrl1), .sdram_wrh1(sd_wrh1),
	.sdram_req1(sd_req1), .sdram_ack1(sd_ack1), .sdram_dout1(sd_dout1),

	.sdram_addr2(sd_addr2), .sdram_din2(sd_din2),
	.sdram_wrl2(sd_wrl2), .sdram_wrh2(sd_wrh2),
	.sdram_req2(sd_req2), .sdram_ack2(sd_ack2), .sdram_dout2(sd_dout2),

	.sdram_addr3(sd_addr3), .sdram_din3(sd_din3),
	.sdram_wrl3(sd_wrl3), .sdram_wrh3(sd_wrh3),
	.sdram_req3(sd_req3), .sdram_ack3(sd_ack3), .sdram_dout3(sd_dout3)
);

///////////////////////   GAME   ///////////////////////////////

// render_x/y prodotti da Raiden2_video_timing più sotto
wire [9:0]  render_x;
wire [8:0]  render_y;

// Palette read-side: indirizzo deciso dal pixel pipeline (priorità tra layer)
wire [10:0] pal_b_addr;   // pilotato da raiden2_mixer

// Text VRAM read wires
wire [10:0] text_vram_addr;
wire [15:0] text_vram_data;

// ── Scroll/ctrl esposti dal Main top (scroll_ram $0F000 + ctrl_w $0E006) ──
// Layout legacy ctrl_l0: [0]=BG, [1]=MG, [2]=FG, [3]=Text, [4]=Spr, [5]=flip
wire [511:0] scroll_words_flat;
// MAME raiden.cpp:391-401 — scroll_ram è u16 array, indici 0xNN sono WORD index
// (non byte address). Ogni m_scroll_ram[N] = u16 word, di cui CPU usa solo low byte.
// Formula (raiden.cpp:398): ((b_hi & 0xF0) << 4) | ((b_lo & 0x7F) << 1) | ((b_lo & 0x80) >> 7)
//   scrollregs[0] = BG_X = scroll_ram[0x09].lo<<4 | scroll_ram[0x0A].lo shifted
//   scrollregs[1] = BG_Y = scroll_ram[0x01].lo<<4 | scroll_ram[0x02].lo shifted
//   scrollregs[2] = FG_X = scroll_ram[0x19].lo<<4 | scroll_ram[0x1A].lo shifted
//   scrollregs[3] = FG_Y = scroll_ram[0x11].lo<<4 | scroll_ram[0x12].lo shifted
// scroll_words_flat[gi*16 +: 16] = scroll_ram[gi]  →  low byte = [gi*16 +: 8]
// (BUG corretto v81: prima usavamo byte indices, MAME usa word indices.)
// ── Raiden II: scroll/enable/flip vengono dal Seibu CRTC (dentro main_top),
// NON più da scroll_ram $F000 / ctrl $E006 di Raiden 1. Il decode a bytes
// (sr_b*) resta inerte finché scroll_words_flat non viene rimosso del tutto.
wire        crtc_layer_en_bg, crtc_layer_en_mg, crtc_layer_en_fg;
wire        crtc_layer_en_text, crtc_layer_en_spr, crtc_flip_screen;
wire [15:0] crtc_scroll_bg_x, crtc_scroll_bg_y;
wire [15:0] crtc_scroll_mg_x, crtc_scroll_mg_y;
wire [15:0] crtc_scroll_fg_x, crtc_scroll_fg_y;
wire [15:0] crtc_base_bg_x, crtc_base_bg_y, crtc_base_mg_x, crtc_base_mg_y;
wire [15:0] crtc_base_fg_x, crtc_base_fg_y, crtc_base_txt_x, crtc_base_txt_y;
wire [15:0] gfx_bank;   // [2:0]=BG [6:4]=MID [10:8]=FG (consumer: renderer _old)

wire [15:0] map_xscroll_l0 = crtc_scroll_bg_x;   // BG X (CRTC $00)
wire [15:0] map_yscroll_l0 = crtc_scroll_bg_y;   // BG Y
wire [15:0] map_xscroll_l1 = crtc_scroll_fg_x;   // FG X (CRTC $04-$05)
wire [15:0] map_yscroll_l1 = crtc_scroll_fg_y;   // FG Y
wire        ctrl_bg_en, ctrl_fg_en, ctrl_tx_en, ctrl_sp_en, ctrl_flipscreen;
// Layout legacy ctrl_l0 consumato dai renderer: [0]=BG,[2]=FG,[3]=Text,
// [4]=Spr,[5]=flip — ora alimentato dal CRTC.
wire [15:0] map_ctrl_l0 = {9'd0, 1'b0, crtc_flip_screen,
                            crtc_layer_en_spr, crtc_layer_en_text,
                            crtc_layer_en_fg, crtc_layer_en_mg, crtc_layer_en_bg};

// ── Shared RAM: NON esiste su Raiden II (niente sub-CPU). Le porte del
// main_top restano (mai selezionate: shared_memrq=0); slot SS 3 terminato.
wire [11:1] main_shared_addr;
wire        main_shared_cs;
wire  [1:0] main_shared_we;
wire [15:0] main_shared_wdata;
wire [15:0] main_shared_rdata = 16'hFFFF;

// Lo slot 3 (su Raiden 1 la RAM condivisa, qui inesistente) ospita ora i
// REGISTRI DI BANCO, che su questa scheda decidono quale pagina di programma
// la CPU sta eseguendo e prima non li salvava nessuno.


// ── Sound comm bus: pilotato da Z80 audio reale (Raiden2_audio_z80) ────────
wire        snd_cs;
wire  [4:1] snd_addr;   // registro seibu (offset>>1): 4 bit come la _old
wire        snd_wr;
wire        snd_rd;
wire [15:0] snd_wdata;
wire [15:0] snd_rdata;

// ── Palette → RGB888 — COPIATA dalla _old (Raiden2_palette.sv:85-96,
// video HW-perfetto): Raiden2 = xBGR_555 (legionna.cpp:1212), NON 444.
// Layout word: bit[4:0]=R, bit[9:5]=G, bit[14:10]=B. Espansione 5→8
// standard MAME palette_device: {val5, val5[4:2]}. 2026-08-13.
wire [15:0] pal_word_raw;   // parola di palette -> raiden2_mixer (conversione 555 dentro il modulo)

// ── Audio Seibu reale: Z80 + YM3812 (jtopl2) + OKI6295 (jt6295) ─────────────
// MRA layout audiocpu: 0x0A0000-0x0AFFFF (64KB raw byte-pack)
// OKI ROM: SDRAM @ OKI_BASE (oki_rom_addr/data/ok via SDRAM bridge)
// Coin button (joy[11]) → Z80 0x4013 → sub2main → main 0xA0004 → coin_credit
// Audio Raiden II (modulo della _old): Z80 seibu liscio + YM2151 (jt51) +
// 2× OKI M6295 (OKI1 su SDRAM, OKI2 su DDR3). Clock e guadagni collaudati
// nel modulo (Z80 96/27, OKI 96/94 = 1.02128 MHz; vol_sel 0 = default).
Raiden2_audio_z80 #(.SS_IDX_ZRAM(10), .SS_IDX_Z80(12),
                    .SS_IDX_YMSH(13), .SS_IDX_GLUE(14)) u_audio (
	.clk           (clk_sys),
	.reset         (reset),
	.pause         (paused_safe),
	.clk_sel       (2'd0),         // legacy, ignored
	// Fette a QUATTRO bit, 1:1 con le voci dichiarate nella stringa OSD.
	// Prima erano [86:84]/[90:88]/... a tre bit contro un menu da 16 voci:
	// il bit alto cadeva e da "200%" in su le voci non facevano nulla.
	.fm_vol_sel (status[87:84]),
	.oki_vol_sel (status[91:88]),
	.oki2_vol_sel (status[11:8]),
	.oki_ch_vol_sel0 (status[69:67]),
	.oki_ch_vol_sel1 (status[72:70]),
	.oki_ch_vol_sel2 (status[75:73]),
	.oki_ch_vol_sel3 (status[78:76]),
	.oki2_ch_vol_sel0 (status[14:12]),
	.oki2_ch_vol_sel1 (status[17:15]),
	.oki2_ch_vol_sel2 (status[22:20]),
	.oki2_ch_vol_sel3 (status[25:23]),
	.fm_ch_vol_sel0 (status[42:40]),
	.fm_ch_vol_sel1 (status[45:43]),
	.fm_ch_vol_sel2 (status[48:46]),
	.fm_ch_vol_sel3 (status[51:49]),
	.fm_ch_vol_sel4 (status[114:112]),
	.fm_ch_vol_sel5 (status[117:115]),
	.fm_ch_vol_sel6 (status[120:118]),
	.fm_ch_vol_sel7 (status[125:123]),
	.ioctl_download(ioctl_download),
	.ioctl_wr      (ioctl_wr),
	.ioctl_addr    (ioctl_addr),
	.ioctl_dout    (ioctl_dout),
	.snd_cs        (snd_cs),
	.snd_addr      (snd_addr),
	.snd_wr        (snd_wr),
	.snd_rd        (snd_rd),
	.snd_wdata     (snd_wdata),
	.snd_rdata     (snd_rdata),
	.snd_nmi_n     (1'b1),
	.snd_reset_in  (1'b0),
	.coin_input    (coin_input),
	.oki_rom_addr  (oki_rom_addr),
	.oki_rom_data  (oki_rom_data),
	.oki_rom_ok    (oki_rom_ok),
	.oki2_rom_addr (oki2_rom_addr),
	.oki2_rom_data (oki2_rom_data),
	.oki2_rom_ok   (oki2_rom_ok),
	.audio_l       (game_audio_l),
	.audio_r       (game_audio_r),
	.ss_zram       (ssb[10]),
	.ss_z80        (ssb[12]),
	.ss_ymsh       (ssb[13]),
	.ss_glue       (ssb[14]),
	.z80_ss_ready  (z80_ss_ready)
);
// Gli slot 12/13/14 sono ora dell'audio (regs Z80, shadow YM2151, glue):
// prima erano terminati a vuoto e lo stato del sonoro non veniva salvato.

// ── Isolatore OSD → CPU (2-FF sync, attributi preserve). ──
// Aggiungere bit OSD altrove NON destabilizza più le CPU.
wire        pause_iso;
wire  [2:0] main_clk_sel_iso, sub_clk_sel_iso;
raiden2_osd_iso u_osd_iso (
	.clk              (clk_sys),
	.pause_in         (paused_safe),
	.main_clk_sel_in  (3'd0),   // CPU FISSA 10 MHz — scollegata dall'OSD (no overclock)
	.sub_clk_sel_in   (3'd0),   // CPU FISSA 10 MHz — scollegata dall'OSD (no overclock)
	.pause_out        (pause_iso),
	.main_clk_sel_out (main_clk_sel_iso),
	.sub_clk_sel_out  (sub_clk_sel_iso)
);

// ── Savestate: park delle CPU a confine d'istruzione ──────────────────────
// Bug savestate: la cattura è frame-aligned (vblank), non allineata al confine
// istruzione del V30. Se una CPU è a metà istruzione al save, reg_ip punta a
// metà istruzione e la FSM microcode (non salvata) è persa → al restore la CPU
// misdecoda → freeze (intermittente). Fix: quando arriva la pausa, la CPU
// continua a girare finché non raggiunge CPUSTAGE_IDLE (cpu_idle=1), POI si
// congela (park). La cattura SS parte solo quando ENTRAMBE sono a confine.
wire main_cpu_idle;
wire main_cpu_pause = pause_iso & main_cpu_idle;   // gira finché non è idle, poi park
// Cattura savestate sicura: V30 a confine bus E COP3 fermo (il COP scrive le
// stesse RAM che gli adaptor dirottano al ssbus — pattern _old).
wire z80_ss_ready;   // Z80 audio parcheggiato a confine istruzione
wire cpus_ss_ready  = main_cpu_idle & ~cop_busy_main & z80_ss_ready;

// ── Main V30 (raiden2_state::main_map) ──
Raiden2_main_top #(.SS_IDX_SPR(7), .SS_IDX_CPU(8), .SS_IDX_BANK(3),
                  .SS_IDX_COPA(11), .SS_IDX_COPR(15)) u_main (
	.board_dx         (board_dx),
	.clk              (clk_sys),
	.reset            (reset),
	.pause            (main_cpu_pause),
	.cpu_idle         (main_cpu_idle),
	.clk_sel          (main_clk_sel_iso),      // = 0 → 10 MHz fisso (no overclock OSD)
	.p1_input         (p1_input),
	.p2_input         (p2_input),
	.dsw_input        (dip_sw),
	.dsw2_input       (dip_sw2),
	.system_input     (system_input16),
	.main_rom_rdata   (game_main_data),
	.main_rom_ready   (game_main_ready),
	.main_rom_addr    (game_main_addr),
	.main_rom_req     (game_main_req),
	.vblank_in        (VBlank),
	.ioctl_download   (ioctl_download),
	.ctrl_bg_en       (ctrl_bg_en),
	.ctrl_fg_en       (ctrl_fg_en),
	.ctrl_tx_en       (ctrl_tx_en),
	.ctrl_sp_en       (ctrl_sp_en),
	.ctrl_flipscreen  (ctrl_flipscreen),
	.scroll_words_flat(scroll_words_flat),
	.snd_cs           (snd_cs),
	.snd_addr         (snd_addr),
	.snd_wr           (snd_wr),
	.snd_rd           (snd_rd),
	.snd_wdata        (snd_wdata),
	.snd_rdata        (snd_rdata),
	.text_vram_addr   (text_vram_addr),
	.text_vram_data   (text_vram_data),
	.spr_vram_addr    (spr_vram_addr),
	.spr_vram_data    (spr_vram_data),
	.main_shared_addr (main_shared_addr),
	.main_shared_cs   (main_shared_cs),
	.main_shared_we   (main_shared_we),
	.main_shared_wdata(main_shared_wdata),
	.main_shared_rdata(main_shared_rdata),
	.bg_vram_addr     (bg_vram_addr),
	.bg_vram_data     (bg_vram_data),
	.fg_vram_addr     (fg_vram_addr),
	.fg_vram_data     (fg_vram_data),
	.mg_vram_addr     (mg_vram_addr),
	.mg_vram_data     (mg_vram_data),
	.pal_vram_addr    (pal_b_addr),
	.pal_vram_data    (pal_word_raw),
	.ss_bg            (ssb[4]),
	.ss_fg            (ssb[5]),
	.ss_pal           (ssb[6]),
	.ss_crtc          (ssb[9]),
	.cop_rom_addr     (game_sub_addr),
	.cop_rom_req      (game_sub_req),
	.cop_rom_rdata    (game_sub_data),
	.cop_rom_ready    (game_sub_ready),
	.cop_busy_out     (cop_busy_main),
	.crtc_layer_en_bg  (crtc_layer_en_bg),
	.crtc_layer_en_mg  (crtc_layer_en_mg),
	.crtc_layer_en_fg  (crtc_layer_en_fg),
	.crtc_layer_en_text(crtc_layer_en_text),
	.crtc_layer_en_spr (crtc_layer_en_spr),
	.crtc_flip_screen  (crtc_flip_screen),
	.crtc_scroll_bg_x  (crtc_scroll_bg_x), .crtc_scroll_bg_y(crtc_scroll_bg_y),
	.crtc_scroll_mg_x  (crtc_scroll_mg_x), .crtc_scroll_mg_y(crtc_scroll_mg_y),
	.crtc_scroll_fg_x  (crtc_scroll_fg_x), .crtc_scroll_fg_y(crtc_scroll_fg_y),
	.crtc_base_bg_x    (crtc_base_bg_x),   .crtc_base_bg_y  (crtc_base_bg_y),
	.crtc_base_mg_x    (crtc_base_mg_x),   .crtc_base_mg_y  (crtc_base_mg_y),
	.crtc_base_fg_x    (crtc_base_fg_x),   .crtc_base_fg_y  (crtc_base_fg_y),
	.crtc_base_txt_x   (crtc_base_txt_x),
	.crtc_base_txt_y   (crtc_base_txt_y),
	.gfx_bank          (gfx_bank),
	.dbg_irq_pending  (main_irq_pending_probe),
	.ss_workram       (ssb[0]),
	.ss_txt           (ssb[1]),
	.ss_scroll        (ssb[2]),
	.ss_spr           (ssb[7]),
	.ss_cpu           (ssb[8]),
	.ss_bank          (ssb[3]),
	.ss_copa          (ssb[11]),
	.ss_copr          (ssb[15]),
	.ss_cpu_reload    (ss_cpu_reload)
);

// ── Sub V30: NON esiste su Raiden II. BG/FG/PAL vivono nel main_top
// (porte cablate nell'istanza u_main qui sopra). Slot SS 9/11 terminati;
// porta ROM sub del bridge SDRAM libera (la riusera' il COP per le hitbox,
// pattern _old).
// Porta ROM sub del bridge = ora del COP (hitbox b100/b900), via u_sub_cache.
// game_sub_addr/req pilotati dall'istanza u_main (cop_rom_*).
assign sub_irq_pending_probe = 1'b0;   // (wire dichiarato nella sezione probe)
wire cop_busy_main;
// ssb[9] ora e' il CRTC (dentro u_main). Solo l'11 resta terminato.
// Lo slot 11 (su Raiden la work RAM del sub, qui inesistente) ospita ora gli
// ARRAY del COP; i suoi scalari stanno sul 15.

// Wires per palette overlay tap (sub rimosso: inerti)
wire [19:0] sub_dbg_cpu_addr     = 20'd0;
wire [15:0] sub_dbg_cpu_dout     = 16'd0;
wire  [1:0] sub_dbg_cpu_be       = 2'd0;
wire        sub_dbg_cpu_wr       = 1'b0;
wire        sub_dbg_palette_memrq = 1'b0;

///////////////////////   VIDEO   ///////////////////////////////

// Raiden II: 320x240 visibili @ 55.408 Hz. Pixel clock 8 MHz = clk_sys/12,
// HTotal 512, VTotal 282 (raiden2.cpp: set_raw(32MHz/4, 512,0,320, 282,0,240)).
// ⚠ Qui c'era scritto "6 MHz, HTotal 384, VTotal 263": e' LEGIONNAIRE. Da quel
// commento venivano anche H_TOTAL_RD/V_TOTAL_RD sbagliati piu' sotto.
wire ce_pix;
wire HBlank, VBlank, HSync, VSync, video_de;
wire [9:0] H_VIS_START_TOP;
wire [8:0] V_VIS_START_TOP;
wire [9:0] timing_hpos;
wire [9:0] timing_vpos;

Raiden2_video_timing u_video_timing (
	.clk        (clk_sys),
	.reset      (video_reset),
	.mode_60hz  (status[19]),
	.ce_pix     (ce_pix),
	.hpos       (timing_hpos),
	.vpos       (timing_vpos),
	.active_x   (render_x),
	.active_y   (render_y),
	.hblank     (HBlank),
	.vblank     (VBlank),
	.hsync      (HSync),
	.vsync      (VSync),
	.de         (video_de),
	.h_vis_start(H_VIS_START_TOP),
	.v_vis_start(V_VIS_START_TOP)
);

// ── Flip screen (CRTC reg 0x1A bit 0) ───────────────────────────────────────
// MAME: BIT(reg_1a, 0) → flip_screen. Arcade reale = monitor CRT capovolto;
// game scrive in VRAM convinto che lo schermo sia ruotato 180° → noi vediamo
// flippato finchè non invertiamo.
//
// Strategia:
//  - X flip: il read-side dei line_buffer (dentro tile_layer/text_renderer)
//    legge linebuf[hpos]. Sostituisco hpos con (319-hpos) → mostra mirror H.
//  - Y flip: la prefetch durante display vpos=N riempie il buffer per il
//    display vpos=N+1. In flip ON serve che mostri riga ROM (V_VISIBLE-1-(N+1))
//    = (V_VISIBLE-2-N). Sostituisco vpos del prefetch con (V_VISIBLE-2-vpos)
//    quando flip on, così target_y = V_VISIBLE-2-vpos + 1 = V_VISIBLE-1-vpos.
// SIM_FLIP: forza il Flip Screen a 1 per la simulazione, senza aspettare che il
// gioco legga il DIP e scriva il registro video. Inerte in Quartus (la macro
// non e' nel .qsf), come SIM_FORCE_DIP su Raiden 1.
`ifdef SIM_FLIP
wire        flip_screen = 1'b1;
`else
wire        flip_screen = map_ctrl_l0[5];
`endif
// Coordinate LOGICHE display (0..319, 0..239) dal timing 512x282 della _old:
// VISIBLE inizia a hpos=140 (38+102), vpos=32 (3+29).
// ⛔ NON ricablare questi due numeri: arrivano dal modulo di timing, che e'
// l'unica fonte. Quando erano localparam scritti a mano (140 e 32) ogni
// cambio di raster li lasciava indietro e il contenuto si distruggeva.
wire [9:0]  hpos_logic = timing_hpos - H_VIS_START_TOP;
wire [8:0]  vpos_logic = timing_vpos[8:0] - V_VIS_START_TOP;
// Blocco prefetch a 240 righe TRASCRITTO dalla _old (Raiden2.sv:1140-1163):
// lo slot di prefetch della PRIMA riga del frame e' SPOSTATO nel tardo
// vblank (raster 11): l'ISR IRQ4 del gioco (DMA 0x14 + write scroll) gira
// alle righe ~249-253 — prefetchando la riga 0 a raster 248 si accoppiava
// contenuto/scroll del vblank VECCHIO col frame nuovo → pop di 16px ai
// confini colonna. Lo slot originale a vpos_logic 239 e' inertizzato (300 ≥
// 240): conteggio toggle del double-buffer resta 240/frame (PARI, no
// flicker).
wire [8:0]  vpos_pf_src   = (timing_vpos == 10'd11)   ? 9'd239 :
                            (vpos_logic == 9'd239)    ? 9'd300 :
                                                        vpos_logic;
// ⛔ NIENTE mirror della vpos qui dentro. Il flip verticale si applica DENTRO
// ai renderer, alla sola riga di contenuto (`target_y`), e la vpos che esce da
// qui resta REALE. Due motivi, entrambi visti sul ferro:
//   - lo slot inertizzato qui sopra vale 300 (>= 240 = non prefetcha). Con il
//     mirror esterno diventava 239-300, negativo, avvolto a 9 bit a 451: lo
//     slot morto tornava vivo e prefetchava una riga a caso.
//   - specchiando il TEMPO invece del CONTENUTO, il prefetch e i suoi slot
//     speciali si sfasano rispetto alla copia della spriteram, e sfondo e
//     sprite si desincronizzano (Raiden 1, commit 0319db3: "ai tile layer va
//     la vpos REALE, il mirror della riga lo fa il layer").
// Read path (X): linebuf[hpos] su 320 colonne visibili.
wire [9:0]  hpos_for_read = flip_screen ? (10'd319 - hpos_logic) : hpos_logic;
// Lookahead dei tile layer: stesso ruolo di hpos_for_spr (il mixer campiona
// un clock dopo ce_pix). Senza, i tile escono indietro di un pixel.
wire [9:0] hpos_for_tile = flip_screen ? (hpos_for_read - 10'd1)
                                       : (hpos_for_read + 10'd1);


// ── Text layer renderer (8x8, 4bpp, 64x32 grid) — blocco della _old ─────────
// Raiden II: "char" = REGIONE PROPRIA 128KB, lineare (no descramble). MRA
// carica char a ioctl 0x100000..0x11FFFF. charrom BRAM 64Kw × 16-bit.
reg [9:0] hpos_prev;
always @(posedge clk_sys) if (ce_pix) hpos_prev <= timing_hpos;
wire layer_new_line = ce_pix && (timing_hpos == 10'd0) && (hpos_prev != 10'd0);
wire        arb_txt_req, arb_txt_valid;
wire [23:0] arb_txt_addr;
wire [31:0] arb_txt_data;

wire        text_opaque;
wire [10:0] text_pen;

wire        text_rom_dl_wr =
	ioctl_download && ioctl_wr && (ioctl_index == 16'd0) &&
	(ioctl_addr >= 27'h100000) && (ioctl_addr < 27'h120000);
// 17-bit address relativo alla region char (= [16:0] di 0x1xxxxx).
wire [16:0] text_rom_dl_offset = ioctl_addr[16:0];
Raiden2_text_renderer u_text (
	.clk          (clk_sys),
	.reset        (reset),
	.ce_pix       (ce_pix),
	.hpos         (hpos_for_read),
	.hpos_rd      (hpos_for_tile),
	.vpos         (vpos_logic),
	.flip_screen  (flip_screen),
	.de           (video_de),
	.layer_en     (map_ctrl_l0[3]),
	.scroll_x     (WIN_X),    // stessa finestra dei tile layer (0, 0)
	.scroll_y     (WIN_Y),
	.xoff         (osd_txt_xoff),
	.yoff         (osd_txt_yoff),
	.new_line     (layer_new_line),
	.vram_addr    (text_vram_addr),
	.vram_data    (text_vram_data),
	// char da SDRAM (porta r4 dell'arbitro, gfx_kind 4 -> TXT_BASE):
	// la charrom in BRAM costava 128 M10K per una ROM che la SDRAM aveva gia'.
	.rom_req      (arb_txt_req),
	.rom_addr     (arb_txt_addr),
	.rom_data     (arb_txt_data),
	.rom_valid    (arb_txt_valid),
	.opaque       (text_opaque),
	.pen_index    (text_pen)
);

// ── BG/MG/FG layer renderer (16x16, 4bpp, 32x32) — blocco della _old ─────
wire        bg_opaque, mg_opaque, fg_opaque;
wire [10:0] bg_pen, mg_pen, fg_pen;
wire [10:0] bg_vram_addr, mg_vram_addr, fg_vram_addr;
wire [15:0] bg_vram_data, mg_vram_data, fg_vram_data;

// new_line pulse: hpos passa da H_TOTAL-1 a 0

// Scroll CRTC (legionna_v.cpp tile_scroll_w): ram[0/1]→BG, [2/3]→MG, [4/5]→FG.
// Finestra Raiden II: set_raw(..., 282, 0, 30*8) → vbend=0, nessuna finestra.
localparam [15:0] WIN_X = 16'd0;
localparam [15:0] WIN_Y = 16'd0;
wire [15:0] map_xscroll_mg = crtc_scroll_mg_x;
wire [15:0] map_yscroll_mg = crtc_scroll_mg_y;
wire [15:0] bg_scroll_x = map_xscroll_l0 + WIN_X;
wire [15:0] bg_scroll_y = map_yscroll_l0 + WIN_Y;
wire [15:0] mg_scroll_x = map_xscroll_mg + WIN_X;
wire [15:0] mg_scroll_y = map_yscroll_mg + WIN_Y;
wire [15:0] fg_scroll_x = map_xscroll_l1 + WIN_X;
wire [15:0] fg_scroll_y = map_yscroll_l1 + WIN_Y;

// OSD trim per-layer: base 0 (i trim OSD arrivano con la CONF_STR Raiden II)
wire signed [9:0] osd_mg_xoff = 10'sd0;
wire signed [9:0] osd_mg_yoff = 10'sd0;

// Arbiter wires (BG + MG + FG)
wire        arb_bg_req,  arb_mg_req,  arb_fg_req;
wire [23:0] arb_bg_addr, arb_mg_addr, arb_fg_addr;
wire [31:0] arb_bg_data, arb_mg_data, arb_fg_data;
wire        arb_bg_valid, arb_mg_valid, arb_fg_valid;

// Raiden II BG: GFXDECODE "tiles" — back color = (tile>>12) | (0<<4) →
// palette 0x400-0x4FF. set_transparent_pen(15). Banco come bank<<19 sul
// byte offset (HAS_GFX_BANK=0).
Raiden2_tile_layer #(
	.COLOR_BASE   (11'h400),
	.HAS_TRANSP   (1),
	.HAS_GFX_BANK (0),
	.TILE_KIND    (3'd0),
	.MAP_HEIGHT_4 (0)
) u_bg (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_read), .hpos_rd(hpos_for_tile), .vpos(vpos_pf_src),
	.flip_screen(flip_screen),
	.de(video_de), .layer_en(map_ctrl_l0[0]),
	.new_line(layer_new_line),
	.scroll_x(bg_scroll_x), .scroll_y(bg_scroll_y),
	.xoff(osd_bg_xoff), .yoff(osd_bg_yoff),
	.gfx_bank({13'd0, gfx_bank[2:0]}),   // m_bg_bank (tile_bank_01_w) — reset 0
	.vram_addr(bg_vram_addr), .vram_data(bg_vram_data),
	.rom_req(arb_bg_req), .rom_addr(arb_bg_addr),
	.rom_data(arb_bg_data), .rom_valid(arb_bg_valid),
	.opaque(bg_opaque), .pen_index(bg_pen)
);

// Raiden II MG: share BG ROM, +0x1000 dal banco m_mid_bank (reset 1);
// color = (tile>>12) | (2<<4) → palette 0x600-0x6FF.
Raiden2_tile_layer #(
	.COLOR_BASE    (11'h600),
	.HAS_TRANSP    (1),
	.HAS_GFX_BANK  (0),
	.TILE_KIND     (3'd1),
	.TILE_CODE_OFS (13'h0000)
) u_mg (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_read), .hpos_rd(hpos_for_tile), .vpos(vpos_pf_src),
	.flip_screen(flip_screen),
	.de(video_de), .layer_en(map_ctrl_l0[1]),
	.new_line(layer_new_line),
	.scroll_x(mg_scroll_x), .scroll_y(mg_scroll_y),
	.xoff(osd_mg_xoff), .yoff(osd_mg_yoff),
	.gfx_bank({13'd0, gfx_bank[6:4]}),   // m_mid_bank = 1 | (d&2) — reset 1
	.vram_addr(mg_vram_addr), .vram_data(mg_vram_data),
	.rom_req(arb_mg_req), .rom_addr(arb_mg_addr),
	.rom_data(arb_mg_data), .rom_valid(arb_mg_valid),
	.opaque(mg_opaque), .pen_index(mg_pen)
);

// Raiden II FG: color = (tile>>12) | (1<<4) → palette 0x500-0x5FF; region
// tile unica (stesso decode DCBA di BG/MG), banco m_fg_bank (reset 6).
Raiden2_tile_layer #(
	.COLOR_BASE    (11'h500),
	.HAS_TRANSP    (1),
	.HAS_GFX_BANK  (0),
	.TILE_KIND     (3'd2),
	.MAP_HEIGHT_4  (0),
	.TILE_CODE_OFS (13'h0000),
	.PEN_ORDER     (0)
) u_fg (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_read), .hpos_rd(hpos_for_tile), .vpos(vpos_pf_src),
	.flip_screen(flip_screen),
	.de(video_de), .layer_en(map_ctrl_l0[2]),
	.new_line(layer_new_line),
	.scroll_x(fg_scroll_x), .scroll_y(fg_scroll_y),
	.xoff(osd_fg_xoff), .yoff(osd_fg_yoff),
	.gfx_bank({13'd0, gfx_bank[10:8]}),  // m_fg_bank = 4 | (d>>14) — reset 6
	.vram_addr(fg_vram_addr), .vram_data(fg_vram_data),
	.rom_req(arb_fg_req), .rom_addr(arb_fg_addr),
	.rom_data(arb_fg_data), .rom_valid(arb_fg_valid),
	.opaque(fg_opaque), .pen_index(fg_pen)
);

// ── Sprite ROM su DDR3 CON DECRYPT — blocco della _old ──────────────────────
// Raiden II sprite = 8MB (obj-1..4, ROM_REGION32_LE), MRA ioctl
// 0x600000..0xDFFFFF. La ROM e' CIFRATA (MAME init_raiden2 →
// raiden2_decrypt_sprites): l'algoritmo lavora su word da 32 bit, quindi si
// accumulano le due meta' a 16 bit da ioctl, si decifra, si scrivono entrambe.
localparam [27:0] SPR_DDR3_BASE = 28'h0000000;
wire [26:0] spr_dl_off = ioctl_addr - 27'h600000;
wire        spr_dl_sel = ioctl_download & (ioctl_index == 16'd0) &
                         (ioctl_addr >= 27'h600000) & (ioctl_addr < 27'hE00000);

localparam [27:0] OKI2_DDR_BASE = 28'h0800000;   // 8 MB: subito dopo gli sprite
wire        oki2_dl_sel  = ioctl_download & (ioctl_index == 16'd0) &
                           (ioctl_addr >= 27'hE00000) & (ioctl_addr < 27'hE40000);
wire [26:0] oki2_dl_off  = ioctl_addr - 27'hE00000;

reg  [27:0] spr_ddr_waddr;
reg  [15:0] spr_ddr_wdata;
reg         spr_ddr_we_req = 1'b0;
wire        spr_ddr_we_ack;
reg         spr_dl_wr_d = 1'b0;

reg         oki2_dl_wr_d = 1'b0;
reg  [15:0] dec_lo_q;          // meta' bassa in attesa della alta
reg  [26:0] dec_off_q;         // offset della word 32-bit (allineato a 4)
reg   [2:0] dec_pipe;          // shift-reg: latenza 2 del modulo di decrypt
reg  [31:0] dec_din_q;
reg  [21:0] dec_widx_q;
wire [31:0] dec_dout;
reg  [31:0] dec_plain;
reg   [1:0] wr_state;

raiden2_sprite_decrypt u_spr_dec (
	.clk  (clk_sys),
	.widx (dec_widx_q),
	.din  (dec_din_q),
	.dout (dec_dout)
);

always @(posedge clk_sys) begin
	spr_dl_wr_d  <= ioctl_wr & spr_dl_sel;
	oki2_dl_wr_d <= ioctl_wr & oki2_dl_sel;
	dec_pipe     <= {dec_pipe[1:0], 1'b0};

	if (ioctl_wr & spr_dl_sel & ~spr_dl_wr_d) begin
		if (~spr_dl_off[1]) begin
			dec_lo_q  <= ioctl_dout;                 // meta' bassa: si accumula
		end else begin
			dec_din_q  <= {ioctl_dout, dec_lo_q};    // {alta, bassa}
			// widx = indice word 32-bit nella region (8MB = 2M word: 22 bit).
			dec_widx_q <= spr_dl_off[23:2];
			dec_off_q  <= {spr_dl_off[26:2], 2'b00};
			dec_pipe   <= {dec_pipe[1:0], 1'b1};     // parte la pipeline
		end
	end

	// dec_pipe[1] alto = uscita del decrypt valida (2 stadi registrati)
	if (dec_pipe[1]) begin
		dec_plain <= dec_dout;
		wr_state  <= 2'd1;
	end

	// ROM OKI2: scrittura diretta (nessuna cifratura); finestre ioctl disgiunte.
	if (ioctl_wr & oki2_dl_sel & ~oki2_dl_wr_d & (wr_state == 2'd0)) begin
		spr_ddr_waddr  <= OKI2_DDR_BASE + {1'b0, oki2_dl_off};
		spr_ddr_wdata  <= ioctl_dout;
		spr_ddr_we_req <= ~spr_ddr_we_req;
	end

	case (wr_state)
		2'd1: begin   // scrive la meta' bassa
			spr_ddr_waddr  <= SPR_DDR3_BASE + {1'b0, dec_off_q};
			spr_ddr_wdata  <= dec_plain[15:0];
			spr_ddr_we_req <= ~spr_ddr_we_req;
			wr_state       <= 2'd2;
		end
		2'd2: if (spr_ddr_we_ack == spr_ddr_we_req) begin   // poi la alta
			spr_ddr_waddr  <= SPR_DDR3_BASE + {1'b0, dec_off_q} + 28'd2;
			spr_ddr_wdata  <= dec_plain[31:16];
			spr_ddr_we_req <= ~spr_ddr_we_req;
			wr_state       <= 2'd3;
		end
		2'd3: if (spr_ddr_we_ack == spr_ddr_we_req) wr_state <= 2'd0;
		default: ;
	endcase

	// NON usare 'reset': include ioctl_download e azzererebbe la FSM proprio
	// durante il download (stesso ragionamento del bridge SDRAM).
	if (bridge_reset) begin
		wr_state <= 2'd0;
		dec_pipe <= 3'd0;
	end
end

// DDR3 read (sprite fetch): bridge dal protocollo rom_req/rom_valid del renderer
wire        arb_spr_req;
wire [23:0] arb_spr_addr;
wire [31:0] arb_spr_data;
wire        arb_spr_valid;
reg  [27:0] spr_ddr_raddr;
reg         spr_ddr_rd_req = 1'b0;
wire        spr_ddr_rd_ack;
wire [31:0] spr_ddr_rdata;
reg  [1:0]  spr_rd_state = 2'd0;
reg         spr_rom_valid_r = 1'b0;
reg  [31:0] spr_rom_data_r;
always @(posedge clk_sys) begin
	spr_rom_valid_r <= 1'b0;
	case (spr_rd_state)
		2'd0: if (arb_spr_req) begin
			spr_ddr_raddr  <= SPR_DDR3_BASE + {4'd0, arb_spr_addr};
			spr_ddr_rd_req <= ~spr_ddr_rd_req;
			spr_rd_state   <= 2'd1;
		end
		2'd1: if (spr_ddr_rd_ack == spr_ddr_rd_req) begin
			spr_rom_data_r  <= spr_ddr_rdata;
			spr_rom_valid_r <= 1'b1;
			spr_rd_state    <= 2'd2;
		end
		2'd2: if (!arb_spr_req) spr_rd_state <= 2'd0;
	endcase
end
assign arb_spr_data  = spr_rom_data_r;
assign arb_spr_valid = spr_rom_valid_r;

// ─── OKI2: polling a byte verso la seconda porta DDR3 (stub finché l'audio
// Raiden II non porta il secondo jt6295: allora oki2_rom_addr verra' dal
// modulo audio) ─────────────────────────────────────────────────────────────
wire [18:0] oki2_rom_addr;   // pilotato dal modulo audio (secondo jt6295)
wire  [7:0] oki2_rom_data;
wire        oki2_rom_ok;
wire [27:0] oki2_ddr_raddr = OKI2_DDR_BASE + {9'd0, oki2_rom_addr};
reg  [27:0] oki2_addr_prev = '1;
reg         oki2_ddr_rd_req = 1'b0;
wire        oki2_ddr_rd_ack;
wire  [7:0] oki2_ddr_rdata;
reg   [7:0] oki2_rom_data_r;
reg         oki2_rom_ok_r;
reg         oki2_pending;
always @(posedge clk_sys) begin
	if (reset) begin
		oki2_addr_prev  <= '1;
		oki2_ddr_rd_req <= 1'b0;
		oki2_rom_ok_r   <= 1'b0;
		oki2_pending    <= 1'b0;
	end else begin
		if ((oki2_ddr_raddr != oki2_addr_prev) && !oki2_pending) begin
			oki2_addr_prev  <= oki2_ddr_raddr;
			oki2_ddr_rd_req <= ~oki2_ddr_rd_req;
			oki2_pending    <= 1'b1;
			oki2_rom_ok_r   <= 1'b0;
		end
		if (oki2_pending && (oki2_ddr_rd_ack == oki2_ddr_rd_req)) begin
			oki2_rom_data_r <= oki2_ddr_rdata;
			oki2_rom_ok_r   <= 1'b1;
			oki2_pending    <= 1'b0;
		end
	end
end
assign oki2_rom_data = oki2_rom_data_r;
assign oki2_rom_ok   = oki2_rom_ok_r;

// DDR bus interfaces (pattern Taito F2 ddr_if + ddr_mux)
ddr_if ddr_host();    // bus reale → pin DDRAM_* (game: sprite+rotate)
ddr_if ddr_spr();     // client A: sprite ROM
ddr_if ddr_rot();     // client B: rotate framebuffer
ddr_if ddr_ss();      // savestate client (memory_stream) → gate → pin

// ── Savestate bus (ssbus) + save_state_data ─────────────────────────────
// SS_IDX_* = indice univoco di ogni blocco di stato salvato.
localparam SS_IDX_WORKRAM = 0;   // main work RAM (ram_lo/hi) — contiene lo score
// slave: 0=workram,1=txt,2=scroll; 3=shared; 4=bg,5=fg,6=pal; 7=sprite;
// 8=V30 main regs; 9=V30 sub regs; 10=z80_ram; 11=Sub work RAM.
localparam SS_NSLAVES     = 16;  // 11 = array COP, 15 = scalari COP  // 12 = regs Z80 audio (T80s REG/DIR), 13 = shadow YM2151,
                                 // 14 = glue audio (ULTIMO: il commit fa da trigger al replay)
localparam SS_MS_COUNT    = 16;  // memory_stream COUNT (>= SS_NSLAVES, pot. di 2)

// ss_busy dichiarato sopra (vicino a paused_safe che lo usa)
ssbus_if ssbus();
ssbus_if ssb[SS_NSLAVES]();

// raiden2_ss_manager: coordinatore frame-aligned. ss_save/ss_load NON triggano
// il DMA direttamente (partirebbe a metà frame). Il manager mette in pausa PRIMA,
// aspetta paused_safe stabile (confine frame), POI pulsa read/write_start.
wire ss_slot_empty;   // 1 = l'ultimo load ha trovato uno slot mai scritto
wire ss_mgr_wr, ss_mgr_rd, ss_cpu_reload;   // ss_mgr_pause dichiarato sopra (blocco paused_safe)
raiden2_ss_manager u_ss_mgr (
	.clk           (clk_sys),
	.reset         (reset),
	.ss_save       (ss_save),
	.ss_load       (ss_load),
	.paused_safe   (paused_safe & cpus_ss_ready),   // cattura/load SOLO con entrambe le V30 a confine istruzione
	.ss_busy       (ss_busy),
	.slot_empty    (ss_slot_empty),
	.ss_pause      (ss_mgr_pause),
	.write_start   (ss_mgr_wr),
	.read_start    (ss_mgr_rd),
	.ss_cpu_reload (ss_cpu_reload)
);

// save_state_data: DMA stato ↔ DDR (regione SS3E000000, slot ss_slot).
// Trigger dal manager (frame-aligned), NON da ss_save/ss_load diretti.
save_state_data #(.COUNT(SS_MS_COUNT)) u_ss_data (
	.clk         (clk_sys),
	.reset       (reset),
	.ddr         (ddr_ss),
	.read_start  (ss_mgr_rd),
	.write_start (ss_mgr_wr),
	.index       (ss_slot),
	.busy        (ss_busy),
	.slot_empty  (ss_slot_empty),   // load su slot mai scritto: niente reload CPU
	.ssbus       (ssbus)
);

// ssbus_mux: multiplexa gli slave ssb[] (masters del mux) verso ssbus (slave).
ssbus_mux #(.COUNT(SS_NSLAVES)) u_ssbus_mux (
	.clk    (clk_sys),
	.masters(ssb),
	.slave  (ssbus)
);

wire        ss_hold, ss_ddr_grant;   // dal ss_ddr_gate (sotto); dichiarati qui per il mux

raiden2_ddr_mux u_ddr_mux (
	.clk     (clk_sys),
	.ss_hold (ss_hold),   // durante SS blocca l'emissione sprite/rotate alla sorgente
	.x       (ddr_host),
	.a       (ddr_spr),
	.b       (ddr_rot)
);

// ── Savestate DDR gating ────────────────────────────────────────────────
// Il gioco (ddr_host, sprite+rotate) e il savestate (ddr_ss) condividono i pin
// DDRAM_*. ss_ddr_gate instrada game↔ss su ss_busy, con drain per non troncare
// burst in volo. A SS idle: pin ← game (comportamento identico a prima).
// DDRAM_ADDR (29 bit) = ddr_host.addr[31:3], ESATTAMENTE come l'assegnazione
// originale (assign DDRAM_ADDR = ddr_host.addr[31:3]). Prima avevo troncato a
// [28:3] buttando i bit 31:29 → sprite ROM (0x30000000) con bit alto tagliato →
// sprite rotti. NON aggiungere zeri: addr è già l'indirizzo giusto per DDRAM_ADDR.
wire [28:0] game_DDRAM_ADDR = ddr_host.addr[31:3];
wire        ss_tx_inflight = ddr_ss.read | ddr_ss.write;

ss_ddr_gate #(.AW(29), .DRAIN_TH(3)) u_ss_ddr_gate (
	.clk             (clk_sys),
	.reset           (reset),
	.ss_busy         (ss_busy),
	.ss_tx_inflight  (ss_tx_inflight),
	// game (ddr_host): l'emissione sprite/rotate è già bloccata alla sorgente dal
	// raiden2_ddr_mux (ss_hold → x.read/write=0), quindi ddr_host.read/write sono già 0
	// durante SS. Il gate conta i beat dei burst già in volo prima di concedere.
	.game_burstcnt   (ddr_host.burstcnt),
	.game_addr       (game_DDRAM_ADDR),
	.game_rd         (ddr_host.read),
	.game_din        (ddr_host.wdata),
	.game_be         (ddr_host.byteenable),
	.game_we         (ddr_host.write),
	// savestate (ddr_ss)
	.ss_burstcnt     (ddr_ss.burstcnt),
	.ss_addr         (ddr_ss.addr[31:3]),
	.ss_rd           (ddr_ss.read),
	.ss_din          (ddr_ss.wdata),
	.ss_be           (ddr_ss.byteenable),
	.ss_we           (ddr_ss.write),
	// controller
	.DDRAM_BUSY      (DDRAM_BUSY),
	.DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.DDRAM_BURSTCNT  (DDRAM_BURSTCNT),
	.DDRAM_ADDR      (DDRAM_ADDR),
	.DDRAM_RD        (DDRAM_RD),
	.DDRAM_DIN       (DDRAM_DIN),
	.DDRAM_BE        (DDRAM_BE),
	.DDRAM_WE        (DDRAM_WE),
	.ss_hold         (ss_hold),
	.ss_ddr_grant    (ss_ddr_grant)
);

// rdata/busy: entrambi i client vedono il ritorno DDR. Il gate garantisce che
// solo il client attivo (ss_ddr_grant) abbia transazioni in volo.
assign ddr_host.rdata       = DDRAM_DOUT;
// rdata_ready al gioco solo quando NON è concesso al SS (i beat SS non vanno ai client).
assign ddr_host.rdata_ready = ss_ddr_grant ? 1'b0 : DDRAM_DOUT_READY;
// busy al gioco alto se: SS ha il grant, OPPURE ss_hold (fase di drain): così i client
// sprite/rotate stallano e non emettono → il bus si drena → il gate concede al SS.
assign ddr_host.busy        = (ss_ddr_grant | ss_hold) ? 1'b1 : DDRAM_BUSY;
assign ddr_ss.rdata         = DDRAM_DOUT;
assign ddr_ss.rdata_ready   = ss_ddr_grant ? DDRAM_DOUT_READY : 1'b0;
assign ddr_ss.busy          = ss_ddr_grant ? DDRAM_BUSY : 1'b1;

// I pin DDRAM_* passano per u_ddr_mux + u_ss_ddr_gate (sezione sopra).
ddram_sprite u_ddram_spr (
	.clk(clk_sys),
	.wraddr(spr_ddr_waddr), .din(spr_ddr_wdata), .we_req(spr_ddr_we_req), .we_ack(spr_ddr_we_ack),
	.rdaddr(spr_ddr_raddr), .dout(spr_ddr_rdata), .rd_req(spr_ddr_rd_req), .rd_ack(spr_ddr_rd_ack),
	.rdaddr2(oki2_ddr_raddr), .dout2(oki2_ddr_rdata), .rd2_req(oki2_ddr_rd_req), .rd2_ack(oki2_ddr_rd_ack),
	.ddr(ddr_spr)
);

// ── Sprite renderer (SEI252/RISE, blocco della _old) ────────────────────────
wire        spr_opaque;
wire [10:0] spr_pen;
wire  [1:0] spr_pri;
wire [10:0] spr_vram_addr_int;  // 512 entry × 4 word = 2048 word
wire [10:0] spr_vram_addr = spr_vram_addr_int;
wire [15:0] spr_vram_data;

// Lookahead +1 pixel SOLO per il read-side sprite (fix CRT spram latency-2,
// HW-ok): anticipando indirizzo e DE di 1, il dato campionato al tick e'
// quello del pixel CORRENTE — allineamento coi tile e colonna 0 coperta.
// In flip il pixel successivo dello schermo legge hpos-1 (mirror).
wire [9:0] hpos_for_spr = flip_screen ? (hpos_for_read - 10'd1)
                                      : (hpos_for_read + 10'd1);
wire       de_spr = (vpos_logic < 9'd240) && ((hpos_logic + 10'd1) < 10'd320);
Raiden2_sprite_renderer u_spr (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_spr), .vpos(vpos_logic), .flip_screen(flip_screen),
	.de(de_spr), .layer_en(map_ctrl_l0[4]),    // bit4 = sprite enable
	.new_line(layer_new_line),
	.xoff(osd_spr_xoff), .yoff(osd_spr_yoff),
	.spr_addr(spr_vram_addr_int), .spr_data(spr_vram_data),
	.rom_req(arb_spr_req), .rom_addr(arb_spr_addr),
	.rom_data(arb_spr_data), .rom_valid(arb_spr_valid),
	.opaque(spr_opaque), .pen_index(spr_pen), .pri_code(spr_pri)
);

// ── Tile ROM arbiter (BG/FG/Sprite; MG/text slot tied off) ──
tile_rom_arbiter u_arb (
	.clk(clk_sys), .reset(reset), .hblank(HBlank),
	.r0_req(arb_bg_req),  .r0_addr(arb_bg_addr),  .r0_data(arb_bg_data),  .r0_valid(arb_bg_valid),
	.r1_req(arb_mg_req),  .r1_addr(arb_mg_addr),  .r1_data(arb_mg_data),  .r1_valid(arb_mg_valid),  // MG (Raiden II)
	.r2_req(arb_fg_req),  .r2_addr(arb_fg_addr),  .r2_data(arb_fg_data),  .r2_valid(arb_fg_valid),
	.r3_req(1'b0), .r3_addr(24'd0), .r3_data(), .r3_valid(),                          // SPR ora su DDR3 (raiden2_sprite_rom_cache)
	.r4_req(arb_txt_req),  .r4_addr(arb_txt_addr),  .r4_data(arb_txt_data),  .r4_valid(arb_txt_valid),
	.tile_req(game_tile_req), .tile_addr(game_tile_addr), .tile_kind(game_tile_kind),
	.tile_data(game_tile_data), .tile_valid(game_tile_valid)
);

// Pixel pipeline MAME RAIDEN (raiden.cpp:286-358 draw_sprites + 361-397 update):
//   Render order MAME: BG (priority 1) → FG (priority 2) → Text (priority 4)
//   Sprite con pri_mask:
//     pri=0 → SKIP (gestito nel renderer, opaque=0)
//     pri=1 → mask = GFX_PMASK_4 | GFX_PMASK_2 → sotto FG e Text → SOPRA SOLO BG
//     pri=2,3 → mask = GFX_PMASK_4 → sotto Text only → SOPRA BG e FG
// Nota: nessun sprite "above all" (sopra Text). Text è sempre sopra tutti gli sprite.
// PROBE: latch quando le 2 CPU accettano IRQ (falling edge irq_pending).
// In fase backdrop bypassiamo palette per mostrare stato direttamente:
//   nero    = nessuno IRQ acceptato
//   rosso   = solo Main IRQ
//   verde   = solo Sub IRQ
//   giallo  = entrambi
wire main_irq_pending_probe;
wire sub_irq_pending_probe;
reg  sub_irq_prev, main_irq_prev;
reg  sub_irq_seen, main_irq_seen;
always @(posedge clk_sys) begin
	if (reset) begin
		sub_irq_seen <= 0; main_irq_seen <= 0;
		sub_irq_prev <= 0; main_irq_prev <= 0;
	end else begin
		sub_irq_prev  <= sub_irq_pending_probe;
		main_irq_prev <= main_irq_pending_probe;
		if (sub_irq_prev  && !sub_irq_pending_probe)  sub_irq_seen  <= 1;
		if (main_irq_prev && !main_irq_pending_probe) main_irq_seen <= 1;
	end
end
// ── Composizione raiden2_v (dalla _old): draw_raw + blend_layer con
// m_cur_spri = {0,1,2,3,-1} (raiden2.cpp:3165-3167). Dal FONDO in avanti:
//   spr0 → BG → spr1 → MG → spr2 → FG → spr3 → TEXT
// Dal davanti: TEXT, spr3, FG, spr2, MG, spr1, BG, spr0.
// Tutti i pen (tile/text/sprite) sono ASSOLUTI: renderer della _old.
wire [10:0] spr_pen_eff = spr_pen;
wire spr_pri0 = spr_opaque & (spr_pri == 2'd0);
wire spr_pri1 = spr_opaque & (spr_pri == 2'd1);
wire spr_pri2 = spr_opaque & (spr_pri == 2'd2);
wire spr_pri3 = spr_opaque & (spr_pri == 2'd3);

// ── COMPOSITORE — modulo raiden2_mixer ──────────────────────────────────
// Estratto in un modulo perche' era DUPLICATO qui e nel banco di simulazione
// (la copia del banco decodificava la palette a 444 invece di 555: divergenza
// rimasta invisibile finche' non ho provato a validare i colori).
// Implementa il modello MAME per intero: fondale nero, poi gli 8 livelli dal
// FONDO in avanti, e un indice in tabella si MESCOLA al 50% con l'accumulato
// invece di sostituirlo. Cosi' due trasparenze sovrapposte si compongono.
// Provato contro un modello scritto dal codice MAME: sim/tb_mixer.cpp,
// 20000 pixel, ZERO differenze (1957 casi con piu' trasparenze sovrapposte).
// L'uscita e' combinatoria dall'accumulatore: stessa latenza del compositore
// precedente, dove video_r era un wire — nessuna compensazione sui sincronismi.
wire [7:0] video_r, video_g, video_b;
raiden2_mixer u_mixer (
	.clk        (clk_sys),
	.ce_pix     (ce_pix),
	.spr_pen    (spr_pen),
	.spr_pri0   (spr_opaque & (spr_pri == 2'd0)),
	.spr_pri1   (spr_opaque & (spr_pri == 2'd1)),
	.spr_pri2   (spr_opaque & (spr_pri == 2'd2)),
	.spr_pri3   (spr_opaque & (spr_pri == 2'd3)),
	.bg_pen     (bg_pen),   .bg_opaque  (bg_opaque),
	.mg_pen     (mg_pen),   .mg_opaque  (mg_opaque),
	.fg_pen     (fg_pen),   .fg_opaque  (fg_opaque),
	.text_pen   (text_pen), .text_opaque(text_opaque),
	.video_de   (video_de),
	.pal_addr   (pal_b_addr),
	.pal_word   (pal_word_raw),
	.rgb_r      (video_r), .rgb_g (video_g), .rgb_b (video_b)
);

assign CLK_VIDEO = clk_sys;

// Pause overlay: dim video + logo 48x48 al centro durante pausa.
// OSD "Clean Pause" (status[18]): ON=video raw senza addon, OFF=overlay attivo.
// Output su bus intermedi av_* (poi H-Shift/V-Shift/H-Size → VGA_*).
wire [7:0] av_r, av_g, av_b;
pause_overlay u_pause_ovl (
	.clk         (clk_sys),
	.pause       (pause),
	.clean       (status[18]),
	.vblank      (VBlank),
	.rotate_en   (rotate_en),
	.render_x_in (render_x[8:0]),
	.render_y_in (render_y),
	.rgb_r_in    (video_r),
	.rgb_g_in    (video_g),
	.rgb_b_in    (video_b),
	.rgb_r_out   (av_r),
	.rgb_g_out   (av_g),
	.rgb_b_out   (av_b)
);

// ── CRT Adjust (modulo unificato, copia byte-identica del repo canonico) ────
// Sostituisce lo schema vecchio (analog_hsize + due shift register SUI SYNC per
// H-Shift e V-Shift). Regole: MiSTer_Discovery_Docs/17.
// Cosa non andava, punto per punto:
//   1.7  il generatore di lettura aveva base 64 in QUARTI = 16 cicli per pixel,
//        cioe' un core a 6 MHz. Noi siamo a 8 MHz = 12 cicli/pixel: base 96 in
//        OTTAVI (passo 1/96 = 1.04%).
//   1.3  si resettava sull'HSync SPOSTATO invece che su hs_ref_out del modulo
//        -> scrittura, contatore di lettura e ritmo esterno non ripartivano
//        sullo stesso fronte: lo shrink derivava di fase e DESINCRONIZZAVA.
//   1.10 la finestra DE dell'OSD era ancorata ai contatori NATIVI. Il documento
//        lo chiama "Caso Raiden 2026-08-22": su PVM da' nero e sync perso gia'
//        a +/-1, il Cabinet lo nasconde perche' i tempi di riga restano nativi.
//        Forma giusta: de_osd = ~str_hb & ~str_vb, solo dalle uscite del modulo.
//   sez.3 l'H-Position avvolgeva a 128 su una lista di 97 voci: la voce "-1"
//        valeva -32 px, quindi da 0 a -1 l'immagine SALTAVA di 32 pixel.
// L'H-Shift sui sync e' stato RIMOSSO (come su Raiden 1): sposta il sync e fa
// perdere l'aggancio; l'H-Position sposta il CONTENUTO e fa lo stesso lavoro.
localparam int H_TOTAL_RD = 473;   // riga intera di QUESTO core
// Frame intero di QUESTO core, per lo shift register del V-Shift. E' il raster
// NATIVO (282); nel modo 60 Hz il campo e' 260 e il registro resta piu' lungo
// del necessario: funziona, ma il tap NEGATIVO del V-Shift e' fuori scala di
// 22 righe (limite noto del modulo, docs 17 §2).
localparam int V_TOTAL_RD = 282;

// Off = bypass puro. Gatato su scandoubler e scaler (regola 1.6): con lo
// scandoubler il CE raddoppia e la base del generatore non sarebbe piu' valida.
wire crt_adj_on = status[79] & ~(|status[7:5]) & ~forced_scandoubler;

// H-Size: 0 = nessuna scala, +1..+15 allarga, -1..-16 stringe.
reg signed [4:0] hsize_s;
always @(posedge clk_sys) if (ce_pix) hsize_s <= crt_adj_on ? $signed(status[66:62]) : 5'sd0;

// H-Position: sposta il CONTENUTO nel line buffer, sync nativo (HPOS_MODE 1).
// La lista dell'OSD ha 97 voci (0, +1..+48, -48..-1) e il menu salva l'INDICE:
// l'avvolgimento va fatto sulla LUNGHEZZA DELLA LISTA, non a 128.
// V-Size: il modulo usa +N = immagine piu' BASSA, quindi si nega: cosi' il
// "+" dell'OSD vuol dire piu' ALTA, coerente con l'H-Size dove "+" = piu'
// grande.
// ESTESO CON SEGNO: il campo OSD e' a complemento a due (1111 = -1, non +15).
// Con lo zero-extension il "-1" diventava +15 -> 45 righe: il lato negativo
// chiudeva l'immagine e desincronizzava, mentre il "+" sembrava funzionare.
//
// LA SCALA DIPENDE DAL MODO, perche' i due modi hanno fisiche diverse.
//  PVM (retimer): 1 RIGA per scatto. Il televisore non reagisce a una
//    percentuale ma a due numeri ASSOLUTI: il tetto di aggancio orizzontale
//    (~16,3 kHz, misurato: Raiden 1 molla a 274 righe, Raiden II a 294 —
//    stessa frequenza, conteggi diversi) e le ~285 righe/campo del
//    discriminatore 525/625, oltre le quali il TV ripresetta la deflessione
//    verticale sulla famiglia 625 e l'immagine cala del 17-25% in un colpo
//    solo, con il sync fermo e senza tagliare niente.
//    Raiden II parte da 282 righe, TRE righe sotto quel confine: col vecchio
//    passo da 3 righe il primo scatto ci saltava oltre di netto e il quarto
//    desincronizzava. A 1 riga per scatto: -1/-2 restano nella zona fine
//    (283, 284), -3 entra di proposito nella zona schiacciata, e da -4 in
//    poi si rifinisce dentro di essa.
//  Cabinet (ottico): timing nativo, nessuna frequenza in ballo e nessun
//    precipizio -> resta a 3 righe per scatto, che e' la corsa che serve.
wire signed [5:0] vsz_step = $signed({{2{status[95]}}, status[95:92]});
wire signed [5:0] vsz_lines = status[96] ? -(vsz_step + (vsz_step <<< 1))
                                         : -vsz_step;
reg  signed [5:0] crt_vsize_d;
reg               crt_vsmode_d;
always @(posedge clk_sys) if (ce_pix) begin
	crt_vsize_d  <= crt_adj_on ? vsz_lines : 6'sd0;
	crt_vsmode_d <= status[96];
end

reg [6:0] hsize_hoff_d;
always @(posedge clk_sys) if (ce_pix) hsize_hoff_d <= crt_adj_on ? status[104:98] : 7'd0;
wire signed [8:0] hsize_hoffset = (hsize_hoff_d <= 7'd48)
	? $signed({2'b0, hsize_hoff_d})
	: $signed({2'b0, hsize_hoff_d}) - 9'sd97;

// V-Shift: lo fa il modulo (voffset), non piu' uno shift register sul VSync.
wire line_tick = ce_pix && (timing_hpos == 10'(H_TOTAL_RD - 1));
reg signed [5:0] osd_vga_vshift_d;
always @(posedge clk_sys) if (line_tick) osd_vga_vshift_d <= crt_adj_on ? $signed(status[61:56]) : 6'sd0;

// Generatore del clock enable di LETTURA. Base = (clk/pixel) x 8 = 12 x 8 = 96
// ottavi; hsize sposta di un ottavo per scatto (1.04%).
// REGOLA CHIAVE: si resetta sul fronte di hs_ref_out del modulo, MAI sull'HSync.
wire hs_ref;
reg  hs_ref_d;
always @(posedge clk_sys) hs_ref_d <= hs_ref;
wire hs_ref_rise = hs_ref & ~hs_ref_d;
// Generatore di lettura STANDARD: base 104 = 13 clk/px x 8 (ottavi).
// ⛔ La base deve seguire il pixel clock VERO del core: se il lato lettura gira
// piu' veloce della scrittura la finestra sfora la riga e H-Position e V-Size
// impazziscono (documentato su DenjinMakai, stesso modulo).
wire [7:0] rd_period = 8'd104 + {{3{hsize_s[4]}}, hsize_s};  // 13 clk/px x 8
reg  [7:0] rd_acc;
wire rd_tick = (rd_acc + 8'd8) >= {1'b0, rd_period};
always @(posedge clk_sys) begin
	if      (hs_ref_rise) rd_acc <= 8'd0;
	else if (rd_tick)     rd_acc <= rd_acc + 8'd8 - {1'b0, rd_period};
	else                  rd_acc <= rd_acc + 8'd8;
end

wire rd_ce = crt_adj_on ? rd_tick : vz_ce;   // a valle del V-Size tutto dai _vz

// ── V-Size PRIMA di crt_adjust (regola: chi allarga/stringe la finestra
// verticale viene per primo e rigenera il PROPRIO vb, che alimenta lo stadio
// dopo). Con active=0 il modulo e' bypass puro: uscite = ingressi.
wire [7:0] vz_r, vz_g, vz_b;
wire       vz_hs, vz_vs, vz_de, vz_vb, vz_ce;
crt_vsize #(
	// Parametri come DenjinMakai, che usa lo stesso modulo con un raster
	// della stessa famiglia. LINE_PX = larghezza ATTIVA (slot del ring per riga).
	.RING_LINES (46),          // >= |vsize|*2 + 4 = 46 per vsize max 21
	.LINE_PX    (320),
	// Limiti ASSOLUTI del televisore, in cicli di clk_sys (96 MHz) per riga
	// d'uscita. Sostituiscono il vecchio tetto cieco a +/-21 righe, che era
	// un delta e quindi non poteva sapere dove si finiva davvero.
	//   96e6 / 16,20 kHz = 5926 -> shrink fermo a 292 righe (16,17 kHz),
	//                              appena sotto il desync misurato a 294.
	//   96e6 / 15,12 kHz = 6351 -> enlarge fermo a 273 righe. Non e' la banda
	//                              del CRT a fermarlo ma il FRONT PORCH: a 272
	//                              righe e' finito (V_FP 10) e la riga 240
	//                              comincia a non uscire piu' (misurato).
	// Frame Raiden II = 282 x 6149 = 1.734.018 clk, quindi la corsa utile e'
	// 273..292 righe: ne' il desync ne' il taglio in basso sono piu'
	// raggiungibili per costruzione.
	.LINE_CLK_MIN (5926),
	.LINE_CLK_MAX (6351)
) u_crt_vsize (
	.clk       (clk_sys),
	.pxl_cen   (ce_pix),
	.active    (crt_adj_on),
	.tube_mode (crt_vsmode_d),
	.vsize     (crt_vsize_d),
	.r_in      (av_r), .g_in (av_g), .b_in (av_b),
	.hs_in     (HSync),
	.vs_in     (VSync),
	.de_in     (video_de),
	.vb_in     (VBlank),          // VBlank VERO, mai il blank combinato
	.r_out     (vz_r), .g_out (vz_g), .b_out (vz_b),
	.hs_out    (vz_hs),
	.vs_out    (vz_vs),
	.de_out    (vz_de),
	.vb_out    (vz_vb),
	.ce_out    (vz_ce)
);

wire [7:0] str_r, str_g, str_b;
wire       str_hs, str_vs, str_hb, str_vb;
crt_adjust #(
	.VTOTAL    (V_TOTAL_RD),
	.HTOTAL    (H_TOTAL_RD),
	// CONTENTSHIFT (1): Raiden II ha 320 pixel attivi su 512, largo e centrato
	// -> il contenuto ha margine da entrambi i lati e non esce dalla finestra.
	// SYNCSHIFT (0) serve ai giochi stretti ancorati a un lato (256 attivi).
	.HPOS_MODE (1)
) u_crt_adjust (
	.clk      (clk_sys),
	.pxl_cen  (vz_ce),            // dal V-Size, non il CE nativo
	.pxl2_cen (rd_ce),
	.active   (crt_adj_on),
	.hsize    (hsize_s),
	.hoffset  (hsize_hoffset),
	.voffset  (osd_vga_vshift_d),
	.r_in     (vz_r), .g_in (vz_g), .b_in (vz_b),
	.hs_in    (vz_hs),
	.vs_in    (vz_vs),
	.hb_in    (~vz_de),
	.vb_in    (vz_vb),           // il vb RIGENERATO dal V-Size, mai quello nativo
	.r_out    (str_r), .g_out (str_g), .b_out (str_b),
	.hs_out   (str_hs), .vs_out (str_vs),
	.hb_out   (str_hb), .vb_out (str_vb),
	.hs_ref_out (hs_ref)
);

// Finestra DE: SOLO dalle uscite del modulo (regola 1.10).
wire de_osd = ~str_hb & ~str_vb;

// ── Allineamento RGB/DE al confine del pixel ────────────────────────────────
// Il mixer non presenta il pixel all'inizio del suo periodo: latcha quando
// l'accumulazione dei layer e' finita (8 passi + 2 di pipeline), cioe' intorno
// all'11esimo ciclo su 13. Il VGA_DE invece apre e chiude ESATTAMENTE ai
// confini del pixel. Chi campiona una volta per pixel sul fronte di ce_pix
// (HDMI, e lo scaler) non se ne accorge; il DAC analogico campiona in continuo
// e vede il disallineamento:
//   - a destra il dato della colonna 319 arriva quando il DE sta gia' per
//     chiudere -> la colonna si vede per pochi tredicesimi = PIXEL MANGIATO
//   - a sinistra, per gli stessi cicli, il pixel 0 mostra ancora la coda della
//     riga precedente, che cambia mentre il fondo scorre = COLONNA CHE SI MUOVE
// Rimedio (lo stesso gia' usato nel renderer sprite per il suo glitch
// sub-pixel): si latcha l'uscita E il DE insieme su ce_pix. Cosi' cambiano
// tutti e due esattamente al confine del pixel e restano allineati fra loro.
// L'immagine trasla di un pixel intero e uniforme, senza mangiarne nessuno.
// Nel registro ci vanno ANCHE i sync: se ci passano solo RGB e DE, quei due
// escono spostati di un pixel rispetto a HS/VS e su analogico l'immagine
// trasla a destra di 1 px (in TATE: sale, e il bordo alto finisce dietro la
// maschera). L'HDMI non lo vede perche' si aggancia al DE, non al sync.
reg [7:0] pxo_r, pxo_g, pxo_b;
reg       pxo_de, pxo_hs, pxo_vs;
always @(posedge clk_sys) if (ce_pix) begin
	pxo_r  <= av_r;
	pxo_g  <= av_g;
	pxo_b  <= av_b;
	pxo_de <= ~(HBlank | VBlank);
	pxo_hs <= HSync;
	pxo_vs <= VSync;
end

// Uscita analogica: Adjust On -> dal modulo (rigenera hb/vb allineati ai suoi
// dati); Off -> percorso nativo con RGB e DE allineati al pixel.
assign VGA_R  = crt_adj_on ? str_r  : pxo_r;
assign VGA_G  = crt_adj_on ? str_g  : pxo_g;
assign VGA_B  = crt_adj_on ? str_b  : pxo_b;
assign VGA_HS = crt_adj_on ? str_hs : pxo_hs;
assign VGA_VS = crt_adj_on ? str_vs : pxo_vs;
assign CE_PIXEL = rd_ce;

// Aspect ratio: Original = 4:3 arcade display, Full Screen = 0:0.
// Quando ruota (TATE) swap ARX/ARY: la scena è già ruotata dal framebuffer
// dell'HPS scaler → da 4:3 landscape diventa 3:4 portrait.
wire [11:0] arx = (!ar) ? (rotate_en ? 12'd3 : 12'd4) : (ar - 1'd1);
wire [11:0] ary = (!ar) ? (rotate_en ? 12'd4 : 12'd3) : 12'd0;

// Integer scaling forzato: Narrower HV-Integer (default), V-Integer, HV-Integer.
// Normal scaling rimosso perché senza setup utente preciso dà sempre risultato sbagliato.
video_freak video_freak
(
	.CLK_VIDEO(clk_sys),
	.CE_PIXEL(rd_ce),
	.VGA_VS(VSync),
	.HDMI_WIDTH(HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE(VGA_DE),
	.VIDEO_ARX(VIDEO_ARX),
	.VIDEO_ARY(VIDEO_ARY),
	.VGA_DE_IN(crt_adj_on ? de_osd : pxo_de),
	.ARX(arx),
	.ARY(ary),
	.CROP_SIZE(12'd0),
	.CROP_OFF(5'd0),
	.SCALE((status[7:5] == 3'd0) ? 3'd0 :   // Normal (default)
	        (status[7:5] == 3'd1) ? 3'd1 :   // V-Integer
	        (status[7:5] == 3'd2) ? 3'd4 :   // HV-Integer
	                                3'd2)    // Narrower HV-Integer
);

// LED: blink during download
assign LED_USER = ioctl_download;

// ============================================================
// Screen rotation (TATE) - pattern Taito F2 (ddr_if + FIFO)
// ============================================================
// status[2:1]: 00=No rotate, 01=CCW (TATE), 10=CW. status[3]=Flip 180.
// screen_rotate snoopa VGA_* a CLK_VIDEO; FIFO 1024 entry assorbe i write;
// ddr_mux arbitra tra sprite (a) e rotate (b).
wire [1:0] rotate_sel = status[2:1];
wire rotate_en  = (rotate_sel != 2'd0);
wire rotate_ccw = (rotate_sel == 2'd1);
wire flip_180   = status[3];
wire video_rotated;

// VGA_SCALER deve restare 0: il CRT analogico non deve MAI cambiare routing
// quando attivi rotate. La rotazione HDMI e' gestita da screen_rotate via
// framebuffer HPS, NON tramite VGA_SCALER. Con VGA_SCALER legato a
// video_rotated, abilitare la rotazione dirottava anche l'uscita analogica
// (stesso bug corretto su Raiden 1, commit 31a4711; pattern di SkySmasher).
assign VGA_SCALER = 0;

wire [28:0] rot_addr;
wire [63:0] rot_data;
wire  [7:0] rot_be;
wire        rot_we;

screen_rotate u_screen_rotate
(
	.CLK_VIDEO     (clk_sys),
	.CE_PIXEL      (rd_ce),

	.VGA_R         (VGA_R),
	.VGA_G         (VGA_G),
	.VGA_B         (VGA_B),
	.VGA_HS        (VGA_HS),
	.VGA_VS        (VGA_VS),
	.VGA_DE        (VGA_DE),

	.rotate_ccw    (rotate_ccw),
	.no_rotate     (~rotate_en),
	.flip          (flip_180),
	.video_rotated (video_rotated),

	.FB_EN         (FB_EN),
	.FB_FORMAT     (FB_FORMAT),
	.FB_WIDTH      (FB_WIDTH),
	.FB_HEIGHT     (FB_HEIGHT),
	.FB_BASE       (FB_BASE),
	.FB_STRIDE     (FB_STRIDE),
	.FB_VBL        (FB_VBL),
	.FB_LL         (FB_LL),

	.DDRAM_CLK     (),
	.DDRAM_BUSY    (1'b0),         // FIFO assorbe (pattern Taito F2)
	.DDRAM_BURSTCNT(),
	.DDRAM_ADDR    (rot_addr),
	.DDRAM_DIN     (rot_data),
	.DDRAM_BE      (rot_be),
	.DDRAM_WE      (rot_we),
	.DDRAM_RD      ()
);

raiden2_rotate_fifo u_rot_fifo (
	.clk      (clk_sys),
	.rot_addr (rot_addr),
	.rot_data (rot_data),
	.rot_be   (rot_be),
	.rot_we   (rot_we),
	.ddr      (ddr_rot)
);

endmodule
