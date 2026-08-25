// SPDX-License-Identifier: GPL-3.0-or-later
// Original: Martin Donlon (wickerwaka) - Arcade-TaitoF2 savestate. Modified by Umberto Parisi.
//
/*  This file is part of Raiden2_MiSTer.
    GPL-3.
    Original author: Martin Donlon (wickerwaka) — Arcade-TaitoF2 savestate system.
    Modified/adapted for BoogieWings by: Umberto Parisi (rmonc79)
*/

//============================================================================
//  BoogieWings Savestate — adaptor per le RAM inferite inline
//
//  Le RAM di BoogieWings sono `reg [7:0] mem[0:N]` con porta CPU separata in
//  byte (lo/hi). L'adaptor si interpone IN SERIE sulle linee della porta:
//  in modo normale passa i segnali del gioco; quando il ssbus accede a SS_IDX,
//  dirotta la porta verso il bus savestate (read/write).
//  NON aggiunge BRAM.
//
//  Riferimento: _reference/taitof2_ss/ram.sv (ram_ss_adaptor / m68k_ram_ss_adaptor)
//
//  Variante "byte-pair" (lo+hi a 16 bit, indirizzo word) per le RAM tipiche del
//  68K di BoogieWings: ram_lo/ram_hi, pf*_vram_lo/hi, sprite, palette, mirror.
//============================================================================

`timescale 1ns / 1ps

// Adaptor per una coppia di BRAM byte (lo+hi) indirizzate a word.
// WIDTHAD = bit dell'indirizzo word (es. 15 per 32K word, 12 per 4K word).
// Espone una porta WR a 16 bit sul ssbus; la lettura usa q (= {q_hi,q_lo}) della BRAM.
//
// Uso: collegare we_lo/we_hi/addr/wdata del gioco agli _in; gli _out vanno alla BRAM.
// q_in = dato letto dalla BRAM all'indirizzo addr_out (latenza 1 ck → read_delay).
module ss_ram16_adaptor #(
    parameter WIDTHAD = 15,
    parameter SS_IDX  = -1
) (
    input                    clk,

    // lato gioco (in)
    input                    we_lo_in,
    input                    we_hi_in,
    input      [WIDTHAD-1:0] addr_in,
    input      [15:0]        wdata_in,

    // lato BRAM (out)
    output                   we_lo_out,
    output                   we_hi_out,
    output     [WIDTHAD-1:0] addr_out,
    output     [15:0]        wdata_out,

    // dato letto dalla BRAM (q_hi,q_lo) all'indirizzo addr_out
    input      [15:0]        q_in,

    ssbus_if.slave           ssbus
);

wire sel = ssbus.access(SS_IDX);

assign addr_out  = sel ? ssbus.addr[WIDTHAD-1:0] : addr_in;
assign wdata_out = sel ? ssbus.data[15:0]        : wdata_in;
assign we_lo_out = sel ? ssbus.write             : we_lo_in;
assign we_hi_out = sel ? ssbus.write             : we_hi_in;

wire [31:0] SIZE = 32'd1 << WIDTHAD;

reg read_delay;
always @(posedge clk) begin
    ssbus.setup(SS_IDX, SIZE, 1);  // width 1 = 16 bit

    if (ssbus.access(SS_IDX)) begin
        if (ssbus.write) begin
            ssbus.write_ack(SS_IDX);
        end else if (ssbus.read) begin
            if (read_delay) begin
                ssbus.read_response(SS_IDX, {48'd0, q_in});
            end
            read_delay <= 1;
        end
    end else begin
        read_delay <= 0;
    end
end

endmodule


// Adaptor per una singola BRAM a WIDTH bit (es. pal_buf_top 24-bit, ace 16-bit,
// ram audio 8-bit). Una sola linea wren.
module ss_ram_adaptor #(
    parameter WIDTH   = 8,
    parameter WIDTHAD = 13,
    parameter SS_IDX  = -1
) (
    input                    clk,

    input                    wren_in,
    input      [WIDTHAD-1:0] addr_in,
    input      [WIDTH-1:0]   wdata_in,

    output                   wren_out,
    output     [WIDTHAD-1:0] addr_out,
    output     [WIDTH-1:0]   wdata_out,

    input      [WIDTH-1:0]   q_in,

    ssbus_if.slave           ssbus
);

wire sel = ssbus.access(SS_IDX);

assign addr_out  = sel ? ssbus.addr[WIDTHAD-1:0] : addr_in;
assign wdata_out = sel ? ssbus.data[WIDTH-1:0]   : wdata_in;
assign wren_out  = sel ? ssbus.write             : wren_in;

wire [31:0] SIZE = 32'd1 << WIDTHAD;

reg read_delay;
always @(posedge clk) begin
    ssbus.setup(SS_IDX, SIZE, ((WIDTH + 7) / 8) - 1);

    if (ssbus.access(SS_IDX)) begin
        if (ssbus.write) begin
            ssbus.write_ack(SS_IDX);
        end else if (ssbus.read) begin
            if (read_delay) begin
                ssbus.read_response(SS_IDX, { {(64-WIDTH){1'b0}}, q_in });
            end
            read_delay <= 1;
        end
    end else begin
        read_delay <= 0;
    end
end

endmodule
