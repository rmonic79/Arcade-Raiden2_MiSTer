// SPDX-License-Identifier: GPL-3.0-or-later
// Original: Martin Donlon (wickerwaka) - Arcade-TaitoF2 (ddram). Modified by Umberto Parisi.
//
// raiden2_ddr_if.sv
// Bus DDRAM condiviso multi-client. Copia esatta del pattern taito_f2_baseline/rtl/ddram.sv.
//
// Client (modport from_host) chiede `acquire=1` quando vuole il bus.
// L'arbitro (ddr_mux) latcha chi acquire per primo, l'altro vede busy=1.
// Quando il client rilascia (acquire=0), l'altro può prendere.
//
// In Raiden:
//   a = sprite (read-only, fa burst quando rd_req != rd_ack)
//   b = rotate FIFO (write-only, fa burst quando FIFO non vuota)

interface ddr_if;
	logic        acquire;

	logic [31:0] addr;
	logic [63:0] wdata;
	logic [63:0] rdata;
	logic        read;
	logic        write;
	logic  [7:0] burstcnt;
	logic  [7:0] byteenable;
	logic        busy;
	logic        rdata_ready;

	modport to_host(
		output addr, wdata, read, write, burstcnt, byteenable, acquire,
		input rdata, busy, rdata_ready
	);

	modport from_host(
		output rdata, busy, rdata_ready,
		input addr, wdata, read, write, burstcnt, byteenable, acquire
	);
endinterface

module raiden2_ddr_mux(
	input clk,
	input ss_hold,   // 1 durante savestate: blocca l'emissione, client vedono busy

	ddr_if.to_host x,

	ddr_if.from_host a,
	ddr_if.from_host b
);

reg a_active = 0;

always_comb begin
	a.rdata = x.rdata;
	b.rdata = x.rdata;

	if (ss_hold) begin
		// SAVESTATE: NON emettere verso i pin; i client vedono busy=1 → si stallano
		// pulito (non emettono read/write, non aspettano risposte mai arrivate).
		x.addr = 32'd0;
		x.wdata = 64'd0;
		x.read = 1'b0;
		x.write = 1'b0;
		x.burstcnt = 8'd1;
		x.byteenable = 8'd0;
		a.busy = 1'b1;  a.rdata_ready = 1'b0;
		b.busy = 1'b1;  b.rdata_ready = 1'b0;
	end else if (a_active) begin
		x.addr = a.addr;
		x.wdata = a.wdata;
		x.read = a.read;
		x.write = a.write;
		x.burstcnt = a.burstcnt;
		x.byteenable = a.byteenable;

		a.busy = x.busy;
		a.rdata_ready = x.rdata_ready;
		a.rdata = x.rdata;

		b.busy = 1;
		b.rdata_ready = 0;
	end else begin
		x.addr = b.addr;
		x.wdata = b.wdata;
		x.read = b.read;
		x.write = b.write;
		x.burstcnt = b.burstcnt;
		x.byteenable = b.byteenable;

		b.busy = x.busy;
		b.rdata_ready = x.rdata_ready;
		b.rdata = x.rdata;

		a.busy = 1;
		a.rdata_ready = 0;
	end
end

// acquire: durante hold NON acquisiamo il bus (lasciamo che il SS lo prenda).
assign x.acquire = ss_hold ? 1'b0 : (a.acquire | b.acquire);

always_ff @(posedge clk) begin
	if (a.acquire & ~b.acquire) a_active <= 1;
	if (~a.acquire & b.acquire) a_active <= 0;
end

endmodule
