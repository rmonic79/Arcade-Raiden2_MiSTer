// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// ss_null_slave — terminatore per gli slot ssbus rimasti liberi dopo lo
// smontaggio del sub-CPU di Raiden 1 (SS_IDX 3 shared, 9 sub-V30, 11 subram).
// Espone count 0 (stream vuoto) e acka qualunque accesso, cosi' il master
// del savestate non resta mai appeso su uno slot senza slave.

module ss_null_slave #(parameter SS_IDX = -1) (
	input  wire    clk,
	ssbus_if.slave ssbus
);

always @(posedge clk) begin
	ssbus.setup(SS_IDX, 32'd0, 1);
	if (ssbus.access(SS_IDX)) begin
		if (ssbus.write) ssbus.write_ack(SS_IDX);
		else if (ssbus.read) ssbus.read_response(SS_IDX, 64'd0);
	end
end

endmodule
