// gw5a-specific DSP map.  Yosys's mul2dsp.v emits $__MULxxx blackbox cells per
// dsp_rules_gw5a (see synth_gowin.cc); this file binds those to the actual gw5a
// DSP primitives (MULT27X36, MULT12X12).
//
// Notes:
// - MULT12X12 / MULT27X36 are unsigned multipliers in their core; sign handling
//   is done by extending operands one bit upstream when A_SIGNED/B_SIGNED set.
// - All registers (AREG/BREG/PREG/OREG) parameterised to BYPASS so the cell is
//   purely combinational from yosys's perspective; nextpnr-himbaechel can later
//   retime if pipeline registers are present in the source.
// - CLK/CE/RESET ports are tied off; if a clocked variant is needed later we can
//   wire in a $dff in front and let mul2dsp's pipeline-stage option drive it.

module \$__MUL27X36 (input [A_WIDTH-1:0] A, input [B_WIDTH-1:0] B, output [Y_WIDTH-1:0] Y);
	parameter A_WIDTH = 27;
	parameter B_WIDTH = 36;
	parameter Y_WIDTH = 63;
	parameter A_SIGNED = 0;
	parameter B_SIGNED = 0;

	wire [26:0] a_ext = A_SIGNED ? {{(27-A_WIDTH){A[A_WIDTH-1]}}, A} : {{(27-A_WIDTH){1'b0}}, A};
	wire [35:0] b_ext = B_SIGNED ? {{(36-B_WIDTH){B[B_WIDTH-1]}}, B} : {{(36-B_WIDTH){1'b0}}, B};
	wire [62:0] dout;

	MULT27X36 #(
		.AREG_CLK("BYPASS"),
		.BREG_CLK("BYPASS"),
		.DREG_CLK("BYPASS"),
		.PADDSUB_IREG_CLK("BYPASS"),
		.PREG_CLK("BYPASS"),
		.PSEL_IREG_CLK("BYPASS"),
		.OREG_CLK("BYPASS"),
		.MULT_RESET_MODE("SYNC"),
		.DYN_P_SEL("FALSE"),
		.P_SEL(1'b0),
		.DYN_P_ADDSUB("FALSE"),
		.P_ADDSUB(1'b0)
	) __TECHMAP_REPLACE__ (
		.CLK(2'b00),
		.CE(2'b00),
		.RESET(2'b00),
		.A(a_ext),
		.B(b_ext),
		.D(26'b0),
		.PSEL(1'b0),
		.PADDSUB(1'b0),
		.DOUT(dout)
	);

	assign Y = dout[Y_WIDTH-1:0];
endmodule

module \$__MUL12X12 (input [A_WIDTH-1:0] A, input [B_WIDTH-1:0] B, output [Y_WIDTH-1:0] Y);
	parameter A_WIDTH = 12;
	parameter B_WIDTH = 12;
	parameter Y_WIDTH = 24;
	parameter A_SIGNED = 0;
	parameter B_SIGNED = 0;

	wire [11:0] a_ext = A_SIGNED ? {{(12-A_WIDTH){A[A_WIDTH-1]}}, A} : {{(12-A_WIDTH){1'b0}}, A};
	wire [11:0] b_ext = B_SIGNED ? {{(12-B_WIDTH){B[B_WIDTH-1]}}, B} : {{(12-B_WIDTH){1'b0}}, B};
	wire [23:0] dout;

	MULT12X12 #(
		.AREG_CLK("BYPASS"),
		.BREG_CLK("BYPASS"),
		.PREG_CLK("BYPASS"),
		.OREG_CLK("BYPASS"),
		.MULT_RESET_MODE("SYNC")
	) __TECHMAP_REPLACE__ (
		.CLK(2'b00),
		.CE(2'b00),
		.RESET(2'b00),
		.A(a_ext),
		.B(b_ext),
		.DOUT(dout)
	);

	assign Y = dout[Y_WIDTH-1:0];
endmodule
