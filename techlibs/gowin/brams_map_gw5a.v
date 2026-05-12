`define DEF_FUNCS \
	function [255:0] init_slice_x8; \
		input integer idx; \
		integer i; \
		for (i = 0; i < 32; i = i + 1) begin \
			init_slice_x8[i*8+:8] = INIT[(idx * 32 + i) * 9+:8]; \
		end \
	endfunction \
	function [287:0] init_slice_x9; \
		input integer idx; \
		init_slice_x9 = INIT[idx * 288+:288]; \
	endfunction \

`define x8_width(width) (width / 9 * 8 + width % 9)
`define x8_rd_data(data) {1'bx, data[31:24], 1'bx, data[23:16], 1'bx, data[15:8], 1'bx, data[7:0]}
`define x8_wr_data(data) {data[34:27], data[25:18], data[16:9], data[7:0]}
`define addrbe(width, addr, w_be) (width < 18 ? addr :  width == 18 ? {addr[13:4], 2'b00, w_be[1:0]} : {addr[13:5], 1'b0, w_be[3:0]})


`define INIT(func) \
	.INIT_RAM_00(func('h00)), \
	.INIT_RAM_01(func('h01)), \
	.INIT_RAM_02(func('h02)), \
	.INIT_RAM_03(func('h03)), \
	.INIT_RAM_04(func('h04)), \
	.INIT_RAM_05(func('h05)), \
	.INIT_RAM_06(func('h06)), \
	.INIT_RAM_07(func('h07)), \
	.INIT_RAM_08(func('h08)), \
	.INIT_RAM_09(func('h09)), \
	.INIT_RAM_0A(func('h0a)), \
	.INIT_RAM_0B(func('h0b)), \
	.INIT_RAM_0C(func('h0c)), \
	.INIT_RAM_0D(func('h0d)), \
	.INIT_RAM_0E(func('h0e)), \
	.INIT_RAM_0F(func('h0f)), \
	.INIT_RAM_10(func('h10)), \
	.INIT_RAM_11(func('h11)), \
	.INIT_RAM_12(func('h12)), \
	.INIT_RAM_13(func('h13)), \
	.INIT_RAM_14(func('h14)), \
	.INIT_RAM_15(func('h15)), \
	.INIT_RAM_16(func('h16)), \
	.INIT_RAM_17(func('h17)), \
	.INIT_RAM_18(func('h18)), \
	.INIT_RAM_19(func('h19)), \
	.INIT_RAM_1A(func('h1a)), \
	.INIT_RAM_1B(func('h1b)), \
	.INIT_RAM_1C(func('h1c)), \
	.INIT_RAM_1D(func('h1d)), \
	.INIT_RAM_1E(func('h1e)), \
	.INIT_RAM_1F(func('h1f)), \
	.INIT_RAM_20(func('h20)), \
	.INIT_RAM_21(func('h21)), \
	.INIT_RAM_22(func('h22)), \
	.INIT_RAM_23(func('h23)), \
	.INIT_RAM_24(func('h24)), \
	.INIT_RAM_25(func('h25)), \
	.INIT_RAM_26(func('h26)), \
	.INIT_RAM_27(func('h27)), \
	.INIT_RAM_28(func('h28)), \
	.INIT_RAM_29(func('h29)), \
	.INIT_RAM_2A(func('h2a)), \
	.INIT_RAM_2B(func('h2b)), \
	.INIT_RAM_2C(func('h2c)), \
	.INIT_RAM_2D(func('h2d)), \
	.INIT_RAM_2E(func('h2e)), \
	.INIT_RAM_2F(func('h2f)), \
	.INIT_RAM_30(func('h30)), \
	.INIT_RAM_31(func('h31)), \
	.INIT_RAM_32(func('h32)), \
	.INIT_RAM_33(func('h33)), \
	.INIT_RAM_34(func('h34)), \
	.INIT_RAM_35(func('h35)), \
	.INIT_RAM_36(func('h36)), \
	.INIT_RAM_37(func('h37)), \
	.INIT_RAM_38(func('h38)), \
	.INIT_RAM_39(func('h39)), \
	.INIT_RAM_3A(func('h3a)), \
	.INIT_RAM_3B(func('h3b)), \
	.INIT_RAM_3C(func('h3c)), \
	.INIT_RAM_3D(func('h3d)), \
	.INIT_RAM_3E(func('h3e)), \
	.INIT_RAM_3F(func('h3f)),

module $__GOWIN_SP_ (...);

parameter INIT = 0;
parameter OPTION_RESET_MODE = "SYNC";

parameter PORT_A_WIDTH = 36;
parameter PORT_A_OPTION_WRITE_MODE = 0;
parameter PORT_A_WR_BE_WIDTH = 4;

input PORT_A_CLK;
input PORT_A_CLK_EN;
input PORT_A_WR_EN;
input PORT_A_RD_SRST;
input PORT_A_RD_ARST;
input [13:0] PORT_A_ADDR;
input [PORT_A_WR_BE_WIDTH-1:0] PORT_A_WR_BE;
input [PORT_A_WIDTH-1:0] PORT_A_WR_DATA;
output [PORT_A_WIDTH-1:0] PORT_A_RD_DATA;

`DEF_FUNCS

wire RST = OPTION_RESET_MODE == "SYNC" ? PORT_A_RD_SRST : PORT_A_RD_ARST;
wire [13:0] AD = `addrbe(PORT_A_WIDTH, PORT_A_ADDR, PORT_A_WR_BE);

generate

if (PORT_A_WIDTH < 9) begin

	wire [31:0] DI = `x8_wr_data(PORT_A_WR_DATA);
	wire [31:0] DO;

	assign PORT_A_RD_DATA = `x8_rd_data(DO);

	SP #(
		`INIT(init_slice_x8)
		.READ_MODE(1'b0),
		.WRITE_MODE(PORT_A_OPTION_WRITE_MODE),
		.BIT_WIDTH(`x8_width(PORT_A_WIDTH)),
		.BLK_SEL(3'b000),
		.RESET_MODE(OPTION_RESET_MODE),
	) _TECHMAP_REPLACE_ (
		.BLKSEL(3'b000),
		.CLK(PORT_A_CLK),
		.CE(PORT_A_CLK_EN),
		.WRE(PORT_A_WR_EN),
		.RESET(RST),
		.OCE(1'b1),
		.AD(AD),
		.DI(DI),
		.DO(DO),
	);

end else begin

	wire [35:0] DI = PORT_A_WR_DATA;
	wire [35:0] DO;

	assign PORT_A_RD_DATA = DO;

	SPX9 #(
		`INIT(init_slice_x9)
		.READ_MODE(1'b0),
		.WRITE_MODE(PORT_A_OPTION_WRITE_MODE),
		.BIT_WIDTH(PORT_A_WIDTH),
		.BLK_SEL(3'b000),
		.RESET_MODE(OPTION_RESET_MODE),
	) _TECHMAP_REPLACE_ (
		.BLKSEL(3'b000),
		.CLK(PORT_A_CLK),
		.CE(PORT_A_CLK_EN),
		.WRE(PORT_A_WR_EN),
		.RESET(RST),
		.OCE(1'b1),
		.AD(AD),
		.DI(DI),
		.DO(DO),
	);

end

endgenerate

endmodule


module $__GOWIN_DP_ (...);

parameter INIT = 0;
parameter OPTION_RESET_MODE = "SYNC";

parameter PORT_A_WIDTH = 18;
parameter PORT_A_OPTION_WRITE_MODE = 0;
parameter PORT_A_WR_BE_WIDTH = 4;

parameter PORT_B_WIDTH = 18;
parameter PORT_B_OPTION_WRITE_MODE = 0;
parameter PORT_B_WR_BE_WIDTH = 4;

input PORT_A_CLK;
input PORT_A_CLK_EN;
input PORT_A_WR_EN;
input PORT_A_RD_SRST;
input PORT_A_RD_ARST;
input [13:0] PORT_A_ADDR;
input [PORT_A_WR_BE_WIDTH-1:0] PORT_A_WR_BE;
input [PORT_A_WIDTH-1:0] PORT_A_WR_DATA;
output [PORT_A_WIDTH-1:0] PORT_A_RD_DATA;

input PORT_B_CLK;
input PORT_B_CLK_EN;
input PORT_B_WR_EN;
input PORT_B_RD_SRST;
input PORT_B_RD_ARST;
input [13:0] PORT_B_ADDR;
input [PORT_B_WR_BE_WIDTH-1:0] PORT_B_WR_BE;
input [PORT_A_WIDTH-1:0] PORT_B_WR_DATA;
output [PORT_A_WIDTH-1:0] PORT_B_RD_DATA;

`DEF_FUNCS

wire RSTA = OPTION_RESET_MODE == "SYNC" ? PORT_A_RD_SRST : PORT_A_RD_ARST;
wire RSTB = OPTION_RESET_MODE == "SYNC" ? PORT_B_RD_SRST : PORT_B_RD_ARST;
wire [13:0] ADA = `addrbe(PORT_A_WIDTH, PORT_A_ADDR, PORT_A_WR_BE);
wire [13:0] ADB = `addrbe(PORT_B_WIDTH, PORT_B_ADDR, PORT_B_WR_BE);

generate

// Codex round 19 follow-up (EXP_BB): broaden the DPB (8-bit-via-x8-packing)
// branch to also handle 9-bit memories. yosys's libmap pads our 8-bit
// `logic [7:0] sector_buffer [0:511]` to 9 bits (since brams.txt lists
// supported widths {1,2,4,9,18} — no native 8). Default condition was
// `< 9` (DPB only for narrow widths), so 9-bit memories went to DPX9B.
// Forcing `<= 9` puts our 8-bit-padded-to-9 memory through DPB instead.
// DPB packs 8 bits per cell (bit 8 unused). Diagnostic-only override (not
// gated): if EXP_BB hardware works while EXP_Z does not, the bug is
// specifically in DPX9B's narrow-write path on GW5A.
if (PORT_A_WIDTH <= 9 || PORT_B_WIDTH <= 9) begin

	wire [15:0] DIA = `x8_wr_data(PORT_A_WR_DATA);
	wire [15:0] DIB = `x8_wr_data(PORT_B_WR_DATA);
	wire [15:0] DOA;
	wire [15:0] DOB;

	assign PORT_A_RD_DATA = `x8_rd_data(DOA);
	assign PORT_B_RD_DATA = `x8_rd_data(DOB);

	DPB #(
		`INIT(init_slice_x8)
		.READ_MODE0(1'b0),
		.READ_MODE1(1'b0),
		.WRITE_MODE0(PORT_A_OPTION_WRITE_MODE),
		.WRITE_MODE1(PORT_B_OPTION_WRITE_MODE),
		.BIT_WIDTH_0(`x8_width(PORT_A_WIDTH)),
		.BIT_WIDTH_1(`x8_width(PORT_B_WIDTH)),
		.BLK_SEL_0(3'b000),
		.BLK_SEL_1(3'b000),
		.RESET_MODE(OPTION_RESET_MODE),
	) _TECHMAP_REPLACE_ (
		.BLKSELA(3'b000),
		.BLKSELB(3'b000),

		.CLKA(PORT_A_CLK),
		.CEA(PORT_A_CLK_EN),
		.WREA(PORT_A_WR_EN),
		.RESETA(RSTA),
		.OCEA(1'b1),
		.ADA(ADA),
		.DIA(DIA),
		.DOA(DOA),

		.CLKB(PORT_B_CLK),
		.CEB(PORT_B_CLK_EN),
		.WREB(PORT_B_WR_EN),
		.RESETB(RSTB),
		.OCEB(1'b1),
		.ADB(ADB),
		.DIB(DIB),
		.DOB(DOB),
	);

end else begin

	wire [17:0] DIA = PORT_A_WR_DATA;
	wire [17:0] DIB = PORT_B_WR_DATA;
	wire [17:0] DOA;
	wire [17:0] DOB;

	assign PORT_A_RD_DATA = DOA;
	assign PORT_B_RD_DATA = DOB;

	DPX9B #(
		`INIT(init_slice_x9)
		.READ_MODE0(1'b0),
		.READ_MODE1(1'b0),
		.WRITE_MODE0(PORT_A_OPTION_WRITE_MODE),
		.WRITE_MODE1(PORT_B_OPTION_WRITE_MODE),
		.BIT_WIDTH_0(PORT_A_WIDTH),
		.BIT_WIDTH_1(PORT_B_WIDTH),
		.BLK_SEL_0(3'b000),
		.BLK_SEL_1(3'b000),
		.RESET_MODE(OPTION_RESET_MODE),
	) _TECHMAP_REPLACE_ (
		.BLKSELA(3'b000),
		.BLKSELB(3'b000),

		.CLKA(PORT_A_CLK),
		.CEA(PORT_A_CLK_EN),
		.WREA(PORT_A_WR_EN),
		.RESETA(RSTA),
		.OCEA(1'b1),
		.ADA(ADA),
		.DIA(DIA),
		.DOA(DOA),

		.CLKB(PORT_B_CLK),
		.CEB(PORT_B_CLK_EN),
		.WREB(PORT_B_WR_EN),
		.RESETB(RSTB),
		.OCEB(1'b1),
		.ADB(ADB),
		.DIB(DIB),
		.DOB(DOB),
	);

end

endgenerate

endmodule


module $__GOWIN_SDP_ (...);

parameter INIT = 0;
parameter OPTION_RESET_MODE = "SYNC";

parameter PORT_R_WIDTH = 18;
parameter PORT_W_WIDTH = 18;
parameter PORT_W_WR_BE_WIDTH = 4;

input PORT_R_CLK;
input PORT_R_CLK_EN;
input PORT_R_RD_SRST;
input PORT_R_RD_ARST;
input [13:0] PORT_R_ADDR;
output [PORT_R_WIDTH-1:0] PORT_R_RD_DATA;

input PORT_W_CLK;
input PORT_W_CLK_EN;
input PORT_W_WR_EN;
input [13:0] PORT_W_ADDR;
input [PORT_W_WR_BE_WIDTH-1:0] PORT_W_WR_BE;
input [PORT_W_WIDTH-1:0] PORT_W_WR_DATA;

`DEF_FUNCS

wire RST = OPTION_RESET_MODE == "SYNC" ? PORT_R_RD_SRST : PORT_R_RD_ARST;
wire [13:0] ADW = `addrbe(PORT_W_WIDTH, PORT_W_ADDR, PORT_W_WR_BE);
wire WRE = PORT_W_CLK_EN & PORT_W_WR_EN;

// 2026-05-12 (Codex thread 019e1b34): family selector for SDP techmap.
// libmap may request widths from the 8-bit family (1/2/4/8/16/32 — emitted
// as SDPB native), the 9-bit family (9/18/36 — emitted as SDPX9B native),
// or the small shared subset (1/2/4 — historically goes through x8
// transcoding macros).
// The previous condition `< 9 ? SDPB : SDPX9B` correctly handled the case
// where brams.txt advertised `widths 9 18 36 per_port` only. After the
// Phase-11 patch advertised `widths 1 2 4 8 16 32 per_port` (8-bit family),
// libmap started requesting widths 8/16/32 which fell into the `>= 9` else
// branch and emitted SDPX9B cells with illegal BIT_WIDTH=32 — nextpnr
// rejected the resulting cells. New condition distinguishes the three
// families explicitly.
`define x8_native_width(w) (w == 8 || w == 16 || w == 32)
`define x9_family_width(w) (w == 9 || w == 18 || w == 36)

generate

if (`x8_native_width(PORT_W_WIDTH) && `x8_native_width(PORT_R_WIDTH)) begin
	// Native SDPB at widths 8/16/32.
	//
	// 2026-05-12 (Codex thread 019e1b34 Phase 14): when memory_widen_mixed
	// produces a 32-bit write port WITH BYTE ENABLES (PORT_W_WR_BE_WIDTH > 1),
	// we must configure SDPB ASYMMETRICALLY (BIT_WIDTH_0=8 byte write,
	// BIT_WIDTH_1=PORT_R_WIDTH word read).  SDPB has no native byte-enable
	// on a symmetric 32x32 configuration — a 32-bit write overwrites all 4
	// bytes, zeroing the 3 bytes outside the lane we intended to write.
	// That's the "51 00 00 49 42 FF FF" garbage we saw on OLED in Day-8.
	//
	// Asymmetric SDPB: the cell stores depth*8 bits with byte-wise writes
	// to an 11-bit address (byte_addr = word_addr * (R_W/8) + byte_idx) and
	// 32-bit (or wider) reads at a 9-bit address (word_addr).
	//
	// byte_idx priority-encoded from PORT_W_WR_BE (assumed one-hot per cycle —
	// memory_widen_mixed produces one-hot byte enables for original 8-bit
	// writers, which is our case).
	//
	// OCE tied GND to match Gowin reference (READ_MODE=0 bypass).
	// RESET tied GND to match Gowin reference (read output is combinational
	// off the BSRAM array, no register to reset).
	if (PORT_W_WIDTH == 32 && PORT_W_WR_BE_WIDTH == 4) begin
		// memory_widen_mixed output: 32-bit byte-enabled write, 32-bit read.
		wire [1:0] byte_idx = PORT_W_WR_BE[3] ? 2'd3 :
		                      PORT_W_WR_BE[2] ? 2'd2 :
		                      PORT_W_WR_BE[1] ? 2'd1 : 2'd0;
		wire WRE_any = PORT_W_CLK_EN & PORT_W_WR_EN & (|PORT_W_WR_BE);
		wire [13:0] ADA_byte = {PORT_W_ADDR[11:0], byte_idx};
		wire [7:0]  DI_byte = byte_idx == 2'd3 ? PORT_W_WR_DATA[31:24] :
		                      byte_idx == 2'd2 ? PORT_W_WR_DATA[23:16] :
		                      byte_idx == 2'd1 ? PORT_W_WR_DATA[15:8]  :
		                                         PORT_W_WR_DATA[7:0];
		wire [31:0] DO_word;
		assign PORT_R_RD_DATA = DO_word[PORT_R_WIDTH-1:0];

		SDPB #(
			`INIT(init_slice_x8)
			.READ_MODE(1'b0),
			.BIT_WIDTH_0(8),
			.BIT_WIDTH_1(PORT_R_WIDTH),
			.BLK_SEL_0(3'b000),
			.BLK_SEL_1(3'b000),
			.RESET_MODE(OPTION_RESET_MODE),
		) _TECHMAP_REPLACE_ (
			.BLKSELA(3'b000),
			.BLKSELB(3'b000),
			.CLKA(PORT_W_CLK),
			.CEA(WRE_any),
			.ADA(ADA_byte),
			.DI({24'b0, DI_byte}),
			.CLKB(PORT_R_CLK),
			.CEB(1'b1),         // Phase-12: defeat opt_dff CEB folding
			.RESET(1'b0),       // Phase-14: match Gowin RESET=GND (no read reset needed at READ_MODE=0)
			.OCE(1'b0),         // Phase-14: match Gowin OCE=GND
			.ADB(PORT_R_ADDR),
			.DO(DO_word),
		);
	end else begin
		// Symmetric SDPB at native widths 8/16/32 (no byte enables).
		wire [31:0] DI;
		wire [31:0] DO;
		if (PORT_W_WIDTH == 32) assign DI = PORT_W_WR_DATA;
		else if (PORT_W_WIDTH == 16) assign DI = {16'b0, PORT_W_WR_DATA};
		else /* 8 */ assign DI = {24'b0, PORT_W_WR_DATA};
		assign PORT_R_RD_DATA = DO[PORT_R_WIDTH-1:0];

		SDPB #(
			`INIT(init_slice_x8)
			.READ_MODE(1'b0),
			.BIT_WIDTH_0(PORT_W_WIDTH),
			.BIT_WIDTH_1(PORT_R_WIDTH),
			.BLK_SEL_0(3'b000),
			.BLK_SEL_1(3'b000),
			.RESET_MODE(OPTION_RESET_MODE),
		) _TECHMAP_REPLACE_ (
			.BLKSELA(3'b000),
			.BLKSELB(3'b000),
			.CLKA(PORT_W_CLK),
			.CEA(WRE),
			.ADA(ADW),
			.DI(DI),
			.CLKB(PORT_R_CLK),
			.CEB(1'b1),
			.RESET(1'b0),       // Phase-14: match Gowin RESET=GND
			.OCE(1'b0),         // Phase-14: match Gowin OCE=GND
			.ADB(PORT_R_ADDR),
			.DO(DO),
		);
	end

end else if (PORT_W_WIDTH < 9 || PORT_R_WIDTH < 9) begin
	// Narrow widths {1, 2, 4} — historical SDPB path with x8 transcoding.
	wire [31:0] DI = `x8_wr_data(PORT_W_WR_DATA);
	wire [31:0] DO;

	assign PORT_R_RD_DATA = `x8_rd_data(DO);

	SDPB #(
		`INIT(init_slice_x8)
		.READ_MODE(1'b0),
		.BIT_WIDTH_0(`x8_width(PORT_W_WIDTH)),
		.BIT_WIDTH_1(`x8_width(PORT_R_WIDTH)),
		.BLK_SEL_0(3'b000),
		.BLK_SEL_1(3'b000),
		.RESET_MODE(OPTION_RESET_MODE),
	) _TECHMAP_REPLACE_ (
		.BLKSELA(3'b000),
		.BLKSELB(3'b000),

		.CLKA(PORT_W_CLK),
		.CEA(WRE),
		.ADA(ADW),
		.DI(DI),

		.CLKB(PORT_R_CLK),
		.CEB(1'b1),
		.RESET(RST),
		.OCE(1'b1),
		.ADB(PORT_R_ADDR),
		.DO(DO),
	);

end else begin
	// 9-bit family widths {9, 18, 36} — SDPX9B native.
	wire [35:0] DI = PORT_W_WR_DATA;
	wire [35:0] DO;

	assign PORT_R_RD_DATA = DO;

	SDPX9B #(
		`INIT(init_slice_x9)
		.READ_MODE(1'b0),
		.BIT_WIDTH_0(PORT_W_WIDTH),
		.BIT_WIDTH_1(PORT_R_WIDTH),
		.BLK_SEL_0(3'b000),
		.BLK_SEL_1(3'b000),
		.RESET_MODE(OPTION_RESET_MODE),
	) _TECHMAP_REPLACE_ (
		.BLKSELA(3'b000),
		.BLKSELB(3'b000),

		.CLKA(PORT_W_CLK),
		.CEA(WRE),
		.ADA(ADW),
		.DI(DI),

		.CLKB(PORT_R_CLK),
		.CEB(PORT_R_CLK_EN),
		.RESET(RST),
		.OCE(1'b1),
		.ADB(PORT_R_ADDR),
		.DO(DO),
	);

end

endgenerate

endmodule
