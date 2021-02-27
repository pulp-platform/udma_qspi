package qspi_pkg;
	// qspi structure
	typedef struct packed {
		logic sd0_o;
		logic sd0_oe;
		logic sd1_o;
		logic sd1_oe;
		logic sd2_o;
		logic sd2_oe;
		logic sd3_o;
		logic sd3_oe;
		logic csn0_o;
		logic csn1_o;
		logic csn2_o;
		logic csn3_o;
		logic sck_o;
	} qspi_to_pad_t;
	typedef struct packed {
		logic sd0_i;
		logic sd1_i;
		logic sd2_i;
		logic sd3_i;
	} pad_to_qspi_t;
endpackage