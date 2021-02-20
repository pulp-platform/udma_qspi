/* 
 * Copyright (C) 2018-2020 ETH Zurich, University of Bologna
 * Copyright and related rights are licensed under the Solderpad Hardware
 * License, Version 0.51 (the "License"); you may not use this file except in
 * compliance with the License.  You may obtain a copy of the License at
 *
 *                http://solderpad.org/licenses/SHL-0.51. 
 *
 * Unless required by applicable law
 * or agreed to in writing, software, hardware and materials distributed under
 * this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 *
 * Alfio Di Mauro <adimauro@iis.ee.ethz.ch>
 *
 */
 module udma_qspi_wrap
    import udma_pkg::udma_evt_t;
(
    input  logic         sys_clk_i,
    input  logic         periph_clk_i,
	input  logic         rstn_i,

	input  logic  [31:0] cfg_data_i,
	input  logic   [4:0] cfg_addr_i,
	input  logic         cfg_valid_i,
	input  logic         cfg_rwn_i,
	output logic         cfg_ready_o,
    output logic  [31:0] cfg_data_o,

    input  udma_evt_t    ch_events_i,
    output udma_evt_t    events_o,
    input  udma_evt_t    events_i,

    // UDMA CHANNEL CONNECTION
    UDMA_LIN_CH.tx_in    tx_ch[ 0:0],
    UDMA_LIN_CH.tx_in    cmd_ch[0:0],
    UDMA_LIN_CH.rx_out   rx_ch[ 0:0],

    // PAD SIGNALS CONNECTION
	BIPAD_IF.PERIPH_SIDE PAD_QSPI_SCLK,
	BIPAD_IF.PERIPH_SIDE PAD_QSPI_CSN0,
    BIPAD_IF.PERIPH_SIDE PAD_QSPI_CSN1,
    BIPAD_IF.PERIPH_SIDE PAD_QSPI_CSN2,
    BIPAD_IF.PERIPH_SIDE PAD_QSPI_CSN3,

    BIPAD_IF.PERIPH_SIDE PAD_QSPI_MOSI_IO0,
    BIPAD_IF.PERIPH_SIDE PAD_QSPI_MISO_IO1,
    BIPAD_IF.PERIPH_SIDE PAD_QSPI_IO2,
    BIPAD_IF.PERIPH_SIDE PAD_QSPI_IO3

);

import udma_pkg::TRANS_SIZE;     
import udma_pkg::L2_AWIDTH_NOAL; 

logic pad_qspi_mosi_io0_oe;
logic pad_qspi_miso_io1_oe;
logic pad_qspi_io2_oe     ;     
logic pad_qspi_io3_oe     ;     

udma_spim_top #(
    .L2_AWIDTH_NOAL      ( L2_AWIDTH_NOAL        ),
    .TRANS_SIZE          ( TRANS_SIZE            ),
    .REPLAY_BUFFER_DEPTH ( 6                     ) // FIXME ALFIO crosscheck
) i_udma_spim_top (
    .sys_clk_i           ( sys_clk_i             ),
    .periph_clk_i        ( periph_clk_i          ),
    .rstn_i              ( rstn_i                ),

    .dft_test_mode_i     ( dft_test_mode_i       ),
    .dft_cg_enable_i     ( dft_cg_enable_i       ),

    .spi_eot_o           ( events_o[3]           ),
    .spi_event_i         ( events_i              ),

    .cfg_data_i          ( cfg_data_i            ),
    .cfg_addr_i          ( cfg_addr_i            ),
    .cfg_valid_i         ( cfg_valid_i           ),
    .cfg_rwn_i           ( cfg_rwn_i             ),
    .cfg_data_o          ( cfg_data_o            ),
    .cfg_ready_o         ( cfg_ready_o           ),

    .cfg_cmd_startaddr_o ( cmd_ch[0].startaddr   ),
    .cfg_cmd_size_o      ( cmd_ch[0].size        ),
    .cfg_cmd_continuous_o( cmd_ch[0].continuous  ),
    .cfg_cmd_en_o        ( cmd_ch[0].cen         ),
    .cfg_cmd_clr_o       ( cmd_ch[0].clr         ),
    .cfg_cmd_en_i        ( cmd_ch[0].en          ),
    .cfg_cmd_pending_i   ( cmd_ch[0].pending     ),
    .cfg_cmd_curr_addr_i ( cmd_ch[0].curr_addr   ),
    .cfg_cmd_bytes_left_i( cmd_ch[0].bytes_left  ),
    .cmd_req_o           ( cmd_ch[0].req         ),
    .cmd_gnt_i           ( cmd_ch[0].gnt         ),
    .cmd_datasize_o      ( cmd_ch[0].datasize    ),
    .cmd_i               ( cmd_ch[0].data        ),
    .cmd_valid_i         ( cmd_ch[0].valid       ),
    .cmd_ready_o         ( cmd_ch[0].ready       ),

    .cfg_rx_startaddr_o  ( rx_ch[0].startaddr    ),
    .cfg_rx_size_o       ( rx_ch[0].size         ),
    .cfg_rx_continuous_o ( rx_ch[0].continuous   ),
    .cfg_rx_en_o         ( rx_ch[0].cen          ),
    .cfg_rx_clr_o        ( rx_ch[0].clr          ),
    .cfg_rx_en_i         ( rx_ch[0].en           ),
    .cfg_rx_pending_i    ( rx_ch[0].pending      ),
    .cfg_rx_curr_addr_i  ( rx_ch[0].curr_addr    ),
    .cfg_rx_bytes_left_i ( rx_ch[0].bytes_left   ),
    .data_rx_datasize_o  ( rx_ch[0].datasize     ),
    .data_rx_o           ( rx_ch[0].data         ),
    .data_rx_valid_o     ( rx_ch[0].valid        ),
    .data_rx_ready_i     ( rx_ch[0].ready        ),

    .cfg_tx_startaddr_o  ( tx_ch[0].startaddr    ),
    .cfg_tx_size_o       ( tx_ch[0].size         ),
    .cfg_tx_continuous_o ( tx_ch[0].continuous   ),
    .cfg_tx_en_o         ( tx_ch[0].cen          ),
    .cfg_tx_clr_o        ( tx_ch[0].clr          ),
    .cfg_tx_en_i         ( tx_ch[0].en           ),
    .cfg_tx_pending_i    ( tx_ch[0].pending      ),
    .cfg_tx_curr_addr_i  ( tx_ch[0].curr_addr    ),
    .cfg_tx_bytes_left_i ( tx_ch[0].bytes_left   ),
    .data_tx_req_o       ( tx_ch[0].req          ),
    .data_tx_gnt_i       ( tx_ch[0].gnt          ),
    .data_tx_datasize_o  ( tx_ch[0].datasize     ),
    .data_tx_i           ( tx_ch[0].data         ),
    .data_tx_valid_i     ( tx_ch[0].valid        ),
    .data_tx_ready_o     ( tx_ch[0].ready        ),

    .spi_clk_o           ( PAD_QSPI_SCLK.OUT     ),
    .spi_csn0_o          ( PAD_QSPI_CSN0.OUT     ),
    .spi_csn1_o          ( PAD_QSPI_CSN1.OUT     ),
    .spi_csn2_o          ( PAD_QSPI_CSN2.OUT     ),
    .spi_csn3_o          ( PAD_QSPI_CSN3.OUT     ),
    .spi_oen0_o          ( pad_qspi_mosi_io0_oe  ),
    .spi_oen1_o          ( pad_qspi_miso_io1_oe  ),
    .spi_oen2_o          ( pad_qspi_io2_oe       ),
    .spi_oen3_o          ( pad_qspi_io3_oe       ),
    .spi_sdo0_o          ( PAD_QSPI_MOSI_IO0.OUT ),
    .spi_sdo1_o          ( PAD_QSPI_MISO_IO1.OUT ),
    .spi_sdo2_o          ( PAD_QSPI_IO2.OUT      ),
    .spi_sdo3_o          ( PAD_QSPI_IO3.OUT      ),
    .spi_sdi0_i          ( PAD_QSPI_MOSI_IO0.IN  ),
    .spi_sdi1_i          ( PAD_QSPI_MISO_IO1.IN  ),
    .spi_sdi2_i          ( PAD_QSPI_IO2.IN       ),
    .spi_sdi3_i          ( PAD_QSPI_IO3.IN       )

);

// invert polarity
assign PAD_QSPI_MOSI_IO0.OE = ~pad_qspi_mosi_io0_oe;
assign PAD_QSPI_MISO_IO1.OE = ~pad_qspi_miso_io1_oe;
assign PAD_QSPI_IO2.OE      = ~pad_qspi_io2_oe     ;
assign PAD_QSPI_IO3.OE      = ~pad_qspi_io3_oe     ;

// statically assign polarity for SPI CSi and SCLK
assign PAD_QSPI_CSN0.OE     = 1'b1;
assign PAD_QSPI_CSN1.OE     = 1'b1;
assign PAD_QSPI_CSN2.OE     = 1'b1;
assign PAD_QSPI_CSN3.OE     = 1'b1;
assign PAD_QSPI_SCLK.OE     = 1'b1;

// assigning unused signals
assign rx_ch[0].stream      = '0;
assign rx_ch[0].stream_id   = '0;
assign rx_ch[0].destination = '0;
assign tx_ch[0].destination = '0;

assign events_o[2:0] = ch_events_i[2:0];

endmodule : udma_qspi_wrap