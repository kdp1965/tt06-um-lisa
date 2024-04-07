`default_nettype none
`timescale 1ns / 1ps

/*
this testbench just instantiates the module and makes some convenient wires
that can be driven / tested by the cocotb test.py
*/

module tb
(
   // testbench is controlled by test.py
   input          clk,
   input          rst_n,
   input          ena,
   input [7:0]    tx_d,
   input          tx_wr,
   output         tx_buf_empty,
   input          rx_rd,
   output [7:0]   rx_d,
   output         rx_avail,
   input  [1:0]   uart_port_sel,
   input  [7:0]   porta_in,
   output [7:0]   porta_out
);

   wire     [7:0] ui_in;
   wire     [7:0] uo_out;
   wire     [7:0] uio_oe;
   wire     [7:0] uio_out;
   wire     [7:0] uio_in;

   wire           ce0;
   wire           ce1;
   wire           sclk;
   wire           dq0;
   wire           dq1;
   wire           dq2;
   wire           dq3;

   // this part dumps the trace to a vcd file that can be viewed with GTKWave
   initial begin
     $dumpfile("tb.vcd");
     $dumpvars(0, tb);
     #1;
   end

   // ==========================================================================
   // instantiate the DUT
   // ==========================================================================
   tt_um_lisa lisa
   (
`ifdef GL_TEST
      .VPWR(1'b1),
      .VGND(1'b0),
`endif
      .ui_in   ( ui_in    ),
      .uo_out  ( uo_out   ),
      .uio_in  ( uio_in   ),
      .uio_out ( uio_out  ),
      .uio_oe  ( uio_oe   ),
      .ena     ( ena      ),
      .clk     ( clk      ),
      .rst_n   ( rst_n    )
   );

   // ==========================================================================
   // Connect the QSPI signals to the SPI flash
   // ==========================================================================
   assign ce0  = uio_out[0];
   assign sclk = uio_out[3];
   assign dq0  = uio_oe[1] ? uio_out[1] : 1'bz;
   assign dq1  = uio_oe[2] ? uio_out[2] : 1'bz;
   assign dq2  = uio_oe[6] ? uio_out[6] : 1'bz;
   assign dq3  = uio_oe[7] ? uio_out[7] : 1'bz;
   assign uio_in[1] = dq0;
   assign uio_in[2] = dq1;
   assign uio_in[6] = uart_port_sel == 2'h2 ? txd : dq2;
   assign uio_in[7] = dq3;

   // ==========================================================================
   // Connect the UART to the port specified
   // ==========================================================================
   assign ui_in[3]  = uart_port_sel == 2'h0 ? txd : porta_in[3];
   assign uio_in[4] = uart_port_sel == 2'h1 ? txd : 1'b0;
   assign rxd = uart_port_sel == 2'h0 ? uo_out[4] : 
                uart_port_sel == 2'h1 ? uio_out[5] :
                uart_port_sel == 2'h2 ? uio_out[5] :
                1'b0;

   // ==========================================================================
   // Create a testbench UART to interface with the design
   // ==========================================================================
   wire        baud_ref;
   wire        txd;
   wire        rxd;
   wire        baud_set;
   wire [6:0]  baud_div;

   lisa_tx8n i_tx8n
   (
      .clk        ( clk          ),
      .rst_n      ( rst_n        ),
      .baud_ref   ( baud_ref     ),  // Baud Rate reference
      .wr         ( tx_wr        ),  // Write signal
      .d          ( tx_d         ),  // Input data
      .txd        ( txd          ),  // Output serial data
      .buf_empty  ( tx_buf_empty )
   );

   lisa_rx8n i_rx8n
   (
      .clk        ( clk          ),
      .rst_n      ( rst_n        ),
      .baud_ref   ( baud_ref     ),
      .rxd        ( rxd          ),
      .rd         ( rx_rd        ),
      .d          ( rx_d         ),
      .data_avail ( rx_avail     )
   );

   debug_brg i_brg
   (
      // Timing and reset inputs
      .clk        ( clk          ),       // System clock
      .rst_n      ( rst_n        ),       // Active low reset
      .wr         ( 1'b0         ),       // Data write strobe
      .d          ( 8'h0         ),       // Data input
      .baud_set   ( baud_set     ),
      .baud_div   ( baud_div     ),
      .baud_ref   ( baud_ref     )
   );
   assign baud_set = 1'b1;
   assign baud_div = 7'h3;

   // ==========================================================================
   // Assign the port I/O
   // ==========================================================================
   assign porta_out = uo_out;
   assign ui_in[2:0] = porta_in[2:0];
   assign ui_in[7:4] = porta_in[7:4];
   
   // ==========================================================================
   // Instantiate a SPI flash
   // ==========================================================================
   spiflash
   #(
      .FILENAME("firmware.hex")  // change the hex file to match your project
   )
   spiflash
   (
      .csb  ( ce0                  ),
      .clk  ( sclk                 ),
      .io0  ( dq0                  ),
      .io1  ( dq1                  ),
      .io2  ( dq2                  ),
      .io3  ( dq3                  )
   );

   // ==========================================================================
   // Instantiate a SPI SRAM
   // ==========================================================================
   psram psram_I
   (
      .ce_n ( ce1                  ),
      .sck  ( sclk                 ),
      .dio  ( {dq3, dq2, dq1, dq0} )
   );
   assign ce1 = 1'b1;

endmodule

