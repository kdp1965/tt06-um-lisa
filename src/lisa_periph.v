/*
==============================================================================
lisa_periph.v:  Little ISA (LISA) peripherals.

Copyright 2024 by Ken Pettit <pettitkd@gmail.com>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.

==============================================================================
*/

/*
==========================================================================================
lisa_periph:   A peripheral controller for the lisa core.

Address Map:

   GPIO
   ====
   0x00:       PORT A input / output
   0x01:       PORT A direction (1 = output)
   0x02:       PORT B input / output
   0x03:       PORT B direction (1 = output)
   0x04:       PORT C input / output
   0x05:       PORT C direction (1 = output)
   0x06:       PORT D input / output
   0x07:       PORT D direction (1 = output)

   TIMER
   =====
   0x08:       Pre Divisor Low
   0x09:       Pre Divisor High
   0x0A:       Divisor Low
   0x0B:       Divisor High
   0x0C:       Control / Status
   0x0D:       Timer tick

   UART
   ====
   0x10:       TX/RX register
   0x11:       Status register

   I2C
   ====
   0x20:       CLOCK PRE div LSB register
   0x21:       CLOCK PRE div MSB register
   0x22:       CTRL
   0x23:       RX
   0x24:       STATUS
   0x25:       TX
   0x26:       CMD

==========================================================================================
*/
module lisa_periph
(
   input                clk,
   input                rst_n,

   // UART access
   input  [6:0]         d_addr,
   input  [7:0]         d_i,
   input                d_periph,
   input                d_we,
   input                d_rd,
   output wire [7:0]    d_o,

   // GPIO
   output reg [7:0]     porta,
   input  wire [7:0]    porta_in,
   output reg [7:0]     porta_dir,
   output reg [3:0]     portb,
   input  wire [3:0]    portb_in,
   output reg [3:0]     portb_dir,

   // UART
   output wire [7:0]    uart_tx_d,
   output wire          uart_tx_wr,
   output wire          uart_rx_rd,
   input  wire [7:0]    uart_rx_d,
   input  wire          uart_rx_data_avail,
   input  wire          uart_tx_buf_empty,

   // I2C
   input  wire          scl_pad_i,
   output wire          scl_pad_o,
   output wire          scl_padoen_o,
   input  wire          sda_pad_i,
   output wire          sda_pad_o,
   output wire          sda_padoen_o
);

   reg   [15:0]         ms_count;
   reg   [15:0]         ms_prediv;
   reg   [15:0]         ms_preload;
   reg   [15:0]         ms_timer;
   reg   [7:0]          ms_tick;
   reg                  ms_enable;
   reg                  ms_rollover;
   wire  [7:0]          porta_read;
   wire  [3:0]          portb_read;
   reg                  d_o_r;
   wire  [7:0]          i2c_d_o;

   always @(posedge clk)
   begin
      if (~rst_n)
      begin
         porta       <= 8'h00;
         porta_dir   <= 8'h00;
         portb       <= 4'h00;
         portb_dir   <= 4'h0;
         ms_prediv   <= 16'd29494;
         ms_count    <= 16'h0;
         ms_timer    <= 16'h0;
         ms_preload  <= 16'h0;
         ms_tick     <= 8'h0;
         ms_enable   <= 1'h0;
         ms_rollover <= 1'b0;
      end
      else
      begin
         // ==============================================
         // GPIO signals
         // ==============================================
         // Latch input data as PORTA
         if (d_periph && d_we && d_addr == 7'h00)
            porta <= d_i;

         // Latch input data as PORTA_DIR
         if (d_periph && d_we && d_addr == 7'h01)
            porta_dir <= d_i;

         // Latch input data as PORTB
         if (d_periph && d_we && d_addr == 7'h02)
            portb <= d_i[3:0];

         // Latch input data as PORTB_DIR
         if (d_periph && d_we && d_addr == 7'h03)
            portb_dir <= d_i[3:0];

         // ==============================================
         // Timer signals
         // ==============================================
         // The control signal
         if (d_periph && d_we && d_addr == 7'h0C)
            ms_enable <= d_i[0];

         // The pre divider
         if (d_periph && d_we && d_addr == 7'h08)
            ms_prediv[7:0] <= d_i;

         // The pre divider
         if (d_periph && d_we && d_addr == 7'h09)
            ms_prediv[15:8] <= d_i;

         // The preload
         if (d_periph && d_we && d_addr == 7'h0A)
            ms_preload[7:0] <= d_i;

         // The preload
         if (d_periph && d_we && d_addr == 7'h0B)
            ms_preload[15:8] <= d_i;

         // The ms_rollover bit
         if (ms_count == ms_prediv && ms_timer == 8'h01)
            ms_rollover <= 1'b1;
         else if (d_periph && 
                ((d_rd && d_addr == 7'h0C) ||
                 (d_we && d_addr == 7'h0D)))
            ms_rollover <= 1'b0;

         // Loading tick value also resets 1ms timimg
         if (d_periph && d_we && d_addr == 7'h0D)
         begin
            ms_tick <= d_i;
            ms_timer <= ms_preload;
            ms_count <= 16'h0; 
         end

         // Test if timer is enabled
         else if (ms_enable)
         begin
            // Implement a 1ms tick
            if (ms_count == ms_prediv)
            begin
               ms_count <= 16'h0; 
            
               // Count down to 1 then increment tick and load the preload
               if (ms_timer == 8'h01)
               begin
                  ms_tick <= ms_tick + 1;
                  ms_timer <= ms_preload;
               end
               else
                  ms_timer <= ms_timer - 1;
            end
            else
               ms_count <= ms_count + 16'h1;
         end
         else
         begin
            ms_tick <= 8'h0;
            ms_timer <= ms_preload;
            ms_count <= 16'h0; 
         end
      end
   end

   // ==============================================
   // The read signals
   // ==============================================
   generate
   genvar x;
   for (x = 0; x < 8; x = x + 1)
   begin
      assign porta_read[x] = porta_dir[x] ? porta[x] : porta_in[x];
   end
   for (x = 0; x < 4; x = x + 1)
   begin
      assign portb_read[x] = portb_dir[x] ? portb[x] : portb_in[x];
   end
   endgenerate 

   always @*
   begin
      case (d_addr)
      // GPIO readback
      7'h00:   d_o_r <= porta_read;
      7'h01:   d_o_r <= porta_dir;
      7'h02:   d_o_r <= {4'h0, portb_read};
      7'h03:   d_o_r <= {4'h0, portb_dir};

      // Timer readback
      7'h08:   d_o_r <= ms_prediv[7:0];
      7'h09:   d_o_r <= ms_prediv[15:8];
      7'h0A:   d_o_r <= ms_preload[7:0];
      7'h0B:   d_o_r <= ms_preload[15:8];
      7'h0C:   d_o_r <= { ms_rollover, 6'h0, ms_enable};
      7'h0D:   d_o_r <= ms_tick;

      // UART read
      7'h10:   d_o_r <= uart_rx_d;
      7'h11:   d_o_r <= {6'h0, uart_tx_buf_empty, uart_rx_data_avail};

      default: d_o_r <= 8'h00;
      endcase
   end

   // ==============================================
   // UART read / write controls
   // ==============================================
   assign uart_tx_wr = d_periph && d_we && d_addr == 7'h10;
   assign uart_rx_rd = d_periph && d_rd && d_addr == 7'h10;
   assign uart_tx_d  = d_i;

   // ==============================================
   // Instantiate I2C master
   // ==============================================
   lisa_i2c i_lisa_i2c
   (
      .clk          ( clk          ),
      .rst_n        ( rst_n        ),
      .d_addr       ( d_addr       ),
      .d_i          ( d_i          ),
      .d_periph     ( d_periph     ),
      .d_we         ( d_we         ),
      .d_rd         ( d_rd         ),
      .d_o          ( i2c_d_o      ),
      .scl_pad_i    ( scl_pad_i    ),
      .scl_pad_o    ( scl_pad_o    ),
      .scl_padoen_o ( scl_padoen_o ),
      .sda_pad_i    ( sda_pad_i    ),
      .sda_pad_o    ( sda_pad_o    ),
      .sda_padoen_o ( sda_padoen_o )
   );

   assign d_o = d_o_r | i2c_d_o;
endmodule

