## How it works


                                                                                                      
              |\         +-------------------+                +--------------------+     +-------------------+ 
 rx1  -----*->| |        |                   |                |                    |     |                   |
           |  | |        |   debug_ctrl      +--------------->|    lisa_qspi       |     |   lisa_qqspi      | QSPI Pins
 rx2  ---*-|->| +------->|                   |                |     controller     +<--->|                   +-----------> 
         | |  | |        |  Debug Interface  |                |  (QSPI Arbiter)    |     |  QSPI Controller  |
 rx3  -*-|-|->| |        |                   +---+            |                    |     |                   |
       | | |  |/         +-------------------+   |            +--------------------+     +-------------------+
       | | |   |                 ^               |                     ^
       | | |   |                 |               |                     |                                  
       v v v   |                 v               |            +--------+-----------+     +-------------+       
      +--------+---+       +--------------+      +----------->|                    |     |  lisa_dbg   |       
      |   debug    |       |              |                   |    lisa_core       +<--->|  Debug reg/ |       
      |  autobaud  |       | debug_regs   |    +---------+    |                    |     |    RAM      |       
      |            |       |              |    |  RAM32  |<-->|                    |     |   access    |    
      +------------+       +--------------+    +---------+    +--------------------+     +-------------+    
                                                          
                                                                           
                                                                                                         
This is an 8-Bit Little ISA (LISA) processor.  It includes the following:

   - Harvard architecture LISA Core (16-bit instruction, 15-bit address space)
   - Debug interface
      * UART controlled
      * Auto detects port from one of 3 interfaces
      * Auto detects the baud rate
      * Interfaces with SPI / QSPI SRAM or FLASH
      * Can erase / program the (Q)SPI FLASH
      * Read/write LISA core registers
      * Set LISA breakpoints, single step, etc.
      * SPI/QSPI programmability (single/quad, port location, CE selects)
   - (Q)SPI interface for instruction fetch
   - Onboard 128 Byte RAM for DATA / DATA CACHE
   - 16-bit programmable timer (with pre-divide)
   - Debug UART available to LISA core also
   - 8-bit Input and Output port (PORTA)
   - 4-bit BIDIR port (PORTB)
   - Hardware 8x8 integer multiplier
   - Programmable I/O mux for maximum flexibility of I/O usage.
                                                                         
It uses a 32x32 1RW [DFFRAM](https://github.com/AUCOHL/DFFRAM) macro to implement a 128 bytes (1 kilobit) RAM module.

Reseting the project **does not** reset the RAM contents.

## How to test

You will need to download and compile the C-based assembler, linker and C compiler I wrote (will make available)
Also need to download the Python based debugger.

  - Assembler is fully functional
    - Includes limited libraries for crt0, signed int compare, math, etc.
    - Libraries are still a work in progress
  - Linker is fully functional
  - C compiler is functional (no float support at the moment) but is a work in progress.
  - Python debugger can erase/program the FLASH, program SPI SRAM, start/stop the LISA core, read SRAM and registers.

