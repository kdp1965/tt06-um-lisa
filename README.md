![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

## LISA (Little ISA) 8-Bit microcontroller SOC

This project is a fully functional (hopefully)  microcontroller with an 8-bit processor, CACHE RAM,
peripherals, timer and QSPI interface for fetching instructions and extended RAM size.

It project includes a 32x32 (1024 bits) DFFRAM macro with a single read/write port (1RW) which it
uses as a DATA CACHE for the 8-bit processor data bus.  The Instruction bus is implemented over the
SPI / QuadSPI port to either the RP2040 or an external SPI FLASH or SPI SRAM chip.  Additionally,
the instruction bus features a 4-entry single line CACHE.

Interface to the project is via Debug UART (auto-senses the port and auto detects the baud rate).  The
debug interface also supports programming the SPI SRAM, erasing and programming a SPI FLASH, and 
accessing the LISA core debug features including register access, stop and resume, setting 
breakpoints, viewing and modifying RAM contents, etc.

The Debugger UART is available to the LISA processor core by sending it an 'l' command.  After
that, the LISA processor can send / receive data over the UART.  Issuing a "+++" command with a
0.5s guard time before / after will connect the UART back to the debugger.

Features include:
   - UART (shared with debugger)
   - 16-Bit timer
   - GPIO (8-bit A port, 4-bit bidir B port)
   - I2C Master
   - I/O Mux to select pin functions
   - 8x8 Hardware multiplier
   - 16/16 (or 16/8) Hardware divider
   - Three HW breakpoints
   - Software breakpoints
   - 32K (64KByte) Instruction space
   - 32K Byte Data Space
   - Configurable (Q)SPI chip selects
     - Two CE outputs
     - Programmable SCLK frequency per CE
     - Programmable inter-CE activation timer
     - Programmable mode (rising / falling SCLK data change)
     - Programmable dummy read delay per CE
     - 16 or 24 bit addressing per CE
     - FLASH or SRAM per CE
     - SPI or QSPI per CE
     - Support QSPI with 2 CE lines on single custom PMOD board
       with a QSPI FLASH, QSPI SRAM plus UART or I2C.  This 
       is accomplished using latches on the PMOD to latch the
       CE signal (on MISO and MOSI) at the beginning of the QSPI
       transaction, and the SoC has an I/O mux option to support
       this.
     - Configurable SPI / QSPI base addresses for Debugger, 
       LISA Instruction and LISA Data accesses.
     - Configureable pinout

The size of this project is 2x6 tiles.

I have written (in C) and assembler, linker and C compiler (still a work in progress).  Also I have
a Python based debugger that can load and verify the SPI FLASH, start and stop the core, view registers, etc.

The debug interface has a bank of configuration register that enable configuring the SPI / QSPI interface
to work with various devices.  It support up to 2 Chip Enable (CE) lines, each of which can be configured
as either single SPI or Quad SPI.  Additionaly, each can be configured ether as FLASH or SRAM, 16 or 24 bit
access, and the Debug interface, LISA Instruction CACHE and LISA Data CACHE can be individually configured
to use either of the two CE lines.  The debugger has special registers to allow sending custom commands to
the SPI / QSPI device, and there are registers to control the SCLK frequency and delay between successive
CE activations (to work with RP2040).

More documentation to come...

## What is Tiny Tapeout?

TinyTapeout is an educational project that aims to make it easier and cheaper than ever to get your digital designs manufactured on a real chip.

To learn more and get started, visit https://tinytapeout.com.

## Building the project locally

1. Install [OpenLane 2 with nix](https://openlane2.readthedocs.io/en/latest/getting_started/nix_installation/index.html).
   Set the `OPENLANE2_ROOT` environment variable to the path where you cloned the openlane2 repository.
2. Clone tt-support-tools: `git clone -b tt06 https://github.com/TinyTapeout/tt-support-tools tt`
3. Run the following command:

```bash
rm -rf runs && nix-shell ${OPENLANE2_ROOT}/shell.nix --run "python build.py"
```

The build.py script will create a runs directory and run the OpenLane flow. The results will be in the runs/wokwi directory.

When you run the build for the first time, nix will download all the dependencies. This can take a while, especially if you
haven't configured nix to use binary caches. Once the dependencies are downloaded, the build should take up to ten minutes.

## Changing the position of the RAM32 macro

You can change the position of the RAM32 macro in your design by editing `config.json` as follows:

1. Set `MACROS.RAM32.instances.ram1.location` to the x/y coordinates you want the RAM32 to be placed at.
2. Set `FP_PDN_VOFFSET` to the x coordinate of the RAM + 16.32 (so if the RAM is at 10, set `FP_PDN_VOFFSET` to 26.32).

Note that PDN (power distribution network) stripes of your design must match the PDN stripes of the RAM32 macro. Therefore, you must keep 
`FP_PDN_VPITCH` at the default value (153.6), and set `FP_PDN_VOFFSET` to the x coordinate of the RAM + 16.32 (as explained above).

## Resources

- [FAQ](https://tinytapeout.com/faq/)
- [Digital design lessons](https://tinytapeout.com/digital_design/)
- [Learn how semiconductors work](https://tinytapeout.com/siliwiz/)
- [Join the community](https://discord.gg/rPK2nSjxy8)

## What next?

- Submit your design to the next shuttle [on the website](https://tinytapeout.com/#submit-your-design). The closing date is **November 4th**.
- Edit this [README](README.md) and explain your design, how it works, and how to test it.
- Share your GDS on your social network of choice, tagging it #tinytapeout and linking Matt's profile:
  - LinkedIn [#tinytapeout](https://www.linkedin.com/search/results/content/?keywords=%23tinytapeout) [matt-venn](https://www.linkedin.com/in/matt-venn/)
  - Mastodon [#tinytapeout](https://chaos.social/tags/tinytapeout) [@matthewvenn](https://chaos.social/@matthewvenn)
  - Twitter [#tinytapeout](https://twitter.com/hashtag/tinytapeout?src=hashtag_click) [@matthewvenn](https://twitter.com/matthewvenn)

