# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
TOPLEVEL_LANG ?= verilog
SRC = $(PWD)/../src

# normal simulation
ifneq ($(GATES),yes)

# this is the only part you should need to modify:
VERILOG_SOURCES += $(PWD)/tb.v $(SRC)/tt_um_lisa.v $(SRC)/RAM32.v
VERILOG_SOURCES += $(SRC)/lisa_core.v $(SRC)/lisa_dbg.v $(SRC)/lisa_periph.v
VERILOG_SOURCES += $(SRC)/lisa_rx8n.v $(SRC)/lisa_tx8n.v $(SRC)/lisa_qqspi.v
VERILOG_SOURCES += $(SRC)/lisa_qspi_controller.v $(SRC)/lisa_io_mux.v
VERILOG_SOURCES += $(SRC)/lisa_div.v $(SRC)/data_cache.v $(SRC)/inst_cache.v
VERILOG_SOURCES += $(SRC)/debug_ctrl.v $(SRC)/debug_autobaud.v
VERILOG_SOURCES += $(SRC)/debug_brg.v $(SRC)/debug_regs.v $(SRC)/lisa_uart.v $(SRC)/fops.v
VERILOG_SOURCES += $(SRC)/lisa_i2c.v $(SRC)/i2c_master_byte_ctrl.v $(SRC)/i2c_master_bit_ctrl.v
VERILOG_SOURCES += $(PWD)/psram.v $(PWD)/spiflash.v
COMPILE_ARGS    += -I$(SRC)

else

# gate level simulation requires some extra setup, you shouldn't need to touch this
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DUSE_POWER_PINS
COMPILE_ARGS    += -DSIM
COMPILE_ARGS    += -DUNIT_DELAY=\#1
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/tb.v $(PWD)/gate_level_netlist.v $(PWD)/../macros/RAM32.pnl.v
VERILOG_SOURCES += $(SRC)/lisa_rx8n.v $(SRC)/lisa_tx8n.v $(SRC)/debug_brg.v
VERILOG_SOURCES += $(PWD)/psram.v $(PWD)/spiflash.v
endif

# TOPLEVEL is the name of the toplevel module in your Verilog or VHDL file
TOPLEVEL = tb

# MODULE is the basename of the Python test file
MODULE = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
