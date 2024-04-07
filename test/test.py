import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

async def send_tx_byte(dut, data):
   # Wait for the UART TX to be ready 
   await int(dut.tx_buf_empty.value) == 1

   # Configure the write
   dut.tx_d.value = data
   dut.tx_wr.value = 1
   await ClockCycles(dut.clk, 2)

   dut.tx_wr.value = 0
   await ClockCycles(dut.clk, 2)

@cocotb.test()
async def test_lisa(dut):
    dut._log.info("start")
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    dut._log.info("ena")
    dut.ena.value = 1
    dut.porta_in.value = 0
    dut.uart_port_sel.value = 0
    dut.tx_wr.value = 0
    dut.rx_rd.vauee = 0
    dut.tx_d.value  = 0

    dut._log.info("reset")
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)

    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 32)

    # Send a \n character
    await send_tx_byte(dut, 0x0a)
    await send_tx_byte(dut, 0x0a)

    dut._log.info("all good!")
