import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# ==================================================
# Write a byte to the UART
# ==================================================
async def send_tx_byte(dut, data):
   count = 0

   # Wait for the UART TX to be ready 
   while not dut.tx_buf_empty.value and count < 80000:
      await ClockCycles(dut.clk, 1)
      count = count + 1

   # Test for timeout
   assert count != 20000

   # Configure the write
   dut.tx_d.value = data
   dut.tx_wr.value = 1
   await ClockCycles(dut.clk, 2)

   dut.tx_wr.value = 0
   await ClockCycles(dut.clk, 2)

# ==================================================
# Read a byte from the UART
# ==================================================
async def read_rx_byte(dut):
   count = 0

   # Wait for the UART TX to be ready 
   while not dut.rx_avail.value and count < 80000:
      await ClockCycles(dut.clk, 1)
      count = count + 1

   # Test for timeout
   assert count != 20000

   # Perform a read
   retval = dut.rx_d.value
   dut.rx_rd.value = 1
   await ClockCycles(dut.clk, 1)

   dut.rx_rd.value = 0
   await ClockCycles(dut.clk, 1)
   return retval

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

    # Send a \n character to set the Baud rate
    await send_tx_byte(dut, 0x0a)

    # Wait a while for baud rate determination
    await ClockCycles(dut.clk, 131072)

    # ======================================================
    # Send CR to the debug interface and test for response
    # ======================================================
    await send_tx_byte(dut, 0x0a)

    # Read the LF
    retval = await read_rx_byte(dut)
    dut._log.info(f'retval = 0x{int(retval):02x}')

    # Read the CR
    retval = await read_rx_byte(dut)
    dut._log.info(f'retval = 0x{int(retval):02x}')

    # Read the CR
    retval = await read_rx_byte(dut)
    dut._log.info(f'retval = 0x{int(retval):02x}')

    dut._log.info("all good!")

