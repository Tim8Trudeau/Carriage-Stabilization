#!/usr/bin/env python3
import sys
import time
import pigpio

# ---- User settings ----
SPI_CHANNEL = 0          # 0 or 1 (we won't use its CE pin physically)
SPI_BAUD    = 100000      # 1 MHz (adjust as needed)
SPI_MODE    = 0          # CPOL=0, CPHA=0 typical
CS_GPIO     = 25         # <-- your manual CS pin to the device

REG_ADDR    = 0x18
WRITE_VALUE = 0x28
READ_BIT    = 0x80       # many devices use MSB=1 for read
# -----------------------

def main():
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio daemon not running. Start it with: sudo pigpiod", file=sys.stderr)
        sys.exit(1)

    try:
        # Manual CS as output (inactive high)
        pi.set_mode(CS_GPIO, pigpio.OUTPUT)
        pi.write(CS_GPIO, 1)

        # Open SPI (CE line from this channel is NOT used physically)
        h = pi.spi_open(SPI_CHANNEL, SPI_BAUD, SPI_MODE)

        # --- Write 0x3C to register 0x18 ---
        pi.write(CS_GPIO, 0)  # CS low
        pi.spi_write(h, bytes([REG_ADDR & 0x7F, WRITE_VALUE]))  # clear MSB for write
        pi.write(CS_GPIO, 1)  # CS high

        # --- Read back register 0x18 ---
        # Keep CS low across address and clocking the read byte. We do:
        #   1) spi_write(address | READ_BIT)
        #   2) spi_read(1) to clock out one byte
        pi.write(CS_GPIO, 0)  # CS low

        # Send the read address (no data yet)
        pi.spi_write(h, bytes([REG_ADDR | READ_BIT]))

        # Clock out one byte (pigpio will generate clocks and read MISO)
        n, data = pi.spi_read(h, 1)

        pi.write(CS_GPIO, 1)  # CS high

        if n != 1:
            print(f"Read failed: expected 1 byte, got {n}", file=sys.stderr)
            sys.exit(2)

        value = data[0]
        print(f"Register 0x{REG_ADDR:02X} = 0x{value:02X}")

        # Optional quick sanity check
        if value != WRITE_VALUE:
            print("Warning: readback does not match written value.")

        pi.spi_close(h)

    finally:
        # Leave CS high (inactive) and clean up
        pi.write(CS_GPIO, 1)
        pi.stop()

if __name__ == "__main__":
    main()
