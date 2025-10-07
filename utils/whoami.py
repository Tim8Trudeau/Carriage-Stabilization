#!/usr/bin/env python3
import sys
import time
import pigpio

SPI_CHANNEL = 0        # 0 -> CE0, 1 -> CE1
SPI_BAUD    = 100_000
SPI_MODE    = 0        # CPOL=0, CPHA=0 (adjust if your device needs 1/2/3)

REG_ADDR    = 0x0F     # register to read (WHO_AM_I on many IMUs)
READ_BIT    = 0x80     # many SPI devices use MSB=1 for read

def main():
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio daemon not running. Start it with: sudo pigpiod", file=sys.stderr)
        sys.exit(1)

    try:
        # Open SPI
        h = pi.spi_open(SPI_CHANNEL, SPI_BAUD, SPI_MODE)

        # Many devices expect the read bit set in the address.
        addr_byte = REG_ADDR | READ_BIT

        # Write just the address first…
        # (Some chips also need a dummy byte in the same CS frame, but we're
        # restricted to spi_write/spi_read, so we do two calls.)
        pi.spi_write(h, bytes(REG_ADDR))

 
        # …then read back one byte.
        count, data = pi.spi_read(h, 1)
        if count != 1:
            print(f"SPI read failed (count={count})", file=sys.stderr)
            sys.exit(2)

        value = data[0]
        print(f"Reg 0x{REG_ADDR:02X} = 0x{value:02X} ({value})")

    finally:
        try:
            pi.spi_close(h)
        except Exception:
            pass
        pi.stop()

if __name__ == "__main__":
    main()
