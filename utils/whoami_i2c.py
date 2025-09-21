#!/usr/bin/env python3
# Read LSM6DS3 WHO_AM_I (0x0F) over I2C using pigpio
# Try: 0x6A (SDO/SA0=0) and 0x6B (SDO/SA0=1)
# Expected WHO_AM_I value for LSM6DS3 / LSM6DS3TR-C is 0x69.

import sys
import pigpio

WHO_AM_I_REG = 0x0F
DEFAULT_BUS = 1  # Raspberry Pi uses I2C bus 1 on GPIO2(SDA)/GPIO3(SCL)

def read_whoami(pi: pigpio.pi, bus: int, addr: int) -> int:
    """Open I2C device at addr, read WHO_AM_I, close, return integer 0..255."""
    h = pi.i2c_open(bus, addr)
    try:
        val = pi.i2c_read_byte_data(h, WHO_AM_I_REG)
        if val < 0:
            raise RuntimeError(f"i2c_read_byte_data returned error {val}")
        return val
    finally:
        pi.i2c_close(h)

def main():
    # Optional CLI: python whoami_i2c.py [bus] [addr]
    # (addr is 7-bit, e.g. 0x6A or 0x6B). If not provided, we'll try both.
    bus = DEFAULT_BUS
    addrs = [0x6A, 0x6B]

    if len(sys.argv) >= 2:
        bus = int(sys.argv[1], 0)
    if len(sys.argv) >= 3:
        addrs = [int(sys.argv[2], 0)]

    pi = pigpio.pi()
    if not pi.connected:
        print("Error: pigpio daemon not running. Start it with: sudo pigpiod", file=sys.stderr)
        sys.exit(1)

    try:
        found = False
        for addr in addrs:
            try:
                val = read_whoami(pi, bus, addr)
                print(f"I2C bus {bus}, addr 0x{addr:02X} → WHO_AM_I = 0x{val:02X}")
                found = True
            except Exception as e:
                # Silence common "no device" errors when probing both addresses
                if len(addrs) == 1:
                    print(f"I2C bus {bus}, addr 0x{addr:02X} → read failed: {e}", file=sys.stderr)
        if not found and len(addrs) > 1:
            print(f"No response at 0x6A or 0x6B on I2C bus {bus}. Check wiring and SDO/SA0 level.")
    finally:
        pi.stop()

if __name__ == "__main__":
    main()
