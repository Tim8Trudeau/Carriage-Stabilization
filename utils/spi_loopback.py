#!/usr/bin/env python3
"""
SPI loopback test using pigpio.
- Short MOSI (GPIO10, pin 19) to MISO (GPIO9, pin 21).
- Make sure NO slave is selected during transfers (use a CE that's not wired,
  or disconnect the slave's CS). Otherwise you'll short two outputs together.

It tries CE0 and CE1 in modes 0 and 3, sends a few patterns,
and checks that RX == TX.
"""
import sys
import time
import pigpio

PATTERNS = [
    bytes([0x00]),
    bytes([0xFF]),
    bytes([0xAA, 0x55, 0xF0, 0x0F]),
    bytes(range(16)),
]

def loopback_once(pi, handle, tx: bytes):
    n, rx = pi.spi_xfer(handle, tx)
    if n != len(tx):
        return False, f"count {n} != {len(tx)}", rx
    return (rx == tx), None, rx

def try_combo(channel: int, mode: int, baud: int = 100_000):
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio not running (start with: sudo pigpiod)", file=sys.stderr)
        sys.exit(1)
    try:
        # In pigpio, 'mode' is passed in the 'flags' argument (bits 0-1).
        handle = pi.spi_open(channel, baud, mode)
        time.sleep(0.001)

        all_ok = True
        for tx in PATTERNS:
            ok, err, rx = loopback_once(pi, handle, tx)
            tag = f"CE{channel}, mode {mode}, {len(tx)}B"
            if ok:
                print(f"{tag}: OK  TX==RX {list(tx)}")
            else:
                all_ok = False
                print(f"{tag}: FAIL ({err})  TX={list(tx)} RX={list(rx)}")
        return all_ok
    finally:
        try:
            pi.spi_close(handle)
        except Exception:
            pass
        pi.stop()

if __name__ == "__main__":
    overall = True
    for ch in (0, 1):      # Use the CE that is NOT wired to your IMU
        for mode in (0, 3):
            ok = try_combo(ch, mode)
            overall = overall and ok
    sys.exit(0 if overall else 1)
