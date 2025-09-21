#!/usr/bin/env python3
import pigpio, sys, time

WHO_AM_I = 0x0F
CTRL3_C  = 0x12

def rreg(pi, h, addr):
    tx = bytes([addr | 0x80, 0x00])  # READ bit set, dummy
    n, rx = pi.spi_xfer(h, tx)
    return rx[1] if n == 2 else None

def try_combo(ch, mode):
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio not running (sudo pigpiod).", file=sys.stderr)
        sys.exit(1)
    try:
        h = pi.spi_open(ch, 100_000, mode)  # ch: 0=>CE0, 1=>CE1
        time.sleep(0.001)
        who = rreg(pi, h, WHO_AM_I)
        c3  = rreg(pi, h, CTRL3_C)
        print(f"CE{ch}, mode {mode}: WHO_AM_I=0x{who:02X}  CTRL3_C=0x{c3:02X}")
    finally:
        try:
            pi.spi_close(h)
        except Exception:
            pass
        pi.stop()

if __name__ == "__main__":
    for ch in (0, 1):
        for mode in (0, 3):
            try_combo(ch, mode)
