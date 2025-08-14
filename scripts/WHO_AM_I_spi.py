import board, busio, digitalio

spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)
cs = digitalio.DigitalInOut(board.CE0)
cs.direction = digitalio.Direction.OUTPUT
cs.value = True  # inactive (high)

while not spi.try_lock():
    pass
try:
    spi.configure(baudrate=1_000_000, polarity=0, phase=0, bits=8)

    def xfer(tx: bytes) -> bytes:
        rx = bytearray(len(tx))
        cs.value = False
        spi.write_readinto(tx, rx)
        cs.value = True
        return bytes(rx)

    WHO_AM_I = 0x0F
    READ = 0x80  # bit7=1 means read on ST parts
    resp = xfer(bytes([WHO_AM_I | READ, 0x00]))
    print("WHO_AM_I:", hex(resp[1]))  # expect 0x69
finally:
    spi.unlock()
