# test/unit/test_lsm6ds3tr_driver_mapping.py

# Registers we care about (match I2C driver)
CTRL3_C    = 0x12
STATUS_REG = 0x1E
OUTX_L_G   = 0x22

XLDA = 0x01
GDA  = 0x02


class FakePiHost:
    """
    Minimal pigpio.pi()-like shim for I2C:
      - i2c_open / i2c_close / stop
      - i2c_read_byte_data(STATUS_REG) returns XLDA|GDA
      - i2c_read_i2c_block_data(OUTX_L_G, 12) returns a supplied 12-byte block
      - i2c_write_byte_data captures CTRL3_C writes (BDU/IF_INC set)
    """
    def __init__(self, *, block12: bytes, ctrl3c_init=0x00):
        assert len(block12) == 12
        self._handles = {}
        self._next_h = 1
        self._ctrl3c = ctrl3c_init
        self._writes = []
        self._block12 = block12

    def i2c_open(self, bus: int, addr: int, flags: int = 0):
        h = self._next_h
        self._next_h += 1
        self._handles[h] = (bus, addr, flags)
        return h

    def i2c_close(self, handle: int):
        self._handles.pop(handle, None)

    def stop(self):
        self._handles.clear()

    def i2c_read_byte_data(self, handle: int, reg: int) -> int:
        if reg == STATUS_REG:
            return XLDA | GDA
        if reg == CTRL3_C:
            return self._ctrl3c & 0xFF
        return 0

    def i2c_read_i2c_block_data(self, handle: int, reg: int, count: int):
        if reg == OUTX_L_G and count >= 12:
            return self._block12[:count] if count < 12 else self._block12
        return bytes([0] * count)

    def i2c_write_byte_data(self, handle: int, reg: int, val: int):
        self._writes.append((reg & 0xFF, val & 0xFF))
        if reg == CTRL3_C:
            self._ctrl3c = val & 0xFF
        return 0


def test_read_ax_ay_gz_bytes_maps_expected_order(monkeypatch):
    # Distinctive 12-byte block:
    # [GX_L,GX_H, GY_L,GY_H, GZ_L,GZ_H, AX_L,AX_H, AY_L,AY_H, AZ_L,AZ_H]
    block12 = bytes([
        0x11, 0x12,  # GX
        0x21, 0x22,  # GY
        0x31, 0x32,  # GZ
        0x41, 0x42,  # AX
        0x51, 0x52,  # AY
        0x61, 0x62,  # AZ
    ])

    # Driver should return [AX_L,AX_H, AY_L,AY_H, GZ_L,GZ_H]
    import hardware.i2c_driver as i2c_drv
    monkeypatch.setattr(
        i2c_drv, "get_i2c_host",
        lambda _params=None: FakePiHost(block12=block12),
        raising=True,
    )

    from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver
    dev = LSM6DS3TRDriver(controller_params={})
    try:
        six = dev.read_ax_ay_gz_bytes()
    finally:
        dev.close()

    assert six == b'\x61\x62\x51\x52\x11\x12'


def test_driver_sets_bdu_and_ifinc(monkeypatch):
    # Start with BDU/IF_INC clear (0x00); verify write to CTRL3_C sets both bits
    block12 = bytes([0] * 12)
    host = FakePiHost(block12=block12, ctrl3c_init=0x00)

    import hardware.i2c_driver as i2c_drv
    monkeypatch.setattr(i2c_drv, "get_i2c_host", lambda _p=None: host, raising=True)

    from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver
    dev = LSM6DS3TRDriver(controller_params={})
    dev.close()

    # BDU (bit6) and IF_INC (bit2) should be set at some point
    wrote = dict(host._writes)
    assert CTRL3_C in wrote and (wrote[CTRL3_C] & 0x44) == 0x44, f"CTRL3_C writes: {host._writes}"
