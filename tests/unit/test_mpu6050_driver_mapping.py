# tests/unit/test_mpu6050_driver_mapping.py
import pytest

MPU_WHO_AM_I      = 0x75
MPU_PWR_MGMT_1    = 0x6B
MPU_SMPLRT_DIV    = 0x19
MPU_CONFIG        = 0x1A
MPU_GYRO_CONFIG   = 0x1B
MPU_ACCEL_CONFIG  = 0x1C
MPU_ACCEL_XOUT_H  = 0x3B


class FakePiHost:
    def __init__(self, regmap=None):
        self._next_h = 1
        self.regmap = dict(regmap or {})
        self.writes = []
        self.open_calls = []
        self.close_calls = []
        self.stopped = False

    def i2c_open(self, bus, addr, flags=0):
        self.open_calls.append((bus, addr, flags))
        h = self._next_h
        self._next_h += 1
        return h

    def i2c_close(self, handle):
        self.close_calls.append(handle)

    def stop(self):
        self.stopped = True

    def i2c_read_byte_data(self, handle, reg):
        return int(self.regmap.get(reg & 0xFF, 0)) & 0xFF

    def i2c_write_byte_data(self, handle, reg, val):
        self.writes.append((reg & 0xFF, val & 0xFF))
        self.regmap[reg & 0xFF] = val & 0xFF

    def i2c_read_i2c_block_data(self, handle, reg, count):
        data = bytes(int(self.regmap.get((reg + i) & 0xFF, 0)) & 0xFF for i in range(count))
        return (count, data)

@pytest.mark.unit
def test_read_all_axes_big_endian_signed(monkeypatch):
    regmap = {MPU_WHO_AM_I: 0x68}

    burst = [
        0x12, 0x34,   # AX
        0xED, 0xCC,   # AY (-0x1234)
        0x00, 0x01,   # AZ
        0x00, 0x00,   # TEMP
        0xFF, 0xFF,   # GX (-1)
        0x00, 0x02,   # GY (+2)
        0x80, 0x00,   # GZ (-32768)
    ]
    for i, b in enumerate(burst):
        regmap[MPU_ACCEL_XOUT_H + i] = b

    host = FakePiHost(regmap=regmap)

    import hardware.mpu6050_i2c_driver as mpu
    monkeypatch.setattr(mpu, "get_i2c_host", lambda _p=None, **_kw: host, raising=True)

    from hardware.mpu6050_i2c_driver import MPU6050Driver
    dev = MPU6050Driver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x68, "SIM_MODE": True})
    try:
        ax, ay, az, gx, gy, gz = dev.read_all_axes()
    finally:
        dev.close()

    assert ax == 0x1234
    assert ay == -0x1234
    assert az == 1
    assert gx == -1
    assert gy == 2
    assert gz == -32768

@pytest.mark.unit
def test_init_writes_expected_registers(monkeypatch):
    host = FakePiHost(regmap={MPU_WHO_AM_I: 0x68})

    import hardware.i2c_driver as i2c_drv
    monkeypatch.setattr(i2c_drv, "get_i2c_host", lambda _p=None, **_kw: host, raising=True)
    from hardware.mpu6050_i2c_driver import MPU6050Driver
    dev = MPU6050Driver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x68, "SIM_MODE": True})
    dev.close()

    wrote = dict(host.writes)
    assert wrote.get(MPU_PWR_MGMT_1) == 0x00
    assert wrote.get(MPU_CONFIG) == 0x03
    assert wrote.get(MPU_GYRO_CONFIG) == 0x00
    assert wrote.get(MPU_ACCEL_CONFIG) == 0x00
    assert wrote.get(MPU_SMPLRT_DIV) == 0x07

    assert host.stopped is True