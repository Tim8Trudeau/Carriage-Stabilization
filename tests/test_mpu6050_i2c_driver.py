import sys
import types


# MPU6050 register constants we verify against
MPU_WHO_AM_I      = 0x75
MPU_PWR_MGMT_1    = 0x6B
MPU_SMPLRT_DIV    = 0x19
MPU_CONFIG        = 0x1A
MPU_GYRO_CONFIG   = 0x1B
MPU_ACCEL_CONFIG  = 0x1C
MPU_ACCEL_XOUT_H  = 0x3B


class FakePiI2C:
    '''
    Minimal pigpio-like API used by MPU6050Driver.
    Records writes and serves reads from a register map.
    '''
    def __init__(self, regmap=None):
        self.regmap = dict(regmap or {})
        self.writes = []        # (handle, reg, value)
        self.open_calls = []
        self.close_calls = []
        self.stopped = False

    def i2c_open(self, bus, addr, flags):
        self.open_calls.append((bus, addr, flags))
        return 1

    def i2c_close(self, handle):
        self.close_calls.append(handle)

    def i2c_write_byte_data(self, handle, reg, value):
        self.writes.append((handle, reg & 0xFF, value & 0xFF))
        self.regmap[reg & 0xFF] = value & 0xFF

    def i2c_read_byte_data(self, handle, reg):
        return int(self.regmap.get(reg & 0xFF, 0)) & 0xFF

    def stop(self):
        self.stopped = True


def _install_fake_i2c_driver(fake_pi):
    '''
    Provide a fake `hardware.i2c_driver` module so importing the driver does not
    pull in pigpio or platform-specific deps during Windows test runs.
    '''
    mod = types.ModuleType("hardware.i2c_driver")
    mod.get_i2c_host = lambda _params=None: fake_pi
    sys.modules["hardware.i2c_driver"] = mod


def test_init_writes_expected_registers(monkeypatch):
    fake_pi = FakePiI2C(regmap={MPU_WHO_AM_I: 0x68})
    _install_fake_i2c_driver(fake_pi)

    import hardware.mpu6050_i2c_driver as mpu

    dev = mpu.MPU6050Driver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x68})
    try:
        assert fake_pi.open_calls == [(1, 0x68, 0)]

        wr = [(r, v) for (_h, r, v) in fake_pi.writes]

        assert wr[0] == (MPU_PWR_MGMT_1, 0x00)  # wake
        assert wr[1] == (MPU_CONFIG, 0x03)      # DLPF=44Hz
        assert wr[2] == (MPU_GYRO_CONFIG, 0x00) # ±250 dps
        assert wr[3] == (MPU_ACCEL_CONFIG, 0x00)# ±2g
        assert wr[4] == (MPU_SMPLRT_DIV, 0x07)  # 125 Hz
    finally:
        dev.close()

    assert fake_pi.close_calls == [1]
    assert fake_pi.stopped is True


def test_read_all_axes_big_endian_signed(monkeypatch):
    regmap = {MPU_WHO_AM_I: 0x68}

    # AX=+0x1234, AY=-0x1234, AZ=+1, TEMP ignored,
    # GX=-1, GY=+2, GZ=-32768
    burst = [
        0x12, 0x34,       # AX
        0xED, 0xCC,       # AY (-0x1234)
        0x00, 0x01,       # AZ
        0x00, 0x00,       # TEMP
        0xFF, 0xFF,       # GX (-1)
        0x00, 0x02,       # GY (+2)
        0x80, 0x00,       # GZ (-32768)
    ]
    for i, b in enumerate(burst):
        regmap[MPU_ACCEL_XOUT_H + i] = b

    fake_pi = FakePiI2C(regmap=regmap)
    _install_fake_i2c_driver(fake_pi)

    import hardware.mpu6050_i2c_driver as mpu

    dev = mpu.MPU6050Driver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x68})
    try:
        ax, ay, az, gx, gy, gz = dev.read_all_axes()
        assert ax == 0x1234
        assert ay == -0x1234
        assert az == 1
        assert gx == -1
        assert gy == 2
        assert gz == -32768
    finally:
        dev.close()
