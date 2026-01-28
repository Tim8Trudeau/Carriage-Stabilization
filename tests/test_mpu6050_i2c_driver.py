import pytest


def _set_14byte_block(fake_host, bus, addr, start_reg, data14: bytes):
    regmap = fake_host.regs.setdefault((bus, addr), {})
    for i, b in enumerate(data14):
        regmap[(start_reg + i) & 0xFF] = b


def test_init_sequence_writes_expected_registers(install_fake_i2c_host):
    fake = install_fake_i2c_host
    import hardware.mpu6050_i2c_driver as mpu

    # WHO_AM_I must read 0x68 (default). Populate register for our fake.
    fake.regs[(1, 0x68)] = {mpu.MPU_WHO_AM_I: 0x68}

    d = mpu.MPU6050Driver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x68})

    # Verify that init wrote key configuration registers (order not critical)
    written_regs = {(reg, val) for (_h, reg, val) in fake.writes}

    assert (mpu.MPU_PWR_MGMT_1, 0x00) in written_regs
    assert (mpu.MPU_CONFIG, 0x03) in written_regs
    assert (mpu.MPU_GYRO_CONFIG, 0x00) in written_regs
    assert (mpu.MPU_ACCEL_CONFIG, 0x00) in written_regs
    assert (mpu.MPU_SMPLRT_DIV, 0x07) in written_regs

    d.close()
    assert fake.stopped is True


def test_read_all_axes_parses_big_endian(install_fake_i2c_host):
    fake = install_fake_i2c_host
    import hardware.mpu6050_i2c_driver as mpu

    fake.regs[(1, 0x68)] = {mpu.MPU_WHO_AM_I: 0x68}

    d = mpu.MPU6050Driver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x68})

    # Construct a 14-byte block:
    # AX=0x0102 (258), AY=0xFFFE (-2), AZ=0x7FFF (32767), TEMP ignored,
    # GX=0x8000 (-32768), GY=0x0001 (1), GZ=0x1234 (4660)
    block = bytes(
        [
            0x01,
            0x02,  # AX
            0xFF,
            0xFE,  # AY
            0x7F,
            0xFF,  # AZ
            0x00,
            0x00,  # TEMP
            0x80,
            0x00,  # GX
            0x00,
            0x01,  # GY
            0x12,
            0x34,  # GZ
        ]
    )
    _set_14byte_block(fake, 1, 0x68, mpu.MPU_ACCEL_XOUT_H, block)

    ax, ay, az, gx, gy, gz = d.read_all_axes()
    assert ax == 0x0102
    assert ay == -2
    assert az == 32767
    assert gx == -32768
    assert gy == 1
    assert gz == 0x1234

    d.close()


def test_read_block_reads_byte_by_byte(install_fake_i2c_host):
    fake = install_fake_i2c_host
    import hardware.mpu6050_i2c_driver as mpu

    fake.regs[(1, 0x68)] = {mpu.MPU_WHO_AM_I: 0x68}
    d = mpu.MPU6050Driver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x68})

    # Set a small pattern at 0x3B..0x3B+13
    pattern = bytes(range(14))
    _set_14byte_block(fake, 1, 0x68, mpu.MPU_ACCEL_XOUT_H, pattern)

    _ = d.read_all_axes()

    # Ensure we performed 14 byte reads from successive registers
    read_regs = [
        reg
        for (_h, reg) in fake.reads
        if reg >= mpu.MPU_ACCEL_XOUT_H and reg < mpu.MPU_ACCEL_XOUT_H + 14
    ]
    assert read_regs == list(range(mpu.MPU_ACCEL_XOUT_H, mpu.MPU_ACCEL_XOUT_H + 14))

    d.close()


def test_write_error_raises_runtimeerror(monkeypatch, fake_i2c_host):
    import hardware.mpu6050_i2c_driver as mpu

    # Driver now calls hardware.mpu6050_i2c_driver.i2c_driver.get_i2c_host(...)
    # Patch that entrypoint (patch where it's USED).
    monkeypatch.setattr(mpu.i2c_driver, "get_i2c_host", lambda params=None: fake_i2c_host)

    # WHO_AM_I ok
    fake_i2c_host.regs[(1, 0x68)] = {mpu.MPU_WHO_AM_I: 0x68}

    # Make writes fail after open
    def boom(*args, **kwargs):
        raise Exception("nope")

    monkeypatch.setattr(fake_i2c_host, "i2c_write_byte_data", boom)

    with pytest.raises(RuntimeError) as ei:
        mpu.MPU6050Driver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x68})
    assert "MPU6050 I2C write failed" in str(ei.value)


def test_close_idempotent(install_fake_i2c_host):
    fake = install_fake_i2c_host
    import hardware.mpu6050_i2c_driver as mpu

    fake.regs[(1, 0x68)] = {mpu.MPU_WHO_AM_I: 0x68}
    d = mpu.MPU6050Driver(controller_params={"I2C_BUS": 1, "I2C_ADDR": 0x68})

    d.close()
    d.close()  # should not raise
    assert fake.stopped is True
