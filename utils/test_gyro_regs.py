from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver
import time

CTRL1_XL = 0x10
CTRL2_G  = 0x11
CTRL3_C  = 0x12
CTRL4_C  = 0x13
CTRL10_C = 0x19
STATUS   = 0x1E

dev = LSM6DS3TRDriver(controller_params={"I2C_BUS":1,"I2C_ADDR":0x6B})

print("\nApplying manual gyro enable sequence...\n")

dev.write(CTRL3_C, b"\x44")     # BDU + IF_INC
dev.write(CTRL4_C, b"\x00")     # no sleep
dev.write(CTRL1_XL, b"\x30")    # accel on
dev.write(CTRL2_G,  b"\x30")    # gyro ODR=52Hz
dev.write(CTRL10_C, b"\x07")    # ENABLE GYRO AXES

time.sleep(0.05)

regs = {
    "CTRL2_G": CTRL2_G,
    "CTRL10_C": CTRL10_C,
    "STATUS": STATUS,
}

for name, reg in regs.items():
    val = dev.read(reg, 1)[0]
    print(f"{name:10s} = 0x{val:02X}")

dev.close()
