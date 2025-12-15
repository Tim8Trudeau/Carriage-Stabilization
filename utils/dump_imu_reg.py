from hardware.LSM6DS3TR_i2c_driver import LSM6DS3TRDriver

REGS = {
    "WHO_AM_I": 0x0F,
    "CTRL1_XL": 0x10,
    "CTRL2_G":  0x11,
    "CTRL3_C":  0x12,
    "CTRL4_C":  0x13,
    "CTRL5_C":  0x14,
    "CTRL6_C":  0x15,
    "CTRL7_G":  0x16,
    "CTRL8_XL": 0x17,
    "CTRL9_XL": 0x18,
    "CTRL10_C": 0x19,
    "STATUS":   0x1E,
}

def main():
    dev = LSM6DS3TRDriver(controller_params={"I2C_BUS":1,"I2C_ADDR":0x6B})
    print("\nLSM6DS3TR Register Dump:\n")
    for name, reg in REGS.items():
        v = dev.read(reg, 1)[0]
        print(f"{name:10s} (0x{reg:02X}): 0x{v:02X}")
    dev.close()

if __name__ == "__main__":
    main()
