# hardware/imu_driver.py

import math
import logging
from typing import Tuple

from hardware.spi_driver import SPIBus
try:
    from hardware.LSM6DS3TR_driver import LSM6DS3TRDriver as IMUDev
    REAL_HW = True
except Exception:
    # In tests, import the mock directly
    from test.mocks.mock_LSM6DS3TR_driver import MockLSM6DS3TRDriver as IMUDev
    REAL_HW = False

# Set up logging
#logging.basicConfig(level=logging.DEBUG if REAL_HW else logging.INFO)
imu_log = logging.getLogger("imu")


class IMU_Driver:
    """
    Reads 6 bytes from the IMU via SPI (little-endian, signed 16-bit):
        raw_x (accel), raw_y (accel), raw_omega (gyro)
    Converts to:
        theta_rads = atan2(raw_x, raw_y)
        theta_norm in [-1, +1] via division by theta_range_rad (default π)
        omega_norm in [-1, +1] via division by 32768.0
    Applies a 1st-order low-pass filter to omega_norm.
    Also computes omega_rps (rad/s) for logging.
    """

    def __init__(self, iir_params: dict, controller_params: dict) -> None:
        if REAL_HW:
            import pigpio
            pi = pigpio.pi()
            self.imu = IMUDev(pi, spi_channel=self.controller_params.get("SPI_CHANNEL", 0),
                                spi_baud=self.controller_params.get("SPI_BAUD", 1_000_000),
                                spi_flags=self.controller_params.get("SPI_FLAGS", 0))
        else:
            self.imu = IMUDev()  # mock self-loads MOCK_SPI from flc_config.toml

        # Save params for later use
        self.controller_params = dict(controller_params) if controller_params else {}
        self.iir_params = dict(iir_params) if iir_params else {}

        # SPI & register addresses
        self.status_reg = self.controller_params.get("IMU_STAT_REG", 0x1E)
        self.data_reg = self.controller_params.get("IMU_DATA_REG", 0x22)
        self.spi = SPIBus(self.controller_params)

        # Normalization ranges
        # Theta is normalized by this range (default π for [-1,+1] mapping)
        self.theta_range_rad = float(self.controller_params.get("THETA_RANGE_RAD", math.pi))

        # Gyro full-scale (rad/s) used to convert normalized omega to physical units for logging
        # Prefer the config value; it should reflect the sensor mode (e.g., 250 dps ≈ 4.363 rad/s)
        self.gyro_full_scale_rps = float(
            self.controller_params.get("GYRO_FULL_SCALE_RADS_S", 4.363)
        )

        # (Accel full-scale is not required for atan2; same scale cancels out.
        # Kept for completeness if needed elsewhere.)
        self.accel_full_scale_g = float(
            self.controller_params.get("ACCEL_FULL_SCALE_G", 2.0)
        )

        # IIR low-pass setup for omega_norm
        self.sample_rate_hz = float(self.iir_params.get("SAMPLE_RATE_HZ", 50.0))
        self.cutoff_freq_hz = float(self.iir_params.get("CUTOFF_FREQ_HZ", 5.0))

        dt = 1.0 / self.sample_rate_hz
        rc = 1.0 / (2.0 * math.pi * self.cutoff_freq_hz)
        self.alpha = dt / (rc + dt)

        # State: filtered omega in RAW units (LSBs).  <-- CHANGED
        # We filter in raw units for speed/precision, then scale to norm/rad/s after.
        self._omega_raw_filt = 0.0
        self._OMEGA_FS = 32768  # LSB full-scale for ±250 dps setting

        imu_log.info(
            "IMU_Driver init: sample_rate=%.3f Hz, cutoff=%.3f Hz, alpha=%.6f, "
            "theta_range_rad=%.4f, gyro_full_scale_rps=%.4f",
            self.sample_rate_hz, self.cutoff_freq_hz, self.alpha,
            self.theta_range_rad, self.gyro_full_scale_rps
        )

    def read_normalized(self) -> Tuple[float, float]:
        """
        Returns:
            theta_norm (float): normalized theta in [-1.0, +1.0]
            omega_norm_filt (float): low-pass filtered normalized omega in [-1.0, +1.0]
        """
        # Read 6 bytes starting at the IMU data register.
        # Expect SPIBus to support readfrom_into(register, buffer).
        buf = bytearray(6)
        self.imu.readfrom_into(0x22, buf)  # same register you used before

        # Unpack little-endian signed 16-bit values: x, y, gyro
        raw_x = int.from_bytes(buf[0:2], "little", signed=True)
        raw_y = int.from_bytes(buf[2:4], "little", signed=True)
        raw_omega = int.from_bytes(buf[4:6], "little", signed=True)

        # Theta (radians), then normalized by theta_range_rad (default π), clipped
        theta_rads = math.atan2(raw_x, raw_y)
        theta_norm = max(-1.0, min(1.0, theta_rads / self.theta_range_rad))

        # IIR on RAW omega (LSBs).
        self._omega_raw_filt += self.alpha * (float(raw_omega) - self._omega_raw_filt)

        # Scale *after* the filter
        omega_norm = float(self._omega_raw_filt / self._OMEGA_FS)
        omega_norm = max(-1.0, min(1.0, omega_norm))

        omega_rps = omega_norm * self.gyro_full_scale_rps
        imu_log.debug(
            "raw_x= %d raw_y= %d raw_omega= %d | "
            "theta_deg= %.2f theta_norm= %.4f | "
            "omega_raw_filt= %.2f omega_norm= %.4f omega_rps= %.4f",
            raw_x, raw_y, raw_omega,
            math.degrees(theta_rads), theta_norm,
            self._omega_raw_filt, omega_norm, omega_rps
        )
        return theta_norm, omega_norm

    def close(self) -> None:
        """Release SPI resources."""
        try:
            if getattr(self, "spi", None) is not None:
                self.spi.close()
        finally:
            self.spi = None
