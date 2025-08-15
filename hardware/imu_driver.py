# imu_driver.py

"""
Driver for the inertial measurement unit (IMU).

This module reads from the accelerometer/gyro, applies a low-pass filter, and
outputs normalized values for
theta in range [-1.5, +1.5] and
omega in range [-1.0, +1.0].
The IMU accelerometer is set 2g max and gyro is set to 250dps max.
The IMU is accessed via SPI, and the driver is designed to work on Raspberry Pi.
Mock SPI is used on non-Raspberry Pi platforms for development.
"""

import math
import logging
from hardware.spi_driver import SPIBus  # will use mock_spi if needed

imu_log = logging.getLogger("imu")

class IMU_Driver:
    """
    Driver to read and normalize data from an LS6DS3 sensor via SPI.
    The local axes are defined such that:
      - X-axis is radial (points inward to wheel center)
      - Y-axis is tangential (along the path of motion)
    """
    def __init__(self, iir_params, controller_params):
        """
        Initializes the accelerometer driver and the IIR filter.

        Args:
            iir_params (dict): Dictionary with 'SAMPLE_RATE_HZ' and
                'CUTOFF_FREQ_HZ'.
            controller_params (dict): Dictionary with 'THETA_RANGE_RAD' and
                'OMEGA_RANGE_RAD_S'.
        """
        try:
            from hardware.spi_driver import SPIBus as HW_SPIBus
            self.spi = HW_SPIBus(spi_channel=0, baud=1_000_000, mode=0)
            imu_log.info("SPIBus (pigpio) initialized")
        except Exception as e:
            imu_log.warning("SPIBus fallback to mock: %s", e)
            mock_cfg = controller_params.get("MOCK_SPI", {})
            from test.mocks.mock_spi import MockSPIBus
            self.spi = MockSPIBus(
                spi_channel=0,
                baud=1_000_000,
                mode=0,
                omega_mode=mock_cfg.get("OMEGA_MODE", "constant"),       # "constant" | "noisy"
                omega_raw_base=mock_cfg.get("OMEGA_RAW_BASE", 5000),     # raw counts
                noise_span=mock_cfg.get("NOISE_SPAN", 2000),             # raw counts ±span
            )
        # --- Low-pass filter parameters ---
        self.sample_rate_hz = iir_params["SAMPLE_RATE_HZ"]
        self.cutoff_freq_hz = iir_params["CUTOFF_FREQ_HZ"]

        dt = 1.0 / self.sample_rate_hz
        rc = 1.0 / (2 * math.pi * self.cutoff_freq_hz)
        self.alpha = dt / (rc + dt)

        imu_log.info(
            "LPF initialized: sample_rate=%.2f Hz, cutoff=%.2f Hz, alpha=%.6f",
            self.sample_rate_hz, self.cutoff_freq_hz, self.alpha
        )

        self.filtered_omega = 0.0  # filter state

        self.theta_range = controller_params.get("THETA_RANGE_RAD", math.pi / 2)
        self.omega_range = controller_params.get("OMEGA_RANGE_RAD_S", math.pi)
        self.gyro_full_scale_dps = controller_params.get("GYRO_FULL_SCALE_DPS", 250.0)


    def read_normalized(self):
        """
        Reads 6 bytes of data from the IMU and returns normalized theta and omega.

        Returns:
            tuple: (theta, omega) each as a float in range
             theta [-1.5, 1.5]
             omega [-1.0, 1.0]
        """
        buffer = bytearray(6)
        self.spi.readfrom_into(0x00, buffer)  # type: ignore

        # Extract raw 16-bit signed values
        # By convention if angle(theta) < 0 the rotation error is CCW
        raw_y = int.from_bytes(buffer[0:2], "little", signed=True)
        raw_x = int.from_bytes(buffer[2:4], "little", signed=True)
        raw_omega = int.from_bytes(buffer[4:6], "little", signed=True)

        # Compute theta from atan2(x, y) so 0 is at top of circle
        theta_rads = math.atan2(raw_x, raw_y)  # 0 radians at top (y=1)
        theta_norm = theta_rads / self.theta_range  # Normalize to [-1.5, +1.5]

        # --- RAW -> rad/s ---
        # raw_omega is signed 16-bit with full-scale = GYRO_FULL_SCALE_DPS (e.g., 250 dps)
        # Convert raw -> dps -> rad/s
        omega_dps_instant = (raw_omega * self.gyro_full_scale_dps) / 32768.0
        omega_rps_instant = omega_dps_instant * (math.pi / 180.0)

        # --- First-order low-pass filter in rad/s ---
        self.filtered_omega += self.alpha * (omega_rps_instant - self.filtered_omega)

        # --- Normalize using configured rad/s range ---
        omega_norm = self.filtered_omega / self.omega_range

        # Optional: also compute filtered dps for logging
        omega_dps_filtered = self.filtered_omega * (180.0 / math.pi)

        imu_log.debug(
            "raw_x=%d raw_y=%d raw_omega=%d theta_rad=%.4f theta_deg=%.2f "
            "omega_dps=%.2f omega_dps_filt=%.2f omega_rps_filt=%.4f",
            raw_x, raw_y, raw_omega,
            theta_rads, math.degrees(theta_rads),
            omega_dps_instant, omega_dps_filtered, self.filtered_omega
        )

        return theta_norm, omega_norm
