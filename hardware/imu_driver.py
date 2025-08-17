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

from hardware.spi_driver import SPIBus

imu_log = logging.getLogger("imu")


class IMU_Driver:
    """
    Driver to read and normalize data from an LSM6DS3-like IMU via SPI.
    Local axes:
      - X: radial (inward)
      - Y: tangential
    """

    def __init__(self, iir_params, controller_params):
        # NOTE: pigpio vs. mock selection is centralized in SPIBus.
        # We pass controller_params through so that, if SPIBus falls back to
        # MockSPIBus, it can parameterize the simulation from flc_config.toml.
        mock_cfg = controller_params.get("MOCK_SPI", {})
        self.spi = SPIBus(
            spi_channel=controller_params.get("SPI_CHANNEL", 0),
            baud=controller_params.get("SPI_BAUD", 1_000_000),
            io_mode=controller_params.get("SPI_MODE", 0),
            mock_params=mock_cfg,  # <<< pass ONLY the [controller_params.MOCK_SPI] table
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

        # Normalization ranges (tunable from controller_params)
        # theta_range is in radians; omega_range is in rad/s
        self.theta_range = controller_params.get("THETA_RANGE_RAD", 1.5)
        self.omega_range = controller_params.get("OMEGA_RANGE_RAD_S", math.pi)

        # Gyro full-scale in degrees per second (sensor configured to 250 dps)
        self.gyro_full_scale_dps = controller_params.get("GYRO_FULL_SCALE_DPS", 250.0)

        # State for filtered omega (rad/s)
        self.filtered_omega = 0.0

    def read_normalized(self):
        # Read 6 bytes: [Y_L, Y_H, X_L, X_H, GZ_L, GZ_H]
        buffer = bytearray(6)
        # Register 0x00 is a placeholder for the start address; SPIBus handles protocol details.
        self.spi.readfrom_into(0x00, buffer)

        raw_y = int.from_bytes(buffer[0:2], "little", signed=True)
        raw_x = int.from_bytes(buffer[2:4], "little", signed=True)
        raw_omega = int.from_bytes(buffer[4:6], "little", signed=True)

        # theta from accelerometer: atan2(x, y) using local axes
        theta_rads = math.atan2(raw_x, raw_y)
        theta_norm = theta_rads / self.theta_range

        # Gyro counts -> dps using configured full-scale, then -> rad/s
        omega_dps_instant = (raw_omega * self.gyro_full_scale_dps) / 32768.0
        omega_rps_instant = omega_dps_instant * (math.pi / 180.0)

        # 1st-order IIR low-pass filter on omega (rad/s)
        self.filtered_omega += self.alpha * (omega_rps_instant - self.filtered_omega)
        omega_norm = self.filtered_omega / self.omega_range

        # Optional: also compute filtered dps for logging
        omega_dps_filtered = self.filtered_omega * (180.0 / math.pi)

        imu_log.debug(
            "raw_x=%d raw_y=%d raw_omega=%d theta_deg=%.2f "
            "omega_dps=%.2f omega_dps_filt=%.2f",
            raw_x, raw_y,
            raw_omega,
            math.degrees(theta_rads),
            omega_dps_instant,
            omega_dps_filtered
        )

        return theta_norm, omega_norm

    def close(self):
        """
        Close the SPI interface and release resources.
        For pigpio: closes the SPI handle and stops the pigpio connection.
        For mock: does nothing.
        """
        # Delegates to SPIBus; behavior matches the comment above.
        try:
            if hasattr(self, "spi") and self.spi is not None:
                self.spi.close()
        finally:
            self.spi = None
