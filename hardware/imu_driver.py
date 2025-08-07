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

# import math
import numpy as np
from scipy.signal import butter
import math
import logging
from hardware.spi_driver import SPIBus  # will use mock_spi if needed
from typing import Tuple
from typing import cast

imu_log = logging.getLogger("imu")


def get_lowpass_coefficients(
    order: int, cutoff: float
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Returns (b, a) IIR filter coefficients for a Butterworth low-pass filter.

    Args:
        order: Filter order (e.g., 1 for first-order)
        cutoff: Normalized cutoff frequency (0 to 1, where 1 = Nyquist)

    Returns:
        Tuple of (b coefficients, a coefficients) as NumPy arrays
    """
    coeffs = butter(order, cutoff, btype="low", analog=False, output="ba")
    return cast(Tuple[np.ndarray, np.ndarray], coeffs)


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
            self.cs_pin = 5
            self.spi = SPIBus(cs_pin=self.cs_pin)
            imu_log.info("SPIBus initialized")
        except Exception as e:
            imu_log.warning("SPIBus fallback due to: %s", e)
            self.spi = None
        # ********************************************************************************
        self.sample_rate_hz = iir_params["SAMPLE_RATE_HZ"]
        self.cutoff_freq_hz = iir_params["CUTOFF_FREQ_HZ"]

        nyquist = 0.5 * self.sample_rate_hz
        normal_cutoff = self.cutoff_freq_hz / nyquist
        self.b_iir, self.a_iir = get_lowpass_coefficients(1, normal_cutoff)

        imu_log.info("IIR filter coefficients: b=%s, a=%s", self.b_iir, self.a_iir)
        self.theta_range = controller_params.get("THETA_RANGE_RAD", math.pi / 2)
        self.omega_range = controller_params.get("OMEGA_RANGE_RAD_S", math.pi)
        self.gyro_full_scale_dps = controller_params.get("GYRO_FULL_SCALE_DPS", 250.0)

        self.prev_omega_vals = [0.0]
        self.prev_filtered_vals = [0.0]

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
        raw_y = int.from_bytes(buffer[0:2], "little", signed=True)
        raw_x = int.from_bytes(buffer[2:4], "little", signed=True)
        raw_omega = int.from_bytes(buffer[4:6], "little", signed=True)

        # Compute theta from atan2(x, y) so 0 is at top of circle
        theta_rads = math.atan2(raw_x, raw_y)  # 0 radians at top (y=1)
        theta_norm = theta_rads / (math.pi / 2.0)  # Normalize to [-1.5, +1.5]

        # print(
        #     f"Theta_rads: {theta_rads:.2f}, Theta_norm: {theta_norm:.2f} raw_y: {raw_y}, raw_x: {raw_x}"
        # )
        # Normalize angular velocity
        omega_dps = raw_omega * self.gyro_full_scale_dps / 32768.0
        omega_norm = omega_dps * (math.pi / 180.0) / self.omega_range

        # Optional debug info
        theta_deg = math.degrees(theta_rads)
        imu_log.debug(
            "Theta_norm %.2f Theta: %.2f°, Omega_dps: %.2f°/s | Raw(y=%d, x=%d, ω=%d)",
            theta_norm,
            theta_deg,
            omega_dps,
            raw_y,
            raw_x,
            raw_omega,
        )

        return theta_norm, omega_norm
