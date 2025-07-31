# imu_driver.py

"""
Driver for the inertial measurement unit (IMU).

This module reads from the accelerometer/gyro, applies a low-pass filter, and
outputs normalized values for theta and omega in range [-1.0, +1.0].

Mock SPI is used on non-Raspberry Pi platforms for development.
"""

# import math
import numpy as np
from scipy.signal import butter
import math
import logging
from hardware.spi_driver import SPIBus  # will use mock_spi if needed

imu_log = logging.getLogger("imu")

from typing import Tuple
from typing import cast


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
        self.theta_range = controller_params.get("THETA_RANGE_RAD", np.pi)
        self.omega_range = controller_params.get("OMEGA_RANGE_RAD_S", 1.0)

        try:
            self.cs_pin = 5
            self.spi = SPIBus(cs_pin=self.cs_pin)
            imu_log.info("SPIBus initialized")
        except Exception as e:
            imu_log.warning("SPIBus fallback due to: %s", e)
            self.spi = None

        fs = iir_params["SAMPLE_RATE_HZ"]
        fc = iir_params["CUTOFF_FREQ_HZ"]
        nyq = 0.5 * fs
        normal_cutoff = fc / nyq

        from typing import Tuple

        self.b_iir, self.a_iir = get_lowpass_coefficients(1, normal_cutoff)

        self.zi = np.zeros((max(len(self.a_iir), len(self.b_iir)) - 1, 3))
        imu_log.info("IIR filter created for fs=%.1f Hz, fc=%.1f Hz", fs, fc)

    def read_normalized(self):
        """
        Reads 6 bytes of data from the IMU and returns normalized theta and omega.

        Returns:
            tuple: (theta, omega) each as a float in range [-1.0, 1.0]
        """
        buffer = bytearray(6)

        self.spi.readfrom_into(0x00, buffer)

        raw_y = int.from_bytes(buffer[0:2], "little", signed=True)
        raw_x = int.from_bytes(buffer[2:4], "little", signed=True)
        raw_omega = int.from_bytes(buffer[4:6], "little", signed=True)

        # Normalize raw accelerometer readings to [-1.0, +1.0]
        norm_x = raw_x / 32768.0
        norm_y = raw_y / 32768.0

        # Calculate theta from atan2(y, x)
        theta = math.atan2(norm_y, norm_x)  # Result in radians, range [-π, π]
        omega = raw_omega / 32768.0 * self.omega_range

        imu_log.debug("Raw Y: %d, Raw X: %d, Raw Omega: %d", raw_y, raw_x, raw_omega)
        imu_log.debug("Theta: %.3f rad, Omega: %.3f rad/s", theta, omega)

        return theta, omega
