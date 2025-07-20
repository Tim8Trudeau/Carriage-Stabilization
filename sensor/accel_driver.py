"""
Driver for the inertial measurement unit (IMU).

This module is responsible for reading data from the accelerometer/gyroscope,
applying a low-pass filter, and calculating the normalized physical values
(theta and omega) required by the FLC. For testing on a non-Raspberry Pi
platform, it simulates the I2C interface and generates mock sensor data.
"""
import logging
import numpy as np
from scipy.signal import butter, lfilter

# Mock busio for Windows development
try:
    import busio
    import board
except (ImportError, NotImplementedError):
    print("Warning: 'busio' or 'board' not found. Using mock I2C for development.")
    class MockI2C:
        def __init__(self, scl, sda):
            print("MockI2C Init")
        def readfrom_into(self, address, buffer):
            # Simulate some noisy sensor data for testing
            # A slow sine wave for position, and its derivative for velocity
            import time
            t = time.time()
            print("time ", t)
            # Raw values would be int16, let's simulate that
            raw_x = int(16384 * np.cos(t * 0.5))
            raw_y = int(16384 * np.sin(t * 0.5))
            raw_z_gyro = int(8192 * np.cos(t * 0.5) * 0.5) # Derivative of sin is cos
            print("raw ", raw_x)
            # Pack into a 6-byte buffer (3x 16-bit signed integers)
            buffer[0:2] = raw_y.to_bytes(2, 'little', signed=True)
            buffer[2:4] = raw_x.to_bytes(2, 'little', signed=True)
            buffer[4:6] = raw_z_gyro.to_bytes(2, 'little', signed=True)
            print("buffer ", buffer)
    class MockBoard:
        SCL = None
        SDA = None

    busio = type('busio', (), {'I2C': MockI2C})
    print("busio", busio)
    board = MockBoard()

sensor_log = logging.getLogger('sensor')

class AccelDriver:
    """
    Handles IMU sensor communication and data processing.
    
    Attributes:
        theta_range (float): The maximum absolute value for theta in radians.
        omega_range (float): The maximum absolute value for omega in rad/s.
        i2c (busio.I2C): The I2C bus object.
        b_iir (np.ndarray): The numerator coefficients for the IIR filter.
        a_iir (np.ndarray): The denominator coefficients for the IIR filter.
        zi (np.ndarray): The initial state for the IIR filter.
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
        self.theta_range = controller_params.get('THETA_RANGE_RAD', np.pi)  # Default to pi radians if missing
        self.omega_range = controller_params.get('OMEGA_RANGE_RAD_S', 1.0) # Default to 1.0 rad/s if missing
        self.i2c_address = 0x68  # Common address for MPU6050

        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            sensor_log.info("I2C bus initialized successfully.")
        except (NameError, AttributeError, ValueError) as e:
            sensor_log.warning("Could not initialize real I2C bus: %s. Using mock.", e)
            self.i2c = busio.I2C(None, None)

        # Setup IIR low-pass filter
        fs = iir_params['SAMPLE_RATE_HZ']
        fc = iir_params['CUTOFF_FREQ_HZ']
        nyq = 0.5 * fs
        normal_cutoff = fc / nyq
        self.b_iir, self.a_iir = butter(1, normal_cutoff, btype='low', analog=False)
        
        # Initial state for 3 channels (x, y, omega)
        self.zi = np.zeros((max(len(self.a_iir), len(self.b_iir)) - 1, 3))
        sensor_log.info("IIR filter created for fs=%.1f Hz, fc=%.1f Hz", fs, fc)

    def _read_raw_data(self) -> tuple[int, int, int]:
        """
        Reads raw X/Y accelerometer and Z gyroscope data from the I2C bus.
        
        Returns:
            Tuple[int, int, int]: A tuple containing raw (y, x, omega_z).
            Note the y,x ordering to match atan2(y,x) convention.
        """
        buffer = bytearray(6)
        try:
            self.i2c.readfrom_into(self.i2c_address, buffer)
            # Data is returned as 16-bit signed big-endian values
            y = int.from_bytes(buffer[0:2], 'little', signed=True)
            x = int.from_bytes(buffer[2:4], 'little', signed=True)
            omega_z = int.from_bytes(buffer[4:6], 'little', signed=True)
            return y, x, omega_z
        except Exception as e:
            sensor_log.error("Failed to read from I2C device: %s", e)
            return 0, 0, 0

    def get_processed_inputs(self) -> tuple[float, float]:
        """
        Reads, filters, processes, and normalizes sensor data.

        This is the main public method for this class. It performs a full
        read-and-process cycle.

        Returns:
            Tuple[float, float]: A tuple containing the normalized (theta, omega)
            values, scaled to the range [-1.0, 1.0].
        """
        raw_y, raw_x, raw_omega = self._read_raw_data()
        
        # Apply IIR filter to the raw data
        raw_vector = np.array([raw_y, raw_x, raw_omega])
        filtered_vector, self.zi = lfilter(self.b_iir, self.a_iir, [raw_vector], axis=0, zi=self.zi)
        filtered_y, filtered_x, filtered_omega = filtered_vector[0]

        # Calculate theta using atan2(y, x)
        # This directly gives the angle in radians from [-pi, +pi]
        theta_rad = np.arctan2(filtered_y, filtered_x)

        # For this project, omega is read directly from the gyro Z-axis.
        # In a real system, you might need to scale this raw value by a
        # sensitivity factor from the datasheet to get deg/s or rad/s.
        # We assume the raw value here corresponds to our operational range.
        
        # Normalize inputs to [-1.0, 1.0]
        # Assuming max raw accelerometer value (for a non-moving axis) is ~16384 (1g)
        # and max raw gyro is ~16384 for the defined omega_range.
        # This is a simplification; a real implementation would use calibrated values.
        norm_theta = np.clip(theta_rad / self.theta_range, -1.0, 1.0)
        
        # A simple scaling for omega, assuming max raw value maps to max range
        # This would need to be tuned based on the sensor's sensitivity setting.
        norm_omega = np.clip(filtered_omega / 16384.0, -1.0, 1.0)

        sensor_log.debug("Raw(y,x,w):(%d,%d,%d) -> Filt(y,x,w):(%.1f,%.1f,%.1f) -> Norm(th,w):(%.3f,%.3f)",
                       raw_y, raw_x, raw_omega,
                       filtered_y, filtered_x, filtered_omega,
                       norm_theta, norm_omega)

        return norm_theta, norm_omega

