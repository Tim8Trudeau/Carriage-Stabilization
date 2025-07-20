"""
Driver for the H-Bridge motor controller.

This module converts the FLC's normalized output command into a format
suitable for the hardware motor driver (a 13-bit signed integer) and sends
it over an I2C interface. For development, the I2C bus is mocked.
"""
import logging

# Mock busio for Windows development
try:
    import busio
except (ImportError, NotImplementedError):
    print("Warning: 'busio' not found. Using mock I2C for development.")
    class MockI2C:
        def __init__(self, scl, sda):
            self._last_val = 0
        def writeto(self, address, buffer):
            self._last_val = int.from_bytes(buffer, 'little', signed=True)

    class MockBoard:
        SCL = None
        SDA = None

    busio = type('busio', (), {'I2C': MockI2C})
    board = MockBoard()

motor_log = logging.getLogger('motor')

class MotorDriver:
    """
    Controls the motor by sending commands over I2C.
    
    Attributes:
        i2c (busio.I2C): The I2C bus object for communication.
        i2c_address (int): The I2C address of the motor driver hardware.
    """
    # 13-bit signed integer means values from -4096 to +4095
    MAX_13_BIT_SIGNED = 4095 
    MIN_13_BIT_SIGNED = -4096

    def __init__(self):
        """Initializes the motor driver's I2C interface."""
        self.i2c_address = 0x40  # Example I2C address for motor driver
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            motor_log.info("I2C bus for motor driver initialized.")
        except (NameError, AttributeError, ValueError) as e:
            motor_log.warning("Could not init real I2C bus: %s. Using mock.", e)
            self.i2c = busio.I2C(None, None)

    def set_speed(self, normalized_speed: float):
        """
        Converts normalized speed to a 13-bit integer and sends it to the motor.

        Args:
            normalized_speed (float): The motor command from the FLC, in the
                range [-1.0, 1.0].
        """
        # Clamp the input just in case
        normalized_speed = max(-1.0, min(1.0, normalized_speed))

        # Convert normalized float to 13-bit signed integer
        if normalized_speed >= 0:
            # Scale [0.0, 1.0] to [0, 4095]
            value = int(normalized_speed * self.MAX_13_BIT_SIGNED)
        else:
            # Scale [-1.0, 0.0) to [-4096, 0)
            value = int(normalized_speed * abs(self.MIN_13_BIT_SIGNED))
        
        # Pack the 16-bit signed integer into a 2-byte buffer
        # The hardware is expected to only read the relevant 13 bits
        buffer = value.to_bytes(2, 'little', signed=True)

        try:
            self.i2c.writeto(self.i2c_address, buffer)
            motor_log.debug("Set motor speed: norm=%.4f -> 13bit=%d",
                          normalized_speed, value)
        except Exception as e:
            motor_log.error("Failed to write to motor driver via I2C: %s", e)

