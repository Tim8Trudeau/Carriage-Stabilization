# mock_spi.py

from __future__ import annotations
import logging
import tomllib
from test.mocks.simulation import CarriageSimulator, SimParams

imu_log = logging.getLogger("imu_log")

# Mailbox for latest motor command (set by the controller loop / plot script)
_MOTOR_CMD = 0.0
def set_motor_cmd(u: float) -> None:
    global _MOTOR_CMD
    _MOTOR_CMD = max(-1.0, min(1.0, float(u)))

class MockSPIBus:
    """
    Minimal IMU/SPI adapter that exposes imu_read() for the controller.
    Internally steps the physics simulator and emits raw-like bytes:
        [xL,xH, yL,yH, omegaL,omegaH]
    """
    def __init__(self, controller_params: dict | None = None, sim_config_path: str = "sim_config.toml"):
        self.controller_params = dict(controller_params) if controller_params else {}

        # Load simulation config (simulation-only parameters)
        with open(sim_config_path, "rb") as f:
            sc = tomllib.load(f)

        timing = sc.get("simulation", {}).get("timing", {})
        ic = sc.get("simulation", {}).get("initial_conditions", {})
        mech = sc.get("simulation", {}).get("mechanics", {})
        imu = sc.get("simulation", {}).get("imu_model", {})

        self.params = SimParams(
            sample_rate_hz=float(timing.get("SAMPLE_RATE_HZ", 50.0)),
            duration_s=float(timing.get("DURATION_S", 10.0)),
            theta0_rad=float(ic.get("THETA_INITIAL_RAD", 0.0)),
            omega0_rad_s=float(ic.get("OMEGA_INITIAL_RAD_S", 0.0)),
            wheel_radius_m=float(mech.get("WHEEL_RADIUS_M", 0.604)),
            carriage_mass_kg=float(mech.get("CARRIAGE_MASS_KG", 1.0)),
            motor_force_n=float(mech.get("MOTOR_FORCE_N", 8.71)),
            gravity_m_s2=float(mech.get("GRAVITY_M_S2", 9.80665)),
            gyro_fs_rad_s=float(imu.get("GYRO_FS_RAD_S", 4.363)),
            accel_raw_fs=int(imu.get("ACCEL_RAW_FS", 16384)),
            noise_span=int(imu.get("NOISE_SPAN", 0)),
        )

        self.last_theta = 0.0
        self.sim = CarriageSimulator(self.params)
        imu_log.info(
            "MockSPI: dt=%.4f s, steps=%d, mass=%.3f kg, force=%.3f N, g=%.4f m/s^2",
            self.params.dt, self.params.steps, self.params.carriage_mass_kg,
            self.params.motor_force_n, self.params.gravity_m_s2
        )

    def _synthesize(self) -> tuple[int,int,int]:
        # feed latest motor command, then advance
        self.sim.set_motor_cmd(_MOTOR_CMD)
        x_raw, y_raw, omega_raw, theta, _ = self.sim.step()  # θ, ω already returned by step
        self.last_theta = float(theta)  # NEW: remember θ for logging/compare
        return x_raw, y_raw, omega_raw

    # NEW: allow other components to read sim θ
    def get_sim_theta(self) -> float:
        return float(self.last_theta) # match imu sign convention CW+

    def imu_read(self, **_) -> bytes:
        x_raw, y_raw, omega_raw = self._synthesize()
        buf = bytearray(6)
        buf[0:2] = int(x_raw).to_bytes(2, "little", signed=True)
        buf[2:4] = int(y_raw).to_bytes(2, "little", signed=True)
        buf[4:6] = int(omega_raw).to_bytes(2, "little", signed=True)
        return bytes(buf)

# Back-compat export
SPIBus = MockSPIBus
__all__ = ["MockSPIBus", "SPIBus", "set_motor_cmd"]
