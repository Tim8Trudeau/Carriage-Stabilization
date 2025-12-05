import numpy as np
import matplotlib.pyplot as plt

# Carriage + wheel parameters
m = 1.088          # kg
g = 9.80665        # m/s^2
r = 0.622          # m (COM radius)
I = 0.42           # kg*m^2
tau_motor = 16.27  # N*m (two synchronized drive rollers)

# Compute gravity torque
theta = np.linspace(-np.pi, np.pi, 500)
tau_g = m * g * r * np.sin(theta)

# Net torque for CCW and CW correction
tau_net_ccw = tau_motor - tau_g
tau_net_cw = -tau_motor - tau_g

# 1. Torque balance plot
plt.figure(figsize=(10,5))
plt.plot(theta, tau_g, label="Gravity Torque τ_g")
plt.plot(theta, tau_net_ccw, label="Net Torque (CCW Correction)")
plt.plot(theta, tau_net_cw, label="Net Torque (CW Correction)")
plt.axhline(0, color='black')
plt.title("Stability Analysis: Torque Balance")
plt.xlabel("Theta (rad)")
plt.ylabel("Torque (N·m)")
plt.grid()
plt.legend()
plt.show()

# 2. Phase portrait
theta_vals = np.linspace(-np.pi, np.pi, 40)
omega_vals = np.linspace(-4, 4, 40)
T, W = np.meshgrid(theta_vals, omega_vals)

tau_g_field = m * g * r * np.sin(T)
tau_control = -np.sign(T) * tau_motor   # simple bang-bang stabilizer
alpha = (tau_control - tau_g_field) / I

theta_dot = W
omega_dot = alpha

plt.figure(figsize=(10,5))
plt.quiver(T, W, theta_dot, omega_dot, angles="xy")
plt.title("Phase Portrait (theta vs omega)")
plt.xlabel("Theta (rad)")
plt.ylabel("Omega (rad/s)")
plt.grid()
plt.show()

# 3. Correction envelope
plt.figure(figsize=(10,5))
plt.plot(theta, np.abs(tau_g), label="|Gravity Torque|")
plt.axhline(tau_motor, color='green', linestyle='--', label="Motor Limit")
plt.title("Correction Envelope")
plt.xlabel("Theta (rad)")
plt.ylabel("Torque (N·m)")
plt.legend()
plt.grid()
plt.show()
