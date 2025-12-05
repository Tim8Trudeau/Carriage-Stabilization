import numpy as np
import matplotlib.pyplot as plt

m = 1.088
g = 9.80665
r = 0.622
I = 0.42
tau_motor_one = 0.16
n_rollers = 2
r_roller = 0.012
r_wheel = 0.6096

tau_motor = (tau_motor_one * n_rollers / r_roller) * r_wheel   # ~16.27 N*m

theta = np.linspace(-np.pi, np.pi, 500)
tau_g = m * g * r * np.sin(theta)

plt.figure(figsize=(10,5))
plt.plot(theta, tau_g, label="Gravity Torque")
plt.axhline(tau_motor, color='g', linestyle='--', label="Motor Torque Limit")
plt.axhline(-tau_motor, color='g', linestyle='--')
plt.title("Motor vs Gravity Torque")
plt.xlabel("Theta (rad)")
plt.ylabel("Torque (N·m)")
plt.grid()
plt.legend()
plt.show()
