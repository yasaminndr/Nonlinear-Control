import numpy as np
import matplotlib.pyplot as plt

# Inertia matrix (principal axes)
I = np.array([[1.8, 0, 0],
              [0, 1.7, 0],
              [0, 0, 3]], dtype=float)
# Inverse inertia matrix
I_inv = np.linalg.inv(I)

# Simulation parameters
dt = 0.07  # seconds
total_time = 100
steps = int(total_time / dt)

# Control gains
Kp = 0.5
Kd = 0.1

# Initial attitudes (Euler angles in radians)
current_angles = np.array([0.0, 0.0, 0.0], dtype=float)  # roll, pitch, yaw
# Initial angular velocities
current_omega = np.array([0.0, 0.0, 0.0], dtype=float)

# Desired attitude (example target angles in radians)
desired_angles = np.array([np.radians(10), np.radians(5), np.radians(20)], dtype=float)

# Data storage for plotting
attitude_history = {'time': [], 'roll': [], 'pitch': [], 'yaw': []}

for t in np.linspace(0, total_time, steps):
    # Compute error in Euler angles
    angle_error = desired_angles - current_angles

    # Control torque (PD control)
    torque = -Kp * angle_error - Kd * current_omega  # shape (3,)

    # Angular acceleration: I_inv * torque
    alpha = I_inv.dot(torque)  # shape (3,)

    # Update angular velocities
    current_omega += alpha * dt

    # Update Euler angles
    current_angles += current_omega * dt

    # Store data
    attitude_history['time'].append(t)
    attitude_history['roll'].append(np.degrees(current_angles[0]))
    attitude_history['pitch'].append(np.degrees(current_angles[1]))
    attitude_history['yaw'].append(np.degrees(current_angles[2]))

# Plot results
plt.figure(figsize=(12,8))
plt.subplot(3,1,1)
plt.plot(attitude_history['time'], attitude_history['roll'])
plt.ylabel('Roll (degrees)')
plt.grid()

plt.subplot(3,1,2)
plt.plot(attitude_history['time'], attitude_history['pitch'])
plt.ylabel('Pitch (degrees)')
plt.grid()

plt.subplot(3,1,3)
plt.plot(attitude_history['time'], attitude_history['yaw'])
plt.xlabel('Time (s)')
plt.ylabel('Yaw (degrees)')
plt.grid()

plt.tight_layout()
plt.show()