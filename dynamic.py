import numpy as np
import matplotlib.pyplot as plt

# Quaternion operations
def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_multiply(q1, q2):
    w0, x0, y0, z0 = q1
    w1, x1, y1, z1 = q2
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1
    ])

def quat_to_euler(q):
    w, x, y, z = q
    # Roll
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp) if abs(sinp) < 1 else np.pi/2 * np.sign(sinp)

    # Yaw
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])

# Inertia matrix
I = np.array([[1.8, 0, 0],
              [0, 1.7, 0],
              [0, 0, 3]])

# Simulation parameters
dt = 0.1  # seconds
total_time = 20
steps = int(total_time / dt)

# Control gains
Kp = 0.5
Kd = 0.1

# Initial states
current_quat = np.array([1.0, 0.0, 0.0, 0.0])   # ensure float
current_omega = np.array([0.0, 0.0, 0.0])       # rad/sec

# Desired attitude
desired_quat = np.array([0.9239, 0.0, 0.3827, 0.0])  # example
desired_quat /= np.linalg.norm(desired_quat)  # normalize

# Data for plotting
attitude_history = {'time': [], 'roll': [], 'pitch': [], 'yaw': []}

for t in np.linspace(0, total_time, steps):
    # Compute attitude error quaternion
    q_error = quat_multiply(quat_conjugate(current_quat), desired_quat)
    angle = 2 * np.arccos(np.clip(q_error[0], -1, 1))
    if abs(angle) < 1e-6:
        axis = np.array([0, 0, 0])
    else:
        axis = q_error[1:] / np.sin(angle/2)
    attitude_error = angle * axis

    # Control torque (PD control)
    torque = -Kp * attitude_error - Kd * current_omega

    # Compute angular acceleration: inv(I) * torque
    angular_acceleration = np.linalg.inv(I).dot(torque)

    # Update angular velocity
    current_omega += angular_acceleration * dt

    # Update quaternion
    omega_quat = np.hstack(([0], current_omega))
    dq = 0.5 * quat_multiply(current_quat, omega_quat)
    current_quat += dq * dt
    current_quat /= np.linalg.norm(current_quat)  # normalize

    # Save data for plotting
    euler_angles = quat_to_euler(current_quat)
    attitude_history['time'].append(t)
    attitude_history['roll'].append(np.degrees(euler_angles[0]))
    attitude_history['pitch'].append(np.degrees(euler_angles[1]))
    attitude_history['yaw'].append(np.degrees(euler_angles[2]))

# Plot
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
