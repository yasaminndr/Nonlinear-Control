import numpy as np
import matplotlib.pyplot as plt
t = np.linspace(0, 10, 1000)

# Reference signal (desired trajectory)
#r = np.vstack([np.sin(t), np.cos(t)]).T  # Example reference trajectory

# Initial conditions
angles = [100 , 20 , 30]
omega = [0.01 , 0 , 0]
d = omega = [0 , 0 , 0]
# Simulation
q_cal = np.zeros((len(t), 2))
omega_cal = np.zeros((len(t), 2))
u1 = np.zeros((len(t), 1))
u2 = np.zeros((len(t), 1))
u3 = np.zeros((len(t), 1))


k = 0.01  # Control gain
dt = 0.07
for i in range(len(t)):
    dt = t[i] - t[i - 1]
    print(dt)