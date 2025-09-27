import os
os.system('cls')
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
class SpacecraftDynamic:

    def q_cal(self , Q_pervious, dt, omega):
        q_vector_pervious = [Q_pervious[0], Q_pervious[1], Q_pervious[2]]
        q4_pervious = Q_pervious[3]
        x = q_vector_pervious
       
        # Omega matrix (skew-symmetric form)
        q_cross = np.array([
            [0., -x[2] , x[1]],
            [x[2], 0., -x[0]],
            [-x[1], x[0], 0]
        ])
        # Quaternion derivative: q_dot = 0.5 * Omega * q
        q_vector_dot =0.5 *  np.dot((q4_pervious * np.eye(3) + q_cross) , omega)
        q_4_dot = -0.5 * np.dot(np.transpose(q_vector_pervious) , omega)
        q_vector_new = (dt * q_vector_dot) + q_vector_pervious
        q4_new = (dt * q_4_dot) + q4_pervious
        Q_new = np.concatenate((q_vector_new , q4_new) , axis = None)
        return Q_new

    def q_dot(self , Q_pervious, omega):
        q_vector_pervious = [Q_pervious[0], Q_pervious[1], Q_pervious[2]]
        q4_pervious = Q_pervious[3]
        x = q_vector_pervious
       
        # Omega matrix (skew-symmetric form)
        q_cross = np.array([
            [0., -x[2] , x[1]],
            [x[2], 0., -x[0]],
            [-x[1], x[0], 0]
        ])
        # Quaternion derivative: q_dot = 0.5 * Omega * q
        q_vector_dot =0.5 *  np.dot((q4_pervious * np.eye(3) + q_cross) , omega)
        return q_vector_dot

    def Omega_dot(self, dt, J,  torque, disturbace, omega):
        omega = np.array(omega)
        x = omega
        omega_cross = np.array([
        [0., -x[2] , x[1]],
        [x[2], 0., -x[0]],
        [-x[1], x[0], 0]
        ])
        invJ =  np.linalg.inv(J)          
        omega_dot = np.dot(invJ , (- np.dot( omega_cross , np.dot( J , omega)) + torque + disturbace))
        omega_new = omega + dt * omega_dot
        return omega_dot , omega_new
    
    def nonlinear_sliding_surface(self, Q_pervious, q_vector_dot, omega):
        q_vector = np.array([Q_pervious[0], Q_pervious[1], Q_pervious[2]])
        # Calculate error and its derivative
        e = q_vector
        de = q_vector_dot
        # Nonlinear sliding surface
        #s = de + np.tanh(e)
        s = omega + 1.5 * e
        return s
    
    def sat(x):
        return np.clip(x,-1,1)
    

    def controller(self, k, s):
        s = s.astype(float)  # ensure float
        #u = -k * np.sign(s
        x = s/0.01
        u = -k * np.clip(x,-1,1)

        #u = np.clip(u, -0.123, 0.123)
        return u
    def sat_re(self,tau_desired, max_torque,margin):
        tau_allocated = np.clip(tau_desired, -max_torque, max_torque)
        saturated = np.abs(tau_desired) > max_torque
        free =~ saturated 
        weights = [0,0,0]      
        if np.any(free) and np.any(saturated):
            tau_residual = tau_desired - tau_allocated
            total_extra = np.sum(tau_residual)
            weights = np.zeros_like(tau_desired, dtype=float)
            weights[free] = margin/ np.sum(free)
            tau_allocated += total_extra*weights
            tau_allocated = np.clip(tau_allocated, -max_torque, max_torque)
        return tau_allocated, weights
    

J = np.array([[1.8140, -0.1185, 0.0275],
              [-0.11185, 1.7350, 0.0169],
              [0.0274, 0.0169, 3.4320],
              ])



# Time vector
t = np.linspace(0, 100, 10000)

# Reference signal (desired trajectory)
#r = np.vstack([np.sin(t), np.cos(t)]).T  # Example reference trajectory

# Initial conditions
angles_initial = [15 , 10 , 30]
omega = [0.01 , 0 , 0]

rotation = R.from_euler('xyz', angles_initial, degrees=True)
Q_initial = rotation.as_quat()
Q_pervious = Q_initial
d = [0 , 0 , 0]
# Simulation
q_cal = np.zeros((len(t), 2))
omega_cal = np.zeros((len(t), 2))
u1 = np.zeros((len(t), 1))
u2 = np.zeros((len(t), 1))
u3 = np.zeros((len(t), 1))
max_torque = 0.123
dt = t[1] - t[0]
k = 0.2 # Control gain
margin= 1 # allocating gain
attitude_history = {'time': [], 'roll': [], 'pitch': [], 'yaw': []}
Angular_velocity_history = {'time': [], 'omega_x': [], 'omega_y': [], 'omega_z': []}
Control_input_history = {'time': [], 'u_1': [], 'u_2': [], 'u_3': []}
Saturated= {'time': [], 's_1': [], 's_2': [], 's_3': []}

# for k in np.arange(-5, 0, 1):
#     print(k)
sat = SpacecraftDynamic()
for i in range(len(t)):
    qv_dot = sat.q_dot(Q_pervious, omega)

    s = sat.nonlinear_sliding_surface(Q_pervious, qv_dot, omega)
    tau = sat.controller(k , s)
    u , weights = sat.sat_re(tau , max_torque, margin)
    (omega_dot , omega) = sat.Omega_dot(dt , J , u , d, omega)
    Q_new  = sat.q_cal(Q_pervious , dt , omega)
    Q_pervious = Q_new 
    rotation = R.from_quat(Q_new)
    euler_angles = rotation.as_euler('xyz', degrees=True)
    attitude_history['time'].append(t[i])
    attitude_history['roll'].append(np.degrees(euler_angles[0]))
    attitude_history['pitch'].append(np.degrees(euler_angles[1]))
    attitude_history['yaw'].append(np.degrees(euler_angles[2]))


    Angular_velocity_history['time'].append(t[i])
    Angular_velocity_history['omega_x'].append(omega[0])
    Angular_velocity_history['omega_y'].append(omega[1])
    Angular_velocity_history['omega_z'].append(omega[2])

    Control_input_history['time'].append(t[i])
    Control_input_history['u_1'].append(u[0])
    Control_input_history['u_2'].append(u[1])
    Control_input_history['u_3'].append(u[2])

    Saturated['time'].append(t[i])
    Saturated['s_1'].append(weights[0])
    Saturated['s_2'].append(weights[1])
    Saturated['s_3'].append(weights[2])


    u1[i] = u[0]
    u2[i] = u[1]
    u3[i] = u[2]


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
plt.title("First Figure")
plt.tight_layout()


plt.figure(figsize=(12,8))
plt.subplot(3,1,1)
plt.plot(Control_input_history['time'], Control_input_history['u_1'])
plt.ylabel('u1 ')
plt.grid()
plt.subplot(3,1,2)
plt.plot(Control_input_history['time'], Control_input_history['u_2'])
plt.ylabel('u2 ')
plt.grid()
plt.subplot(3,1,3)
plt.plot(Control_input_history['time'], Control_input_history['u_3'])
plt.xlabel('Time (s)')
plt.ylabel('u3 ')
plt.grid()
plt.tight_layout()
plt.title("third Figure")
plt.show()

plt.figure(figsize=(12,8))
plt.subplot(3,1,1)
plt.plot(Angular_velocity_history['time'], Angular_velocity_history['omega_x'])
plt.ylabel('omega1 ')
plt.grid()
plt.subplot(3,1,2)
plt.plot(Angular_velocity_history['time'], Angular_velocity_history['omega_y'])
plt.ylabel('omega2 ')
plt.grid()
plt.subplot(3,1,3)
plt.plot(Angular_velocity_history['time'], Angular_velocity_history['omega_z'])
plt.xlabel('Time (s)')
plt.ylabel('omega3 ')
plt.grid()
plt.tight_layout()
plt.title("second Figure")
plt.show()

plt.figure(figsize=(12,8))
plt.subplot(3,1,1)
plt.plot(Saturated['time'], Saturated['s_1'])
plt.ylabel('s1 ')
plt.grid()
plt.subplot(3,1,2)
plt.plot(Saturated['time'], Saturated['s_2'])
plt.ylabel('s2 ')
plt.grid()
plt.subplot(3,1,3)
plt.plot(Saturated['time'], Saturated['s_3'])
plt.xlabel('Time (s)')
plt.ylabel('s3 ')
plt.grid()
plt.tight_layout()
plt.title("fourth Figure")
plt.show()
