import numpy as np
import matplotlib.pyplot as plt
tau_desired=[0,0.05,3]
max_torque=0.1
tau_allocated = np.clip(tau_desired, -max_torque, max_torque)
saturated = np.abs(tau_desired) > max_torque
free =~ saturated 
weights = [0,0,0]      
if np.any(free) and np.any(saturated):
    tau_residual = tau_desired - tau_allocated
    total_extra = np.sum(tau_residual)
    weights = np.zeros_like(tau_desired, dtype=float)
    weights[free] = 1.0 / np.sum(free)
    tau_allocated += total_extra*weights
    tau_allocated = np.clip(tau_allocated, -max_torque, max_torque)
a=0.5/ np.sum(free)
print(a)

# def quat_to_euler(euler):
#     roll = np.deg2rad(euler[0])
#     pitch = np.deg2rad(euler[1])
#     yaw = np.deg2rad(euler[2])
#     cy = np.cos(yaw * 0.5)
#     sy = np.sin(yaw * 0.5)
#     cp = np.cos(pitch * 0.5)
#     sp = np.sin(pitch * 0.5)
#     cr = np.cos(roll * 0.5)
#     sr = np.sin(roll * 0.5)

#     w = cr * cp * cy + sr * sp * sy
#     x = sr * cp * cy - cr * sp * sy
#     y = cr * sp * cy + sr * cp * sy
#     z = cr * cp * sy - sr * sp * cy
#     Q= [x, y, z, w]

#     return Q


# def q_cal(Q_pervious, dt, omega):
#     q_vector_pervious = [Q_pervious[0], Q_pervious[1], Q_pervious[2]]
#     q4_pervious = Q_pervious[3]
#     x = q_vector_pervious
    
#     # Omega matrix (skew-symmetric form)
#     q_cross = np.array([
#         [0., -x[2] , x[1]],
#         [x[2], 0., -x[0]],
#         [-x[1], x[0], 0]
#     ])
#     # Quaternion derivative: q_dot = 0.5 * Omega * q
#     q_vector_dot =0.5 *  np.dot((q4_pervious * np.eye(3) + q_cross) , omega)
#     q_4_dot = -0.5 * np.dot(np.transpose(q_vector_pervious) , omega)
#     q_vector_new = (dt * q_vector_dot) + q_vector_pervious
#     q4_new = (dt * q_4_dot) + q4_pervious
#     Q_new = np.concatenate((q_vector_new , q4_new) , axis = None)
#     return Q_new

# def q_dot(Q_pervious, omega):
#     q_vector_pervious = [Q_pervious[0], Q_pervious[1], Q_pervious[2]]
#     q4_pervious = Q_pervious[3]
#     x = q_vector_pervious
    
#     # Omega matrix (skew-symmetric form)
#     q_cross = np.array([
#         [0., -x[2] , x[1]],
#         [x[2], 0., -x[0]],
#         [-x[1], x[0], 0]
#     ])
#     # Quaternion derivative: q_dot = 0.5 * Omega * q
#     q_vector_dot =0.5 *  np.dot((q4_pervious * np.eye(3) + q_cross) , omega)
#     return q_vector_dot

# def Omega_dot(dt, J,  torque, disturbace, omega):
#     omega = np.array(omega)
#     x = omega
#     omega_cross = np.array([
#     [0., -x[2] , x[1]],
#     [x[2], 0., -x[0]],
#     [-x[1], x[0], 0]
#     ])
#     invJ =  np.linalg.inv(J)          
#     omega_dot = np.dot(invJ , (- np.dot( omega_cross , np.dot( J , omega)) + torque + disturbace))
#     omega_new = omega + dt * omega_dot
#     return omega_dot , omega_new

# def nonlinear_sliding_surface(Q_pervious, q_vector_dot, omega):
#     q_vector = np.array([Q_pervious[0], Q_pervious[1], Q_pervious[2]])
#     # Calculate error and its derivative
#     e = q_vector
#     de = q_vector_dot
#     # Nonlinear sliding surface
#     #s = de + np.tanh(e)
#     s = omega + q_vector + q_vector_dot
#     return s

# def controller( k, s):
#     s = s.astype(float)  # ensure float
#     u = -k * np.sign(s) * s
#     u = np.clip(u, -0.123, 0.123)
#     return u

# J = np.array([[1.8140, -0.1185, 0.0275],
#               [-0.11185, 1.7350, 0.0169],
#               [0.0274, 0.0169, 3.4320],
#               ])
# angles_initial = [100 , 20 , 30]
# omega = [0.01 , 0 , 0]
# d = [0 , 0 , 0]
# k = 0.07
# dt = 0.07
# Q_pervious = quat_to_euler(angles_initial )
# qv_dot = q_dot(Q_pervious, omega)
# q_vector = [Q_pervious[0], Q_pervious[1], Q_pervious[2]]
# print(q_vector)
# s = nonlinear_sliding_surface(Q_pervious, qv_dot,omega)
# # print(s)
# # u = controller(k , s)
# # (omega_dot , omega) = Omega_dot(dt , J , u , d, omega)
# # Q_new  = q_cal(Q_pervious , dt , omega)
# # for i in range(0,0.5,0.1):

# #     print(i)

# # qv_dot = sat.q_dot(Q_pervious, omega)
# #     s = sat.nonlinear_sliding_surface(Q_pervious, qv_dot)
# #     u = sat.controller(k , s)
# #     (omega_dot , omega) = sat.Omega_dot(dt , J , u , d, omega)
# #     Q_new  = sat.q_dot(Q_pervious , dt , omega)
# #     Q_pervious = Q_new 