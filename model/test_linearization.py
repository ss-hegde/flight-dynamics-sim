import numpy as np
import yaml
import matplotlib.pyplot as plt
from trim_analysis import trim_f16_full
from linearization import linearize, extract_longitudinal_lateral
from simplified_nonlinear_6dof_model import f16_dynamics
from numpy.linalg import eigvals
from scipy.signal import StateSpace, step

# Load aircraft parameters
with open("model/config/f16_params.yaml", "r") as f:
    params = yaml.safe_load(f)

# Trim at steady level flight
x_trim, u_trim = trim_f16_full(params, debug=False)

# Linearize full system
A, B = linearize(f16_dynamics, x_trim, u_trim, params)

# Print shape of A and B matrices
print("\nShape of A matrix:", A.shape)
print("Shape of B matrix:", B.shape)

# Eigenvalue analysis - full system
eigvals_A = eigvals(A)
print("\nEigenvalues of A:")
print(np.round(eigvals_A, 5))

# Plot eigenvalues of full A
plt.figure(figsize=(6, 5))
plt.plot(eigvals_A.real, eigvals_A.imag, 'rx')
plt.axvline(0, color='gray', linestyle='--')
plt.xlabel("Real")
plt.ylabel("Imaginary")
plt.title("Full System Eigenvalues")
plt.grid(True)
plt.tight_layout()
plt.show()

# ---------------------------------------------
# Separate into longitudinal and lateral subsystems
(A_long, B_long), (A_lat, B_lat) = extract_longitudinal_lateral(A, B)

# Eigenvalues of subsystems
eig_long = eigvals(A_long)
eig_lat = eigvals(A_lat)

print("\nEigenvalues of Longitudinal A:")
print(np.round(eig_long, 5))

print("\nEigenvalues of Lateral A:")
print(np.round(eig_lat, 5))

# Plot eigenvalues of subsystems
plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.plot(eig_long.real, eig_long.imag, 'bo')
plt.axvline(0, color='gray', linestyle='--')
plt.title("Longitudinal Eigenvalues")
plt.xlabel("Real")
plt.ylabel("Imaginary")
plt.grid(True)

plt.subplot(1, 2, 2)
plt.plot(eig_lat.real, eig_lat.imag, 'go')
plt.axvline(0, color='gray', linestyle='--')
plt.title("Lateral-Directional Eigenvalues")
plt.xlabel("Real")
plt.ylabel("Imaginary")
plt.grid(True)

plt.tight_layout()
plt.show()

# ---------------------------------------------
# Step Response

# Longitudinal: u and theta as outputs
C_long = np.array([
    [1, 0, 0, 0, 0],  # output u (forward speed)
    [0, 0, 0, 1, 0]   # output theta (pitch angle)
])
D_long = np.zeros((2, 2))
sys_long = StateSpace(A_long, B_long, C_long, D_long)

# Lateral: phi and psi as outputs
C_lat = np.array([
    [0, 0, 0, 1, 0, 0],  # output phi (roll)
    [0, 0, 0, 0, 1, 0]   # output psi (yaw)
])
D_lat = np.zeros((2, 2))
sys_lat = StateSpace(A_lat, B_lat, C_lat, D_lat)

# # Simulate step input for each input channel separately
# t = np.linspace(0, 10, 500)

# # Step response for elevator (longitudinal input 0)
# t1, y1 = step(sys_long, T=t, input=0)

# # Step response for rudder (lateral input 1)
# t2, y2 = step(sys_lat, T=t, input=1)

# # Plot longitudinal step response
# plt.figure(figsize=(10, 4))
# plt.plot(t1, y1[:, 0], label='u (m/s)')
# plt.plot(t1, y1[:, 1], label='theta (rad)')
# plt.title("Longitudinal Step Response (Elevator Input)")
# plt.xlabel("Time (s)")
# plt.grid(True)
# plt.legend()
# plt.tight_layout()
# plt.show()

# # Plot lateral step response
# plt.figure(figsize=(10, 4))
# plt.plot(t2, y2[:, 0], label='phi (rad)')
# plt.plot(t2, y2[:, 1], label='psi (rad)')
# plt.title("Lateral Step Response (Rudder Input)")
# plt.xlabel("Time (s)")
# plt.grid(True)
# plt.legend()
# plt.tight_layout()
# plt.show()



