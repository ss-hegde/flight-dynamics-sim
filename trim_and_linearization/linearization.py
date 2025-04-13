import numpy as np
from simplified_nonlinear_6dof_model import f16_dynamics


def linearize(f, x_trim, u_trim, params, epsilon=1e-5):
    """
    Linearize the nonlinear system at the trim point using finite differences.
    Returns A, B matrices.
    """
    n_states = len(x_trim)
    n_inputs = len(u_trim)

    A = np.zeros((n_states, n_states))
    B = np.zeros((n_states, n_inputs))

    f0 = f(x_trim, u_trim, params)

    # Compute A matrix (df/dx)
    for i in range(n_states):
        dx = np.zeros_like(x_trim)
        dx[i] = epsilon
        f_plus = f(x_trim + dx, u_trim, params)
        A[:, i] = (f_plus - f0) / epsilon

    # Compute B matrix (df/du)
    for i in range(n_inputs):
        du = np.zeros_like(u_trim)
        du[i] = epsilon
        f_plus = f(x_trim, u_trim + du, params)
        B[:, i] = (f_plus - f0) / epsilon

    return A, B


def extract_longitudinal_lateral(A, B):
    # Longitudinal dynamics
    long_state_idx = [0, 2, 4, 7, 11]  # u, w, q, theta, z_e
    long_input_idx = [0, 3]            # delta_elevator, throttle

    A_long = A[np.ix_(long_state_idx, long_state_idx)]
    B_long = B[np.ix_(long_state_idx, long_input_idx)]

    # Lateral dynamics
    lat_state_idx = [1, 3, 5, 6, 8, 10]  # v, p, r, phi, psi, y_e
    lat_input_idx = [1, 2]               # delta_aileron, delta_rudder

    A_lat = A[np.ix_(lat_state_idx, lat_state_idx)]
    B_lat = B[np.ix_(lat_state_idx, lat_input_idx)]

    return (A_long, B_long), (A_lat, B_lat)