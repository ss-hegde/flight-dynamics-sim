import numpy as np

def f16_dynamics(X, U, params):
    """
    Nonlinear 6-DOF F-16 model (simplified)
    
    States:
        X = [u, v, w, p, q, r, phi, theta, psi, x_e, y_e, z_e]
    Inputs:
        U = [d_eta, d_xi, d_zeta, d_T]  (elevator, aileron, rudder, throttle)
    """
    # Unpack state
    u_b, v_b, w_b, p, q, r, phi, theta, psi, x_e, y_e, z_e = X
    d_eta, d_xi, d_zeta, d_T = U
    
    # Unpack parameters
    m = params['mass']
    g = params['gravity']
    Ixx = params['Ixx']
    Iyy = params['Iyy']
    Izz = params['Izz']
    Ixz = params['Ixz']
    S = params['S']
    c̄ = params['c']
    b = params['b']
    rho = params['rho']
    
    # Compute airspeed and angles
    V = np.sqrt(u_b**2 + v_b**2 + w_b**2)
    alpha = np.arctan2(w_b, u_b) if u_b != 0 else 0
    beta = np.arcsin(v_b / V) if V != 0 else 0
    
    # Aerodynamic coefficients (simplified aerodynamic model)
    CL = 0.5 * alpha + 4.0 * d_eta
    CD = 0.02 + 0.3 * alpha**2
    CY = 0.1 * beta + 0.5 * d_zeta
    
    # Cl = 0.1 * d_xi
    Cl = 2.0 * d_xi - 0.3 * p  # roll moment from aileron and damping from roll rate
    Cm = -1.2 * alpha + 1.5 * d_eta
    Cn = -0.1 * beta + 0.3 * d_zeta
    
    q_bar = 0.5 * rho * V**2
    L = Cl * q_bar * S * b
    M = Cm * q_bar * S * c̄
    N = Cn * q_bar * S * b
    
    # Forces in body frame
    X = -CD * q_bar * S + d_T * 10000  # Simplified thrust model
    Y = CY * q_bar * S
    Z = -CL * q_bar * S
    
    # Translational accelerations
    u_dot = r * v_b - q * w_b + X / m
    v_dot = p * w_b - r * u_b + Y / m
    w_dot = q * u_b - p * v_b + Z / m + g * np.sin(theta)
    
    # Rotational dynamics (simplified)
    p_dot = (L - (Izz - Iyy) * q * r) / Ixx
    q_dot = (M - (Ixx - Izz) * p * r) / Iyy
    r_dot = (N - (Iyy - Ixx) * p * q) / Izz
    
    # Euler angles
    phi_dot = p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
    theta_dot = q * np.cos(phi) - r * np.sin(phi)
    psi_dot = q * np.sin(phi) / np.cos(theta) + r * np.cos(phi) / np.cos(theta)
    
    # Position update in Earth frame (optional)
    x_dot = u_b
    y_dot = v_b
    z_dot = w_b

    return np.array([
        u_dot, v_dot, w_dot,
        p_dot, q_dot, r_dot,
        phi_dot, theta_dot, psi_dot,
        x_dot, y_dot, z_dot
    ])
