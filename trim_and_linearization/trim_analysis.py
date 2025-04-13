import numpy as np
from scipy.optimize import minimize
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
from model.nonlinear_6DOF_aircraft_model import aircraft_dynamics


def trim_aircraft(params, target_u=85.0, initial_guess=None, debug=False):
    """
    Perform trim for level, steady flight at given airspeed.
    Only optimize angle of attack (alpha), elevator (d_eta), and throttle (d_T).
    """
    g = params['gravity']

    if initial_guess is None:
        initial_guess = [0.005, 0.0, 0.7]  # alpha, de, dt

    def trim_cost(x_opt):
        alpha, d_eta, d_T = x_opt

        # Construct longitudinal state
        u = target_u * np.cos(alpha)
        w = target_u * np.sin(alpha)

        x = np.array([
            u, 0.0, w,
            0.0, 0.0, 0.0,
            0.0, alpha, 0.0,
            0.0, 0.0, 3000.0*0.2808
        ])
        u_input = np.array([d_eta, 0.0, 0.0, d_T])

        x_dot = aircraft_dynamics(x, u_input, params)

        # Only minimize u̇, ẇ, q̇
        cost_vector = np.array([x_dot[0], x_dot[2], x_dot[4]])
        cost = np.linalg.norm(cost_vector)

        if debug:
            print(f"[DEBUG] alpha: {alpha:.4f}, d_eta: {d_eta:.4f}, d_T: {d_T:.4f}, "
                  f"u̇: {x_dot[0]:+.4f}, ẇ: {x_dot[2]:+.4f}, q̇: {x_dot[4]:+.4f}, cost: {cost:.6f}")

        return cost

    bounds = [(-0.3, 0.3), (-0.5, 0.5), (0.3, 1.0)]

    result = minimize(trim_cost, initial_guess, bounds=bounds, method='SLSQP')

    if result.success:
        alpha, d_eta, d_T = result.x
        print(f"\n Trim successful!")
        print(f"  → α = {np.degrees(alpha):.2f} deg")
        print(f"  → δeta = {d_eta:.4f}")
        print(f"  → δT = {d_T:.4f}")

        u = target_u * np.cos(alpha)
        w = target_u * np.sin(alpha)

        x_trim = np.array([
            u, 0.0, w,
            0.0, 0.0, 0.0,
            0.0, alpha, 0.0,
            0.0, 0.0, -1000.0
        ])
        u_trim = np.array([d_eta, 0.0, 0.0, d_T])
        return x_trim, u_trim
    else:
        raise RuntimeError(f"Trim failed: {result.message}")



def trim_aircraft_full(params, target_u=150.0, initial_guess=None, debug=False):
    """
    Full 6-DOF trim for steady, level flight.
    Optimizes: [α, φ, v, p, r, δe, δa, δr, δT]
    Minimizes: body-axis accelerations [u̇, v̇, ẇ, ṗ, q̇, ṙ]
    """

    if initial_guess is None:
        # [α, φ, v, p, r, δe, δa, δr, δT]
        initial_guess = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7]

    def trim_cost(x_opt):
        alpha, phi, v, p, r, de, da, dr, dt1, dt2 = x_opt

        # Forward and vertical velocities from alpha
        u = target_u * np.cos(alpha)
        w = target_u * np.sin(alpha)

        # Build full state vector
        x = np.array([
            u, v, w,         # velocities
            p, 0.0, r,       # angular rates (q = 0 for level flight)
            phi, alpha, 0.0, # Euler angles: phi, theta, psi
            0.0, 0.0, -1000.0
        ])
        u_input = np.array([de, da, dr, dt1, dt2])

        x_dot = aircraft_dynamics(x, u_input, params)

        # Minimize body-axis accelerations: u̇, v̇, ẇ, ṗ, q̇, ṙ
        cost_vector = x_dot[[0, 1, 2, 3, 4, 5]]
        cost = np.linalg.norm(cost_vector)

        if debug:
            print(f"[DEBUG] α={np.degrees(alpha):5.2f}°, φ={np.degrees(phi):5.2f}°, v={v:.3f}, p={p:.3f}, r={r:.3f}, "
                  f"δe={de:+.3f}, δa={da:+.3f}, δr={dr:+.3f}, δT={dt:.3f}, ‖ẋ_body‖={cost:.6f}")

        return cost

    # Bounds for optimization variables
    bounds = [
        (-0.3, 0.3),     # alpha (rad)
        (-0.5, 0.5),     # phi (rad)
        (-20.0, 20.0),   # v (m/s)
        (-1.0, 1.0),     # p (rad/s)
        (-1.0, 1.0),     # r (rad/s)
        (-0.5, 0.5),     # elevator
        (-0.5, 0.5),     # aileron
        (-0.5, 0.5),     # rudder
        (0.3, 1.0)       # throttle
    ]

    result = minimize(trim_cost, initial_guess, bounds=bounds, method='SLSQP')

    if result.success:
        alpha, phi, v, p, r, de, da, dr, dt = result.x
        print(f"\n Trim Successful!")
        print(f"  → α  = {np.degrees(alpha):.2f} deg")
        print(f"  → φ  = {np.degrees(phi):.2f} deg")
        print(f"  → v  = {v:.3f} m/s, p = {p:.3f} rad/s, r = {r:.3f} rad/s")
        print(f"  → δe = {de:.4f}, δa = {da:.4f}, δr = {dr:.4f}, δT = {dt:.4f}")

        u = target_u * np.cos(alpha)
        w = target_u * np.sin(alpha)

        x_trim = np.array([
            u, v, w,
            p, 0.0, r,
            phi, alpha, 0.0,
            0.0, 0.0, -1000.0
        ])
        u_trim = np.array([de, da, dr, dt])
        return x_trim, u_trim
    else:
        raise RuntimeError(f"Trim failed: {result.message}")