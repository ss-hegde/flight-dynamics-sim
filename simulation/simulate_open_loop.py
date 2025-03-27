import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import yaml
import sys
from pathlib import Path

# Add the 'model' directory to the path
sys.path.append(str(Path(__file__).resolve().parents[1]))


from model.simplified_nonlinear_6dof_model import f16_dynamics


def load_params(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def simulate_open_loop(x0, u, t_span, params):
    def dynamics(t, x):
        return f16_dynamics(x, u, params)

    sol = solve_ivp(dynamics, t_span, x0, t_eval=np.linspace(t_span[0], t_span[1], 1000))
    return sol


def plot_states(sol):
    labels = [
        'u (m/s)', 'v (m/s)', 'w (m/s)',
        'p (rad/s)', 'q (rad/s)', 'r (rad/s)',
        'phi (rad)', 'theta (rad)', 'psi (rad)',
        'x (m)', 'y (m)', 'z (m)'
    ]

    plt.figure(figsize=(15, 12))
    for i in range(12):
        plt.subplot(4, 3, i + 1)
        plt.plot(sol.t, sol.y[i, :])
        plt.xlabel('Time (s)')
        plt.ylabel(labels[i])
        plt.grid(True)
    plt.suptitle('Open Loop Simulation (No Control)')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # Load aircraft parameters
    param_path = Path("model/config/f16_params.yaml")
    params = load_params(param_path)

    # Initial state (straight and level flight with small pitch rate disturbance)
    X0 = np.array([
        150.0, 0.0, 0.0,   # u, v, w
        0.0, 0.01, 0.0,    # p, q, r
        0.0, 0.0, 0.0,     # phi, theta, psi
        0.0, 0.0, -1000.0  # x, y, z (start at 1000m altitude)
    ])

    # Control inputs (no deflection, medium throttle)
    U = np.array([0.0, 0.0, 0.0, 0.5])  # d_eta, d_xi, d_zeta, d_T

    # Simulate for 20 seconds
    t_span = (0, 20)

    # Run simulation
    sol = simulate_open_loop(X0, U, t_span, params)

    # Plot
    plot_states(sol)
