import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import yaml
import sys
from pathlib import Path

# Add the 'model' directory to the path
sys.path.append(str(Path(__file__).resolve().parents[1]))


from model.nonlinear_6DOF_aircraft_model import aircraft_dynamics
from flightgear_interface.stream_to_flightgear import stream_simulation_to_fg

# Flag to enable or disable streaming to FlightGear
STREAM_TO_FLIGHTGEAR = True
USE_JOYSTICK = True  # Set to True if you want to use joystick input
PLOT_STATES = True  # Set to True if you want to plot the states

def load_params(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def simulate_open_loop(x0, u, t_span, params):
    def dynamics(t, x):
        return aircraft_dynamics(x, u, params)

    sol = solve_ivp(dynamics, t_span, x0, t_eval=np.linspace(t_span[0], t_span[1], 1000))
    return sol


def plot_states(sol):
    labels = [
        'u (m/s)', 'v (m/s)', 'w (m/s)',
        'p (rad/s)', 'q (rad/s)', 'r (rad/s)',
        'phi (deg)', 'theta (deg)', 'psi (deg)',
        'Longitude (deg)', 'Latitude (deg)', 'h (m)'
    ]

    # Convert phi, theta, psi, longitude, latitude from radians to degrees
    sol_in_degrees = sol.y.copy()
    sol_in_degrees[6:9, :] = np.rad2deg(sol.y[6:9, :])  # phi, theta, psi
    sol_in_degrees[9:11, :] = np.rad2deg(sol.y[9:11, :])  # Longitude, Latitude

    plt.figure(figsize=(15, 12))
    for i in range(12):
        plt.subplot(4, 3, i + 1)
        plt.plot(sol.t, sol_in_degrees[i, :])
        plt.xlabel('Time (s)')
        plt.ylabel(labels[i])
        plt.grid(True)
    plt.suptitle('Open Loop Simulation')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # Load aircraft parameters
    param_path = Path("model/config/general_aircraft_parameters.yaml")
    params = load_params(param_path)

    # Initial state (straight and level flight with small pitch rate disturbance)
    X0 = np.array([
        85.0, 0.0, 0.0,   # u, v, w
        0.0, 0.0, 0.0,    # p, q, r
        0.0, 0.1, 0.0,     # phi, theta, psi
        44.94028*np.pi/180, -73.09747*np.pi/180, 3000*0.2808])  # x, y, z (start at 1000m altitude)

    if USE_JOYSTICK:
        # Initialize joystick input (if using joystick)
        from flightgear_interface.joystick_input import get_joystick_input
        read_inputs = get_joystick_input()
        inputs = read_inputs()
        U = np.array([inputs["aileron"],
                      inputs["elevator"],
                      inputs["rudder"],
                      inputs["throttle"],
                      inputs["throttle"]])
    else:
        # Control inputs (no deflection, medium throttle)
        U = np.array([0.0, 0.1, 0.0, 0.0, 0.0])  # d_eta, d_xi, d_zeta, d_T1, d_T2

    # Simulate for 20 seconds
    t_span = (0, 240)

    # Run simulation
    sol = simulate_open_loop(X0, U, t_span, params)

    # Stream to FlightGear (optional)
    stream_simulation_to_fg(sol.y, sol.t)

    # Plot
    # plot_states(sol)
