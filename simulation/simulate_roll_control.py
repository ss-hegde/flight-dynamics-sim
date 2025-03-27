import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import yaml
from pathlib import Path
import sys

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parents[1]))

from model.simplified_nonlinear_6dof_model import f16_dynamics
from model.trim_analysis import trim_f16
from control.pid_roll_control import PIDRollController


def closed_loop_dynamics(t, x, controller, params, phi_command):
    phi = x[6]  # roll angle
    x[3] = np.clip(x[3], -5.0, 5.0)
    p = x[3]    # roll rate

    error = phi_command - phi
    dt = 0.01
    # da = controller.update(error, dt)
    da = np.clip(controller.update(error, dt), -0.3, 0.3)

    # Use trimmed elevator and throttle
    de = params['de_trim']
    dt_val = params['dt_trim']
    dr = 0.0  # rudder fixed for now

    u_input = np.array([de, da, dr, dt_val])

    return f16_dynamics(x, u_input, params)


def run_simulation():
    # Load parameters
    with open("model/config/f16_params.yaml", "r") as f:
        params = yaml.safe_load(f)

    # Trim state
    x_trim, u_trim = trim_f16(params, debug=False)
    params['dt_trim'] = u_trim[3]
    params['de_trim'] = u_trim[0]

    # Initial state: 10° roll to the right
    x0 = x_trim.copy()
    x0[6] += np.radians(10)  # roll disturbance

    # Roll command
    phi_command = x_trim[6]

    # PID controller for roll
    pid = PIDRollController(Kp=0.05, Ki=0.1, Kd=0.5)

    def dynamics(t, x):
        return closed_loop_dynamics(t, x, pid, params, phi_command)

    print("Running simulation...")

    # Run simulation
    t_span = (0, 20)
    sol = solve_ivp(dynamics, t_span, x0, t_eval=np.linspace(*t_span, 1000))

    print("Simulation complete!")

    # Plot roll angle and roll rate
    plt.figure()
    plt.plot(sol.t, np.degrees(sol.y[6]), label='ϕ (deg)')
    plt.plot(sol.t, np.degrees(sol.y[3]), label='p (deg/s)')
    plt.axhline(np.degrees(phi_command), linestyle='--', color='k', label='ϕ_command')
    plt.xlabel("Time (s)")
    plt.ylabel("Angle / Rate")
    plt.legend()
    plt.title("Closed-Loop Roll Control with PID")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_simulation()
