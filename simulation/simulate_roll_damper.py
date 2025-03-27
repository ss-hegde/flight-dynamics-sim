import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import yaml
import sys
from pathlib import Path

# Add root to Python path
sys.path.append(str(Path(__file__).resolve().parents[1]))

from model.simplified_nonlinear_6dof_model import f16_dynamics
from model.trim_analysis import trim_f16
from control.roll_damper import RollDamper


def closed_loop_dynamics(t, x, controller, params):
    da = controller.compute(x)

    # Use trimmed elevator and throttle, rudder = 0
    u_input = np.array([params['de_trim'], da, 0.0, params['dt_trim']])
    return f16_dynamics(x, u_input, params)


def run_simulation():
    # Load parameters
    with open("model/config/f16_params.yaml", "r") as f:
        params = yaml.safe_load(f)

    # Trim
    x_trim, u_trim = trim_f16(params, debug=False)
    params['de_trim'] = u_trim[0]
    params['dt_trim'] = u_trim[3]

    # Add initial roll rate disturbance
    x0 = x_trim.copy()
    x0[3] += np.radians(5)  # 5 deg/s roll rate

    # Initialize controller
    controller = RollDamper(K_p=0.5)

    def dynamics(t, x):
        return closed_loop_dynamics(t, x, controller, params)

    # Simulate
    t_span = (0, 20)
    sol = solve_ivp(dynamics, t_span, x0, t_eval=np.linspace(*t_span, 1000))

    # Plot p and phi
    plt.figure()
    plt.plot(sol.t, np.degrees(sol.y[3]), label='p (deg/s)')
    plt.plot(sol.t, np.degrees(sol.y[6]), label='ϕ (deg)')
    plt.title("Roll Damper Response")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle / Rate")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_simulation()
