import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import yaml
import sys
from pathlib import Path

# Add root to path
sys.path.append(str(Path(__file__).resolve().parents[1]))

from model.simplified_nonlinear_6dof_model import f16_dynamics
from model.trim_analysis import trim_f16_full
from control.yaw_damper import YawDamper


def closed_loop_dynamics(t, x, controller, params):
    dr = controller.compute(x)

    # Use trimmed elevator and throttle, fixed aileron
    u_input = np.array([params['de_trim'], 0.0, dr, params['dt_trim']])
    return f16_dynamics(x, u_input, params)


def run_simulation():
    with open("model/config/f16_params.yaml", "r") as f:
        params = yaml.safe_load(f)

    x_trim, u_trim = trim_f16_full(params, debug=False)
    params['de_trim'] = u_trim[0]
    params['dt_trim'] = u_trim[3]

    # Initial yaw rate disturbance
    x0 = x_trim.copy()
    x0[5] += np.radians(5)  # 5 deg/s yaw rate

    controller = YawDamper(K_r=4.5)

    def dynamics(t, x):
        return closed_loop_dynamics(t, x, controller, params)

    t_span = (0, 20)
    sol = solve_ivp(dynamics, t_span, x0, t_eval=np.linspace(*t_span, 1000))

    # Plot r and psi
    plt.figure()
    plt.plot(sol.t, np.degrees(sol.y[5]), label='r (deg/s)')
    plt.plot(sol.t, np.degrees(sol.y[8]), label='ψ (deg)')
    plt.title("Yaw Damper Response")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle / Rate")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_simulation()
