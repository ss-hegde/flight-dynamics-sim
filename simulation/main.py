import numpy as np
import time
import matplotlib.pyplot as plt
import yaml
import sys
from pathlib import Path

# Add 'model' directory to path
sys.path.append(str(Path(__file__).resolve().parents[1]))
from model.nonlinear_6DOF_aircraft_model import aircraft_dynamics
# from flightgear_interface.stream_to_flightgear import stream_simulation_to_fg
from flightgear_interface.stream_to_flightgear import FGStreamer
from flightgear_interface.joystick_input import get_joystick_input
from governing_equations.rk4_integration import rk4_step
from simulation.live_plot import LivePlotter



# Flags
USE_JOYSTICK = True
PLOT_STATES = False
STREAM_TO_FLIGHTGEAR = True

def load_params(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

if __name__ == "__main__":
    fg = FGStreamer()
    # Load aircraft parameters
    param_path = Path("model/config/general_aircraft_parameters.yaml")
    params = load_params(param_path)

    # Initial state
    x = np.array([
        85.0, 0.0, 0.0,   # u, v, w
        0.0, 0.0, 0.0,    # p, q, r
        0.0, 0.1, 0.0,    # phi, theta, psi
        44.94028*np.pi/180, -73.09747*np.pi/180, 3000*0.2808  # lon, lat, alt
    ])
    
    t = 0.0
    dt = 0.05  # 20 Hz update
    t_final = 60.0

    # Setup joystick
    read_inputs = get_joystick_input() if USE_JOYSTICK else None

    # Setup plotting
    if PLOT_STATES:
        plotter = LivePlotter(['u', 'v', 'w', 'phi', 'theta', 'psi'])

    # Storage for FG
    state_log = [x.copy()]
    time_log = [t]
    start_time = time.time()

    print("[INFO] Starting real-time simulation...")

    while t < t_final:
        # Real-time pacing
        # elapsed = time.time() - start_time
        # if elapsed < t:
        #     time.sleep(t - elapsed)
        loop_start = time.time()

        # Get joystick inputs
        if USE_JOYSTICK:
            inputs = read_inputs()
            u = np.array([
                inputs["elevator"],
                inputs["aileron"],
                inputs["rudder"],
                inputs["throttle"],
                inputs["throttle"]
            ])
            print(f"Joystick: δ_elevator={inputs['elevator']:.2f}, throttle={inputs['throttle']:.2f}")
        else:
            u = np.array([0.0, 0.1, 0.0, 0.5, 0.5])  # fallback fixed input

        # Integrate one step (RK4)
        
        # Wrap to make it compatible with rk4_step signature
        def wrapped_dynamics(t, x, u, params):
            return aircraft_dynamics(x, u, params)

        x = rk4_step(wrapped_dynamics, t, x, u, params, dt)

        # t += dt

        print("STATES:", x)

        # Real-time pacing
        loop_time = time.time() - loop_start
        sleep_time = max(0, dt - loop_time)
        time.sleep(sleep_time)

        
        print(f"[SIM] Updated FG state at t = {t:.2f}")
        # Stream to FlightGear
        if STREAM_TO_FLIGHTGEAR:
            # stream_simulation_to_fg(x.reshape(-1, 1), np.array([t]))
            fg.update_state(x)
            print("[FGStreamer] State updated.")
            

        # Plot selected states
        if PLOT_STATES:
            plotter.update(t, [x[0], x[1], x[2], x[6], x[7], x[8]])

        # Store for log
        state_log.append(x.copy())
        time_log.append(t)

        t = t+dt

    print("[INFO] Simulation complete.")
