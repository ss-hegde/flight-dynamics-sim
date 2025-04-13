import time
import numpy as np
from flightgear_python.fg_if import FDMConnection

# def stream_simulation_to_fg(states, times):
#     """
#     Streams aircraft states to FlightGear using the FGNetFDM interface.
#     Requires FlightGear to be running with:
#     --fdm=null --native-fdm=socket,out,30,localhost,5501,udp --native-fdm=socket,in,30,localhost,5502,udp --max-fps=30
#     """

#     sim_data = {
#         'states': states,
#         'times': times,
#         'index': 0,
#         'start_time': time.time(),
#         'sim_start': times[0]
#     }

#     def fdm_callback(fdm_data, _):
#         idx = sim_data['index']
#         if idx >= len(sim_data['times']):
#             return fdm_data  # Stop updating

#         state = sim_data['states'][:, idx]
#         u, v, w = state[0:3]
#         p, q, r = state[3:6]
#         phi, theta, psi = state[6:9]
#         lon, lat, alt_m = state[9:12]

#         # Update FDM fields
#         fdm_data.longitude_deg = np.rad2deg(lon)
#         fdm_data.latitude_deg = np.rad2deg(lat)
#         fdm_data.alt_m = alt_m
#         fdm_data.phi_rad = phi
#         fdm_data.theta_rad = theta
#         fdm_data.psi_rad = psi

#         # Optional: send velocities and rates
#         fdm_data.vcas_kts = u * 1.94384  # approx knots
#         fdm_data.urad_s = p
#         fdm_data.vrad_s = q
#         fdm_data.wrad_s = r

#         # Advance simulation index based on real time
#         elapsed = time.time() - sim_data['start_time']
#         sim_elapsed = sim_data['times'][idx] - sim_data['sim_start']
#         if elapsed >= sim_elapsed:
#             sim_data['index'] += 1

#         return fdm_data

#     # Start FlightGear connection
#     fdm_conn = FDMConnection()
#     fdm_conn.connect_rx('localhost', 5501, fdm_callback)
#     fdm_conn.connect_tx('localhost', 5502)
#     fdm_conn.start()

from flightgear_python.fg_if import FDMConnection
import numpy as np

def fdm_callback(fdm_data, event_pipe):
    if event_pipe.child_poll():
        x, = event_pipe.child_recv()
        u, v, w = x[0:3]
        p, q, r = x[3:6]
        phi, theta, psi = x[6:9]
        lon, lat, alt_m = x[9:12]

        print(f"[FGStreamer] Sending: lat={np.rad2deg(lat):.4f}, lon={np.rad2deg(lon):.4f}, alt={alt_m:.2f}")

        fdm_data.longitude_deg = np.rad2deg(lon)
        fdm_data.latitude_deg = np.rad2deg(lat)
        fdm_data.alt_m = alt_m
        fdm_data.phi_rad = phi
        fdm_data.theta_rad = theta
        fdm_data.psi_rad = psi

        fdm_data.vcas_kts = u * 1.94384
        fdm_data.urad_s = p
        fdm_data.vrad_s = q
        fdm_data.wrad_s = r

    return fdm_data

class FGStreamer:
    def __init__(self):
        self.fdm_conn = FDMConnection()
        self.pipe = self.fdm_conn.connect_rx('localhost', 5501, fdm_callback)
        self.fdm_conn.connect_tx('localhost', 5502)
        self.fdm_conn.start()

    def update_state(self, x):
        self.pipe.parent_send((x,))
