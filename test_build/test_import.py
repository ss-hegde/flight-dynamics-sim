# test_imports.py

from model import nonlinear_6DOF_aircraft_model
from control import pitch_damper
from flightgear_interface import stream_to_flightgear
from governing_equations import euler_integration
from simulation import main
from trim_and_linearization import trim_analysis

print("All modules imported successfully.")