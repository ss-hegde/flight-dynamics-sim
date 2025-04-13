# flight-dynamics-sim
Flight dynamics simulator

A modular and extensible flight dynamics simulator for fixed-wing aircraft, featuring a full nonlinear 6-DOF state-space model. This repository is part of an ongoing project aimed at modeling, controlling, and visualizing aircraft dynamics in real-time, with future extensions planned for AI-driven modeling and simulation to support faster, more adaptive, and data-informed flight dynamics analysis.

- Full 6-DOF nonlinear dynamics
- Trim and linearization routines (can be used if necessary)
- Autopilot design
- Simulation and visualization
- Real-time visualization

---

## To-Do

- [✅] Nonlinear aircraft model
- [✅] Trim and linearization
- [✅] Real-time visualization (FlightGear)
- [ ] Control and stability augmentation
- [ ] Closed-loop simulation
- [ ] Autopilot


---
## Repository Structure

flight-dynamics-sim/
├── `model/`                    # Nonlinear and linearized aircraft dynamics
├── `control/`                  # Controllers and stability augmentation logic
├── `trim_and_linearization/`   # Trim and linearization routines
├── `simulation/`               # Scripts for running open/closed-loop simulations
├── `flightgear_interface/`     # FlightGear streaming and setup
├── `governing_equations/`      # Integration schemes
└── README.md

---
### Joystick Control & Real-Time Visualization

This simulator supports real-time pilot input using a joystick or game controller. Aircraft states are streamed live to FlightGear, providing a fully interactive simulation experience.

To launch the simulation with visualization:
1. Ensure FlightGear is installed and configured correctly.
2. Run the provided batch file: `./runfg.bat`
3. Run the simulation: `/simulation/main.py`
4. Use your joystick to control the aircraft during the simulation.

## About the Author

Hi! I am Sharath Hegde, an aerospace engineer passionate about simulation, control systems, and flight dynamics.
This project is part of my ongoing work in developing and analyzing 6-DOF aircraft models for research in flight dynamics, control systems, and simulation environments.

Feel free to connect or drop feedback!



