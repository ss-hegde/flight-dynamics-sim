import pygame
import numpy as np

def get_joystick_input():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick found!")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to joystick: {joystick.get_name()}")

    def read_inputs():
        pygame.event.pump()

        # Raw inputs: -1 to 1
        raw_roll = joystick.get_axis(0)    # aileron (left/right)
        raw_pitch = -joystick.get_axis(1)  # elevator (forward/back), flipped
        raw_yaw = joystick.get_axis(2)     # rudder (twist)
        raw_throttle = joystick.get_axis(3)  # throttle (usually 1 to -1)

        # Map to physical limits (in radians for control input)
        aileron = np.deg2rad(raw_roll * 25)                  # [-25°, 25°]
        elevator = np.deg2rad(raw_pitch * 25)                # [-25°, 25°]
        rudder = np.deg2rad(raw_yaw * 30)                    # [-30°, 30°]
        throttle = np.clip((1 - raw_throttle) / 2, 0, 1)     # [0, 1]

        return {
            "aileron": aileron,
            "elevator": elevator,
            "rudder": rudder,
            "throttle": throttle
        }

    return read_inputs