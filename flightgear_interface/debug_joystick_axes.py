# tools/debug_joystick_axes.py

# tools/debug_joystick_axes.py

import pygame
import time
import numpy as np

def debug_joystick_axes():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("❌ No joystick found.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"✅ Connected to joystick: {joystick.get_name()}")
    print("🕹️ Move the controls to see live raw and scaled values.")
    print("-" * 90)

    try:
        while True:
            pygame.event.pump()

            # Raw inputs
            raw_roll = joystick.get_axis(0)       # aileron
            raw_pitch = -joystick.get_axis(1)     # elevator (flipped for natural mapping)
            raw_yaw = joystick.get_axis(2)        # rudder
            raw_throttle = joystick.get_axis(3)   # throttle (usually +1 = min, -1 = max)

            # Scaled inputs (in degrees or 0–1)
            aileron_deg = raw_roll * 25
            elevator_deg = raw_pitch * 25
            rudder_deg = raw_yaw * 30
            throttle = (1 - raw_throttle) / 2

            # Print raw and scaled values
            print(
                f"Aileron   Axis 0: raw={raw_roll:+.2f} → {aileron_deg:+5.1f}°   | "
                f"Elevator Axis 1: raw={-raw_pitch:+.2f} → {elevator_deg:+5.1f}°   | "
                f"Rudder    Axis 2: raw={raw_yaw:+.2f} → {rudder_deg:+5.1f}°   | "
                f"Throttle  Axis 3: raw={raw_throttle:+.2f} → {throttle:.2f}     ",
                end="\r"
            )

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        pygame.quit()

if __name__ == "__main__":
    debug_joystick_axes()
