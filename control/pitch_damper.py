import numpy as np

class PitchDamper:
    def __init__(self, K_q, elevator_limit=np.radians(30)):
        self.K_q = K_q
        self.elevator_limit = elevator_limit

    def compute(self, state):
        q = state[4]  # pitch rate
        de = -self.K_q * q
        de = np.clip(de, -self.elevator_limit, self.elevator_limit)
        return de
