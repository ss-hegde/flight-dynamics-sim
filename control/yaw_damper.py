import numpy as np

class YawDamper:
    def __init__(self, K_r, rudder_limit=np.radians(25)):
        self.K_r = K_r
        self.rudder_limit = rudder_limit

    def compute(self, state):
        r = state[5]  # yaw rate
        dr = -self.K_r * r
        dr = np.clip(dr, -self.rudder_limit, self.rudder_limit)
        return dr
