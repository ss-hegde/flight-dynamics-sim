import numpy as np

class RollDamper:
    def __init__(self, K_p, aileron_limit=np.radians(25)):
        self.K_p = K_p
        self.aileron_limit = aileron_limit

    def compute(self, state):
        p = state[3]  # roll rate
        da = self.K_p * p
        da = np.clip(da, -self.aileron_limit, self.aileron_limit)
        return da
