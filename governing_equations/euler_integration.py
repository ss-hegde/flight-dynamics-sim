import numpy as np

def euler_step(dynamics_func, t, x, u, params, dt):
    dx = dynamics_func(t, x, u, params)
    return x + dx * dt