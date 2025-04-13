def rk4_step(dynamics_func, t, x, u, params, dt):
    """
    4th-order Runge-Kutta step for fixed time step integration.
    
    Args:
        dynamics_func: callable(t, x, u, params) → dx/dt
        t: current simulation time
        x: current state (numpy array)
        u: current control input (numpy array)
        params: aircraft parameters
        dt: time step

    Returns:
        Updated state vector after one integration step.
    """
    k1 = dynamics_func(t, x, u, params)
    k2 = dynamics_func(t + dt / 2, x + dt / 2 * k1, u, params)
    k3 = dynamics_func(t + dt / 2, x + dt / 2 * k2, u, params)
    k4 = dynamics_func(t + dt, x + dt * k3, u, params)

    return x + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)