import numpy as np

from cstr import *
from ekf import *


class MPC:
    """Non-linear Model Predictive Controller"""
    def __init__(self, model, h, Q, R, observer=None):
        self.model = model
        self.horizon = h
        self.Q = Q
        self.R = R
        self.observer = observer

    def cost(self, ustar, y, ref, dt):
        """Cost function over the receding horizon"""
        J = 0.0
        for i in range(self.horizon):
            # constraint to follow the dynamics of the system
            y[0, i+1], y[1, i+1] = self.model.discrete_step([y[0, i], y[1, i]], dt, ustar[i], y[0, :], y[1, :], i)
            # cost to be minimized
            J += ((y[1][i+1] - ref)**2) * self.Q
            J += (ustar[i]**2) * self.R
        return J

    def control(self, x0, ref, dt):
        """Minimize cost function over the current receiding horizon"""
        # x0: state at time t
        y = np.zeros([2, self.horizon+1])  # first row: concentration, second row: temperature
        y[:, 0] = x0
        u_init = np.ones(self.horizon + 1) * 297  # initial guess for control input
        sol = minimize(self.cost, u_init, args=(y, ref, dt), bounds=([(250, 350) for i in range(self.horizon+1)]))
        if not sol.success:
            print("Couldn't optimize")
        return sol.x[0]

    def mpc(self, ref, t):
        """Compute optimal input u"""
        x0 = [self.model.Ca0, self.model.T0]  # initial conditions
        ustar = np.ones(len(t) + self.horizon + 1)  # initializing control vector
        # reactor variables
        Ca = np.ones(len(t)+self.horizon+1) * self.model.Ca0
        T = np.ones(len(t)+self.horizon+1) * self.model.T0
        for i in range(len(t)-1):
            dt = t[i+1] - t[i]  # current time step
            ustar[i] = self.control(x0, ref, dt)  # minimize cost function
            x0 = self.model.discrete_step(x0, dt, ustar[i], Ca, T, i)  # make a discrete step
            x0 = np.array(x0)
            # adding noise
            x0[0] += np.random.normal(0, 0.1)
            x0[1] += np.random.normal(0, 0.1)
            if self.observer is not None:
                # estimate state
                x0 = self.observer.observe(x0, ustar[i], np.eye(2), dt)
        return Ca, T, ustar

