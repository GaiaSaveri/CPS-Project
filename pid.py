from scipy.optimize import minimize
from scipy.interpolate import interp1d
import copy
import matplotlib.pyplot as plt
import numpy as np
from cstr import *


class PID:
    """Adaptive Proportional-Integral-Derivative Controller tuning"""
    def __init__(self, model, Kc=None, tau_i=None, tau_d=None, d=False, observer=None):
        self.model = model
        # eventually computed via identification
        self.id = None  # identification parameters
        # eventually computed via tuning
        self.Kc = Kc
        self.tau_i = tau_i
        self.d = d  # if True PID control, else PI control
        if self.d:
            self.tau_d = tau_d
        # whether to use EKF
        self.observer = observer

    def fopdt(self, y, t, x, u, u_fn, yy):
        """First-order plus dead time (FOPDT) approximation"""
        Kp, tau, theta = x
        try:
            if (t - theta) <= 0:
                um = u_fn(0.0)
            else:
                um = u_fn(t - theta)
        except:
            um = u[0]
        dydt = (-(y - yy[0]) + Kp * (um - u[0])) / tau
        return dydt

    def sim_fopdt(self, x, u, u_fn, t, y):
        """Simulate FOPDT model"""
        # Kp, tau, theta = x
        dt = t[1] - t[0]  # assuming all time-steps are equal
        steps = len(t)
        yy = np.zeros(steps)  # for storing results of simulation
        yy[0] = y[0]  # initial condition
        for i in range(steps - 1):
            ts = [dt * i, dt * (i + 1)]
            s = odeint(self.fopdt, yy[i], ts, args=(x, u, u_fn, y))
            yy[i + 1] = s[-1]
        return yy

    def objective(self, x, u, u_fn, t, y):
        """Cost function: sum of squares error between FOPDT and simulation data"""
        # simulate the model
        yy = self.sim_fopdt(x, u, u_fn, t, y)
        # calculate objective
        obj = 0.0
        for i in range(len(yy)):
            # sum of squares error
            obj = obj + (yy[i] - y[i]) ** 2
        return obj

    def identification(self, x0, bounds, u, t, y):
        """Minimize cost function"""
        u_fn = interp1d(t, u)
        solution = minimize(self.objective, x0, args=(u, u_fn, t, y), method="SLSQP", bounds=bounds)
        self.id = solution.x  # [K, tau, theta]

    def plot_fopdt_proxy(self, t, u, y):
        assert (self.id is not None)
        u_fn = interp1d(t, u)
        sim = self.sim_fopdt(self.id, u, u_fn, t)
        plt.plot(t, sim, 'k--', label="Approximation")
        plt.plot(t, y, 'b--', label="Output")
        plt.legend()
        plt.show()

    def imc(self):
        """Internal Model Control (IMC) tuning"""
        assert (self.id is not None)
        tauc = max(0.1*self.id[1], 0.8*self.id[2])
        self.Kc = (1/self.id[0]) * ((self.id[1] + 0.5 * self.id[2])/(tauc + 0.5 * self.id[2]))
        self.tau_i = self.id[1] + 0.5 * self.id[2]
        if self.d:
            self.tau_d = ((self.id[1]*self.id[2])/(2*self.id[1] + self.id[2]))

    def tune_controller(self, u, t):
        """Find PID parameters given input u"""
        x0 = np.array([3.0, 3.0, 0])  # for identification
        bounds = ((-1.0e10, 1.0e10), (0.01, 1.0e10), (0.0, 5.0))
        _, y = self.model.simulate(u, t)
        self.identification(x0, bounds, u, t, y)
        self.imc()

    def pid(self, ref, t):
        """Discrete PID control"""
        # ref: reference temperature to track (set point)
        assert (self.Kc is not None)
        assert (self.tau_i is not None)
        if self.d:
            assert(self.tau_d is not None)
        out = np.zeros(len(t))  # controller output
        proc = np.zeros(len(t))  # controlled variable
        e = np.zeros(len(t))  # error
        ig = np.zeros(len(t))  # integral of the error
        if self.d:
            d = np.zeros(len(t))  # derivative of the error
            D = np.zeros(len(t))
        P = np.zeros(len(t))
        I = np.zeros(len(t))
        # reactor variables
        Ca = np.ones(len(t)) * self.model.Ca0
        T = np.ones(len(t)) * self.model.T0
        # PID loop
        u = np.ones(len(t))
        x0 = [self.model.Ca0, self.model.T0]  # initial conditions of the cstr
        for i in range(len(t)-1):
            dt = t[i+1] - t[i]  # current time-step
            e[i] = ref - proc[i]  # current step error
            if i > 0:
                ig[i] = ig[i-1] + e[i] * dt  # discrete integral
                if self.d:
                    d[i] = (e[i] - e[i-1])/dt  # discrete derivative
            P[i] = self.Kc * e[i]  # proportional term
            I[i] = self.Kc/self.tau_i * ig[i]  # integral term
            if self.d:
                D[i] = self.tau_d * d[i]
            out[i] = P[i] + I[i] if not self.d else P[i] + I[i] + D[i]
            # bounds for the controlled variable
            if out[i] > 350:
                out[i] = 350
            if out[i] < 250:
                out[i] = 250
            ts = [t[i], t[i+1]]  # current time step
            u[i] = out[i]
            x0 = self.model.sim_step(x0, u[i], ts, Ca, T, i)
            # adding noise to the output
            Ca[i + 1] += np.random.normal(0, 0.1)
            T[i + 1] += np.random.normal(0, 0.1)
            # observer
            if self.observer is not None:
                Ca[i+1], T[i+1] = self.observer.observe([Ca[i+1], T[i+1]], u[i], np.eye(2), dt)
            proc[i+1] = T[i+1]
        return Ca, T, u
