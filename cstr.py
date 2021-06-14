import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import seaborn as sns

from pid import *
from ekf import *

sns.set_context('talk')
sns.set_theme()


class CSTR:
    """Class describing a Continuous Stirred Tank Reactor"""
    def __init__(self):
        # initializing constants
        self.Ea = 72750  # [J/mol] activation energy
        self.k0 = 7.2e10  # [min^{-1}] arrhenius k0
        self.R = 8.314  # [J/(mol*K)] gas constant
        self.V = 100.0  # [m^3] reactor volume
        self.rho = 1000.0  # [kg/m^3] density of AB mixture
        self.Cp = 0.239  # [J/(kg*K)] heat capacity
        self.dHr = -5.0e4  # [J/mol] enthalpy of the reaction
        self.UA = 5.0e4  # [J/(min*K)] heat transfer coefficient
        self.q = 100.0  # [m^3/min] feed flow-rate
        self.Caf = 1.0  # [mol/m^3] inlet feed concentration
        self.Tf = 350.0  # [K] feed temperature
        self.Ca0 = 0.87  # [mol/m^3] initial concentration
        self.T0 = 324.0  # [K] initial temperature

    def rate(self, T):
        """Arrhenius law for the reaction rate"""
        k = self.k0 * np.exp(-self.Ea / (self.R * T))
        return k

    def odes(self, x, t, u):
        """ODEs descirbing the dynamics of the system"""
        Ca, T = x  # state of the system
        Tc = u  # control input (coolant temperature)
        kT = self.rate(T)
        dCadt = (self.q / self.V) * (self.Caf - Ca) - kT * Ca
        dTdt = (self.q / self.V) * (self.Tf - T) + (-self.dHr / (self.rho * self.Cp)) * kT * Ca \
               + (self.UA / (self.rho * self.Cp * self.V)) * (Tc - T)
        return [dCadt, dTdt]

    def discrete_step(self, x,  dt, u, Ca, T, i):
        """Helper function to make a discrete step following the dynamics of the system"""
        dCadt, dTdt = self.odes(x, dt, u)
        Ca[i+1] = Ca[i] + dCadt*dt
        T[i+1] = T[i] + dTdt*dt
        x = Ca[i+1], T[i+1]
        return x

    def simple_step(self, x, u, dt):
        """Helper function to make a discrete step (without index dependency)"""
        dCadt, dTdt = self.odes(x, dt, u)
        Ca = x[0] + dCadt * dt
        T = x[1] + dTdt*dt
        return Ca, T

    def discrete_sim(self, u, t):
        """Simulate the discrete dynamics of the system"""
        x = [self.Ca0, self.T0]
        Ca = np.ones(len(t)) * x[0]
        T = np.ones(len(t)) * x[1]
        for i in range(len(t)-1):
            dt = t[i+1] - t[i]
            x = self.discrete_step(x, dt, u[i], Ca, T, i)
        return Ca, T

    def sim_step(self, x, u, ts, Ca, T, i):
        """Make a step following the continuous dynamics of the system"""
        y = odeint(self.odes, x, ts, args=(u,))
        Ca[i + 1], T[i + 1] = y[1]
        x = Ca[i + 1], T[i + 1]
        return x

    def simulate(self, u, t):
        """Simulate the continuous dynamics of the system"""
        x = [self.Ca0, self.T0]  # initial conditions
        # to store results
        Ca = np.ones(len(t)) * x[0]
        T = np.ones(len(t)) * x[1]
        for i in range(len(t) - 1):
            ts = [t[i], t[i + 1]]  # current time-step
            x = self.sim_step(x, u[i], ts, Ca, T, i)
        return Ca, T

    def plot_simulation(self, T, t, Ca=None, set=None, plot_Ca=True):
        """Utility function to plot results of simulation"""
        if plot_Ca:
            plt.figure(figsize=(16, 4))
            plt.subplot(1, 2, 1)
            plt.plot(t, Ca)
            plt.title('Concentration')
            plt.ylabel(r'mol/$m^3$')
            plt.xlabel('time [min]')
            plt.subplot(1, 2, 2)
        else:
            plt.figure(figsize=(8, 4))
        plt.plot(t, T, label='Measured')
        if set is not None:
            s = [set for i in range(len(t))]
            plt.plot(t, s, 'r:', label='Set point')
        plt.title('Rector temperature')
        plt.xlabel('time [min]')
        plt.ylabel('Kelvin')
        plt.legend()
        plt.tight_layout()
        # plt.savefig("figures/fig5.png", format='png')
        plt.show()
