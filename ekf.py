import numpy as np

from cstr import *
import sympy as sp
import copy


class EKF:
    """Extended Kalman Filter"""
    def __init__(self, model, R, Q, P):
        self.model = model
        self.A = self.get_matrices()[0]
        self.B = self.get_matrices()[1]
        self.Q_k = Q
        self.R_k = R
        self.P_k_1 = P

    def symbolic_diff(self):
        """Sympy differentiation of the ODEs of the CSTR"""
        Ca= sp.Symbol('Ca')
        T = sp.Symbol('T')
        Tc = sp.Symbol('Tc')
        dCadt = (self.model.q / self.model.V) * (self.model.Caf - Ca) - (self.model.k0*sp.E**(-self.model.Ea/(self.model.R * T))) * Ca
        dTdt = (self.model.q / self.model.V) * (self.model.Tf - T) + (-self.model.dHr / (self.model.rho * self.model.Cp)) * (self.model.k0*sp.E**(-self.model.Ea/(self.model.R * T))) * Ca \
                + (self.model.UA / (self.model.rho * self.model.Cp * self.model.V)) * (Tc - T)
        df1dCa = dCadt.diff(Ca)
        df1dT = dCadt.diff(T)
        df2dT = dTdt.diff(T)
        df2dCa = dTdt.diff(Ca)
        df2du = dTdt.diff(Tc)
        return [df1dCa, df1dT, df2dT, df2dCa, df2du]

    def get_matrices(self):
        """Get symbolic Jacobians of the dynamics of the CSTR"""
        df1dCa, df1dT, df2dT, df2dCa, df2du = self.symbolic_diff()
        A = np.array([[df1dCa, df1dT], [df2dCa, df2dT]])
        B = np.array([[0], [df2du]])
        return A, B

    def evaluate_matrices(self, CC, Tt, u):
        """Evaluate Jacobians at a given operating point"""
        A = copy.deepcopy(self.A)
        B = copy.deepcopy(self.B)
        Ca = sp.Symbol('Ca')
        T = sp.Symbol('T')
        Tc = sp.Symbol('Tc')
        A[0][0] = sp.lambdify([Ca, T], A[0][0], 'numpy')
        a00 = A[0][0](CC, Tt)
        A[0][1] = sp.lambdify([Ca, T], A[0][1], 'numpy')
        a01 = A[0][1](CC, Tt)
        A[1][0] = sp.lambdify([Ca, T, Tc],  A[1][0], 'numpy')
        a10 = A[1][0](CC, Tt, u)
        A[1][1] = sp.lambdify([Ca, T, Tc], A[1][1], 'numpy')
        a11 = A[1][1](CC, Tt, u)
        B[1][0] = sp.lambdify([Ca, T, Tc], B[1][0], 'numpy')
        b01 = B[1][0](CC, Tt, u)
        AA = np.array([[a00, a01], [a10, a11]])
        BB = np.array([[0], [b01]])
        return AA, BB

    def predict(self, x_k_1, u_k, dt):
        """Predict step of the EKF"""
        x_k = self.model.simple_step(x_k_1, u_k, dt)  # x_k|k-1: predicted state estimate
        F_k = self.evaluate_matrices(x_k[0], x_k[1], u_k)[0]  # Jacobian of the dynamics at the predicted state
        P_k = np.matmul(np.matmul(F_k, self.P_k_1), np.transpose(F_k)) + self.Q_k  # P_k|k-1: predicted covariance estimate
        return x_k, P_k

    def update(self, x_k, y_k, P_k, H_k):
        """Update step of the EKF"""
        y_k = np.array(y_k)
        x_k = np.array(x_k)
        z_k = y_k - x_k  # z_k: innovation
        S_k = self.R_k + np.matmul((np.matmul(H_k, P_k)), np.transpose(H_k))  # S_k: residual covariance
        K_k = np.matmul(np.matmul(P_k, np.transpose(H_k)), np.linalg.inv(S_k)) # K_k: near optimal Kalman gain
        x_kk = x_k + np.dot(K_k, z_k)  # x_k|k: updated state estimate
        P_kk = np.matmul((np.eye(2) - np.matmul(K_k, H_k)), P_k) # P_k|k: updated covariance estimate
        self.P_k_1 = copy.deepcopy(P_kk)
        return x_kk

    def observe(self, y, u, H_k, dt):
        """State Estimation by EKF given noisy observations of the system"""
        x_k_1 = copy.deepcopy(y)
        x_k, P_k = self.predict(x_k_1, u, dt)
        return self.update(x_k, y, P_k, H_k)
