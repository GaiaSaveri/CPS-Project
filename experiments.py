import pickle

from mpc import *
from pid import *
from ctrl_performance import *


def save(path, controller, ref, T):
    with open(path + controller + '_' + str(ref) + '.pickle', 'wb') as f:
        pickle.dump(T, f, protocol=pickle.HIGHEST_PROTOCOL)


def load(path, controller, ref):
    with open(path + controller + "_" + str(ref) + '.pickle', 'rb') as f:
        trace = pickle.load(f)
    return trace


# simulation time grid
t = np.linspace(0, 20, 251)

# model
cstr = CSTR()

# observer (Extended Kalman Filter)
Q = np.diag([1e-2, 1e-4])  # covariance matrix for process noise
R = np.diag([1e-4, 1e-2])  # covariance matrix for measurement noise
P = np.diag([0.1, 0.1])  # initial state covariance matrix
ekf = EKF(cstr, R, Q, P)

ekf.get_matrices()

# PID controller
pid_controller = PID(cstr, Kc=1.7, tau_i=0.8, tau_d=0.2, d=True, observer=ekf)

# MPC controller
mpc = MPC(cstr, h=10, Q=2.0, R=0.001, observer=None)

# constant signals (i.e. desired temperature to reach)
ref = [320, 325, 330, 335, 340, 345, 350, 355, 360, 365, 370, 375, 380, 385]

'''' Simulate and store trajectories
# PID CONTROL
for i in range(len(ref)):
    Ca, T, u1 = pid_controller.pid(ref[i], t)
    cstr.plot_simulation(Ca[:-1], T[:-1], t[:-1], ref[i], plot_Ca=True)
    save(path="data/", controller="PID", ref=ref[i], T=T)

# MPC control
for i in range(len(ref)):
    Ca, T, ustar = mpc.mpc(ref[i], t)
    cstr.plot_simulation(Ca[:len(t)], T[:len(t)], t[:len(t)], ref[i], plot_Ca=False)
    save(path="data/", controller="MPC", ref=ref[i], T=T)

# compute performance measures for each of the generated traces
for i in ["PID", "MPC"]:
    for j in range(len(ref)):
        dict = {}
        T = load(path="data/", controller=i, ref=ref[j])
        if i == "MPC":
            T = T[:len(t)]
        os = overshoot(T, ref[j])  # overshoot
        dict['overshoot'] = os
        rt = rise_time(T, t, ref[j])  # rise time
        dict['rise_time'] = rt
        sse = steady_state_error(T, ref[j])  # steady state error
        dict['steady_state_error'] = sse
        st = settling_time(T, t)  # settling time
        dict['settling_time'] = st
        save(path="data/", controller=i + "_perf", ref=ref[j], T=dict)
'''




