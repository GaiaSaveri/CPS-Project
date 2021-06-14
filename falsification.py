import os
os.environ['JAVA_HOME'] = "/Library/Java/JavaVirtualMachines/adoptopenjdk-8.jdk/Contents/Home"
from moonlight import *

from stl import monitor_steady, diff
from experiments import ekf, t, save, load
from mpc import *


def falsification(N, ref):
    cstr = CSTR()
    minSTL = float('Inf')
    dict = {}
    for i in range(N):
        Q = abs(np.random.normal(2.0, 1))
        R = abs(np.random.normal(0.001, 0.01))
        mpc = MPC(cstr, h=10, Q=Q, R=R, observer=ekf)
        dict['Q'] = Q
        dict['R'] = R
        _, T, _ = mpc.mpc(ref, t)
        dict['T'] = T[:len(t)]
        d = diff(T[:len(T)], ref)
        d_signal = [[dd] for dd in d]
        result = monitor_steady.monitor(list(t), d_signal)
        stl = result[0][1]
        dict['rob'] = stl
        if stl < minSTL:
            minSTL = stl
            save(path="data/", controller="MPC_fals", ref=ref, T=dict)
        if minSTL < 0:
            break


N = 100
ref = [320, 325, 330, 335, 340, 345, 350, 355, 360, 365, 370, 375, 380, 385]
for i in range(len(ref)):
    falsification(N, ref[i])

