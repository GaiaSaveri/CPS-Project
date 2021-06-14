from experiments import *
import os
os.environ['JAVA_HOME'] = "/Library/Java/JavaVirtualMachines/adoptopenjdk-8.jdk/Contents/Home"
from moonlight import *


def oscillations(T):
    osc = [abs(T[i] - T[i-1]) for i in range(1, len(T))]
    return osc


def diff(T, ref):
    r = [ref for i in range(len(T))]
    d = abs(T - r)
    return d


# requirements regarding oscillations
script = """
signal {real o;}
domain minmax;
formula oscillations = {eventually {globally [0.0, 10.0] (o < 5.0)}}; 
formula stability = globally [10.0, 20.0] (o < 3.0);
"""

moonlightScript = ScriptLoader.loadFromText(script)
monitor_oscillations = moonlightScript.getMonitor("oscillations")
monitor_stability = moonlightScript.getMonitor("stability")


# requirement regarding steady state
script1 = """
signal {real diff;}
domain minmax;
formula steady = globally [15.0, 20.0] (diff < 3.0);
"""

moonlightScript1 = ScriptLoader.loadFromText(script1)
monitor_steady = moonlightScript1.getMonitor("steady")


# verification of requirements
for i in ["PID", "MPC"]:
    for j in range(len(ref)):
        dict = {}
        T = load(path="data/", controller=i,  ref=ref[j])
        if i == "MPC":
            T = T[:len(t)]
        d = diff(T, ref[j])
        osc = oscillations(T)
        o_signal = [[oo] for oo in osc]
        result_oscillations = monitor_oscillations.monitor(list(t[:-1]), o_signal)
        dict["oscillations"] = result_oscillations[0][1]
        result_stability = monitor_stability.monitor(list(t[:-1]), o_signal)
        dict["stability"] = result_stability[0][1]
        diff_signal = [[dd] for dd in d]
        result_steady = monitor_steady.monitor(list(t), diff_signal)
        dict["steady"] = result_steady[0][1]
        save(path="data/", controller=i+"_rob", ref=ref[j], T=dict)


