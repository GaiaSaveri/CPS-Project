from cstr import *
from stl import load
from experiments import t, ref

from tabulate import tabulate
from texttable import Texttable
import latextable


cstr = CSTR()

''' Initial values table
rows = [['Quantity', 'Value', 'Unit'],
        ['Arrhenius pre-exponential', str(cstr.k0), '1/min'],
        ['Activation Energy', str(cstr.Ea), 'J/mol'],
        ['Gas constant', str(cstr.R), 'J/(mol*K)'],
        ['Reactor Volume', str(cstr.V), r'm\^{3}'],
        ['Density', str(cstr.rho), 'kg/m^3'],
        ['Heat capacity', str(cstr.Cp), 'J/(kg*K)'],
        ['Enthalpy of reaction', str(cstr.dHr), 'J/mol'],
        ['Heat transfer coefficient', str(cstr.UA), 'J/(min*K)'],
        ['Feed flowrate', str(cstr.q), '$m^3$/min'],
        ['Feed concentration', str(cstr.Caf), 'mol/m^3'],
        ['Feed temperature', str(cstr.Tf), 'K'],
        ['Initial concentration', str(cstr.Ca0), 'mol/m^3'],
        ['Initial temperature', str(cstr.T0), 'K']]

table = Texttable()
table.set_cols_align(["c"] * 3)
table.set_deco(Texttable.HEADER | Texttable.VLINES)
table.add_rows(rows)

print(tabulate(rows, headers='firstrow', tablefmt='latex'))
'''

# T = load(path="data/", controller="PID", ref=355)
# cstr.plot_simulation(T, t, set=355, plot_Ca=False)

# T = load(path="data/", controller="MPC", ref=375)
# cstr.plot_simulation(T[:len(t)], t, set=375, plot_Ca=False)

''' Robustness Table
rows = [['Reference', 'phi_1', 'phi_2', 'phi_3', 'Type']]

for i in range(len(ref)):
    dict_pid = load(path="data/", controller="PID_rob", ref=ref[i])
    dict_mpc = load(path="data/", controller="MPC_rob", ref=ref[i])
    row_pid = [str(ref[i]), str(dict_pid['oscillations']), str(dict_pid['stability']), str(dict_pid['steady']), 'PID']
    rows.append(row_pid)
    row_mpc = [str(ref[i]), str(dict_mpc['oscillations']), str(dict_mpc['stability']), str(dict_mpc['steady']), 'MPC']
    rows.append(row_mpc)

table = Texttable()
table.set_cols_align(["c"] * 5)
table.set_deco(Texttable.HEADER | Texttable.VLINES)
table.add_rows(rows)

print(tabulate(rows, headers='firstrow', tablefmt='latex'))
'''

''' Falsification Table
rows = [['Reference', 'Q', 'R', 'Robustness']]

for i in range(len(ref)):
    dict_mpc = load(path="data/", controller="MPC_fals", ref=ref[i])
    row_mpc = [str(ref[i]), str(dict_mpc['Q']), str(dict_mpc['R']), str(dict_mpc['rob'])]
    rows.append(row_mpc)

table = Texttable()
table.set_cols_align(["c"] * 4)
table.set_deco(Texttable.HEADER | Texttable.VLINES)
table.add_rows(rows)

print(tabulate(rows, headers='firstrow', tablefmt='latex'))
'''

''' Performance table
rows = [['Reference', 'Overshoot', 'Rise time', 'Steady state error', 'Settling time', 'Type']]

for i in range(len(ref)):
    dict_pid = load(path="data/", controller="PID_perf", ref=ref[i])
    dict_mpc = load(path="data/", controller="MPC_perf", ref=ref[i])
    row_pid = [str(ref[i]), str(dict_pid['overshoot']), str(dict_pid['rise_time']), str(dict_pid['steady_state_error']), str(dict_pid['settling_time']), 'PID']
    rows.append(row_pid)
    row_mpc = [str(ref[i]), str(dict_mpc['overshoot']), str(dict_mpc['rise_time']), str(dict_mpc['steady_state_error']), str(dict_mpc['settling_time']), 'MPC']
    rows.append(row_mpc)

table = Texttable()
table.set_cols_align(["c"] * 6)
table.set_deco(Texttable.HEADER | Texttable.VLINES)
table.add_rows(rows)

print(tabulate(rows, headers='firstrow', tablefmt='latex'))
'''




