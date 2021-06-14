**CPS Course Project: Temperature Control of a CSTR**

**Author**: Gaia Saveri

The repository is structured as follows:

* `cstr.py` contains the implementation of the plant of a Continuous Stirred Tank Reactor (CSRT) and code for its simulation.

* `ctrl_performance.py` contains functions to measure the performance of the controller, most of the code is taken from the first control lab lecture of the course.

* `ekf.py` contains the implementation of the Extended Kalman Filter, as well as code for linearizing the dynamics of the CSTR.

* `experiments.py` contains code to simulate trajectories and measure the performance of both controllers against difference reference inputs.

* `falsification.py` contains code to perform falsification using Moonlight.

* `mpc.py` contains the implementation of the MPC controller.

* `pid.py` contains the implementation of the PID controller, as well as code to tune it with IMC technique.

* `stl.py` contains code to verify the requirements agains different references using Moonlight.

* `tables.py` utility file to produce tables used in the report.

* `data/` folder containing collected data (simulated trajectories, performance measures, verification and falsification robustness).

* `figures/` folder containing images and tables used in the report and in the presentation of the project.
