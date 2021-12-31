'''!@example UseOnlineTrajectory.py
@brief Demo of using Rn space online trajectory.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from threading import Thread

import matplotlib.pyplot as plt
import numpy as np
from IPython import embed

from RVBUST.RPS import *
Logger.SetLevelForAll(1)

dof = 2
start = TrajectoryStateRn(dof)
start.position = Rx([0.05, 0.05])
target = TrajectoryStateRn(dof)
target.position = Rx([1.0, 0.5])

targets = [target.position.Coeffs()]

vel_limits = np.ones(dof)
acc_limits = np.ones(dof) * 3
jerk_limits = np.ones(dof) * 30

period = 0.008
speed_ratio = 1.0
online_traj = OnlineTrajectoryRn(
    start, target, vel_limits, acc_limits, jerk_limits, period, speed_ratio)
positions, velocities, accelerations, jerks, ts = [], [], [], [], []

t = 0.0
plt.ion()
ax = plt.plot([0], [0], "r-")[0]
plt.xlim(0, 1)
plt.ylim(0, 1)
plt.title("Trajectory")

while not online_traj.IsReached():
    state = online_traj.ComputeNextState()

    positions.append(state.position.Coeffs())
    velocities.append(state.velocity.Coeffs())
    accelerations.append(state.acceleration.Coeffs())
    jerks.append(state.jerk.Coeffs())
    ax.set_data(*np.asarray(positions).T)
    plt.pause(period)
    ts.append(t)
    if t > 0.3 and t < 0.3 + period:
        target.position = Rx([1., 0.5])
        online_traj.UpdateTargetState(target)
        online_traj.UpdateTrajectory()
        targets.append(target.position.Coeffs())
    if t > 0.7 and t < 0.7 + period:
        print("stop trajectory")
        online_traj.StopTrajectory()
    t += period

positions = np.array(positions)
plt.clf()
plt.close()
plt.ioff()
subplotshape = (2, 4)
plt.subplot2grid(subplotshape, (0, 0))
plt.title("position")
plt.plot(ts, positions)
plt.legend(list("123"))

plt.subplot2grid(subplotshape, (0, 1))
plt.title("velocity")
plt.plot(ts, velocities)
plt.legend(list("123"))

plt.subplot2grid(subplotshape, (1, 0))
plt.title("acceleration")
plt.plot(ts, accelerations)
plt.legend(list("123"))

plt.subplot2grid(subplotshape, (1, 1))
plt.title("jerk")
plt.plot(ts, jerks)
plt.legend(list("123"))

plt.subplot2grid(subplotshape, (0, 2), 2, 2)
plt.plot(positions[:, 0], positions[:, 1])
plt.plot(*np.asarray(targets).T, "g*")
plt.axis("scaled")

plt.suptitle("Online traj")
plt.show()
