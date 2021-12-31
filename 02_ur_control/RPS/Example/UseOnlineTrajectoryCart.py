'''!@example UseOnlineCreateTrajectory.py
@brief Demo of using Pose space online trajectory.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from threading import Thread

import matplotlib.pyplot as plt
import numpy as np
from IPython import embed

from RVBUST.RPS import *

Logger.SetLevelForAll(Logger.LoggerLevel_Info)

dof = 6
start = TrajectoryStateSE3(dof)
start.position = Pose([0., 0., 0., 0, 0, 0, 1])
target = TrajectoryStateSE3(dof)
target.position = Pose([0.2, 1., 0., 0, 0, 0, 1])

targets = [start.position.Coeffs(), target.position.Coeffs()]
switches = [start.position.Coeffs()]

vel_limits = np.ones(dof) * 0.3
acc_limits = np.ones(dof) * 1
jerk_limits = np.ones(dof) * 2

period = 0.008
speed_ratio = 1.0
online_traj = OnlineTrajectorySE3(
    start, target, vel_limits, acc_limits, jerk_limits, period, speed_ratio, use_change_direction=True)
positions, velocities, accelerations, jerks, ts = [], [], [], [], []

t = 0.0
plt.ion()
plt.plot(0, 0, 'g^')
plt.plot(*target.position.Coeffs()[:2], 'g*')
ax = plt.plot([0], [0], "r-")[0]
plt.xlim(-0.1, 1.2)
plt.ylim(-0.1, 1.2)
plt.title("Trajectory")

while not online_traj.IsReached():
    state = online_traj.ComputeNextState()

    positions.append(state.position.Coeffs())
    velocities.append(state.velocity.Coeffs())
    accelerations.append(state.acceleration.Coeffs())
    jerks.append(state.jerk.Coeffs())
    ax.set_data(np.asarray(positions)[:, :2].T)
    ts.append(t)
    if 1.5 < t < 1.5 + period:
        switches.append(state.position.Coeffs())
        target.position = Pose([1., 1., 0., 0, 0, 0, 1])
        plt.plot(*target.position.Coeffs()[:2], 'g*')
        online_traj.UpdateTargetState(target)
        online_traj.UpdateTrajectory()
        targets.append(target.position.Coeffs())
        print(t)
    if 4.5 < t < 4.5 + period:
        switches.append(state.position.Coeffs())
        target.position = Pose([1., 0., 0., 0, 0, 0, 1])
        plt.plot(*target.position.Coeffs()[:2], 'g*')
        online_traj.UpdateTargetState(target)
        online_traj.UpdateTrajectory()
        targets.append(target.position.Coeffs())
        print(t)
    # if t > 3. and t < 3. + period:
    #     online_traj.StopTrajectory()

    plt.pause(period)
    t += period

plt.clf()
plt.close()
plt.ioff()

positions = np.array(positions)
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
plt.plot(*np.asarray(targets)[:, :2].T, "g*--")
plt.plot(*np.asarray(switches)[:, :2].T, "bo--")
plt.axis("scaled")

plt.suptitle("Online traj")
plt.show()
