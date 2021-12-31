'''!@example UseTrajectoryWithPath.py
@brief Demostration about trajectory usage.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import numpy as np
from RVBUST.RPS import *
from IPython import embed
Logger.SetLevelForAll(3)

dof = 6
waypoints = np.array([
    [0., 0., 0., 0., 0., 0.],
    [1., 2., 0.5, -0.5, 0.3, 1.2],
    [0.3, 1.2, 0.9, 0.8, 0.7, 0.9],
    [0.6, 0.2, 0.9, 0.8, 0.7, 0.9],
])
vel_limits = np.ones(dof)
acc_limits = np.ones(dof)
jerk_limits = np.ones(dof)
traj_types = (TrajType_DoubleS, TrajType_Trapezoidal,
              TrajType_Toppra, TrajType_Totp)
path_types = (PathType_Bezier5thBlend, PathType_Bezier2ndBlend,
              PathType_Bezier2ndBlend, PathType_CircleBlend)
for path_type, traj_type in zip(path_types, traj_types):
    path = CreatePath(waypoints, 1.0, path_type)
    traj = CreateTrajectory(path, vel_limits, acc_limits,
                            jerk_limits, traj_type)
    DrawTraj(traj, title=f"{path_type}-{traj_type}-R{dof} Trajectory")


waypoints = [
    Pose(),
    Pose(x=0.1),
    Pose(x=0.1, y=0.3),
    Pose(x=0.2, y=0.3, z=-0.2, roll=np.pi/2),
    Pose(x=0.2, y=-0.3, z=-0.2, pitch=np.pi/2),
    Pose(x=0., y=0.1, z=-0.2, yaw=np.pi/2),
]
for path_type, traj_type in zip(path_types, traj_types):
    path = CreatePath(waypoints, 1.0, path_type)
    traj = CreateTrajectory(path, vel_limits, acc_limits,
                            jerk_limits, traj_type)
    DrawTraj(traj, title=f"{path_type}-{traj_type}-Cartesian Trajectory")
embed()
