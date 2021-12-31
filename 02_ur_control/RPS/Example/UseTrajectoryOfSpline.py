'''!@example UseTrajectoryOfSpline.py
@brief Example about how to use spline trajectories.  
@list
- CreateTrajectoryICSP
- CreateTrajectoryCubicSpline
- CreateTrajectoryBSpline4th
- CreateTrajectoryICSP
- CreateTrajectoryCubicSpline
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''
import numpy as np
from IPython import embed
from RVBUST.RPS import *

dof = 6
vel_limits = np.ones(dof)
acc_limits = np.ones(dof) * 4
jerk_limits = np.ones(dof) * 15
waypoints = np.array([
    [0., 0., 0., 0., 0., 0.],
    [1., 2., 0.5, -0.5, 0.3, 1.2],
    [0.3, 1.2, 0.9, 0.8, 0.7, 0.9],
    [0.6, 0.2, 0.9, 0.8, 0.7, 0.9],
])
path = CreatePath(waypoints, 0.1, PathType_Bezier2ndBlend)

traj = TrajectoryRnICSP(waypoints, vel_limits, acc_limits, jerk_limits)
PlotTraj(traj, title=f"ICSP Trajectory")

traj = TrajectoryRnICSP(path, vel_limits, acc_limits,
                        jerk_limits, sample_step=path.GetLength()/10)
PlotTraj(traj, title=f"ICSP Trajectory With Path Sampled Points")

traj = TrajectoryRnCubicSpline(
    waypoints, vel_limits, acc_limits, jerk_limits)
PlotTraj(traj, title=f"CubicSpline Trajectory")

traj = TrajectoryRnCubicSpline(
    path, vel_limits, acc_limits, jerk_limits, sample_step=path.GetLength()/10)
PlotTraj(traj, title=f"CubicSpline Trajectory With Path Sampled Points")

traj = TrajectoryRnBSpline4th(
    waypoints, vel_limits, acc_limits, jerk_limits)
PlotTraj(traj, title=f"BSpline4th Trajectory")
traj = TrajectoryRnBSpline4th(
    path, vel_limits, acc_limits, jerk_limits, sample_step=path.GetLength()/10)
PlotTraj(traj, title=f"BSpline4th Trajectory With Path Sampled Points")

waypoints = [
    Pose([0, 0, 0, 0, 0, 0, 1]),
    Pose([1, 0, 0, 0, 0, 1, 1]),
    Pose([1, 1, 0, 0, 1, 0, 1]),
    Pose([1, 1, 1, 1, 0, 0, 1]),
    Pose([0, 0, 0, 0, 0, 0, 1]),
]
path = CreatePath(waypoints, 0.1, PathType_Bezier2ndBlend)

traj = TrajectorySE3ICSP(waypoints, vel_limits, acc_limits, jerk_limits)
PlotTraj(traj, waypoints=waypoints,
         title=f"ICSP Spline Trajectory in Cartesian space")

traj = TrajectorySE3ICSP(
    path, vel_limits, acc_limits, jerk_limits, sample_step=path.GetLength()/10)
PlotTraj(traj, waypoints=waypoints,
         title=f"ICSP Spline Trajectory in Cartesian space with sampled points")

traj = TrajectorySE3CubicSpline(
    waypoints, vel_limits, acc_limits, jerk_limits)
PlotTraj(traj, waypoints=waypoints,
         title=f"Cubic Spline Trajectory in Cartesian space")

traj = TrajectorySE3CubicSpline(
    path, vel_limits, acc_limits, jerk_limits, sample_step=path.GetLength()/10)
PlotTraj(traj, waypoints=waypoints,
         title=f"Cubic Spline Trajectory in Cartesian space with sampled points")
