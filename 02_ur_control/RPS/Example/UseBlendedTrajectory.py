'''!@example UseBlendedTrajectory.py
@brief Example about blending several trajectories as one. 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''
import numpy as np
from IPython import embed
from RVBUST.RPS import *

waypoints = np.array([
    Pose([0, 0., 0., 0., 0., 0., 1.0]),
    Pose([1, 3, 0., 0., 0., 0., 1.0]),
    Pose([3, 3, 0., 0., 0., 0., 1.0]),
    Pose([3, 0, 0., 0., 0., 0., 1.0]),
    Pose([0, 0, 0., 0., 0., 0., 1.0]),
])

speed1 = 0.4
speed2 = 0.5
speed3 = 0.4
amax = 1.0
jmax = 1.0

waypoints1, waypoints2, waypoints3 = waypoints[:2], waypoints[2:4], waypoints[3:5]

path = CreatePath(waypoints1, 0.5, PathType_Bezier5thBlend)
traj1 = CreateTrajectory(
    path, speed1, vel_init=0, vel_end=speed1, acc_max=amax, jerk_max=jmax)

path = CreatePath(waypoints2, 0.5, PathType_Bezier5thBlend)
traj2 = CreateTrajectory(
    path, speed2, vel_init=speed2, vel_end=0, acc_max=amax, jerk_max=jmax)

path = CreatePath(waypoints3, 0.5, PathType_Bezier5thBlend)
traj3 = CreateTrajectory(
    path, speed3, vel_init=speed3, vel_end=0, acc_max=amax, jerk_max=jmax)

# a blended trajectory will generated between each pair of neighbour trajectories
trajb = BlendedTrajectorySE3(
    [traj1, traj2, traj3], zone=0.5, speed_limit=1.0, acc_limit=1.0)
print(trajb.GetTrajs())
PlotTraj(trajb)
embed()
