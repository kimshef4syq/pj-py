'''!@example UseControllerJoggerFollowTrajectoryCartesian.py
@brief An example about using controller jogger to follow given trajectory.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time
import numpy as np
from threading import Thread

from IPython import embed
from RVBUST.RPS import *
Logger.SetLevelForAll(4)

r: GenericRobotController = SimController.Create("Motoman_GP12")
rvis = RobotVis()
rvis.AddBody(r)

r.Connect("192.168.10.122")
r.EnableRobot()
kin_solver: GenericKinematics = r.GetKinSolver()
cart_limits = r.GetCartesianLimits()
_, pose = r.GetPose()

# 跟踪一个轨迹
points = [pose,
          pose.Translate(0, -0.5, 0),
          pose.Translate(0, -0.5, -0.5),
          pose.Translate(0, 0.5, -0.5),
          pose.Translate(0, 0.5, 0),
          pose
          ]
path = CreatePath(points, 0.1, PathType_Bezier5thBlend)
traj = CreateTrajectory(path, cart_limits*0.2, TrajType_DoubleS)
hs = DrawTraj(traj, kin_solver=kin_solver,
              view=rvis.GetView(), show_in_vis=True, show_in_plt=False)

jogger = ControllerJogger(r, 1.0)
jogger.UpdateSpeedRatio(0.5)
jogger.StartJog(use_change_direction=True)

state = TrajectoryStateSE3(6)
T = 0.1
t = 3*T
h = None
while True:
    state.position = traj.GetPosition(t)
    if h:
        rvis.GetView().Delete(h)
    jogger.UpdateTarget(state)
    h = rvis.PlotFrame(state.position)
    if jogger.IsReached():
        break
    time.sleep(T)
    t += T

jogger.StopJog()
