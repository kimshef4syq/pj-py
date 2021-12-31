'''!@example UseControllerJoggerFollowTrajectory.py
@brief An example about using controller jogger to follow given trajectory.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time
import numpy as np
from threading import Thread

from IPython import embed
from RVBUST.RPS import *
Logger.SetLevelForAll(2)

r: GenericRobotController = SimController.Create("Motoman_GP12")
rvis = RobotVis()
rvis.AddBody(r)

r.Connect("192.168.10.122")
r.EnableRobot()
kin_solver: GenericKinematics = r.GetKinSolver()
jls = kin_solver.GetJointLimits()

# 跟踪一个轨迹
points = [[0.,  0.,  0.,  0.,  0.,  0.],
          [-0.8228, -0.119, -0.1119,  0., -0.0071,  0.8228],
          [-0.8228,  0.1519, -0.6007,  0.,  0.7525,  0.8228],
          [0.6764,  0.5512, -0.0274, -0.,  0.5786, -0.6764],
          [0.8264,  0.1422,  0.0863, -0.,  0.0559, -0.8264],
          [0.,  0.,  0.,  0.,  0.,  0.]
          ]
path = CreatePath(points, 0.1, PathType_Bezier5thBlend)
traj = CreateTrajectory(path, jls, TrajType_DoubleS)
hs = DrawTraj(traj, kin_solver=kin_solver,
              view=rvis.GetView(), show_in_vis=True, show_in_plt=False)

jogger = ControllerJogger(r, 1.0)
jogger.UpdateSpeedRatio(0.3)
jogger.StartJog()

state = TrajectoryStateRn(6)
T = 0.1
t = 2*T
h = None
while True:
    state.position = traj.GetPosition(t)
    pose = kin_solver.GetPositionFK(state.position)[1]
    if h:
        rvis.GetView().Delete(h)
    jogger.UpdateTarget(state)
    h = rvis.PlotFrame(pose)
    if jogger.IsReached():
        break
    time.sleep(T)
    t += T

jogger.StopJog()
