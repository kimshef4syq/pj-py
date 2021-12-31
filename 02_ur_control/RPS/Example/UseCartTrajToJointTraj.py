'''!@example UseCartTrajToJointTraj.py
@brief This file constructs a Pose trajectory, convert it to CreateTrajectorySpline 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time

import matplotlib.pyplot as plt
import numpy as np
from IPython import embed
from RVBUST.RPS import *

Logger.SetLevelForAll(Logger.LoggerLevel_Error)
logger = RichLogger(__name__)

PI = np.pi
SPEED_RATION = 1
HOME_JOINTS = np.array([0, 0, 0, 0, -PI/2, 0])

rvis = RobotVis()
controller: SimController = SimController.Create("Motoman_GP12")
rvis.AddBody(controller)
kin_solver = controller.GetKinSolver()
controller.Connect()
controller.EnableRobot()
controller.SetSpeedRatio(SPEED_RATION)
controller.MoveJoints(HOME_JOINTS)
HOME_POSE = controller.GetPose()[1].Coeffs()

joint_limits = ConvertJointLimits2Arr(kin_solver.GetJointLimits())
poses = [
    Pose([0.3323,  0.7222,  0.3978,  0.842,  0.5395, -0.,  0.]),
    Pose([0.3323,  0.7222,  0.6978,  0.842,  0.5395, -0.,  0.]),
    Pose([0.4383,  0.0478,  0.6285,  0.9835,  0.18, -0.0186,  0.0034]),
    Pose([0.1104, -0.8495,  0.7941,  0.9835,  0.18, -0.0186,  0.0034]),
    Pose([0.1104, -0.8495,  0.5941,  0.9835,  0.18, -0.0186,  0.0034]),
]

# cartesian constant speed trajectory planned in Pose space
path = CreatePath(poses, blend_tolerance=0.1,
                  path_type=PathType_Bezier5thBlendCartesian)
traj = CreateTrajectory(path, 0.5)
hs = DrawTraj(traj, show_in_vis=True, view=rvis.GetView(),
              kin_solver=kin_solver, points_number=200, axis_length=0.01)
start_joint = Rx([1.13956, 0.370476, -0.830743, -
                  1.4846e-07, -0.369576, 0.00011391])
controller.MoveJoints(start_joint)
controller.ExecuteTrajectory(traj)
controller.MoveJoints(HOME_JOINTS)

# cartesian constant speed trajectory in joint space
traj2 = CartTrajToJointTraj(
    traj, kin_solver, joint_seed=start_joint, time_opt=False)
hs2 = DrawTraj(traj2, show_in_vis=True, view=rvis.GetView(),
               kin_solver=kin_solver, points_number=200, axis_length=0.02)
controller.MoveJoints(start_joint)
controller.ExecuteTrajectory(traj2)
controller.MoveJoints(HOME_JOINTS)
embed()
