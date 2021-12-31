'''!@example UseTrajWithVariantVelConstrain.py
@brief Test to construct a trajectory with different velocity at start stage, running stage and end stage. 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time

import matplotlib.pyplot as plt
import numpy as np
from IPython import embed
from RVBUST.RPS import *
from RVBUST.RPS.ControllerJoggerUI import ControllerJoggerUI

# from RVBUST.Motoman.PyMotoman import *
GP12RVDF = "/home/rvbust/Rvbust/Data/Multibody/RobotModels/Motoman/GP12/GP12.rvdf"
Logger.SetLevelForAll(Logger.LoggerLevel_Error)
PI = np.pi
SPEED_RATION = 1
HOME_JOINTS = np.array([0, 0, 0, 0, -PI/2, 0])

robot_vis = RobotVis()
robot = RobotModel()
robot.InitFromRVDF(GP12RVDF)
robot_vis.AddBody(robot)
robot_vis.GetView().Home()
rr: SimController = SimController.Create(robot.GetActiveManipulator())
# rr = GP12Controller.Create(robot.GetActiveManipulator())
kin_solver = rr.GetKinSolver()
rr.Connect()
rr.EnableRobot()
jogger = ControllerJoggerUI(rr, limits_scale=0.3)
rr.SetKinSolver(kin_solver)
rr.SetSpeedRatio(SPEED_RATION)
rr.MoveJoints(HOME_JOINTS)
robot_vis.GetView().Home()
HOME_POSE = rr.GetPose()[1].Coeffs()

joint_limits = ConvertJointLimits2Arr(kin_solver.GetJointLimits())
cart_limits = rr.GetCartesianLimits()
joints = np.array([HOME_JOINTS,
                   [0.4738, -0.2722, -0.2367,  0., -1.6063, -0.4738],
                   [0.4738, -0.2547, -0.9399,  0., -0.8856, -0.4738],
                   [0.,  0.0275, -0.6536,  0., -0.8889,  0.],
                   [-0.5162, -0.1522, -0.8223, -0.0005, -0.9,  0.5166],
                   [-0.5162, -0.208, -0.3294, -0.0004, -1.4487,  0.5163],
                   HOME_JOINTS])
poses = [
    Pose([0.3323,  0.7222,  0.3978,  0.842,  0.5395, -0.,  0.]),
    Pose([0.3323,  0.7222,  0.6978,  0.842,  0.5395, -0.,  0.]),
    Pose([0.1104, -0.8495,  0.7941,  0.9835,  0.18, -0.0186,  0.0034]),
    Pose([0.1104, -0.8495,  0.5941,  0.9835,  0.18, -0.0186,  0.0034]),
]

# Rn space, low-high-low speed phases
path = CreatePath(joints, blend_tolerance=0.1,
                  path_type=PathType_CircleBlend)
length = path.GetLength()
limits_profile = PiecewiseUniformScaleProfileConstraints(
    [0, length * 0.3, length*0.7, length], [0.1, 1., 0.2], *joint_limits)
# limits_profile = PiecewiseRampScaleProfileConstraints(
#     [0, length * 0.2, length*0.8, length], [0.1, 1., 1., 0.2], *joint_limits)
# traj = CreateTrajectory(path, limits_profile,
#                     traj_type=TrajType_Totp)
traj = CreateTrajectory(path, limits_profile,
                        traj_type=TrajType_Toppra)
hs = DrawTraj(traj, show_in_vis=True, view=robot_vis.GetView(),
              kin_solver=kin_solver)
rr.ExecuteTrajectory(traj)
robot_vis.GetView().Delete(hs)

# Pose space, low-high-low speed phases
path = CreatePath(poses, blend_tolerance=0.1,
                  path_type=PathType_Bezier5thBlend)
length = path.GetLength()
# limits_profile = PiecewiseUniformCartesianProfileConstraints(path,
#                                                       [0, length * 0.1, length*0.9, length], [0.1, 0.5, 0.1])
limits_profile = PiecewiseRampCatersianProfileConstraints(path,
                                                          # segments break points
                                                          [0, length * 0.1, length * 0.3,
                                                           length*0.7, length*0.9, length],
                                                          # segment TCP speed, ramp interpolation
                                                          [0.1, 0.1, 0.5, 0.5, 0.1, 0.1])
traj = CreateTrajectory(path, limits_profile,
                        traj_type=TrajType_Toppra)
hs = DrawTraj(traj, show_in_vis=True, view=robot_vis.GetView(),
              kin_solver=kin_solver, points_number=200)
traj_start_joint = Rx([1.13956, 0.370476, -0.830743, -
                       1.4846e-07, -0.369576, 0.00011391])
rr.MoveJoints(traj_start_joint)
rr.ExecuteTrajectory(traj)
rr.MoveJoints(HOME_JOINTS)
embed()
