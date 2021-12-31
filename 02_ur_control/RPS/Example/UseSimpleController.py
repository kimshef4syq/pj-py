'''!@example UseSimpleController.py
@brief Demonstration about how to use simple controller.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import numpy as np
from IPython import embed
from RVBUST.RPS import *

robot = LoadRobotModelFromFile(GetRobotFile("Motoman", "GP12"))
mani = robot.GetActiveManipulator()

rvis = RobotVis()
rvis.AddBody(robot)
rvis.GetView().Home()
rvis.GetView().SetCameraPose([2.6494, -3.0163,  1.4683],
                             [2.0473, -2.2404,  1.2801],
                             [-0.1095,  0.1532,  0.9821])

simu_controller = SimController.Create(mani)
simu_controller.SetSpeedRatio(0.2)
simu_controller.Connect()
jls = simu_controller.GetKinSolver().GetJointLimits()

# Simulation controller server
controller_server = ControllerServer(simu_controller)
controller_server.StartServer("0.0.0.0", 9999)

# Logger.SetLevelForAll(Logger.LoggerLevel_Debug)
robot2 = LoadRobotModelFromFile(GetRobotFile("Motoman", "GP12"))
mani2 = robot2.GetActiveManipulator()
simp_controller = SimpleController.Create(mani2, False)
simp_controller.Connect("127.0.0.1", 9999)
simp_controller.EnableRobot()

# simple controller server
controller_server2 = ControllerServer(simp_controller)
controller_server2.StartServer("0.0.0.0", 8888)
robot3 = LoadRobotModelFromFile(GetRobotFile("Motoman", "GP12"))
mani3 = robot2.GetActiveManipulator()
simp_controller = SimpleController.Create(mani3, False)
simp_controller.Connect("127.0.0.1", 8888)
simp_controller.EnableRobot()

# simple controller client
_, pose = simp_controller.GetPose()
_, q = simp_controller.GetJointPosition()
pose1 = SE3Tangent(0, 0, -0.3, 0, 0, 0) + pose
q1 = RxTangent([0, 0, -1, 0.4, 0, 0]) + q
q2 = RxTangent([1, 0, 1, 0.4, 0, 0]) + q
path = CreatePath([q, q1, q2], 0.1, PathType_Bezier5thBlend)
traj = CreateTrajectory(path, jls, TrajType_DoubleS)
hs = DrawTraj(traj, kin_solver=simu_controller.GetKinSolver(),
              show_in_vis=True, show_in_plt=False, view=rvis.GetView())
simp_controller.ExecuteTrajectory(traj)
simp_controller.MoveLinear(pose1)
simp_controller.MoveJoints(q1)
simp_controller.MoveJoints(q)
simp_controller.SetDigitalOutput(2, True)
_, v = simp_controller.GetDigitalOutput(2)
embed()
simp_controller.Disconnect()
