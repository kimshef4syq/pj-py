'''!@example UseConstraintTransportTrajectory.py
@brief A demo about transporting a glass of water with robot arm.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time
from IPython import embed
from RVBUST.RPS import *

robot_model = RobotModel()
robot_model.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/ABB/IRB1200_5_90/IRB1200_5_90.rvdf")
robot_model.SetActiveManipulator("Arm")
manip = robot_model.GetActiveManipulator()

robot_vis = RobotVis()
env = Environment()
robot_vis.LoadEnvironment(env)

v1 = [2.6586620807647705, 1.8155426979064941, 2.139530658721924]
v2 = [1.918289303779602, 1.2871607542037964, 1.7240062952041626]
v3 = [-0.33165839314460754, -0.2505359649658203, 0.9095242619514465]
robot_vis.GetView().SetCameraPose(v1, v2, v3)
env.AddBody(robot_model)
robot_vis.ChooseShowMode(robot_model, 5)

controller = SimController.Create(manip)
controller.Connect()
controller.EnableRobot()
bp = BasicMotionUtilities(controller)
bp.RotateBY(np.pi/2)
bp.MoveBZ(-0.1)

max_vels = 3 * np.array([1, 1, 1])
max_accs = 5 * np.array([1, 1, 1])
max_jerks = 15 * np.array([1, 1, 1])
blend_tolerance = 0.1

controller.SetTCP(Pose(-0.1, 0, 0, 0, 1, 0, 0))
res, T1 = controller.GetPose()
T2 = Pose(0.5, 0.5, 0.3, 0, 0, 0, 1)

traj = ConstraintTransportTrajectory()
embed()
while True:
    controller.SetSpeedRatio(1)
    res, T1 = controller.GetPose()

    position = np.array([0.3, -0.7, -0.05]) + \
        np.array([0.5, 1.4, 0.8]) * np.random.random(3)

    T2 = Pose(R3(position), Rotation())
    traj.Init([T1, T2], max_vels, max_accs, max_jerks, blend_tolerance)
    controller.ExecuteTrajectory(traj)
    time.sleep(1)

embed()
