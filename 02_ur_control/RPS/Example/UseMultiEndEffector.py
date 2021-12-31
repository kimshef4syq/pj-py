'''!@example UseMultiEndEffector.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from IPython import embed
from RVBUST.RPS import *

# setup environment
rvis = RobotVis()

robot_model = RobotModel()
robot_model.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/Motoman/GP12/GP12.rvdf")

rvis.AddBody(robot_model)

manipulator = robot_model.GetActiveManipulator()
default_eef = manipulator.GetActiveEndEffector()

# rvis.AddBody(default_eef)

gripper = EndEffector()
gripper.InitFromRVDF(
    GetDataPath() + "Multibody/EndEffectors/Grippers/TwoFingerGripper/TwoFingerGripper.rvdf")

rvis.AddBody(gripper)

box = Multibody()
box.InitFromBox(.1, .1, .04)
box.SetBaseTransformation(Pose(1, 0, 0.4, 0, 0, 0, 1))
rvis.AddBody(box)

rvis.GetView().Home()

print("\nExit to print manipulator\n")
embed()
print(manipulator)


# Setup controller but without binding to manipulator
controller1 = SimController.Create(manipulator)
controller1.Connect()
controller1.EnableRobot()
manipulator.SetDoFPositions(
    Rx([-0.0235506, 0.360441, -0.350679, -0.0931698, -0.863494, 1.65505]))
controller2 = SimController.Create(manipulator)
controller2.Connect()
controller2.EnableRobot()

print("controller1 joint positions = {}".format(
    controller1.GetJointPosition()[1]))
print("controller2 joint positions = {}".format(
    controller2.GetJointPosition()[1]))

print("\nExit to Set and add another gripper\n")
embed()
manipulator.AddEndEffector(gripper)
print(manipulator)


print("\nExit to use controller\n")
embed()
manipulator.SetArmController(controller2)
bp2 = BasicMotionUtilities(controller2)
bp2.RotateBY(np.pi/3)
bp2.RotateBY(-np.pi/3)
manipulator.SetActiveEndEffector(gripper)
bp2.RotateBY(np.pi/3)
bp2.RotateBY(-np.pi/3)


print("\nExit to Grab box\n")
embed()
manipulator.SetActiveEndEffector(default_eef)
default_eef.Grab(box)
controller2.MoveJoints(Rx([1, 0.5, -0.5, 0, -0.5, 0]))
print(manipulator.GetEndEffectors())

print("\nUse another eef to grab the box, should be failed\n")
embed()
default_eef.Release(box)
manipulator.SetActiveEndEffector(gripper)
gripper.Grab(box)
controller2.MoveJoints(
    Rx([-0.0235506, 0.360441, -0.350679, -0.0931698, -0.863494, 1.65505]))

embed()

print("\nExit to change controller\n")
manipulator.SetArmController(controller1)
controller1.MoveJoints(Rx([1, 0.5, -0.5, 0, -0.5, 0]))
embed()
