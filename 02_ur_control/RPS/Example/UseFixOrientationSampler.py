'''!@example UseFixOrientationSampler.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''
import time
from RVBUST.RPS import *

robot_model = RobotModel()
robot_model.InitFromRVDF(
    "/home/rvbust/Rvbust/Data/Multibody/RobotModels/ABB/IRB1200_7_70/IRB1200_7_70.rvdf")
manipulator = robot_model.GetActiveManipulator()
rvis = RobotVis()
rvis.AddBody(robot_model)
rvis.GetView().Home()

controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()


sampler = FixOrientationSampler(manipulator)

print("\n\nSampler Uniform in region\n\n")
embed()
for i in range(100):
    res, joint = sampler.SampleUniform()
    if res:
        manipulator.SetDoFPositions(joint)
    time.sleep(0.05)


print("\n\nSampler Uniform near a seed\n\n")
embed()

seed_joint = Rx([0, 0.839552, 0.591505, 0, 0.139743, 0])

for i in range(100):
    res, joint = sampler.SampleUniformNear(seed_joint, 0.04)
    if res:
        manipulator.SetDoFPositions(joint)
    time.sleep(0.05)
embed()

print("\n\nSampler Gaussian\n\n")

for i in range(100):
    res, joint = sampler.SampleGaussian(seed_joint, 0.04)
    if res:
        manipulator.SetDoFPositions(joint)
    time.sleep(0.05)

embed()
