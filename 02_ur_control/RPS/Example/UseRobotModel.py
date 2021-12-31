'''!@example UseRobotModel.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time

from IPython import embed
from RVBUST.RPS import *

robot_logger = Logger.GetConsoleLogger("RVS_CL")
robot_logger.SetLevel(0)
robot_vis = RobotVis()
env = Environment()
robot_vis.LoadEnvironment(env)

robot = RobotModel()
robot.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5.rvdf")

robot_copy = robot.Copy()
robot_copy.SetName("UR5_copy")
robot_copy.SetBaseTransformation(Pose([5, 0, 0, 0, 0, 0, 1]))
env.AddBody(robot)
env.AddBody(robot_copy)

robot_vis.ChooseShowMode(robot, 2)
robot_vis.ChooseShowMode(robot_copy, 2)
robot_vis.GetView().Home()
time.sleep(.5)

links = robot.GetLinks()
cylinder = links[1].GetBoundingCylinder()
cylinder_handle = robot_vis.PlotGeometry(cylinder, links[1].GetPose())
box = links[2].GetBoundingBox()
box_handle = robot_vis.PlotGeometry(box, links[2].GetPose())
spheres = links[3].GetDecomposedShperes()
sh = []
for sphere in spheres:
    sh.append(robot_vis.PlotGeometry(sphere, links[3].GetPose()))
embed()

robot_vis.GetView().SetCameraPose([-13.642976760864258, 3.8319621086120605, 20.48929214477539],
                                  [-13.065342903137207, 3.8720803260803223,
                                   19.673982620239258],
                                  [0.8156203627586365, 0.012262186966836452, 0.5784574747085571])
time.sleep(.5)

scara = RobotModel()
scara.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/Epson/LS6_502S/LS6_502S.rvdf")
scara.SetBaseTransformation(Pose(-2, 0, 0, 0, 0, 0, 1))
env.AddBody(scara)

body1 = Multibody()
body1.InitFromMeshFile(
    GetDataPath() + "Multibody/3DModels/Regr01.dae")
body1.SetBaseTransformation(Pose([0, 0, 1, 0, 0, 0, 1]))
body2 = Multibody()
body2.InitFromMeshFile(
    GetDataPath() + "Multibody/3DModels/Box.obj")
body2.SetBaseTransformation(Pose([2, 0, 0, 0, 0, 0, 1]))
body3 = Multibody()
body3.InitFromMeshFile(
    GetDataPath() + "Multibody/3DModels/Cube.ply")
body3.SetBaseTransformation(Pose([0, 2, 0, 0, 0, 0, 1]))
body4 = Multibody()
body4.InitFromMeshFile(
    GetDataPath() + "Multibody/3DModels/Nutbox.stl")
body4.SetBaseTransformation(Pose([0, 2, 2, 0, 0, 0, 1]))

body5 = Multibody()
body5.InitFromBox(0.1, 0.2, 0.3)
body5.SetBaseTransformation(Pose([0, 7, 0, 0, 0, 0, 1]))

body6 = Multibody()
body6.InitFromCylinder(0.5, 0.5)
body6.SetBaseTransformation(Pose([0, 9, 0, 0, 0, 0, 1]))

body7 = Multibody()
body7.InitFromSphere(0.1)
body7.SetBaseTransformation(Pose([0, 11, 0, 0, 0, 0, 1]))


env.AddBody(body1)
env.AddBody(body2)
env.AddBody(body3)
env.AddBody(body4)
env.AddBody(body5)
env.AddBody(body6)
env.AddBody(body7)
embed()
