'''!@example UseContinuousCheckCollision.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import math
import time

from IPython import embed
from RVBUST.RPS import *

robot_logger = Logger.GetConsoleLogger("RVS_CL")
robot_logger.SetLevel(2)

robot_vis = RobotVis()

robot_vis.GetView().SetCameraPose([-4.42, 2.24, 6.08],
                                  [-3.85, 2.00, 5.30],
                                  [0.77, -0.19, 0.61])
env = Environment()
robot_vis.LoadEnvironment(env)

pose1_beg = Pose([1, 0, 0, 0, 0, 0, 1])
# pose1_end = math.pi/2*Pose([0, 1, 0, 0, 0, 0, 1]).Adj() * \
#     SE3Tangent(0, 0, 0, 0, 0, 1)+pose1_beg
pose1_end = math.pi * SE3Tangent(1, 0, 0, 0, 0, 1) + pose1_beg
body2_pose = pose1_beg + 0.5*(pose1_end-pose1_beg)

body1 = Multibody()
body1.InitFromBox(0.1, 0.2, 0.3)
body1.SetName("box1")
body1.SetBaseTransformation(pose1_beg)

body2 = Multibody()
body2.InitFromBox(0.1, 0.2, 0.3)
body2.SetName("box2")
body2.SetBaseTransformation(body2_pose)

env.AddBody(body1, True)
env.AddBody(body2, True)
time.sleep(1)
robot_vis.GetView().SetColor(robot_vis.GetLinkData(
    body1.GetLinks()[0])[1].m_collision_geometry_handle, [1, 0, 0, 0.7])
robot_vis.GetView().SetColor(robot_vis.GetLinkData(
    body2.GetLinks()[0])[1].m_collision_geometry_handle, [0, 1, 0, 0.7])
robot_vis.ChooseShowMode(mode=2)
collision_checker = FCLCollisionChecker()
collision_checker.InitFromEnv(env)


path1 = CreatePath([Pose(pose1_beg.Coeffs()), Pose(
    pose1_end.Coeffs())], 0.01, PathType_NoBlend)
seg = 100
embed()
for i in range(seg):
    t1 = i/seg*path1.GetLength()
    pose1 = Pose(path1.GetConfig(t1))
    body1.SetBaseTransformation(pose1)
    b = robot_vis.GetView().Box([0, 0, 0], [0.1, 0.2, 0.3], [
        0.9254902, 0.9254902, 0.9058824, 1])
    robot_vis.GetView().SetTransparency(b, 0.9)
    robot_vis.GetView().SetTransform(
        b, pose1.GetR3().Coeffs(), pose1.GetSO3().Coeffs(),)
    time.sleep(0.02)
t_s = time.time()
[res, reporter] = collision_checker.ContinuousCheckCollision(
    body1.GetLinks()[0], pose1_beg, pose1_end, body2.GetLinks()[0], body2_pose, body2_pose)
t_e = time.time()
print("ContinuousCheckCollision result {}".format(res))
print("ContinuousCheckCollision compuate time: {} s".format(t_e-t_s))
if res:
    body1.SetBaseTransformation(reporter.m_contact_pose0)
    body2.SetBaseTransformation(reporter.m_contact_pose1)

embed()
collision_matrix = env.GetCollisionMatrix()
t_s = time.time()
res, report = collision_checker.CheckCollision(collision_matrix)
t_e = time.time()
if res is True:
    print("There exists collision between two box.")
else:
    print("No collision")
print("CheckCollision compuate time: {} s".format(t_e-t_s))
embed()
