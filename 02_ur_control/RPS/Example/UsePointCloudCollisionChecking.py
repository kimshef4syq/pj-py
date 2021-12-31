'''!@example UsePointCloudCollisionChecking.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import random
import time

from IPython import embed
from RVBUST.RPS import *

robot_vis = RobotVis()
vis = robot_vis.GetView()
file_name = GetDataPath() + "PointClouds/Objects/Bunny.txt"
env = Environment()
body1 = Multibody()
body1.InitFromPointCloudFile(file_name)
body2 = Multibody()
body2.InitFromMeshFile(
    GetDataPath() + "Multibody/3DModels/RVBUSTLogo.stl")
env.AddBody(body1, True)
env.AddBody(body2, True)
robot_vis.LoadEnvironment(env)

collision_matrix = env.GetCollisionMatrix()
checker = FCLCollisionChecker()
checker.InitFromEnv(env)
robot_vis.GetView().SetCameraPose([-0.20659139752388, -1.0438505411148071, 0.8726191520690918],
                                  [-0.04991886392235756, -0.2775440216064453,
                                   0.24953961372375488],
                                  [-0.006533326115459204, 0.6316606998443604, 0.7752174139022827])

while True:
    pose = Pose(R3([random.uniform(-0.3, 0.3), random.uniform(-0.3, 0.3),
                    random.uniform(-0.1, 0.1)]), Rotation().Random())
    body1.SetBaseTransformation(pose)
    print("-----------------------------------------------")
    print("Current pose: ", pose)
    hp = robot_vis.PlotFrame(pose, axis_len=0.08, axis_size=4)
    res, report = checker.CheckCollision(collision_matrix)
    print("The result is: \n")
    if res:
        print("There exists collision between \"{}\" and \"{}\".\n\n".format(
            body1.GetUniqueName(), body2.GetUniqueName()))
    else:
        print("No collision\n\n")
    time.sleep(2)
    robot_vis.GetView().Delete(hp)

embed()
