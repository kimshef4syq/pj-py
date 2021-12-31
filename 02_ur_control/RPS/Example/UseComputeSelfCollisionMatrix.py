'''!@example UseComputeSelfCollisionMatrix.py
@brief Computing robot self collision matrix example.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import sys
import time
from IPython import embed
from RVBUST.RPS import *


def PrintRVDFFormat(collision_matrix):
    print("<self_collision>")
    for link_pair in collision_matrix.m_enabled_pairs:
        print('  <enable_pair link_first = "{0}" link_second = "{1}"/>'.format(
            link_pair[0].GetName(), link_pair[1].GetName()))
    print("</self_collision>")
    print("\n")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        file_path = GetDataPath() + "Multibody/RobotModels/ABB/IRB1200_5_90/IRB1200_5_90.rvdf"
    else:
        file_path = sys.argv[1]

    robot_model = RobotModel()
    res = robot_model.InitFromRVDF(file_path)
    if not res:
        print("\nCompute self collision matrix for ABB IRB1200:\tpython3 UseComputeSelfCollisionMatrix.py")
        print("Compute self collision matrix for other robot:\tpython3 UseComputeSelfCollisionMatrix.py path_to_robot_file.rvdf\n")
        sys.exit(0)
    robot_vis = RobotVis()
    env = Environment()
    robot_vis.LoadEnvironment(env)

    env.AddBody(robot_model)
    robot_vis.ChooseShowMode(robot_model, 2)
    robot_vis.GetView().Home()

    ComputeSelfCollisionMatrix(robot_model, 1000)

    collision_matrix = robot_model.GetSelfCollisionMatrix()

    print(collision_matrix)

    # print rvdf format
    PrintRVDFFormat(collision_matrix)

    checker = FCLCollisionChecker()

    embed()

    while True:
        robot_model.SetToRandomState()
        res, report = checker.CheckCollision(collision_matrix)
        print(res, report)
        time.sleep(.1)

    embed()
