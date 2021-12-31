'''!@example UseBulletDiscreteBVHChecker.py
@brief
@date 2021-10-21
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time

import numpy as np
from IPython import embed
from RVBUST.RPS import *

robot_vis = RobotVis()
env = Environment()
robot_vis.LoadEnvironment(env)
robot = CreateRobotModel("UniversalRobots_UR5")
collision_obj = Multibody()
collision_obj.InitFromBox(0.3, 0.3, 0.3)
collision_obj.SetBaseTransformation(Pose(0.5, 0, 0, 0, 0, 0, 1))
manipulator = robot.GetActiveManipulator()
manipulator.SetDoFPositions(Rx([0, -2, 1.57, 0.5, 1.57, 0]))
env.AddBody(robot)
env.AddBody(collision_obj)
robot_vis.GetView().Home()

controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()

checker = BulletDiscreteBVHChecker()
checker.InitFromEnv(robot_vis.m_env)

# robot_vis.StartDragging(manipulator)
robot_vis.ChooseShowMode(mode=2)
embed()

waypoints = [
    Rx([0.000443443, -1.49466, 2.07202, -0.334425, 1.57044, -3.03574e-05]),
    Rx([0.000358966, -1.53772, 2.33957, -0.558911, 1.57036, -5.0679e-05]),
    Rx([-0.000843149, -2.26811, 2.62627, -0.11522, 1.56919, -0.000339856]),
    Rx([0.451301, -0.833467, 1.94013, -0.838022, 2.00713, 0.115885])]
while True:
    for w in waypoints:
        controller.MoveJoints(w)
        res, report = checker.CheckCollision(
            robot_vis.m_env.GetCollisionMatrix())
        print("check time: ", checker.GetCheckTime())
        print(report)
        if res:
            print("collision")
            first_collision_pair = report.m_collision_report[0]
            h = []
            tail = [first_collision_pair.m_nearest_points0[0],
                    first_collision_pair.m_nearest_points0[1],
                    first_collision_pair.m_nearest_points0[2]]
            head = [first_collision_pair.m_nearest_points0[0] - first_collision_pair.m_contact_normal[0] * 0.3,
                    first_collision_pair.m_nearest_points0[1] -
                    first_collision_pair.m_contact_normal[1] * 0.3,
                    first_collision_pair.m_nearest_points0[2] - first_collision_pair.m_contact_normal[2] * 0.3]
            h.append(robot_vis.m_view.Arrow(tail, head, 0.01))
            embed()
            robot_vis.Delete(h)
        else:
            print("no collision")
            embed()
