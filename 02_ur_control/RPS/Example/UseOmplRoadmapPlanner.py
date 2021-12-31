'''!@example UseOmplRoadmapPlanner.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time

import numpy as np
from IPython import embed
from RVBUST.RPS import *

robot_vis = RobotVis()
env = Environment()
robot_vis.LoadEnvironment(env)
robot = RobotModel()
collision_obj = Multibody()
robot.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5.rvdf")
collision_obj.InitFromMeshFile(
    GetDataPath() + "Multibody/3DModels/RVBUSTLogo.stl")

robot_vis.GetView().Home()
robot_vis.GetView().SetCameraPose([-0.5351742506027222, -2.8529000282287598, 1.9045449495315552],
                                  [-0.2558063566684723, -
                                   1.9871060848236084, 1.489389181137085],
                                  [0.09571810066699982, 0.40510550141334534, 0.9092456102371216])

collision_obj.SetBaseTransformation(Pose(.5, 0, .3, 0, -0.4, 0, 1))

start = Rx([-0.16891239, 0.67904491, -1.90386973, -
            1.09577485, 1.85397128, 0.95098244])
goal = Rx([-0.28952, -1.58203, 2.10125, -2.85594, -1.1842, 0.323])
manipulator = robot.GetActiveManipulator()
manipulator.SetDoFPositions(start)
controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()

max_vel, max_acc, max_jerk = ConvertJointLimits2Arr(manipulator.GetDoFLimits())

env.AddBody(robot)
env.AddBody(collision_obj)

filepath = GetDataPath() + "Temp/PlannerData.dat"

configuration = OmplRoadmapMotionPlanner.Configuration(env, [manipulator])
configuration.smooth = False
configuration.simplify = True
configuration.simplify_level = Simplify_Level_Medium
configuration.roadmap_file = filepath
planner = OmplRoadmapMotionPlanner()
embed()

planner.SetConfiguration(configuration)

controller.MoveJoints(start)
verbose = True

while True:
    embed()

    request = planner.CreateRequest(start, goal)

    t0 = time.time()
    res, response = planner.Solve(request, verbose)

    planning_time = time.time() - t0
    print("\n\nPath cost is {}, planning time is {} s\n\n".format(
        response.path_cost, planning_time))
    path = CreatePath(response.joint_trajectory, 0.1,
                      PathType_Bezier2ndBlend)
    traj = CreateTrajectory(path, max_vel, max_acc,
                            traj_type=TrajType_Trapezoidal)

    controller.ExecuteTrajectory(traj)
    # embed()

    start, goal = goal, start

embed()
