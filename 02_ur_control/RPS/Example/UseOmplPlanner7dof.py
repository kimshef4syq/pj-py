'''!@example UseOmplPlanner7dof.py
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
    GetDataPath() + "Multibody/RobotModels/Motoman/SIA10F/SIA10F.rvdf")
collision_obj.InitFromMeshFile(
    GetDataPath() + "Multibody/3DModels/RVBUSTLogo.stl")

robot_vis.GetView().Home()
robot_vis.GetView().SetCameraPose([-0.5351742506027222, -2.8529000282287598, 1.9045449495315552],
                                  [-0.2558063566684723, -
                                   1.9871060848236084, 1.489389181137085],
                                  [0.09571810066699982, 0.40510550141334534, 0.9092456102371216])

collision_obj.SetBaseTransformation(Pose(.5, 0, .5, 0, -0.4, 0, 1))

start = Rx([-1.45043144,  0.41814403, -1.33434741,
            1.8392456,  1.15866588, 0.70217683,  2.16850603])
goal = Rx([0., 1.76648, 0.103305, 0.74603, 0.2, 0.970935, 0.839382])

manipulator = robot.GetActiveManipulator()
manipulator.SetDoFPositions(start)
controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()

max_vel, max_acc, max_jerk = ConvertJointLimits2Arr(manipulator.GetDoFLimits())

env.AddBody(robot)
env.AddBody(collision_obj)

configuration = OmplParallelPlanMotionPlanner.Configuration(env, [manipulator])
configuration.planning_time = 10
configuration.simplify = True
configuration.simplify_level = Simplify_Level_Medium
configuration.planner_types = 1 * [OmplType_RRTConnect]
planner = OmplParallelPlanMotionPlanner()
planner.SetConfiguration(configuration)

embed()

controller.MoveJoints(start)

while(True):
    # embed()
    request = planner.CreateRequest(start, goal)
    t0 = time.time()
    res, response = planner.Solve(request)

    planning_time = time.time() - t0
    print("\nPath cost is {}, planning time is {} s\n".format(
        response.path_cost, planning_time))
    path = CreatePath(response.joint_trajectory, 0.1,
                      PathType_Bezier2ndBlend)
    traj = CreateTrajectory(path, max_vel, max_acc, max_jerk,
                            traj_type=TrajType_Trapezoidal)

    controller.ExecuteTrajectory(traj)
    # embed()

    start, goal = goal, start

embed()
