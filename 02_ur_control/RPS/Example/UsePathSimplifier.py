#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

from RVBUST.RPS import *
from IPython import embed
import numpy as np
import time


rvis = RCI.RobotVis()
robot_model = RCI.RobotModel()
robot_model.InitFromRVDF(
    "/home/rvbust/Rvbust/Data/Multibody/RobotModels/FANUC/LR_Mate_200iD_7L/LR_Mate_200iD_7L.rvdf")
rvis.AddBody(robot_model)

obstacle1 = RCI.Multibody()
obstacle1.InitFromBox(1.5, 1.5, 0.03)
obstacle1.SetBaseTransformation(RCI.Pose(0, 0, 0.05, 0, 0, 0, 1))
rvis.AddBody(obstacle1)

obstacle2 = RCI.Multibody()
obstacle2.InitFromBox(0.5, 0.08, 0.4)
obstacle2.SetBaseTransformation(RCI.Pose(0.5, 0, 0.2, 0, 0, 0, 1))
rvis.AddBody(obstacle2)

manipulator = robot_model.GetActiveManipulator()
controller = RCI.SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()
controller.SetSpeedRatio(1)
kin_solver = controller.GetKinSolver()

configuration = OmplParallelPlanMotionPlanner.Configuration(rvis.m_env, [
                                                            manipulator])
configuration.planner_types = 1 * [OmplType_RRTConnect]
configuration.simplify_level = Simplify_Level_Medium
planner = OmplParallelPlanMotionPlanner()
planner.SetConfiguration(configuration)

q1 = RCI.Rx([0.577095, 0.923068, -1.25649, -0.0615779, -0.347082, -0.518844])
q2 = RCI.Rx([0, -1.5, 3, 0, 0, 0])
q3 = RCI.Rx([-0.627218, 0.941717, -1.21786, 0.0600466, -0.384641, 0.5712])
waypoints = [q1, q2, q3]

print("\n\n\n Original path \n\n\n")
path = RCI.CreatePath(waypoints, blend_tolerance=0.0)
traj = CreateTrajectory(path, manipulator.GetDoFLimits(),
                        traj_type=RCI.TrajType.TrajType_Trapezoidal)
hs = RCI.DrawTraj(traj, show_in_vis=True, show_in_plt=False,
                  view=rvis.m_view, kin_solver=kin_solver)
embed()
controller.MoveJoints(q1)
controller.ExecuteTrajectory(traj)
rvis.m_view.Delete(hs)


print("\n\n\n Simplified path \n\n\n")
t0 = time.time()
path_simplifier = planner.GetConfiguration().path_simplifier
res, waypoints_ = path_simplifier.SimplifyMax(waypoints)
print("\n\nPath simplify time is {} s\n\n".format(time.time() - t0))

path = RCI.CreatePath(waypoints_, blend_tolerance=0.0)
traj = CreateTrajectory(path, manipulator.GetDoFLimits(),
                        traj_type=RCI.TrajType.TrajType_Trapezoidal)
hs = RCI.DrawTraj(traj, show_in_vis=True, show_in_plt=False,
                  view=rvis.m_view, kin_solver=kin_solver)
embed()
controller.MoveJoints(q1)
controller.ExecuteTrajectory(traj)
rvis.m_view.Delete(hs)

embed()
