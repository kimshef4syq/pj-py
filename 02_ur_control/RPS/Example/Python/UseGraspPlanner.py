'''!@example UseGraspPlanner.py
@date 2021-10-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import numpy as np
from IPython import embed
from RVBUST.RPS import *
import time
import random
import json5

RCI.Logger.SetLevelForAll(2)
logger = RCI.Logger.GetConsoleLogger("UseGraspPlanner")


def GetWorkpiecePoses():
    pose = []
    for i in range(5):
        x = random.uniform(-0.4, 0.4)
        y = random.uniform(0.6, 1.0)
        z = random.uniform(-0.5, -0.2)
        rx = random.uniform(-3.14, 3.14)
        ry = random.uniform(-3.14, 3.14)
        rz = random.uniform(-3.14, 3.14)
        pose.append(RCI.Pose(RCI.R3([x, y, z]), RCI.Rotation(rx, ry, rz)))
    return pose


# Construct environment
glb_file = "/home/rvbust/Rvbust/Data/Scenes/BinPicking/JAKABinPick/Env2.glb"
env = RCI.Environment()
env.LoadFromFile(glb_file)
workpiece = env.GetBodiesByName("shhsa_40")[0]

robot_vis = RCI.RobotVis()
robot_vis.LoadEnvironment(env)
robot_vis.m_view.Home()

robot = env.GetAllRobotModels()[0]
manipulator = robot.GetActiveManipulator()
controller = RCI.SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()
controller.SetSpeedRatio(0.3)
joint_limits = manipulator.GetDoFLimits()

# grasp pose info
approach_tangent = RCI.SE3Tangent(0, 0, -0.08, 0, 0, 0)
linear_approach_step = 0.01
retreat_tangent = RCI.SE3Tangent(0, 0, 0.1, 0, 0, 0)
center_retreat_percentage = 0.2
linear_retreat_step = 0.01
# 16 grasp pose
poses_in_obj = []
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0, 0, 0, 1]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0, 0, 0.382683, 0.92388]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0, 0, 0.707107, 0.707107]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0, 0, 0.92388, 0.382683]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0, 0, 1, 2.67949e-08]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0, 0, 0.92388, -0.382683]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0, 0, 0.707107, -0.707107]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0, 0, 0.382683, -0.92388]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 1, 0, 0, 0]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0.382683, 0.92388, 0, 0]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0.707107, 0.707107, 0, 0]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0.92388, 0.382683, 0, 0]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 1, 0, 0, 0]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0.92388, -0.382683, 0, 0]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0.707107, -0.707107, 0, 0]))
poses_in_obj.append(RCI.Pose([0, 0, 0.02, 0.382683, -0.92388, 0, 0]))
# grasp planner
grasp_config = GraspPlanner.Configuration(env, [manipulator])
grasp_config.workpiece_name = "shhsa_40"
pick_region = RCI.Box(0.62, 0.97, 0.6)
pick_region.pose = RCI.Pose([0, 0.81, -0.1, 0, 0, 0.707107, 0.707107])
grasp_config.pick_region = pick_region
grasp_config.pick_region_seed = RCI.Rx(
    [1.42769, 0.688832, -2.01708, 2.90686, 1.58078, 0.659972])
pick_infos = []
for i in range(16):
    pick_info_temp = PickInfo()
    pick_info_temp.pose_in_obj = poses_in_obj[i]
    pick_info_temp.approach_tangent = approach_tangent
    pick_info_temp.linear_approach_step = linear_approach_step
    pick_info_temp.retreat_tangent = retreat_tangent
    pick_info_temp.center_retreat_percentage = center_retreat_percentage
    pick_info_temp.linear_retreat_step = linear_retreat_step
    pick_infos.append(pick_info_temp)
grasp_config.pick_infos = pick_infos
grasp_planner = GraspPlanner()
res = grasp_planner.SetConfiguration(grasp_config)
if not res:
    logger.Error("grasp planner set configuration failed")


embed()
poses = []
workpiece_handle = []

while True:
    if len(workpiece_handle):
        env.RemoveBodies(workpiece_handle)
        time.sleep(0.5)
        workpiece_handle.clear()

    poses = GetWorkpiecePoses()

    for i in range(len(poses)):
        workpiece_handle.append(workpiece.Copy())
        workpiece_handle[-1].SetBaseTransformation(poses[i])
        env.AddBody(workpiece_handle[-1])
    embed()

    start_time = time.time()
    request = GraspPlanner.CreateRequest(poses)
    res, response = grasp_planner.Solve(request)
    print("Grasp planning time is :", time.time() - start_time)
    if not res:
        embed()
        continue

    controller.MoveJoints(response.success_res.pick_approach_waypoints[0])
    embed()

    path = RCI.CreatePath(response.success_res.pick_approach_waypoints, 0.1)
    traj = CreateTrajectory(path, joint_limits, RCI.TrajType_DoubleS)
    controller.ExecuteTrajectory(traj)
    embed()

    path = RCI.CreatePath(response.success_res.pick_retreat_waypoints, 0.1)
    traj = CreateTrajectory(path, joint_limits, RCI.TrajType_DoubleS)
    controller.ExecuteTrajectory(traj)
    embed()
