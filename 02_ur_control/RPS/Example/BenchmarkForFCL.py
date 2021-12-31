'''!@example BenchmarkForFCL.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import math
import random
import time

import numpy as np
from IPython import embed
from RVBUST.RPS import *

if __name__ == '__main__':
    robot_vis = RobotVis()
    env = Environment()
    robot_vis.LoadEnvironment(env)

    container1 = Multibody()
    container1.InitFromMeshFile(
        GetDataPath() + "Multibody/Containers/PlasticBoxSmall.stl")
    container1.SetBaseTransformation(
        Pose(R3([0.6, 0, -0.1]), Rotation(0, 0, 0)))
    env.AddBody(container1)

    container2 = Multibody()
    container2.InitFromMeshFile(
        GetDataPath() + "Multibody/Containers/PlasticBoxSmall.stl")
    container2.SetBaseTransformation(
        Pose(R3([0, 0.7, -0.1]), Rotation(0, 0, math.pi/2)))
    env.AddBody(container2)

    container3 = Multibody()
    container3.InitFromMeshFile(
        GetDataPath() + "Multibody/Containers/PlasticBoxSmall.stl")
    container3.SetBaseTransformation(
        Pose(R3([-0.7, 0.7, -0.1]), Rotation(0, 0, math.pi/2)))
    env.AddBody(container3)

    robot_model = RobotModel()
    robot_model.InitFromRVDF(
        GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5Limit.rvdf")
    manipulator = robot_model.GetActiveManipulator()
    env.AddBody(robot_model)
    home = Rx([0.5, -2.3561, 1.57, -0.7853, -1.5708, 0])

    gripper = EndEffector()
    gripper.InitFromRVDF(
        GetDataPath() + "Multibody/EndEffectors/VacuumPads/ABBVacuumPads/VacuumPadsSimplified/FourFingerVacuumPads.rvdf")
    env.AddBody(gripper)
    manipulator.SetActiveEndEffector(gripper)
    gripper.SetAttachingPose(Pose(0, 0, 0, 0, 1, 0, 1))

    manipulator = robot_model.GetActiveManipulator()
    controller = SimController.Create(manipulator)
    controller.Connect()
    controller.EnableRobot()
    controller.SetSpeedRatio(1)
    kin_solver = controller.GetKinSolver()

    work_piece = Multibody()
    work_piece.InitFromBox(0.065, 0.065, 0.05)
    # work_piece.SetBaseTransformation(Pose([0.8, 0, 0, 0, 0, 0, 1]))
    # work_piece.SetBaseTransformation(
    #     Pose([0.251542, 0.109149, 0.310131, 1.8366e-06, 0.000301837, 5.54355e-10, -1]))
    work_piece.SetBaseTransformation(
        Pose([0.635646, -0.114438, 0.0601376, -0.21483, 0.00546776, 0.00120274, 0.976635]))
    env.AddBody(work_piece, True)

    embed()

    robot_vis.GetView().SetCameraPose(
        [1.8262971639633179, 1.813581943511963, 2.0244243144989014],
        [1.262405276298523, 1.2450536489486694, 1.4254238605499268],
        [-0.471462607383728, -0.373894065618515, 0.7987028360366821])

    # waypoints for group 1
    pose_g1_0 = Pose([0.251526, 0.109149, 0.335131,
                      1.8366e-06, 0.000301837, 5.54355e-10, -1])
    pose_g1_1 = Pose(
        [0.510385, 0.184371, 0.31744, 0, -0.108837, 0, 0.99406])
    pose_g1_2 = Pose([0.637477, 0.095487, 0.223709,
                      0, 0.00559848, 0, 0.999984])
    pose_g1_3 = Pose([0.6359, -0.103947, 0.0828285, -0.21483,
                      0.00546776, 0.00120274, 0.976635])
    # pose_g1_3 = Pose([0.6359, -0.103947, -0.2528285, -0.21483,
    #                  0.00546776, 0.00120274, 0.976635])
    waypoints_g1 = [pose_g1_0, pose_g1_1, pose_g1_2, pose_g1_3]
    path_g1_se3 = CreatePath(
        waypoints_g1, blend_tolerance=0.1, path_type=PathType_Bezier5thBlend)
    trajectory_g1_se3 = CreateTrajectory(path_g1_se3, speed=0.2)
    controller.ExecuteTrajectory(trajectory_g1_se3)
    embed()

    # perform collision check between robot and obstacles
    res, _, _, joints_g1 = CartTrajToJointSpacePoints(
        trajectory_g1_se3, kin_solver, joint_seed=Rx(home))
    if res == RVSReturn_Success:
        collision_checker_g1 = FCLCollisionChecker()
        collision_checker_g1.InitFromEnv(env)
        collision_matrix_g1 = env.GetCollisionMatrix()
        # collision_matrix_g1 = CollisionMatrix()
        # collision_matrix_g1.EnablePair(robot_model, container1)
        # collision_matrix_g1.EnablePair(robot_model, container2)
        # collision_matrix_g1.EnablePair(gripper, container1)
        # collision_matrix_g1.EnablePair(gripper, container2)
        # embed()
        path_g1 = CreatePath(joints_g1, blend_tolerance=0.1,
                             path_type=PathType_Bezier2ndBlend)
        path_length_g1 = path_g1.GetLength()
        t_ins = 0.0
        resolution = 0.05
        is_collided = False
        start_check_time = time.time()
        while t_ins < path_length_g1:
            start_time = time.time()
            curr_joint_value = path_g1.GetConfig(t_ins)
            robot_model.SetJointPositions(Rx(curr_joint_value))
            is_collided, c_report = collision_checker_g1.CheckCollision(
                collision_matrix_g1)
            if is_collided == True:
                break
            else:
                t_ins += resolution
        terminate_check_time = time.time()
        print("**********************Result****************************")
        if is_collided is True:
            print("There exists collision between robot and obstacles.")
        else:
            print("No collision.")
        print("*********************Analysis***************************")
        print("Time statistics: ")
        print("Total time for checking collision: {:.6f} seconds".format(
            terminate_check_time-start_check_time))
        print("Sync time: {:.6f} seconds.".format(
            collision_checker_g1.GetSyncTime()))
        print("Check time: {:.6f} seconds.".format(
            collision_checker_g1.GetCheckTime()))
        print("***********************End******************************")

    embed()

    gripper.Grab(work_piece)

    # waypoints for group 2
    pose_g2_0 = Pose([0.6359, -0.103947, 0.0828285, 0.21483, -
                      0.00546776, -0.00120274, -0.976635])
    pose_g2_1 = Pose([0.602492, 0.0647285, 0.225658, -
                      0.0317169, 0.00559567, 0.000177568, 0.999481])
    pose_g2_2 = Pose([0.442886, 0.448104, 0.34961, -0.0317169,
                      0.00559567, 0.000177568, 0.999481])
    pose_g2_3 = Pose([0.162563, 0.697234, 0.286396, -
                      0.0314932, -0.107507, 0.00376442, 0.993698])
    pose_g2_4 = Pose([-0.013998, 0.695151, 0.255584, -
                      0.0316438, -0.056878, 0.00215938, 0.997877])
    # pose_g2_5 = Pose([-0.105294, 0.754108, 0.0478262, -
    #                  0.0288039, 0.157671, -0.0162515, 0.986938])
    pose_g2_5 = Pose([-0.105294, 0.754108, -0.178262, -
                      0.0288039, 0.157671, -0.0162515, 0.986938])
    waypoints_g2 = [pose_g2_0, pose_g2_1,
                    pose_g2_2, pose_g2_3, pose_g2_4, pose_g2_5]
    path_g2_se3 = CreatePath(
        waypoints_g2, blend_tolerance=0.1, path_type=PathType_Bezier2ndBlendCartesian)
    trajectory_g2_se3 = CreateTrajectory(path_g2_se3, speed=0.3)
    controller.ExecuteTrajectory(trajectory_g2_se3)
    embed()
    # perform collision check between robot and obstacles
    res, _, _, joints_g2 = CartTrajToJointSpacePoints(
        trajectory_g2_se3, kin_solver, joint_seed=Rx(home))
    if res == RVSReturn_Success:
        collision_checker_g2 = FCLCollisionChecker()
        collision_checker_g2.InitFromEnv(env)
        collision_matrix_g2 = env.GetCollisionMatrix()
        # collision_matrix_g1 = CollisionMatrix()
        # collision_matrix_g1.EnablePair(robot_model, container1)
        # collision_matrix_g1.EnablePair(robot_model, container2)
        # collision_matrix_g1.EnablePair(gripper, container1)
        # collision_matrix_g1.EnablePair(gripper, container2)
        # embed()

        path_g2 = CreatePath(joints_g2, blend_tolerance=0.1,
                             path_type=PathType_Bezier2ndBlend)
        path_length_g2 = path_g2.GetLength()
        t_ins = 0.0
        resolution = 0.05
        is_collided = False
        total_time_sync = 0.0
        total_time_check = 0.0
        start_check_time = time.time()
        while t_ins < path_length_g2:
            start_time = time.time()
            curr_joint_value = path_g2.GetConfig(t_ins)
            robot_model.SetJointPositions(Rx(curr_joint_value))
            is_collided, c_report = collision_checker_g2.CheckCollision(
                collision_matrix_g2)
            if is_collided == True:
                break
            else:
                t_ins += resolution
        terminate_check_time = time.time()
        print("**********************Result****************************")
        if is_collided is True:
            print("There exists collision between robot and obstacles.")
        else:
            print("No collision.")
        print("*********************Analysis***************************")
        print("Time statistics: ")
        print("Total time for checking collision: {:.6f} seconds".format(
            terminate_check_time-start_check_time))
        print("Sync time: {:.6f} seconds.".format(
            collision_checker_g2.GetSyncTime()))
        print("Check time: {:.6f} seconds.".format(
            collision_checker_g2.GetCheckTime()))
        print("***********************End******************************")
        embed()
