'''!@example UseLogger.py
@brief The script (UseLogger.py) is written to delineate how to use 
new Logger functions. The content will be synchronously updated 
if new log rules will be adopted in the future 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time
from IPython import embed
from RVBUST.RPS import *


def TestLogger(robot_vis, logger):
    env = Environment()
    robot_vis.LoadEnvironment(env)

    container1 = Multibody()
    container1.InitFromMeshFile(
        "/home/rvbust/Rvbust/Data/Multibody/Containers/PlasticBoxSmall.stl")
    container1.SetBaseTransformation(
        Pose(R3([0.6, 0, -0.1]), Rotation(0, 0, 0)))
    env.AddBody(container1)

    robot_model = RobotModel()
    robot_model.InitFromRVDF(
        "/home/rvbust/Rvbust/Data/Multibody/RobotModels/UniversalRobots/UR5/UR5Limit.rvdf")
    manipulator = robot_model.GetActiveManipulator()
    env.AddBody(robot_model)
    home = Rx([0.5, -2.3561, 1.57, -0.7853, -1.5708, 0])

    gripper = EndEffector()
    gripper.InitFromRVDF(
        "/home/rvbust/Rvbust/Data/Multibody/EndEffectors/VacuumPads/ABBVacuumPads/VacuumPadsSimplified/FourFingerVacuumPads.rvdf")
    env.AddBody(gripper)
    manipulator.SetActiveEndEffector(gripper)
    gripper.SetAttachingPose(Pose(0, 0, 0, 0, 1, 0, 1))

    manipulator = robot_model.GetActiveManipulator()
    controller = SimController.Create(manipulator)
    controller.Connect()
    controller.EnableRobot()
    controller.SetSpeedRatio(1)
    kin_solver = controller.GetKinSolver()

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
    # pose_g1_3 = Pose([0.6359, -0.103947, 0.0828285, -0.21483,
    #                  0.00546776, 0.00120274, 0.976635])
    pose_g1_3 = Pose([0.6359, -0.103947, -0.2528285, -0.21483,
                      0.00546776, 0.00120274, 0.976635])
    waypoints_g1 = [pose_g1_0, pose_g1_1, pose_g1_2, pose_g1_3]
    path_g1_se3 = CreatePath(
        waypoints_g1, blend_tolerance=0.1, path_type=PathType_Bezier2ndBlendCartesian)
    trajectory_g1_se3 = CreateTrajectory(path_g1_se3, speed=0.5)
    controller.ExecuteTrajectory(trajectory_g1_se3)
    # perform collision check between robot and obstacles
    res, _, _, joints_g1 = CartTrajToJointSpacePoints(
        trajectory_g1_se3, kin_solver, joint_seed=Rx(home))
    if res == RVSReturn_Success:
        collision_checker_g1 = FCLCollisionChecker()
        collision_checker_g1.InitFromEnv(env)
        collision_matrix_g1 = env.GetCollisionMatrix()
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
        logger.Trace(
            "**********************Result****************************")
        if is_collided is True:
            logger.Warn("There exists collision between robot and obstacles.")
        else:
            logger.Info("No collision.")
        logger.Trace(
            "*********************Analysis***************************")
        logger.Trace("Time statistics: ")
        logger.Info("Total time for checking collision: {:.6f} seconds".format(
            terminate_check_time-start_check_time))
        logger.Info("Sync time: {:.6f} seconds.".format(
            collision_checker_g1.GetSyncTime()))
        logger.Info("Check time: {:.6f} seconds.".format(
            collision_checker_g1.GetCheckTime()))
        logger.Trace(
            "***********************End******************************")
    robot_vis.RemoveBody(container1)
    robot_vis.RemoveBody(robot_model)


if __name__ == '__main__':
    print("\n**********************Test Logger**************************")
    print("Type \"exit\" to continue...\n\n")
    embed()
    # define a local console logger
    print("Define a local console logger and print test info: ")
    logger = Logger.GetConsoleLogger("UseLogger")
    logger.Info("Hello world, I'm a logger.")
    print("Type \"exit\" to continue...\n\n")
    embed()

    print("\n\n----------------------step 1----------------------------")
    print("Print all registered loggers' names: ")
    print("===================Logger names (start)==================")
    print(Logger.GetRegisteredLoggerNames())
    print("===================Logger names (end)====================")
    print("Type \"exit\" to continue...\n\n")
    embed()

    print("\n\n----------------------step 2----------------------------")
    print("Output logs from available loggers: (level: LoggerLevel_Info)")
    robot_vis = RobotVis()
    print("======================Log (start)========================")
    TestLogger(robot_vis=robot_vis, logger=logger)
    print("=======================Log (end)=========================")

    print("\n\n----------------------step 3----------------------------")
    print("Output logs from available loggers according to given \n"
          "level (here is \"LoggerLevel_Trace\")")
    Logger.OutputRegisteredLoggers(
        allow=True, level=LoggerLevel_Trace)
    print("Type \"exit\" to continue...\n\n")
    embed()
    print("======================Log (start)========================")
    TestLogger(robot_vis=robot_vis, logger=logger)
    print("=======================Log (end)=========================")

    print("\n\n----------------------step 4----------------------------")
    print("Disable output from loggers whose names are: ")
    print("n1: \"RVS\"")
    Logger.OutputRegisteredLoggers(
        allow=True, level=LoggerLevel_Info)
    Logger.SpecifyDisabledLoggerList(
        logger_names=["RVS"])
    print("Type \"exit\" to continue...\n\n")
    embed()
    print("======================Log (start)========================")
    TestLogger(robot_vis=robot_vis, logger=logger)
    print("=======================Log (end)=========================")

    print("\n\n----------------------step 5----------------------------")
    print("Specify output from loggers whose names are: ")
    print("n1: \"RVS\"")
    Logger.DisableRegisteredLoggers(disable=True)
    Logger.SpecifyOutputLoggerList(
        logger_names=["RVS"],
        level=LoggerLevel_Info)
    print("Type \"exit\" to continue...\n\n")
    embed()
    print("======================Log (start)========================")
    TestLogger(robot_vis=robot_vis, logger=logger)
    print("=======================Log (end)=========================")

    print("\n\n----------------------step 6----------------------------")
    print("Disable output from all loggers: ")
    Logger.DisableRegisteredLoggers(disable=True)
    print("Type \"exit\" to continue...\n\n")
    embed()
    print("=============Start(no log will be printed)==============")
    TestLogger(robot_vis=robot_vis, logger=logger)
    print("===============End(no log will be printed)==============")

    print("\n\n----------------------step 7----------------------------")
    print("Output log into file: ")
    log_file_path = "/home/rvbust/TestLog/rvs.log"
    Logger.OutputRegisteredLoggers(
        allow=True, level=LoggerLevel_Info)
    logger.AddFileLogger(log_file_path,
                         LoggerLevel_Info)
    print("Type \"exit\" to continue...\n\n")
    embed()
    logger.Info("This text will be record in conosle and file simultaneously.")
    for i in range(100):
        logger.Info(
            "This is test example for printing log into file and console."
            "(see file \"{}\")".format(log_file_path))

    print("\n\n----------------------step 8----------------------------")
    print("Set output log level for file sink and console sink separately: ")
    logger.SetLevelForConsole("UseLogger", LoggerLevel_Warn)
    logger.SetLevelForFile("UseLogger", LoggerLevel_Trace)
    print("Type \"exit\" to continue...\n\n")
    embed()
    print("======================Log (start)========================")
    for i in range(50):
        logger.Trace("The info will only be printed into file."
                     "(see file \"{}\")".format(log_file_path))
    for i in range(50):
        logger.Error(
            "The info will be exported into console and file simultaneously."
            "(see file \"{}\")".format(log_file_path))
    print("=======================Log (end)=========================")

    print("\n\n----------------------step 9----------------------------")
    print("Output log into all files corresponding to registered loggers: ")
    logger.AddFileLoggerToAll(
        log_file_path, LoggerLevel_Info)
    print("Type \"exit\" to continue...\n\n")
    embed()
    print("======================Log (start)========================")
    for i in range(20):
        logger.Info("The text is from \"{}\" logger.".format(logger.GetName()))
    TestLogger(robot_vis=robot_vis, logger=logger)
    print("=======================Log (end)=========================\n"
          "(see file \"{}\")".format(log_file_path))

    print("\n\n**********************END(Test Logger)*******************\n\n")
    print("---Please refer to class Logger for utilizing other member methods...---\n\n")
    embed()
