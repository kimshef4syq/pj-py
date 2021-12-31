'''!@example UseFCLCollisionChecker.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from RVBUST.RPS import *
from IPython import embed
import time

robot_logger = Logger.GetConsoleLogger("RVS_CL")
robot_logger.SetLevel(0)
robot_vis = RobotVis()
env = Environment()
robot_vis.LoadEnvironment(env)

# creat Multibody from two bodies and joint_info
robot = RobotModel()
robot.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/ABB/IRB6700_235_265/IRB6700_235_265.rvdf")
manip = robot.GetActiveManipulator()

gripper = EndEffector()
gripper.InitFromRVDF(
    GetDataPath() + "Multibody/EndEffectors/FANUCSpotWeldingGun/SpotWeldingGun.rvdf")
manip.SetActiveEndEffector(gripper)
# create box
box = Multibody()
box.InitFromBox(0.1, 0.2, 0.3)
box.SetBaseTransformation(Pose([1.5, 0, 0.7, 0, 0, 0, 1]))
env.AddBody(box)
env.AddBody(gripper)
env.AddBody(robot)

manipulator = robot.GetActiveManipulator()
controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()
bp = BasicMotionUtilities(controller)
bp.MoveBZ(-0.55)

# collision checker
robot_vis.ChooseShowMode(mode=2)
robot_vis.GetView().Home()
collision_checker = FCLCollisionChecker()
collision_matrix = CollisionMatrix()
collision_matrix.EnablePair(robot, box)
collision_matrix.EnablePair(gripper, box)

print(collision_matrix)
embed()
[result, collision_reporter] = collision_checker.CheckCollision(
    collision_matrix)
robot_logger.Info("CheckCollision result {}".format(result))

collision_checker.SetContactDistanceThreshold(0.001)
robot_logger.Info("contact distance threshold {}".format(
    collision_checker.GetContactDistanceThreshold()))
[result, collision_reporter] = collision_checker.CheckCollision(
    collision_matrix)
robot_logger.Info("CheckDistanceCollision result {}".format(result))
embed()
