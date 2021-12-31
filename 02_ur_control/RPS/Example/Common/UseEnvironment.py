'''!@example UseEnvironment.py
@brief 
@date 2021-08-20
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from IPython import embed
from RVBUST.RPS import *

robot_logger = Logger.GetConsoleLogger("RVS_CL")
robot_logger.SetLevel(2)
env = Environment()

container1 = Multibody()
container1.InitFromMeshFile(
    GetDataPath() + "Multibody/Containers/BlueBoxThickened.stl")
container1.SetName("container1")
container1.SetBaseTransformation(Pose(0.5, 0, 0.0, 0, 0, 0, 1))

container2 = Multibody()

container2.InitFromMeshFile(
    GetDataPath() + "Multibody/Containers/BlueBoxThickened.stl")
container2.SetName("container2")
container2.SetBaseTransformation(Pose(0, 0.5, 0.137, 0, 0, 1, 1))


robot = RobotModel()
robot.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/ABB/IRB1200_5_90/IRB1200_5_90.rvdf")
manipulator = robot.GetActiveManipulator()

gripper = EndEffector()
gripper.InitFromRVDF(
    GetDataPath() + "Multibody/EndEffectors/VacuumPads/ABBVacuumPads/VacuumPadsSimplified/FourFingerVacuumPadsSimplified.rvdf")
env.AddBodies([container1, container2, robot, gripper])
manipulator.SetActiveEndEffector(gripper)

controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()


robot_vis = RobotVis()

robot_logger.Info("\nConstruct environment\n")
robot_vis.LoadEnvironment(env)
robot_vis.GetView().Home()
embed()


robot_logger.Info("\nRemove container1\n")
env.RemoveBody(container1)
embed()

robot_logger.Info("\nAdd container1\n")
env.AddBody(container1)
embed()

robot_logger.Info("\nCopy environment\n")
env_copy = env.Copy()
robot_vis.LoadEnvironment(env_copy)

embed()

# Load and save json file
robot_logger.Info("\nSave json file\n")
file_path = "./Environment.glb"
env_copy.SaveToFile(file_path)


embed()

# robot_logger.Info("\nLoad json file\n")
env_load = Environment()
env_load.LoadFromFile(file_path)
robot_vis.LoadEnvironment(env_load)
embed()
