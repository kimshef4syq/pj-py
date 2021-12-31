'''!@example UseRVScriptBoxPick.py
@brief Using RVScript to program a box picking task in python.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from pathlib import Path
import numpy as np
from IPython import embed
from RVBUST.RPS import *
from RVBUST.Vis.PyVis import SetLogLevel
SetLogLevel("Off")

HOME_JOINTS = Rx([0, 0, 0, 0, -np.pi/2, 0])

rvis = RobotVis()
robot = LoadRobotModelFromFile(GetRobotFile("Motoman", "GP12"))
mani = robot.GetActiveManipulator()
controller = SimController.Create(mani)
controller.Connect()
controller.EnableRobot()
controller.MoveJoints(HOME_JOINTS)
kin_solver = controller.GetKinSolver()

body = Multibody()
body.InitFromBox(0.3, 0.2, 0.1)
body.SetBaseTransformation(SE3(1, 0, 0.5, 0, 0, 0, 1))

env = Environment()
env.AddBody(body)
env.AddBody(robot)

rvis.AddBody(robot)
rvis.AddBody(body)

container = ScriptContainer.Create()
container += ScriptBlock.CreateBlockMsg("Start to move home")

# move to home
traj_data = TrajectoryData(is_constant_velocity=False)
waypoint = Waypoint(controller.GetJointPosition()[1])
traj_data.AddWaypoint(waypoint)
waypoint = Waypoint(HOME_JOINTS)
traj_data.AddWaypoint(waypoint)
container += ScriptBlock.CreateBlockTraj(traj_data)

# loop statements
statements = ScriptContainer.Create()
# receive pick pose
block = ScriptBlock.CreateBlockRemoteUpdateBodyPose(
    body.GetUniqueName(), "127.0.0.1", 9999)
statements += block

# pick
traj_data = TrajectoryData(blend_tolerance=0.5, motion_type=MotionType_MoveL,
                           is_constant_velocity=False, velocity=0.5, traj_type=TrajType_Trapezoidal)
waypoint = Waypoint(HOME_JOINTS)
traj_data.AddWaypoint(waypoint)
waypoint = Waypoint(SE3([-0.016, 0, 0.2, 1, 0, -0, 0]),
                    controller.GetJointPosition()[1], body.GetUniqueName())
traj_data.AddWaypoint(waypoint)
waypoint = Waypoint(SE3([-0.016, 0, 0.05, 1, 0, -0, 0]),
                    controller.GetJointPosition()[1], body.GetUniqueName())
traj_data.AddWaypoint(waypoint)
statements += ScriptBlock.CreateBlockTraj(traj_data)
statements += ScriptBlock.CreateBlockGrab(body.GetUniqueName(), 1, True)

# place
traj_data = TrajectoryData(blend_tolerance=0.1, motion_type=MotionType_MoveJ,
                           is_constant_velocity=False, velocity=0.5, traj_type=TrajType_Trapezoidal)
waypoint = Waypoint(SE3([-0.016, 0, 0.05, 1, 0, -0, 0]),
                    controller.GetJointPosition()[1], body.GetUniqueName())
traj_data.AddWaypoint(waypoint)
waypoint = Waypoint(
    Rx([1.25331, 0.9409, -0.413536, 0, -0.216364, 8.88178e-16]))
traj_data.AddWaypoint(waypoint)
statements += ScriptBlock.CreateBlockTraj(traj_data)
statements += ScriptBlock.CreateBlockRelease(body.GetUniqueName(), 1, False)

# move to home
traj_data = TrajectoryData(blend_tolerance=0.1, motion_type=MotionType_MoveJ,
                           is_constant_velocity=False, velocity=0.5, traj_type=TrajType_Trapezoidal)
waypoint = Waypoint(HOME_JOINTS)
traj_data.AddWaypoint(waypoint)
statements += ScriptBlock.CreateBlockTraj(traj_data)

container += ScriptBlock.CreateBlockLogicFor(statements, 10)

prog_path = Path.home() / 'BinPick.json'
with open(prog_path, 'w') as f:
    f.write(container.ToJson())

executor = ScriptExecutor(controller, env)
executor.ExecuteScriptContainer(container)

# with open(prog_path, 'r') as f:
#     jstr = f.read()
# container2 = ScriptContainer.Create()
# container2.FromJson(jstr)
# executor.ExecuteScriptContainer(container)

embed()