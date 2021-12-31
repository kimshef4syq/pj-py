'''!@example UseRobotProgram.py
@brief Using RVScript demo.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import os
import time

import numpy as np
from IPython import embed

from RVBUST.RPS import *

Logger.SetLevelForAll(Logger.LoggerLevel_Warn)
Logger.GetConsoleLogger("RVScript").SetLevel(2)
pi = np.pi

home_position = [pi, -pi/2,  pi/2, -pi/2, -pi/2,  0]
model = RobotModel()
model.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5.rvdf")
model.SetName(model.GetName() + "_sim")
rvis = RobotVis()
rvis.AddBody(model)
controller = SimController.Create(model.GetActiveManipulator())
controller.SetJointThreshold(10)
controller.SetSpeedRatio(1.0)
controller.Connect()
controller.EnableRobot()
controller.MoveJoints(home_position)
kin_solver = controller.GetKinSolver()
cart_limits = controller.GetCartesianLimits()

hs = []
traj_data = TrajectoryData(blend_tolerance=0.01,
                           motion_type=MotionType.MotionType_MoveJ,
                           is_constant_velocity=False,
                           velocity=0.4)
container = ScriptContainer.Create(ScriptBlock.CreateBlockMsg("start"))

# go home with movej
traj_data.SetName('Go Home Traj')
traj_data.AddWaypoint(home_position)
position = [3.14085, -1.80829, 1.77913, -1.54171, -1.57131, -0.000622098]
traj_data.AddWaypoint(position)

traj_data.SetMotionType(MotionType_MoveJ)
hs.append(DrawTraj(traj_data.GenerateJointTrajectory(kin_solver), show_in_vis=True, show_in_plt=False,
                   view=rvis.GetView(), kin_solver=kin_solver))
container += ScriptBlock.CreateBlockTraj(traj_data.Copy())
container += ScriptBlock.CreateBlockSetDO(0, True)
container += ScriptBlock.CreateBlockSetDO(1, True)

# Move along a circle
position0 = [3.14085, -1.80829, 1.77913, -1.54171, -1.57131, -0.000622098]
position1 = [3.14056, -2.05242, 2.18614, -1.70472, -1.57143, -0.00105268]
position2 = [3.14087, -1.68014, 2.21005, -2.10088, -1.57142, -0.000932519]
block = ScriptBlock.CreateBlockTrajCircle(Waypoint(Rx(position0)), Waypoint(
    Rx(position1)), Waypoint(Rx(position2)), 0.1, False)
hs.append(DrawTraj(block.traj_data.GenerateCartesianTrajectory(kin_solver, cart_limits=cart_limits), show_in_vis=True, show_in_plt=False,
                   view=rvis.GetView(), kin_solver=kin_solver))
condition = ScriptBlock.CreateBlockGetDO(0)
statements = ScriptContainer.Create()
statements += ScriptBlock.CreateBlockMsg("Execute if block")
statements += block
container += ScriptBlock.CreateBlockLogicIf(condition, True, statements)

traj_data.SetMotionType(MotionType_MoveJ)
condition = ScriptBlock.CreateBlockGetDO(1)
statements = ScriptContainer.Create(
    ScriptBlock.CreateBlockMsg("Execute elif block"))
statements += ScriptBlock.CreateBlockTraj(traj_data.Copy())
container += ScriptBlock.CreateBlockLogicElif(
    condition, True, statements)

traj_data.SetMotionType(MotionType_MoveL)
statements = ScriptContainer.Create([ScriptBlock.CreateBlockMsg("Execute else block"),
                                     ScriptBlock.CreateBlockTraj(traj_data.Copy())])
container += ScriptBlock.CreateBlockLogicElse(statements)

container += ScriptBlock.CreateBlockSetDO(3, True)

traj_data.ClearWaypoints()
traj_data.SetMotionType(MotionType_MoveL)
position = [3.14087, -1.68014, 2.21005, -2.10088, -1.57142, -0.000932519]
traj_data.AddWaypoint(position)
position = [3.14099, -1.44497, 1.97245, -2.0985, -1.57145, -0.000717465]
traj_data.AddWaypoint(position)
hs.append(DrawTraj(traj_data.GenerateCartesianTrajectory(kin_solver, cart_limits=cart_limits), show_in_vis=True, show_in_plt=False,
                   view=rvis.GetView(), kin_solver=kin_solver))
container += ScriptBlock.CreateBlockTraj(traj_data.Copy())

# Move along a rectangle
traj_data.ClearWaypoints()
traj_data.SetMotionType(MotionType_MoveJ)
traj_data.SetBlendTolerance(0.1)
traj_data.SetIsConstantVelocity(False)
position = [3.14099, -1.44497, 1.97245, -2.0985, -1.57145, -0.000717465]
traj_data.AddWaypoint(position)
position = [3.51664, -1.27402, 1.75626, -2.05291, -1.5709, 0.375317]
traj_data.AddWaypoint(position)
position = [3.51671, -1.38232, 1.36408, -1.55239, -1.57093, 0.375245]
traj_data.AddWaypoint(position)
position = [2.73441, -1.579, 1.58023, -1.57196, -1.57127, -0.407115]
traj_data.AddWaypoint(position)
position = [2.73442, -1.45294, 1.9812, -2.099, -1.57136, -0.407079]
traj_data.AddWaypoint(position)
position = [3.14099, -1.44497, 1.97245, -2.0985, -1.57145, -0.000717465]
traj_data.AddWaypoint(position)
hs.append(DrawTraj(traj_data.GenerateJointTrajectory(kin_solver), show_in_vis=True, show_in_plt=False,
                   view=rvis.GetView(), kin_solver=kin_solver))
condition = ScriptBlock.CreateBlockGetDO(3)
statements = ScriptContainer.Create([ScriptBlock.CreateBlockMsg("Execute while block"),
                                     ScriptBlock.CreateBlockTraj(
                                         traj_data.Copy()),
                                     ScriptBlock.CreateBlockLogicWait(0.5)])
embed_if_condition = ScriptBlock.CreateBlockGetDO(0)
embed_if_statements = ScriptContainer.Create(
    ScriptBlock.CreateBlockMsg("embed if block in while block"))
statements += ScriptBlock.CreateBlockLogicIf(
    embed_if_condition, True, embed_if_statements)
statements += ScriptBlock.CreateBlockSetDO(3, False)
container += ScriptBlock.CreateBlockLogicWhile(condition, True, statements)

traj_data.SetMotionType(MotionType_MoveJ)
traj_data.SetIsConstantVelocity(False)
statements = ScriptContainer.Create([ScriptBlock.CreateBlockMsg("execute for block"),
                                     ScriptBlock.CreateBlockTraj(traj_data)])
container += ScriptBlock.CreateBlockLogicFor(statements, 2)

executor = ScriptExecutor(controller)
# execute program without blocking
executor.ExecuteScriptContainer(container, False)
time.sleep(4)
executor.Terminate()
embed()
executor.ExecuteScriptContainer(container, False, False)
time.sleep(2)
executor.Terminate()
embed()
executor.ExecuteScriptContainer(container, True, False)
print("executor is_busy: {}, result: {}".format(
    executor.IsBusy(), executor.GetExecuteProgResult()))
embed()
# time.sleep(5)  # waiting for executing
# executor.Terminate()  # terminate executing

"""
############## Test with real controller #################
from RVS.PyUR import UR5Controller
real_controller = UR5Controller()
real_controller.Connect()
real_controller.EnableRobot()
real_controller.SetSpeedRatio(1.0)
# Test get robot program list
ret, prog_list = real_controller.GetRobotProgramList()
print("\n".join(prog_list))
# Test save as robot program
path = "/home/rvbust/.Temp/RVS_AUTO.script"
ret = real_controller.SaveAsRobotProgram(container, path)
assert ret == RVSReturn_Success, "Save as robot program failed"
# Test upload program
ret = real_controller.UploadRobotProgram(path)
assert ret == RVSReturn_Success, "Upload robot program failed"
# Test execute program
ret = real_controller.ExecuteRobotProgram(path, False)
assert ret == RVSReturn_Success, "Execute robot program failed"
ret, current_prog = real_controller.GetCurrentRobotProgram()
assert ret == RVSReturn_Success, "Get current robot program failed"
time.sleep(10)
ret = real_controller.HoldRobotProgram()
assert ret == RVSReturn_Success, "Hold robot program failed"
time.sleep(5)
ret = real_controller.ResumeRobotProgram()
assert ret == RVSReturn_Success, "Resume robot program failed"

# Test download program
path_download_prog = "/home/rvbust/.Temp/RVS_AUTO_DOWNLOAD"
ret = real_controller.DownloadRobotProgram("RVS_AUTO", path_download_prog)
assert ret == RVSReturn_Success, "Download robot program failed"
# Test delete robot program
ret = real_controller.DeleteRobotProgram("RVS_AUTO")
assert ret == RVSReturn_Success, "Delete robot program failed"
############################################################
embed()
"""
