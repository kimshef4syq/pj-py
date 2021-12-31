'''!@example UseEndEffector.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import math
import random

from IPython import embed
from RVBUST.RPS import *

robot_logger = Logger.GetConsoleLogger("RVS_CL")
robot_logger.SetLevel(2)
robot_vis = RobotVis()
env = robot_vis.m_env


home = Rx([-0.19081442, -1.63731174,  1.01860766, -
           0.95376841, -1.5709634, -0.10605834])
start = Rx([-0.21145149, -1.4599196,  1.82301122, -
            1.93456881, -1.57097389, -0.2115038])
goal = Rx([1.41856167, -0.99444799,  1.18237065, -
           1.75951831, -1.57036389, -0.15135137])
grasp_pose = Pose([0.700147, -6.95075e-05, 0.0401131,
                   0.000158261, 0.000314213, -2.61262e-05, -1])
place_pose = Pose([0, 0.9, 0.03, 0, 0, 0.707107, 0.707107])
grasp_pose_handle = robot_vis.PlotFrame(grasp_pose, 0.1, 1)
place_pose_handle = robot_vis.PlotFrame(place_pose, 0.1, 1)

max_vel = 1 * np.array([4, 4, 4, 4, 4, 4])
max_acc = 1 * np.array([12, 12, 12, 12, 12, 12])
max_jerk = np.ones_like(max_acc)

# init scene

container1 = Multibody()
container1.InitFromMeshFile(
    GetDataPath() + "Multibody/Containers/PlasticBox.dae")
container1.SetBaseTransformation(
    Pose(R3([0.6, 0, 0]), Rotation(0, 0, math.pi/2)))
env.AddBody(container1)

container2 = Multibody()
container2.InitFromMeshFile(
    GetDataPath() + "Multibody/Containers/PlasticBinContainer.dae")
container2.SetBaseTransformation(Pose(R3([0, 0.9, 0]), Rotation()))
env.AddBody(container2)

box = Multibody()
box.InitFromBox(0.06, 0.05, 0.02)
box.SetBaseTransformation(Pose(R3([0.7, 0, 0.03]), Rotation()))
env.AddBody(box)

robot_model = RobotModel()
robot_model.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5Limit.rvdf")
robot_model.SetJointPositions(home)
env.AddBody(robot_model)

gripper = EndEffector()
gripper.InitFromRVDF(
    GetDataPath() + "Multibody/EndEffectors/VacuumPads/ABBVacuumPads/VacuumPadsSimplified/FourFingerVacuumPadsSimplified.rvdf")
env.AddBody(gripper)

robot_vis.GetView().Home()
v1 = [1.3254531621932983, 2.9727749824523926, 2.280062198638916]
v2 = [0.9919242858886719, 2.224724292755127, 1.706321358680725]
v3 = [-0.4827246069908142, -0.3872399628162384, 0.7855075597763062]
robot_vis.GetView().SetCameraPose(v1, v2, v3)

manipulator = robot_model.GetActiveManipulator()
controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()
h = robot_vis.PlotFrame(controller.GetPose()[1], .2, 2)
print(controller.GetKinSolver().GetTCP())

print("\n\nRotate TCP before registrating EndEffector")
embed()

bp = BasicMotionUtilities(controller)
bp.RotateBX(np.pi/3)
bp.RotateBX(-np.pi/3)
robot_model.SetJointPositions(home)

robot_vis.GetView().Delete(h)

print("\n\nAuto registrate EndEffector\n")
embed()

manipulator.SetActiveEndEffector(gripper)
gripper.SetAttachingPose(Pose(0, 0, 0, 0, 1, 0, 1))
controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()
h = robot_vis.PlotFrame(controller.GetPose()[1], .2, 2)
print(controller.GetKinSolver().GetTCP())

print("\n\nRotate TCP after registrating EndEffector")
embed()

bp = BasicMotionUtilities(controller)
bp.RotateBX(np.pi/3)
bp.RotateBX(-np.pi/3)
robot_model.SetJointPositions(home)
robot_vis.GetView().Delete(h)

print("\n\nSet a new kin solver for controller\n")
embed()

kin_solver = CreateKinSolver(manipulator)
controller.SetKinSolver(kin_solver)
h = robot_vis.PlotFrame(controller.GetPose()[1], .2, 2)
print(controller.GetKinSolver().GetTCP())

print("\n\nRotate TCP after Setting a new kinsolver")
embed()

bp.RotateBX(np.pi/3)
bp.RotateBX(-np.pi/3)
robot_model.SetJointPositions(home)


print("\n\nUse planner\n")
embed()

use_roadmap_planner = False

verbose = False

while True:
    box.SetBaseTransformation(Pose(R3([0.7, 0, 0.03]), Rotation()))

    planner = CreateFreespacePlanner(env, manipulator, use_roadmap_planner)
    request = planner.CreateRequest(home, start)
    res, response = planner.Solve(request, verbose)
    waypoints = response.joint_trajectory

    if not res:
        print("planner return fail")
        embed()

    robot_model.SetJointPositions(waypoints[0])
    path = CreatePath(waypoints, 100, PathType_Bezier2ndBlend)
    traj = CreateTrajectory(path, max_vel, max_acc, max_jerk,
                            traj_type=TrajType_Trapezoidal)

    controller.ExecuteTrajectory(traj, True)

    print("\n\nUse EndEffector grab\n")
    embed()
    gripper.Grab(box)

    planner = CreateFreespacePlanner(env, manipulator, use_roadmap_planner)
    request = planner.CreateRequest(start, goal)
    res, response = planner.Solve(request, verbose)
    waypoints = response.joint_trajectory

    if not res:
        print("planner return fail")
        embed()

    robot_model.SetJointPositions(waypoints[0])
    path = CreatePath(waypoints, 100, PathType_Bezier2ndBlend)
    traj = CreateTrajectory(path, max_vel, max_acc,
                            traj_type=TrajType_Trapezoidal)

    controller.ExecuteTrajectory(traj, True)

    print("\n\nUse EndEffector release\n")
    embed()
    gripper.Release(box)

    # collision_pairs = [[robot_model, container2]]
    # planner.SetCollisoinPairs(collision_pairs)
    # res, waypoints, length = planner.PlanWithSimpleSetup(goal, home)

    planner = CreateFreespacePlanner(env, manipulator, use_roadmap_planner)
    request = planner.CreateRequest(goal, home)
    res, response = planner.Solve(request, verbose)
    waypoints = response.joint_trajectory
    if not res:
        print("planner return fail")
        embed()

    robot_model.SetJointPositions(waypoints[0])
    path = CreatePath(waypoints, 100, PathType_Bezier2ndBlend)
    traj = CreateTrajectory(path, max_vel, max_acc,
                            traj_type=TrajType_Trapezoidal)
    controller.ExecuteTrajectory(traj, True)
    embed()
