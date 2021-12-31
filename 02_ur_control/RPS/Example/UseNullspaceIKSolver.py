'''!@example UseNullspaceIKSolver.py
@brief Use null space IK solver, especially for robot with external axis.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import os
import time

from RVBUST.RPS import *

logger.SetLevelForAll(1)

home_dir = os.getenv('HOME')

robot_model = RobotModel()
robot_model.InitFromRVDF(
    os.path.join(
        home_dir,
        "Rvbust/Data/Multibody/RobotModels/Motoman/GP180B1/GP180B1.rvdf"))

env = Environment()
env.AddBody(robot_model)

rvis = RobotVis()
rvis.LoadEnvironment(env)

# desired Pose pose
pose = Pose(-0.486139, 4.66069, 1.532633, 0.707388, 0.706825, 0.000281559,
            -0.000281297)

h_pose = rvis.PlotFrame(pose)

robot_model = env.GetAllRobotModels()[0]
manipulator = robot_model.GetActiveManipulator()
controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()

q_current = controller.GetJointPosition()[1]
pose_current = controller.GetPose()[1]

kin_sovler = controller.GetKinSolver()
dof_weights = [10, 7, 6, 4.7, 4, 3.2, 3.0]
# dof_weights = [1, 1, 1, 1, 1, 1, 1]

# 1. construct NullspaceIKSolver
nik_solver = NullspaceIKSolver(env, manipulator)

# 2. set configurations
configuration = NullspaceIKSolver.Configuration()
configuration.dof_weights = dof_weights
configuration.velocity_limits = False
configuration.acceleration_limits = False
configuration.allow_task_scaling = False

nik_solver.Config(configuration)
embed()

# 3. add state costs
y_axis_in_base_cost = TCPAxisInBaseCost(
    ref_value=1.5, external_axis_idx=0, tcp_axis=TCPAxisInBaseCost.TCP_AXIS_Y
)  # < y axis value of tcp expressed in base
nik_solver.AddStateCost(y_axis_in_base_cost, weight=1)

# 4. get position IK
# or with non-default arguments
# res, ik_report = nik_solver.GetPositionIK(pose, q_seed=q_current, max_iter=1000)
res, ik_report = nik_solver.GetPositionIK(pose, q_current)
if res == RVSReturn_Success:
    q_sol = ik_report[0]
    controller.MoveJoints(q_sol)

embed()

# embed()
# 5. set reference value of cost function
y_axis_in_base_cost.SetReferenceValue(ref_value=0.8)

res, ik_report = nik_solver.GetPositionIK(pose)

if res == RVSReturn_Success:
    q_sol = ik_report[0]
    controller.MoveJoints(q_sol)

embed()

# 5. velocity IK
path = CreatePath([pose_current, pose])
traj = CreateTrajectory(path, speed=1.0)

manipulator.SetDoFPositions(q_current)
t = 0
dt = 0.004
q = q_current
while t < traj.GetDuration():
    x = Pose(traj.GetPosition(t).Coeffs())
    x_desired = Pose(traj.GetPosition(t + dt).Coeffs())
    x_dot = (x_desired - x) / dt

    res, q_dot, vel_scale = nik_solver.GetVelocityIK(x_dot, q)
    if not res:
        logger.Error("cannot find velocity ik")
        embed()

    t += dt
    q = q + q_dot * dt
    manipulator.SetDoFPositions(q)
    time.sleep(dt)

embed()
