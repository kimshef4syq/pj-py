'''!@example UsePoseConstraint.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time
from IPython import embed
from RVBUST.RPS import *

# ========= Global definitios start =========

show_detail = True
num_runs = 10000
use_numerical_ik = False

# ========= Global definitios end =========

# Construct an empty environment and open a view
rvis = RobotVis()
rvis.GetView().SetCameraPose([1.1, -1.6, 1.5],
                             [0.71, -0.88, 1.0],
                             [-0.17, 0.50, 0.85])

# Construct a robot model and load into rvis
robot_model = RobotModel()

robot_model.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/ABB/IRB1200_5_90/IRB1200_5_90.rvdf")

# robot_model.InitFromRVDF(
#     GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5.rvdf")

# Add end effector
manipulator = robot_model.GetActiveManipulator()
manipulator.SetDoFPositions(
    Rx([0, 0.647059, 0.351975, 0, -1, -1.5708]))

eef = EndEffector()
eef.InitFromRVDF(
    GetDataPath() + "Multibody/EndEffectors/Grippers/TwoFingerGripper/TwoFingerGripper.rvdf")

manipulator.SetActiveEndEffector(eef)

if(robot_model.GetName() == "UniversalRobotsUR5"):
    eef.SetAttachingPose(Pose(0, 0, 0, 0, 1, 0, 1))
    manipulator.SetDoFPositions(
        Rx([-0.2, -1.22017, 1.20399, -6.267, 1.37, 1.57]))

rvis.AddBody(robot_model)

# Construct a kin_solver from manipulator

if use_numerical_ik:
    kin_solver = GenericKinematics(manipulator)
else:
    kin_solver = CreateKinSolver(manipulator)


target_pose = Pose([0.86, 0, 0.4, 0.5, 0.5, -0.5, -0.5])

rvis.PlotFrame(target_pose)

ceres = CeresIKOptimizer()

init_pose = manipulator.GetInitPose()
b_list = manipulator.GetBlist()
s_list = manipulator.GetSlist()

ik_constraint = PoseConstraint.IK5_ROTATE_X
# ik_constraint = PoseConstraint.IK5_ROTATE_Y
# ik_constraint = PoseConstraint.IK5_ROTATE_Z
# ik_constraint = PoseConstraint.IK6_POSE
# ik_constraint = PoseConstraint.IK3_TRANSLATION
# ik_constraint = PoseConstraint.IK6_POSE_LOOSE
# ik_constraint = PoseConstraint.IK6_POSE_LOOSE_POS
# ik_constraint = PoseConstraint.IK6_POSE_LOOSE_ROT
# ik_constraint = SE3Tangent(1e-5, 0.1, 1e-5, 1e-3, 1e-3, 1e-3)
# ik_constraint = SE3Tangent(0.01, 0.01, 0.01, 1e-3, 1e-3, 1e-3)


total_run = 0
success_run = 0
runtime = 0

limits = manipulator.GetDoFLimits()
q = manipulator.GetDoFPositions()

kin_solver.SetPoseConstraint(ik_constraint)
# kin_solver.SetConstraintSearchDiscretization(3/2*ik_constraint)

kin_solver.SetConstraintSampleMethod(ConstraintSampleMethod_Uniform)

ik_report = IKReport()


embed()
Logger.SetLevelForAll(6)

for i in range(num_runs):
    q_seed = kin_solver.GetRandomStateNearBy(
        manipulator.GetDoFPositions(), 0.001)
    ik_report.Clear()

    t0 = time.time()

    res, ik_report = kin_solver.GetPositionIK(target_pose)

    total_run += 1
    if(res == RVSReturn_Success):
        success_run += 1
        runtime += (time.time() - t0)

        if show_detail:
            print(time.time() - t0)
            # Display first solution
            manipulator.SetDoFPositions(ik_report[0])

            # Display all solutions
            # for q in ik_report.ik_vector:
            # manipulator.SetDoFPositions(q)
            # time.sleep(0.001)

print("success rate = {}".format(success_run/total_run))
print("average runtime = {}".format(runtime/total_run))
embed()
