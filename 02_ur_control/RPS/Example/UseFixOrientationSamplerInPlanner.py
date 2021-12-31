'''!@example UseFixOrientationSamplerInPlanner.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from RVBUST.RPS import *

robot_model = RobotModel()
robot_model.InitFromRVDF(
    "/home/rvbust/Rvbust/Data/Multibody/RobotModels/ABB/IRB1200_7_70/IRB1200_7_70.rvdf")
manipulator = robot_model.GetActiveManipulator()
start = Rx([-0.564797, 1.61156, -0.787382, -
            2.89484e-06, 0.746618, -0.564795])
goal = Rx([0.549846, 1.59208, -0.742303, 2.9076e-06, 0.721022, 0.549844])
manipulator.SetDoFPositions(start)
max_vel, max_acc, max_jerk = ConvertJointLimits2Arr(manipulator.GetDoFLimits())
rvis = RobotVis()
rvis.AddBody(robot_model)
rvis.GetView().Home()

controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()
controller.SetSpeedRatio(0.1)

container1 = Multibody()
container1.InitFromContainerBox(0.3, 0.3, 0.2, 0.01, 0.01, 0.01)
container1.SetBaseTransformation(Pose([0.5, -0.3, 0.1, 0, 0, 0, 1]))
rvis.m_env.AddBody(container1)

container2 = Multibody()
container2.InitFromContainerBox(0.3, 0.3, 0.2, 0.01, 0.01, 0.01)
container2.SetBaseTransformation(Pose([0.5, 0.3, 0.1, 0, 0, 0, 1]))
rvis.m_env.AddBody(container2)

target_orientation = Rotation([0, 1, 0, 0])
# sampler
sampler = FixOrientationSampler(manipulator)
sampler.SetRefOrientation(target_orientation)

# constraint
constraint = StateOrientationConstraint()
constraint.SetParams(target_orientation, [0, 0, 1], 0.1)

# planner config
configuration = OmplParallelPlanMotionPlanner.Configuration(rvis.m_env, [
                                                            manipulator])
configuration.planning_time = 10
configuration.hybridize = False
configuration.smooth = False
configuration.simplify = True
configuration.planner_types = 4 * [OmplType_RRTConnect]
configuration.SetStateSampler(sampler)  # sampler
configuration.AddStateConstraint(constraint)  # constraint

# planner
planner = OmplParallelPlanMotionPlanner()
planner.SetConfiguration(configuration)

controller.MoveJoints(start)
verbose = True
rvis.ChooseShowMode(mode=5)

embed()
while True:

    request = planner.CreateRequest(start, goal)
    res, response = planner.Solve(request, verbose)
    path = CreatePath(response.joint_trajectory, 0.1,
                      PathType_Bezier2ndBlend)
    traj = CreateTrajectory(path, max_vel, max_acc, max_jerk,
                            traj_type=TrajType_Trapezoidal)

    controller.ExecuteTrajectory(traj)

    if res:
        start, goal = goal, start


embed()
