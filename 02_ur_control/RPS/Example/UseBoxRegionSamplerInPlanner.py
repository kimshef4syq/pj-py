'''!@example UseBoxRegionSamplerInPlanner.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from RVBUST.RPS import *

robot_model = RobotModel()
robot_model.InitFromRVDF(
    "/home/rvbust/Rvbust/Data/Multibody/RobotModels/ABB/IRB1200_7_70/IRB1200_7_70.rvdf")
manipulator = robot_model.GetActiveManipulator()
start = Rx([0, 1.02791, 0.694672, 0, -0.151782, 0])
goal = Rx([0, 1.45237, -0.505944, 0, 0.624378, 0])
manipulator.SetDoFPositions(start)
max_vel, max_acc, max_jerk = ConvertJointLimits2Arr(manipulator.GetDoFLimits())
rvis = RobotVis()
rvis.AddBody(robot_model)
rvis.GetView().Home()

controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()

boxregion = Box(0.5, 0.6, 0.4)
boxregion.pose = Pose(0.55, 0, 0.2, 0, 0, 0, 1)
# bh = rvis.PlotGeometry(boxregion, color=[0, 0, 1, 0.5])

container = Multibody()
container.InitFromContainerBox(0.3, 0.6, 0.3, 0.01, 0.01, 0.01)
rvis.m_env.AddBody(container)
container.SetBaseTransformation(Pose(0.55, 0, 0.15, 0, 0, 0, 1))

# sampler
box_sampler = BoxRegionSampler(manipulator, boxregion)

# planner config
configuration = OmplParallelPlanMotionPlanner.Configuration(rvis.m_env, [
                                                            manipulator])
configuration.planning_time = 10
configuration.longest_valid_segment_length = 0.05
configuration.hybridize = False
configuration.smooth = False
configuration.simplify = True
configuration.planner_types = 1 * [OmplType_RRTConnect]
# Comment out to compare the original effect
configuration.SetStateSampler(box_sampler)

# planner
planner = OmplParallelPlanMotionPlanner()
planner.SetConfiguration(configuration)

embed()

controller.MoveJoints(start)
verbose = True

while True:

    request = planner.CreateRequest(start, goal)
    res, response = planner.Solve(request, verbose)

    path = CreatePath(response.joint_trajectory, 0.1,
                      PathType_Bezier2ndBlend)
    traj = CreateTrajectory(path, max_vel, max_acc, max_jerk,
                            traj_type=TrajType_Trapezoidal)

    controller.ExecuteTrajectory(traj)

    start, goal = goal, start
    embed()

embed()
