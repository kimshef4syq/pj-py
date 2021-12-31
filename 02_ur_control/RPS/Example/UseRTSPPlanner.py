'''!@example UseRTSPPlanner.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from IPython import embed
from RVBUST.RPS import *

# load workspace that contains robot arm and its working environment
robot_vis = RobotVis()
robot_vis.m_env.LoadFromFile(
    GetDataPath() + "Scenes/UsedByPythonExample/RtspCase.glb")
robot_model = robot_vis.m_env.GetBodiesByName("ABB_IRB1200_5_90")[0]
manipulator = robot_model.GetActiveManipulator()
joint_limits = manipulator.GetDoFLimits()
max_vel_limits = [joint.GetMaxVelocity() for joint in joint_limits]
max_vel_limits = np.asarray(max_vel_limits)

# load example waypoints and convert them into desired format
temp_file_path = os.path.join(os.path.dirname(
    __file__), "RtspSampleWaypoints.txt")
temp_wp = np.loadtxt(temp_file_path, delimiter=",")
target_poses = [Pose(wp) for wp in temp_wp]
via_target = [CartesianViapoint(tp) for tp in target_poses]

# create GTSP problem
glkh_problem_des = GLKHProblemDescription()
# set the weight of component of each pose at task space
# create the solver GLKH parameter configuration
glkh_config = GLKHConfig(glkh_problem_des)
# glkh_config.m_trace_level = 1

# create graph search config at configuration (joint) space
# Herein, search algorithm A* is adopted (Maybe you can also utilize
# the Dijkstra's algorithm to search the shortest path at joint space)
gs_with_a_star = GraphSearchWithAStar()
# gs_with_a_star.UpdateHeuristic(HeuristicMethod_Duration)
# gs_with_a_star.UpdateDofLimits(joint_limits)


def metric_dist(lhs, rhs, weights):
    """
       customize a metric distance function (OPTIONAL)
    """
    angle_diff = (lhs-rhs).Coeffs()
    global max_vel_limits
    temp = np.abs(angle_diff/max_vel_limits)
    return max(temp)


# gs_with_a_star.SetMetricFunc(metric_dist)

# ----- Graph search with Dijkstra's algorithm -----
# gs_with_dijkstra = GraphSearchWithDijkstra()
# gs_with_dijkstra.SetSampleParams(manipulator,RotationAxis_Z, np.pi/8)


# create dummy (nonsense) motion plannner config assuming that
# there is collision-free between any two adjacent joint configurations
path_planner_factory = PathPlannerFactory()
pp_with_dummy = path_planner_factory.Create("PathPlannerDummy")

# create RTSP (Robotics Task Sequencing Problem) planner and its config
rtsp_config = RTSPPlanner.Config(glkh_config, gs_with_a_star, pp_with_dummy)
rtsp_planner = RTSPPlanner()
rtsp_planner.SetConfiguration(rtsp_config)
# default value [1, 1, 1, 0, 0, 0] ---> [p_x, p_y, p_z, alpha, beta, gamma]
rtsp_planner.SetPoseWeights([1, 1, 1, 0, 0, 0])
rtsp_planner.SetSampleParams(manipulator, RotationAxis_Z, np.pi/4)
rtsp_planner.SetReferStartTargetAtCS(Rx.IdentityStatic(6))
rtsp_planner.EnableConsistency(True)

# create input (request) and output (response) of rtsp planner
# and solve the corresponding rtsp problem
request = MotionPlannerRequest()
request.viapoints = via_target

embed()

res, response = rtsp_planner.Solve(request)

# visualize and simulate at RobotVis() platform
tour_at_task_space = rtsp_planner.GetPathAtTaskSpace()
shortest_length_traj = response.joint_trajectory
# create controller for the robot
robot_controller = SimController.Create(manipulator)
robot_controller.Connect()
robot_controller.EnableRobot()
robot_controller.MoveJoints(Rx([0.129, 1.008, -0.743, -0.215, 0.37, -2.901]))

# visualize the motion trajectory of robot arm
hs = []
for wp in tour_at_task_space:
    hs.append(robot_vis.PlotFrame(wp, axis_len=0.05))
path_dir = np.array([[]])
for i in range(len(tour_at_task_space)-1):
    curr_p = tour_at_task_space[i].Translation()
    next_p = tour_at_task_space[i+1].Translation()
    temp = np.append(curr_p, next_p)
    if path_dir.size == 0:
        path_dir = np.array([temp])
    else:
        path_dir = np.append(path_dir, [temp], axis=0)

lh = []
for pd in path_dir:
    lh.append(robot_vis.GetView().Line(pd, 3, colors=[1.0, 0.5, 0.5]))

# mark the start and final of goal
sfh = robot_vis.GetView().Point(
    tour_at_task_space[0].Translation(), 15, colors=[1, 0, 0])
time.sleep(1)
for jp in shortest_length_traj:
    robot_controller.MoveJoints(jp)
embed()
