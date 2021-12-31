from RVBUST.RPS import *
from IPython import embed

SPEED_RATIO = 1.0
JOINT_TRAJ_VELOCITY = 3.0
# TCP = Pose([0, 0.017, 0.1406, 0.5, -0.5, -0.5, 0.5])
TCP = Pose([0.0265285, -0.0676526, 0.154851, -
           0.0255559, -0.683285, -0.664119, 0.302347])
HOME_JOINT = Rx([0, 0, 0, 0, -np.pi / 2, 0])
# HOME_JOINT = Rx([0, 0, 0, 0, -np.pi / 2, 0])

TCP_VELOCITY = 0.14
TOOL_RVDF_NAME = GetDataPath() + "/Projects/ShoesGluing_FanQin/GluingGuns/GluingGuns.rvdf"

logger = Logger.GetConsoleLogger("logger")
logger.SetLevelForAll(Logger.LoggerLevel_Info)


def CreateCartesianTrajectory(env, manipulatory, waypoints):
    path = CreatePath(waypoints, 0.01, PathType_Bezier2ndBlendCartesian)
    return True, CreateTrajectory(path, TCP_VELOCITY)


def PlanCartesianTrajectory(env, manipulator, waypoints):

    dof_limits = manipulator.GetDoFLimits()
    for i in range(len(dof_limits)):
        dof_limits[i].SetJerkLimit(2*dof_limits[i].GetMaxJerk())

    manipulator.SetDoFLimits(dof_limits)
    # construct CartesianPlanRequest
    request = CartesianPlanRequest()
    # request.SetLinearRefactoring(True)

    # 1. set global task redundancy, the following bounds
    # specifying a `ROT_Z` with range [-np.pi/3, np.pi/3]
    # region_lb = SE3Tangent([0, 0, 0, 0, 0, -np.pi/3])
    # region_ub = SE3Tangent([0, 0, 0, 0, 0, np.pi/3])
    # request.SetGlobalTaskRedundancy(
    #     region_lb, region_ub, num_redundancy_samples=20)
    region_lb = SE3Tangent([0, 0, 0, 0, 0, -np.pi/3])
    region_ub = SE3Tangent([0, 0, 0, 0, 0, np.pi/3])
    request.SetGlobalTaskRedundancy(
        region_lb, region_ub, num_redundancy_samples=20)

    """
    # or if a small range of ROT-X is allowable:

    # region_lb = SE3Tangent([0, 0, 0, -np.pi/20, 0, -np.pi/3])
    # region_ub = SE3Tangent([0, 0, 0, np.pi/20, 0, np.pi/3])
    # request.SetGlobalTaskRedundancy(
    #     region_lb, region_ub, num_redundancy_samples=[0, 0, 0, 10, 0, 10])
    """

    # 2. Create request by waypoints and lienar velocity,
    request.Create(waypoints, linear_velocity=TCP_VELOCITY,
                   num_resamples=80, acc_time=0.3)

    """
    # or you can create request by a original cartesian trajectory:

    path = CreatePath(waypoints, 0.01, PathType_Bezier2ndBlendCartesian)
    cart_traj = CreateTrajectory(path, TCP_VELOCITY)
    request.Create(cart_traj, num_resamples=100)
    """

    # construct CartesianPlanner
    planner = CartesianPlanner(manipulator, env)
    # 1. Load a default configuration
    configuration = planner.GetDefaultConfiguration()

    """
    # 2. you could bounds joint position additionally (let 5-th joints always be negative)

    joint_lower = [-2.967, -1.1344, -1.2217, -2.3161, -2.3561, -6.2944]
    joint_upper = [2.967, 2.5307, 3.3161, 2.3161, 0, 6.2944]
    configuration.joint_limits_lower_bound = joint_lower
    configuration.joint_limits_upper_bound = joint_upper
    """
    configuration.check_collision = False
    configuration.bound_joint_velocity = True
    configuration.bound_joint_acceleration = True
    configuration.bound_joint_jerk = True
    # configuration.linear_refactoring_joint_distance = 0.01

    """
    # 4. trajectory options
    configuration.polynomial_order = 5
    configuration.minimize_order = 3  #< minimum jerk
    """
    configuration.polynomial_order = 5
    configuration.minimize_order = 2

    planner.Config(configuration)
    ret, response = planner.Solve(request, verbose=True)

    qlist = response.joint_positions
    # start joint position
    q_start = response.joint_position_start
    # end joint position
    q_end = response.joint_position_end
    # trajectory of type TrajectoryRnSpline
    traj = response.joint_trajectory

    if ret != True:
        logger.Error("Make Cartesian Trajectory Failed!")
        return False, traj, qlist

    # optimal discreted joint position

    return True, traj, qlist


env = Environment()
rvis = RobotVis()

rvis.LoadEnvironment(env)
robot_model = RobotModel()
robot_model.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/FANUC/LR_Mate_200iD/LR_Mate_200iD.rvdf")

env.AddBody(robot_model)
manipulator = robot_model.GetActiveManipulator()

eef = EndEffector()
eef.InitFromRVDF(TOOL_RVDF_NAME)
manipulator.SetActiveEndEffector(eef)
manipulator.SetTCP(TCP)

controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()
controller.MoveJoints(HOME_JOINT)
# wps_raw = np.loadtxt(
#     GetDataPath() + "Projects/ShoesGluing_FanQin/Waypoints/poses_r3xso3.txt")

wps_raw = np.loadtxt(
    GetDataPath() + "Projects/ShoesGluing_FanQin/Waypoints/wps_opt.txt")
hs = []
wps = []
for wp in wps_raw:
    new_wp = SE3Tangent([-6.078947368421052655e-01, 3.623076923076922751e-01,
                        1.625000000000000333e-01, 0, 0, 0]) + Pose(wp)
    wps.append(new_wp)
    hs.append(rvis.PlotFrame(new_wp, 0.04, 1))

# # before plan
# logger.Warn("\n\nexit to execute original trajectory\n\n")
# embed()
# res, traj = CreateCartesianTrajectory(env, manipulator, wps)

# controller.MoveLinear(traj.GetPosition(0))
# controller.ExecuteTrajectory(traj)
# logger.Info(
#     "\n\n before plan, the cartesian trajectory was stuck at local minimum, with joint limit excceed\n\n")

# logger.Info("\n\nexit to execute planned trajectory\n\n")

# after plan
success, traj, qlist = PlanCartesianTrajectory(env, manipulator, wps)

if success:
    PlotTraj(traj)
    controller.MoveJoints(HOME_JOINT)
    controller.MoveJoints(traj.GetPosition(0))
    controller.ExecuteTrajectory(traj)
    controller.MoveJoints(HOME_JOINT)

embed()

logger.Info(
    "\n\n after plan, the cartesian trajectory is converted to a feasible joint "
    "trajectory with task redundancy consideration, the total joint motion cost is optimized.\n\n")
