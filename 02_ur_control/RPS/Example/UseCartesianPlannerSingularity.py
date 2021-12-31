from RVBUST.RCI import *
from RVBUST.RPS import *

SPEED_RATIO = 1.0
JOINT_TRAJ_VELOCITY = 3.0
TCP = RCI.Pose()
HOME_JOINT = RCI.Rx([0, 0, 0, 0, -np.pi / 2, 0])
TCP_VELOCITY = 0.2

logger.SetLevelForAll(RCI.Logger.LoggerLevel_Info)


def CreateCartesianTrajectory(env, manipulatory, waypoints):
    path = CreatePath(waypoints, 0.01, PathType_Bezier2ndBlendCartesian)
    return True, CreateTrajectory(path, TCP_VELOCITY)


def PlanCartesianTrajectory(env, manipulator, waypoints):
    # construct CartesianPlanRequest
    request = CartesianPlanRequest()

    """
    # or if a small range of ROT-X is allowable:

    # region_lb = SE3Tangent([0, 0, 0, -np.pi/20, 0, -np.pi/3])
    # region_ub = SE3Tangent([0, 0, 0, np.pi/20, 0, np.pi/3])
    # request.SetGlobalTaskRedundancy(
    #     region_lb, region_ub, num_redundancy_samples=[0, 0, 0, 10, 0, 10])
    """

    # 2. Create request by waypoints and lienar velocity,
    request.Create(waypoints, linear_velocity=TCP_VELOCITY, num_resamples=10)

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
    # 3. check collsision
    configuration.check_collision = False
    configuration.polynomial_order = 7
    configuration.minimize_order = 4

    planner.Config(configuration)
    ret, response = planner.Solve(request)
    if ret != True:
        logger.Error("Make Cartesian Trajectory Failed!")
        embed()
        return False

    # optimal discreted joint position
    qlist = response.joint_positions
    # start joint position
    q_start = response.joint_position_start
    # end joint position
    q_end = response.joint_position_end
    # trajectory of type TrajectoryRnSpline
    traj = response.joint_trajectory

    # PlotTraj(traj, sample_num=100)

    return True, traj


env = RCI.Environment()
rvis = RCI.RobotVis()

rvis.LoadEnvironment(env)
robot_model = RCI.RobotModel()
robot_model.InitFromRVDF(
    RCI.GetDataPath() + "/Multibody/RobotModels/Motoman/GP7/GP7.rvdf")
env.AddBody(robot_model)
manipulator = robot_model.GetActiveManipulator()
manipulator.SetTCP(TCP)

controller = RCI.SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()
controller.MoveJoints(HOME_JOINT)

pose1 = RCI.Pose([0.56, 0.01, 0.9, 0.707107, -
                 1.89468e-08, 0.707107, 1.89469e-08])
pose2 = RCI.Pose([0.56, 0.01, 0.4, 0.707107, -
                 1.89468e-08, 0.707107, 1.89469e-08])
wps = [pose1, pose2]

# before plan
res, traj = CreateCartesianTrajectory(env, manipulator, wps)
controller.MoveLinear(traj.GetPosition(0))
controller.ExecuteTrajectory(traj)
logger.Info(
    "\n\n before plan, the cartesian trajectory pass through with joint velocity limit excceed\n\n")

logger.Info("\n\nexit to execute planned trajectory\n\n")
embed()
# after plan
res, traj = PlanCartesianTrajectory(env, manipulator, wps)

controller.MoveJoints(HOME_JOINT)
controller.MoveJoints(traj.GetPosition(0))
controller.ExecuteTrajectory(traj)
controller.MoveJoints(HOME_JOINT)

logger.Info(
    "\n\n after plan, the cartesian trajectory is converted to a feasible joint "
    "trajectory within joint velocity limit.\n\n")
