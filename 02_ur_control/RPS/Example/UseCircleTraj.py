'''!@example UseCircleTraj.py
@brief Generate a circle trajectory with many loops. 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from RVBUST.RPS import *
from RVBUST.RPS.ControllerJoggerUI import *
from IPython import embed


def GenerateCircleTraj(waypoints, speed: float = 0.5, loops: int = 1000, kin_solver=None, joint_seed=None):
    seg = PathSegmentCircleCircumscribedPose(
        waypoints[:3], continuous_cartesian_tangent=True, is_full_circle=True)
    path1 = CreatePath(seg)
    traj1 = CreateTrajectory(path1, speed, vel_end=speed)
    path2 = path1.Copy()
    traj2 = CreateTrajectory(path2, speed, vel_init=speed, vel_end=speed)
    path3 = path1.Copy()
    traj3 = CreateTrajectory(path3, speed, vel_init=speed)
    traj = traj1
    for _ in range(loops):
        traj.AppendTrajectory(traj2)
    traj.AppendTrajectory(traj3)
    traj.Reset()
    return traj


if __name__ == "__main__":
    robot = RobotModel()
    robot.InitFromRVDF(
        "{}/Rvbust/Data/Multibody/RobotModels/Motoman/GP12/GP12.rvdf".format(os.getenv('HOME')))
    r = SimController.Create(robot.GetActiveManipulator())
    r.Connect()
    r.EnableRobot()
    kin_solver = r.GetKinSolver()
    jogger = ControllerJoggerUI(r, limits_scale=1.0,
                                home=[0, 0, 0, 0, -1.57, 0])
    # 示教三个点
    jogger.Run()
    waypoints = [
        Pose([0.840169, 0.153054, 0.540732, 1, -
              1.06642e-11, 0.000398, 2.67949e-08]),
        Pose([0.840169, -0.244478, 0.540732, 1, -
              1.06644e-11, 0.000398, 2.67949e-08]),
        Pose([0.528006, 0.00875101, 0.540732, 1, -
              1.06644e-11, 0.000398, 2.67949e-08])
    ]
    if len(jogger.m_record_poses) >= 3:
        waypoints = [Pose(p.Coeffs()) for p in jogger.m_record_poses[:3]]
    traj = GenerateCircleTraj(waypoints, loops=10)
    DrawTraj(traj, show_in_vis=True, view=jogger.m_vis.GetView())
    bp = BasicMotionPlanner(r)
    bp.MoveLinear(waypoints)
    r.ExecuteTrajectory(traj, False)
    embed()
