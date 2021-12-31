'''!@example ShoeGluing.py
@brief A FANUC shoe gluing demo.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import threading

from IPython import embed
from RVBUST.RPS import *

t_base_cam = Pose([0.547581, 0.105671, 0.986523,
                   0.702887, -0.708418, 0.0513215, 0.0382106])

# t_end_tool = Pose([-0.033, -0.01, 0.195, 0, -0.24740396, 0, 0.96891242])
t_end_tool = Pose([-0.0113377, 0.0299364, 0.201603, -
                   0.207912, 0, 0, 0.978148])   # glue gun

home_pos = Rx([0, 0, 0, 0, -np.pi/2, 0])
# home_pos = Rx([np.pi/4, 0, 0, 0, -1.047198, - np.pi/2])

MOVE_JOINT = False


def FanucController(rvis, sim=False):

    # controller = FanucLRMate200iDController.Create()
    controller = []
    rm = RobotModel()
    rm.InitFromRVDF(
        GetDataPath() + "Multibody/RobotModels/FANUC/LR_Mate_200iD/LR_Mate_200iD.rvdf")
    manipulator = rm.GetActiveManipulator()
    if sim == False:
        from RVBUST.FANUC.PyFANUC import FanucLRMate200iDController
        controller = FanucLRMate200iDController.Create(manipulator)
        controller.Connect("192.168.10.110")
        controller.SetSpeedRatio(1)
    else:
        controller = SimController.Create(manipulator)
        controller.Connect()
        controller.SetSpeedRatio(1)
    res = controller.EnableRobot()
    if res != RVSReturn_Success:
        print("connect failed.")
        return None

    # add endeffector
    gripper = EndEffector()
    # gripper.InitFromCylinder(0.002, t_end_tool.Z())
    gripper.InitFromRVDF(
        GetDataPath() + "Multibody/EndEffectors/FANUCGlueGun/FANUCGlueGun.rvdf")
    manipulator.SetActiveEndEffector(gripper)
    # gripper.SetAttachingPose (Pose(0, 0, 0, 0, 0.707107, 0, 0.707107))
    gripper.SetAttachingPose(Pose(0, 0, 0, 0, 0, 0, 1))

    controller.SetTCP(t_end_tool)

    if rvis != None:
        rvis.AddBody(controller.GetRobotModel())
        # rvis.ChooseShowMode(controller.GetRobotModel(), 5)
    embed()
    return controller


def GoHome(controller):
    kin_solver = controller.GetKinSolver()
    joint_limits = ConvertJointLimits2Arr(kin_solver.GetJointLimits())

    res, current_joint = controller.GetJointPosition()
    if res != RVSReturn_Success:
        print("Get pose failed.")
        return False

    if not current_joint.IsApprox(home_pos, 1e-2):
        path0 = CreatePath([current_joint, home_pos],  blend_tolerance=0.1,
                           path_type=PathType_Bezier2ndBlend)
        traj0 = CreateTrajectory(
            path0, *joint_limits, traj_type=TrajType_Toppra)
        res = controller.ExecuteTrajectory(
            traj0)
        if res != RVSReturn_Success:
            print("Execute trajectory failed.")
            return False

    return True


if __name__ == "__main__":
    rvis = RobotVis()
    controller = FanucController(rvis, True)

    ws = np.loadtxt(
        os.getenv("HOME") + "/Rvbust/Sources/RVS/Modules/Python/Example/ShoeGluingPoints.txt", delimiter=" ")

    ws2 = []
    hs = []
    for w in ws:
        hs.append(rvis.PlotFrame(Pose(w), .05, 1))
        ws2.append(Pose(w))

    path_filter = CreatePath(ws2, blend_tolerance=0.01)
    ws3 = []
    for s in np.linspace(0, path_filter.GetLength(), 100):
        ws3.append(Pose(path_filter.GetConfig(s).Coeffs()))

    path = CreatePath(ws2, blend_tolerance=0.1,
                      path_type=PathType_Bezier2ndBlendCartesian)

    traj = CreateTrajectory(path, speed=0.2)

    embed()
    controller.MoveLinear(Pose(ws[0]), True)
    controller.ExecuteTrajectory(traj)
    GoHome(controller)

    kin_solver = controller.GetKinSolver(
    )

    res, ik_report, dist = kin_solver.GetNearestIK(ws3[0], home_pos)

    if(ik_report.success):
        start_joint = ik_report[0]

    action_set = np.array(
        [-0.2, -0.15, -0.1, -0.05, 0, 0.05, 0.1, 0.15, 0.2])/4

    planner = STRPlanner(rvis.m_env, controller.GetManipulator())
    planner.Initialize(ws3, start_joint, action_set, 1,
                       False, 40, InitStateType_Manipubility)

    if planner.Plan() == RVSReturn_Success:
        qlist = planner.Getqlist()

    if MOVE_JOINT:
        path = CreatePath(qlist)
        max_vel = np.array([3]*6)
        max_acc = np.array([15]*6)
        max_jerk = np.array([100]*6)
        traj = CreateTrajectory(path, max_vel, max_acc, max_jerk,
                                traj_type=TrajType_Toppra)

        controller.MoveJoints(qlist[0])
        controller.ExecuteTrajectory(traj)
        GoHome(controller)

    ws4 = []
    rvis.GetView().Delete(hs)
    for q in qlist:
        pose = kin_solver.GetPositionFK(q)[1]
        hs.append(rvis.PlotFrame(pose, .05, 1))
        ws4.append(pose)

    path = CreatePath(ws4, blend_tolerance=0.1,
                      path_type=PathType_Bezier2ndBlendCartesian)

    traj = CreateTrajectory(path, speed=0.1)

    # embed()

    res, ik_report, dist = kin_solver.GetNearestIK(
        Pose(ws4[0].Coeffs()), start_joint)
    controller.MoveJoints(ik_report[0])
    controller.ExecuteTrajectory(traj)
    GoHome(controller)

    embed()
