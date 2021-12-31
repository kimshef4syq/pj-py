'''!@example UseCustomedPath.py
@brief This file constructs a customed traj whose shape follows sine wave in joint space. 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time

import matplotlib.pyplot as plt
import numpy as np
from IPython import embed
from RVBUST.RPS import *

Logger.SetLevelForAll(Logger.LoggerLevel_Info)


class CirclePath(object):
    '''
    Example: user-defined circle path
    '''

    def __init__(self, center=Pose([0, 0, 0, 0, 0, 0, 1]), start_p=Pose([0, 0, 0, 0, 0, 0, 1]), angle=0.1):
        self.m_center = center
        self.m_start_p = Pose(
            [*start_p.Coeffs()[:3], *self.m_center.Coeffs()[3:]])
        self.m_radius = (
            self.m_center - self.m_start_p).WeightedNorm(np.array([1, 1, 1, 0, 0, 0]))
        # m_x and m_y refers to PathBlendMethod. Both are unit 3d vector.
        self.m_x = ((self.m_center - self.m_start_p) /
                    self.m_radius).Coeffs()[:3]
        self.m_tnorm = -Pose(self.m_center.Coeffs()).Rotation()[:, 2]
        self.m_y = np.cross(self.m_x, self.m_tnorm)
        self.m_angle = angle
        self.m_length = self.m_radius * self.m_angle
        self.m_x_tan = SE3Tangent(
            [self.m_x[0], self.m_x[1], self.m_x[2], 0, 0, 0])
        self.m_y_tan = SE3Tangent(
            [self.m_y[0], self.m_y[1], self.m_y[2], 0, 0, 0])

    def GetLength(self):
        return self.m_length

    def GetConfig(self, s):
        s = max(0, min(s, self.m_length))
        s_norm = s / self.m_length
        angle = s_norm * self.m_length / self.m_radius
        ret = self.m_center + self.m_radius * \
            (np.cos(angle) * self.m_x_tan + np.sin(angle) * self.m_y_tan)
        return ret

    def GetTangent(self, s):
        s = max(0, min(s, self.m_length))
        s_norm = s / self.m_length
        angle = s_norm * self.m_length / self.m_radius
        ret = -np.sin(angle) * self.m_x_tan + np.cos(angle) * self.m_y_tan
        return ret

    def GetCurvature(self, s):
        s = max(0, min(s, self.m_length))
        s_norm = s / self.m_length
        angle = s_norm * self.m_length / self.m_radius
        ret = -1.0 / self.m_radius * \
            (np.cos(angle) * self.m_x_tan + np.sin(angle) * self.m_y_tan)
        return ret


class SinePath(object):
    '''
    Example: user-defined sine path shape
    '''

    def __init__(self, start_joints=Rx([0, 0, 0, 0, 0, 0]), end_joints=Rx([0, 0, 0, 0, 0, 0])):
        # Each joint follows:
        # theta = A / 2 * sin(pi / m_length * s - pi / 2) + A / 2 + start
        # Thus, joint range: [start, end)
        self.m_A = end_joints.Coeffs() - start_joints.Coeffs()
        self.m_length = np.linalg.norm(self.m_A)
        self.m_start_q = start_joints
        self.m_end_q = end_joints

    def GetLength(self):
        return self.m_length

    def GetConfig(self, s):
        s = max(0, min(s, self.m_length))
        ret = [self.m_A[i] / 2 * (math.sin(np.pi / self.m_length *
                                           s - np.pi / 2)+1) + self.m_start_q[i] for i in range(6)]
        return Rx(ret)

    def GetTangent(self, s):
        s = max(0, min(s, self.m_length))
        ret = [self.m_A[i] * np.pi / self.m_length / 2 *
               math.cos(np.pi / self.m_length * s - np.pi / 2) for i in range(6)]
        return RxTangent(ret)

    def GetCurvature(self, s):
        s = max(0, min(s, self.m_length))
        ret = [-self.m_A[i] * np.pi**2 / self.m_length**2 / 2 *
               math.sin(np.pi / self.m_length * s - np.pi / 2) for i in range(6)]
        return RxTangent(ret)


if __name__ == "__main__":
    rvis = RobotVis()
    rm = RobotModel()
    rm.InitFromRVDF(
        GetDataPath() + "Multibody/RobotModels/ABB/IRB1200_5_90/IRB1200_5_90.rvdf")
    manip = rm.GetActiveManipulator()
    controller = SimController.Create(manip)
    kin_solver = controller.GetKinSolver()
    limits = kin_solver.GetJointLimits()
    limits = ConvertJointLimits2Arr(limits)
    rvis.AddBody(rm)
    controller.Connect()
    controller.EnableRobot()
    controller.SetSpeedRatio(1)

    # Circle path example
    center = Pose([0.5, 0, 0.5, 0, 1, 0, 0])
    start = Pose([0.4, 0.1, 0.4, 0, 1, 0, 0])
    circle_path = CirclePath(center, start, 2 * np.pi)
    # construct a customed path segment from user defined functions in the above class
    circle_seg = PathSegmentCustomedSE3(circle_path.GetLength(
    ), circle_path.GetConfig, circle_path.GetTangent, circle_path.GetCurvature)
    path = CreatePath(circle_seg)
    # Toppra
    traj = CreateTrajectory(path, limits, traj_type=TrajType_Toppra)
    h = DrawTraj(traj, show_in_vis=True, show_in_plt=True,
                 view=rvis.GetView(), kin_solver=kin_solver)
    controller.MoveJoints(kin_solver.GetNearestIK(
        Pose(traj.GetPosition(0).Coeffs()), Rx([0, 0, 0, 0, 1.57, 0]))[1][0])
    controller.ExecuteTrajectory(traj)
    embed()
    rvis.GetView().Delete(h)
    # default traj type
    traj = CreateTrajectory(path, 0.5)
    h = DrawTraj(traj, show_in_vis=True, show_in_plt=True,
                 view=rvis.GetView(), kin_solver=kin_solver)
    controller.MoveJoints(kin_solver.GetNearestIK(
        Pose(traj.GetPosition(0)), Rx([0, 0, 0, 0, 1.57, 0]))[1][0])
    controller.ExecuteTrajectory(traj)
    embed()
    rvis.GetView().Delete(h)

    # Sine Path example
    p1 = Rx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    p2 = Rx([0.3, 0.6, 0.9, -0.3, -0.6, -0.9])
    sin_path = SinePath(p1, p2)
    # construct a customed path segment from user defined functions in the above class
    path_seg = PathSegmentCustomedRn(sin_path.GetLength(),
                                     sin_path.GetConfig, sin_path.GetTangent, sin_path.GetCurvature)
    path = CreatePath(path_seg)
    traj = CreateTrajectory(path, limits, traj_type=TrajType_Toppra)
    h = DrawTraj(traj, show_in_vis=True, show_in_plt=True,
                 view=rvis.GetView(), kin_solver=kin_solver)
    controller.MoveJoints(p1)
    controller.ExecuteTrajectory(traj)
    embed()
    rvis.GetView().Delete(h)

    # Test path appending to make sure this function works normally. Appending is not available for customed path now.
    path1 = CreatePath([p1, p2])
    path2 = CreatePath([p2, p1])
    path1.AppendPath(path2)
    traj = CreateTrajectory(path1, limits, traj_type=TrajType_Toppra)
    h = DrawTraj(traj, show_in_vis=True, show_in_plt=True,
                 view=rvis.GetView(), kin_solver=kin_solver)
    controller.MoveJoints(p1)
    controller.ExecuteTrajectory(traj)
    embed()
