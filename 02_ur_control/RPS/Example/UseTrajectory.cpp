/**
 * @file UseTrajectory.cpp
 * @brief Demonstration about trajectory usage in c++.
 * @date 2021-08-19
 *
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/Kinematics/URsKinematics.h>
#include <RVS/Trajectory/TrajectoryUtilsEx.h>
#include <RVS/Trajectory/TrajectorySpline.h>
#define PI M_PI

using namespace RVS;
int main()
{
    RVS_SET_LEVEL(RVS::LoggerLevel_Debug);
    // Rxd Trajectory
    {
        int dof = 6;
        CVecXd coeffs(dof);
        std::list<Rxd> waypoints;
        auto kin_solver =
            std::make_shared<URsKinematics>(GetUR5KinematicsParameters());
        auto joint_limits = kin_solver->GetJointLimits();
        coeffs << 0, -PI / 2.0, PI * 0.45, -PI / 2, -PI / 2, 0;
        waypoints.push_back(Rxd(coeffs));
        coeffs << 0.5, -PI / 3, PI / 2, -0.4, 0, 0;
        waypoints.push_back(Rxd(coeffs));
        coeffs << -0.5, -PI / 4, PI / 4, -PI / 2, -PI / 2, 0;
        waypoints.push_back(Rxd(coeffs));
        auto path = CreatePath<Rxd>(waypoints, 0.1, PathType_Bezier5thBlend);
        auto traj = CreateTrajectory<Rxd>(path, joint_limits, TrajType_DoubleS);
    }
    // R3xSO3 Trajectory
    {
        std::list<Pose> waypoints;
        CVecXd coeffs;
        std::shared_ptr<TrajectoryBase<Pose>> traj;
        coeffs.resize(7);
        coeffs << 0.49378, 0.10915, 0.50904, 8.13019e-16, 0.649448, 9.51923e-16,
            0.760406;
        Pose home_pose(coeffs);
        waypoints.push_back(Pose(coeffs));
        coeffs.resize(6);
        coeffs << -0.2, -0.2, 0, 0, 0, 0;
        waypoints.push_back(R3xSO3Tangentd(coeffs) + home_pose);
        coeffs << 0.1, -0.2, 0, 0, 0, 0;
        waypoints.push_back(R3xSO3Tangentd(coeffs) + home_pose);
        coeffs << 0.1, 0, 0, 0, 0, 0;
        waypoints.push_back(R3xSO3Tangentd(coeffs) + home_pose);
        coeffs << 0.1, 0.2, 0, 0, 0, 0;
        waypoints.push_back(R3xSO3Tangentd(coeffs) + home_pose);
        coeffs << -0.2, 0.2, 0, 0, 0, 0;
        waypoints.push_back(R3xSO3Tangentd(coeffs) + home_pose);
        waypoints.push_back(home_pose);
        auto path = CreatePath<Pose>(waypoints, 0.1, PathType_Bezier5thBlend);
        traj = CreateTrajectory<Pose>(path, 0.5);
    }
    // SE3 Trajectory
    {
        std::list<SE3d> waypoints;
        CVecXd coeffs;
        std::shared_ptr<TrajectoryBase<SE3d>> traj;
        coeffs.resize(7);
        coeffs << 0.49378, 0.10915, 0.30904, 8.13019e-16, 0.649448, 9.51923e-16,
            0.760406;
        SE3d home_pose(coeffs);
        waypoints.push_back(SE3d(coeffs));
        coeffs.resize(6);
        coeffs << -0.2, -0.2, 0, 0, 0, 0;
        waypoints.push_back(SE3Tangentd(coeffs) + home_pose);
        coeffs << 0.1, -0.2, 0, 0, 0, 0;
        waypoints.push_back(SE3Tangentd(coeffs) + home_pose);
        coeffs << 0.1, 0, 0, 0, 0, 0;
        waypoints.push_back(SE3Tangentd(coeffs) + home_pose);
        coeffs << 0.1, 0.2, 0, 0, 0, 0;
        waypoints.push_back(SE3Tangentd(coeffs) + home_pose);
        coeffs << -0.2, 0.2, 0, 0, 0, 0;
        waypoints.push_back(SE3Tangentd(coeffs) + home_pose);
        waypoints.push_back(home_pose);
        auto path = CreatePath<SE3d>(waypoints, 0.1, PathType_Bezier5thBlend);
        traj = CreateTrajectory<SE3d>(path, 0.5);
    }
    {
        CVec2d dq_max, ddq_max, dddq_max;
        dq_max << 2., 2.;
        ddq_max << 6., 6.;
        dddq_max << 30., 30.;
        CVec2d zeros;
        zeros.setZero();
        auto traj = TrajectoryRnCubicSpline(
            std::vector<Rxd>{Rxd({0., 0.}), Rxd({1., 0.}), Rxd({1., 1.}),
                             Rxd({0., 1.}), Rxd({0., 0.})},
            dq_max, ddq_max, dddq_max, zeros, zeros, zeros, zeros);
    }
    // waiting to show vis window
    // printf("input q to exit:");
    // while (getchar() != 'q') printf("input q to exit:");
    return 0;
}
