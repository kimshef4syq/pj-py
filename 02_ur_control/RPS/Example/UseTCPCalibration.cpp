/**
 * @example UseTCPCalibration.cpp
 * @brief Demo of how to calibrate robot TCP.
 * @date 2021-08-19
 * 
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/Kinematics/TCPCalibration.h>
#include <iostream>

using namespace RVS;

int main()
{
    TCPCalibration tcp_calibration = TCPCalibration();
    SE3d tcp_pose;
    SE3d p0 = SE3d(0.745736, 0.018929, 0.203403, -0.0723118, 0.879697, -0.0938845, 0.460533);
    SE3d p1 = SE3d(0.773886, 0.0836232, 0.158874, -0.388254, 0.756258, -0.521459, 0.073566);
    SE3d p2 = SE3d(0.714161, 0.0287805, 0.106458, -0.494146, 0.346114, -0.588371, 0.538372);
    SE3d p3 = SE3d(0.768611, -0.075825, 0.129232, -0.72319, -0.260229, -0.161023, 0.619151);
    SE3d p4 = SE3d(0.778282, -0.0689607, 0.171271, 0.834091, 0.37506, 0.0786907, -0.396776);

    std::vector<SE3d> poses;
    poses.push_back(p0);
    poses.push_back(p1);
    poses.push_back(p2);
    poses.push_back(p3);
    poses.push_back(p4);

    tcp_calibration.LoadData(poses);
    tcp_calibration.Solve(tcp_pose);
    RVS_INFO("tcp_pose is: {}", tcp_pose.Coeffs().transpose());

    return 0;
}