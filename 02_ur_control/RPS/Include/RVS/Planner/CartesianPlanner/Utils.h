#pragma once
#include <RVS/Common/Eigen.h>
#include <RVS/KinematicsHeader.h>

namespace RVS
{

namespace _INTERNAL
{

double Factorial(size_t n);

double NChooseK(size_t n, size_t k);

CVecXd Diff(const CVecXd &ts);


} // namespace _INTERNAL
bool LinearRefactoring(const std::vector<JointVector> &joint_positions,
                       const std::vector<double> &time_stamps,
                       std::shared_ptr<KinematicsBase> kin_solver,
                       std::vector<JointVector> &joint_positions_refactored,
                       std::vector<double> &time_stamps_refactored,
                       double max_joint_distance = 1e-4);

} // namespace RVS
