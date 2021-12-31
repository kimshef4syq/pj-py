// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/TrajectoryBase.h>
#include <RVS/Trajectory/Utils.h>
#include <RVS/Kinematics/KinematicsBase.h>

namespace RVS
{
///@addtogroup Trajectory
///@{

/**
 * @brief Compute robot manipulator end cartesian velocity and acceleration
 * limits on a trajectory point accroding to robot joint limits
 *
 * @tparam LieGroup
 * @param kin_solver kinematics solver
 * @param traj trajectory
 * @param t trajectory time instant
 * @param joint_seed IK joint seed
 * @param cart_vel_limits[out] cartesian velocity limits
 * @param cart_acc_limits[out] cartesian acceleration limits, not computed
 * currently
 * @param ik_dist_threshold distance threshold between IK result and joint seed
 * @return RVSReturn IKFailed   : IK not found or joint distance is too large
 *                   Sigularity : Trajectory point is singular
 *                   Failed     : other reason causes failure
 *                   Success    : OK
 */
template <typename LieGroup>
RVSReturn ComputeCartesianLimitsFromJointLimits(
    std::shared_ptr<KinematicsBase> kin_solver,
    const TrajectoryBase<LieGroup> &traj, const double t, Rxd &joint_seed,
    CVecXd &cart_vel_limits, CVecXd &cart_acc_limits,
    double ik_dist_threshold = 0.1)
{
    // if |joint_vel| / |cart_vel| >= singular_threshold, robot is near
    // singularity
    const double singular_threshold = 50.0;

    SE3d current_pose = SE3d(traj.GetPosition(t).Coeffs());
    IKReport ik_rst;
    double min_dist = 0.0;
    Rxd current_joints;
    kin_solver->GetNearestIK(current_pose, joint_seed, ik_rst, min_dist);
    if (ik_rst.success && min_dist < ik_dist_threshold) {
        current_joints = ik_rst[0];
    }
    else {
        RVS_WARN("No suitable IK or min_dist({}) is too large: pose [{}], "
                 "joint_seed: [{}], ik success: {}",
                 min_dist, current_pose, joint_seed, ik_rst.success);
        if (ik_rst.success) {
            RVS_ERROR("Reference joint result: {}", ik_rst[0]);
        }
        return RVSReturn_IKFailed;
    }

    MatXd jacob;
    if (kin_solver->GetGeomJacobianWrtSpace(current_joints, jacob)
        != RVSReturn_Success) {
        RVS_WARN("Failed to calculate base jacobian at joints: [{}]",
                 current_joints);
        return RVSReturn_Failed;
    }

    CVecXd cart_vels = traj.GetVelocity(t).Coeffs();
    CVecXd joint_vels = jacob.colPivHouseholderQr().solve(cart_vels);
    if (joint_vels.norm() > singular_threshold * cart_vels.norm()) {
        RVS_WARN("Near singularity: |joint_vels| ({}) >= {} * |end_vels| "
                 "({})\npose: [{}]\njoint: [{}]",
                 joint_vels.norm(), singular_threshold, cart_vels.norm(),
                 current_pose, current_joints);
        return RVSReturn_Singularity;
    }

    const std::vector<JointLimit> &jls = kin_solver->GetJointLimits();
    CVecXd max_joint_vels, max_joint_accs, max_joint_jerks;
    ConvertJointLimitsToCVec(jls, max_joint_vels, max_joint_accs,
                             max_joint_jerks);

    double min_scale = 100;
    for (int i = 0; i < joint_vels.size(); i++) {
        if (std::abs(joint_vels[i]) > Constants<double>::Epsilon()) {
            double scale = std::abs(max_joint_vels[i] / joint_vels[i]);
            if (scale < min_scale) min_scale = scale;
        }
    }
    joint_seed = current_joints;
    cart_vels *= min_scale;

    ///@todo: compute cartesian acceleration limits accoring to joint limits
    // GeometryHessianSpace(); // interface is quite troublesome
    // CVecXd cart_accs = traj.GetAcceleration(t).Coeffs();
    cart_vel_limits = cart_vels;
    cart_acc_limits = CVec6d::Ones() * 1000.0;
    return RVSReturn_Success;
}

/**
 * @brief Given vec1 and vec2, find a scale coefficient k, let k * |vec1[i]| <=
 * |vec2[i]|, and at least 1 equation was satisfied
 *
 * @param vec1
 * @param vec2
 * @return double k, max scale coefficient
 */
inline double ComputeMaxScale(const CVecXd &vec1, const CVecXd &vec2)
{
    double scale = std::numeric_limits<double>::max();
    unsigned n = vec1.size();
    for (unsigned i = 0; i < n; ++i) {
        if (std::abs(vec1[i]) < Constants<double>::Epsilon()) continue;
        scale = std::min<double>(scale, std::abs(vec2[i]) / std::abs(vec1[i]));
    }
    return scale;
}

/**
 * @brief Check whether trajectory is reachable based on joint velocity limits,
 * acceleration limits, and for cartesian trajectory, robot inverse kinematics,
 * robot sigularity will also be considered, and only joint velocity limits is
 * considered.
 *
 * @tparam LieGroup Rxd, SE3d, Pose, realiability of other types are
 * guranteed
 * @param traj trajectory that need to be checked
 * @param kin_solver kinematics solver
 * @param joint_seed joint value seeds, needed when cartesian space
 * @param step sample step along trajectory, the unit is second
 * @param ik_dist_threshold distance threshold between IK result and joint seed
 * @return std::pair<RVSReturn, double> success or not; the max scale
 * coefficient k that should be appled to trajectory duration to make trajectory
 * within bounds
 */
template <typename LieGroup>
std::pair<RVSReturn, double> CheckTrajectoryReachability(
    const TrajectoryBase<LieGroup> &traj,
    std::shared_ptr<KinematicsBase> kin_solver,
    const Rxd &joint_seed = Rxd::IdentityStatic(0), const double step = 0.01,
    [[maybe_unused]] const double ik_dist_threshold = 0.1)
{
    CVecXd vel_limits, acc_limits, jerk_limits;
    const std::vector<JointLimit> &jls = kin_solver->GetJointLimits();
    ConvertJointLimitsToCVec(jls, vel_limits, acc_limits, jerk_limits);
    const double duration = traj.GetDuration();
    double t = 0;
    constexpr bool is_cart_traj =
        std::is_same_v<LieGroup, Pose> || std::is_same_v<LieGroup, SE3d>;
    Rxd joint_seed_(joint_seed);
    if (joint_seed_.DoF() == 0) {
        joint_seed_.Resize(kin_solver->GetDoF());
    }
    bool first_point = true;
    while (true) {
        CVecXd vel = traj.GetVelocity(t).Coeffs();
        CVecXd acc = traj.GetAcceleration(t).Coeffs();
        if constexpr (is_cart_traj) {
            // calculate cartesian velocity and acceleration limits from joint
            // limits, don't check joint threshold at trajectory beginning
            double dist_threshold = first_point ? 100 : ik_dist_threshold;
            first_point = false;
            if (auto ret = ComputeCartesianLimitsFromJointLimits(
                    kin_solver, traj, t, joint_seed_, vel_limits, acc_limits,
                    dist_threshold);
                ret != RVSReturn_Success) {
                return {ret, -1.0};
            }
        }

        double vel_scale = ComputeMaxScale(vel, vel_limits);
        double acc_scale = 10.0;
        if constexpr (!is_cart_traj) {
            acc_scale = ComputeMaxScale(acc, acc_limits);
        }
        if (vel_scale < 1 - Constants<double>::Small()
            || acc_scale < 1 - Constants<double>::Small()) {
            RVS_WARN("velocity limits or acceleration limits not satisfied: "
                     "vel_scale ({}) < 1 or acc_sacle({}) < 1",
                     vel_scale, acc_scale);
            return {RVSReturn_Failed,
                    std::min<double>(vel_scale, std::sqrt(acc_scale))};
        }
        t += step;
        if (t >= duration) break;
    }

    return {RVSReturn_Success, 1.0};
}
///@}
} // namespace RVS
