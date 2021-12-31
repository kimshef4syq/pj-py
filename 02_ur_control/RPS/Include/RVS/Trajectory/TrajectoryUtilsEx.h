// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/TrajectoryWithPath.h>
#include <RVS/Kinematics/KinematicsBase.h>

namespace RVS
{

/// @addtogroup Trajectory
/// @{

/**
 * @brief Create a Path
 *
 * @tparam LieGroup space type
 * @param waypoints path waypionts, each waypoint will be converted to Rn space
 * waypoint
 * @param blend_tolerance path blend tolerance at corner
 * @param path_type path type
 * @param use_preprocess whether to filter out some repeated points
 * @return PathBase<LieGroup>::Ptr generated path
 */
template <typename LieGroup>
typename PathBase<LieGroup>::Ptr
CreatePath(const std::list<CVecXd> &waypoints, double blend_tolerance = 0.1,
           PathType path_type = PathType_Bezier5thBlend,
           bool use_preprocess = true);

/**
 * @brief Create a Path
 *
 * @tparam LieGroup space type
 * @param waypoints path waypionts
 * @param blend_tolerance path blend tolerance at corner
 * @param path_type path type
 * @param use_preprocess whether to filter out some repeated points
 * @return PathBase<LieGroup>::Ptr generated path
 */
template <typename LieGroup>
typename PathBase<LieGroup>::Ptr
CreatePath(const std::list<LieGroup> &waypoints, double blend_tolerance = 0.1,
           PathType path_type = PathType_Bezier5thBlend,
           bool use_preprocess = true);

/**
 * @brief Create a Path only contains the given segment
 *
 * @tparam LieGroup
 * @param seg path segment
 * @return PathBase<LieGroup>::Ptr generated path
 */
template <typename LieGroup>
typename PathBase<LieGroup>::Ptr
CreatePath(typename PathSegmentBase<LieGroup>::Ptr seg);

/**
 * @brief Create a time optimal or desired duration trajectory
 *
 * @tparam LieGroup space type
 * @param path trajectory path
 * @param limits_profile kinematics limits of velocity, accelerationi, jerk
 * @param traj_type trajectory type
 * @param vel_init initial velocity
 * @param vel_end final velocity
 * @param min_duration desired duration, if 0, the optimal duration is
 * caluculated
 * @param step for some numerical compution method, this parameter defines the
 * so-called integration step.
 * @return TrajectoryWithPath<LieGroup>::Ptr
 */
template <typename LieGroup>
typename TrajectoryWithPath<LieGroup>::Ptr
CreateTrajectory(typename PathBase<LieGroup>::Ptr path,
                 TrajProfileConstraintsBaseConstPtr limits_profile,
                 TrajType traj_type = TrajType_DoubleS, double vel_init = 0,
                 double vel_end = 0, double min_duration = 0,
                 double step = 0.01);

/**
 * @brief Create a constant speed trajectory
 *
 * @tparam LieGroup space type
 * @param path trajectory path
 * @param speed desired trajectory speed, m/s for carteisan space
 * @param traj_type trajectory type
 * @param vel_init initial velocity
 * @param vel_end final velocity
 * @param min_duration desired duration, if 0, the optimal duration is
 * caluculated
 * @param step for some numerical compution method, this parameter defines the
 * so-called integration step.
 * @return TrajectoryWithPath<LieGroup>::Ptr
 */
template <typename LieGroup>
typename TrajectoryWithPath<LieGroup>::Ptr
CreateTrajectory(typename PathBase<LieGroup>::Ptr path, double speed = 0.1,
                 double vel_init = 0, double vel_end = 0, double acc_max = 5.0,
                 double jerk_max = 20.0);

/**
 * @brief Create a time optimal or desired duration trajectory
 *
 * @tparam LieGroup space type
 * @param path trajectory path
 * @param joint_limits kinematics limits of velocity, accelerationi, jerk of
 * each joint
 * @param traj_type trajectory type
 * @param vel_init initial velocity
 * @param vel_end final velocity
 * @param min_duration desired duration, if 0, the optimal duration is
 * caluculated
 * @param step for some numerical compution method, this parameter defines the
 * so-called integration step.
 * @return TrajectoryWithPath<LieGroup>::Ptr
 */
template <typename LieGroup>
typename TrajectoryWithPath<LieGroup>::Ptr
CreateTrajectory(typename PathBase<LieGroup>::Ptr path,
                 const std::vector<JointLimit> &joint_limits,
                 TrajType traj_type = TrajType_DoubleS, double vel_init = 0,
                 double vel_end = 0, double min_duration = 0,
                 double step = 0.01);

/**
 * @brief Create a time optimal or desired duration trajectory
 *
 * @tparam LieGroup space type
 * @param path trajectory path
 * @param vel_limits velocity limits
 * @param acc_limits acceleration limits
 * @param jerk_limits jerk limits, needed if traj type is TrajType_DoubleS or
 * TrajType_Totp3
 * @param traj_type trajectory type
 * @param vel_init initial velocity
 * @param vel_end final velocity
 * @param min_duration desired duration, if 0, the optimal duration is
 * caluculated
 * @param step for some numerical compution method, this parameter defines the
 * so-called integration step.
 * @return TrajectoryWithPath<LieGroup>::Ptr
 */
template <typename LieGroup>
typename TrajectoryWithPath<LieGroup>::Ptr CreateTrajectory(
    typename PathBase<LieGroup>::Ptr path, const CVecXd &vel_limits,
    const CVecXd &acc_limits, const CVecXd &jerk_limits = CVecXd::Ones(0),
    TrajType traj_type = TrajType_DoubleS, double vel_init = 0,
    double vel_end = 0, double min_duration = 0, double step = 0.01);

/**
 * @brief Create a time optimal or desired duration trajectory
 *
 * @tparam LieGroup space type
 * @param path trajectory path
 * @param limits vel/acc/jerk limits
 * @param traj_type trajectory type
 * @param vel_init initial velocity
 * @param vel_end final velocity
 * @param min_duration desired duration, if 0, the optimal duration is
 * caluculated
 * @param step for some numerical compution method, this parameter defines the
 * so-called integration step.
 * @return TrajectoryWithPath<LieGroup>::Ptr
 */
template <typename LieGroup>
typename TrajectoryWithPath<LieGroup>::Ptr
CreateTrajectory(typename PathBase<LieGroup>::Ptr path,
                 const Mat<double, 3, Eigen::Dynamic> &limits,
                 TrajType traj_type = TrajType_DoubleS, double vel_init = 0,
                 double vel_end = 0, double min_duration = 0,
                 double step = 0.01);

/**
 * @brief Get a series joint space points according to a cartesian path
 *
 * @param path : cartesian path
 * @param kin_solver : kinematics solver
 * @param joint_seed : kinematics solver joint seed at trajectory start
 * @param tolerance_tangent : tolerance of each freedom
 * @param step : control the points density
 * @return std::tuple<RVSReturn, std::vector<Rxd>> : [RVSReturn,
 * joint_series]
 */
std::pair<RVSReturn, std::vector<Rxd>> CartPathToJointSpacePoints(
    std::shared_ptr<PathBase<Pose>> path,
    std::shared_ptr<KinematicsBase> kin_solver, Rxd joint_seed = Rxd(0),
    const SE3d::Tangent &tolerance_tangent = SE3Tangentd(
        0.001, 0.001, 0.001, DegToRad(1.0), DegToRad(1.0), DegToRad(1.0)),
    double step = 0.01);

/**
 * @brief Get a series joint space points according to a cartesian trajectory
 *
 * @param traj : cartesian trajectory
 * @param kin_solver : kinematics solver
 * @param joint_seed : kinematics solver joint seed at trajectory start
 * @param tolerance_tangent : tolerance of each freedom
 * @param step : control the points density
 * @return std::tuple<RVSReturn, std::vector<double>, std::vector<RxTangentd>,
 * std::vector<Rxd>> : [RVSReturn, time_series, joint_velocity_series,
 * joint_series]
 */
std::tuple<RVSReturn, std::vector<double>, std::vector<RxTangentd>,
           std::vector<Rxd>>
CartTrajToJointSpacePoints(std::shared_ptr<TrajectoryBase<Pose>> traj,
                           std::shared_ptr<KinematicsBase> kin_solver,
                           Rxd joint_seed = Rxd(0),
                           const SE3d::Tangent &tolerance_tangent =
                               SE3Tangentd(0.001, 0.001, 0.001, DegToRad(1.0),
                                           DegToRad(1.0), DegToRad(1.0)),
                           double step = 0.01);

/**
 * @brief Cartesian trajectory to joint trajectory
 *
 * @param traj
 * @param kin_solver
 * @param joint_seed
 * @param tolerance_tangent
 * @param time_opt if true, return TrajectoryRnICSP time opt trajectory; else
 * return trajectory with same duration of input cartesian trajectory, usually
 * for constant cartesian speed trajectory
 * @param limits_profile joint limits profile
 * @return std::shared_ptr<TrajectoryBase<Rxd>>
 */
std::shared_ptr<TrajectoryBase<Rxd>> CartTrajToJointTraj(
    std::shared_ptr<TrajectoryBase<Pose>> traj,
    std::shared_ptr<KinematicsBase> kin_solver, const Rxd &joint_seed = Rxd(0),
    const SE3d::Tangent &tolerance_tangent = SE3Tangentd(
        0.001, 0.001, 0.001, DegToRad(1.0), DegToRad(1.0), DegToRad(1.0)),
    bool time_opt = true,
    TrajProfileConstraintsBaseConstPtr limits_profile = nullptr);

/// @}
} // namespace RVS