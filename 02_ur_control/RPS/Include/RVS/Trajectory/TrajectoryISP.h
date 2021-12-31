// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "TrajectorySplineBase.h"

namespace RVS
{
///@addtogroup Trajectory
///@{
namespace __RVS_HIDE
{
///@addtogroup Trajectory
///@{

/**
 * @brief Optimize a spline by iteratively scale the knots vector according to
 * its derivatives' limits. It's much faster than NLP based method and has a
 * good result for CubicSpline, but for BSpline4th, it's not very good.
 *
 * @tparam SplineType
 */
template <typename SplineType>
class TrajectoryISP : public TrajectorySplineBase<SplineType>
{
public:
    using Ptr = std::shared_ptr<TrajectoryISP>;
    using ConstPtr = std::shared_ptr<const TrajectoryISP>;
    using LieGroup = typename SplineType::LieGroup;
    using Tangent = typename LieGroup::Tangent;
    using Curvature = typename LieGroup::Tangent;
    using Torsion = typename LieGroup::Tangent;

    /**
     * @brief Construct a new TrajectoryISP (iterative optimized spline), the
     * spline will accross all points. Accleration is continuous except two
     * ends, but you can set use_acc_bounds true to make ends's acceleration
     * were satisfied for Rn space.
     *
     * @param qs interpolating points that need to be accrossed
     * @param vel_limits velocity limits of each dof
     * @param acc_limits acceleration limits of each dof
     * @param jerk_limits jerk limits of each dof
     * @param v0 start 1st derivative, aka. velocity, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param v1 end 1st derivative, aka. velocity, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param a0 start 2nd derivative, aka. acceleration, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param a1 end 2nd derivative, aka. acceleration, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param use_acc_bounds whether satisfy a0 / a1 bounds at two ends, with
     * bounds, the duration may be longer.
     */
    TrajectoryISP(const MatXd &qs, const CVecXd &vel_limits,
                  const CVecXd &acc_limits, const CVecXd &jerk_limits,
                  const CVecXd &v0 = CVecXd::Zero(0),
                  const CVecXd &v1 = CVecXd::Zero(0),
                  const CVecXd &a0 = CVecXd::Zero(0),
                  const CVecXd &a1 = CVecXd::Zero(0),
                  bool use_acc_bounds = false);

    /**
     * @brief Construct a new TrajectoryISP (iterative optimized spline), the
     * spline will accross all points. Accleration is continuous except two
     * ends, but you can set use_acc_bounds true to make ends's acceleration
     * were satisfied for Rn space.
     *
     * @param qs interpolating points that need to be accrossed
     * @param vel_limits velocity limits of each dof
     * @param acc_limits acceleration limits of each dof
     * @param jerk_limits jerk limits of each dof
     * @param v0 start 1st derivative, aka. velocity, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param v1 end 1st derivative, aka. velocity, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param a0 start 2nd derivative, aka. acceleration, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param a1 end 2nd derivative, aka. acceleration, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param use_acc_bounds whether satisfy a0 / a1 bounds at two ends, with
     * bounds, the duration may be longer.
     */
    TrajectoryISP(const std::vector<LieGroup> &qs, const CVecXd &vel_limits,
                  const CVecXd &acc_limits, const CVecXd &jerk_limits,
                  const CVecXd &v0 = CVecXd::Zero(0),
                  const CVecXd &v1 = CVecXd::Zero(0),
                  const CVecXd &a0 = CVecXd::Zero(0),
                  const CVecXd &a1 = CVecXd::Zero(0),
                  bool use_acc_bounds = false)
        : TrajectoryISP(ConvertVecLieGroupToMatXd(qs), vel_limits, acc_limits,
                        jerk_limits, v0, v1, a0, a1, use_acc_bounds)
    {
    }

    /**
     * @brief Construct a new TrajectoryRnICSP, the spline will accross some
     * sampled points on given path->
     *
     * @param path Rxd path
     * @param vel_limits velocity limits of each dof
     * @param acc_limits acceleration limits of each dof
     * @param jerk_limits jerk limits of each dof
     * @param v0 start 1st derivative, aka. velocity, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param v1 end 1st derivative, aka. velocity, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param a0 start 2nd derivative, aka. acceleration, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param a1 end 2nd derivative, aka. acceleration, if empty, resized to
     * CVecXd::Zero(DoF)
     * @param use_acc_bounds whether satisfy a0 / a1 bounds at two ends, with
     * bounds, the duration may be longer.
     * @param sample_step sample points along path, the distance of neighbour
     * points is sample_step
     */
    TrajectoryISP(std::shared_ptr<const PathBase<LieGroup>> path,
                  const CVecXd &vel_limits, const CVecXd &acc_limits,
                  const CVecXd &jerk_limits, const CVecXd &v0 = CVecXd::Zero(0),
                  const CVecXd &v1 = CVecXd::Zero(0),
                  const CVecXd &a0 = CVecXd::Zero(0),
                  const CVecXd &a1 = CVecXd::Zero(0),
                  bool use_acc_bounds = false, double sample_step = 0.1)
    {
        double length = path->GetLength();
        sample_step = std::clamp(sample_step, length / 1000.0, length / 4.0);
        auto qs = path->SamplePoints(sample_step);
        new (this) TrajectoryISP(qs, vel_limits, acc_limits, jerk_limits, v0,
                                 v1, a0, a1, use_acc_bounds);
    }

    virtual ~TrajectoryISP() { RVS_TRACE("Destructing TrajectoryISP"); }

    /**
     * @brief Get the waypoints and corresponding timestamps
     *
     * @param waypoints[out] trajectory key waypoints
     * @param timestamps corresponding timestamps of waypoints
     * @return RVSReturn
     */
    virtual RVSReturn
    GetWaypointAndTimeStamps(std::vector<LieGroup> &waypoints,
                             std::vector<double> &timestamps) const override;

private:
    ///@brief Compute time duration scales of each polynomial segment according
    /// to velocity, accleartion, jerk limits.
    CVecXd _ComputeScales(typename SplineType::ConstPtr spline);

    const MatXd m_qs; ///< waypoints
    const CVecXd m_vel_limits; ///< velocity limits
    const CVecXd m_acc_limits; ///< acceleration limits
    const CVecXd m_jerk_limits; ///< jerk limits
    CVecXd m_v0; ///< initial velocity
    CVecXd m_v1; ///< final velocity
    CVecXd m_a0; ///< initial acceleration
    CVecXd m_a1; ///< final acceleration
    unsigned m_dof; ///< degree of freedom
    std::vector<double> m_ts; ///< timestamps of waypoints
};
///@}
} // namespace __RVS_HIDE

// Rn space Iterative Cubic Spline
using TrajectoryRnICSP = __RVS_HIDE::TrajectoryISP<CubicSplineRn>;
// R3xSO3 space Iterative Cubic Spline
using TrajectorySE3ICSP = __RVS_HIDE::TrajectoryISP<CubicSplineSE3>;

///@}
} // namespace RVS
