// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "TrajectorySpline.h"

namespace RVS
{
///@addtogroup Trajectory
///@{
namespace __RVS_HIDE
{
///@addtogroup Trajectory
///@{

/**
 * @brief Calculating a Spline trajectory, optimized for large batch waypoints
 * with a slide window method. It divides the large batch waypoints to small
 * batches and compute one by one. Use this if the number of waypoints are
 * large, eg. > 30.
 *
 * @tparam SplineType
 */
template <typename SplineType>
class TrajectorySplineEx : public TrajectorySplineBase<SplineType>
{
public:
    using Ptr = std::shared_ptr<TrajectorySplineEx>;
    using ConstPtr = std::shared_ptr<const TrajectorySplineEx>;
    using LieGroup = typename SplineType::LieGroup;
    using Tangent = typename LieGroup::Tangent;
    using Curvature = typename LieGroup::Tangent;
    using Torsion = typename LieGroup::Tangent;

    /**
     * @brief Construct a new TrajectorySpline, the spline will accross all
     * points. This method fit for large scale points, divide them into small
     * batches and merge the results into one spline.
     *
     * @param qs interpolating points that need to be accrossed
     * @param vel_limits velocity limits of each dof
     * @param acc_limits acceleration limits of each dof
     * @param jerk_limits jerk limits of each dof
     * @param v0 start 1st derivative, aka. velocity
     * @param v1 end 1st derivative, aka. velocity
     * @param a0 start 2nd derivative, aka. acceleration
     * @param a1 end 2nd derivative, aka. acceleration
     * @param window_size sliding window size, determine the number of points of
     * each batch, different window size makes a significant influence on the
     * success rate and trajectory duration at a specific condition.
     */
    TrajectorySplineEx(const MatXd &qs, const CVecXd &vel_limits,
                       const CVecXd &acc_limits, const CVecXd &jerk_limits,
                       const CVecXd &v0 = CVecXd::Zero(0),
                       const CVecXd &v1 = CVecXd::Zero(0),
                       const CVecXd &a0 = CVecXd::Zero(0),
                       const CVecXd &a1 = CVecXd::Zero(0),
                       unsigned window_size = 2);

    /**
     * @brief Construct a new TrajectorySpline, the spline will accross all
     * points. This method fit for large scale points, divide them into small
     * batches and merge the results into one spline.
     *
     * @param qs interpolating points that need to be accrossed
     * @param vel_limits velocity limits of each dof
     * @param acc_limits acceleration limits of each dof
     * @param jerk_limits jerk limits of each dof
     * @param v0 start 1st derivative, aka. velocity
     * @param v1 end 1st derivative, aka. velocity
     * @param a0 start 2nd derivative, aka. acceleration
     * @param a1 end 2nd derivative, aka. acceleration
     * @param window_size sliding window size, determine the number of points of
     * each batch, different window size makes a significant influence on the
     * success rate and trajectory duration at a specific condition.
     */
    TrajectorySplineEx(const std::vector<LieGroup> &qs,
                       const CVecXd &vel_limits, const CVecXd &acc_limits,
                       const CVecXd &jerk_limits,
                       const CVecXd &v0 = CVecXd::Zero(0),
                       const CVecXd &v1 = CVecXd::Zero(0),
                       const CVecXd &a0 = CVecXd::Zero(0),
                       const CVecXd &a1 = CVecXd::Zero(0),
                       [[maybe_unused]] unsigned window_size = 2)
        : TrajectorySplineEx(ConvertVecLieGroupToMatXd(qs), vel_limits,
                             acc_limits, jerk_limits, v0, v1, a0, a1,
                             window_size)
    {
    }
    /**
     * @brief Construct a new TrajectoryRnSpline, the spline will accross some
     * sampled points on given path->
     *
     * @param path Rxd path
     * @param vel_limits velocity limits of each dof
     * @param acc_limits acceleration limits of each dof
     * @param jerk_limits jerk limits of each dof
     * @param v0 start 1st derivative, aka. velocity
     * @param v1 end 1st derivative, aka. velocity
     * @param a0 start 2nd derivative, aka. acceleration
     * @param a1 end 2nd derivative, aka. acceleration
     * @param window sliding window size, determine the number of points of each
     * batch, different window size makes a significant influence on the success
     * rate and trajectory duration at a specific condition.
     * @param sample_step the distance of neighbour sampled points on the path,
     * @see Path
     */
    TrajectorySplineEx(std::shared_ptr<const PathBase<LieGroup>> path,
                       const CVecXd &vel_limits, const CVecXd &acc_limits,
                       const CVecXd &jerk_limits,
                       const CVecXd &v0 = CVecXd::Zero(0),
                       const CVecXd &v1 = CVecXd::Zero(0),
                       const CVecXd &a0 = CVecXd::Zero(0),
                       const CVecXd &a1 = CVecXd::Zero(0),
                       unsigned window_size = 2, double sample_step = 0.1)
    {
        RVS_ENSURE(path->IsValid(), "path is not valid");
        RVS_ENSURE(path->GetLength() > Constants<double>::Small(),
                   "path is too short");
        double length = path->GetLength();
        sample_step = std::clamp(sample_step, length / 1000.0, length / 4.0);
        auto qs = path->SamplePoints(sample_step);
        new (this) TrajectorySplineEx(qs, vel_limits, acc_limits, jerk_limits,
                                      v0, v1, a0, a1, window_size);
    }

    ~TrajectorySplineEx() { RVS_TRACE("Destrucint TrajectoryRnSplineEx"); }

    virtual int GetDoF() const override
    {
        return LieGroup::DOF > 0 ? LieGroup::DOF : m_qs.cols();
    }

    virtual RVSReturn
    GetWaypointAndTimeStamps(std::vector<LieGroup> &waypoints,
                             std::vector<double> &timestamps) const override
    {
        if (!this->IsValid()) return RVSReturn_Failed;
        waypoints.clear();
        timestamps.clear();
        for (unsigned i = 0; i < m_qs.rows(); ++i) {
            waypoints.emplace_back(LieGroup(m_qs.row(i)));
            timestamps.emplace_back(m_ts_wp[i]);
        }
        return RVSReturn_Success;
    }

private:
    MatXd m_qs; ///< waypoints
    std::vector<double> m_ts_wp; ///< waypoints timestamps
};
///@}
} // namespace __RVS_HIDE

using TrajectoryRnCubicSplineEx = __RVS_HIDE::TrajectorySplineEx<CubicSplineRn>;
using TrajectoryRnBSpline4thEx = __RVS_HIDE::TrajectorySplineEx<BSpline4thRn>;
using TrajectorySE3CubicSplineEx =
    __RVS_HIDE::TrajectorySplineEx<CubicSplineSE3>;
///@}
} // namespace RVS