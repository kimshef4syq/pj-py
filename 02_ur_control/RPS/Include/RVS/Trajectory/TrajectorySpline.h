// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "TrajectorySplineBase.h"

namespace RVS
{
///@addtogroup Trajectory
///{
namespace __RVS_HIDE
{

///@addtogroup Trajectory
///{
/**
 * @brief Calculate a spline trajectory with
 * - Non-linear optimization method with (NLP)
 *  > CubicSpline for Rn/R3xSO3 space, time optimal and jerk-bounded
 *  > BSpline4th for Rn space, time optimal and jerk-bounded
 *
 * Note:
 * - ISP method is much faster than NLP method, but result of NLP is more
 * optimal with shorter duration.
 * - Spline trajectory accrosses (interpolates) all given waypoints, the path
 * may be deformed if waypoints are dense and fluctuated. While geometric path
 * based trajectory don't pass the given waypoints but approximates them with a
 * giving blending distance.
 * - If waypoints number is large, eg. > 30, the process maybe very slow (tens
 * of seconds), then considering using TrajectorySplineEx or TrajectorySplineISP
 * instead.
 *
 * @tparam SplineType
 */
template <typename SplineType>
class TrajectorySpline : public TrajectorySplineBase<SplineType>
{
public:
    using Ptr = std::shared_ptr<TrajectorySpline>;
    using ConstPtr = std::shared_ptr<const TrajectorySpline>;
    using LieGroup = typename SplineType::LieGroup;
    using Tangent = typename LieGroup::Tangent;
    using Curvature = typename LieGroup::Tangent;
    using Torsion = typename LieGroup::Tangent;

    /**
     * @brief Construct a new TrajectoryRnSpline, the spline will accross all
     * points and minimize the trajectory duration with given vel/acc/jerk
     * limits.
     *
     * @param qs interpolating points that need to be accrossed
     * @param vel_limits velocity limits of each dof
     * @param acc_limits acceleration limits of each dof
     * @param jerk_limits jerk limits of each dof
     * @param v0 start 1st derivative, aka. velocity
     * @param v1 end 1st derivative, aka. velocity
     * @param a0 start 2nd derivative, aka. acceleration
     * @param a1 end 2nd derivative, aka. acceleration
     */
    TrajectorySpline(const MatXd &qs, const CVecXd &vel_limits,
                     const CVecXd &acc_limits, const CVecXd &jerk_limits,
                     const CVecXd &v0 = CVecXd::Zero(0),
                     const CVecXd &v1 = CVecXd::Zero(0),
                     const CVecXd &a0 = CVecXd::Zero(0),
                     const CVecXd &a1 = CVecXd::Zero(0));

    /**
     * @brief Construct a new TrajectoryRnSpline, the spline will accross all
     * points and minimize the trajectory duration with given vel/acc/jerk
     * limits.
     *
     * @param qs interpolating points that need to be accrossed
     * @param vel_limits velocity limits of each dof
     * @param acc_limits acceleration limits of each dof
     * @param jerk_limits jerk limits of each dof
     * @param v0 start 1st derivative, aka. velocity
     * @param v1 end 1st derivative, aka. velocity
     * @param a0 start 2nd derivative, aka. acceleration
     * @param a1 end 2nd derivative, aka. acceleration
     */
    inline TrajectorySpline(const std::vector<LieGroup> &qs,
                            const CVecXd &vel_limits, const CVecXd &acc_limits,
                            const CVecXd &jerk_limits,
                            const CVecXd &v0 = CVecXd::Zero(0),
                            const CVecXd &v1 = CVecXd::Zero(0),
                            const CVecXd &a0 = CVecXd::Zero(0),
                            const CVecXd &a1 = CVecXd::Zero(0))
        : TrajectorySpline(ConvertVecLieGroupToMatXd(qs), vel_limits,
                           acc_limits, jerk_limits, v0, v1, a0, a1)
    {
    }

    /**
     * @brief Construct a new TrajectoryRnSpline, the spline will accross some
     * sampled points on given path and minimize the trajectory duration with
     * given vel/acc/jerk limits.
     *
     * @param path Rxd path
     * @param vel_limits velocity limits of each dof
     * @param acc_limits acceleration limits of each dof
     * @param jerk_limits jerk limits of each dof
     * @param v0 start 1st derivative, aka. velocity
     * @param v1 end 1st derivative, aka. velocity
     * @param a0 start 2nd derivative, aka. acceleration
     * @param a1 end 2nd derivative, aka. acceleration
     * @param sample_step the distance of neighbour sampled points on the path
     */
    TrajectorySpline(std::shared_ptr<const PathBase<LieGroup>> path,
                     const CVecXd &vel_limits, const CVecXd &acc_limits,
                     const CVecXd &jerk_limits,
                     const CVecXd &v0 = CVecXd::Zero(0),
                     const CVecXd &v1 = CVecXd::Zero(0),
                     const CVecXd &a0 = CVecXd::Zero(0),
                     const CVecXd &a1 = CVecXd::Zero(0),
                     double sample_step = 0.1)
    {
        RVS_ENSURE(path->IsValid(), "path is not valid");
        RVS_ENSURE(path->GetLength() > Constants<double>::Small(),
                   "path is too short");
        double length = path->GetLength();
        RVS_DEBUG("sample_step will be clamped to [length/100, length/4]");
        sample_step = std::clamp(sample_step, length / 100.0, length / 4.0);
        auto qs = path->SamplePoints(sample_step);
        new (this) TrajectorySpline(qs, vel_limits, acc_limits, jerk_limits, v0,
                                    v1, a0, a1);
    }

    virtual ~TrajectorySpline() { RVS_TRACE("Destructing TrajectorySpline"); }

    virtual int GetDoF() const override { return m_dof; }

    virtual RVSReturn
    GetWaypointAndTimeStamps(std::vector<LieGroup> &waypoints,
                             std::vector<double> &timestamps) const override
    {
        if (!this->IsValid()) {
            RVS_ERROR("Trajectory not valid");
            return RVSReturn_Failed;
        }
        waypoints.clear();
        timestamps.clear();
        for (unsigned i = 0; i < m_qs.rows(); ++i) {
            waypoints.emplace_back(LieGroup(m_qs.row(i)));
            timestamps.emplace_back(m_ts_wp[i]);
        }
        return RVSReturn_Success;
    }

private:
    enum InitialValueType
    {
        VelConstraint = 0,
        AccConstraint = 1,
        JerkConstraint = 2,
        AllConstraint = 3
    };

    bool _ComputeSplineTraj(const MatXd &qs, const CVecXd &vel_limits,
                            const CVecXd &acc_limits, const CVecXd &jerk_limits,
                            const CVecXd &v0, const CVecXd &v1,
                            const CVecXd &a0, const CVecXd &a1);

    static std::vector<double> _ComputeInitialValue(
        const MatXd &qs, const CVecXd &vel_limits, const CVecXd &acc_limits,
        const CVecXd &jerk_limits,
        InitialValueType type = InitialValueType::VelConstraint);

    bool _GetConstraintValue(const double *x, unsigned n, double *c, unsigned m,
                             void *data, const std::vector<unsigned> &var_idx,
                             const CVecXd &v0, const CVecXd &v1,
                             const CVecXd &a0, const CVecXd &a1);

    MatXd m_qs; ///< waypoints
    unsigned m_dof; /// DoF
    std::vector<double> m_ts_wp; ///< ts where the value equal to m_qs

    unsigned m_num_segs; ///< number of segments
};
///@}
} // namespace __RVS_HIDE

using TrajectoryRnCubicSpline = __RVS_HIDE::TrajectorySpline<CubicSplineRn>;
using TrajectoryRnBSpline4th = __RVS_HIDE::TrajectorySpline<BSpline4thRn>;
using TrajectorySE3CubicSpline =
    __RVS_HIDE::TrajectorySpline<CubicSplineSE3>;
///@}
} // namespace RVS