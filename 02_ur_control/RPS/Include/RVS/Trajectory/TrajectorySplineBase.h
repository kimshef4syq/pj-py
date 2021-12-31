// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "SplineComposition.h"
#include <RVS/Trajectory/TrajectoryBase.h>
#include <RVS/Common/Macros.h>

namespace RVS
{
///@addtogroup Trajectory
///@{
namespace __RVS_HIDE
{
///@addtogroup Trajectory
///@{

/**
 * @brief TrajectorySplineBase is used to wrap a Spline with RVS Trajectory
 * interaface.
 *
 * @tparam SplineType
 */
template <typename SplineType>
class TrajectorySplineBase
    : public TrajectoryBase<typename SplineType::LieGroup>
{
public:
    RVS_DECLARE_PTR_MEMBER(TrajectorySplineBase<SplineType>)

    using LieGroup = typename SplineType::LieGroup;
    using Tangent = typename LieGroup::Tangent;
    using Curvature = typename LieGroup::Tangent;
    using Torsion = typename LieGroup::Tangent;

    TrajectorySplineBase(std::shared_ptr<SplineType> spline) : m_spline(spline)
    {
    }

    virtual bool IsValid() const override
    {
        return !!m_spline && m_spline->GetNumOfSegs();
    }

    virtual int GetDoF() const override
    {
        RVS_ENSURE(!!m_spline, "Trajectory spline is nullptr!");
        return m_spline->GetDoF();
    }

    virtual double GetDuration() const override
    {
        return IsValid() ? m_spline->GetKnotLength() : 0;
    }

    virtual LieGroup GetPosition(double t) const override
    {
        RVS_ENSURE(IsValid(), "Trajectory not valid");
        return LieGroup(m_spline->Eval(t, 0));
    }

    virtual Tangent GetVelocity(double t) const override
    {
        RVS_ENSURE(IsValid(), "Trajectory not valid");
        return Tangent(m_spline->Eval(t, 1));
    }

    virtual Curvature GetAcceleration(double t) const override
    {
        RVS_ENSURE(IsValid(), "Trajectory not valid");
        return Tangent(m_spline->Eval(t, 2));
    }

    virtual Torsion GetJerk(double t) const override
    {
        RVS_ENSURE(IsValid(), "Trajectory not valid");
        return Tangent(m_spline->Eval(t, 3));
    }

    virtual RVSReturn
    GetWaypointAndTimeStamps(std::vector<LieGroup> &waypoints,
                             std::vector<double> &timestamps) const override
    {
        if (!IsValid()) return RVSReturn_Failed;
        timestamps = m_spline->GetKnots();
        for (double t : timestamps) {
            waypoints.push_back(LieGroup((*m_spline)(t)));
        }
        return RVSReturn_Success;
    }

    ///@brief Get internal spline. TODO: We may let spline inherit from Path,
    /// then use exist GetPath instead of adding a new interface.
    typename SplineType::ConstPtr GetSpline() const { return m_spline; }

protected:
    TrajectorySplineBase() = default;

    typename SplineType::ConstPtr m_spline;
};

template <typename SplineType>
std::ostream &operator<<(std::ostream &out,
                         const TrajectorySplineBase<SplineType> &traj)
{
    out << fmt::format(
        "<TrajectorySplineBase{} at {}, duration: {:.4f}, valid: {}>",
        SplineType::LieGroup::NameStatic(traj.GetDoF()), (void *)&traj,
        traj.GetDuration(), traj.IsValid());
    return out;
}
///@}
} // namespace __RVS_HIDE
using TrajectoryRnSpline =
    __RVS_HIDE::TrajectorySplineBase<__RVS_HIDE::PSpline<Rxd>>;
///@}
} // namespace RVS
