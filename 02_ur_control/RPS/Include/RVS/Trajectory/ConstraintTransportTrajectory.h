// Copyright (c) RVBUST, Inc - All rights reserved.

#pragma once
#include <RVS/Trajectory/TrajectoryBase.h>

namespace RVS
{
/// @addtogroup Trajectory
/// @{

class ConstraintTransportTrajectory : public TrajectoryBase<Pose>
{
    using Tangent = typename Pose::Tangent;
    using Curvature = typename Pose::Tangent;

public:
    ///@brief Default construction method.
    ConstraintTransportTrajectory();

    /**
     * @brief Init from a cartesian R3(xyz) trajectory
     *
     * @param traj_r3 TCP position trajectory
     * @return true
     * @return false
     */
    bool Init(TrajectoryBase<JointVector>::Ptr traj_r3);

    /**
     * @brief Init from a series of TCP poses.
     *
     * @param poses TCP poese
     * @param max_vels velocity limits
     * @param max_accs acceleration limits
     * @param max_jerks jerk limits
     * @param blend_tolerance blend tolerance of path
     * @return true
     * @return false
     */
    bool Init(const std::vector<SE3d> &poses, const CVec3d &max_vels,
              const CVec3d &max_accs, const CVec3d &max_jerks,
              const double blend_tolerance);

    virtual Pose GetPosition(double t) const override final;
    virtual Tangent GetVelocity(double /* t */) const override final;
    virtual Curvature GetAcceleration(double /* t */) const override final;
    virtual double GetDuration() const override final;
    virtual bool IsValid() const override final;

private:
    TrajectoryBase<JointVector>::Ptr
        m_traj_r3; ///< robot TCP position trajectory
    bool m_valid{false}; ///< flag of whether the trajectory is valid
    CVec3d m_gravity; ///< gravity value
};

/// @}
} // namespace RVS
