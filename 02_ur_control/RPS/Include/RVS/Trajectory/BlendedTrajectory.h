// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/TrajectoryWithPath.h>

namespace RVS
{
///@addtogroup Trajectory
///@{

/**
 * @brief BlendedTrajectory, used to connecting two trajectories. The
 * trajectories to be conneted doesn't need to have same velocities and same
 * positions at connecting ends. Blending trajectory is kept at leat velocity
 * continuous.
 *
 * @tparam LieGroup Rxd, SE3, R3xSO3
 */
template <typename LieGroup>
class BlendedTrajectory : public TrajectoryBase<LieGroup>
{
public:
    using Tangent = typename LieGroup::Tangent;
    using Curvature = typename LieGroup::Tangent;
    using Torsion = typename LieGroup::Tangent;

    /**
     * @brief Construct a new Blender Trajectory object
     *
     * @param trajs trajectories to be connected
     * @param zone blending zone
     * @param speed_limit cartesian TCP linear speed limit for blending segment
     * @param acc_limit cartesian TCP linear acceleartion limit for blending
     * segment
     */
    BlendedTrajectory(
        std::vector<std::shared_ptr<TrajectoryWithPath<LieGroup>>> trajs,
        double zone = 0.5, double speed_limit = 1.0, double acc_limit = 2.0);

    virtual double GetDuration() const override;

    virtual LieGroup GetPosition(double t) const override;

    virtual Tangent GetVelocity(double t) const override;

    virtual Curvature GetAcceleration(double t) const override;

    virtual Torsion GetJerk(double t) const override;

    virtual bool IsValid() const override { return m_valid; }

    ///@brief Get the internal trajectories
    auto GetTrajs() const { return m_trajs; }

    ///@brief Get time knots of internal trajectories
    auto GetTrajTimeKnots() const { return m_traj_time_knots; }

    ///@brief Get start time knot of each trajectories
    auto GetTrajStartTimestamps() const { return m_traj_start_timestamps; }

private:
    std::pair<double, std::shared_ptr<TrajectoryWithPath<LieGroup>>>
    _GetNormlizedTimestampAndTraj(double t) const;

    bool m_valid; ///< whether the blended trajectory is valid or not
    std::vector<std::shared_ptr<TrajectoryWithPath<LieGroup>>>
        m_trajs; ///< store all the trajectories
    std::vector<double> m_traj_time_knots; ///< time knots of each trajectory
    std::vector<double>
        m_traj_start_timestamps; ///< start time knot of each trajectory
};
///@}
} // namespace RVS