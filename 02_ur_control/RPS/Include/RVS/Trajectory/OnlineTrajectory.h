// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Filter/FilterBase.h>
#include <RVS/Trajectory/TrajectoryWithPath.h>

namespace RVS
{

/// @addtogroup Trajectory
/// @{

/**
 * @brief OnlineTrajectory computes the next output state of trajectory within
 * 1ms, so that it could be used online.
 *
 * @note We use different implementation for R3xSO3 online trajectory and Rn
 * online trajectory. For R3xSO3 online trajectory, we use a trapezoidal or
 * double S trajectory. And for Rn online trajectory, we use nonlinear filter or
 * TopiCo library.
 *
 * @tparam Group
 */
template <typename Group>
class OnlineTrajectory
{
public:
    DEFINE_SPACE_TYPES(Group)

    /**
     * @brief Construct a new OnlineTrajectory object
     *
     * @note When initializing, we can use target state same with start state
     *
     * @param start start state
     * @param target target state
     * @param vel_limits velocity limits
     * @param acc_limits acceleration limits
     * @param jerk_limits jerk limits
     * @param period control period
     * @param speed_ratio trajectory speed_ratio
     * @param traj_type TrajType_Trapezoidal (acceleration limited) or
     * TrajType_DoubleS (jerk limited) are recommanded for their compution speed
     * @param use_change_direction allow to try change jog direction without
     * stop at corner point, this will use Bezier5th blending segment with
     * toppra trajectory
     */
    OnlineTrajectory(const TrajectoryState<Group> &start,
                     const TrajectoryState<Group> &target,
                     const CVecXd &vel_limits, const CVecXd &acc_limits,
                     const CVecXd &jerk_limits, double period,
                     double speed_ratio = 1.0,
                     TrajType traj_type = TrajType_DoubleS,
                     bool use_change_direction = true);

    /**
     * @brief Compute the nest output state
     *
     * @return TrajectoryState<Group>
     */
    TrajectoryState<Group> ComputeNextState();

    /**
     * @brief Get the current state
     *
     * @return TrajectoryState<Group>
     */
    TrajectoryState<Group> GetCurrentState() const;

    /**
     * @brief Get the target state
     *
     * @return TrajectoryState<Group>
     */
    TrajectoryState<Group> GetTargetState() const;

    /**
     * @brief Get remain length, which is the distance between target state and
     * current state
     *
     * @return double
     */
    double GetRemainedLength() const;

    ///@brief Update trajectory if some constraints changed
    bool UpdateTrajectory();

    ///@brief Update target state, call UpdateTrajectory after setting new
    /// target state
    void UpdateTargetState(const TrajectoryState<Group> &state);

    ///@brief Update start state (current state), this is called seldomly
    void UpdateStartState(const TrajectoryState<Group> &state);

    ///@brief Update velocity limits
    void UpdateVelocityLimits(const CVecXd &vel_limits);

    ///@brief Update acceleration limits
    void UpdateAccelerationLimits(const CVecXd &acc_limits);

    ///@brief Update jerk limits
    void UpdateJerkLimits(const CVecXd &jerk_limits);

    ///@brief Update speed ratio
    void UpdateSpeedRatio(double speed_ratio);

    ///@brief Whether target state is reached or not
    bool IsReached() const;

    ///@brief Stop the trajectory as soon as possible
    bool StopTrajectory();

    ///@brief Get generated trajectory
    std::shared_ptr<TrajectoryWithPath<Group>> GetTrajectory() const;

private:
    /**
     * @brief Extend the trajectory along current direction
     *
     * @return true
     * @return false
     */
    bool _ExtendTrajectory();

    /**
     * @brief Change trajectory direction when new tangent is not same with
     * current tangent
     *
     * @return true
     * @return false
     */
    bool _ChangeDirection();

    /**
     * @brief Whether current state's velocity direction is same with the
     * direction from current state position to target state position.
     *
     * @return true
     * @return false
     */
    bool _IsDirectionChanged();

    TrajectoryState<Group> m_current_state; ///< current state
    TrajectoryState<Group> m_target_state; ///< target state
    const CVecXd m_vel_limits_max; ///< velocity limits when speed ratio is 1.0
    CVecXd m_vel_limits; ///< actual velocity limits considering speed ratio
    CVecXd m_acc_limits; ///< acceleration limits
    CVecXd m_jerk_limits; ///< jerk limits
    double m_period; ///< control period
    TrajType m_traj_type; ///< trajectory type, DoubleS features with jerk
                          ///< limited, Trapezoidal only acceleration limited
    bool m_use_change_direction; ///< whether to use Toppra and Bezier5th
                                 ///< segment to try change trajectory direction
                                 ///< without stop

    const size_t m_dof; ///< trajectory dof
    double m_current_time; ///< current state time stamp, range [0, trajectory
                           ///< duration]
    bool m_is_stop_trajectory; ///< whether current trajectory is generated by
                               ///< StopTrajectory()

    std::shared_ptr<TrajectoryWithPath<Group>>
        m_trajectory; ///< current trajectory instance
};

std::ostream &operator<<(std::ostream &out,
                         const OnlineTrajectory<Rxd> &online_traj);
std::ostream &operator<<(std::ostream &out,
                         const OnlineTrajectory<Pose> &online_traj);

/// @}
} // namespace RVS
