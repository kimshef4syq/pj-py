// Copyright (c) RVBUST, Inc - All rights reserved.

#pragma once
#include <RVS/Controller/GenericRobotController.h>
#include <RVS/Trajectory/TrajectoryWithPath.h>
#include <variant>

namespace RVS
{
/**
 * @brief TrajectoryJogger is used to jog robot along a specific trajectory.
 *
 */
class TrajectoryJogger
{
public:
    explicit TrajectoryJogger(
        std::shared_ptr<GenericRobotController> controller);

    bool SetSpeedRatio(double ratio);

    /**
     * @brief Set the jogging trajectory
     *
     * @param traj trajectory
     * @return true
     * @return false
     */
    bool SetTrajectory(std::shared_ptr<TrajectoryWithPath<Rxd>> traj);

    /**
     * @brief Set jogging trajectory
     *
     * @param traj trajectory
     * @return true
     * @return false
     */
    bool SetTrajectory(std::shared_ptr<TrajectoryWithPath<Pose>> traj);

    /**
     * @brief Jog along trajectory
     *
     * @param forward move forward or backward
     * @return RVSReturn
     */
    RVSReturn JogTrajectory(bool forward = true);

    /**
     * @brief Stop jogging along trajectory
     *
     * @return true
     * @return false
     */
    bool StopJog();

    /**
     * @brief Get the jogging percentage, how much have been moved.
     *
     * @return double
     */
    double GetJogPercentage() const;

    std::shared_ptr<GenericRobotController> GetController() const;

    ///@brief Print TrajectoryJogger state.
    void PrintState() const;

private:
    std::weak_ptr<GenericRobotController> m_controller; ///< robot controller
    std::variant<std::shared_ptr<TrajectoryWithPath<Rxd>>,
                 std::shared_ptr<TrajectoryWithPath<Pose>>>
        m_traj; ///< jogging trajectory
    bool m_is_running{false}; ///< is robot executing jogging trajectory
};
} // namespace RVS