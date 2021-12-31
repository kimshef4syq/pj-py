// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Controller/GenericRobotController.h>
#include <RVS/Trajectory/OnlineTrajectoryRn.h>
#include <RVS/CollisionChecker/FCLCollisionChecker/FCLCollisionChecker.h>

namespace RVS
{

/// @addtogroup Controller
/// @{
/**
 * @brief ControllerJogger is used to help jog a robot
 *
 */
class ControllerJogger
{
public:
    /**
     * @brief Construct a new ControllerJogger object
     * @note ControllerJogger will only store a weak_ptr of
     * GenericRobotController, user must keep controller is valid outside,
     * otherwise an exception may be thrown when using member functions.
     *
     * @param controller
     * @param limits_scale sacle for maximum jogger velocity
     */
    ControllerJogger(std::shared_ptr<GenericRobotController> controller,
                     double limits_scale = 0.4);

    ~ControllerJogger();

    /**
     * @brief Start controller jog mode
     * @param use_change_direction don't stop when trajectory direction changes,
     * if you will use UpdateTarget mostly, true is suggested; if you will use
     * UpdateDirection a lot, false is suggested.
     * @param traj_type the trajectory type of internal trajectory, double-s for
     * acceleration continuous, but a larger time lag; trapezoidal is more
     * timely
     */
    bool StartJog(bool use_change_direction = false,
                  TrajType traj_type = TrajType_DoubleS);

    ///@brief Stop jog mode. This function will block to wait jog thread
    /// exitting.
    bool StopJog();

    bool IsJogThreadAlive() const;

    ///@brief Stop jog movement, but remain jog mode. Movement may not
    /// stopped
    /// after this function returns, but takes a short time to decelerate,
    /// set "wait = true" as blocking mode.
    bool StopMove(bool wait = false);

    bool ConfigCollisionCheck(std::shared_ptr<Environment> env);

    void DisableCollisionCheck();

    const std::shared_ptr<CollisionReport> &GetCollisionReport() const;

    bool UpdateTarget(const TrajectoryState<Pose> &state);

    bool UpdateTarget(const TrajectoryState<Rxd> &state);

    bool UpdateTarget(const SE3d &target);

    bool UpdateTarget(const Rxd &target);

    ///@brief Update cartesian jog direction
    bool UpdateDirection(const RxTangentd &joint_direction);

    ///@brief Update joint jog direction. If not is_base_frame, jog in tool
    /// frame.
    bool UpdateDirection(const SE3Tangentd &cart_direction,
                         bool is_base_frame = true);

    ///@brief Update jog speed ratio
    bool UpdateSpeedRatio(const double ratio);

    ///@ Get jog speed ratio
    double GetSpeedRatio() const;

    bool IsReached() const;

    void PrintState() const;

    std::shared_ptr<GenericRobotController> GetRobotController() const;

    std::shared_ptr<OnlineTrajectory<Rxd>> GetOnlineTrajectoryJoint() const;

    std::shared_ptr<OnlineTrajectory<Pose>>
    GetOnlineTrajectoryCartesian() const;

private:
    void _JogThread();
    bool _IsInJointPosLimits(const JointVector &joints) const;

    bool _IsInCollision(const JointVector &joints) const;

    /**
     * @brief Update online cartesian trajectory
     *
     * @return int -1 : break jog thread loop
     *              0 : continue next line
     *              1 : continue next loop
     */
    int _UpdateOnlineTraj();
    int _UpdateOnlineCartTraj();

    std::weak_ptr<GenericRobotController> m_controller;
    double m_limits_sacle; ///< max cart/joint vel/acc/jerk limits scale
    double m_servoj_time; ///< controlling period

    bool m_cartesian_space; ///< is jogging in cartesian space
    bool m_stop_jog; ///< is stopping jog mode
    bool m_stop_move; ///< is stopping jog movement, but remain jog mode
    bool m_reached; ///< reached current online trajectory target position
    bool m_need_update_target; ///< target position need to be updated, eg.
                               ///< after direction changed
    bool m_direction_mode; ///< move along direction or approach a target
    bool m_jog_thread_alive; ///< is jog thread running
    bool m_need_update_traj; ///< whether online trajectory constraints changed
                             ///< and need update online trajectory
    double m_jog_speed_ratio; ///< jog speed ratio, [0.001, 1.0]
    RxTangentd m_joint_direction;
    SE3Tangentd m_cart_direction;
    bool m_is_base_frame; ///< jog linear in base / tool frame

    TrajectoryState<Rxd> m_joint_target_state;
    TrajectoryState<Pose> m_cartesian_target_state;
    std::shared_ptr<OnlineTrajectory<Rxd>> m_online_traj_joint;
    std::shared_ptr<OnlineTrajectory<Pose>> m_online_traj_cart;

    TrajectoryState<Rxd> m_servo_state;
    size_t m_dof;
    std::vector<JointLimit> m_joint_limits;

    bool m_enable_collision_check;
    CollisionCheckerBasePtr m_collision_checker;
    std::shared_ptr<Environment> m_env;
    std::shared_ptr<CollisionReport> m_collision_report;
};
/// @}
} // namespace RVS
