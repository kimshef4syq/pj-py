// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/TrajectoryWithPath.h>

namespace RVS
{
/**
 * @brief Trajectory q(t) consists of a path parameter time scaling curve s(t)
 * and geometric path p(s). Trajectory method focuses on calculating smooth
 * s(t).
 * - Representation of trajectory:
 *      - s = s(t), 0 <= t <= traj_duration
 *      - p = p(s), 0 <= s <= path_length
 *      - q(t) = p(s(t))
 *      - q'(t) = p'(s) * s'(t)
 *      - q"(t) = p'(s) * s"(t) + p"(s) * s'(t)^2
 *      - q"'(t) = 3 * p"(s) * s"(t) * s'(t) + p'(s) * s"'(t) + p"'(s) * s'(t)^3
 *
 * This method is able to compute jerk-bounded time optimal trajectory
 * for Rn/Cartesian space. At each state, it computes a current-state to
 * rest-state path parameter curve (Abbr. s(t) ), named as test-trajectory.
 * Test-trajectory comprises three phases:
 * - 1) Phase 1, with max s-jerk, to switching timestamp1 (swt1), namely
 * AccPhase
 * - 2) Phase 2, with min s-jerk, to switching timestamp2 (swt2), namely
 * DecPhase1
 * - 3) Phase 3, with max s-jerk, to switching timestamp3 (swt3), namely
 * DecPhase2 swt2 is found using bin-searching method, so that at swt3, s-vel
 * and s-acc are both zero.
 *
 * Please refer to this paper "Efficient Online Computation of Smooth
 * Trajectories Along Geometric Paths for Robotic Manipulators" for more
 * details.
 *
 */

template <typename LieGroup>
class TrajectoryTotp3 : public TrajectoryWithPath<LieGroup>
{

public:
    /**
     * @brief Generates a time-optimal trajectory path parameter
     *
     * @param path[in] : trajectory geometric path
     * @param limits_profile[in] : kinematic constraints on velocity,
     * acceleration, jerk
     * @param step[in] intergration step
     */
    TrajectoryTotp3(typename PathBase<LieGroup>::Ptr path,
                    TrajProfileConstraintsBaseConstPtr limits_profile,
                    double step = 0.005);

private:
    using State = std::array<double, 3>; // s_pos, s_vel, s_acc

    static PSpline1d::Ptr
    ComputeTrajectory(typename PathBase<LieGroup>::ConstPtr path,
                      TrajProfileConstraintsBaseConstPtr limits_profile,
                      double step = 0.005);

    /**
     * @brief Compute next path parameter state given value of control variable
     * (jerk)
     *
     * @param x current state
     * @param u value of control variable, aka jerk of path parameter
     * @param T integration period
     */
    inline static void _ComputeNextState(State &x, double u, double T)
    {
        const double T2 = T * T;
        const double uT = u * T;
        x[0] += x[1] * T + 0.5 * x[2] * T2 + 1. / 6. * uT * T2;
        x[1] += x[2] * T + 0.5 * uT * T;
        x[2] += uT;
    }

    /**
     * @brief Compute the minimum and maximum value of control variable. The
     * value will satisfy the current and next state (by integrating forward one
     * step) limits.
     *
     * @param path geometric path
     * @param x current state
     * @param vel_limits velocity limits of trajectory
     * @param acc_limits  acceleration limits of trajectory
     * @param jerk_limits  jerk limits of trajectory
     * @param T integration period
     * @param umin[OUT] minimum control value
     * @param umax[OUT] maximum control value
     * @return true
     * @return false
     */
    static bool _GetMinMaxControl(typename PathBase<LieGroup>::ConstPtr path,
                                  const State &x, const CVecXd &vel_limits,
                                  const CVecXd &acc_limits,
                                  const CVecXd &jerk_limits, double T,
                                  double &umin, double &umax);

    /**
     * @brief Given state x and limits, integrate forward with maximum control
     * (s-jerk) until the s-vel is zero, return the duration of integration, aka
     * max value of switching timestamp2.
     *
     * @param path geometric path that that needs to be followed
     * @param x current state
     * @param vel_limits velocity limits of trajectory
     * @param acc_limits acceleration limits of trajectory
     * @param jerk_limits jerk limits of trajectory
     * @param T integration period
     * @param swt2_max[OUT]
     * @return true
     * @return false
     */
    static bool
    _ComputeMaxValueOfSwt2(typename PathBase<LieGroup>::ConstPtr path,
                           const State &x, const CVecXd &vel_limits,
                           const CVecXd &acc_limits, const CVecXd &jerk_limits,
                           double T, double &swt2_max);

    /**
     * @brief Compute s-curve of Decleration phase 1 and 2 between switching
     * timestamp1 and switching timestamp3.
     *
     * @param path geometric path that need to be followed
     * @param x_swt1 state at switching timestamp1
     * @param vel_limits velocity limits of trajectory
     * @param acc_limits acceleration limits of trajectory
     * @param jerk_limits jerk limits of trajectory
     * @param T integration period
     * @param ts[OUT] timestamps of deceleration phase at integration points
     * @param xs[OUT] integration states of deceleration phase at integration
     * points
     * @param swt2_ref[IN,OUT] reference value of switching timestamp2
     * @return true
     * @return false
     */
    static bool _ComputeDecPhase(typename PathBase<LieGroup>::ConstPtr path,
                                 const State &x_swt1, const CVecXd &vel_limits,
                                 const CVecXd &acc_limits,
                                 const CVecXd &jerk_limits, double T,
                                 std::vector<double> &ts,
                                 std::vector<State> &xs,
                                 std::vector<double> &us, double &swt2_ref);

    /**
     * @brief Compute test trajectory from current state to rest state
     *
     * @param path geometric path that need to be followed
     * @param x0 current state
     * @param vel_limits velocity limits of trajectory
     * @param acc_limits acceleration limits of trajectory
     * @param jerk_limits jerk limits of trajectory
     * @param T integration period
     * @param ts[OUT] timestamps of deceleration phase at integration points
     * @param xs[OUT] integration states of deceleration phase at integration
     * points
     * @param swt2_ref[IN,OUT] reference value of switching timestamp2
     * @param last_piece[OUT] whether this test trajectory reaches to the final
     * target (s-pos is equal to length of path)
     * @return true
     * @return false
     */
    static bool
    _ComputeTestTrajectory(typename PathBase<LieGroup>::ConstPtr path,
                           const State &x0, const CVecXd &vel_limits,
                           const CVecXd &acc_limits, const CVecXd &jerk_limits,
                           double T, std::vector<double> &ts,
                           std::vector<State> &xs, std::vector<double> &us,
                           double &swt2_ref, bool &last_piece);
};

} // namespace RVS