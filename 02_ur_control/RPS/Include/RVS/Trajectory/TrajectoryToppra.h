// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/TrajectoryWithPath.h>

namespace RVS
{
/// @addtogroup Trajectory
/// @{

/**
 * @brief Trajectory planning method with TOPPRA
 *
 * @note toppra reference: https://toppra.readthedocs.io/en/latest/index.html
 *
 * - Representation of trajectory:
 *      - q(t) = p(s(t))
 *      - q'(t) = p'(s) * s'(t)
 *      - q"(t) = p'(s) * s"(t) + p"(s) * s'(t)^2
 * - Defination:
 *      - control variable : u = s"(t)
 *      - state variable   : x = s'(t)^2
 * - Two passes:
 *      - BackwardProcess: recursively computes the controllable sets given the
 * desired ending velocity
 *      - ForwardProcess: greedily select the controls
 *
 * @tparam LieGroup space type
 */
template <typename LieGroup>
class TrajectoryToppra : public TrajectoryWithPath<LieGroup>
{
public:
    /**
     * @brief Compute a trajectory path parameter s(t), whose veloticy is
     * continuous like trapezoidal curve, according to given  limits of max
     * velocities, accelerations and min_duration. There is possibility that the
     * min duration cannot be reached subject to max velocities and
     * accelerations
     *
     * @param path[in] : trajectory path
     * @param limits_profile[in] : kinematic constraints on velocity,
     * acceleration, jerk
     * @param min_duration[in] : minium trajectory duration, 0.0 means that the
     * minium duration will be calculated by max velocities and accelerations
     * @param vel_init[in] : initial velocity
     * @param vel_end[in] : end velocity
     * @param min_duration[in] : minium trajectory duration
     * @param step_size[in] : stage step size, controlling calculate accuracy
     */
    TrajectoryToppra(typename PathBase<LieGroup>::Ptr path,
                     TrajProfileConstraintsBaseConstPtr limits_profile,
                     double vel_init = 0.0, double vel_end = 0.0,
                     const double min_duration = 0.0, double step_size = 0.01);

private:
    PSpline1d::Ptr
    ComputeTrajectory(typename PathBase<LieGroup>::ConstPtr path,
                      TrajProfileConstraintsBaseConstPtr limits_profile,
                      double vel_init = 0.0, double vel_end = 0.0,
                      const double min_duration = 0.0, double step_size = 0.01);

    CVecXd GetMaxVelocityConstrain(double s) const;
    CVecXd GetMaxAccelerationConstrain(double s) const;

    /**
     * @brief Solve a 2-dimension linear program problem
     * Minimize:  v[0] * vars[0] + v[1] * vars[1]
     *  s.t.:  a[i] * vars[0] + b[i] * vars[1] <= c[i], 0 <= i <
     * number_of_constrains low[j] <= vars[j] <= high[j], 0 <= j < 2
     *
     * @param v[in] : coeffients of objective
     * @param a[in] : coeffients of first variable
     * @param b[in] : coeffients of second variable
     * @param c[in] : coeffients of constants
     * @param opt_vars[out] : optimal values of variables
     * @param low[in] : low boundary of variables
     * @param high[in] : up boundary of variables
     * @return true
     * @return false
     */
    static bool SolveLP2d(const CVecXd &v, const CVecXd &a, const CVecXd &b,
                          const CVecXd &c, CVecXd &opt_vars, CVecXd &low,
                          CVecXd &high);
    TrajProfileConstraintsBaseConstPtr m_limits_profile;
    constexpr static double maxu = 1e5; // max value of control variable
    constexpr static double maxx = 1e5; // max value of state variable
    constexpr static double inf = 1e10; // unconstrained variable max values
    constexpr static double eps = Constants<float>::Epsilon();
};

/// @}
} // namespace RVS