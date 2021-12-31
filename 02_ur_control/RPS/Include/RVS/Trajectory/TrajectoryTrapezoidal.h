// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/TrajectoryDoubleS.h>
#include "PathBezier2ndBlend.h"

namespace RVS
{

/// @addtogroup Trajectory
/// @{

/**
 * @brief Optimise a trajectory to double-s velocity trajectory
 *
 * @tparam LieGroup : waypoint LieGroup type
 */
template <typename LieGroup>
class TrajectoryTrapezoidal : public TrajectoryDoubleS<LieGroup>
{
public:
    /**
     * @brief Compute a trajectory path parameter s(t), where \f$ \dot s(t) \f$
     * is like a trapezoidal shape
     *
     * @param path[in] : trajectory path
     * @param limits_profile[in] : kinematic constraints on velocity,
     * acceleration, jerk
     * @param min_duration[in] : minium trajectory duration, 0.0 means that the
     * minium duration will be calculated by max velocities and accelerations
     * @param vel_init[in] : initial velocity
     * @param vel_end[in] : end velocity
     * @param min_duration[in] : minium trajectory duration
     */
    TrajectoryTrapezoidal(typename PathBase<LieGroup>::Ptr path,
                          TrajProfileConstraintsBaseConstPtr limits_profile,
                          double vel_init = 0.0, double vel_end = 0.0,
                          double min_duration = 0.0);

protected:
    /**
     * @brief Compute a path parmaeter s(t), with s'(t) being trapezoidal curve
     *
     * @param path : trajectory path
     * @param limits_profile : velocity/acceleration/jerk limits profile of each
     * DoF
     * @param vel_init : trajectory initial point velocity value
     * @param vel_end : trajectory end point velocity value
     * @param min_duration : optional tarjectory minium duration, default the
     * trajectory duration will be as short as possible
     * @return PSpline1d::Ptr path parameter curve s(t)
     */
    static PSpline1d::Ptr
    ComputeTrajectory(typename PathBase<LieGroup>::ConstPtr path,
                      TrajProfileConstraintsBaseConstPtr limits_profile,
                      double vel_init = 0.0, double vel_end = 0.0,
                      double min_duration = 0.0);

    /**
     * @brief Compute a time scaling parameter whose derivative is like
     * trapezoidal shape
     *
     * @param q0[in] : start position
     * @param q1[in] : end position
     * @param v0[in] : start derivative
     * @param v1[in,out] : end derivative
     * @param vmax[in] : max derivative
     * @param amax[in] : max second derivative
     * @param t0[in] : current step timestamp
     * @param tj_steps[out] : the trajectory steps
     * @param seg_no[in] : segment sequence number
     * @return true
     * @return false
     */
    static bool _ComputeTrapezoidalSVel(double q0, double q1, double v0,
                                        double &v1, double vmax, double amax,
                                        double t0,
                                        std::list<TrajectoryStep> &tj_steps,
                                        int seg_no = 0);

    /**
     * @brief When the position, velocity, acceleration constrains cannot be all
     * satisfied, search forward to lower the velocity/acceleration at some
     * steps
     *
     * @param tj_steps[in,out] : trajectory steps
     * @return true
     * @return false
     */
    static bool
    _AdjustReverseWithMaxAcceleration(std::list<TrajectoryStep> &tj_steps);
};

/// @}
} // namespace RVS
