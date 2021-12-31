// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "TrajectorySplineBase.h"
#include <tuple>

namespace RVS
{
///@addtogroup Trajectory
///@{

/**
 * @brief P4 means piecewise polynomial path planner.
 *
 */
class TrajectoryRnP4 : public TrajectoryRnSpline
{
public:
    struct Options
    {
        size_t polynomial_order = 6;
        size_t derivative_order = 3;
        size_t continuity_order = 2;
        size_t num_sample_points_each_segment = 10;
        size_t max_iters = 10000;
        double time_limits = 10.0; //< seconds

        /**
         * @brief Solver options for TrajectoryRnP4
         *
         * @param polynomial_order_ order of polynomial
         * @param derivative_order_ order of polynomial dirivative to be
         * minimized
         * @param continuity_order_ order to be kept continuous
         * @param num_sample_points_each_segment_ sampled points to check
         * constraints on each segment
         * @param max_iters_ max iterations
         * @param time_limits_ max compution time in seconds
         */
        Options(size_t polynomial_order_ = 6, size_t derivative_order_ = 3,
                size_t continuity_order_ = 2,
                size_t num_sample_points_each_segment_ = 10,
                size_t max_iters_ = 10000, double time_limits_ = 10.0)
            : polynomial_order(polynomial_order_),
              derivative_order(derivative_order_),
              continuity_order(continuity_order_),
              num_sample_points_each_segment(num_sample_points_each_segment_),
              max_iters(max_iters_), time_limits(time_limits_)

        {
        }
    };

    /**
     * @brief Construct a new TrajectoryRnP4 trajectory.
     *
     * @param waypoints via points
     * @param timestamps timestamps at waypoints
     * @param vel_limits velocity limits
     * @param acc_limits accleration limits
     * @param jerk_limits jerk limits
     * @param options @see Options
     */
    TrajectoryRnP4(const std::vector<Rxd> &waypoints,
                   const std::vector<double> &timestamps,
                   const CVecXd &vel_limits, const CVecXd &acc_limits,
                   const CVecXd &jerk_limits, Options options = Options());
};
///@}
} // namespace RVS