// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/TrajectoryDoubleS.h>

namespace RVS
{

/// @addtogroup Trajectory
/// @{

/**
 * @brief Generating a constant speed trajectory
 *
 * @tparam LieGroup waypoint LieGroup type
 */
template <typename LieGroup>
class TrajectoryConstantSpeed : public TrajectoryDoubleS<LieGroup>
{
public:
    /**
     * @brief Construct a new constant speed trajectory
     *
     * @param path trajectory path
     * @param speed value of trajectory speed (m/s if only cartesian position
     * speed is considered)
     * @param vel_init initial speed
     * @param vel_end final speed
     * @param acc_max max acceleration
     * @param jerk_max max jerk
     */
    TrajectoryConstantSpeed(typename PathBase<LieGroup>::Ptr path,
                            double speed = 0.1, double vel_init = 0.0,
                            double vel_end = 0.0, double acc_max = 5.0,
                            double jerk_max = 20.0);
};

/// @}
} // namespace RVS
