/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include <RVS/Trajectory/TrajectoryWithPath.h>

namespace RVS
{
/// @addtogroup Trajectory
/// @{
using namespace std;

template <typename LieGroup>
class Trajectory;

template <typename LieGroup>
class TrajectoryTotp : public TrajectoryWithPath<LieGroup>
{
    using Tangent = typename LieGroup::Tangent;
    using Curvature = typename LieGroup::Tangent;

public:
    /**
     * @brief Construct a new trajectory method totp
     *
     * @param path trajectory path
     * @param limits_profile kinematic constraints on velocity,
     * acceleration, jerk
     * @param time_step integration step
     */
    TrajectoryTotp(typename PathBase<LieGroup>::Ptr path,
                   TrajProfileConstraintsBaseConstPtr limits_profile,
                   double time_step = 0.001);

private:
    /**
     * @brief Generates a time-optimal trajectory path parameter s(t)
     *
     * @param path[in] : trajectory path
     * @param limits_profile[in] : kinematic constraints on velocity,
     * acceleration, jerk
     * @return PSpline1d::Ptr path parameter s(t)
     */
    PSpline1d::Ptr
    ComputeTrajectory(typename PathBase<LieGroup>::ConstPtr path,
                      TrajProfileConstraintsBaseConstPtr limits_profile);

    void CalculateSwitchingPoints();
    std::vector<double> GetSwitchingPointsOnSegment(
        typename PathSegmentBase<LieGroup>::ConstPtr segment);
    std::list<std::pair<double, bool>>
    GetSwitchingPointsOnPath(typename PathBase<LieGroup>::ConstPtr path);
    double GetNextSwitchingPointOnPath(double s, bool &discontinuity);

    bool GetNextSwitchingPoint(double path_pos,
                               TrajectoryStep &next_switching_point,
                               double &before_acceleration,
                               double &after_acceleration);
    bool GetNextAccelerationSwitchingPoint(double path_pos,
                                           TrajectoryStep &next_switching_point,
                                           double &before_acceleration,
                                           double &after_acceleration);
    bool GetNextVelocitySwitchingPoint(double path_pos,
                                       TrajectoryStep &next_switching_point,
                                       double &before_acceleration,
                                       double &after_acceleration);
    bool IntegrateForward(std::list<TrajectoryStep> &trajectory,
                          double acceleration);
    void IntegrateBackward(std::list<TrajectoryStep> &start_trajectory,
                           double path_pos, double path_vel,
                           double acceleration);
    double GetMinMaxPathAcceleration(double path_position, double path_velocity,
                                     bool max);
    double GetMinMaxPhaseSlope(double path_position, double path_velocity,
                               bool max);
    double GetAccelerationMaxPathVelocity(double path_pos) const;
    double GetVelocityMaxPathVelocity(double path_pos) const;
    double GetAccelerationMaxPathVelocityDeriv(double path_pos);
    double GetVelocityMaxPathVelocityDeriv(double path_pos);

    const double m_time_step;
    CVecXd m_vel_limits;
    CVecXd m_acc_limits;
    std::list<std::pair<double, bool>>
        m_switching_points; ///< switching points are used for totp trajectory,
                            /// it means the candidate switching points of
                            /// max/min accelerations
    // non-empty only if the trajectory generation failed.
    std::list<TrajectoryStep> end_trajectory;
};

/// @}
} // namespace RVS
