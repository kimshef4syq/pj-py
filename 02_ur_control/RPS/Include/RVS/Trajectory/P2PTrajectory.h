// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "TrajectorySplineBase.h"

namespace RVS
{
///@addtogroup Trajectory
///@{

///@todo Add trapezoidal and double-s trajectory velocity profile

///@brief Point to point cubic polynomial trajectory
CVecXd P2PCubic(double q0, double q1, double v0, double v1, double T = 1.0);

///@brief Point to point cubic polynomial trajectory of multiple axes
MatXd P2PCubicMultiDim(const CVecXd &q0, const CVecXd &q1, const CVecXd &v0,
                       const CVecXd &v1, double T = 1.0);
///@brief Point to point quintic polynomial trajectory
CVecXd P2PQuintic(double q0, double q1, double v0, double v1, double a0,
                  double a1, double T = 1.0);

///@brief Point to point quintic polynomial trajectory of multiple axes
MatXd P2PQuinticMultiDim(const CVecXd &q0, const CVecXd &q1, const CVecXd &v0,
                         const CVecXd &v1, const CVecXd &a0, const CVecXd &a1,
                         double T = 1.0);

///@brief Point to point cubic polynomial trajectory with minimium duration with
/// given bounds
std::pair<double, CVecXd> P2PCubicTimeOpt(double q0, double q1, double v0,
                                          double v1, double v_max,
                                          double a_max);

///@brief Point to point quintic polynomial trajectory with minimium duration
/// with given bounds
std::pair<double, CVecXd> P2PQuinticTimeOpt(double q0, double q1, double v0,
                                            double v1, double a0, double a1,
                                            double v_max, double a_max,
                                            double j_max);

///@brief Compute time optimal cubic trajectory of multiple axes
std::pair<double, MatXd>
P2PCubicTimeOptMultiDim(const CVecXd &q0, const CVecXd &q1, const CVecXd &v0,
                        const CVecXd &v1, const CVecXd &v_max,
                        const CVecXd &a_max, const CVecXd &j_max);

///@brief Compute time optimal quintic trajectory of multiple axes
std::pair<double, MatXd>
P2PQuinticTimeOptMultiDim(const CVecXd &q0, const CVecXd &q1, const CVecXd &v0,
                          const CVecXd &v1, const CVecXd &a0, const CVecXd &a1,
                          const CVecXd &v_max, const CVecXd &a_max,
                          const CVecXd &j_max);

///@brief Wrapper of calculate time optimal quintic trajectory of multiple axes
std::shared_ptr<TrajectoryRnSpline> TrajectoryRnP2POptQuintic(
    const CVecXd &q0, const CVecXd &q1, const CVecXd &vel_limits,
    const CVecXd &acc_limits, const CVecXd &jerk_limits,
    const CVecXd &v0 = CVecXd::Zero(0), const CVecXd &v1 = CVecXd::Zero(0),
    const CVecXd &a0 = CVecXd::Zero(0), const CVecXd &a1 = CVecXd::Zero(0));

///@brief Wrapper of calculate time optimal cubic trajectory of multiple axes
std::shared_ptr<TrajectoryRnSpline>
TrajectoryRnP2POptCubic(const CVecXd &q0, const CVecXd &q1,
                        const CVecXd &vel_limits, const CVecXd &acc_limits,
                        const CVecXd &v0 = CVecXd::Zero(0),
                        const CVecXd &v1 = CVecXd::Zero(0));
///@}
} // namespace RVS