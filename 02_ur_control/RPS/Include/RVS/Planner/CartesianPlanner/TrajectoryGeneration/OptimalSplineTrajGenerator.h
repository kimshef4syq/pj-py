// Copyright (c) RVBUST, Inc - All rights reserved
#pragma once
#include <RVS/Planner/CartesianPlanner/TrajectoryGeneration/BezierQP.h>
#include <RVS/Trajectory/TrajectorySpline.h>
#include <RVS/Planner/CartesianPlanner/Utils.h>

namespace RVS
{

///@addtogroup Planner
///@{

/** @brief
 *  this function is provided to find a polynomial trajectory
    that minimize the integrated jerk of the overall trajectory
    within the velocity and acceleration limits
    defined in kin_solver.

    procedures are as follows:
    1. Given the Number of waypoints: N

        Get the order of the polynomial traj: M

    Define the variable set which represents the coeffs of the polynomial
       for computational considerations, we represent the coeffs in terms of
           the control pooints of bezier curves
        we have (N - 1) segments of polynomial
        of each has (M + 1) control points
        thus, the variable set has a dimension of (N - 1) * (M + 1)

    2. Define the cost function
        find the analytical representation of the 3rd-derivatives of the traj \
            which can be writen as integral{q'''(t) dt} = C^T * x * C (QP form)

    3. Define the constraint
        which can be written as linear inequalities or linear equalities

    4. Solve the problem by calling a QP solver (Sure! in polynomial time)

    5. constrcut a spline trajectory by the solved coeffients
 *
 */
std::pair<bool, std::shared_ptr<TrajectoryRnSpline>>
CalcPolyTrajectory(const MatXd &qs, const CVecXd &ts, const size_t poly_order,
                   const size_t minimize_order, const CVecXd &v_max = CVecXd(),
                   const CVecXd &a_max = CVecXd(),
                   const CVecXd &j_max = CVecXd(), const bool verbose = false,
                   const CVecXd &v0 = CVecXd(), const CVecXd &a0 = CVecXd(),
                   const CVecXd &v1 = CVecXd(), const CVecXd &a1 = CVecXd());


/** @brief
 *  this function is provided to find a polynomial trajectory
    that minimize the integrated jerk of the overall trajectory
    within the velocity and acceleration limits
    defined in kin_solver.

    procedures are as follows:
    1. Given the Number of waypoints: N

        Get the order of the polynomial traj: M

    Define the variable set which represents the coeffs of the polynomial
       for computational considerations, we represent the coeffs in terms of
           the control pooints of bezier curves
        we have (N - 1) segments of polynomial
        of each has (M + 1) control points
        thus, the variable set has a dimension of (N - 1) * (M + 1)

    2. Define the cost function
        find the analytical representation of the 3rd-derivatives of the traj \
            which can be writen as integral{q'''(t) dt} = C^T * x * C (QP form)

    3. Define the constraint
        which can be written as linear inequalities or linear equalities

    4. Solve the problem by calling a QP solver (Sure! in polynomial time)

    5. constrcut a spline trajectory by the solved coeffients
 *
 */
std::pair<bool, std::shared_ptr<TrajectoryRnSpline>> CalcPolyTrajectory(
    const std::vector<JointVector> &qlist, const std::vector<double> &tlist,
    const size_t poly_order, const size_t minimize_order,
    const std::vector<JointLimit> &joint_limits, const bool verbose = false);

///@}
} // namespace RVS
