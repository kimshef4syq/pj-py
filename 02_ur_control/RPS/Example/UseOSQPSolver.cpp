/**
 * @file UseOSQPSolver.cpp
 * @brief Demo of use osqp solver.
 * @date 2021-08-19
 *
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/OptSolver/OSQPAdapter.h>
#include <RVS/Common/LoggerUtils.h>

using namespace RVS;

int main()
{
    auto logger = RVS::Logger::GetConsoleLogger("RVS_QSQP");
    Logger::SetLevelForConsole(logger->GetName(), 2);

    MatXd P(2, 2), A(3, 2);
    CVecXd q(2), lb(3), ub(3);

    // clang-format off
    P <<     4, 1,
             1, 2;

    q <<     1, 
             1;

    A <<     1, 1, 
             1, 0, 
             0, 1;

    lb <<    1, 
             0, 
             0;

    ub <<    1, 
             0.7, 
             0.7;

    // clang-format on

    QP::QPSettings settings;
    settings.time_limit = 2;
    settings.warm_start = false;
    settings.eps_rel = 1e-3;


    CVecXd sol;
    // solving QP in osqp default form
    QP::SolveQP(P, q, sol, A, lb, ub);
    RVS_INFO("optimal :\n{}", sol);

    // solving unconstrained QP
    QP::SolveQP(P, q, sol);
    RVS_INFO("optimal in unconstrained case: \n{}", sol);
}