/**
 * @example UseIpoptSolver.cpp
 * @brief Demo of how to use IpoptSolver to solve an OptModel.
 * @date 2021-08-19
 * 
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/OptModel/OptModel.h>
#include <RVS/OptSolver/IpoptSolver.h>
#include <iostream>

using namespace RVS;
using VecBound = OptModel::VecBound;
using IdxVec = ConstraintSet::IdxVec;

bool GetConstraintValue(const double *x, unsigned /*n*/, double *c,
                        unsigned /*m*/, void * /*data*/,
                        [[maybe_unused]] const IdxVec &var_idx = {})
{
    c[0] = x[0] * x[1];
    c[1] = 3. * x[0] + x[1] * x[1];
    return true;
}

bool GetConstraintJacobian(const double *x, unsigned /*n*/, double *jac,
                           unsigned /*m*/, void * /*data*/,
                           [[maybe_unused]] const IdxVec &var_idx = {})
{
    RVS_INFO("Getting jacobian...");
    jac[0] = x[1];
    jac[1] = x[0];
    jac[2] = 3.0;
    jac[3] = 2.0 * x[1];
    return true;
}

double GetCost(const double *x, unsigned /*n*/, void * /*data*/,
               [[maybe_unused]] const IdxVec &var_idx = {})
{
    return 2.0 * x[0] + 10. * x[0] * x[1] - x[1] * x[1] * x[1];
}

bool GetCostJacobian(const double *x, unsigned /*n*/, double *jac, void *data,
                     bool accumulation = false,
                     [[maybe_unused]] const IdxVec &var_idx = {})
{
    RVS_UNUSED(data);
    if (!accumulation) {
        jac[0] = 2.0 + 10.0 * x[1];
        jac[1] = 10. * x[0] - 3. * x[1] * x[1];
    }
    else {
        jac[0] += 2.0 + 10.0 * x[1];
        jac[1] += 10. * x[0] - 3. * x[1] * x[1];
    }
    return true;
}

int main(int /*argnum*/, char ** /*args*/)
{
    spdlog::set_level(spdlog::level::debug);
    /*
    min. 2.0 * x0 + 10 *x0 * x1 - x1^3
    s.t.
    0 <= x0 <= 10
    0 <= x1 <= 10

    0 <= x0 * x1 <= 4.5
    0 <= 3 * x0 + x1 * x1 <= 20
    */
    // 1. define the problem
    OptModelPtr prob = std::make_shared<OptModel>();
    VecBound bounds(2, Bounds(0.0, 10.0));
    prob->SetVariableSet(std::make_shared<VariableSet>(2, bounds));
    ConstraintSetPtr constraint = std::make_shared<ExConstraint>(
        2, VecBound{Bounds(0, 4.5), Bounds(0, 20)}, GetConstraintValue,
        GetConstraintJacobian);
    prob->AddConstraintSet(constraint);
    CostTermPtr cost = std::make_shared<ExCost>(GetCost, GetCostJacobian);
    prob->AddCostTerm(cost);
    prob->PrintCurrent();

    // // 2. choose solver and options
    IpoptSolver solver;
    std::vector<double> x0{9.9, 5.5};
    solver.Initialize(x0);
    // solver.SetOption("jacobian_approximation", "finite-difference-values");
    solver.SetOption("jacobian_approximation", "exact");
    // 3 . solve
    solver.Solve(prob);

    auto opt_vars = prob->GetVariableSet()->GetValueVec();
    double obj_val = prob->GetCostValue(opt_vars.data(), opt_vars.size());
    PrintVec(opt_vars.data(), opt_vars.size(), "Optimized variables");
    printf("obj valve: %f\n", obj_val);
    return 0;
}
