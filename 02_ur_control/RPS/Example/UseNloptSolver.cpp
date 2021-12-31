/**
 * @example UseNloptSolver.cpp
 * @brief Demo of how to use NloptSolver to solve a OptModel.
 * @date 2021-08-19
 * 
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/OptModel/OptModel.h>
#include <RVS/OptSolver/NloptSolver.h>
#include <iostream>

using namespace RVS;
using VecBound = OptModel::VecBound;
using IdxVec = ConstraintSet::IdxVec;

bool GetConstraintValue(const double *x, unsigned, double *c, unsigned, void *,
                        const IdxVec &)
{
    c[0] = x[0] * x[1];
    c[1] = 3. * x[0] + x[1] * x[1];
    return true;
}

bool GetConstraintJacobian(const double *x, unsigned, double *jac, unsigned,
                           void *, const IdxVec &)
{
    jac[0] = x[1];
    jac[1] = x[0];
    jac[2] = 3.0;
    jac[3] = 2.0 * x[1];
    return true;
}

double GetCost(const double *x, unsigned, void *, const IdxVec &)
{
    return 2.0 * x[0] + 10. * x[0] * x[1] - x[1] * x[1] * x[1];
}

bool GetCostJacobian(const double *x, unsigned n, double *jac, void *,
                     bool accumulation, const IdxVec &)
{
    if (!accumulation) bzero(jac, sizeof(double) * n);
    jac[0] += 2.0 + 10.0 * x[1];
    jac[1] += 10. * x[0] - 3. * x[1] * x[1];
    return true;
}

int main()
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
    NloptSolver solver;
    std::vector<double> x0{9.9, 5.5};
    // solver.Initialize(x0, nlopt::GN_ISRES);
    solver.Initialize(x0, nlopt::AUGLAG_EQ);
    nlopt::opt opt_local(nlopt::algorithm::LD_MMA, x0.size());
    opt_local.set_xtol_rel(0.00001);
    opt_local.set_initial_step(0.01);
    solver.GetNloptApp()->set_local_optimizer(opt_local);
    // solver.Initialize(x0, nlopt::LD_MMA);
    // solver.Initialize(x0, nlopt::LD_SLSQP);
    // 3 . solve
    solver.GetNloptApp()->set_initial_step(0.001);
    solver.GetNloptApp()->set_xtol_rel(0.000001);
    solver.Solve(prob);

    auto opt_vars = prob->GetVariableSet()->GetValueVec();
    double obj_val = prob->GetCostValue(opt_vars.data(), opt_vars.size());
    PrintVec(opt_vars.data(), opt_vars.size(), "Optimized variables");
    printf("obj valve: %f\n", obj_val);
    return 0;
}
