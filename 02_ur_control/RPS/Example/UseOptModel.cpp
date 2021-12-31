/**
 * @example UseOptModel.cpp
 * @brief Demo of how to use OptModel to describe a optimization problem with various constraints.
 * @date 2021-08-19
 * 
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/OptModel/OptModel.h>
using namespace RVS;
using VecBound = OptModel::VecBound;
using IdxVec = ConstraintSet::IdxVec;

struct ExtraData
{
    double GetCostValue(const double *x, unsigned /*n*/,
                        [[maybe_unused]] const IdxVec &var_idx = {})
    {
        return 2.0 * x[0] + x[1] * x[0] * x[0];
    }

    bool GetCostJacobian(const double *x, unsigned n, double *jac,
                         bool accumulation = false,
                         [[maybe_unused]] const IdxVec &var_idx = {})
    {
        if (!accumulation) bzero(jac, sizeof(double) * n);
        jac[0] += 2.0 + 2.0 * x[0] * x[1];
        jac[1] += x[0] * x[0];
        return true;
    }

    bool GetConstraintValue(const double *x, unsigned /*n*/, double *c,
                            unsigned m,
                            [[maybe_unused]] const IdxVec &var_idx = {})
    {
        if (m) {
            RVS_ENSURE(m == 3, "m = {}", m);
        }
        c[0] = 2.5 - x[0] - x[1];
        c[1] = 1.0 - x[0] + x[1];
        c[2] = 3.3 - x[0] * x[1];
        return true;
    }
    bool GetConstraintJacobian(const double *x, unsigned /*n*/, double *jac,
                               unsigned m,
                               [[maybe_unused]] const IdxVec &var_idx = {})
    {
        if (m) {
            RVS_ENSURE(m == 3, "m = {}", m);
        }
        jac[0] = -1.0;
        jac[1] = -1.0;

        jac[2] = -1.0;
        jac[3] = 1.0;

        jac[4] = -x[1];
        jac[5] = -x[0];
        return true;
    }
};

void TestBase()
{
    ExtraData data;
    int n = 2, m = 3;
    VecBound var_bounds(n, Bounds(0.01, 100.0));
    VariableSet variables(n, var_bounds, "test_variables");
    VecBound cons_bounds(m, BoundSmallerZero);
    ConstraintSet constraints(m, cons_bounds, "constraints", &data);
    CostTerm cost("cost", &data);
    variables.Print();
    constraints.Print();
    cost.Print();
    double x[] = {1.1, 2.2};
    variables.SetVariables(x, 2);
    printf("cost values: %f\n", cost.GetValue(x, n));
    printf("cost values: %f\n", data.GetCostValue(x, n));
    double jac[2];
    cost.GetJacobian(x, n, jac);
    PrintVec(jac, 2, "cost jacobian");
    data.GetCostJacobian(x, 2, jac);
    PrintVec(jac, 2, "cost jacobian");

    double c[3] = {0.0, 0.0, 0.0};
    double c_jac[6];
    constraints.GetValue(x, n, c, m);
    PrintVec(c, m, "c");
    data.GetConstraintValue(x, n, c, m);
    PrintVec(c, m, "c");
    constraints.GetJacobian(x, n, c_jac, m);
    PrintVec(c_jac, 6, "c_jac");
    data.GetConstraintJacobian(x, n, c_jac, m);
    PrintVec(c_jac, 6, "c_jac");
}

void TestExBase()
{
    ExtraData data;
    int n = 2, m = 3;
    VecBound var_bounds(n, Bounds(0.01, 100.0));
    VariableSetPtr variables =
        std::make_shared<VariableSet>(n, var_bounds, "test_variables");

    VecBound cons_bounds(m, Bounds(1.0, 10.0));
    ExConstraintPtr constraints = std::make_shared<ExConstraint>(
        m, cons_bounds,
        [](const double *x, unsigned n, double *c, unsigned m, void *data,
           const IdxVec &var_idx) -> bool {
            ExtraData *ex_data = (ExtraData *)data;
            return ex_data->GetConstraintValue(x, n, c, m, var_idx);
        },
        nullptr, "constraint_set", &data);
    ExCostPtr cost = std::make_shared<ExCost>(
        [](const double *x, unsigned n, void *data,
           const IdxVec &var_idx) -> double {
            ExtraData *ex_data = (ExtraData *)data;
            return ex_data->GetCostValue(x, n, var_idx);
        },
        [](const double *x, unsigned n, double *jac, void *data,
           bool accumulation, const IdxVec &var_idx) -> bool {
            ExtraData *ex_data = (ExtraData *)data;
            return ex_data->GetCostJacobian(x, n, jac, accumulation, var_idx);
        },
        "cost", &data);
    variables->Print();
    constraints->Print();
    cost->Print();
    printf("========================\n");
    double x[] = {1.1, 2.2};
    variables->SetVariables(x, 2);
    printf("cost values: %f\n", cost->GetValue(x, n));
    printf("cost values: %f\n", data.GetCostValue(x, n));
    double jac[2];
    cost->GetJacobian(x, n, jac);
    PrintVec(jac, 2, "cost jacobian");
    data.GetCostJacobian(x, 2, jac);
    PrintVec(jac, 2, "cost jacobian");

    double c[3];
    double c_jac[6];
    constraints->GetValue(x, n, c, m);
    PrintVec(c, m, "constraint value");
    data.GetConstraintValue(x, n, c, m);
    PrintVec(c, m, "constraint value");
    constraints->GetJacobian(x, n, c_jac, m);
    PrintVec(c_jac, 6, "constraint jacobian");
    data.GetConstraintJacobian(x, n, c_jac, m);
    PrintVec(c_jac, 6, "constraint jacobian");

    double c_jac_can[12];
    double c_can[6];
    constraints->GetValueCanonicalForm(x, n, c_can, 2 * m);
    PrintVec(c_can, 2 * m, "constraint value canonical form");
    constraints->GetJacobianCanonicalForm(x, n, c_jac_can, 2 * m);
    PrintVec(c_jac_can, 12, "constraint jacobian canonical form");

    std::shared_ptr<OptModel> prob = std::make_shared<OptModel>();
    prob->SetVariableSet(variables);
    prob->AddCostTerm(cost);
    prob->AddCostTerm(cost);
    prob->AddConstraintSet(constraints);
    prob->AddConstraintSet(constraints);
    // prob->PrintCurrent();

    unsigned num_vars = prob->GetNumberOfVariables();
    unsigned num_cons = prob->GetNumberOfConstraints();
    double *vars = new double[num_vars];
    double *jac_cost = new double[num_vars];
    double *jac_cons = new double[num_vars * num_cons];
    double *cons_value = new double[num_cons];
    double *cons_value_cani = new double[num_cons * 2];
    double *jac_cons_cani = new double[num_vars * num_cons * 2];

    printf("========================\n");
    vars[0] = 1.1;
    vars[1] = 2.2;
    printf("cost value from prob: %f\n", prob->GetCostValue(vars, num_vars));
    prob->GetCostJacobian(vars, num_vars, jac_cost);
    PrintVec(jac_cost, num_vars, "cost jacobian from prob");

    prob->GetConstraintValue(vars, num_vars, cons_value, num_cons);
    PrintVec(cons_value, num_cons, "constraint value from prob");
    prob->GetConstraintJacobian(vars, num_vars, jac_cons, num_cons);
    PrintVec(jac_cons, num_cons * num_vars, "constraint jacobian from prob");

    prob->GetConstraintValueCanonicalForm(vars, num_vars, cons_value_cani,
                                          num_cons * 2);
    PrintVec(cons_value_cani, num_cons * 2,
             "constraint value canonical from prob");
    prob->GetConstraintJacobianCanonicalForm(vars, num_vars, jac_cons_cani,
                                             num_cons * 2);
    PrintVec(jac_cons_cani, num_cons * num_vars * 2,
             "constraint jacobian canonical from prob");

    delete (vars);
    delete (jac_cost);
    delete (jac_cons);
    delete (cons_value);
    delete (cons_value_cani);
    delete (jac_cons_cani);
}

int main()
{
    /* min. 2 * x0 + x1 * x0 * x0
    s.t. x0, x1 >= 0
    x0 + x1 >= 2.5
    x0 - x1 >= 1.0
    x0 * x1 >= 3.3
    */
    TestExBase();

    return 0;
}
