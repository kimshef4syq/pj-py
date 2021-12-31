// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "GeneralComponent.h"
#include <deque>

namespace RVS
{
/// @addtogroup OptModel
/// @{

/**
 * @brief The elements to formulate the solver independent optimization problem.
 *
 * An optimization problem usually consists of multiple sets of independent
 * variable- or constraint-sets. Each set represents a common concept, e.g. one
 * set of variables represents spline coefficients, another footstep positions.
 * Similarly, each constraint-set groups a set of similar constraints.
 *
 * The Nonlinear Optimization OptModel to solve is defined as:
 *
 *     find x0, x1                              (variable-sets 0 & 1)
 *     s.t
 *       x0_lower  <= x0 <= x0_upper            (bounds on variable-set)
 *
 *       {x0,x1} = arg min c0(x0,x1)+c1(x0,x1)  (cost-terms 0 and 1)
 *
 *       g0_lower < g0(x0,x1) < g0_upper        (constraint-set 0)
 *       g1_lower < g1(x0,x1) < g0_upper        (constraint-set 1)
 *
 */

RVS_CLASS_FORWARD(OptModel);

/**
 * @brief A generic optimization problem with variables, costs and constraints.
 *
 * This class is responsible for holding all the information of an optimization
 * problem, which includes the optimization variables, their variable bounds,
 * the cost function, the constraints and their bounds and derivatives of
 * all. With this information the problem can be solved by any specific solver.
 * All the quantities (variables, cost, constraint) are represented
 * by the same generic Component class.
 *
 */
class OptModel : public std::enable_shared_from_this<OptModel>
{
public:
    using VecBound = VariableSet::VecBound;

    /**
     * @brief  Creates a optimization problem with no variables, costs or
     * constraints.
     */
    OptModel();

    virtual ~OptModel() = default;

    inline unsigned GetNumberOfVariables() const { return m_n; }
    inline unsigned GetNumberOfConstraints() const { return m_m; }

    /**
     * @brief Set set of variables to the optimization problem.
     * @param variable_set  selection of optimization variables.
     */
    void SetVariableSet(VariableSetPtr variable_set);

    inline VecBound GetVariableBounds() const
    {
        return GetVariableSet()->GetBounds();
    }

    ///@brief Read/write access to the current optimization variables.
    VariableSetPtr GetVariableSet() const;

    /**
     * @brief Add a set of multiple constraints to the optimization problem.
     * @param constraint_set  This can be 1 to infinity number of constraints.
     *
     * This function can be called multiple times for different sets of
     * constraints. It makes sure the overall constraint and Jacobian correctly
     * considers all individual constraint sets.
     */
    void AddConstraintSet(ConstraintSetPtr constraint_set);

    bool GetConstraintValue(const double *x, unsigned n, double *c, unsigned m);

    bool GetConstraintValueCanonicalForm(const double *x, unsigned n, double *c,
                                         unsigned m);

    bool GetConstraintJacobian(const double *x, unsigned n, double *jac,
                               unsigned m);

    bool GetConstraintJacobianCanonicalForm(const double *x, unsigned n,
                                            double *jac, unsigned m);

    VecBound GetConstraintBounds() const;

    inline std::vector<ConstraintSetPtr> GetConstraintSets()
    {
        return m_constraints;
    }

    /**
     * @brief Add a cost term to the optimization problem.
     * @param cost_set  The calculation of the cost from the variables.
     *
     * This function can be called multiple times if the cost function is
     * composed of different cost terms. It makes sure the overall value and
     * gradient is considering each individual cost.
     */
    void AddCostTerm(CostTermPtr cost_set);

    double GetCostValue(const double *x, unsigned n, bool new_x = true);

    inline std::vector<CostTermPtr> GetCostTerms() { return m_costs; }

    /**
     * @brief Get the Cost Jacobian object
     *
     * @param x
     * @param n
     * @param jac
     * @return true
     * @return false
     */
    bool GetCostJacobian(const double *x, unsigned n, double *jac);

    /**
     * @brief Saves the current values of the optimization variables in x_prev.
     *
     * This is used to keep a history of the values for each NLP iterations.
     */
    void SaveCurrent();

    /**
     * @brief Sets the optimization variables to those at iteration iter.
     */
    void SetOptVariables(int iter);

    /**
     * @brief Sets the optimization variables to those of the final iteration.
     */
    void SetOptVariablesFinal();

    /**
     * @brief The number of iterations it took to solve the problem.
     */
    int GetIterationCount() const
    {
        return m_opt_variable_history.size()
               + m_opt_variable_history_start_iter;
    };

    /**
     * @brief Prints the variables, costs and constraints.
     */
    void PrintCurrent() const;

private:
    VariableSetPtr m_variable_set;
    std::vector<ConstraintSetPtr> m_constraints;
    std::vector<CostTermPtr> m_costs;
    std::deque<std::vector<double>>
        m_opt_variable_history; ///< the optimal variables for every iteration.
    int m_opt_variable_history_start_iter;
    unsigned m_n; ///< number of total varialbes
    unsigned m_m; ///< number of total constraints
};

/// @}
} // namespace RVS
