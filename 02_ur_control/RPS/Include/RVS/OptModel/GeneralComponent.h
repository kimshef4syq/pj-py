// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "ComponentBase.h"
#include <functional>
#include <iostream>

namespace RVS
{
/// @addtogroup OptModel
/// @{

RVS_CLASS_FORWARD(ExConstraint);

class ExConstraint : public ConstraintSet
{
    using Super = ConstraintSet;

public:
    /**
     * @brief Construct a new Ex Constraint object
     *
     * @param num_constraints
     * @param bounds
     * @param get_constraint_values : function to get constraint value
     * @param get_jacobian : function to get constraint jacobian, if nullptr, a
     * finite difference method will be used
     * @param name
     * @param extra_data : extra data to computing constraint value / jacobian
     */
    inline ExConstraint(
        unsigned num_constraints, const VecBound &bounds,
        const std::function<bool(const double *, unsigned, double *, unsigned,
                                 void *, const IdxVec &)>
            &get_constraint_values,
        const std::function<bool(const double *, unsigned, double *, unsigned,
                                 void *, const IdxVec &)> &get_jacobian =
            nullptr,
        const std::string &name = "constraints", void *extra_data = nullptr,
        const IdxVec &variable_idx = {})
        : ConstraintSet(num_constraints, bounds, name, extra_data,
                        variable_idx),
          m_get_values(get_constraint_values), m_get_jacobian(get_jacobian)
    {
    }
    inline virtual bool GetValue(const double *x, unsigned n, double *c,
                                 unsigned m = 0) const override
    {
        if (m == 0) m = m_num_constrains;
        return m_get_values(x, n, c, m, m_data, m_variable_idx);
    }
    inline virtual bool GetJacobian(const double *x, unsigned n, double *jac,
                                    unsigned m = 0) const override
    {
        if (m == 0) m = m_num_constrains;
        if (m_get_jacobian) {
            return m_get_jacobian(x, n, jac, m, m_data, m_variable_idx);
        }
        else {
            return Super::GetJacobian(x, n, jac, m);
        }
    }

private:
    const std::function<bool(const double *, unsigned, double *, unsigned,
                             void *, const IdxVec &)>
        m_get_values; ///< args: x   n  c    m   data idx
    const std::function<bool(const double *, unsigned, double *, unsigned,
                             void *, const IdxVec &)>
        m_get_jacobian; ///< args: x   n  jac  m   data idx
};

class CostNormAbs : public CostTerm
{
    using Super = CostTerm;

public:
    /**
     * @brief Construct a new CostTerm, the cost value is (|x0| + |x1| + ... ) *
     * weight / n
     */
    inline CostNormAbs(const double weight = 0.1,
                       const std::string &name = "cost_norm_abs",
                       const IdxVec &variable_idx = {})
        : CostTerm(name, nullptr, variable_idx), m_weight(weight)
    {
    }

    inline virtual double GetValue(const double *x, unsigned n,
                                   const bool new_x = true) const override
    {
        if (new_x) {
            m_last_cost = 0.0;
            if (m_variable_idx.size()) {
                for (auto idx : m_variable_idx) {
                    m_last_cost += std::abs(x[idx]);
                }
                n = m_variable_idx.size();
            }
            else {
                for (unsigned i = 0; i < n; ++i) {
                    m_last_cost += std::abs(x[i]);
                }
            }
            m_last_cost *= m_weight;
            m_last_cost /= n;
        }
        return m_last_cost;
    }
    inline virtual bool GetJacobian(const double *x, unsigned n, double *jac,
                                    bool accumulation = false) const override
    {
        if (!accumulation) bzero(jac, sizeof(double) * n);
        if (m_variable_idx.size() != 0) {
            double v = m_weight / m_variable_idx.size();
            for (auto idx : m_variable_idx) {
                jac[idx] += x[idx] >= 0 ? v : -v;
            }
        }
        else {
            double v = m_weight / n;
            for (unsigned i = 0; i < n; ++i) {
                jac[i] += x[i] >= 0 ? v : -v;
            }
        }
        return true;
    }

private:
    double m_weight;
};

class CostNormSquare : public CostTerm
{
    using Super = CostTerm;

public:
    /**
     * @brief Construct a new CostTerm, the cost value is (x0 * x0 + x1 * x1 +
     * ... ) * weight / n
     */
    inline CostNormSquare(double weight = 0.1,
                          const std::string &name = "cost_norm_square",
                          const IdxVec &variable_idx = {})
        : CostTerm(name, nullptr, variable_idx), m_weight(weight)
    {
    }

    inline virtual double GetValue(const double *x, unsigned n,
                                   const bool new_x = true) const override
    {
        if (new_x) {
            m_last_cost = 0.0;
            if (m_variable_idx.size() != 0) {
                for (auto idx : m_variable_idx) {
                    m_last_cost += x[idx] * x[idx];
                }
                n = m_variable_idx.size();
            }
            else {
                for (unsigned i = 0; i < n; ++i) {
                    m_last_cost += x[i] * x[i];
                }
            }
            m_last_cost *= m_weight;
            m_last_cost /= n;
        }
        return m_last_cost;
    }
    inline virtual bool GetJacobian(const double *x, unsigned n, double *jac,
                                    bool accumulation = false) const override
    {
        if (!accumulation) bzero(jac, sizeof(double) * n);
        if (m_variable_idx.size() != 0) {
            double r = 2.0 * m_weight / m_variable_idx.size();
            for (auto idx : m_variable_idx) {
                jac[idx] += r * x[idx];
            }
        }
        else {
            double r = 2.0 * m_weight / n;
            for (unsigned i = 0; i < n; ++i) {
                jac[i] += r * x[i];
            }
        }
        return true;
    }

private:
    double m_weight;
};

RVS_CLASS_FORWARD(ExCost);

class ExCost : public CostTerm
{
    using Super = CostTerm;

public:
    /**
     * @brief Construct a new Ex Cost object
     *
     * @param get_cost
     * @param get_jacobian : funciton to get cost jacobian, if nullptr, a finite
     * difference method will be used
     * @param name
     * @param extra_data
     */
    inline ExCost(
        const std::function<double(const double *, unsigned, void *,
                                   const IdxVec &)> &get_cost,
        const std::function<bool(const double *, unsigned, double *, void *,
                                 bool, const IdxVec &)> &get_jacobian = nullptr,
        const std::string &name = "cost", void *extra_data = nullptr,
        const IdxVec &variable_idx = {})
        : CostTerm(name, extra_data, variable_idx), m_get_cost(get_cost),
          m_get_jacobian(get_jacobian)
    {
    }

    inline virtual double GetValue(const double *x, unsigned n,
                                   const bool new_x = true) const override
    {
        if (new_x) {
            m_last_cost = m_get_cost(x, n, m_data, m_variable_idx);
        }
        return m_last_cost;
    }

    inline virtual bool GetJacobian(const double *x, unsigned n, double *jac,
                                    bool accumulation = false) const override
    {
        if (m_get_jacobian) {
            return m_get_jacobian(x, n, jac, m_data, accumulation,
                                  m_variable_idx);
        }
        else {
            return Super::GetJacobian(x, n, jac, accumulation);
        }
    }

private:
    const std::function<double(const double *, unsigned, void *,
                               const IdxVec &)>
        m_get_cost; ///< args: x, n, data, idx
    const std::function<bool(const double *, unsigned, double *, void *, bool,
                             const IdxVec &)>
        m_get_jacobian; ///< args: x, n, jac, data, accumulation, idx
};

/// @}
} // namespace RVS