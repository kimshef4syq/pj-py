// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/Types.h>
#include <RVS/Common/LoggerUtils.h>
#include <RVS/Common/Macros.h>

namespace RVS
{
/// @addtogroup OptModel
/// @{

void PrintVec(const double *x, const unsigned n, const char *name = "");

/**
 * @brief Variable bounds
 *
 */
struct Bounds
{
    Bounds(const double lower = 0.0, const double upper = 0.0)
    {
        this->lower = lower <= upper ? lower : upper;
        this->upper = lower <= upper ? upper : lower;
    }

    double lower;
    double upper;
};

static const double inf = std::numeric_limits<double>::infinity();
static const Bounds NoBound = Bounds(-inf, +inf);
static const Bounds BoundZero = Bounds(0.0, 0.0);
static const Bounds BoundGreaterZero = Bounds(0.0, +inf);
static const Bounds BoundSmallerZero = Bounds(-inf, 0.0);


RVS_CLASS_FORWARD(VariableSet);

/**
 * @brief Variable set
 *
 */
class VariableSet
{
public:
    using VecBound = std::vector<Bounds>;

    /**
     * @brief Creates a set of variables representing a single concept.
     * @param num_vars  Number of variables.
     * @param name   What the variables represent to (e.g. "spline
     * coefficients").
     */
    VariableSet(unsigned num_vars, const std::vector<Bounds> &bounds,
                const std::string &name = "variables");

    ~VariableSet() {}

    const double *GetValuePtr() const { return m_x.data(); }

    const std::vector<double> GetValueVec() const { return m_x; }

    /**
     * @brief Get the current variable values
     *
     * @param x[out] : destination variables pointer
     * @param n[in] : variables number
     * @return true
     * @return false
     */
    bool GetValue(double *x, unsigned n = 0) const;

    /**
     * @brief Set the current variables
     *
     * @param x[in] : source variables pointer
     * @param n[in] : variables number
     * @return true
     * @return false
     */
    bool SetVariables(const double *x, unsigned n = 0);

    ///@brief Get current lower and upper bounds of variables
    inline const VecBound &GetBounds() const { return m_bounds; }

    unsigned GetNumberOfVariables() const { return m_num_vars; }

    void Print() const;

    inline const std::string &GetName() const { return m_name; }

private:
    unsigned m_num_vars; ///< number of variables
    std::vector<Bounds> m_bounds;
    std::string m_name; ///< variable set name
    std::vector<double> m_x; ///< variables, allocate when constrcting,
                             ///< deallocate when descontructing
    unsigned m_num_bytes; ///< number of bytes of variables, sizeof(double) *
                          ///< m_num_vars
};


RVS_CLASS_FORWARD(ConstraintSet);

/**
 * @brief Constraints
 *
 */
class ConstraintSet : public std::enable_shared_from_this<ConstraintSet>
{
public:
    using VecBound = std::vector<Bounds>;
    using IdxVec = std::vector<unsigned>;

    /**
     * @brief Creates constraints with
     * normal form: lb_i <= g_i(x) <= ub_i, i = 0, 1, 2, ...
     *
     * @param num_constraints  The number of constraints.
     * @param name  What these constraints represent.
     * @param extra_data  extra data to compute constraint value / jacobian
     * @param variable_idx  indexes of link variables for these constraints
     */
    ConstraintSet(unsigned num_constraints, const std::vector<Bounds> &bounds,
                  const std::string &name = "constraints",
                  void *extra_data = nullptr, const IdxVec &variable_idx = {});

    virtual ~ConstraintSet() {}

    bool IsSatisfied(const double *x, unsigned n,
                     double tolerance = Constants<double>::Small()) const;

    inline const IdxVec &GetVariableIdx() const { return m_variable_idx; }

    inline void SetDifferenceValue(const double dx) { m_dx = std::abs(dx); }

    /**
     * @brief Get the constraint values according to current variable states
     * normal form: lb_i <= g_i(x) <= ub_i, i = 0, 1, 2, ...
     *
     * @param x[in] : varialbes pointer
     * @param n[in] : varialbes number
     * @param c[out] : constraint values, dimension: num_constrains
     * @param m[in] : constraint number, if m = 0, use internal data without
     * checking
     * @return true
     * @return false
     */
    virtual bool GetValue(const double * /*x*/, unsigned /*n*/, double * /*c*/,
                          unsigned /*m*/ = 0) const;

    /**
     * @brief Get the constraint values according to current variable states
     * Canonical form: g_i(x) <= 0, i = 0, 1, 2, 3, ....
     *
     * @param x[in] : varialbes pointer
     * @param n[in] : varialbes number
     * @param c[out] : constraint values, dimension: num_constrains x 2
     * @param m[in] : constraint number, if m = 0, use internal data without
     * checking
     * @return true
     * @return false
     */
    bool GetValueCanonicalForm(const double *x, unsigned n, double *c,
                               unsigned m = 0) const;

    /**
     * @brief  The matrix of derivatives for these constraints and variables.
     * normal form: lb_i <= g_i(x) <= ub_i, i = 0, 1, 2, ...
     *
     * @param x[in] : variables pointer
     * @param n[in] : variables number
     * @param jac[out] : jacobian pointer, dimension: m x n; m is number of
     * constrains, n is number of variables
     * @param m[in] : constraint number, if m = 0, use internal data without
     * checking
     * @return true
     * @return false
     */
    virtual bool GetJacobian(const double *x, unsigned n, double *jac,
                             unsigned m = 0) const;

    /**
     * @brief  The matrix of derivatives for these constraints and variables.
     * Canonical form: g_i(x) <= 0, i = 0, 1, 2, 3, ....
     *
     * @param x[in] : variables pointer
     * @param n[in] : variables number
     * @param jac[out] : jacobian pointer, dimension: m x n; m is number of
     * constrains, n is number of variables
     * @param m[in] : constraint number, if m = 0, use internal data without
     * checking
     * @return true
     * @return false
     */
    virtual bool GetJacobianCanonicalForm(const double *x, unsigned n,
                                          double *jac, unsigned m = 0) const;

    inline const VecBound &GetBounds() const { return m_bounds; }

    inline unsigned GetNumberOfConstraints() const { return m_num_constrains; }

    void Print() const;

    inline const std::string &GetName() const { return m_name; }

protected:
    unsigned m_num_constrains; ///< number of constraints
    std::vector<Bounds> m_bounds; ///< constarint lower/upper bounds
    std::string m_name;
    void *m_data; ///< extra data to compute constraint values / jacobian
    const IdxVec m_variable_idx; ///< indexes of link variables for this
                                 ///< ConstraintSet
    mutable std::vector<double> m_x_copy;
    mutable std::vector<double> m_c0;
    mutable std::vector<double> m_c1;
    double m_dx; ///< diffrenec value to compute jacobian
};

RVS_CLASS_FORWARD(CostTerm);

class CostTerm : public std::enable_shared_from_this<CostTerm>
{
public:
    using IdxVec = std::vector<unsigned>;

    /**
     * @brief Construct a new Cost Term object
     *
     * @param name
     * @param extra_data : extra data used to compute cost value / jacobian
     * @param variable_idx  indexes of link variables for this cost
     */
    CostTerm(const std::string &name = "cost", void *extra_data = nullptr,
             const IdxVec &variable_idx = {});

    virtual ~CostTerm() {}

    inline void SetDifferenceValue(const double dx) { m_dx = std::abs(dx); }

    inline const IdxVec &GetVariableIdx() const { return m_variable_idx; }

    /**
     * @brief Get the Cost according to current variable state
     *
     * @param x[in] : variables ptr
     * @param n[in] : variables number
     * @param new_x[in] : variables are different from last call. Sometimes cost
     * value will be cached and if new_x is false, last cost value will be
     * returned to improve effeciency; if new_x is true, a new compution will be
     * done
     * @return double
     */
    virtual double GetValue(const double * /*x*/, unsigned /*n*/,
                            const bool new_x = true) const;

    /**
     * @brief Get the Cost Jacobian
     *
     * @param x[in] : variables ptr
     * @param n[in] : variables number
     * @param jac[out] : jacobian ptr, an array of n double
     * @param accumulation[in] : whether accumulation jacobian value to input
     * jac, or just set it, eg. `jac[0] = val` or `jac[0] += val`. By passing
     * `accumulation = true`, it's more effenciency to combine several
     * cost-terms' jacobian value.
     * @return true
     * @return false
     */
    virtual bool GetJacobian(const double *x, unsigned n, double *jac,
                             bool accumulation = false) const;

    void Print() const;

    inline const std::string &GetName() const { return m_name; }

protected:
    std::string m_name;
    void *m_data; ///< extra data to compute cost value / jacobian
    const IdxVec m_variable_idx; ///< indexes of link variables for this
                                 ///< CostTerm
    mutable double m_last_cost;
    mutable std::vector<double> m_x_copy;
    double m_dx; ///< difference value to compute jacobian
};

/// @}
} // namespace RVS