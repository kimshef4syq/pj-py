// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/Eigen.h>
#include <Eigen/Sparse>


namespace RVS
{
namespace QP
{
typedef struct
{
    size_t max_iter = 4000;

    double time_limit = 0;

    double eps_abs = 1e-3;

    double eps_rel = 1e-3;

    bool warm_start = true;

    bool verbose = false;

} QPSettings;

/**
 * @brief solve a unconstrained QP problem
 * here is the problem definition
 * min        1/2 x^T P x +  q^T x
 *
 * @param[in] P: as problem definition
 * @param[in] q: as problem definition
 * @param[out] sol: output solution: x
 * @return bool: true if success, else false
 */
RVSReturn SolveQPUnconstrained(const MatXd &P, const CVecXd &q, CVecXd &sol);


/**
 * @brief solve a QP problem in osqp form
 * here is the problem definition
 * min        1/2 x^T P x +  q^T x
 * s.t.        lu  <= A x <=  ub
 *
 * @param[in] P: as problem definition
 * @param[in] q: as problem definition
 * @param[in] A: as problem definition
 * @param[in] b: as problem definition
 * @param[in] lb: as problem definition
 * @param[in] ub: as problem definition
 * @param[out] sol: output solution: x
 * @return bool: true if success, else false
 *
 */
RVSReturn SolveQP(const MatXd &P, const CVecXd &q, CVecXd &sol,
                  const MatXd &A = MatXd(), const CVecXd &lb = CVecXd(),
                  const CVecXd &ub = CVecXd(),
                  const QPSettings settings = QPSettings());


/**
 * @brief solve a QP problem in canonical-form
 * here is the problem definition
 * min        1/2 x^T P x +  q^T x
 * s.t.           A x =  b
 *          h1 <= G x <= h2
 *            lb <= x <= ub
 *
 * @param[in] P: as problem definition
 * @param[in] q: as problem definition
 * @param[in] A: as problem definition
 * @param[in] b: as problem definition
 * @param[in] G: as problem definition
 * @param[in] h1: as problem definition
 * @param[in] h2: as problem definition
 * @param[in] lb: as problem definition
 * @param[in] ub: as problem definition
 * @param[out] sol: output solution: x
 * @return bool: true if success, else false
 *
 */
RVSReturn
SolveQPCanonicalForm(const MatXd &P, const CVecXd &q, CVecXd &sol,
                     const MatXd &A = MatXd(), const CVecXd &b = CVecXd(),
                     const MatXd &G = MatXd(), const CVecXd &h1 = CVecXd(),
                     const CVecXd &h2 = CVecXd(), const CVecXd &lb = CVecXd(),
                     const CVecXd &ub = CVecXd(),
                     const QPSettings settings = QPSettings());

} // namespace QP
} // namespace RVS