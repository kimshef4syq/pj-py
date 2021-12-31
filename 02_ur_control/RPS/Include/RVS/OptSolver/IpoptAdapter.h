// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#define HAVE_CSTDDEF
#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <coin/IpTNLP.hpp>
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>
RVS_COMMON_IGNORE_WARNINGS_POP
#include <RVS/OptModel/OptModel.h>

/**
 * @brief namespace defined by the Ipopt solver.
 *
 * Sine this adapter wraps all of the Ipopt functions, we define the adapter
 * in this namespace as well.
 */
namespace Ipopt
{

/**
 * @brief Solves the optimization problem using the IPOPT solver.
 *
 * Given an optimization OptModel with variables, costs and constraints, this
 * class wraps it and makes it conform with the interface defined by IPOPT
 * https://projects.coin-or.org/Ipopt
 *
 * This implements the Adapter pattern. This class should not add any
 * functionality, but merely delegate it to the Adaptee (the OptModel object).
 */
class IpoptAdapter : public TNLP
{
public:
    using OptModelPtr = RVS::OptModelPtr;
    using VecBound = RVS::OptModel::VecBound;

    /**
     * @brief  Creates an IpoptAdapter wrapping the @a nlp.
     * @param  prob  The specific nonlinear programming problem.
     *
     * This constructor holds and modifies the passed nlp.
     */
    IpoptAdapter(OptModelPtr prob);
    virtual ~IpoptAdapter() = default;

private:
    OptModelPtr m_prob; ///< The solver independent problem definition

    /** Method to return some info about the nlp */
    virtual bool get_nlp_info(Index &n, Index &m, Index &nnz_jac_g,
                              Index &nnz_h_lag, IndexStyleEnum &index_style);

    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info(Index n, double *x_l, double *x_u, Index m,
                                 double *g_l, double *g_u);

    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(Index n, bool init_x, double *x,
                                    bool init_z, double *z_L, double *z_U,
                                    Index m, bool init_lambda, double *lambda);

    /** Method to return the objective value */
    virtual bool eval_f(Index n, const double *x, bool new_x,
                        double &obj_value);

    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(Index n, const double *x, bool new_x,
                             double *grad_f);

    /** Method to return the constraint residuals */
    virtual bool eval_g(Index n, const double *x, bool new_x, Index m,
                        double *g);

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    virtual bool eval_jac_g(Index n, const double *x, bool new_x, Index m,
                            Index nele_jac, Index *iRow, Index *jCol,
                            double *values);

    /** This is called after every iteration and is used to save intermediate
     *  solutions in the nlp */
    virtual bool intermediate_callback(
        AlgorithmMode mode, Index iter, double obj_value, double inf_pr,
        double inf_du, double mu, double d_norm, double regularization_size,
        double alpha_du, double alpha_pr, Index ls_trials,
        const IpoptData *ip_data, IpoptCalculatedQuantities *ip_cq);

    /** This method is called when the algorithm is complete so the TNLP can
     * store/write the solution */
    virtual void finalize_solution(SolverReturn status, Index n,
                                   const double *x, const double *z_L,
                                   const double *z_U, Index m, const double *g,
                                   const double *lambda, double obj_value,
                                   const IpoptData *ip_data,
                                   IpoptCalculatedQuantities *ip_cq);
};

} // namespace Ipopt