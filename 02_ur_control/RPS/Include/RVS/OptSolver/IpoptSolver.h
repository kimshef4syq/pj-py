// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/OptModel/OptModel.h>
#include "SolverBase.h"
#include "IpoptAdapter.h"
#include <map>

namespace RVS
{
/// @addtogroup OptSolver
/// @{

RVS_CLASS_FORWARD(IpoptSolver);

/**
 * @brief An interface to IPOPT, fully hiding its implementation.
 *
 * To set specific options, see:
 * https://www.coin-or.org/Ipopt/documentation/node40.html
 *
 * @ingroup Solvers
 */
class IpoptSolver : public SolverBase
{
public:
    IpoptSolver();
    virtual ~IpoptSolver(){};

    bool Initialize(const std::vector<double> &x0);

    /** @brief  Creates an IpoptAdapter and solves the NLP.
     * @param [in/out]  prob  The specific problem.
     */
    virtual bool Solve(OptModelPtr prob) override;

    /** Set options for the IPOPT solver. A complete list can be found here:
     * https://coin-or.github.io/Ipopt/OPTIONS.html
     */
    void SetOption(const std::string &name, const std::string &value);
    void SetOption(const std::string &name, int value);
    void SetOption(const std::string &name, double value);

    /** @brief  Get the total wall clock time for the optimization, including
     * function evaluations.
     */
    double GetTotalWallclockTime();

    /** @brief  Get the return status for the optimization.
     */
    int GetReturnStatus();

private:
    std::shared_ptr<Ipopt::IpoptApplication> m_ipopt_app;
    int m_status;
    std::vector<double> m_x0;
    std::map<int, std::string> m_result_info{
        {0, "Solve_Succeeded"},
        {1, "Solved_To_Acceptable_Level"},
        {2, "Infeasible_OptModel_Detected"},
        {3, "Search_Direction_Becomes_Too_Small"},
        {4, "Diverging_Iterates"},
        {5, "User_Requested_Stop"},
        {6, "Feasible_Point_Found"},
        {-1, "Maximum_Iterations_Exceeded"},
        {-2, "Restoration_Failed"},
        {-3, "Error_In_Step_Computation"},
        {-4, "Maximum_CpuTime_Exceeded"},
        {-10, "Not_Enough_Degrees_Of_Freedom"},
        {-11, "Invalid_OptModel_Definition"},
        {-12, "Invalid_Option"},
        {-13, "Invalid_Number_Detected"},
        {-100, "Unrecoverable_Exception"},
        {-101, "NonIpopt_Exception_Thrown"},
        {-102, "Insufficient_Memory"},
        {-199, "Internal_Error"},
    };
};

/// @}
} // namespace RVS
