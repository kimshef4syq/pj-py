// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <memory>
#include <RVS/OptModel/OptModel.h>

namespace RVS
{
/// @addtogroup OptSolver
/// @{

RVS_CLASS_FORWARD(SolverBase);

/**
 * @brief Base solver interface, all other solver wrapper should inherit from
 * this.
 *
 */
class SolverBase : public std::enable_shared_from_this<SolverBase>
{
public:
    virtual ~SolverBase() = default;

    /** @brief  Uses a specific solver eg. IPOPT/CeresSolver to solve the NLP.
     * @param [in/out]  prob  The nonlinear programming problem.
     * @return true: success
     *         false: failure
     */
    virtual bool Solve(OptModelPtr prob) = 0;
};

/// @}
} /* namespace RVS */
