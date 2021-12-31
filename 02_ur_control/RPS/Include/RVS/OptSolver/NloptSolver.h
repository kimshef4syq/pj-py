// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <nlopt.hpp>
#include "SolverBase.h"
#include <map>


namespace RVS
{
/// @addtogroup OptSolver
/// @{

RVS_CLASS_FORWARD(NloptSolver);

class NloptSolver : public SolverBase
{
public:
    NloptSolver() {}

    // About nlopt algorithms:
    // https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/
    bool Initialize(const std::vector<double> &x0, nlopt::algorithm method);

    virtual bool Solve(OptModelPtr prob) override;

    std::shared_ptr<nlopt::opt> GetNloptApp() const { return m_opt; }

private:
    std::shared_ptr<nlopt::opt> m_opt;
    std::vector<double> m_x0; ///< initialized variabled
    std::map<nlopt::result, std::string> m_result_info{
        {nlopt::result::FAILURE, "FAILURE"},
        {nlopt::result::INVALID_ARGS, "INVALID_ARGS"},
        {nlopt::result::OUT_OF_MEMORY, "OUT_OF_MEMORY"},
        {nlopt::result::ROUNDOFF_LIMITED, "ROUNDOFF_LIMITED"},
        {nlopt::result::FORCED_STOP, "FORCED_STOP"},
        {nlopt::result::SUCCESS, "SUCCESS"},
        {nlopt::result::STOPVAL_REACHED, "STOPVAL_REACHED"},
        {nlopt::result::FTOL_REACHED, "FTOL_REACHED"},
        {nlopt::result::XTOL_REACHED, "XTOL_REACHED"},
        {nlopt::result::MAXEVAL_REACHED, "MAXEVAL_REACHED"},
        {nlopt::result::MAXTIME_REACHED, "MAXTIME_REACHED"},

    };
};

/// @}
} // namespace RVS