// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

// #include <Json11/json.hpp>
#include <RVS/Common/LoggerUtils.h>
#include "Types.h"

namespace RVS
{
/// @addtogroup Planner
/// @{

class MotionPlannerBase
{
public:
    explicit MotionPlannerBase(const std::string &name);

    explicit MotionPlannerBase(const ManipulatorVector &manipulators,
                               const std::shared_ptr<Environment> &env,
                               const std::string &name);

    explicit MotionPlannerBase(const std::shared_ptr<Manipulator> &manipulator,
                               const std::shared_ptr<Environment> &env,
                               const std::string &name);

    virtual ~MotionPlannerBase() = default;

    /**
     * @brief Get the planner name
     *
     * @return const std::string&
     */
    const std::string &GetName() const { return m_name; }

    /**
     * @brief Check if the planner parameters are configured properly.
     * @return RVSReturn_Success if configured, RVSReturn_NotInitialized
     * otherwise
     */
    virtual RVSReturn IsConfigured() const = 0;

    /**
     * @brief Solve planning problem.
     *
     * @param request planning request
     * @param response response The results from the planner
     * @param verbose Flag for printing more detailed planning information,
     * default is false
     * @return true
     * @return false
     */
    virtual bool Solve(const MotionPlannerRequest &request,
                       MotionPlannerResponse &response,
                       const bool verbose = false) = 0;

    /**
     * @brief If Solve() is running, terminate the computation. Return false if
     * termination not possible. No-op if Solve() is not running (returns true).
     */
    virtual bool Terminate() = 0;

    /** @brief Clear the data structures used by the planner */
    virtual void Clear() = 0;

protected:
    std::string m_name;
    ManipulatorVector m_manipulators;
    std::shared_ptr<Environment> m_env;
};

/// @}
} // namespace RVS