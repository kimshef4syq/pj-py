// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
// #include <ompl/base/StateSampler.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/OptimizationObjective.h>
#include <mutex>
RVS_COMMON_IGNORE_WARNINGS_POP
#include <RVS/Planner/Core/CostConstraintBase.h>
#include "WeightedRealVectorStateSpace.h"

namespace RVS
{
/// @addtogroup Planner
/// @{

/// @todo: StateSampler, GoalRegion, benchmarking tools etc.
// class StateSampler : public OB::StateSamper
// {
// };

/** @brief StateConstraint to ompl StateValidityChecker*/
class StateValidityChecker : public OB::StateValidityChecker
{
public:
    StateValidityChecker(const OB::SpaceInformationPtr &space_info,
                         StateConstraintPtr constraint);

    virtual bool isValid(const OB::State *state) const override;

private:
    StateConstraintPtr m_constraint;
};

/** @brief MotionConstraint to ompl MotionValidator*/
class MotionValidator : public OB::MotionValidator
{
public:
    MotionValidator(const OB::SpaceInformationPtr &space_info,
                    MotionConstraintPtr constraint);

    virtual ~MotionValidator() override = default;

    /** @brief Check if the path between two states (from \e s1 to \e s2) is
     * valid. This function assumes \e s1 is valid.
     *
     * @note This function updates the number of valid and invalid segments. */
    virtual bool checkMotion(const OB::State *s1,
                             const OB::State *s2) const override;

    /** @brief Check if the path between two states is valid. Also compute the
     * last state that was valid and the time of that state. The time is used to
     * parametrize the motion from \e s1 to \e s2, \e s1 being at t = 0 and \e
     * s2 being at t = 1. This function assumes \e s1 is valid.
     *
     * @param s1 start state of the motion to be checked (assumed to be valid)
     *
     * @param s2 final state of the motion to be checked
     *
     * @param lastValid
     *  - first: storage for the last valid state (may be nullptr, if the user
     * does not care about the exact state); this need not be different from \e
     * s1 or \e s2.
     *  - second: the time (between 0 and 1) of the last valid state, on the
     * motion from \e s1 to \e s2.
     *  - If the function returns false, \e
     * lastValid.first must be set to a valid state, even if that implies
     * copying \e s1 to \e lastValid.first (in case \e lastValid.second = 0).
     *  - If the function returns true, \e lastValid. first and \e lastValid.
     * second should \b not be modified.
     *
     * @note This function updates the number of valid and invalid segments.
     **/
    virtual bool
    checkMotion(const OB::State *s1, const OB::State *s2,
                std::pair<OB::State *, double> &lastValid) const override;

private:
    MotionConstraintPtr m_constraint;
};


class OptimizationObjective : public OB::OptimizationObjective
{
public:
    OptimizationObjective(const OB::SpaceInformationPtr &space_info,
                          StateCostPtr state_cost, MotionCostPtr motion_cost);

    OptimizationObjective(const OB::SpaceInformationPtr &space_info,
                          StateCostPtr state_cost);

    OptimizationObjective(const OB::SpaceInformationPtr &space_info,
                          MotionCostPtr motion_cost);

    virtual ~OptimizationObjective() override = default;

    /** @brief state cost */
    OB::Cost stateCost(const OB::State *s) const override;

    /** @brief Motion cost*/
    OB::Cost motionCost(const OB::State *s1,
                        const OB::State *s2) const override;

private:
    StateCostPtr m_state_cost;
    MotionCostPtr m_motion_cost;
};

/// @}
} // namespace RVS