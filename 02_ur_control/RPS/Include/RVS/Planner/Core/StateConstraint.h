// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/OptModel/ComponentBase.h>
#include <RVS/OptModel/GeneralComponent.h>
#include <RVS/EnvironmentHeader.h>
#include <RVS/KinematicsHeader.h>
#include <RVS/CollisionChecker/FCLCollisionChecker/FCLCollisionChecker.h>

#include "StateCost.h"

namespace RVS
{
/// @addtogroup Planner
/// @{

///@todo: maybe we can add some function to get ConstraintSet from
/// StateConstraint

RVS_CLASS_FORWARD(StateCollisionConstraint);
/**
 * @brief State collision constraint
 *
 */
class StateCollisionConstraint : public StateConstraint
{
public:
    StateCollisionConstraint(const double threshold = 0);

    virtual ~StateCollisionConstraint() override = default;

    virtual bool SetState(const JointVector &state) override;

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override;

    virtual bool IsSatisfied(const JointVector &state) override;

    virtual bool IsSatisfied() const override;

    /**
     * @brief Add new multibodies to collsion constraint(we make a copy of each
     * multibodies, so their unique name will change, in order to be able to
     * easily access these newly added bodies later, we need to return their
     * unique name).
     *
     */
    virtual bool
    AddBodiesInConstraint(const std::vector<std::shared_ptr<Multibody>> &bodies,
                          std::vector<std::string> &unique_names);

    /**
     * @brief Update multibody's pose according to the unique name.
     *
     */
    virtual bool UpdateBodyStateInConstraint(const std::string &unique_name,
                                             const SE3d &new_pose);

    /**
     * @brief Remove multibody according to the unique name.
     *
     */
    virtual bool RemoveBodyInConstraint(const std::string &unique_name);

    /**
     * @brief Remove all temporarily added multibodies .
     *
     */
    virtual bool RemoveAllBodiesInConstraint();

private:
    CollisionCheckerBasePtr m_state_collision_checker;

    /** @brief Max distance over which collisions are checked, current
     * implementation is realized by add padding to manipulator, and may lead to
     * two problems:
     * 1. Endless padding operation (Multibody::SetCollisionPadding may stuck
     * somewhere)
     * 2. RobotModel padding may lead to always self collision
     * 3. Multiply paddings
     */
    double m_threshold;

    std::vector<std::shared_ptr<Multibody>>
        m_temp_bodies; ///< Save the temporarily added multibody
    std::vector<std::string>
        m_temp_body_names; ///< Save the temporarily added multibody
                           ///< unique_names, It is convenient to update a
                           ///< single body by name
};

RVS_CLASS_FORWARD(StateCostConstraint);
/**
 * @brief Constraint from cost, when cost value <= threshold, constraint is
 * satisfied
 *
 */
class StateCostConstraint : public StateConstraint
{
public:
    StateCostConstraint(StateCostPtr cost, const double threshold);

    virtual ~StateCostConstraint() override = default;

    const StateCostPtr &GetCost() const { return m_cost; }

    void SetThreshold(const double threshold)
    {
        m_threshold = std::max(threshold, Constants<double>::Small());
    }

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override
    {
        return GetCost()->Config(env, manipulators);
    }

    virtual std::shared_ptr<Environment> GetEnvironment() const override
    {
        return GetCost()->GetEnvironment();
    }

    virtual const std::vector<std::shared_ptr<Manipulator>> &
    GetManipulators() const override
    {
        return GetCost()->GetManipulators();
    }

    virtual bool IsConfigured() const override
    {
        return GetCost()->IsConfigured();
    }

    virtual int GetDoF() const override { return GetCost()->GetDoF(); }


    virtual bool SetState(const JointVector &state) override
    {
        return GetCost()->SetState(state);
    }

    virtual JointVector GetState() const override
    {
        return GetCost()->GetState();
    }

    virtual bool IsSatisfied() const override
    {
        return (GetCost()->GetValue() <= m_threshold);
    }

protected:
    StateCostPtr m_cost;
    double m_threshold;
};

RVS_CLASS_FORWARD(StateOrientationConstraint);
/**
 * @brief State orientation constraint
 *
 */
class StateOrientationConstraint : public StateCostConstraint
{
public:
    StateOrientationConstraint();

    virtual ~StateOrientationConstraint() override = default;

    void SetParams(const SO3d &target_orientation, const CVec3d &weights,
                   const double threshold);
};

// RVS_CLASS_FORWARD(WorkZoneConstraint);
// class WorkZoneConstraint : public StateConstraint
// {
// public:
//     WorkZoneConstraint::WorkZoneConstraint();
//     virtual ~WorkZoneConstraint() = default;
// };


/// @}
} // namespace RVS