// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/LoggerUtils.h>
#include <RVS/Common/Types.h>
#include <RVS/EnvironmentHeader.h>
#include <RVS/OptModelHeader.h>
#include "Viapoint.h"

namespace RVS
{
/// @addtogroup Planner
/// @{

/** @todos:
 *       1. Enable multi-thread checking;
 *       2. Benchmarking cost/constraint evaluation time.
 */
RVS_CLASS_FORWARD(CostConstraintBase);

class CostConstraintBase
    : public std::enable_shared_from_this<CostConstraintBase>
{
public:
    inline bool Config(std::shared_ptr<Environment> env,
                       std::shared_ptr<Manipulator> manipulator)
    {
        return Config(env,
                      std::vector<std::shared_ptr<Manipulator>>{manipulator});
    }

    virtual bool
    Config(std::shared_ptr<Environment> env,
           const std::vector<std::shared_ptr<Manipulator>> &manipulators);

    virtual std::shared_ptr<Environment> GetEnvironment() const
    {
        return m_env;
    }

    virtual const std::vector<std::shared_ptr<Manipulator>> &
    GetManipulators() const
    {
        return m_manipulators;
    }

    virtual bool IsConfigured() const { return m_is_config; }

    virtual int GetDoF() const { return m_dof; }

    virtual bool SetState(const JointVector &state);

    virtual JointVector GetState() const;

    virtual ~CostConstraintBase() = default;

protected:
    CostConstraintBase() = default;

protected:
    std::mutex m_state_mutex;
    bool m_is_config{false};
    int m_dof{0};
    std::shared_ptr<Environment> m_env{nullptr};
    std::vector<std::shared_ptr<Manipulator>> m_manipulators;
};


/////// Cost and Constraint Base class ///////

RVS_CLASS_FORWARD(StateCost);

/**
 * @brief State Cost
 *
 */
class StateCost : public CostConstraintBase
{
public:
    /**
     * @brief Get the Cost value of current state
     *
     * @return double
     */
    virtual double GetValue() const = 0;

    /**
     * @brief Get the Cost value of input state
     *
     * @return double
     */
    virtual double GetValue(const JointVector &state);

    /**
     * @brief Get the gradient value of current state
     *
     * @return JointVector, default is all-zeros
     */
    virtual JointVector GetGradient() const
    {
        return JointVector::IdentityStatic();
    }

    /**
     * @brief Get the gradient value of input state
     *
     * @return JointVector
     */
    virtual JointVector GetGradient(const JointVector &state);

protected:
    StateCost() = default;
};

RVS_CLASS_FORWARD(StateConstraint);

/**
 * @brief State constraint
 *
 */
class StateConstraint : public CostConstraintBase
{
public:
    /**
     * @brief Check if the input state is valid
     *
     * @param state
     * @return true
     * @return false
     */
    virtual bool IsSatisfied(const JointVector &state);

    /**
     * @brief Check if current state is valid;
     *
     * @return true
     * @return false
     */
    virtual bool IsSatisfied() const = 0;

protected:
    StateConstraint() = default;
};


RVS_CLASS_FORWARD(MotionCost);

/**
 * @brief Motion Cost
 *
 * @todo Maybe we should define a Motion type, rather than using two states
 *
 */
class MotionCost : public CostConstraintBase
{
public:
    /**
     * @brief Get the motion cost from state0 to state1
     *
     * @param s0 state 0
     * @param s1 state 1
     * @return double
     */
    virtual double GetValue(const JointVector &s0, const JointVector &s1) = 0;

protected:
    MotionCost() = default;
};


RVS_CLASS_FORWARD(MotionConstraint);

/**
 * @brief Motion Constraint
 *
 */
class MotionConstraint : public CostConstraintBase
{
public:
    virtual bool IsSatisfied(const JointVector &s0, const JointVector &s1) = 0;

    // virtual bool IsSatisfied(const JointVector &s0,
    //                          const JointVector &s1, std::pair<JointVector,
    //                          double> &last_valid) = 0;

protected:
    MotionConstraint() = default;
};


/////// Compound Cost Constraint ///////

RVS_CLASS_FORWARD(CompoundStateCost);
/**
 * @brief Compund State Cost, costs are added by different weights.
 *
 */
class CompoundStateCost : public StateCost
{
public:
    CompoundStateCost() = default;

    virtual ~CompoundStateCost() = default;

    virtual void AddStateCost(StateCostPtr cost, const double weight = 1.0);

    virtual void Reset();

    virtual int GetNumComponents() const;

    virtual bool SetState(const JointVector &state) override;

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override;

    virtual double GetValue() const override;

    virtual JointVector GetGradient() const override;

private:
    /** @brief Defines a pairing of an objective and its weight */
    struct Component
    {
        Component(StateCostPtr cost, double weight) : cost(cost), weight(weight)
        {
        }
        StateCostPtr cost;
        double weight;
    };

    /** @brief List of cost/weight pairs */
    std::vector<Component> m_components;
};

RVS_CLASS_FORWARD(CompoundStateConstraint);
/**
 * @brief Compund State Constraint, all constraints should be satisfied.
 *
 */
class CompoundStateConstraint : public StateConstraint
{
public:
    CompoundStateConstraint() = default;

    virtual ~CompoundStateConstraint() = default;

    virtual void AddStateConstraint(StateConstraintPtr constraint);

    virtual void Reset();

    virtual bool SetState(const JointVector &state) override;

    /** @brief Return all constraints.*/
    virtual const std::vector<StateConstraintPtr> &GetComponents() const;

    virtual int GetNumComponents() const;

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override;

    virtual bool IsSatisfied() const override;

private:
    std::vector<StateConstraintPtr> m_constraints;
};

RVS_CLASS_FORWARD(CompoundMotionCost);
/**
 * @brief Compund Motion Cost, costs are added by different weights.
 *
 */
class CompoundMotionCost : public MotionCost
{
public:
    CompoundMotionCost() = default;

    virtual ~CompoundMotionCost() = default;

    virtual void AddMotionCost(MotionCostPtr cost, const double weight = 1.0);

    virtual void Reset();

    virtual int GetNumComponents() const;

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override;

    virtual double GetValue(const JointVector &s0,
                            const JointVector &s1) override;

private:
    /** @brief Defines a pairing of an motion cost and its weight */
    struct Component
    {
        Component(MotionCostPtr cost, double weight)
            : cost(cost), weight(weight)
        {
        }
        MotionCostPtr cost;
        double weight;
    };

    /** @brief List of cost/weight pairs */
    std::vector<Component> m_components;
};

RVS_CLASS_FORWARD(CompoundMotionConstraint);
/**
 * @brief Compund Motion Constraint, all constraints should be satisfied.
 *
 */
class CompoundMotionConstraint : public MotionConstraint
{
public:
    CompoundMotionConstraint() = default;

    CompoundMotionConstraint(MotionConstraintPtr constraint);

    virtual ~CompoundMotionConstraint() = default;

    virtual void AddMotionConstraint(MotionConstraintPtr constraint);

    virtual void Reset();

    virtual int GetNumComponents() const;

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override;

    virtual bool IsSatisfied(const JointVector &s0,
                             const JointVector &s1) override;

    // virtual bool
    // IsSatisfied(const JointVector &s0, const JointVector &s1,
    //             std::pair<JointVector, double> &last_valid) override;

private:
    std::vector<MotionConstraintPtr> m_constraints;
};

/// @}
} // namespace RVS