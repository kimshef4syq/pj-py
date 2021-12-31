// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "StateConstraint.h"
#include "MotionCost.h"

namespace RVS
{
/// @addtogroup Planner
/// @{

RVS_CLASS_FORWARD(DiscreteMotionConstraint);

class DiscreteMotionConstraint : public MotionConstraint
{
public:
    DiscreteMotionConstraint(StateConstraintPtr state_constraint);

    virtual ~DiscreteMotionConstraint() override = default;

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override;

    bool SetLongestValidSegmentCountFactor(const unsigned int factor);

    inline unsigned int GetValidSegmentCountFactor() const
    {
        return m_longest_valid_segment_count_factor;
    }

    bool SetLongestValidSegmentLength(const double length);

    inline double GetLongestValidSegmentLength() const
    {
        return m_longest_valid_segment;
    }

    virtual bool IsSatisfied(const JointVector &s0,
                             const JointVector &s1) override;

    // virtual bool IsSatisfied(const JointVector &s0,
    //                          const JointVector &s1, std::pair<JointVector,
    //                          double> &last_valid) override;


private:
    unsigned int _ValidSegmentCount(const JointVector &s0,
                                    const JointVector &s1) const;

private:
    StateConstraintPtr m_state_constraint;

    unsigned int m_longest_valid_segment_count_factor{
        1}; ///< The factor to multiply the
            ///< value returned by
            ///< _ValidSegmentCount().
            ///< Rarely used but useful for
            ///< things like doubling the
            ///< resolution, default value
            ///< is 1

    double m_longest_valid_segment{0.1}; ///< The longest valid segment
                                         ///< length, default is 0.1 (rad/m)
};


///@todo: ContinuousPathCollisionChecking, GravityConstraint, EnergyConstraint,
/// etc.

RVS_CLASS_FORWARD(MotionCostConstraint);
/**
 * @brief Constraint from cost, when cost value <= threshold, constraint is
 * satisfied
 *
 */
class MotionCostConstraint : public MotionConstraint
{
public:
    MotionCostConstraint(MotionCostPtr cost, const double threshold);

    virtual ~MotionCostConstraint() override = default;

    const MotionCostPtr &GetCost() const { return m_cost; }

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

    virtual bool IsSatisfied(const JointVector &s0,
                             const JointVector &s1) override
    {
        return (GetCost()->GetValue(s0, s1) <= m_threshold);
    }

protected:
    MotionCostPtr m_cost;
    double m_threshold;
};


/// @}
} // namespace RVS