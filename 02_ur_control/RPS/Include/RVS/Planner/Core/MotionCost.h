// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/OptModel/ComponentBase.h>
#include <RVS/OptModel/GeneralComponent.h>
#include <RVS/EnvironmentHeader.h>
#include <RVS/KinematicsHeader.h>
#include <RVS/CollisionChecker/FCLCollisionChecker/FCLCollisionChecker.h>

#include "CostConstraintBase.h"

namespace RVS
{
/// @addtogroup Planner
/// @{

RVS_CLASS_FORWARD(MotionDistanceCost);

class MotionDistanceCost : public MotionCost
{
public:
    MotionDistanceCost() = default;

    virtual ~MotionDistanceCost() override = default;

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override;

    /**
     * @brief Set the space dof weights
     *
     * @param weights
     * @return true
     * @return false
     */
    bool SetWeights(const CVecXd &weights);

    inline CVecXd GetWeights() const { return m_weights; }

    virtual double GetValue(const JointVector &s0,
                            const JointVector &s1) override;

    // virtual JointVector GetGradient(const JointVector &s0,
    //                                 const JointVector &s1) override;

private:
    CVecXd m_weights;
};


///@todo: GravityConstraint, EnergyConstraint, etc.


/// @}
} // namespace RVS