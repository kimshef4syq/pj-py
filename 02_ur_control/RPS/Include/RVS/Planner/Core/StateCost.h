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

///@todo: Maybe we can add some function to get CostTerm from StateCost


RVS_CLASS_FORWARD(TCPAxisInBaseCost);
class TCPAxisInBaseCost : public StateCost
{
public:
    typedef enum
    {
        TCP_AXIS_X = 0,
        TCP_AXIS_Y = 1,
        TCP_AXIS_Z = 2,
    } TCP_AXIS;

    TCPAxisInBaseCost(double ref_value, int external_axis_idx = 0,
                      const TCP_AXIS tcp_axis = TCP_AXIS::TCP_AXIS_Y)
        : StateCost(), m_kin_solver(), m_ref_value(ref_value),
          m_external_axis_idx(external_axis_idx), m_axis_proj_mat()
    {
        m_axis_proj_mat = MatXd::Zero(1, 3);
        switch (tcp_axis) {
        case (TCP_AXIS_X): {
            m_axis_proj_mat(0, 0) = 1;
            break;
        }
        case (TCP_AXIS_Y): {
            m_axis_proj_mat(0, 1) = 1;
            break;
        }
        case (TCP_AXIS_Z): {
            m_axis_proj_mat(0, 2) = 1;
            break;
        }
        }
    }

    void SetReferenceValue(const double ref_value) { m_ref_value = ref_value; }

    virtual ~TCPAxisInBaseCost() = default;

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override;

    virtual double GetValue() const override;

    virtual JointVector GetGradient() const override;

private:
    std::shared_ptr<KinematicsBase> m_kin_solver;
    double m_ref_value;
    int m_external_axis_idx;
    MatXd m_axis_proj_mat;
};

RVS_CLASS_FORWARD(EXStateCost);
class ExStateCost : public StateCost
{
public:
    explicit ExStateCost(std::function<double(const Rxd &)> get_value_function,
                         std::function<Rxd(const Rxd &)> get_gradient_function)
        : m_get_value_function(std::move(get_value_function)),
          m_get_gradient_function(std::move(get_gradient_function))
    {
    }

    virtual ~ExStateCost() override = default;

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override
    {
        RVS_UNUSED(env);
        RVS_UNUSED(manipulators);
        return true;
    }

    virtual double GetValue(const JointVector &state) override
    {
        return m_get_value_function(state);
    }

    virtual JointVector GetGradient(const JointVector &state) override
    {
        return m_get_gradient_function(state);
    }

    virtual double GetValue() const override
    {
        RVS_ERROR("Not Implemented");
        return 0;
    }

    virtual JointVector GetGradient() const override
    {
        RVS_ERROR("Not Implemented");
        return JointVector();
    }


private:
    std::function<double(const Rxd &)> m_get_value_function;
    std::function<Rxd(const Rxd &)> m_get_gradient_function;
};

RVS_CLASS_FORWARD(ManipulabilityCost);
/**
 * @brief Manipulability Cost
 *
 */
class ManipulabilityCost : public StateCost
{
public:
    ManipulabilityCost(
        const ManipubilityType type = ManipubilityType_MultiplyAll);

    virtual ~ManipulabilityCost() override = default;

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override;

    virtual double GetValue() const override;

private:
    ManipubilityType m_type;
    std::vector<std::shared_ptr<KinematicsBase>> m_kin_solvers;
};

RVS_CLASS_FORWARD(JointLimitsCost);
/**
 * @brief Joint limit cost
 *
 */
class JointLimitsCost : public StateCost
{
public:
    JointLimitsCost() = default;

    virtual ~JointLimitsCost() override = default;

    virtual bool Config(
        std::shared_ptr<Environment> env,
        const std::vector<std::shared_ptr<Manipulator>> &manipulators) override;

    /**
     * @brief Joint limit cost function, gives higher weight to the
     * joints nearing their limits and goes to infinity at the joint
     * bounds
     *
     * @note Chan, Tan Fung, and Rajiv V. Dubey. "A weighted
     * least-norm solution based scheme for avoiding joint limits
     * for redundant joint manipulators." IEEE Transactions on
     * Robotics and Automation 11.2 (1995): 286-292.
     * - JointLimitsCost: H(q) =
     * \sum_{i=1}^{n}{\frac{1}{4}}{\frac{(q_{i,max}-q_{i,min})^2}{(q_{i,max}-q)(q-q_{i,min})}}
     *
     * @todo: I'm not sure if we should add joint weights here
     *
     * @return double cost values
     */
    virtual double GetValue() const override;

    /**
     * @brief Gradient of JointLimitsCost.
     *  - JointLimitsGradient: d{H(q_i)} =
     * \frac{\partial{H(q)}}{\partial{q_i}}
     *
     * @return JointVector gradient in each dof
     */
    virtual JointVector GetGradient() const override;

private:
    std::vector<JointLimit> m_joint_limits;
};


RVS_CLASS_FORWARD(StateOrientationCost);
/**
 * @brief State orientation cost
 *
 */
class StateOrientationCost : public StateCost
{
public:
    StateOrientationCost();

    virtual ~StateOrientationCost() override = default;

    void SetParams(const SO3d &target_orientation, const CVec3d &weights);

    virtual double GetValue() const override;

    // virtual JointVector GetGradient() const override;

private:
    SO3d m_target_orientation;
    CVec3d m_weights;
};

RVS_CLASS_FORWARD(StatePoseCost);
/**
 * @brief State pose cost (position and orientation)
 *
 */
class StatePoseCost : public StateCost
{
public:
    StatePoseCost();

    virtual ~StatePoseCost() override = default;

    void SetParams(const SE3d &target_pose, const CVec6d &weights);

    virtual double GetValue() const override;

    // virtual JointVector GetGradient() const override;

private:
    SE3d m_target_pose;
    CVec6d m_weights;
};


/// @}
} // namespace RVS