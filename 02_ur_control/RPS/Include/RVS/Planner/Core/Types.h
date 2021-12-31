// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/LoggerUtils.h>
#include <RVS/Common/Types.h>
#include <RVS/EnvironmentHeader.h>
#include <RVS/OptModelHeader.h>

#include "CostConstraintBase.h"
#include "StateConstraint.h"
#include "MotionConstraint.h"
#include "StateCost.h"
#include "MotionCost.h"
#include "StateSampler.h"

namespace RVS
{
/// @addtogroup Planner
/// @{


using ManipulatorVector = std::vector<std::shared_ptr<Manipulator>>;

RVS_CLASS_FORWARD(ConfigurationBase);

/**
 * @brief Configuration base class, this class is used to configurate planning
 * environment, manipulators, costs, constraints and other planner parameters
 *
 * @todo add a lock?
 *
 * @todo maybe we can use std::map<std::string, std::string> to hold config
 * ("param", "value"), so that the following codes can be used.
 * ```cpp
 * auto it = cfg.find("param");
 *   if (it != cfg.end())
 *     param_value = string_to_double(it->second);
 * ```
 */
struct ConfigurationBase
    : public std::enable_shared_from_this<ConfigurationBase>
{
public:
    RVS_DECLARE_PTR_MEMBER(ConfigurationBase);

    ConfigurationBase(std::shared_ptr<Environment> env_in,
                      const ManipulatorVector &manipulators_in);
    /**
     * @brief Construct a new ConfigurationBase from json file
     *
     * @param json_file_path
     */
    ConfigurationBase(const std::string &json_file_path);

    virtual ~ConfigurationBase() = default;

    ///@todo: maybe we should use json to construct cost/constraint

    void AddStateCost(StateCostPtr state_cost, const double weight = 1.0);

    void AddStateConstraint(StateConstraintPtr state_constraint);

    void AddMotionCost(MotionCostPtr motion_cost, const double weight = 1.0);

    void AddMotionConstraint(MotionConstraintPtr motion_constraint);

    const CompoundStateCostPtr &GetStateCost() const { return m_state_cost; }

    const CompoundStateConstraintPtr &GetStateConstraint() const
    {
        return m_state_constraint;
    }

    const CompoundMotionCostPtr &GetMotionCost() const { return m_motion_cost; }

    const CompoundMotionConstraintPtr &GetMotionConstraint() const
    {
        return m_motion_constraint;
    }

    void SetStateSampler(StateSamplerPtr state_sampler)
    {
        m_state_sampler = state_sampler;
    }

    const StateSamplerPtr &GetStateSampler() const { return m_state_sampler; }

    /**
     * @brief Reset all costs and constraints
     *
     */
    virtual void Reset();

    /**
     * @brief Setup the planning problem (costs, constraints, planner type
     * etc.)
     * @return True on success, false on failure
     */
    virtual bool Setup();

public:
    /** @brief Environment. ***REQUIRED*** */
    std::shared_ptr<Environment> env = nullptr;

    /** @brief Manipulators used for path planning ***REQUIRED*** */
    ManipulatorVector manipulators;

    /// @todo Following constraint params is a dirty implementation for
    /// easy-use, we should find a better method to define and construct
    /// Planner cost and constraints
    /*********** State Constraints ***********/
    /**
     * @brief If true, StateCollisionConstraint will be enabled.
     * Default: true
     *
     */
    bool collision_check = true;

    /** @brief Max distance over which collisions are checked, current
     * implementation is realized by add padding to manipulator, and may
     * lead to two problems:
     * 1. Endless padding operation (Multibody::SetCollisionPadding may
     * stuck somewhere)
     * 2. RobotModel padding may lead to always self collision
     * 3. Multiply paddings
     */
    double collision_distance_threshold = 0;

protected:
    virtual void _ConstructStateValidator();

protected:
    CompoundStateCostPtr m_state_cost;

    CompoundStateConstraintPtr m_state_constraint;

    CompoundMotionCostPtr m_motion_cost;

    CompoundMotionConstraintPtr m_motion_constraint;

    StateSamplerPtr m_state_sampler;
};

/**
 * @brief MotionPlanner request
 *
 */
struct MotionPlannerRequest
{
    std::vector<ViapointPtr> viapoints;
};

/**
 * @brief MotionPlanner response
 *
 */
struct MotionPlannerResponse
{
    RVSReturn status; ///< planning result
    double path_cost;
    std::vector<JointVector> joint_trajectory;
};

/// @todo TaskPlanner Request and response, task planner should return a list of
/// joint_trajectory; or maybe we should define different req/res for each
/// TaskPlanner

/**
 * @brief Add multibodies to planner constraint
 *
 * @param state_constraint
 * @param bodies multibody need to be added to planner constraint
 * @param unique_names we make a copy of added multibody, return the unique name
 * of added body
 * @return true
 * @return false
 */
bool AddBodiesInConstraint(
    const StateConstraintPtr &state_constraint,
    const std::vector<std::shared_ptr<Multibody>> &bodies,
    std::vector<std::string> &unique_names);

/**
 * @brief Update multibody in planner constraint
 *
 * @param state_constraint
 * @param unique_name target multibody's unique name(get from
 * "AddBodiesInConstraint")
 * @param new_pose
 * @return true
 * @return false
 */
bool UpdateBodyStateInConstraint(const StateConstraintPtr &state_constraint,
                                 const std::string &unique_name,
                                 const SE3d &new_pose);

/**
 * @brief Remove multibody in planner constraint
 *
 * @param state_constraint
 * @param unique_name target multibody's unique name(get from
 * "AddBodiesInConstraint")
 * @return true
 * @return false
 */
bool RemoveBodyInConstraint(const StateConstraintPtr &state_constraint,
                            const std::string &unique_name);

/**
 * @brief Remove all multibodies in planner constraint
 *
 * @param state_constraint
 * @return true
 * @return false
 */
bool RemoveAllBodiesInConstraint(const StateConstraintPtr &state_constraint);

/// @}
} // namespace RVS