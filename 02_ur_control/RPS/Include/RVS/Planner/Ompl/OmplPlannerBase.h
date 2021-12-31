// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
RVS_COMMON_IGNORE_WARNINGS_POP
#include <RVS/Common/Time.h>
#include <RVS/Planner/Core/Types.h>
#include <RVS/Planner/Core/MotionPlannerBase.h>
#include <RVS/Planner/Ompl/OmplPlannerFactory.h>
#include <RVS/Planner/Ompl/Algorithms/PathSimplifier.h>

namespace RVS
{
/// @addtogroup Planner
/// @{

namespace OM = ompl::msg;
namespace OT = ompl::tools;
namespace OG = ompl::geometric;

class OmplPlannerBase : public MotionPlannerBase
{
public:
    explicit OmplPlannerBase(const std::string &name = "OmplPlanner");

    /** @brief The OmplPlannerBase Configuration struct
     *
     * @todo maybe we can use std::map<std::string, std::string> to hold config
     * ("param", "value"), so that the following codes can be used.
     * ```cpp
     * auto it = cfg.find("param");
     *   if (it != cfg.end())
     *     param_value = string_to_double(it->second);
     * ```
     */
    struct Configuration : public ConfigurationBase
    {
        RVS_DECLARE_PTR_MEMBER(Configuration);

        Configuration(
            std::shared_ptr<Environment> env_in,
            const std::vector<std::shared_ptr<Manipulator>> &manipulators_in);

        Configuration(std::shared_ptr<Environment> env_in,
                      const std::shared_ptr<Manipulator> &manipulator_in);

        virtual ~Configuration() override = default;

        /**
         * @brief Setup the OMPL problem (costs, constraints, planner type etc.)
         * @return True on success, false on failure
         */
        virtual bool Setup() override;

        /// Ompl params

        /** @brief the OMPL planning context; this contains the problem
         * definition and the planner used ***REQUIRED*** */
        OG::SimpleSetupPtr ompl_simple_setup;

        /** @brief ompl path simplifier*/
        PathSimplifierPtr path_simplifier;

        SimplifyLevel simplify_level = Simplify_Level_Medium;

        /** @brief Max planning time allowed in seconds */
        double planning_time = 5.0;

        /** @brief Simplify trajectory. */
        bool simplify = true;

        /** @brief Smooth trajectory. */
        bool smooth = false;

        /****** Motion Constraint **********/

        ///@todo: The following parameters are not valid now
        // /** @brief If true, use continuous collision checking */
        // bool collision_continuous = false;

        /**
         * @brief Set the resolution at which state validity needs to be
         * verified in order for a motion between two states to be considered
         * valid in post checking of trajectory returned by trajopt.
         *
         * The resolution is equal to longest_valid_segment_fraction *
         * state_space.getMaximumExtent()
         *
         * Note: The planner takes the conservative of either
         * longest_valid_segment_fraction or longest_valid_segment_length.
         *
         */
        double longest_valid_segment_fraction = 0.01; // 1%

        /**
         * @brief Set the resolution at which state validity needs to be
         * verified in order for a motion between two states to be considered
         * valid. If norm(state1 - state0) > longest_valid_segment_length.
         *
         * Note: This gets converted to longest_valid_segment_fraction.
         *       longest_valid_segment_fraction = longest_valid_segment_length /
         * state_space.getMaximumExtent()
         *
         */
        double longest_valid_segment_length = 0.1; // rad or m

        /*** State Constraint *****/
        /**
         * @brief If true, StateOrientationConstraint will be enabled. Default:
         * false
         *
         */
        bool orientation_check = false;
        SO3d target_orientation = SO3d::IdentityStatic();
        CVec3d orientation_weights = CVec3d(1, 1, 0);
        double orientation_threshold = 0.1;

    protected:
        void
        _ProcessLongestValidSegment(const OB::StateSpacePtr &state_space_ptr);

        virtual void _ConstructStateValidator() override;
    };

    //// help functions
    static MotionPlannerRequest CreateRequest(const JointVector &start,
                                              const JointVector &goal);

    const Configuration::Ptr &GetConfiguration() const
    {
        return m_configuration;
    }

    virtual bool Terminate() override;

    virtual void Clear() override;

    double GetLastPlanningTime() const { return m_planning_time; }

    double GetLastSimplificationTime() const { return m_simplification_time; }

    double GetLastSmoothTime() const { return m_smooth_time; }

protected:
    /** @brief The ompl planner configuration */
    Configuration::Ptr m_configuration;

    double m_planning_time; ///< path planning time

    double m_simplification_time; ///< path simplification time

    double m_smooth_time; ///< Bspline smooth time

    const OB::PlannerTerminationCondition *m_ptc;
    std::mutex m_ptc_lock;

protected:
    bool _ProcessRequest(const MotionPlannerRequest &request);

    /// @todo OB::GoalPtr _ConstructGoal();

    void _PreSolve();
    void _PostSolve();

    /// lazy goal sampling in background thread
    void _StartGoalSampling();
    void _StopGoalSampling();

    void
    _RegisterTerminationCondition(const OB::PlannerTerminationCondition &ptc);
    void _UnregisterTerminationCondition();

    /// path simplification and smooth
    bool _SimplifyPath(const OB::PathPtr &ompl_path);
    void _SmoothPath(const OB::PathPtr &ompl_path);

    void _ConvertSolution(const OB::PathPtr &ompl_path,
                          MotionPlannerResponse &response);
};

/// @}
} // namespace RVS
