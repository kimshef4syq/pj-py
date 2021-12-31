// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/geometric/PathSimplifier.h>
#include <utility>
#include <type_traits>
RVS_COMMON_IGNORE_WARNINGS_POP

#include "RVS/Planner/Core/MotionPlannerBase.h"
#include "RVS/Planner/Core/Viapoint.h"
#include "OmplPlannerFactory.h"
#include "OmplPlannerBase.h"

namespace RVS
{
/// @addtogroup Planner
/// @{

/**
 * @brief ParallelPlan wrapper to enable Reset problem definition;
 *
 */
class ParallelPlan : public OT::ParallelPlan
{
public:
    ParallelPlan(const OB::ProblemDefinitionPtr &pdef) : OT::ParallelPlan(pdef)
    {
    }

    bool SetProblemDefinition(const OB::ProblemDefinitionPtr &pdef);
};

/** @brief This planner is intended to provide an easy to use interface to OMPL
 * for parallel freespace planning. It is made to take a start and end point and
 * automate the generation of the OMPL problem.
 */
class OmplParallelPlanMotionPlanner : public OmplPlannerBase
{
public:
    explicit OmplParallelPlanMotionPlanner(
        const std::string &name = "OmplParallel");

    struct Configuration : public OmplPlannerBase::Configuration
    {
        RVS_DECLARE_PTR_MEMBER(Configuration);
        Configuration(
            std::shared_ptr<Environment> env_in,
            const std::vector<std::shared_ptr<Manipulator>> &manipulators_in)
            : OmplPlannerBase::Configuration(env_in, manipulators_in)
        {
        }

        Configuration(
            std::shared_ptr<Environment> env_in,
            const std::vector<std::shared_ptr<Manipulator>> &manipulators_in,
            const std::vector<OmplType> &planner_types_in)
            : OmplPlannerBase::Configuration(env_in, manipulators_in),
              planner_types(planner_types_in)
        {
        }

        virtual ~Configuration() override = default;

        /** @brief Generates the OMPL problem */
        virtual bool Setup() override
        {
            if (planner_types.empty()) {
                RVS_INFO("Empty planners found, use 4 thread RRTConnect");
                planner_types.insert(planner_types.begin(), 4,
                                     OmplType_RRTConnect);
            }

            return OmplPlannerBase::Configuration::Setup();
        }

        /**
         * @brief The planner enums, default using 4 RRTConnect
         *
         * This will create a new thead for each planner configurator provided.
         */
        std::vector<OmplType> planner_types;


        /**
         * @brief The max number of solutions. If max solutions are hit it will
         * exit even if other threads are running.
         *
         */
        int max_solutions = 10;

        /** if more solution paths are available, they are hybridized. default
           is false, because there is a bug which will return a trajectory that
           starts at the end state and finishes at the end state */
        bool hybridize = true;

        /**
         * @brief This uses all available planning time to create the most
         * optimized trajectory given the objective function.
         *
         * This is required because not all OMPL planners are optimize graph
         * planners. If the planner you choose is an optimize graph planner then
         * setting this to true has no affect. In the case of non-optimize
         * planners they still use the OptimizeObjective function but only when
         * searching the graph to find the most optimize solution based on the
         * provided optimize objective function. In the case of these type of
         * planners like RRT and RRTConnect if set to true it will leverage all
         * planning time to keep finding solutions up to your max solutions
         * count to find the most optimal solution.
         */
        bool optimize = false;
    };

    bool SetConfiguration(const Configuration::Ptr &configuration);

    /**
     * @brief checks if the planner is configured for planning
     * @return RVSReturn_Success if configured, RVSReturn_NotInitialized
     * otherwise
     */
    virtual RVSReturn IsConfigured() const override;

    /**
     * @brief Sets up the OMPL problem then solves. It is intended to simplify
     * setting up and solving freespace motion problems.
     *
     * This planner leverages OMPL ParallelPlan which supports being called
     * multiple times. So you are able to setConfiguration and keep calling
     * solve and with each solve it will continue to build the existing planner
     * graph. If you want to start with a clean graph you must call
     * setConfiguration() before calling solve again.
     *
     * This planner (and the associated config passed to the setConfiguration)
     * does not expose all of the available configuration data in OMPL. This is
     * done to simplify the interface. However, many problems may require more
     * specific setups. In that case, the source code for this planner may be
     * used as an example.
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
                       const bool verbose = false) override;

    virtual void Clear() override;

protected:
    /** @brief OMPL Parallel planner */
    std::shared_ptr<ParallelPlan> m_parallel_plan;

protected:
    Configuration::Ptr _CastConfiguration() const
    {
        return std::dynamic_pointer_cast<Configuration>(GetConfiguration());
    }
};

/// @}
} // namespace RVS