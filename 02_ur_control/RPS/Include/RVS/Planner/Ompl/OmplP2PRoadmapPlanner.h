// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <utility>
#include <type_traits>
RVS_COMMON_IGNORE_WARNINGS_POP

#include "RVS/Planner/Core/MotionPlannerBase.h"
#include "RVS/Planner/Core/Viapoint.h"
#include "OmplPlannerFactory.h"
#include "OmplPlannerBase.h"
#include "RVS/Planner/Ompl/Algorithms/PathSimplifier.h"

namespace RVS
{
/// @addtogroup Planner
/// @{

/** @brief This planner is designed for point-to-point tasks.
 * The planner closes the random sampling in the PRM algorithm(Because the
 * random sampling path is of poor quality, and the most fatal disadvantage is
 * that it is uncontrollable, which is not allowed in industrial production),
 * uses deterministic sampling points to construct the road map, and uses A*
 * algorithm to search the road map during planning.The planner also solves the
 * problem of sometimes returning approximate solution in ompl.
 * Advantage 1: Planning is faster and stable within a certain period of time.
 * Advantage 2: The quality of the planned path is better than random sampling
 *
 * @todo In some palletizing scenarios, the environment will change with the
 * stacked goods, the original road map is no longer applicable, and the road
 * map needs to be dynamically modified
 */
class OmplP2PRoadmapPlanner : public OmplPlannerBase
{
public:
    explicit OmplP2PRoadmapPlanner(
        const std::string &name = "OmplP2PRoadmapPlanner");

    struct Configuration : public OmplPlannerBase::Configuration
    {
        RVS_DECLARE_PTR_MEMBER(Configuration);
        Configuration(
            std::shared_ptr<Environment> env_in,
            const std::vector<std::shared_ptr<Manipulator>> &manipulators_in)
            : OmplPlannerBase::Configuration(env_in, manipulators_in)
        {
        }

        virtual ~Configuration() = default;

        /** @brief Generates the OMPL problem */
        virtual bool Setup() override
        {
            return OmplPlannerBase::Configuration::Setup();
        }

        /**
         * @brief Planning data file name
         *
         */
        std::string roadmap_file;

        /**
         * @brief A flag indicates whether to load and construct a new roadmap
         * from deterministic samples . If true, then regenerate the road map,
         * otherwise load the road map that has already been generated
         *
         */
        bool construct_edges = false;

        /**
         * @brief Deterministic sampling points obtained from discrete workspace
         *
         */
        std::vector<std::vector<double>> deterministic_samples;
    };

    bool SetConfiguration(const Configuration::Ptr &configuration);

    /**
     * @brief checks if the planner is configured for planning
     * @return RVSReturn_Success if configured, RVSReturn_NotInitialized
     * otherwise
     */
    virtual RVSReturn IsConfigured() const override;

    /**
     * @brief Single query the roadmap
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

    /**
     * @brief Clear roadmap data
     *
     */
    virtual void Clear() override;

    void SaveRoadMap(const std::string &filename = "") const;

protected:
    Configuration::Ptr _CastConfiguration() const
    {
        return std::dynamic_pointer_cast<Configuration>(GetConfiguration());
    }

    /** @brief OMPL OmplP2PRoadmapPlanner*/
    OB::PlannerPtr m_planner;
};

/// @}
} // namespace RVS