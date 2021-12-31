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

namespace RVS
{
/// @addtogroup Planner
/// @{

/** @brief This planner is intended to provide an easy to use interface to OMPL
 * Roadmap planner for freespace planning. It is made to take a start and end
 * point and automate the generation of the OMPL problem.
 *
 * @todo add help function like AddVertex, AddEdge, GetGraph etc.
 */
class OmplRoadmapMotionPlanner : public OmplPlannerBase
{
public:
    explicit OmplRoadmapMotionPlanner(const std::string &name = "OmplRoadmap");

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
            if (roadmap_construction_time < Constants<double>::Epsilon()) {
                roadmap_construction_time = 0.0;
            }
            return OmplPlannerBase::Configuration::Setup();
        }

        /**
         * @brief Roadmap planner used here, default is PRM planner
         *
         */
        OmplRoadmapType planner_type = OmplRoadmapType_PRM;

        /**
         * @brief Planning data file name
         *
         */
        std::string roadmap_file;

        /**
         * @brief Roadmap construction time after resetting planner
         *
         */
        double roadmap_construction_time = 1.0; // sec
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

    /** @brief OMPL Roadmap planner */
    OB::PlannerPtr m_planner;
};

/// @}
} // namespace RVS