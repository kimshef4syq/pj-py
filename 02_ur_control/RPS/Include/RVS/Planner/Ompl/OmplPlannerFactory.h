// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerDataStorage.h>
RVS_COMMON_IGNORE_WARNINGS_POP
#include "WeightedRealVectorStateSpace.h"

namespace RVS
{
/// @addtogroup Planner
/// @{

/*** For single-query planner */
enum OmplType
{
    OmplType_SBL = 0,
    OmplType_EST = 1,
    OmplType_LBKPIECE1 = 2,
    OmplType_BKPIECE1 = 3,
    OmplType_KPIECE1 = 4,
    OmplType_RRT = 5,
    OmplType_RRTConnect = 6,
    OmplType_RRTstar = 7,
    OmplType_TRRT = 8,
    OmplType_SPARS = 9
};

struct OmplPlannerFactory
{
    static OB::PlannerPtr Create(OB::SpaceInformationPtr si,
                                 const OmplType type);
};

/*** For multi-query planner */
enum OmplRoadmapType
{
    OmplRoadmapType_PRM = 0,
    OmplRoadmapType_PRMstar = 1,
    OmplRoadmapType_LazyPRM = 2,
    OmplRoadmapType_LazyPRMstar = 3
};

struct OmplRoadmapPlannerFactory
{
    static OB::PlannerPtr Create(OB::SpaceInformationPtr si,
                                 const OmplRoadmapType type,
                                 const std::string &roadmap_file = "");

    /**
     * @brief Create a PRM Planner using deterministic samples
     *
     * @param si
     * @param roadmap_file
     * @param deterministic_samples Discretize the working space, and then
     * calculate the sampling points through the inverse kinematic
     * @param construct_edges A flag indicates whether to load and construct a
     * new roadmap from deterministic samples . If true, then regenerate the
     * road map, otherwise load the road map from roadmap_file
     * @return OB::PlannerPtr
     */
    static OB::PlannerPtr CreateUsingDeterministicSamples(
        OB::SpaceInformationPtr si, const std::string &roadmap_file,
        const std::vector<std::vector<double>> &deterministic_samples,
        bool construct_edges, std::string name = "PRM Planner");
};

/// @}
} // namespace RVS
