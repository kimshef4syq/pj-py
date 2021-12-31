// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <ompl/geometric/PathSimplifier.h>
#include <ompl/tools/config/MagicConstants.h>

#include <RVS/Common/LoggerUtils.h>
#include <RVS/Common/Macros.h>
#include <RVS/LieGroup/Rn.h>
#include <RVS/Planner/Ompl/WeightedRealVectorStateSpace.h>

namespace RVS
{

using JointVector = Rxd;

RVS_CLASS_FORWARD(PathSimplifier);

enum SimplifyLevel
{
    Simplify_Level_Low = 0,
    Simplify_Level_Medium,
    Simplify_Level_High
};

class PathSimplifier : public ompl::geometric::PathSimplifier
{
public:
    explicit PathSimplifier(const ompl::base::SpaceInformationPtr &si);

    virtual ~PathSimplifier() = default;

    /**
     * @brief Given a path, attempt to remove vertices from it while keeping the
     path valid. This function returns true if changes were made to the path.
     *
     * @param path
     * @return true(changes made to the path), false(the path has not changed)
     */
    bool ReduceVertices(ompl::geometric::PathGeometric &path);

    /**
     * @brief Given a path, attempt to shorten it while maintaining its
     validity. This function returns true if changes were made to the path.
     https://outgoing.energid.info/documentation/actin/PAGE_PathPlanningActin.html
     *
     * @param path
     * @return true(changes made to the path), false(the path has not changed)
     */
    bool ShortCutPath(ompl::geometric::PathGeometric &path);

    /**
     * @brief Simplify Ompl path
     * @param path
     * @return true(path is valid), false(path is invalid)
     */
    bool SimplifyMax(ompl::geometric::PathGeometric &path);

    /**
     * @brief Simplify RVS path
     * @param path
     * @return true(path is valid), false(path is invalid)
     */
    bool SimplifyMax(std::vector<JointVector> &path);

    /**
     * @brief Set Simplify Level
     * @param level
     */
    void SetSimplifyLevel(SimplifyLevel level) { m_simplify_level = level; }

private:
    /** @brief convert ompl path to rvs path */
    void _ConvertToOmplPath(const std::vector<JointVector> &rvs_path,
                            ompl::geometric::PathGeometricPtr &ompl_path);

    /** @brief convert rvs path to ompl path */
    void _ConvertToRVSPath(const ompl::geometric::PathGeometricPtr &ompl_path,
                           std::vector<JointVector> &rvs_path);

    SimplifyLevel m_simplify_level;
};
} // namespace RVS
