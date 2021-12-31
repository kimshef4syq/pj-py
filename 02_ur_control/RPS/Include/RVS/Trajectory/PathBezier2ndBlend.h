// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/PathBase.h>

namespace RVS
{
/// @addtogroup Trajectory
/// @{

/**
 * @brief Used to smooth a path with Bezier2nd segment blending method
 *
 * @tparam LieGroup path waypoints LieGroup type
 */
template <typename LieGroup>
class PathBezier2ndBlend : public PathBase<LieGroup>
{
public:
    RVS_DECLARE_PTR_MEMBER(PathBezier2ndBlend<LieGroup>);
    DEFINE_SPACE_TYPES(LieGroup)

    /**
     * @brief Using Bezier5th curve blend segment to smooth a path
     *
     * @param waypoints path waypoints
     * @param blend_tolerance blend tolerance
     * @param continuous_cartesian_tangent when path is R3xSO3, if
     * true, Tangent weights will be set to [1, 1, 1, 0, 0, 0], which means only
     *                                           R3Tangent will affect the norm
     * @param use_preprocess use preprocess to filter out very close
     * points of path tangent
     * @return true
     * @return false
     */
    PathBezier2ndBlend(const std::list<LieGroup> &waypoints,
                       double blend_tolerance = 0.5,
                       bool continuous_cartesian_tangent = false,
                       bool use_preprocess = true);
};

/// @}
} // namespace RVS
