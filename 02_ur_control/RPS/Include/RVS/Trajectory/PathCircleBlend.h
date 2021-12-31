// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/PathBase.h>

namespace RVS
{
/// @addtogroup Trajectory
/// @{

/**
 * @brief Used to smooth a path with Circle segment blending
 *
 * @tparam LieGroup : path waypoints LieGroup type
 */
template <typename LieGroup>
class PathCircleBlend : public PathBase<LieGroup>
{
public:
    RVS_DECLARE_PTR_MEMBER(PathCircleBlend<LieGroup>);
    DEFINE_SPACE_TYPES(LieGroup)

    /**
     * @brief Using Circle curve blend segment to smooth a path
     *
     * @param waypoints path waypoints
     * @param blend_tolerance blend tolerance
     * @param continuous_cartesian_tangent when path is SE3 or R3xSO3, if
     * true, Tangent weights will be set to [1, 1, 1, 0, 0, 0], which means only
     *                                           R3Tangent will affect the norm
     * @param use_preprocess use preprocess to filter out very close
     * points of path tangent
     * @return true
     * @return false
     */
    PathCircleBlend(const std::list<LieGroup> &waypoints,
                    double blend_tolerance = 0.5,
                    bool continuous_cartesian_tangent = false,
                    bool use_preprocess = true);

private:
};

/// @}
} // namespace RVS
