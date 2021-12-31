// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/PathSegmentBase.h>

namespace RVS
{
/// @addtogroup Trajectory
/// @{

template <typename LieGroup>
class PathSegmentBezier2nd : public PathSegmentBase<LieGroup>
{
public:
    RVS_DECLARE_PTR_MEMBER(PathSegmentBezier2nd<LieGroup>)

    DEFINE_SPACE_TYPES(LieGroup)

    /**
     * @brief Construct a new PathSegmentBezier2nd
     *
     * @param waypoints start, middle, end point of bezier segment
     * @param s0 : start position of this segment respect to the whole
     * path begining (the length of the path before the segment)
     * @param continuous_cartesian_tangent : if true, the R3::Tangent norm will
     * be kept as 1.0 when space is R3xSO3/SE3, else R3xSO3/SE3 Tangent will be
     * kept as 1.0
     */
    PathSegmentBezier2nd(const std::array<LieGroup, 3> &waypoints,
                         double s0 = 0.0,
                         bool continuous_cartesian_tangent = false);
    virtual typename PathSegmentBase<LieGroup>::Ptr Copy() const override
    {
        return std::make_shared<PathSegmentBezier2nd>(*this);
    }
    virtual LieGroup GetConfig(double s) const override;
    virtual Tangent GetTangent(double s) const override;
    virtual Curvature GetCurvature(double s) const override;

private:
    std::vector<LieGroupCalType>
        m_control_points; ///< control points of Bezier segment
};

/// @}
} // namespace RVS
