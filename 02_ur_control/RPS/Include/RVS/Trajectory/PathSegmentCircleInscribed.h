// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/PathSegmentBase.h>

namespace RVS
{
/// @addtogroup Trajectory
/// @{

template <typename LieGroup>
class PathSegmentCircleInscribed : public PathSegmentBase<LieGroup>
{
public:
    RVS_DECLARE_PTR_MEMBER(PathSegmentCircleInscribed<LieGroup>)
    DEFINE_SPACE_TYPES(LieGroup)
    /**
     * @brief Construct a new PathSegmentCircleInscribed
     *
     * @param waypoints start, middle, end point of bezier segment
     * @param blend_tolerance : max deviation distance of the circle arc
     * @param s0 : start position of this segment respect to the whole
     * path begining (the length of the path before the segment)
     * @param continuous_cartesian_tangent : if true, the R3::Tangent norm will
     * be kept as 1.0 when space is R3xSO3/SE3, else R3xSO3/SE3 Tangent will be
     * kept as 1.0
     */
    PathSegmentCircleInscribed(const std::array<LieGroup, 3> &waypoints,
                               double blend_tolerance, double s0 = 0.0,
                               bool continuous_cartesian_tangent = false);
    virtual typename PathSegmentBase<LieGroup>::Ptr Copy() const override
    {
        return std::make_shared<PathSegmentCircleInscribed>(*this);
    }
    virtual LieGroup GetConfig(double s) const override;
    virtual Tangent GetTangent(double s) const override;
    virtual Curvature GetCurvature(double s) const override;

    ///@brief Get cirlce radius
    double GetRadius() const { return m_radius; }

    ///@brief Get circle center
    LieGroup GetCircleCenter() const { return m_center; }

    ///@brief Get circle center to start point direction
    Tangent GetDirectionX() const { return m_x; }

    ///@brief Get direction perpendicular to DerectionX
    Tangent GetDirectionY() const { return m_y; }

private:
    double m_radius{0.0}; ///< radius of circle segment
    LieGroupCalType m_center; ///< center of circle segment
    TangentCalType m_x; ///< unit vector from center position to start position
                        ///< of circle segment
    TangentCalType m_y; ///< unit vector which is perpendicular to m_x
};

/// @}
} // namespace RVS
