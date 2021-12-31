// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/PathSegmentBase.h>
#include <functional>
namespace RVS
{
/// @addtogroup Trajectory
/// @{

/**
 * @brief Path segment customed. This class is mainly used for user-defined
 * external functions. To use well-defined path shape like circle, user should
 * write new class derived from base.
 *
 * @tparam LieGroup
 */
template <typename LieGroup>
class PathSegmentCustomed : public PathSegmentBase<LieGroup>
{
    using Tangent = typename LieGroup::Tangent;
    using Curvature = typename LieGroup::Tangent;
    using Torsion = typename LieGroup::Tangent;

public:
    /**
     * @brief Default constructor
     *
     */
    PathSegmentCustomed();

    /**
     * @brief Construct a new PathSegmentCustomed object from length and
     * functions input
     *
     * @param length segment length
     * @param get_config function to get segment configuration
     * @param get_tangent function to get segment tangent
     * @param get_curvature function to get segment curvature
     */
    PathSegmentCustomed(double length,
                        const std::function<LieGroup(double)> &get_config,
                        const std::function<Tangent(double)> &get_tangent,
                        const std::function<Curvature(double)> &get_curvature);

    virtual ~PathSegmentCustomed();

    /**
     * @brief Make a copy of this object
     *
     * @return std::shared_ptr<PathSegmentBase<LieGroup>>
     */
    virtual std::shared_ptr<PathSegmentBase<LieGroup>> Copy() const override;

    /**
     * @brief Get the position of path at s
     *
     * @param s
     * @return LieGroup
     */
    virtual LieGroup GetConfig(double s) const override;

    /**
     * @brief Get the Tangent at s
     *
     * @param s
     * @return Tangent
     */
    virtual Tangent GetTangent(double s) const override;

    /**
     * @brief Get the Curvature of path at s
     *
     * @param s
     * @return Curvature
     */
    virtual Curvature GetCurvature(double s) const override;

protected:
    std::function<LieGroup(double)>
        m_f_get_config; ///< function to get segment configuration
    std::function<Tangent(double)>
        m_f_get_tangent; ///< function to get segment tangent
    std::function<Curvature(double)>
        m_f_get_curvature; ///< function to get segment curvatue
};

/// @}
} // namespace RVS
