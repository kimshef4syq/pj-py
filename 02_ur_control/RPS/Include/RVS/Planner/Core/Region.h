// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/Macros.h>
#include <RVS/LieGroupHeader.h>

/** @todo
 *  1. Sampling in different dimension
 *
 */
namespace RVS
{
/// @addtogroup Planner
/// @{

template <typename LieGroup>
class Region
{
    using Tangent = typename LieGroup::Tangent;

public:
    /**
     * @brief Construct a Region
     * @param origin: the origin of the region
     * @param lower_bound: the lower bound of the region
     * @param upper_bound: the upper bound of the region
     *
     */

    Region(const LieGroup &origin, const Tangent &lower_bound,
           const Tangent &upper_bound);

    explicit Region(const LieGroup &origin)
        : Region(origin, Tangent::ZeroStatic(), Tangent::ZeroStatic())
    {
    }

    explicit Region(const Region &others)
        : m_origin(others.m_origin), m_lower_bound(others.m_lower_bound),
          m_upper_bound(others.m_upper_bound)
    {
        RVS_TRACE("Copy Constructing Region");
    }

    /**
     * @brief Get the origin config
     */
    const LieGroup &GetOrigin() const { return m_origin; }

    /**
     * @brief Get the upper bound
     */
    const Tangent &GetUpperBound() const { return m_upper_bound; }

    /**
     * @brief Get the lower bound
     */
    const Tangent &GetLowerBound() const { return m_lower_bound; }

    /**
     * @brief Set the upper bound
     */
    void SetUpperBound(const Tangent &upper_bound)
    {
        if (!IsVaildBound(m_lower_bound, upper_bound)) {
            RVS_ERROR("upper_bound should not be less than lower_bound");
            return;
        }
        m_upper_bound = upper_bound;
    }

    /**
     * @brief Set the lower bound
     */
    void SetLowerBound(const Tangent &lower_bound)
    {
        if (!IsVaildBound(lower_bound, m_upper_bound)) {
            RVS_ERROR("upper_bound should not be less than lower_bound");
            return;
        }
        m_lower_bound = lower_bound;
    }

    /**
     * @brief Set the origin
     */
    void SetOrigin(const LieGroup &origin) { m_origin = origin; }

    /**
     * @brief perform an uniform samping within the region defined by upper
     and
     * lower bound
     *
     * @param num_samples: number of samples on each dimension
     * @return poses: vector of sampled poses
     */
    std::vector<LieGroup>
    SampleUniformInRegion(const std::vector<size_t> &num_samples) const;


    /**
     * @brief check if a pose is in the region
     *
     * @param position
     * @return true or false
     */
    bool IsInRegion(const LieGroup &position) const;


    /**
     * @brief Calculate the displacement from a given position to the region
     *
     * @param position
     * @return a tangent vector expressed in the region local frame
     */
    Tangent GetDisplacement(const LieGroup &position) const;

protected:
    bool IsVaildBound(const Tangent &lower_bound,
                      const Tangent &upper_bound) const;

    LieGroup m_origin;
    Tangent m_lower_bound;
    Tangent m_upper_bound;
};


} // namespace RVS