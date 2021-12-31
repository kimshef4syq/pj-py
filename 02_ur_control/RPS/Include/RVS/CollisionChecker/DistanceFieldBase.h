// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/CollisionChecker/Types.h>

namespace RVS
{
/// @addgroup CollisionChecker
/// @{

/**
 * @brief This is distance field base class which is targeted at collecting
 * general properties and operations of distance field.
 *
 */
class DistanceFieldBase
{
public:
    /**
     * @brief Construct a new distance field base object
     *
     */
    DistanceFieldBase() { RVS_DEBUG("Initialize the distance field..."); }

    /**
     * @brief To add several objects (consisting of points) into distance field
     *
     * @param obj_pos  The 3-dim world coordinates (wrt left
     * bottom vertex of cuboid distance field) of adding objects
     */
    virtual void
    AddObjectsIntoField(const std::vector<std::array<double, 3>> &obj_pos) = 0;

    /**
     * @brief To delete objects from the distance field
     *
     * @param obj_pos The 3-dim world coordinates (wrt left
     * bottom vertex of cuboid distance field) of removing objects
     *
     */
    virtual void RemoveObjectsFromField(
        const std::vector<std::array<double, 3>> &obj_pos) = 0;

    // default destructor
    virtual ~DistanceFieldBase()
    {
        RVS_DEBUG("Destructing the distance field!");
    }

    /**
     * @brief Get the name of cuboid distance field
     *
     * @return const std::string& The name of current distance field
     *
     */
    virtual const std::string &GetFieldName() const = 0;

    /**
     * @brief To update distance field into initial configuration
     *
     * @return true The result if the field has been initialized successfully
     * @return false The result if it fails to do it
     *
     */
    virtual bool InitDistField() = 0;

    /**
     * @brief Get the distance to nearest obstacle at given position (voxel) in
     * distance field
     *
     * @param obj_pos The input 3-dim coordinate (wrt the left bottom vertex of
     * cuboid distance field) of voxel
     * @return double The smallest distance to the obstacle
     */
    virtual double
    GetDistanceAtPosition(const std::array<double, 3> &obj_pos) = 0;

    /**
     * @brief Get the distance to nearest obstacle at voxel with given 3-dim
     * index
     *
     * @param obj_idx The input 3-dim index of voxel
     * @return double Similarly above
     */
    virtual double GetDistanceAtIndex(const std::array<int, 3> &obj_idx) = 0;

    /**
     * @brief Get the gradient info at given position (voxel) in the distance
     * field
     *
     * @param obj_pos The input 3-dim coordinate (wrt the left bottom vertex of
     * cuboid distance field) of voxel
     * @return std::array<double,3> The current gradient value at given position
     *
     */
    virtual std::array<double, 3>
    GetGradientAtPosition(const std::array<double, 3> &obj_pos) = 0;

    /**
     * @brief Get the gradient info at voxel with given 3-dim index
     *
     * @param obj_idx The input 3-dim index of voxel
     * @return std::array<double,3> Similarly above
     */
    virtual std::array<double, 3>
    GetGradientAtIndex(const std::array<int, 3> &obj_idx) = 0;

    /**
     * @brief Get the maximum valid distance in distance field. In other words,
     * the some object will not have any effect in another object if the
     * distance betwen them is greater than maximum valid distance
     *
     * @return double The maximum valid distance of generating field for each
     * object in cuboid distance field
     */
    virtual double GetMaximumValidDistance() const = 0;
};

/// @}
} // namespace RVS