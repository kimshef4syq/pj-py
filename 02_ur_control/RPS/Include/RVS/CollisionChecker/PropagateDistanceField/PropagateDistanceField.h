// Copyright (c) RVBUST, Inc - All rights reserved.

#pragma once

#include "VoxelStack.h"
#include "Voxel.h"
#include <RVS/CollisionChecker/DistanceFieldBase.h>

namespace RVS
{
/// @addgroup CollisionChecker
/// @{
class PropagateDistanceField : public DistanceFieldBase
{
public:
    using BucketQueue = std::vector<std::vector<std::array<int, 3>>>;
    using NeighDirections =
        std::vector<std::vector<std::set<std::array<int, 3>>>>;
    using VoxelBlock = std::vector<std::pair<std::array<int, 3>, int>>;

    /**
     * @brief Construct a new propagate distance field object
     *
     * @param field_name The name of propagation distance field
     * @param max_valid_distance The maximum valid distance of generating field
     * for each object in cuboid distance field
     * @param is_reverse_propagate If it is true, the negative distance and
     * negative voxel (non-obstacle voxel) of each voxel will be updated when
     * some obstacle voxels are added into or removed from distance field. It
     * will be ignored, otherwise.
     *
     */
    PropagateDistanceField(const std::string &field_name,
                           double max_valid_distance,
                           bool is_reverse_propagate = false);

    /**
     * @brief Construct a new propagate distance field object
     *
     * @param field_bottom_left The coordinate of bottom left vertex of
     * (rectangular) cuboid distance field
     * @param field_top_right The coordinate of top right vertex of cuboid
     * distance field
     * @param resolution  The distance of adjacent two voxel (unit: meter)
     * @param field_name The name of propagation distance field
     * @param max_valid_distance The maximum valid distance in distance
     * field. In other words, the some object will not have any effect in
     * another object if the distance betwen them is greater than maximum valid
     * distance
     * @param is_reverse_propagate Similarly above
     *
     */
    PropagateDistanceField(const std::array<double, 3> &field_bottom_left,
                           const std::array<double, 3> &field_top_right,
                           double resolution, const std::string &field_name,
                           double max_valid_distance,
                           bool is_reverse_propagate = false);
    // default destructor
    virtual ~PropagateDistanceField()
    {
        RVS_DEBUG("Destructing the progpagate distance field.");
    }

    /**
     * @brief Get the name of cuboid distance field
     *
     * @return const std::string& The name of current distance field
     *
     */
    virtual const std::string &GetFieldName() const override
    {
        return m_field_name;
    }

    /**
     * @brief Set the range of cuboid distance field
     *
     * @param field_bottom_left  The coordinate of bottom left vertex of
     * (rectangular) cuboid distance field
     * @param field_top_right  The coordinate of top right vertex of cuboid
     * distance field
     *
     */
    void SetFieldRange(const std::array<double, 3> &field_bottom_left,
                       const std::array<double, 3> &field_top_right);

    /**
     * @brief Get the size of field (cuboid)
     *
     * @return const std::array<double, 3>& The three components of the return
     * array are corresponding to length, width and height of cuboid field
     * respectively
     *
     */
    const std::array<double, 3> &GetFieldSize() const;

    /**
     * @brief Get the reference point of the distance field. Its definition is
     * left bottom vertex of cuboid field
     *
     * @return std::array<double, 3> The x,y and z coordinate of
     * reference point of distance field
     *
     */
    std::array<double, 3> GetReferOriginPoint() const;

    /**
     * @brief Set the resolution (also called sampling interval) of distance
     * field
     *
     * @param resolution The distance of adjacent two voxel (unit: meter)
     *
     */
    void SetFieldResolution(double resolution);

    /**
     * @brief Get the resolution of distance field
     *
     * @return double The current resolution of propagation distance
     * field
     *
     */
    double GetFieldResolution() const;

    /**
     * @brief To add several objects into distance field
     * @see abstract base class DistanceFieldBase
     */
    virtual void AddObjectsIntoField(
        const std::vector<std::array<double, 3>> &obj_pos) override;

    /**
     * @brief To delete objects from the distance field
     * @see abstract base class DistanceFieldBase
     *
     */
    virtual void RemoveObjectsFromField(
        const std::vector<std::array<double, 3>> &obj_pos) override;

    /**
     * @brief To update distance field into initial configuration
     * @see abstract base class DistanceFieldBase
     */
    virtual bool InitDistField() override;

    /**
     * @brief Get the distance to nearest obstacle at given position (voxel) in
     * distance field
     * @see abstract base class DistanceFieldBase
     */
    virtual double
    GetDistanceAtPosition(const std::array<double, 3> &obj_pos) override;

    /**
     * @brief Get the distance to nearest obstacle at voxel with given 3-dim
     * index
     * @see abstract base class DistanceFieldBase
     *
     */
    virtual double
    GetDistanceAtIndex(const std::array<int, 3> &obj_idx) override;

    /**
     * @brief Get the gradient info at given position (voxel) in the distance
     * field
     * @see abstract base class DistanceFieldBase
     */
    virtual std::array<double, 3>
    GetGradientAtPosition(const std::array<double, 3> &obj_pos) override;

    /**
     * @brief Get the gradient info at voxel with given 3-dim index
     * @see abstract base class DistanceFieldBase
     *
     */
    virtual std::array<double, 3>
    GetGradientAtIndex(const std::array<int, 3> &obj_idx) override;

    /**
     * @brief To enable update gradident of near voxels after adding new objects
     * into field
     *
     * @param is_update If it is true, the switch of updating gradient is on. It
     * will be switched off otherwise.
     *
     */
    void EnableUpdateGradientOfNearVoxels(bool is_update)
    {
        m_update_grad_near_voxels = is_update;
    }

    /**
     * @brief Get the maximum valid distance in distance field. In other words,
     * the some object will not have any effect in another object if the
     * distance betwen them is greater than maximum valid distance
     * @see abstract base class DistanceFieldBase
     *
     */
    virtual double GetMaximumValidDistance() const override
    {
        return m_max_valid_distance;
    }

private:
    // to initialize unparalleled parameters (instead of some parameters
    // from voxel stack) with the propagation distance field.
    void _InitializeDistanceField();

    // to set the neighbour directions (27 possible directions)
    void _FillNeighbourDirections(const std::set<int> &directions,
                                  const int deri,
                                  const std::array<int, 3> &curr_dir,
                                  const int norm_dir);

    // to reset propagation distance field (all voxels will be set to be
    // empty)
    void _ResetDistanceField();

    // to modify voxels that are specified by given voxel indices into obstacle
    // voxels
    void
    _InsertObstacleVoxels(const std::vector<std::array<int, 3>> &voxel_indices);

    // to modify voxels that are specified by given voxel indices into
    // non-obstacle voxels
    void
    _RemoveObstacleVoxels(const std::vector<std::array<int, 3>> &voxel_indices);

    // to propagate distance field along positive direction
    void _PropagatePositive();

    // to update the gradient information of near voxels of current voxel
    void _UpdateGradientNearVoxels(const std::array<int, 3> &voxel_idx);

    // to propagate distance field along negative direction
    void _PropagateNegative();

    // the smallest distance, i.e, It denotes the smallest distance to
    // non-obstacle voxel if the voxel is occupied by some obstacle. It will be
    // smallest distance to the obstacle voxel, otherwise.
    double _GetNearestDistance(const Voxel &voxel) const;

    // to transform 3-dim direction into 1-dim one
    int _GetNormalizedDirection(const std::array<int, 3> &direction);

    // to transform 1-dim direction into 3-dim one
    const std::array<int, 3> &_Get3DDirection(int norm_dir);

    // To initialize the possible propagation directions for each voxel when
    // propagating distance along positive or negative direction
    void _InitNeighbourDirections();

    // To calculate the Euclidean distance of two voxels
    int _CalcEuclideanDistance(const std::array<int, 3> &voxel_idx_lhs,
                               const std::array<int, 3> &voxel_idx_rhs);

    std::string m_field_name; ///< the name of distance field
    std::shared_ptr<VoxelStack<Voxel>>
        m_voxel_stack; ///< the container that contains all voxels in distance
                       ///< field
    double m_max_valid_distance; ///< to hold maximum valid distance
    int m_max_integer_squared_dist; ///< to hold maximum valid integer squared
                                    ///< distance
    bool m_reverse_propagate; ///< to determine whether the distance field will
                              ///< be propagated along negative direction
    bool m_update_grad_near_voxels; ///< to determine whether the gradient of
                                    ///< near voxels will be updated when there
                                    ///< are obstacles adding into or removing
                                    ///< from field
    BucketQueue m_plus_bucket_queue; ///< the container that holds modified
                                     ///< voxels that will guide the positive
                                     ///< propagation of distance field
    BucketQueue m_minus_bucket_queue; ///< the container that holds modified
                                      ///< voxels that will guide negative
                                      ///< propagation of distance field
    std::vector<double>
        m_square_root_table; ///< to hold whole valid integer distances with
                             ///< given distance field
    NeighDirections
        m_neighbour_directions; ///< the possible propagation
                                ///< directions of distance for each voxel
    std::vector<std::array<int, 3>>
        m_3d_directions; ///< to hold 27 possible 3-dim directions
    VoxelBlock m_influenced_voxels; ///<  to hold the near voxels that may be
                                    ///<  influenced by current obstacle voxel
};

inline double
PropagateDistanceField::_GetNearestDistance(const Voxel &voxel) const
{
    // The m_plus_squared_distance will be 0, if the voxel is occupied by some
    // obstacle. In contrast, the m_minus_squared_distance will be 0, if the
    // voxel is empty (non-obstacle)
    return m_square_root_table[voxel.m_plus_squared_distance]
           - m_square_root_table[voxel.m_minus_squared_distance];
}

inline int PropagateDistanceField::_GetNormalizedDirection(
    const std::array<int, 3> &direction)
{
    return (direction[0] + 1) * 9 + (direction[1] + 1) * 3 + direction[2] + 1;
}

inline const std::array<int, 3> &
PropagateDistanceField::_Get3DDirection(int norm_dir)
{
    return m_3d_directions[norm_dir];
}

inline int PropagateDistanceField::_CalcEuclideanDistance(
    const std::array<int, 3> &voxel_idx_lhs,
    const std::array<int, 3> &voxel_idx_rhs)
{
    if (!m_voxel_stack->IsValidVoxel(voxel_idx_lhs)
        || !m_voxel_stack->IsValidVoxel(voxel_idx_rhs)) {
        // It indicates that the distance of two voxels is more than valid range
        // of distance field
        return m_max_integer_squared_dist + 1;
    }
    CVec3d temp_voxel_lhs(voxel_idx_lhs[0], voxel_idx_lhs[1], voxel_idx_lhs[2]);
    CVec3d temp_voxel_rhs(voxel_idx_rhs[0], voxel_idx_rhs[1], voxel_idx_rhs[2]);
    return int((temp_voxel_lhs - temp_voxel_rhs).squaredNorm());
}


/// @}
} // namespace RVS
