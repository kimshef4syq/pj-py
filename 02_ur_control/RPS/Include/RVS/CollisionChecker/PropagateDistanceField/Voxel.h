// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/CollisionChecker/Types.h>

namespace RVS
{
/// @addgroup CollisionChecker
/// @{

struct Voxel
{
    // default constructor
    Voxel();

    /**
     * @brief Construct a new voxel object
     *
     * @param sq_dist_to_nearest_obs_vox The squared distance to nearest
     * obstacle. Further, if the voxel belongs to the obstacle, the distance
     * will be 0. Otherwise, the distance will vary from 0 to maximum valid
     * squared distance
     * @param sq_dist_to_nearest_free_vox The squared distance to nearest
     * non-obstacle. Further, if the voxel don't belong to obstacle, the
     * distance will be 0. Otherwise, the distance will vary from 0 to maximum
     * valid squared distance.
     *
     * @note The formula for calculating the squared distance between two voxels
     * is as follow:
     *  sq_dist = (voxel1_x - voxel2_x)^2 + (voxel1_y -voxel2_y)^2
     *                   + (voxel1_z - voxel2_z)^2
     *  Where the voxeli_j denotes the index of i-th voxel along j axis (i =
     *         1 or 2, j = x, y or z)
     *
     */
    Voxel(int sq_dist_to_nearest_obs_vox, int sq_dist_to_nearest_free_vox);

    int m_plus_squared_distance; ///< the squared distance to the nearest
                                 ///< obstacle
    int m_minus_squared_distance; ///< the squared distance to the nearest
                                  ///< non-obstacle
    std::array<int, 3>
        m_nearest_plus_voxel; ///< the 3-dim index of the nearest obstacle voxel
                              ///< wrt current voxel
    std::array<int, 3>
        m_nearest_minus_voxel; ///< the 3-dim index of the nearest non-obstacle
                               ///< voxel wrt current voxel
    int m_plus_update_direction; ///< the propagation direction of positive
                                 ///< (plus) distance of this voxel
    int m_minus_update_direction; ///< the propagation direction of negative
                                  ///< (minus) distance of this voxel

    static const int
        UNINITIALIZED; ///< the value that signifies an uninitialized voxel
    std::array<double, 3>
        m_gradient_at_voxel; ///< the direction of maximum change of distance
                             ///< field at the voxel
};

inline const int Voxel::UNINITIALIZED = -1;

inline Voxel::Voxel()
    : m_plus_squared_distance(UNINITIALIZED),
      m_minus_squared_distance(UNINITIALIZED),
      m_nearest_plus_voxel({UNINITIALIZED, UNINITIALIZED, UNINITIALIZED}),
      m_nearest_minus_voxel({UNINITIALIZED, UNINITIALIZED, UNINITIALIZED}),
      m_plus_update_direction(UNINITIALIZED),
      m_minus_update_direction(UNINITIALIZED),
      m_gradient_at_voxel({0.0, 0.0, 0.0})
{
}

inline Voxel::Voxel(int sq_dist_to_nearest_obs_vox,
                    int sq_dist_to_nearest_free_vox)
    : Voxel()
{
    m_plus_squared_distance = sq_dist_to_nearest_obs_vox;
    m_minus_squared_distance = sq_dist_to_nearest_free_vox;
}

/// @}
} // namespace RVS