// Copyright (c) RVBUST, Inc - All rights reserved.

#pragma once
#include "PropagateDistanceField.h"
#include <RVS/Environment/Geometry.h>
#include <RVS/Environment/Link.h>

namespace RVS
{
/// @addgroup CollisionChecker
/// @{

struct LinkVoxels
{
    /**
     * @brief Get the voxels of link
     *
     * @param is_local_frame The reference frame of coordinates of voxels with
     * link (local frame or global frame)
     * @return const std::vector<std::array<double, 3>>& The coordinates of
     * voxels with link expressed in local frame or global frame
     *
     */
    const std::vector<std::array<double, 3>> &
    GetVoxels(bool is_local_frame) const;
    std::array<double, 3>
        m_grad_nearest_to_obs; ///<  The gradient value at the voxel in link
                               ///<  nearest to the obstacle
    std::array<double, 3>
        m_sum_of_grad_with_link; ///< The sum of gradient of all voxels
                                 ///< corresponding to the link
    double m_min_plus_distance; ///< The minimal distance to the obstacle voxel
    std::array<double, 3>
        m_voxel_nearest_to_obs; ///< The voxel belonging to link is nearest to
                                ///< the obstacle
private:
    friend class LinkDecomposition;
    LinkVoxels()
        : m_grad_nearest_to_obs({0.0, 0.0, 0.0}),
          m_sum_of_grad_with_link({0.0, 0.0, 0.0}),
          m_min_plus_distance(Voxel::UNINITIALIZED),
          m_voxel_nearest_to_obs({Voxel::UNINITIALIZED, Voxel::UNINITIALIZED,
                                  Voxel::UNINITIALIZED}),
          m_link(nullptr)
    {
        m_voxels_local_frame.clear();
        m_voxels.clear();
    }
    std::vector<std::array<double, 3>>
        m_voxels_local_frame; ///< The voxels who are occupied by this link
                              ///< expressed in local frame
    mutable std::vector<std::array<double, 3>>
        m_voxels; ///< The voxels who are occupied by this link expressed
                  ///< in global or local frame
    std::shared_ptr<Link> m_link; ///< the object link
};

inline const std::vector<std::array<double, 3>> &
LinkVoxels::GetVoxels(bool is_local_frame) const
{
    if (m_link == nullptr) {
        RVS_DEBUG("The pointer of link is empty!");
        return m_voxels_local_frame;
    }
    if (!is_local_frame) {
        m_voxels = m_voxels_local_frame;
        for (auto &voxel : m_voxels) {
            CVec3d temp_voxel =
                m_link->GetPose() * CVec3d(voxel[0], voxel[1], voxel[2]);
            voxel = std::array<double, 3>(
                {temp_voxel.x(), temp_voxel.y(), temp_voxel.z()});
        }
        return m_voxels;
    }
    else {
        return m_voxels_local_frame;
    }
}

/**
 * @brief The class LinkDecomposition is served as converter which transforms a
 * single link into corresponding collision spheres if link is from robot model,
 * or decomposes the link into voxels in the distance field if the link is an
 * obstacle
 *
 */
class LinkDecomposition
{
public:
    /**
     * @brief Construct a new Link Decomposition object
     *
     * @param link The link that will be decomposed
     * @param resolution The sampling interval when decomposing the link
     * @param link_voxel_decomposition Assuming the link is from a robot model,
     * it will determine whether the voxel decomposition of link will be
     * utilized as the center of collision spheres or not (Note: if it is false,
     * the spheres decomposition will be applied into link)
     *
     */
    LinkDecomposition(std::shared_ptr<Link> link, double resolution,
                      bool link_voxel_decomposition = false);
    /**
     * @brief Get the voxels of link in the distance field assuming the link is
     * an obstacle
     *
     * @return std::shared_ptr<LinkVoxels> The info which is related to the
     * voxels of link
     */
    std::shared_ptr<LinkVoxels> GetLinkVoxel();

    /**
     * @brief Get the link (collision) spheres that will be used to check
     * collision with other objects in the given environment
     * @param is_local_frame The reference frame of coordinates of center of
     * collision spheres of link (link local frame or global world frame )
     *
     * @return const std::vector<std::shared_ptr<Sphere>>& The collision spheres
     * from input link (assuming the link belongs to some robot model)
     *
     */
    const std::vector<std::shared_ptr<Sphere>> &
    GetLinkSpheres(bool is_local_frame) const;

private:
    // To find the inner points in given link whose area of searching is a
    // cuboid (box)
    void _FindLinkInnerPoints(std::shared_ptr<Geometry> collision_geom);

    // To yield collision spheres for link who is derived from a robot model
    void _GenerateLinkSpheres();

    // To decompose link into cuboid grid points which may be used in the
    // distance field
    void _MapIntoDistanceField(std::shared_ptr<Geometry> collision_geom,
                               std::shared_ptr<const Box> xyz_range,
                               double resolution);
    std::shared_ptr<Link> m_link; ///< the object link
    std::shared_ptr<LinkVoxels>
        m_link_voxels; ///< the struct that holds the info voxels of link
    double m_resolution; ///< the sampling interval that decomposing links
    bool m_link_voxel_decomposition; ///< the switch that determines whether the
                                     ///< voxel decomposition will be used when
                                     ///< decomposing robot model link (not
                                     ///< obstacle link)
    std::vector<std::shared_ptr<Sphere>>
        m_link_spheres_local_frame; ///< the collision spheres of link from
                                    ///< robot model expressed in local frame
    mutable std::vector<std::shared_ptr<Sphere>>
        m_link_spheres; ///< the collision spheres of link from robot model
                        ///< expressed in global frame
};


/// @}
} // namespace RVS