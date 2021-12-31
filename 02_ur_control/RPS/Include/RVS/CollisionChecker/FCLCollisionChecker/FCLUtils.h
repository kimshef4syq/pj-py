// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <fcl/fcl.h>
#include <RVS/Environment/Geometry.h>
#include <RVS/Common/Macros.h>
#include <RVS/CollisionChecker/Types.h>
#include <RVS/Environment/Link.h>

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace RVS
{
/// @addtogroup CollisionChecker
/// @{

/// FCLCollisionChecker broadphase type
enum BroadphaseType
{
    BroadphaseType_Naive = 0, // Brute force N-body collision manager
    BroadphaseType_DynamicAABBTree = 1,
    BroadphaseType_DynamicAABBTreeArray = 2,
    BroadphaseType_IntervalTree = 3,
    BroadphaseType_SaP = 4,
    BroadphaseType_SSaP = 5,
    BroadphaseType_SpatialHashing = 6,
};

/// bounding volume type
enum BVRepType
{
    BVRepType_AABB = 0, ///< a box in 3D space determined by two diagonal points
    BVRepType_OBB = 1, ///< Oriented bounding box class
    BVRepType_RSS = 2, ///< a rectangle sphere-swept bounding volume
    BVRepType_OBBRSS = 3, ///< Class merging the OBB and RSS, can handle
                          ///< collision and distance simultaneously
    BVRepType_kIOS = 4, ///< a set of spheres.
    BVRepType_KDOP16 =
        5, ///< The KDOP structure is defined by some pairs of parallel planes
           ///< defined by some axes, K is set as the template parameter, which
    /// should be 16, 18, or 24
    BVRepType_KDOP18 = 6,
    BVRepType_KDOP24 = 7,
};

enum CollisionFilterGroups
{
    DefaultFilter = 1,
    StaticFilter = 2,
    KinematicFilter = 4,
    AllFilter =
        -1 // all bits sets: DefaultFilter | StaticFilter | KinematicFilter
};

inline fcl::Transform3<double> SE3ToTransform(const SE3d &pose)
{
    return fcl::Transform3<double>(pose.Transform());
}


RVS_CLASS_FORWARD(CollisionObjectWrapper)

class CollisionObjectWrapper
{
public:
    CollisionObjectWrapper(const std::string &name,
                           const std::shared_ptr<Geometry> &shape,
                           const SE3d &shape_pose);
    bool m_enabled;

    const std::string &GetName() const { return m_name; }

    const std::shared_ptr<Geometry> &GetCollisionGeometries() const
    {
        return m_shape;
    }

    const SE3d &GetCollisionGeometriesTransforms() const
    {
        return m_shape_pose;
    }

    void SetCollisionObjectsTransform(const SE3d &pose)
    {
        m_world_pose = pose;
        std::shared_ptr<fcl::CollisionObject<double>> &co = m_collision_objects;
        co->setTransform(SE3ToTransform(pose * m_shape_pose));
        co->computeAABB();
    }

    const SE3d &GetCollisionObjectTransform() const { return m_world_pose; }

    const std::shared_ptr<fcl::CollisionObject<double>> &
    GetCollisionObject() const
    {
        return m_collision_objects;
    }

    std::shared_ptr<fcl::CollisionObject<double>> &GetCollisionObject()
    {
        return m_collision_objects;
    }

    std::shared_ptr<CollisionObjectWrapper> Clone() const
    {
        std::shared_ptr<CollisionObjectWrapper> clone_cow(
            new CollisionObjectWrapper(m_name, m_shape, m_shape_pose,
                                       m_collision_geometries,
                                       m_collision_objects));
        clone_cow->m_enabled = m_enabled;
        return clone_cow;
    }

    std::string m_name; // name of the collision object
    std::weak_ptr<Link> m_link;
    std::shared_ptr<Geometry> m_shape;
    SE3d m_shape_pose;
    std::shared_ptr<fcl::CollisionGeometry<double>> m_collision_geometries;
    std::shared_ptr<fcl::CollisionObject<double>> m_collision_objects;
    SE3d m_world_pose; /**< @brief Collision Object World Transformation */

protected:
    CollisionObjectWrapper(
        const std::string &name, const std::shared_ptr<Geometry> &shape,
        const SE3d &shape_pose,
        const std::shared_ptr<fcl::CollisionGeometry<double>>
            &collision_geometries,
        const std::shared_ptr<fcl::CollisionObject<double>> &collision_objects);
};

using Link2COW = std::map<std::string, CollisionObjectWrapperPtr>;
using Link2ConstCOW = std::map<std::string, CollisionObjectWrapperConstPtr>;

inline CollisionObjectWrapperPtr
CreateFCLCollisionObject(const std::string &name,
                         const std::shared_ptr<Geometry> &shape,
                         const SE3d &shape_pose, bool enabled)
{
    // dont add object that does not have geometry
    // todo

    CollisionObjectWrapperPtr new_cow =
        std::make_shared<CollisionObjectWrapper>(name, shape, shape_pose);

    new_cow->m_enabled = enabled;

    return new_cow;
}

bool CollisionCallback(fcl::CollisionObject<double> *o1,
                       fcl::CollisionObject<double> *o2, void *data);

bool DistanceCallback(fcl::CollisionObject<double> *o1,
                      fcl::CollisionObject<double> *o2, void *data,
                      double &min_dist);
/// @}
} // namespace RVS