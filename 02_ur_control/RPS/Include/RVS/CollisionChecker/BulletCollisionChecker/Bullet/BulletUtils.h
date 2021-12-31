// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/Macros.h>
#include <RVS/Common/Logger.h>
#include <RVS/Environment/Geometry.h>
#include <RVS/CollisionChecker/BulletCollisionChecker/Core/Common.h>

RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <btBulletCollisionCommon.h>
RVS_COMMON_IGNORE_WARNINGS_POP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <iostream>
#include <string>
#include <map>
namespace RVS
{
/// @addtogroup CollisionChecker
/// @{


#define METERS

const btScalar BULLET_MARGIN = btScalar(0.0);
const btScalar BULLET_SUPPORT_FUNC_TOLERANCE = btScalar(0.01) METERS;
const btScalar BULLET_LENGTH_TOLERANCE = btScalar(0.001) METERS;
const btScalar BULLET_EPSILON = btScalar(1e-3);
const btScalar BULLET_DEFAULT_CONTACT_DISTANCE = btScalar(0.05);
const bool BULLET_COMPOUND_USE_DYNAMIC_AABB = true;

inline btVector3 convertEigenToBt(const Eigen::Vector3d &v)
{
    return btVector3{static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]),
                     static_cast<btScalar>(v[2])};
}

inline Eigen::Vector3d convertBtToEigen(const btVector3 &v)
{
    return Eigen::Vector3d{static_cast<double>(v.x()),
                           static_cast<double>(v.y()),
                           static_cast<double>(v.z())};
}

inline btQuaternion convertEigenToBt(const Eigen::Quaterniond &q)
{
    return btQuaternion{
        static_cast<btScalar>(q.x()), static_cast<btScalar>(q.y()),
        static_cast<btScalar>(q.z()), static_cast<btScalar>(q.w())};
}

inline btMatrix3x3 convertEigenToBt(const Eigen::Matrix3d &r)
{
    return btMatrix3x3{
        static_cast<btScalar>(r(0, 0)), static_cast<btScalar>(r(0, 1)),
        static_cast<btScalar>(r(0, 2)), static_cast<btScalar>(r(1, 0)),
        static_cast<btScalar>(r(1, 1)), static_cast<btScalar>(r(1, 2)),
        static_cast<btScalar>(r(2, 0)), static_cast<btScalar>(r(2, 1)),
        static_cast<btScalar>(r(2, 2))};
}

inline Eigen::Matrix3d convertBtToEigen(const btMatrix3x3 &r)
{
    Eigen::Matrix3d m;
    m << static_cast<double>(r[0][0]), static_cast<double>(r[0][1]),
        static_cast<double>(r[0][2]), static_cast<double>(r[1][0]),
        static_cast<double>(r[1][1]), static_cast<double>(r[1][2]),
        static_cast<double>(r[2][0]), static_cast<double>(r[2][1]),
        static_cast<double>(r[2][2]);
    return m;
}

inline btTransform convertEigenToBt(const Eigen::Isometry3d &t)
{
    const Eigen::Matrix3d &rot = t.matrix().block<3, 3>(0, 0);
    const Eigen::Vector3d &tran = t.translation();

    return btTransform{convertEigenToBt(rot), convertEigenToBt(tran)};
}

inline Eigen::Isometry3d convertBtToEigen(const btTransform &t)
{
    Eigen::Isometry3d i = Eigen::Isometry3d::Identity();
    i.linear() = convertBtToEigen(t.getBasis());
    i.translation() = convertBtToEigen(t.getOrigin());

    return i;
}

/**
 * @brief This is a tesseract bullet collsion object.
 *
 * It is a wrapper around bullet's collision object which
 * contains specific information related to tesseract
 */
class BulletCollisionObjectWrapper : public btCollisionObject
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<BulletCollisionObjectWrapper>;
    using ConstPtr = std::shared_ptr<const BulletCollisionObjectWrapper>;

    BulletCollisionObjectWrapper() = default;
    BulletCollisionObjectWrapper(std::string name, const int &type_id,
                                 std::shared_ptr<Geometry> shape,
                                 const SE3d &shape_pose);

    ~BulletCollisionObjectWrapper() {}

    short int m_collisionFilterGroup{btBroadphaseProxy::KinematicFilter};
    short int m_collisionFilterMask{btBroadphaseProxy::StaticFilter
                                    | btBroadphaseProxy::KinematicFilter};
    bool m_enabled{true};

    const std::string &getName() const { return m_name; }

    const int &getTypeID() const { return m_type_id; }

    const std::shared_ptr<Geometry> &GetCollisionGeometries() const
    {
        return m_shape;
    }

    const SE3d &GetCollisionGeometriesTransforms() const
    {
        return m_shape_pose;
    }

    /**
     * @brief Get the collision objects axis aligned bounding box
     * @param aabb_min The minimum point
     * @param aabb_max The maximum point
     */
    void getAABB(btVector3 &aabb_min, btVector3 &aabb_max) const
    {
        getCollisionShape()->getAabb(getWorldTransform(), aabb_min, aabb_max);
        const btScalar &d = getContactProcessingThreshold();
        btVector3 contactThreshold(d, d, d);
        aabb_min -= contactThreshold;
        aabb_max += contactThreshold;
    }

    /**
     * @brief This clones the collision objects but not the collision shape wich
     * is const.
     * @return Shared Pointer to the cloned collision object
     */
    std::shared_ptr<BulletCollisionObjectWrapper> clone()
    {
        auto clone_cow = std::make_shared<BulletCollisionObjectWrapper>();
        clone_cow->m_name = m_name;
        clone_cow->m_type_id = m_type_id;
        clone_cow->m_shape = m_shape;
        clone_cow->m_shape_pose = m_shape_pose;
        clone_cow->m_data = m_data;
        clone_cow->setCollisionShape(getCollisionShape());
        clone_cow->setWorldTransform(getWorldTransform());
        clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
        clone_cow->m_collisionFilterMask = m_collisionFilterMask;
        clone_cow->m_enabled = m_enabled;
        clone_cow->setBroadphaseHandle(nullptr);
        return clone_cow;
    }

    void manage(const std::shared_ptr<btCollisionShape> &t)
    {
        m_data.push_back(t);
    }

protected:
    std::string m_name; /**< @brief The name of the collision object */
    int m_type_id{-1}; /**< @brief A user defined type id */
    std::shared_ptr<Geometry> m_shape;
    SE3d m_shape_pose;

    /** @brief This manages the collision shape pointer so they get destroyed */
    std::vector<std::shared_ptr<btCollisionShape>> m_data;
};

using COW = BulletCollisionObjectWrapper;
using Link2Cow = std::map<std::string, COW::Ptr>;
using Link2ConstCow = std::map<std::string, COW::ConstPtr>;


struct CastHullShape : public btConvexShape
{
public:
    btConvexShape *m_shape;
    btTransform m_t01;

    CastHullShape(btConvexShape *shape, const btTransform &t01)
        : m_shape(shape), m_t01(t01)
    {
        m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;
        setUserIndex(m_shape->getUserIndex());
    }

    void updateCastTransform(const btTransform &t01) { m_t01 = t01; }
    btVector3 localGetSupportingVertex(const btVector3 &vec) const override
    {
        btVector3 sv0 = m_shape->localGetSupportingVertex(vec);
        btVector3 sv1 =
            m_t01 * m_shape->localGetSupportingVertex(vec * m_t01.getBasis());
        return (vec.dot(sv0) > vec.dot(sv1)) ? sv0 : sv1;
    }

    /// getAabb's default implementation is brute force, expected derived
    /// classes to implement a fast dedicated version
    void getAabb(const btTransform &t_w0, btVector3 &aabbMin,
                 btVector3 &aabbMax) const override
    {
        m_shape->getAabb(t_w0, aabbMin, aabbMax);
        btVector3 min1, max1;
        m_shape->getAabb(t_w0 * m_t01, min1, max1);
        aabbMin.setMin(min1);
        aabbMax.setMax(max1);
    }

    const char *getName() const override { return "CastHull"; }
    btVector3
    localGetSupportingVertexWithoutMargin(const btVector3 &v) const override
    {
        return localGetSupportingVertex(v);
    }

    // LCOV_EXCL_START
    // notice that the vectors should be unit length
    void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3 *,
                                                           btVector3 *,
                                                           int) const override
    {
        throw std::runtime_error(
            "If you are seeing this error message then something in Bullet "
            "must have changed. Attach "
            "a debugger and inspect the call stack to find the function in "
            "Bullet calling this "
            "function, then review commit history to determine what change.");
    }

    void getAabbSlow(const btTransform &, btVector3 &,
                     btVector3 &) const override
    {
        throw std::runtime_error(
            "If you are seeing this error message then something in Bullet "
            "must have changed. Attach "
            "a debugger and inspect the call stack to find the function in "
            "Bullet calling this "
            "function, then review commit history to determine what change.");
    }

    void setLocalScaling(const btVector3 &) override
    {
        throw std::runtime_error(
            "If you are seeing this error message then something in Bullet "
            "must have changed. Attach "
            "a debugger and inspect the call stack to find the function in "
            "Bullet calling this "
            "function, then review commit history to determine what change.");
    }

    const btVector3 &getLocalScaling() const override
    {
        static btVector3 out(1, 1, 1);
        return out;
    }

    void setMargin(btScalar) override {}

    btScalar getMargin() const override { return 0; }

    int getNumPreferredPenetrationDirections() const override { return 0; }

    void getPreferredPenetrationDirection(int, btVector3 &) const override
    {
        throw std::runtime_error(
            "If you are seeing this error message then something in Bullet "
            "must have changed. Attach "
            "a debugger and inspect the call stack to find the function in "
            "Bullet calling this "
            "function, then review commit history to determine what change.");
    }

    void calculateLocalInertia(btScalar, btVector3 &) const override
    {
        throw std::runtime_error(
            "If you are seeing this error message then something in Bullet "
            "must have changed. Attach "
            "a debugger and inspect the call stack to find the function in "
            "Bullet calling this "
            "function, then review commit history to determine what change.");
    }
    // LCOV_EXCL_STOP
};

inline void GetAverageSupport(const btConvexShape *shape,
                              const btVector3 &localNormal,
                              btScalar &outsupport, btVector3 &outpt)
{
    btVector3 ptSum(0, 0, 0);
    btScalar ptCount = 0;
    btScalar maxSupport = -1000;

    const auto *pshape = dynamic_cast<const btPolyhedralConvexShape *>(shape);
    if (pshape != nullptr) {
        int nPts = pshape->getNumVertices();

        for (int i = 0; i < nPts; ++i) {
            btVector3 pt;
            pshape->getVertex(i, pt);

            btScalar sup = pt.dot(localNormal);
            if (sup > maxSupport + BULLET_EPSILON) {
                ptCount = 1;
                ptSum = pt;
                maxSupport = sup;
            }
            else if (sup < maxSupport - BULLET_EPSILON) {
            }
            else {
                ptCount += 1;
                ptSum += pt;
            }
        }
        outsupport = maxSupport;
        outpt = ptSum / ptCount;
    }
    else {
        // The margins are set to zero for most shapes, but for a sphere the
        // margin is used so must use localGetSupportingVertex instead of
        // localGetSupportingVertexWithoutMargin.
        outpt = shape->localGetSupportingVertex(localNormal);
        outsupport = localNormal.dot(outpt);
    }
}

inline btTransform getLinkTransformFromCOW(const btCollisionObjectWrapper *cow)
{
    if (cow->m_parent != nullptr) {
        if (cow->m_parent->m_parent != nullptr) {
            assert(cow->m_parent->m_parent->m_parent == nullptr);
            return cow->m_parent->m_parent->getWorldTransform();
        }

        return cow->m_parent->getWorldTransform();
    }

    return cow->getWorldTransform();
}

inline bool needsCollisionCheck(const COW &cow1, const COW &cow2,
                                const IsContactAllowedFn &acm,
                                bool verbose = false)
{
    return cow1.m_enabled && cow2.m_enabled
           && (cow2.m_collisionFilterGroup & cow1.m_collisionFilterMask)
           && // NOLINT
           (cow1.m_collisionFilterGroup & cow2.m_collisionFilterMask)
           && // NOLINT
           !isContactAllowed(cow1.getName(), cow2.getName(), acm, verbose);
}

inline btScalar addDiscreteSingleResult(
    btManifoldPoint &cp, const btCollisionObjectWrapper *colObj0Wrap,
    const btCollisionObjectWrapper *colObj1Wrap, ContactTestData &collisions)
{
    assert(dynamic_cast<const BulletCollisionObjectWrapper *>(
               colObj0Wrap->getCollisionObject())
           != nullptr);
    assert(dynamic_cast<const BulletCollisionObjectWrapper *>(
               colObj1Wrap->getCollisionObject())
           != nullptr);
    const auto *cd0 = static_cast<const BulletCollisionObjectWrapper *>(
        colObj0Wrap->getCollisionObject()); // NOLINT
    const auto *cd1 = static_cast<const BulletCollisionObjectWrapper *>(
        colObj1Wrap->getCollisionObject()); // NOLINT

    ObjectPairKey pc = getObjectPairKey(cd0->getName(), cd1->getName());

    const auto &it = collisions.res->find(pc);
    bool found = (it != collisions.res->end());

    //    size_t l = 0;
    //    if (found)
    //    {
    //      l = it->second.size();
    //      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >=
    //      m_collisions.req->max_contacts_per_body)
    //          return 0;

    //    }

    btTransform tf0 = getLinkTransformFromCOW(colObj0Wrap);
    btTransform tf1 = getLinkTransformFromCOW(colObj1Wrap);
    btTransform tf0_inv = tf0.inverse();
    btTransform tf1_inv = tf1.inverse();

    ContactResult contact;
    contact.link_names[0] = cd0->getName();
    contact.link_names[1] = cd1->getName();
    contact.shape_id[0] = colObj0Wrap->getCollisionShape()->getUserIndex();
    contact.shape_id[1] = colObj1Wrap->getCollisionShape()->getUserIndex();
    contact.subshape_id[0] = colObj0Wrap->m_index;
    contact.subshape_id[1] = colObj1Wrap->m_index;
    contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
    contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
    contact.nearest_points_local[0] =
        convertBtToEigen(tf0_inv * cp.m_positionWorldOnA);
    contact.nearest_points_local[1] =
        convertBtToEigen(tf1_inv * cp.m_positionWorldOnB);
    contact.transform[0] = convertBtToEigen(tf0);
    contact.transform[1] = convertBtToEigen(tf1);
    contact.type_id[0] = cd0->getTypeID();
    contact.type_id[1] = cd1->getTypeID();
    contact.distance = static_cast<double>(cp.m_distance1);
    contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

    if (processResult(collisions, contact, pc, found) == nullptr) return 0;

    return 1;
}

inline void calculateContinuousData(ContactResult *col,
                                    const btCollisionObjectWrapper *cow,
                                    const btVector3 &pt_world,
                                    const btVector3 &normal_world,
                                    const btTransform &link_tf_inv,
                                    size_t link_index)
{
    assert(dynamic_cast<const CastHullShape *>(cow->getCollisionShape())
           != nullptr);
    const auto *shape =
        static_cast<const CastHullShape *>(cow->getCollisionShape());
    assert(shape != nullptr);

    // Get the start and final location of the shape
    btTransform shape_tfWorld0 = cow->getWorldTransform();
    btTransform shape_tfWorld1 = cow->getWorldTransform() * shape->m_t01;

    // Given the shapes final location calculate the links transform at the
    // final location
    Eigen::Isometry3d s =
        col->transform[link_index].inverse() * convertBtToEigen(shape_tfWorld0);
    col->cc_transform[link_index] =
        convertBtToEigen(shape_tfWorld1) * s.inverse();

    // Get the normal in the local shapes coordinate system at start and final
    // location
    btVector3 shape_normalLocal0 = normal_world * shape_tfWorld0.getBasis();
    btVector3 shape_normalLocal1 = normal_world * shape_tfWorld1.getBasis();

    // Calculate the contact point at the start location using the casted normal
    // vector in thapes local coordinate system
    btVector3 shape_ptLocal0;
    btScalar shape_localsup0{std::numeric_limits<btScalar>::max()};
    GetAverageSupport(shape->m_shape, shape_normalLocal0, shape_localsup0,
                      shape_ptLocal0);
    btVector3 shape_ptWorld0 = shape_tfWorld0 * shape_ptLocal0;

    // Calculate the contact point at the final location using the casted normal
    // vector in thapes local coordinate system
    btVector3 shape_ptLocal1;
    btScalar shape_localsup1{std::numeric_limits<btScalar>::max()};
    GetAverageSupport(shape->m_shape, shape_normalLocal1, shape_localsup1,
                      shape_ptLocal1);
    btVector3 shape_ptWorld1 = shape_tfWorld1 * shape_ptLocal1;

    btScalar shape_sup0 = normal_world.dot(shape_ptWorld0);
    btScalar shape_sup1 = normal_world.dot(shape_ptWorld1);

    // TODO: this section is potentially problematic. think hard about the math
    if (shape_sup0 - shape_sup1 > BULLET_SUPPORT_FUNC_TOLERANCE) {
        // LCOV_EXCL_START
        col->cc_time[link_index] = 0;
        col->cc_type[link_index] = ContinuousCollisionType::CCType_Time0;
        // LCOV_EXCL_STOP
    }
    else if (shape_sup1 - shape_sup0 > BULLET_SUPPORT_FUNC_TOLERANCE) {
        // LCOV_EXCL_START
        col->cc_time[link_index] = 1;
        col->cc_type[link_index] = ContinuousCollisionType::CCType_Time1;
        // LCOV_EXCL_STOP
    }
    else {
        // Given the contact point at the start and final location along with
        // the casted contact point the time between 0 and 1 can be calculated
        // along the path between the start and final location contact occurs.
        btScalar l0c = (pt_world - shape_ptWorld0).length();
        btScalar l1c = (pt_world - shape_ptWorld1).length();

        col->nearest_points_local[link_index] = convertBtToEigen(
            link_tf_inv
            * (shape_tfWorld0 * ((shape_ptLocal0 + shape_ptLocal1) / 2.0)));
        col->cc_type[link_index] = ContinuousCollisionType::CCType_Between;

        if (l0c + l1c < BULLET_LENGTH_TOLERANCE) {
            col->cc_time[link_index] = .5; // LCOV_EXCL_LINE
        }
        else {
            col->cc_time[link_index] = static_cast<double>(l0c / (l0c + l1c));
        }
    }
}

inline btScalar addCastSingleResult(btManifoldPoint &cp,
                                    const btCollisionObjectWrapper *colObj0Wrap,
                                    int,
                                    const btCollisionObjectWrapper *colObj1Wrap,
                                    int, ContactTestData &collisions)
{
    assert(dynamic_cast<const BulletCollisionObjectWrapper *>(
               colObj0Wrap->getCollisionObject())
           != nullptr);
    assert(dynamic_cast<const BulletCollisionObjectWrapper *>(
               colObj1Wrap->getCollisionObject())
           != nullptr);
    const auto *cd0 = static_cast<const BulletCollisionObjectWrapper *>(
        colObj0Wrap->getCollisionObject()); // NOLINT
    const auto *cd1 = static_cast<const BulletCollisionObjectWrapper *>(
        colObj1Wrap->getCollisionObject()); // NOLINT

    const std::pair<std::string, std::string> &pc =
        cd0->getName() < cd1->getName()
            ? std::make_pair(cd0->getName(), cd1->getName())
            : std::make_pair(cd1->getName(), cd0->getName());

    auto it = collisions.res->find(pc);
    bool found = it != collisions.res->end();

    //    size_t l = 0;
    //    if (found)
    //    {
    //      l = it->second.size();
    //      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >=
    //      m_collisions.req->max_contacts_per_body)
    //          return 0;
    //    }

    btTransform tf0 = getLinkTransformFromCOW(colObj0Wrap);
    btTransform tf1 = getLinkTransformFromCOW(colObj1Wrap);
    btTransform tf0_inv = tf0.inverse();
    btTransform tf1_inv = tf1.inverse();

    ContactResult contact;
    contact.link_names[0] = cd0->getName();
    contact.link_names[1] = cd1->getName();
    contact.shape_id[0] = colObj0Wrap->getCollisionShape()->getUserIndex();
    contact.shape_id[1] = colObj1Wrap->getCollisionShape()->getUserIndex();
    contact.subshape_id[0] = colObj0Wrap->m_index;
    contact.subshape_id[1] = colObj1Wrap->m_index;
    contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
    contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
    contact.nearest_points_local[0] =
        convertBtToEigen(tf0_inv * cp.m_positionWorldOnA);
    contact.nearest_points_local[1] =
        convertBtToEigen(tf1_inv * cp.m_positionWorldOnB);
    contact.transform[0] = convertBtToEigen(tf0);
    contact.transform[1] = convertBtToEigen(tf1);
    contact.type_id[0] = cd0->getTypeID();
    contact.type_id[1] = cd1->getTypeID();
    contact.distance = static_cast<double>(cp.m_distance1);
    contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

    ContactResult *col = processResult(collisions, contact, pc, found);
    if (col == nullptr) return 0;

    if (cd0->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter
        && cd1->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter) {
        calculateContinuousData(col, colObj0Wrap, cp.m_positionWorldOnA,
                                -1 * cp.m_normalWorldOnB, tf0_inv, 0);
        calculateContinuousData(col, colObj1Wrap, cp.m_positionWorldOnB,
                                cp.m_normalWorldOnB, tf1_inv, 1);
    }
    else {
        bool castShapeIsFirst =
            (cd0->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
                ? true
                : false;
        btVector3 normalWorldFromCast =
            -(castShapeIsFirst ? 1 : -1) * cp.m_normalWorldOnB;
        const btCollisionObjectWrapper *firstColObjWrap =
            (castShapeIsFirst ? colObj0Wrap : colObj1Wrap);
        const btTransform &first_tf_inv =
            (castShapeIsFirst ? tf0_inv : tf1_inv);
        const btVector3 &ptOnCast =
            castShapeIsFirst ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;

        if (castShapeIsFirst) {
            std::swap(col->nearest_points[0], col->nearest_points[1]);
            std::swap(col->nearest_points_local[0],
                      col->nearest_points_local[1]);
            std::swap(col->transform[0], col->transform[1]);
            std::swap(col->link_names[0], col->link_names[1]);
            std::swap(col->type_id[0], col->type_id[1]);
            std::swap(col->shape_id[0], col->shape_id[1]);
            std::swap(col->subshape_id[0], col->subshape_id[1]);
            col->normal *= -1;
        }

        calculateContinuousData(col, firstColObjWrap, ptOnCast,
                                normalWorldFromCast, first_tf_inv, 1);
    }

    return 1;
}

struct TesseractBridgedManifoldResult : public btManifoldResult
{
    btCollisionWorld::ContactResultCallback &m_resultCallback;

    TesseractBridgedManifoldResult(
        const btCollisionObjectWrapper *obj0Wrap,
        const btCollisionObjectWrapper *obj1Wrap,
        btCollisionWorld::ContactResultCallback &resultCallback)
        : btManifoldResult(obj0Wrap, obj1Wrap), m_resultCallback(resultCallback)
    {
    }

    void addContactPoint(const btVector3 &normalOnBInWorld,
                         const btVector3 &pointInWorld, btScalar depth) override
    {
        bool isSwapped =
            m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
        btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
        btVector3 localA;
        btVector3 localB;
        if (isSwapped) {
            localA =
                m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(
                    pointA);
            localB =
                m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(
                    pointInWorld);
        }
        else {
            localA =
                m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(
                    pointA);
            localB =
                m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(
                    pointInWorld);
        }

        btManifoldPoint newPt(localA, localB, normalOnBInWorld, depth);
        newPt.m_positionWorldOnA = pointA;
        newPt.m_positionWorldOnB = pointInWorld;

        // BP mod, store contact triangles.
        if (isSwapped) {
            newPt.m_partId0 = m_partId1;
            newPt.m_partId1 = m_partId0;
            newPt.m_index0 = m_index1;
            newPt.m_index1 = m_index0;
        }
        else {
            newPt.m_partId0 = m_partId0;
            newPt.m_partId1 = m_partId1;
            newPt.m_index0 = m_index0;
            newPt.m_index1 = m_index1;
        }

        // experimental feature info, for per-triangle material etc.
        const btCollisionObjectWrapper *obj0Wrap =
            isSwapped ? m_body1Wrap : m_body0Wrap;
        const btCollisionObjectWrapper *obj1Wrap =
            isSwapped ? m_body0Wrap : m_body1Wrap;
        m_resultCallback.addSingleResult(newPt, obj0Wrap, newPt.m_partId0,
                                         newPt.m_index0, obj1Wrap,
                                         newPt.m_partId1, newPt.m_index1);
    }
};

struct BroadphaseContactResultCallback
{
    ContactTestData &collisions_;
    double contact_distance_;
    bool verbose_;

    BroadphaseContactResultCallback(ContactTestData &collisions,
                                    double contact_distance,
                                    bool verbose = false)
        : collisions_(collisions), contact_distance_(contact_distance),
          verbose_(verbose)
    {
    }

    virtual ~BroadphaseContactResultCallback() = default;
    BroadphaseContactResultCallback(const BroadphaseContactResultCallback &) =
        default;
    BroadphaseContactResultCallback &
    operator=(const BroadphaseContactResultCallback &) = delete;
    BroadphaseContactResultCallback(BroadphaseContactResultCallback &&) =
        default;
    BroadphaseContactResultCallback &
    operator=(BroadphaseContactResultCallback &&) = delete;

    virtual bool needsCollision(const BulletCollisionObjectWrapper *cow0,
                                const BulletCollisionObjectWrapper *cow1) const
    {
        return !collisions_.done
               && needsCollisionCheck(*cow0, *cow1, collisions_.fn, verbose_);
    }

    virtual btScalar
    addSingleResult(btManifoldPoint &cp,
                    const btCollisionObjectWrapper *colObj0Wrap, int partId0,
                    int index0, const btCollisionObjectWrapper *colObj1Wrap,
                    int partId1, int index1) = 0;
};

struct DiscreteBroadphaseContactResultCallback
    : public BroadphaseContactResultCallback
{
    DiscreteBroadphaseContactResultCallback(ContactTestData &collisions,
                                            double contact_distance,
                                            bool verbose = false)
        : BroadphaseContactResultCallback(collisions, contact_distance, verbose)
    {
    }

    btScalar addSingleResult(btManifoldPoint &cp,
                             const btCollisionObjectWrapper *colObj0Wrap, int,
                             int, const btCollisionObjectWrapper *colObj1Wrap,
                             int, int) override
    {
        if (cp.m_distance1 > static_cast<btScalar>(contact_distance_)) return 0;

        return addDiscreteSingleResult(cp, colObj0Wrap, colObj1Wrap,
                                       collisions_);
    }
};

struct CastBroadphaseContactResultCallback
    : public BroadphaseContactResultCallback
{
    CastBroadphaseContactResultCallback(ContactTestData &collisions,
                                        double contact_distance,
                                        bool verbose = false)
        : BroadphaseContactResultCallback(collisions, contact_distance, verbose)
    {
    }

    btScalar addSingleResult(btManifoldPoint &cp,
                             const btCollisionObjectWrapper *colObj0Wrap, int,
                             int index0,
                             const btCollisionObjectWrapper *colObj1Wrap, int,
                             int index1) override
    {
        if (cp.m_distance1 > static_cast<btScalar>(contact_distance_)) return 0;

        return addCastSingleResult(cp, colObj0Wrap, index0, colObj1Wrap, index1,
                                   collisions_);
    }
};

struct TesseractBroadphaseBridgedManifoldResult : public btManifoldResult
{
    BroadphaseContactResultCallback &result_callback_;

    TesseractBroadphaseBridgedManifoldResult(
        const btCollisionObjectWrapper *obj0Wrap,
        const btCollisionObjectWrapper *obj1Wrap,
        BroadphaseContactResultCallback &result_callback)
        : btManifoldResult(obj0Wrap, obj1Wrap),
          result_callback_(result_callback)
    {
    }

    void addContactPoint(const btVector3 &normalOnBInWorld,
                         const btVector3 &pointInWorld, btScalar depth) override
    {
        if (result_callback_.collisions_.done
            || depth
                   > static_cast<btScalar>(result_callback_.contact_distance_))
            return;

        bool isSwapped =
            m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
        btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
        btVector3 localA;
        btVector3 localB;
        if (isSwapped) {
            localA =
                m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(
                    pointA);
            localB =
                m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(
                    pointInWorld);
        }
        else {
            localA =
                m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(
                    pointA);
            localB =
                m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(
                    pointInWorld);
        }

        btManifoldPoint newPt(localA, localB, normalOnBInWorld, depth);
        newPt.m_positionWorldOnA = pointA;
        newPt.m_positionWorldOnB = pointInWorld;

        // BP mod, store contact triangles.
        if (isSwapped) {
            newPt.m_partId0 = m_partId1;
            newPt.m_partId1 = m_partId0;
            newPt.m_index0 = m_index1;
            newPt.m_index1 = m_index0;
        }
        else {
            newPt.m_partId0 = m_partId0;
            newPt.m_partId1 = m_partId1;
            newPt.m_index0 = m_index0;
            newPt.m_index1 = m_index1;
        }

        // experimental feature info, for per-triangle material etc.
        const btCollisionObjectWrapper *obj0Wrap =
            isSwapped ? m_body1Wrap : m_body0Wrap;
        const btCollisionObjectWrapper *obj1Wrap =
            isSwapped ? m_body0Wrap : m_body1Wrap;
        result_callback_.addSingleResult(newPt, obj0Wrap, newPt.m_partId0,
                                         newPt.m_index0, obj1Wrap,
                                         newPt.m_partId1, newPt.m_index1);
    }
};

class TesseractCollisionPairCallback : public btOverlapCallback
{
    const btDispatcherInfo &dispatch_info_;
    btCollisionDispatcher *dispatcher_;
    BroadphaseContactResultCallback &results_callback_;

public:
    TesseractCollisionPairCallback(
        const btDispatcherInfo &dispatchInfo, btCollisionDispatcher *dispatcher,
        BroadphaseContactResultCallback &results_callback)
        : dispatch_info_(dispatchInfo), dispatcher_(dispatcher),
          results_callback_(results_callback)
    {
    }

    ~TesseractCollisionPairCallback() override = default;
    TesseractCollisionPairCallback(const TesseractCollisionPairCallback &) =
        default;
    TesseractCollisionPairCallback &
    operator=(const TesseractCollisionPairCallback &) = delete;
    TesseractCollisionPairCallback(TesseractCollisionPairCallback &&) = default;
    TesseractCollisionPairCallback &
    operator=(TesseractCollisionPairCallback &&) = delete;

    bool processOverlap(btBroadphasePair &pair) override
    {
        if (results_callback_.collisions_.done) return false;

        const auto *cow0 = static_cast<const BulletCollisionObjectWrapper *>(
            pair.m_pProxy0->m_clientObject);
        const auto *cow1 = static_cast<const BulletCollisionObjectWrapper *>(
            pair.m_pProxy1->m_clientObject);

        if (results_callback_.needsCollision(cow0, cow1)) {
            btCollisionObjectWrapper obj0Wrap(
                nullptr, cow0->getCollisionShape(), cow0,
                cow0->getWorldTransform(), -1, -1);
            btCollisionObjectWrapper obj1Wrap(
                nullptr, cow1->getCollisionShape(), cow1,
                cow1->getWorldTransform(), -1, -1);

            // dispatcher will keep algorithms persistent in the collision pair
            if (pair.m_algorithm == nullptr) {
                pair.m_algorithm = dispatcher_->findAlgorithm(
                    &obj0Wrap, &obj1Wrap, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
            }

            if (pair.m_algorithm != nullptr) {
                TesseractBroadphaseBridgedManifoldResult contactPointResult(
                    &obj0Wrap, &obj1Wrap, results_callback_);
                contactPointResult.m_closestPointDistanceThreshold =
                    static_cast<btScalar>(results_callback_.contact_distance_);

                // discrete collision detection query
                pair.m_algorithm->processCollision(
                    &obj0Wrap, &obj1Wrap, dispatch_info_, &contactPointResult);
            }
        }
        return false;
    }
};


class TesseractOverlapFilterCallback : public btOverlapFilterCallback
{
public:
    TesseractOverlapFilterCallback(bool verbose = false) : verbose_(verbose) {}

    bool needBroadphaseCollision(btBroadphaseProxy *proxy0,
                                 btBroadphaseProxy *proxy1) const override
    {
        // Note: We do not pass the allowed collision matrix because if it
        // changes we do not know and this function only gets called under
        // certain cases and it could cause overlapping pairs to not be
        // processed.
        return needsCollisionCheck(
            *(static_cast<BulletCollisionObjectWrapper *>(
                proxy0->m_clientObject)),
            *(static_cast<BulletCollisionObjectWrapper *>(
                proxy1->m_clientObject)),
            nullptr, verbose_);
    }

private:
    bool verbose_{false};
};


inline void updateCollisionObjectFilters(const std::vector<std::string> &active,
                                         const COW::Ptr &cow)
{
    cow->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;

    if (!isLinkActive(active, cow->getName())) {
        cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
    }

    if (cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter) {
        cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else {
        cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter
                                     | btBroadphaseProxy::KinematicFilter;
    }
}

inline COW::Ptr createCollisionObject(const std::string &name,
                                      const int &type_id,
                                      std::shared_ptr<Geometry> shape,
                                      const SE3d &shape_pose,
                                      bool enabled = true)
{
    std::shared_ptr<BulletCollisionObjectWrapper> new_cow =
        std::make_shared<BulletCollisionObjectWrapper>(name, type_id, shape,
                                                       shape_pose);

    new_cow->m_enabled = enabled;
    new_cow->setContactProcessingThreshold(BULLET_DEFAULT_CONTACT_DISTANCE);

    return new_cow;
}

struct DiscreteCollisionCollector
    : public btCollisionWorld::ContactResultCallback
{
    ContactTestData &collisions_;
    const COW::Ptr cow_;
    double contact_distance_;
    bool verbose_;

    DiscreteCollisionCollector(ContactTestData &collisions, COW::Ptr cow,
                               btScalar contact_distance, bool verbose = false)
        : collisions_(collisions), cow_(std::move(cow)),
          contact_distance_(contact_distance), verbose_(verbose)
    {
        m_closestDistanceThreshold = contact_distance;
        m_collisionFilterGroup = cow_->m_collisionFilterGroup;
        m_collisionFilterMask = cow_->m_collisionFilterMask;
    }

    btScalar addSingleResult(btManifoldPoint &cp,
                             const btCollisionObjectWrapper *colObj0Wrap, int,
                             int, const btCollisionObjectWrapper *colObj1Wrap,
                             int, int) override
    {
        if (cp.m_distance1 > static_cast<btScalar>(contact_distance_)) return 0;

        return addDiscreteSingleResult(cp, colObj0Wrap, colObj1Wrap,
                                       collisions_);
    }

    bool needsCollision(btBroadphaseProxy *proxy0) const override
    {
        return !collisions_.done
               && needsCollisionCheck(
                   *cow_,
                   *(static_cast<BulletCollisionObjectWrapper *>(
                       proxy0->m_clientObject)),
                   collisions_.fn, verbose_);
    }
};

struct CastCollisionCollector : public btCollisionWorld::ContactResultCallback
{
    ContactTestData &collisions_;
    const COW::Ptr cow_;
    double contact_distance_;
    bool verbose_;

    CastCollisionCollector(ContactTestData &collisions, COW::Ptr cow,
                           double contact_distance, bool verbose = false)
        : collisions_(collisions), cow_(std::move(cow)),
          contact_distance_(contact_distance), verbose_(verbose)
    {
        m_closestDistanceThreshold = static_cast<btScalar>(contact_distance);
        m_collisionFilterGroup = cow_->m_collisionFilterGroup;
        m_collisionFilterMask = cow_->m_collisionFilterMask;
    }

    btScalar addSingleResult(btManifoldPoint &cp,
                             const btCollisionObjectWrapper *colObj0Wrap, int,
                             int index0,
                             const btCollisionObjectWrapper *colObj1Wrap, int,
                             int index1) override
    {
        if (cp.m_distance1 > static_cast<btScalar>(contact_distance_)) return 0;

        return addCastSingleResult(cp, colObj0Wrap, index0, colObj1Wrap, index1,
                                   collisions_);
    }

    bool needsCollision(btBroadphaseProxy *proxy0) const override
    {
        return !collisions_.done
               && needsCollisionCheck(
                   *cow_,
                   *(static_cast<BulletCollisionObjectWrapper *>(
                       proxy0->m_clientObject)),
                   collisions_.fn, verbose_);
    }
};

inline COW::Ptr makeCastCollisionObject(const COW::Ptr &cow)
{
    COW::Ptr new_cow = cow->clone();

    btTransform tf;
    tf.setIdentity();

    if (btBroadphaseProxy::isConvex(
            new_cow->getCollisionShape()->getShapeType())) {
        assert(dynamic_cast<btConvexShape *>(new_cow->getCollisionShape())
               != nullptr);
        auto *convex =
            static_cast<btConvexShape *>(new_cow->getCollisionShape());
        assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

        auto shape = std::make_shared<CastHullShape>(convex, tf);
        assert(shape != nullptr);

        new_cow->manage(shape);
        new_cow->setCollisionShape(shape.get());
    }
    else if (btBroadphaseProxy::isCompound(
                 new_cow->getCollisionShape()->getShapeType())) {
        assert(dynamic_cast<btCompoundShape *>(new_cow->getCollisionShape())
               != nullptr);
        auto *compound =
            static_cast<btCompoundShape *>(new_cow->getCollisionShape());
        auto new_compound = std::make_shared<btCompoundShape>(
            BULLET_COMPOUND_USE_DYNAMIC_AABB, compound->getNumChildShapes());

        for (int i = 0; i < compound->getNumChildShapes(); ++i) {
            if (btBroadphaseProxy::isConvex(
                    compound->getChildShape(i)->getShapeType())) {
                auto *convex =
                    static_cast<btConvexShape *>(compound->getChildShape(i));
                assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

                btTransform geomTrans = compound->getChildTransform(i);

                auto subshape = std::make_shared<CastHullShape>(convex, tf);
                assert(subshape != nullptr);

                new_cow->manage(subshape);
                subshape->setMargin(BULLET_MARGIN);
                new_compound->addChildShape(geomTrans, subshape.get());
            }
            else if (btBroadphaseProxy::isCompound(
                         compound->getChildShape(i)->getShapeType())) {
                auto *second_compound =
                    static_cast<btCompoundShape *>(compound->getChildShape(i));
                auto new_second_compound = std::make_shared<btCompoundShape>(
                    BULLET_COMPOUND_USE_DYNAMIC_AABB,
                    second_compound->getNumChildShapes());
                for (int j = 0; j < second_compound->getNumChildShapes(); ++j) {
                    assert(!btBroadphaseProxy::isCompound(
                        second_compound->getChildShape(j)->getShapeType()));
                    assert(dynamic_cast<btConvexShape *>(
                               second_compound->getChildShape(j))
                           != nullptr);

                    auto *convex = static_cast<btConvexShape *>(
                        second_compound->getChildShape(j));
                    assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

                    btTransform geomTrans =
                        second_compound->getChildTransform(j);

                    auto subshape = std::make_shared<CastHullShape>(convex, tf);
                    assert(subshape != nullptr);

                    new_cow->manage(subshape);
                    subshape->setMargin(BULLET_MARGIN);
                    new_second_compound->addChildShape(geomTrans,
                                                       subshape.get());
                }

                btTransform geomTrans = compound->getChildTransform(i);

                new_cow->manage(new_second_compound);
                new_second_compound->setMargin(BULLET_MARGIN);

                new_compound->addChildShape(geomTrans,
                                            new_second_compound.get());
            }
            else {
                throw std::runtime_error(
                    "I can only collision check convex shapes and compound "
                    "shapes made of convex shapes");
            }
        }

        new_compound->setMargin(BULLET_MARGIN);
        new_cow->manage(new_compound);
        new_cow->setCollisionShape(new_compound.get());
        new_cow->setWorldTransform(cow->getWorldTransform());
    }
    else {
        throw std::runtime_error("I can only collision check convex shapes and "
                                 "compound shapes made of convex shapes");
    }

    return new_cow;
}


inline void
updateBroadphaseAABB(const COW::Ptr &cow,
                     const std::unique_ptr<btBroadphaseInterface> &broadphase,
                     const std::unique_ptr<btCollisionDispatcher> &dispatcher)
{
    // Calculate the aabb
    btVector3 aabb_min, aabb_max;
    cow->getAABB(aabb_min, aabb_max);

    // Update the broadphase aabb
    assert(cow->getBroadphaseHandle() != nullptr);
    broadphase->setAabb(cow->getBroadphaseHandle(), aabb_min, aabb_max,
                        dispatcher.get());
}


inline void removeCollisionObjectFromBroadphase(
    const COW::Ptr &cow,
    const std::unique_ptr<btBroadphaseInterface> &broadphase,
    const std::unique_ptr<btCollisionDispatcher> &dispatcher)
{
    btBroadphaseProxy *bp = cow->getBroadphaseHandle();
    if (bp != nullptr) {
        // only clear the cached algorithms
        broadphase->getOverlappingPairCache()->cleanProxyFromPairs(
            bp, dispatcher.get());
        broadphase->destroyProxy(bp, dispatcher.get());
        cow->setBroadphaseHandle(nullptr);
    }
}


inline void addCollisionObjectToBroadphase(
    const COW::Ptr &cow,
    const std::unique_ptr<btBroadphaseInterface> &broadphase,
    const std::unique_ptr<btCollisionDispatcher> &dispatcher)
{
    btVector3 aabb_min, aabb_max;
    cow->getAABB(aabb_min, aabb_max);

    // Add the active collision object to the broadphase
    int type = cow->getCollisionShape()->getShapeType();
    cow->setBroadphaseHandle(broadphase->createProxy(
        aabb_min, aabb_max, type, cow.get(), cow->m_collisionFilterGroup,
        cow->m_collisionFilterMask, dispatcher.get()));
}


inline void updateCollisionObjectFilters(
    const std::vector<std::string> &active, const COW::Ptr &cow,
    const std::unique_ptr<btBroadphaseInterface> &broadphase,
    const std::unique_ptr<btCollisionDispatcher> &dispatcher)
{
    updateCollisionObjectFilters(active, cow);

    // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets
    // called again. The BroadPhaseFilter only gets called once, so if you
    // change when two objects can be in collision, like filters this must be
    // called or contacts between shapes will be missed.
    broadphase->getOverlappingPairCache()->cleanProxyFromPairs(
        cow->getBroadphaseHandle(), dispatcher.get());
}


inline void
refreshBroadphaseProxy(const COW::Ptr &cow,
                       const std::unique_ptr<btBroadphaseInterface> &broadphase,
                       const std::unique_ptr<btCollisionDispatcher> &dispatcher)
{
    if (cow->getBroadphaseHandle() != nullptr) {
        broadphase->destroyProxy(cow->getBroadphaseHandle(), dispatcher.get());

        btVector3 aabb_min, aabb_max;
        cow->getAABB(aabb_min, aabb_max);

        // Add the active collision object to the broadphase
        int type = cow->getCollisionShape()->getShapeType();
        cow->setBroadphaseHandle(broadphase->createProxy(
            aabb_min, aabb_max, type, cow.get(), cow->m_collisionFilterGroup,
            cow->m_collisionFilterMask, dispatcher.get()));
    }
}

} // namespace RVS