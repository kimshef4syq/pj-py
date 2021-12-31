// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPersistentManifold.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/CollisionDispatch/btCollisionCreateFunc.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <LinearMath/btTransformUtil.h>
#include <BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.h>
RVS_COMMON_IGNORE_WARNINGS_POP

#include <RVS/CollisionChecker/BulletCollisionChecker/Core/Types.h>

class btConvexPenetrationDepthSolver;

// LCOV_EXCL_START
namespace RVS
{

/// Enabling USE_SEPDISTANCE_UTIL2 requires 100% reliable distance computation.
/// However, when using large size ratios GJK can be imprecise so the distance
/// is not conservative. In that case, enabling this USE_SEPDISTANCE_UTIL2 would
/// result in failing/missing collisions. Either improve GJK for large size
/// ratios (testing a 100 units versus a 0.1 unit object) or only enable the
/// util for certain pairs that have a small size ratio

//#define USE_SEPDISTANCE_UTIL2 1

/// The convexConvexAlgorithm collision algorithm implements time of impact,
/// convex closest points and penetration depth calculations between two convex
/// objects. Multiple contact points are calculated by perturbing the
/// orientation of the smallest object orthogonal to the separating normal. This
/// idea was described by Gino van den Bergen in this forum topic
/// http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=4&t=288&p=888#p888

/**
 * @brief This is a modifed Convex to Convex collision algorithm
 *
 * This was modified to leverage the Tesseract contact request to enable and
 * disable different parts of the algorithm. The algorithm first does a quick
 * binary check if the two objects are in collision. If in collision it runs the
 * penetration algorithm to get the nearest points and penetration depths. If
 * not in collision it runs the an algorithm to get the nearest points and
 * distance. The Tesseract contact request allow you to now decide if you need
 * all three. Example, in the case of OMPL if you have a contact distance of
 * zero you can get a performance increase, by disabling the penetration and
 * distance calculation because they add no value.
 *
 * Note: This will not be able to be removed.
 */
class TesseractConvexConvexAlgorithm : public btActivatingCollisionAlgorithm
{
#ifdef USE_SEPDISTANCE_UTIL2
    btConvexSeparatingDistanceUtil m_sepDistance;
#endif
    btConvexPenetrationDepthSolver *m_pdSolver;

    btVertexArray worldVertsB1;
    btVertexArray worldVertsB2;

    bool m_ownManifold;
    btPersistentManifold *m_manifoldPtr;
    bool m_lowLevelOfDetail;

    int m_numPerturbationIterations;
    int m_minimumPointsPerturbationThreshold;

    ContactTestData *m_cdata;

    /// cache separating vector to speedup collision detection

public:
    TesseractConvexConvexAlgorithm(
        btPersistentManifold *mf,
        const btCollisionAlgorithmConstructionInfo &ci,
        const btCollisionObjectWrapper *body0Wrap,
        const btCollisionObjectWrapper *body1Wrap,
        btConvexPenetrationDepthSolver *pdSolver, int numPerturbationIterations,
        int minimumPointsPerturbationThreshold);

    ~TesseractConvexConvexAlgorithm() override;
    TesseractConvexConvexAlgorithm(const TesseractConvexConvexAlgorithm &) =
        delete;
    TesseractConvexConvexAlgorithm &
    operator=(const TesseractConvexConvexAlgorithm &) = delete;
    TesseractConvexConvexAlgorithm(TesseractConvexConvexAlgorithm &&) = delete;
    TesseractConvexConvexAlgorithm &
    operator=(TesseractConvexConvexAlgorithm &&) = delete;

    void processCollision(const btCollisionObjectWrapper *body0Wrap,
                          const btCollisionObjectWrapper *body1Wrap,
                          const btDispatcherInfo &dispatchInfo,
                          btManifoldResult *resultOut) override;

    btScalar calculateTimeOfImpact(btCollisionObject *body0,
                                   btCollisionObject *body1,
                                   const btDispatcherInfo &dispatchInfo,
                                   btManifoldResult *resultOut) override;

    void getAllContactManifolds(btManifoldArray &manifoldArray) override
    {
        /// should we use m_ownManifold to avoid adding duplicates?
        if (m_manifoldPtr && m_ownManifold) // NOLINT
            manifoldArray.push_back(m_manifoldPtr);
    }

    void setLowLevelOfDetail(bool useLowLevel);

    const btPersistentManifold *getManifold() { return m_manifoldPtr; }

    struct CreateFunc : public btCollisionAlgorithmCreateFunc
    {
        btConvexPenetrationDepthSolver *m_pdSolver;
        int m_numPerturbationIterations;
        int m_minimumPointsPerturbationThreshold;

        CreateFunc(btConvexPenetrationDepthSolver *pdSolver);

        btCollisionAlgorithm *CreateCollisionAlgorithm(
            btCollisionAlgorithmConstructionInfo &ci,
            const btCollisionObjectWrapper *body0Wrap,
            const btCollisionObjectWrapper *body1Wrap) override
        {
            void *mem = ci.m_dispatcher1->allocateCollisionAlgorithm(
                sizeof(TesseractConvexConvexAlgorithm));
            return new (mem) TesseractConvexConvexAlgorithm(
                ci.m_manifold, ci, body0Wrap, body1Wrap, m_pdSolver,
                m_numPerturbationIterations,
                m_minimumPointsPerturbationThreshold);
        }
    };
};

} // namespace RVS
