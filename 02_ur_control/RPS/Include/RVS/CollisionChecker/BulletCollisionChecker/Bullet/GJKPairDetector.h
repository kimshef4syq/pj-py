// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h>
#include <BulletCollision/CollisionShapes/btCollisionMargin.h>
class btConvexShape;
#include <BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h>
class btConvexPenetrationDepthSolver;
RVS_COMMON_IGNORE_WARNINGS_POP
#include <RVS/CollisionChecker/BulletCollisionChecker/Core/Types.h>

namespace RVS
{
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
class TesseractGjkPairDetector : public btDiscreteCollisionDetectorInterface
{
    btVector3 m_cachedSeparatingAxis;
    btConvexPenetrationDepthSolver *m_penetrationDepthSolver;
    btSimplexSolverInterface *m_simplexSolver;
    const btConvexShape *m_minkowskiA;
    const btConvexShape *m_minkowskiB;
    int m_shapeTypeA;
    int m_shapeTypeB;
    btScalar m_marginA;
    btScalar m_marginB;

    bool m_ignoreMargin;
    btScalar m_cachedSeparatingDistance{0};

    const ContactTestData *m_cdata;

public:
    // some debugging to fix degeneracy problems
    int m_lastUsedMethod;
    int m_curIter{0};
    int m_degenerateSimplex{0};
    int m_catchDegeneracies;
    int m_fixContactNormalDirection;

    TesseractGjkPairDetector(
        const btConvexShape *objectA, const btConvexShape *objectB,
        btSimplexSolverInterface *simplexSolver,
        btConvexPenetrationDepthSolver *penetrationDepthSolver,
        const ContactTestData *cdata);

    TesseractGjkPairDetector(
        const btConvexShape *objectA, const btConvexShape *objectB,
        int shapeTypeA, int shapeTypeB, btScalar marginA, btScalar marginB,
        btSimplexSolverInterface *simplexSolver,
        btConvexPenetrationDepthSolver *penetrationDepthSolver,
        const ContactTestData *cdata);

    void getClosestPoints(const ClosestPointInput &input, Result &output,
                          class btIDebugDraw *debugDraw,
                          bool swapResults = false) override;

    void getClosestPointsNonVirtual(const ClosestPointInput &input,
                                    Result &output,
                                    class btIDebugDraw *debugDraw);

    void setMinkowskiA(const btConvexShape *minkA) { m_minkowskiA = minkA; }

    void setMinkowskiB(const btConvexShape *minkB) { m_minkowskiB = minkB; }
    void setCachedSeparatingAxis(const btVector3 &separatingAxis)
    {
        m_cachedSeparatingAxis = separatingAxis;
    }

    const btVector3 &getCachedSeparatingAxis() const
    {
        return m_cachedSeparatingAxis;
    }
    btScalar getCachedSeparatingDistance() const
    {
        return m_cachedSeparatingDistance;
    }

    void setPenetrationDepthSolver(
        btConvexPenetrationDepthSolver *penetrationDepthSolver)
    {
        m_penetrationDepthSolver = penetrationDepthSolver;
    }

    /// don't use setIgnoreMargin, it's for Bullet's internal use
    void setIgnoreMargin(bool ignoreMargin) { m_ignoreMargin = ignoreMargin; }
};
} // namespace RVS
