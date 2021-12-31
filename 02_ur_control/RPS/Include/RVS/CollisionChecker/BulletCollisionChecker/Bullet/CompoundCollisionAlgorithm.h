// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/BroadphaseCollision/btDispatcher.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseInterface.h>
#include <BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h>
#include <BulletCollision/NarrowPhaseCollision/btPersistentManifold.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletCollision/CollisionDispatch/btCollisionCreateFunc.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <BulletCollision/BroadphaseCollision/btDbvt.h>
RVS_COMMON_IGNORE_WARNINGS_POP

class btCollisionObject;
class btCollisionShape;

// LCOV_EXCL_START
namespace RVS
{
/**
 * @brief Supports collision between CompoundCollisionShapes and other collision
 * shapes
 *
 * The original implementation would check all collision objects before exiting
 * the bvh of the compound shape. The original code had a callback, but it only
 * passed in the collision shape and no the collision object which is where the
 * user data is located. This was modifed to check if collision is done for the
 * contact test type FIRST during the internal broadphase of the compound shapes
 * and exit early.
 *
 * Note: This could be removed in the future but the callback need to be modifed
 * to accept the collision object along with the collision shape. I don't
 * believe this will be an issue since all of the other callback in Bullet
 * accept both.
 */
class TesseractCompoundCollisionAlgorithm
    : public btActivatingCollisionAlgorithm // NOLINT
{
    btNodeStack stack2;
    btManifoldArray manifoldArray;

protected:
    btAlignedObjectArray<btCollisionAlgorithm *> m_childCollisionAlgorithms;
    bool m_isSwapped;

    class btPersistentManifold *m_sharedManifold;
    bool m_ownsManifold;

    int m_compoundShapeRevision; // to keep track of changes, so that
                                 // childAlgorithm array can be updated

    void removeChildAlgorithms();

    void preallocateChildAlgorithms(const btCollisionObjectWrapper *body0Wrap,
                                    const btCollisionObjectWrapper *body1Wrap);

public:
    TesseractCompoundCollisionAlgorithm(
        const btCollisionAlgorithmConstructionInfo &ci,
        const btCollisionObjectWrapper *body0Wrap,
        const btCollisionObjectWrapper *body1Wrap, bool isSwapped);

    ~TesseractCompoundCollisionAlgorithm() override;
    TesseractCompoundCollisionAlgorithm(
        const TesseractCompoundCollisionAlgorithm &) = default;
    TesseractCompoundCollisionAlgorithm &
    operator=(const TesseractCompoundCollisionAlgorithm &) = default;
    TesseractCompoundCollisionAlgorithm(
        TesseractCompoundCollisionAlgorithm &&) = default;
    TesseractCompoundCollisionAlgorithm &
    operator=(TesseractCompoundCollisionAlgorithm &&) = default;

    btCollisionAlgorithm *getChildAlgorithm(int n) const
    {
        return m_childCollisionAlgorithms[n];
    }

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
        for (int i = 0; i < m_childCollisionAlgorithms.size(); i++) {
            if (m_childCollisionAlgorithms[i] != nullptr)
                m_childCollisionAlgorithms[i]->getAllContactManifolds(
                    manifoldArray);
        }
    }

    struct CreateFunc : public btCollisionAlgorithmCreateFunc
    {
        btCollisionAlgorithm *CreateCollisionAlgorithm(
            btCollisionAlgorithmConstructionInfo &ci,
            const btCollisionObjectWrapper *body0Wrap,
            const btCollisionObjectWrapper *body1Wrap) override
        {
            void *mem = ci.m_dispatcher1->allocateCollisionAlgorithm(
                sizeof(TesseractCompoundCollisionAlgorithm));
            return new (mem) TesseractCompoundCollisionAlgorithm(
                ci, body0Wrap, body1Wrap, false);
        }
    };

    struct SwappedCreateFunc : public btCollisionAlgorithmCreateFunc
    {
        btCollisionAlgorithm *CreateCollisionAlgorithm(
            btCollisionAlgorithmConstructionInfo &ci,
            const btCollisionObjectWrapper *body0Wrap,
            const btCollisionObjectWrapper *body1Wrap) override
        {
            void *mem = ci.m_dispatcher1->allocateCollisionAlgorithm(
                sizeof(TesseractCompoundCollisionAlgorithm));
            return new (mem) TesseractCompoundCollisionAlgorithm(
                ci, body0Wrap, body1Wrap, true);
        }
    };
};
} // namespace RVS
