// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include <RVS/Common/Time.h>
#include <RVS/CollisionChecker/CollisionCheckerBase.h>
#include <RVS/CollisionChecker/BulletCollisionChecker/Bullet/BulletDiscreteBVHManager.h>


namespace RVS
{
///@addtogroup CollisionChecker
///@{

RVS_CLASS_FORWARD(BulletDiscreteBVHChecker);

/// @brief CollisionChecker use Bullet lib
class BulletDiscreteBVHChecker : public CollisionCheckerBase
{
public:
    BulletDiscreteBVHChecker();

    virtual ~BulletDiscreteBVHChecker();

    static std::string GetName() { return "BulletDiscreteBVHChecker"; }

    static CollisionCheckerBasePtr Create()
    {
        return std::make_shared<BulletDiscreteBVHChecker>();
    }

    CollisionCheckerBasePtr Clone() const override;

    virtual bool InitFromEnv(std::shared_ptr<Environment> env) override;

    virtual void Clear() override;

    virtual bool AddCollisionObject(const std::string &name,
                                    std::shared_ptr<Geometry> shape,
                                    const SE3d &shape_pose,
                                    bool enabled = true) override;

    virtual bool AddCollisionObject(const Link &link,
                                    bool enabled = true) override;

    virtual bool AddCollisionObject(std::shared_ptr<Multibody const> mul,
                                    bool enabled = true,
                                    bool add_into_env = true) override;

    void AddCollisionObject(
        const std::shared_ptr<BulletCollisionObjectWrapper> &cow);

    virtual bool RemoveCollisionObject(const std::string &name) override;

    virtual bool RemoveCollisionObject(const Link &link) override;

    virtual bool RemoveCollisionObject(std::shared_ptr<Multibody const> mul,
                                       bool remove_from_env = true) override;

    virtual bool HasCollisionObject(const std::string &name) const override;

    virtual bool HasCollisionObject(const Link &link) const override
    {
        return HasCollisionObject(link.GetUniqueName());
    }

    virtual bool EnableCollisionObject(const std::string &name) override;

    virtual bool DisableCollisionObject(const std::string &name) override;

    virtual void SetCollisionObjectsTransform(const std::string &name,
                                              const SE3d &pose) override;

    virtual bool CheckCollision(const CollisionMatrix &collision_matrix,
                                CollisionReport &report) const override;

    virtual bool CheckCollision(const Link &link1, const Link &link2,
                                CollisionReport &report) const override;
    virtual bool
    ContinuousCheckCollision(const Link *link1, const SE3d &pose1_beg,
                             const SE3d &pose1_end, const Link *link2,
                             const SE3d &pose2_beg, const SE3d &pose2_end,
                             CollisionReport &report) const override;

    /**
     * @brief Get the total time of checking collision
     *
     * @return double  check time (unit: second)
     */
    double GetCheckTime() const;

    /**
     * @brief Get the time for synchronizing state of multibodies into FCL
     * objects
     *
     * @return double synchronization time (unit: second)
     */
    double GetSyncTime() const;

    /**
     * @brief To update the observer's state when publishers publish its new
     * subject
     * @see abstract base class ObserverBase
     *
     */
    virtual void UpdateState(const OSDatagram &os_datagram) override;

protected:
    virtual bool
    _ContinuousCheckCollision(std::shared_ptr<Manipulator> manipulator,
                              const JointVector &start_state,
                              const JointVector &end_state,
                              const CollisionMatrix &collision_matrix,
                              CollisionReport &report) const override;

private:
    void _AllocCollisionBroadPhase() const;
    std::shared_ptr<BulletDiscreteBVHManager> m_manager;
    mutable TimeDuration m_check_time; ///< time for checking collison
    mutable TimeDuration m_sync_time; ///< time for synchronizing objects state
    std::list<std::shared_ptr<Multibody>>
        m_publishers; ///< publisher list from which the checker subscribes the
                      ///< subject (i.e., the base transformation of multibody)
    std::shared_ptr<Environment>
        m_env; ///< the environment that holds all collision objects
};
/// @}

} // namespace RVS